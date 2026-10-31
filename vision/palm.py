"""Open-palm detection with MediaPipe HandLandmarker (Tasks API; mediapipe 1.x has no mp.solutions)."""

import math
from dataclasses import dataclass
from pathlib import Path

import cv2
import numpy as np

MODEL = Path(__file__).resolve().parent.parent / "models" / "hand_landmarker.task"

WRIST = 0
THUMB_IP, THUMB_TIP = 3, 4
FINGERS = {  # name: (mcp, pip, tip)
    "index": (5, 6, 8), "middle": (9, 10, 12), "ring": (13, 14, 16), "pinky": (17, 18, 20),
}
PALM_POINTS = [0, 5, 9, 13, 17]
CONNECTIONS = [(0, 1), (1, 2), (2, 3), (3, 4), (0, 5), (5, 6), (6, 7), (7, 8), (5, 9), (9, 10), (10, 11),
               (11, 12), (9, 13), (13, 14), (14, 15), (15, 16), (13, 17), (17, 18), (18, 19), (19, 20), (0, 17)]


@dataclass
class Hand:
    landmarks_px: np.ndarray  # (21, 2) int pixel coords
    center_px: tuple[int, int]  # palm centre: mean of wrist + 4 MCPs
    fingers_extended: int  # 0..5
    spread_ok: bool
    is_open_palm: bool
    palm_facing_camera: bool | None  # informational only
    score: float
    handedness: str


def _angle_deg(a: np.ndarray, b: np.ndarray) -> float:
    c = float(np.dot(a, b) / (np.linalg.norm(a) * np.linalg.norm(b) + 1e-9))
    return math.degrees(math.acos(max(-1.0, min(1.0, c))))


def classify(lm: np.ndarray, handedness: str) -> tuple[int, bool, bool, bool | None, list[bool]]:
    """lm: (21, 3) normalised landmarks. Returns (n_extended, spread_ok, is_open, facing, per-finger)."""
    d = lambda i, j: float(np.linalg.norm(lm[i] - lm[j]))  # noqa: E731
    ext = [d(WRIST, tip) > 1.15 * d(WRIST, pip) for (_, pip, tip) in FINGERS.values()]
    thumb = d(THUMB_TIP, 17) > 1.05 * d(THUMB_IP, 17)
    dirs = [lm[tip, :2] - lm[mcp, :2] for (mcp, _, tip) in FINGERS.values()]
    spread = float(np.mean([_angle_deg(dirs[k], dirs[k + 1]) for k in range(3)]))
    spread_ok = spread > 8.0
    is_open = all(ext) and spread_ok
    # Palm normal sign; MediaPipe assumes a mirrored (selfie) image, so for an unmirrored camera the
    # handedness label is swapped. Only a hint — not used for the open-palm decision.
    z = float(np.cross(lm[5, :2] - lm[0, :2], lm[17, :2] - lm[0, :2]))
    facing = None if abs(z) < 1e-4 else ((z > 0) == (handedness == "Left"))
    return int(sum(ext) + thumb), spread_ok, is_open, facing, ext + [thumb]


class PalmDetector:
    def __init__(self, model_path: Path = MODEL, num_hands: int = 2, min_conf: float = 0.5):
        from mediapipe.tasks.python import BaseOptions, vision

        opts = vision.HandLandmarkerOptions(
            base_options=BaseOptions(model_asset_path=str(model_path)),
            running_mode=vision.RunningMode.VIDEO, num_hands=num_hands,
            min_hand_detection_confidence=min_conf, min_hand_presence_confidence=min_conf,
            min_tracking_confidence=min_conf)
        self._lm = vision.HandLandmarker.create_from_options(opts)
        self._last_t = -1

    def detect(self, bgr: np.ndarray, t_ms: int) -> list[Hand]:
        import mediapipe as mp

        t_ms = max(int(t_ms), self._last_t + 1)  # VIDEO mode requires strictly increasing timestamps
        self._last_t = t_ms
        img = mp.Image(image_format=mp.ImageFormat.SRGB, data=cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB))
        res = self._lm.detect_for_video(img, t_ms)
        h, w = bgr.shape[:2]
        hands = []
        for i, lms in enumerate(res.hand_landmarks):
            lm = np.array([[p.x, p.y, p.z] for p in lms], dtype=np.float32)
            cat = res.handedness[i][0] if res.handedness and res.handedness[i] else None
            label, score = (cat.category_name, float(cat.score)) if cat else ("?", 0.0)
            n, spread_ok, is_open, facing, _ = classify(lm, label)
            px = np.column_stack([np.clip(lm[:, 0] * w, 0, w - 1), np.clip(lm[:, 1] * h, 0, h - 1)]).astype(int)
            c = px[PALM_POINTS].mean(axis=0).astype(int)
            hands.append(Hand(px, (int(c[0]), int(c[1])), n, spread_ok, is_open, facing, score, label))
        return hands

    def close(self) -> None:
        self._lm.close()


def palm_xyz(hand: Hand, depth_mm: np.ndarray, intrinsics, win: int = 7):
    """3D palm centre in metres in the colour-camera frame (depth aligned to colour), or None."""
    import pyrealsense2 as rs

    if depth_mm is None or intrinsics is None:
        return None
    x, y = hand.center_px
    h, w = depth_mm.shape
    patch = depth_mm[max(0, y - win):min(h, y + win + 1), max(0, x - win):min(w, x + win + 1)]
    valid = patch[patch > 0]
    if valid.size < 5:
        return None
    z_m = float(np.median(valid)) / 1000.0
    return tuple(rs.rs2_deproject_pixel_to_point(intrinsics, [float(x), float(y)], z_m))


def in_roi(center_px, shape, roi) -> bool:
    """roi = (x0, y0, x1, y1) normalised to 0..1, or None for the whole image."""
    if roi is None:
        return True
    h, w = shape[:2]
    x, y = center_px[0] / w, center_px[1] / h
    return roi[0] <= x <= roi[2] and roi[1] <= y <= roi[3]


class PalmTrigger:
    """Fires once an open palm has stayed (roughly still) inside the ROI for hold_s seconds."""

    def __init__(self, roi=None, hold_s: float = 0.8, max_motion_px: float = 40.0):
        self.roi = roi
        self.hold_s = hold_s
        self.max_motion_px = max_motion_px
        self.reset()

    def reset(self) -> None:
        self._since = None
        self._anchor = None
        self._last_seen = None
        self.palm: Hand | None = None

    def _pick(self, hands: list[Hand], shape) -> Hand | None:
        cands = [h for h in hands if h.is_open_palm and in_roi(h.center_px, shape, self.roi)]
        return max(cands, key=lambda h: h.score) if cands else None

    def update(self, hands: list[Hand], shape, now: float) -> bool:
        """Feed one frame's detections; returns True when the trigger condition is met."""
        if any(in_roi(h.center_px, shape, self.roi) for h in hands):
            self._last_seen = now
        palm = self._pick(hands, shape)
        self.palm = palm
        if palm is None:
            self._since = self._anchor = None
            return False
        c = np.array(palm.center_px, dtype=float)
        if self._anchor is None or np.linalg.norm(c - self._anchor) > self.max_motion_px:
            self._since, self._anchor = now, c  # (re)start the hold timer when the palm moves a lot
        return now - self._since >= self.hold_s

    def progress(self, now: float) -> float:
        return 0.0 if self._since is None else min(1.0, (now - self._since) / self.hold_s)

    def present(self, now: float, within_s: float = 0.3) -> bool:
        """An open palm was seen in the ROI in the last update (and the hold timer is running)."""
        return self.palm is not None and self._since is not None and now - (self._last_seen or 0) <= within_s

    def hand_gone(self, now: float, clear_s: float = 1.5) -> bool:
        """No hand of any kind inside the ROI for clear_s seconds."""
        return self._last_seen is None or now - self._last_seen >= clear_s


def draw_overlay(bgr: np.ndarray, hands: list[Hand], roi=None) -> np.ndarray:
    out = bgr.copy()
    h, w = out.shape[:2]
    if roi is not None:
        cv2.rectangle(out, (int(roi[0] * w), int(roi[1] * h)), (int(roi[2] * w), int(roi[3] * h)), (255, 200, 0), 2)
    for hand in hands:
        col = (0, 220, 0) if hand.is_open_palm else (0, 150, 255)
        for a, b in CONNECTIONS:
            cv2.line(out, tuple(hand.landmarks_px[a]), tuple(hand.landmarks_px[b]), col, 2)
        for p in hand.landmarks_px:
            cv2.circle(out, tuple(p), 3, col, -1)
        cv2.circle(out, hand.center_px, 8, (255, 255, 255), 2)
        txt = f"{'OPEN PALM' if hand.is_open_palm else 'hand'} f={hand.fingers_extended}"
        cv2.putText(out, txt, (hand.center_px[0] + 10, hand.center_px[1] - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, col, 2)
    return out
