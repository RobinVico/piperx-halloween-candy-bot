"""SO-101 leader -> PiPER X joint mapping and safety filter.

Shared by the simulation (sim/teleop_sim.py) and, later, the real-arm driver, so whatever is
validated in sim is exactly what runs on hardware.

Conventions:
  * SO-101 joints come from lerobot in degrees (gripper in 0..100 %).
  * PiPER X targets are degrees for J1..J6 and millimetres of total opening for the gripper.
  * PiPER X zero pose = arm folded flat (its rest pose). The SO-101 "rest" readings below are the
    leader's folded pose, so folding the leader folds the PiPER X.
"""

import json
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np

MAPPING_FILE = Path(__file__).resolve().parent / "mapping_config.json"

SO_JOINTS = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper"]

# PiPER X limits from the official URDF (agx_arm_urdf/piper_x), degrees.
PIPER_X_LIMITS_DEG = np.array([
    [-150.0, 150.0],  # J1 base yaw
    [0.0, 180.0],     # J2 shoulder
    [-170.0, 0.0],    # J3 elbow
    [-89.0, 89.0],    # J4 wrist pitch
    [-89.0, 89.0],    # J5 wrist yaw (SO-101 has no equivalent)
    [-170.0, 170.0],  # J6 wrist roll
])
GRIPPER_MAX_MM = 70.0
# Hard stops the folded rest pose rests on (gripper pointing up, J4 at its -90 deg stop).
REST_STOP_DEG = {"J2_min": -3.0, "J3_max": 3.0, "J4_min": -90.0}


@dataclass
class MappingConfig:
    # Which SO-101 joint drives each PiPER joint (None = held at `fixed_deg`).
    # PiPER X wrist differs from PiPER: J4 = pitch (axis parallel to J2/J3), J5 = yaw, J6 = roll.
    # SO-101 has no wrist yaw, so J5 is held fixed.
    source: tuple = ("shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", None, "wrist_roll")
    sign: np.ndarray = field(default_factory=lambda: np.array([-1.0, 1.0, 1.0, 1.0, 1.0, 1.0]))
    # SO-101 reading that corresponds to PiPER 0 deg (leader's folded rest pose for J2/J3).
    so_zero: np.ndarray = field(default_factory=lambda: np.array([0.0, -92.6, 90.1, 4.2, 0.0, 84.4]))
    scale: np.ndarray = field(default_factory=lambda: np.ones(6))
    fixed_deg: np.ndarray = field(default_factory=lambda: np.zeros(6))
    # Extra margin kept away from the hard limits, degrees.
    limit_margin_deg: float = 5.0
    # SO-101 gripper % -> PiPER opening; leader reads ~0-15 closed, ~100 fully open.
    gripper_pct_closed: float = 10.0
    gripper_pct_open: float = 100.0


def map_leader(so_deg: dict, cfg: MappingConfig) -> tuple[np.ndarray, float]:
    """Raw mapping: SO-101 reading -> (PiPER J1..J6 deg, gripper opening mm). No limits applied."""
    q = cfg.fixed_deg.copy()
    for i, name in enumerate(cfg.source):
        if name is not None:
            q[i] = cfg.sign[i] * (so_deg[name] - cfg.so_zero[i]) * cfg.scale[i]
    pct = (so_deg["gripper"] - cfg.gripper_pct_closed) / (cfg.gripper_pct_open - cfg.gripper_pct_closed)
    grip_mm = float(np.clip(pct, 0.0, 1.0) * GRIPPER_MAX_MM)
    return q, grip_mm


def capture_rest(so_deg: dict, cfg: MappingConfig) -> None:
    """Set `so_zero` from the current leader pose, which must match the PiPER zero (folded) pose."""
    for i, name in enumerate(cfg.source):
        if name is not None:
            cfg.so_zero[i] = so_deg[name]


def load_config(path: Path = MAPPING_FILE) -> MappingConfig:
    cfg = MappingConfig()
    if path.exists():
        d = json.loads(path.read_text())
        cfg.sign = np.array(d.get("sign", cfg.sign), dtype=float)
        cfg.so_zero = np.array(d.get("so_zero", cfg.so_zero), dtype=float)
        cfg.scale = np.array(d.get("scale", cfg.scale), dtype=float)
        cfg.gripper_pct_closed = d.get("gripper_pct_closed", cfg.gripper_pct_closed)
        cfg.gripper_pct_open = d.get("gripper_pct_open", cfg.gripper_pct_open)
    return cfg


def save_config(cfg: MappingConfig, path: Path = MAPPING_FILE) -> None:
    path.write_text(json.dumps({
        "sign": cfg.sign.tolist(),
        "so_zero": [round(v, 2) for v in cfg.so_zero.tolist()],
        "scale": cfg.scale.tolist(),
        "gripper_pct_closed": cfg.gripper_pct_closed,
        "gripper_pct_open": cfg.gripper_pct_open,
    }, indent=2))


class SafetyFilter:
    """Clamp to limits, cap joint speed, reject single-frame jumps, and ramp in at start-up."""

    """Clamp to limits, reject glitches, low-pass the target, then track it with capped speed AND
    acceleration so motion starts and stops smoothly (no "stutter"). Uses the measured dt."""

    def __init__(self, cfg: MappingConfig, start_deg: np.ndarray, start_grip_mm: float,
                 max_speed_deg_s: float = 60.0, max_acc_deg_s2: float = 300.0,
                 smooth_tau_s: float = 0.06, track_time_s: float = 0.12,
                 max_grip_mm_s: float = 80.0, jump_reject_deg: float = 45.0, ramp_s: float = 2.0):
        lim = PIPER_X_LIMITS_DEG.copy()
        lim[:, 0] += cfg.limit_margin_deg
        lim[:, 1] -= cfg.limit_margin_deg
        # The real arm's rest pose sits on hard stops slightly outside the URDF limits
        # (measured 2026-10-31: J2=-2.7, J3=+2.4, J4=-89.7), so the rest pose must stay reachable.
        lim[1, 0] = REST_STOP_DEG["J2_min"]
        lim[2, 1] = REST_STOP_DEG["J3_max"]
        lim[3, 0] = REST_STOP_DEG["J4_min"]
        self.lim = lim
        self.q = np.clip(np.asarray(start_deg, dtype=float), lim[:, 0], lim[:, 1])
        self.v = np.zeros(6)
        self.target_f = self.q.copy()
        self.grip = float(start_grip_mm)
        self.grip_f = self.grip
        self.max_speed = max_speed_deg_s
        self.max_acc = max_acc_deg_s2
        self.tau = smooth_tau_s
        self.track_time = track_time_s
        self.max_grip_speed = max_grip_mm_s
        self.jump_reject = jump_reject_deg
        self.ramp_s = ramp_s
        self.t = 0.0
        self.prev_target = None
        self.rejected = 0
        self._consecutive_rejects = 0

    def step(self, target_deg: np.ndarray, target_grip_mm: float, dt: float) -> tuple[np.ndarray, float]:
        dt = float(np.clip(dt, 1e-4, 0.05))  # guard against stalls producing a huge step
        target = np.clip(target_deg, self.lim[:, 0], self.lim[:, 1])
        if (self.prev_target is not None and self._consecutive_rejects < 5
                and np.max(np.abs(target - self.prev_target)) > self.jump_reject):
            # A leader reading jumped implausibly far in one frame: ignore it (likely a bad packet).
            # After 5 frames in a row it is real (e.g. leader moved while paused), so accept it.
            self.rejected += 1
            self._consecutive_rejects += 1
            target = self.prev_target
        else:
            self._consecutive_rejects = 0
        self.prev_target = target

        self.t += dt
        # During the ramp the speed cap grows from 10% to 100%, so start-up is gentle.
        ramp = min(1.0, 0.1 + 0.9 * self.t / self.ramp_s) if self.ramp_s > 0 else 1.0

        # 1) Low-pass the target to remove leader quantisation/hand tremor.
        alpha = 1.0 - np.exp(-dt / self.tau) if self.tau > 0 else 1.0
        self.target_f += alpha * (target - self.target_f)

        # 2) Smooth tracker: desired speed is linear near the target (no chatter), sqrt-shaped
        #    further away (can always brake within max_acc), and capped at max_speed.
        err = self.target_f - self.q
        vmax = self.max_speed * ramp
        a = self.max_acc * 0.8
        v_des = np.sign(err) * np.minimum.reduce([
            np.full(6, vmax), np.sqrt(2.0 * a * np.abs(err)), np.abs(err) / self.track_time])
        self.v += np.clip(v_des - self.v, -self.max_acc * dt, self.max_acc * dt)
        self.q += self.v * dt

        # Never leave the limits; kill velocity into a limit.
        lo, hi = self.lim[:, 0], self.lim[:, 1]
        hit = (self.q < lo) | (self.q > hi)
        self.q = np.clip(self.q, lo, hi)
        self.v[hit] = 0.0

        # Gripper: low-pass + speed cap.
        self.grip_f += alpha * (target_grip_mm - self.grip_f)
        gstep = self.max_grip_speed * ramp * dt
        self.grip += float(np.clip(self.grip_f - self.grip, -gstep, gstep))
        return self.q.copy(), self.grip

    def hold(self) -> None:
        """Freeze the target at the current position (used when the leader stream is lost)."""
        self.target_f = self.q.copy()
        self.prev_target = self.q.copy()
