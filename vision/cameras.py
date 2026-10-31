"""Camera readers: RealSense D415 (fixed, colour + aligned depth) and UVC cameras via OpenCV.

UVC index 1 = Orbbec Dabai DC1 colour (mounted on the PiPER gripper); index 0 = laptop camera.
"""

import threading
import time

import cv2
import numpy as np


class RealSenseCam:
    """D415 colour 640x480@30 + depth aligned to colour. read() -> (bgr, depth_mm) or (None, None)."""

    def __init__(self, width: int = 640, height: int = 480, fps: int = 30):
        self.width, self.height, self.fps = width, height, fps
        self.pipeline = None
        self.align = None
        self._intr = None

    def start(self) -> "RealSenseCam":
        import pyrealsense2 as rs

        if len(rs.context().query_devices()) == 0:
            raise RuntimeError("No RealSense camera found. Plug the D415 into a USB 3 port (blue / SS) "
                               "with a USB 3 data cable.")
        self.pipeline = rs.pipeline()
        cfg = rs.config()
        cfg.enable_stream(rs.stream.color, self.width, self.height, rs.format.bgr8, self.fps)
        cfg.enable_stream(rs.stream.depth, self.width, self.height, rs.format.z16, self.fps)
        profile = self.pipeline.start(cfg)
        self.align = rs.align(rs.stream.color)
        # z16 units are depth_scale metres (1 mm on the D4xx by default); convert to mm in read().
        self.depth_scale = profile.get_device().first_depth_sensor().get_depth_scale()
        self._intr = profile.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()
        return self

    @property
    def intrinsics(self):
        """pyrealsense2.intrinsics of the colour stream (depth is aligned to it)."""
        return self._intr

    def read(self, timeout_ms: int = 1000):
        try:
            frames = self.pipeline.wait_for_frames(timeout_ms)
        except RuntimeError:
            return None, None
        frames = self.align.process(frames)
        c, d = frames.get_color_frame(), frames.get_depth_frame()
        if not c or not d:
            return None, None
        depth = np.asanyarray(d.get_data())
        if abs(self.depth_scale - 0.001) > 1e-6:
            depth = (depth.astype(np.float32) * (self.depth_scale * 1000.0)).astype(np.uint16)
        return np.asanyarray(c.get_data()).copy(), depth.copy()

    def stop(self) -> None:
        if self.pipeline is not None:
            self.pipeline.stop()
            self.pipeline = None


class UvcCam:
    """OpenCV (DirectShow) camera read in a background thread; latest() returns the newest frame."""

    def __init__(self, index: int = 1, width: int | None = None, height: int | None = None):
        self.index = index
        self.cap = cv2.VideoCapture(index, cv2.CAP_DSHOW)
        if not self.cap.isOpened():
            raise RuntimeError(f"UVC camera index {index} could not be opened")
        if width and height:
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        self._lock = threading.Lock()
        self._frame = None
        self._stamp = 0.0
        self._stop = threading.Event()
        self._th = threading.Thread(target=self._run, daemon=True)

    def start(self) -> "UvcCam":
        self._th.start()
        return self

    def _run(self):
        while not self._stop.is_set():
            ok, frame = self.cap.read()
            if ok:
                with self._lock:
                    self._frame, self._stamp = frame, time.perf_counter()
            else:
                time.sleep(0.01)

    def latest(self):
        with self._lock:
            return None if self._frame is None else self._frame.copy()

    def age(self) -> float:
        with self._lock:
            return time.perf_counter() - self._stamp if self._stamp else float("inf")

    def stop(self) -> None:
        self._stop.set()
        self._th.join(timeout=1.0)
        self.cap.release()
