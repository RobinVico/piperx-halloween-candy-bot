"""Small wrapper around the MuJoCo passive viewer showing the PiPER X model at a given pose."""

from pathlib import Path

import mujoco
import mujoco.viewer
import numpy as np

SCENE = Path(__file__).resolve().parent / "piper_x_scene.xml"


class PiperView:
    def __init__(self):
        self.model = mujoco.MjModel.from_xml_path(str(SCENE))
        self.data = mujoco.MjData(self.model)
        self.viewer = None

    def __enter__(self):
        self.viewer = mujoco.viewer.launch_passive(self.model, self.data, show_left_ui=False, show_right_ui=False)
        self.viewer.cam.lookat[:] = [0.15, 0, 0.25]
        self.viewer.cam.distance, self.viewer.cam.azimuth, self.viewer.cam.elevation = 1.2, 135, -20
        return self

    def __exit__(self, *exc):
        self.viewer.close()

    def is_running(self) -> bool:
        return self.viewer.is_running()

    def show(self, q_deg: np.ndarray, grip_mm: float) -> None:
        self.data.qpos[:6] = np.radians(q_deg)
        self.data.qpos[6], self.data.qpos[7] = grip_mm / 2000.0, -grip_mm / 2000.0  # total mm -> m per finger
        mujoco.mj_forward(self.model, self.data)
        self.viewer.sync()
