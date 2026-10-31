"""Offscreen render of the PiPER X model at a few test poses (for checking joint directions)."""

import sys
from pathlib import Path

import mujoco
import numpy as np
from PIL import Image

HERE = Path(__file__).resolve().parent
model = mujoco.MjModel.from_xml_path(str(HERE / "piper_x_scene.xml"))
data = mujoco.MjData(model)
renderer = mujoco.Renderer(model, 360, 480)

cam = mujoco.MjvCamera()
cam.lookat[:] = [0.15, 0, 0.25]
cam.distance = 1.1
cam.azimuth = 135
cam.elevation = -20

POSES = {  # degrees: j1..j6, gripper opening (m)
    "zero": ([0, 0, 0, 0, 0, 0], 0.0),
    "j2_45": ([0, 45, 0, 0, 0, 0], 0.0),
    "j2_45_j3_-60": ([0, 45, -60, 0, 0, 0], 0.0),
    "j1_45_bent": ([45, 60, -90, 0, 30, 0], 0.07),
    "j4_60": ([0, 60, -90, 60, 0, 0], 0.03),
}
tiles = []
for name, (deg, grip) in POSES.items():
    data.qpos[:6] = np.radians(deg)
    data.qpos[6], data.qpos[7] = grip / 2, -grip / 2
    mujoco.mj_forward(model, data)
    renderer.update_scene(data, cam)
    img = renderer.render()
    tiles.append(img)
    tcp = data.body("gripper_base").xpos
    print(f"{name:14s} gripper_base xyz = {np.round(tcp, 3)}")

out = HERE / "render_check.png"
Image.fromarray(np.concatenate(tiles, axis=1)).save(out)
print("saved", out)
