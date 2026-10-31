"""Orbbec Dabai DC1 colour + depth preview (Tk window).

Colour comes through the standard UVC camera (OpenCV index 1); depth comes through Orbbec's OpenNI2
SDK (needs Orbbec SensorDriver installed + drivers/orbbec/openni_sdk extracted).

    .venv\\Scripts\\python.exe tools\\rgbd_preview.py [colour_index]
Move the mouse over the depth image to read the distance. S = save snapshot, ESC = quit.
"""

import sys
import time
import tkinter as tk
from pathlib import Path

import cv2
import numpy as np
from openni import openni2
from PIL import Image, ImageTk

ROOT = Path(__file__).resolve().parent.parent
OPENNI_LIB = (ROOT / "drivers/orbbec/openni_sdk/win/OpenNI_2.3.0.86_202210111950_4c8f5aa4_beta6_windows"
              / "Win64-Release/sdk/libs")
COLOR_INDEX = int(sys.argv[1]) if len(sys.argv) > 1 else 1
OUT = ROOT / "captures"
MAX_MM = 3000  # colour scale range for the depth view


def open_depth():
    openni2.initialize(str(OPENNI_LIB))
    dev = openni2.Device.open_any()
    ds = dev.create_depth_stream()
    for m in dev.get_sensor_info(openni2.SENSOR_DEPTH).videoModes:
        if (m.resolutionX, m.resolutionY, m.fps) == (640, 480, 30) and "1_MM" in str(m.pixelFormat):
            ds.set_video_mode(m)
            break
    ds.start()
    return dev, ds


def main():
    dev, ds = open_depth()
    cap = cv2.VideoCapture(COLOR_INDEX, cv2.CAP_DSHOW)

    root = tk.Tk()
    root.title("Dabai DC1  colour | depth   (S = snapshot, ESC = quit)")
    row = tk.Frame(root)
    row.pack()
    color_lbl, depth_lbl = tk.Label(row), tk.Label(row)
    color_lbl.grid(row=0, column=0)
    depth_lbl.grid(row=0, column=1)
    info = tk.Label(root, font=("Consolas", 11), anchor="w")
    info.pack(fill="x")
    st = {"depth": None, "color": None, "mouse": None, "t": time.perf_counter(), "fps": 0.0, "msg": ""}

    def on_motion(e):
        st["mouse"] = (e.x, e.y)

    def snapshot(_=None):
        if st["depth"] is None:
            return
        OUT.mkdir(exist_ok=True)
        stamp = time.strftime("%Y%m%d_%H%M%S")
        np.save(OUT / f"depth_{stamp}.npy", st["depth"])
        if st["color"] is not None:
            cv2.imwrite(str(OUT / f"color_{stamp}.jpg"), st["color"])
        st["msg"] = f"saved color_{stamp}.jpg + depth_{stamp}.npy"

    def tick():
        f = ds.read_frame()
        d = np.frombuffer(f.get_buffer_as_uint16(), dtype=np.uint16).reshape(f.height, f.width).copy()
        st["depth"] = d
        vis = cv2.applyColorMap((255 - np.clip(d, 0, MAX_MM) * 255 // MAX_MM).astype(np.uint8), cv2.COLORMAP_JET)
        vis[d == 0] = 0  # no depth -> black
        dimg = ImageTk.PhotoImage(Image.fromarray(cv2.cvtColor(vis, cv2.COLOR_BGR2RGB)))
        depth_lbl.config(image=dimg)
        depth_lbl.image = dimg

        ok, c = cap.read()
        if ok:
            st["color"] = c
            c = cv2.resize(c, (d.shape[1], d.shape[0]))
            cimg = ImageTk.PhotoImage(Image.fromarray(cv2.cvtColor(c, cv2.COLOR_BGR2RGB)))
            color_lbl.config(image=cimg)
            color_lbl.image = cimg

        now = time.perf_counter()
        st["fps"] = 0.9 * st["fps"] + 0.1 / max(now - st["t"], 1e-3)
        st["t"] = now
        text = f"depth {d.shape[1]}x{d.shape[0]}  {st['fps']:4.1f} fps  valid {100 * np.count_nonzero(d) / d.size:3.0f}%"
        if st["mouse"]:
            x, y = st["mouse"]
            if 0 <= x < d.shape[1] and 0 <= y < d.shape[0]:
                text += f"   distance at ({x},{y}) = {d[y, x]} mm"
        if st["msg"]:
            text += "   " + st["msg"]
        info.config(text=text)
        root.after(1, tick)

    depth_lbl.bind("<Motion>", on_motion)
    root.bind("<Escape>", lambda e: root.destroy())
    root.bind("<KeyPress-s>", snapshot)
    tick()
    try:
        root.mainloop()
    finally:
        ds.stop()
        cap.release()
        openni2.unload()


if __name__ == "__main__":
    main()
