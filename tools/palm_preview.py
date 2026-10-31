"""Open-palm detection preview.

    .venv\\Scripts\\python.exe tools\\palm_preview.py              # RealSense D415 colour + depth
    .venv\\Scripts\\python.exe tools\\palm_preview.py --uvc 1      # any UVC camera (1 = Dabai, 0 = laptop)
ESC quits.
"""

import argparse
import sys
import time
import tkinter as tk
from pathlib import Path

import cv2
import numpy as np
from PIL import Image, ImageTk

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from vision.cameras import RealSenseCam, UvcCam  # noqa: E402
from vision.palm import PalmDetector, PalmTrigger, draw_overlay, palm_xyz  # noqa: E402


def depth_vis(d: np.ndarray, max_mm: int = 2000) -> np.ndarray:
    vis = cv2.applyColorMap((255 - np.clip(d, 0, max_mm) * 255 // max_mm).astype(np.uint8), cv2.COLORMAP_JET)
    vis[d == 0] = 0
    return vis


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--uvc", type=int, default=None, help="use this UVC camera index instead of the D415")
    ap.add_argument("--seconds", type=float, default=0.0, help="auto-quit after N seconds (testing)")
    ap.add_argument("--roi", type=float, nargs=4, default=None, metavar=("X0", "Y0", "X1", "Y1"),
                    help="trigger region, normalised 0..1")
    args = ap.parse_args()

    if args.uvc is None:
        cam = RealSenseCam().start()
        read = cam.read
    else:
        cam = UvcCam(args.uvc).start()
        read = lambda: (cam.latest(), None)  # noqa: E731

    det = PalmDetector()
    trig = PalmTrigger(roi=args.roi)
    t0 = time.perf_counter()

    root = tk.Tk()
    root.title("Palm detector  (" + ("D415" if args.uvc is None else f"UVC {args.uvc}") + ")  ESC = quit")
    row = tk.Frame(root)
    row.pack()
    color_lbl, depth_lbl = tk.Label(row), tk.Label(row)
    color_lbl.grid(row=0, column=0)
    depth_lbl.grid(row=0, column=1)
    info = tk.Label(root, font=("Consolas", 12), anchor="w")
    info.pack(fill="x")
    st = {"fired": 0, "fps": 0.0, "t": time.perf_counter()}

    def show(lbl, bgr):
        img = ImageTk.PhotoImage(Image.fromarray(cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)))
        lbl.config(image=img)
        lbl.image = img

    def tick():
        if args.seconds and time.perf_counter() - t0 > args.seconds:
            root.destroy()
            return
        bgr, depth = read()
        if bgr is None:
            root.after(10, tick)
            return
        now = time.perf_counter()
        hands = det.detect(bgr, int((now - t0) * 1000))
        fired = trig.update(hands, bgr.shape, now)
        if fired:
            st["fired"] += 1
            trig.reset()
        show(color_lbl, draw_overlay(bgr, hands, args.roi))
        if depth is not None:
            show(depth_lbl, depth_vis(depth))

        st["fps"] = 0.9 * st["fps"] + 0.1 / max(now - st["t"], 1e-3)
        st["t"] = now
        palm = trig.palm
        txt = f"{st['fps']:4.1f} fps  hands={len(hands)}  "
        if palm is not None:
            xyz = palm_xyz(palm, depth, cam.intrinsics) if depth is not None else None
            txt += "OPEN PALM  hold {:3.0f}%".format(100 * trig.progress(now))
            if xyz is not None:
                txt += "  xyz=({:+.2f}, {:+.2f}, {:.2f}) m".format(*xyz)
        else:
            txt += "no open palm"
        txt += f"   triggers={st['fired']}"
        info.config(text=txt)
        root.after(1, tick)

    root.bind("<Escape>", lambda e: root.destroy())
    tick()
    try:
        root.mainloop()
    finally:
        det.close()
        cam.stop()
    print(f"palm_preview exited cleanly, triggers={st['fired']}")


if __name__ == "__main__":
    main()
