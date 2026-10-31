"""Live camera preview (Tk window; the venv has opencv-python-headless, so no cv2.imshow).

    .venv\\Scripts\\python.exe tools\\camera_preview.py        # index 1 = Orbbec Dabai DC1 colour
    .venv\\Scripts\\python.exe tools\\camera_preview.py 0      # index 0 = laptop camera
Press S in the window to save a snapshot to captures\\, ESC/close to quit.
"""

import sys
import time
import tkinter as tk
from pathlib import Path

import cv2
from PIL import Image, ImageTk

INDEX = int(sys.argv[1]) if len(sys.argv) > 1 else 1
OUT = Path(__file__).resolve().parent.parent / "captures"


def main():
    cap = cv2.VideoCapture(INDEX, cv2.CAP_DSHOW)
    if not cap.isOpened():
        raise SystemExit(f"Camera index {INDEX} could not be opened")
    root = tk.Tk()
    root.title(f"Camera {INDEX} preview  (S = snapshot, ESC = quit)")
    label = tk.Label(root)
    label.pack()
    info = tk.Label(root, font=("Consolas", 10))
    info.pack(fill="x")
    state = {"frame": None, "t": time.perf_counter(), "fps": 0.0}

    def snapshot(_=None):
        if state["frame"] is not None:
            OUT.mkdir(exist_ok=True)
            fn = OUT / time.strftime(f"cam{INDEX}_%Y%m%d_%H%M%S.jpg")
            cv2.imwrite(str(fn), state["frame"])
            info.config(text=f"saved {fn.name}")

    def tick():
        ok, frame = cap.read()
        if ok:
            state["frame"] = frame
            now = time.perf_counter()
            state["fps"] = 0.9 * state["fps"] + 0.1 / max(now - state["t"], 1e-3)
            state["t"] = now
            img = ImageTk.PhotoImage(Image.fromarray(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)))
            label.config(image=img)
            label.image = img
            if not info.cget("text").startswith("saved"):
                info.config(text=f"{frame.shape[1]}x{frame.shape[0]}  {state['fps']:4.1f} fps")
        root.after(10, tick)

    root.bind("<Escape>", lambda e: root.destroy())
    root.bind("<KeyPress-s>", snapshot)
    tick()
    root.mainloop()
    cap.release()


if __name__ == "__main__":
    main()
