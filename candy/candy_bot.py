"""Halloween candy bot: when an open palm is held out in the hand-off zone, the PiPER X picks a candy
from the box and drops it into the hand.

Cameras: RealSense D415 (fixed, watches the hand-off zone) triggers; Dabai DC1 colour (on the gripper)
is shown in the panel. Motion = taught waypoints (candy/waypoints.json, recorded in the teleop panel),
joined by the same smooth, speed/acceleration-limited filter used for teleop.

    .venv\\Scripts\\python.exe -m candy.candy_bot                    # dry run: simulated arm only
    .venv\\Scripts\\python.exe -m candy.candy_bot --live             # real arm (enable after confirm)
    .venv\\Scripts\\python.exe -m candy.candy_bot --trigger-uvc 0    # no D415 yet: use a UVC camera to trigger
"""

import argparse
import sys
import threading
import time
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from candy import waypoints as wp  # noqa: E402
from teleop.mapping import SafetyFilter, load_config  # noqa: E402
from teleop.timing import HighResTimer, Rate  # noqa: E402

GRIP_OPEN_MM = 60.0
EMPTY_GRIP_MM = 3.0  # closed narrower than this after grasping -> nothing in the gripper


class Aborted(Exception):
    pass


# ---------------------------------------------------------------------- arm executor
class ArmExec:
    """100 Hz loop: streams the filtered setpoint to the arm (live) or just simulates it (dry run)."""

    def __init__(self, live: bool, start_q, start_grip, max_acc: float, speed_pct: int, track_err_stop: float):
        self.live = live
        self.arm = None
        if live:
            from teleop.piper_driver import PiperX
            self.arm = PiperX(allow_motion=True)
            self.arm.connect()
            time.sleep(0.5)
            start_q, start_grip = self.arm.joints_deg(), self.arm.gripper_mm()
        self.flt = SafetyFilter(load_config(), start_deg=start_q, start_grip_mm=start_grip,
                                max_speed_deg_s=20.0, max_acc_deg_s2=max_acc, ramp_s=0.0)
        self.target_q = np.array(start_q, dtype=float)
        self.target_grip = float(start_grip)
        self.speed = 20.0
        self.speed_scale = 1.0
        self.speed_pct = speed_pct
        self.track_err_stop = track_err_stop
        self.q_cmd, self.g_cmd = self.flt.q.copy(), self.flt.grip
        self.q_fb, self.g_fb = self.q_cmd.copy(), self.g_cmd
        self.enabled = False
        self.fault = None
        self.lock = threading.Lock()
        self.stop_evt = threading.Event()
        self.view_pose = None
        self.thread = threading.Thread(target=self._run, daemon=True)

    def enable(self):
        if self.live:
            self.arm.enable(speed_pct=self.speed_pct)
            q = self.arm.joints_deg()
            with self.lock:  # restart from the measured pose: no jump
                self.flt.q[:] = q
                self.flt.v[:] = 0
                self.flt.target_f[:] = q
                self.flt.prev_target = None
                self.target_q = q.copy()
        self.enabled = True

    def _run(self):
        rate = Rate(100)
        dt = 0.01
        bad_since = None
        with HighResTimer():
            while not self.stop_evt.is_set():
                with self.lock:
                    self.flt.max_speed = self.speed * self.speed_scale
                    q, g = self.flt.step(self.target_q, self.target_grip, dt)
                self.q_cmd, self.g_cmd = q, g
                if self.live:
                    if self.enabled and self.fault is None:
                        self.arm.command(q, g, self.speed_pct)
                    self.q_fb, self.g_fb = self.arm.joints_deg(), self.arm.gripper_mm()
                    err = float(np.max(np.abs(q - self.q_fb)))
                    if self.enabled and err > self.track_err_stop:
                        bad_since = bad_since or time.perf_counter()
                        if time.perf_counter() - bad_since > 0.5 and self.fault is None:
                            self.emergency_stop(f"跟踪误差 {err:.0f}° 超过 0.5 s（被挡住/碰撞？）")
                    else:
                        bad_since = None
                else:
                    self.q_fb, self.g_fb = q, g
                self.view_pose = (self.q_fb.copy(), self.g_fb)
                dt = rate.sleep()

    def emergency_stop(self, why="急停"):
        self.fault = why
        if self.live and self.arm is not None and self.arm.piper is not None:
            try:
                self.arm.emergency_stop()
            except Exception:  # noqa: BLE001
                pass

    def hold(self):
        with self.lock:
            self.target_q = self.flt.q.copy()
            self.flt.hold()

    def move(self, q, grip=None, speed=20.0, check=None, timeout=30.0):
        """Blocking joint move; `check()` is called while waiting and may raise Aborted."""
        with self.lock:
            self.target_q = np.asarray(q, dtype=float).copy()
            if grip is not None:
                self.target_grip = float(grip)
            self.speed = speed
            self.flt.prev_target = None  # a new waypoint is a deliberate jump, not a glitch
        target = np.clip(self.target_q, self.flt.lim[:, 0], self.flt.lim[:, 1])
        t0 = time.time()
        while True:
            if check:
                check()
            if self.fault:
                raise Aborted(self.fault)
            near_cmd = np.max(np.abs(self.q_cmd - target)) < 0.3 and abs(self.g_cmd - self.target_grip) < 1.0
            near_fb = not self.live or np.max(np.abs(self.q_fb - target)) < 2.5
            if near_cmd and near_fb:
                return
            if time.time() - t0 > timeout:
                raise Aborted(f"移动超时（{timeout:.0f}s）")
            time.sleep(0.01)

    def grip(self, mm, settle=0.8, check=None):
        with self.lock:
            self.target_grip = float(mm)
        t0 = time.time()
        while time.time() - t0 < settle + abs(self.g_cmd - mm) / 80.0:
            if check:
                check()
            if self.fault:
                raise Aborted(self.fault)
            time.sleep(0.02)

    def disable(self):
        if self.live and self.enabled:
            self.arm.disable()
        self.enabled = False

    def close(self):
        self.stop_evt.set()
        self.thread.join(timeout=1.0)
        if self.arm is not None:
            self.arm.disconnect()


# ---------------------------------------------------------------------- vision
class Vision:
    """Trigger camera (D415 or UVC) + hand detection in a thread; wrist camera frames for display."""

    def __init__(self, trigger_uvc: int | None, wrist_index: int | None, roi, hold_s: float):
        from vision.palm import PalmDetector, PalmTrigger
        self.detector = PalmDetector()
        self.trigger = PalmTrigger(roi=roi, hold_s=hold_s)
        self.roi = roi
        self.trigger_uvc = trigger_uvc
        self.cam = None
        self.wrist = None
        self.wrist_index = wrist_index
        self.overlay = None
        self.fired = threading.Event()
        self.armed = False  # only fire while the bot is idle and waiting
        self.fps = 0.0
        self.palm_xyz = None
        self.error = None
        self.stop_evt = threading.Event()
        self.thread = threading.Thread(target=self._run, daemon=True)

    def _open(self):
        from vision.cameras import RealSenseCam, UvcCam
        if self.trigger_uvc is None:
            self.cam = RealSenseCam().start()
        else:
            self.cam = UvcCam(self.trigger_uvc).start()
        if self.wrist_index is not None:
            try:
                self.wrist = UvcCam(self.wrist_index).start()
            except Exception as e:  # noqa: BLE001
                self.error = f"爪上相机打不开: {e}"

    def _read(self):
        if self.trigger_uvc is None:
            bgr, depth = self.cam.read()
            return bgr, depth
        return self.cam.latest(), None

    def _run(self):
        from vision.palm import draw_overlay, palm_xyz
        try:
            self._open()
        except Exception as e:  # noqa: BLE001
            self.error = f"触发相机打不开: {e}"
            return
        t_last = time.perf_counter()
        while not self.stop_evt.is_set():
            bgr, depth = self._read()
            if bgr is None:
                time.sleep(0.01)
                continue
            now = time.perf_counter()
            hands = self.detector.detect(bgr, int(now * 1000))
            if self.trigger.update(hands, bgr.shape, now) and self.armed:
                self.fired.set()
            palm = self.trigger.palm
            self.palm_xyz = palm_xyz(palm, depth, self.cam.intrinsics) if (palm and depth is not None) else None
            self.overlay = draw_overlay(bgr, hands, self.roi)
            self.fps = 0.9 * self.fps + 0.1 / max(now - t_last, 1e-3)
            t_last = now
            if self.trigger_uvc is not None:
                time.sleep(0.02)

    def wrist_frame(self):
        return self.wrist.latest() if self.wrist is not None else None

    def close(self):
        self.stop_evt.set()
        self.thread.join(timeout=2.0)
        for c in (self.cam, self.wrist):
            if c is not None:
                try:
                    c.stop()
                except Exception:  # noqa: BLE001
                    pass
        self.detector.close()


# ---------------------------------------------------------------------- behaviour
class CandyBot:
    def __init__(self, args):
        self.args = args
        self.wps = wp.load(Path(args.waypoints)) if args.waypoints else wp.load()
        self.state = "INIT"
        self.message = ""
        self.given = 0
        self.misses = 0
        self.holding = False
        self.start_evt = threading.Event()
        self.pause_evt = threading.Event()
        self.finish_evt = threading.Event()
        self.estop_evt = threading.Event()
        self.done = threading.Event()
        self.arm: ArmExec | None = None
        self.vision: Vision | None = None
        self.thread = threading.Thread(target=self._run, daemon=True)

    def say(self, state=None, msg=None):
        if state:
            self.state = state
        if msg is not None:
            self.message = msg

    def emergency_stop(self):
        self.estop_evt.set()
        if self.arm:
            self.arm.emergency_stop("急停按钮")

    def _check(self):
        if self.estop_evt.is_set():
            raise Aborted("急停")
        if self.finish_evt.is_set():
            raise Aborted("finish")
        if self.pause_evt.is_set():
            self.arm.hold()
            prev = self.state
            self.say("PAUSED")
            while self.pause_evt.is_set():
                if self.estop_evt.is_set() or self.finish_evt.is_set():
                    raise Aborted("急停" if self.estop_evt.is_set() else "finish")
                time.sleep(0.05)
            self.say(prev)
            raise Aborted("resume")  # restart the current step cleanly after a pause

    def _sp(self, base):
        return base * self.args.speed_scale

    def _go(self, name, grip=None, speed=25.0):
        self.arm.move(wp.q(self.wps, name), grip=grip, speed=self._sp(speed), check=self._check)

    def _pick(self) -> bool:
        grasps = wp.grasp_names(self.wps)
        for attempt in range(self.args.max_attempts):
            g = grasps[(self.misses + attempt) % len(grasps)]
            self.say("PICK", f"夹糖：第 {attempt + 1} 次（{g}）")
            self._go("box_above", grip=GRIP_OPEN_MM, speed=30)
            self._go(g, grip=GRIP_OPEN_MM, speed=15)
            self.arm.grip(0.0, settle=1.0, check=self._check)
            width = self.arm.g_fb if self.args.live else 15.0
            self._go("box_above", speed=15)
            if width > EMPTY_GRIP_MM:
                self.say(msg=f"夹到了（夹爪开口 {width:.1f} mm）")
                return True
            self.misses += 1
            self.say(msg=f"没夹到（开口 {width:.1f} mm），换下一个抓取点")
        return False

    def _deliver(self):
        self.say("DELIVER", "送到递糖区")
        if "transit" in self.wps:
            self._go("transit", speed=30)
        self._go("handoff_above", speed=20)
        self.arm.grip(GRIP_OPEN_MM, settle=1.0, check=self._check)
        self.holding = False
        self.given += 1
        self.say(msg=f"已发出第 {self.given} 颗糖 🍬")
        if "transit" in self.wps:
            self._go("transit", speed=30)
        self._go("home", grip=GRIP_OPEN_MM / 2, speed=30)

    def _cycle(self):
        if not self.holding:
            if not self._pick():
                self._go("home", speed=30)
                self.say("IDLE", "连续没夹到糖，请检查糖盒（糖太少/位置变了），然后继续")
                return
            self.holding = True
        self._deliver()

    def _run(self):
        a = self.args
        try:
            miss = wp.missing(self.wps)
            if miss:
                self.say("ERROR", f"缺少点位: {', '.join(miss)}。请先在遥操作面板里用“记录点位”示教。")
                return
            self.say("INIT", "打开相机…")
            self.vision = Vision(a.trigger_uvc, a.wrist_index, a.roi, a.hold_s)
            self.vision.thread.start()
            self.say("INIT", "连接机械臂…" if a.live else "仿真模式（真机不会动）")
            self.arm = ArmExec(a.live, wp.q(self.wps, "home"), self.wps["home"]["grip"],
                               a.max_acc, a.speed_pct, a.track_err_stop)
            self.arm.thread.start()
            if a.live and np.max(np.abs(self.arm.q_fb - wp.q(self.wps, "home"))) > 15:
                self.say("WAIT_START", "⚠ 机械臂不在休息位附近（差 > 15°）。开始后会先慢慢移到休息位。")
            else:
                self.say("WAIT_START", "准备好了，点“开始”")
            while not self.start_evt.is_set():
                if self.finish_evt.is_set() or self.estop_evt.is_set():
                    return
                time.sleep(0.05)
            self.arm.enable()
            self.say("HOMING", "移动到休息位")
            self._go("home", grip=GRIP_OPEN_MM / 2, speed=15)

            while True:
                try:
                    self.say("IDLE", "等待摊开的手掌…" + ("（手里还拿着一颗糖）" if self.holding else ""))
                    self.vision.fired.clear()
                    self.vision.trigger.reset()
                    self.vision.armed = True
                    while not self.vision.fired.is_set():
                        self._check()
                        time.sleep(0.02)
                    self.vision.armed = False
                    self._cycle()
                    self.say("COOLDOWN", "等手离开…")
                    t0 = time.time()
                    while not self.vision.trigger.hand_gone(time.perf_counter(), 1.5) and time.time() - t0 < 10:
                        self._check()
                        time.sleep(0.05)
                except Aborted as e:
                    if str(e) == "resume":
                        continue
                    raise
        except Aborted as e:
            if str(e) == "finish":
                self._finish()
            else:
                self.say("ESTOP", f"已停止：{e}。托住机械臂；恢复需断电重启。")
        except Exception as e:  # noqa: BLE001
            if self.arm:
                self.arm.emergency_stop(str(e))
            self.say("ERROR", f"{type(e).__name__}: {e}")
            raise
        finally:
            if self.vision:
                self.vision.armed = False
            self.done.set()

    def _finish(self):
        self.finish_evt.clear()
        if self.arm is None or not self.arm.enabled:
            self.say("STOPPED", "已结束")
            return
        self.say("HOMING", "回休息位后失能…")
        self._go("home", grip=self.wps["home"]["grip"], speed=15)
        time.sleep(0.5)
        self.arm.disable()
        self.say("STOPPED", f"已回休息位并失能。今天共发出 {self.given} 颗糖。")

    def close(self):
        if self.vision:
            self.vision.close()
        if self.arm:
            self.arm.close()


# ---------------------------------------------------------------------- panel
STATE_STYLE = {
    "INIT": ("初始化…", "#888888"), "WAIT_START": ("准备就绪，点“开始”", "#2e8b57"),
    "HOMING": ("移动到休息位", "#1565c0"), "IDLE": ("等待摊开的手掌 ✋", "#1e7d32"),
    "PICK": ("夹糖中", "#6a1b9a"), "DELIVER": ("送糖中", "#ef6c00"), "COOLDOWN": ("等手离开", "#00796b"),
    "PAUSED": ("已暂停（原地保持）", "#d98c00"), "STOPPED": ("已结束", "#555555"),
    "ESTOP": ("已急停", "#c62828"), "ERROR": ("出错", "#c62828"),
}


def run_panel(bot: CandyBot):
    import tkinter as tk
    from tkinter import messagebox

    import cv2
    from PIL import Image, ImageTk

    live = bot.args.live
    root = tk.Tk()
    root.title(("真机 LIVE" if live else "仿真 DRY RUN") + " · 🎃 万圣节递糖机器人")
    root.attributes("-topmost", True)
    root.configure(bg="#1e1e1e")
    fg, bg = "#eeeeee", "#1e1e1e"
    big = ("Microsoft YaHei UI", 14, "bold")

    tk.Label(root, text="LIVE 真机模式" if live else "DRY RUN（真机不会动，仿真窗口里看动作）", font=big,
             fg="white", bg="#c62828" if live else "#455a64").pack(fill="x")
    top = tk.Frame(root, bg=bg)
    top.pack(fill="x", padx=10, pady=8)
    estop = tk.Button(top, text="急 停 (ESC)", font=("Microsoft YaHei UI", 24, "bold"), fg="white", bg="#d32f2f",
                      activebackground="#b71c1c", width=10, height=2, command=bot.emergency_stop)
    estop.pack(side="left")
    side = tk.Frame(top, bg=bg)
    side.pack(side="left", fill="both", expand=True, padx=10)
    state_lbl = tk.Label(side, text="", font=big, fg="white", bg="#888888")
    state_lbl.pack(fill="x")
    msg_lbl = tk.Label(side, text="", font=("Microsoft YaHei UI", 10), fg=fg, bg=bg, wraplength=560, justify="left")
    msg_lbl.pack(fill="x", pady=4)
    count_lbl = tk.Label(side, text="", font=("Microsoft YaHei UI", 12, "bold"), fg="#ffcc80", bg=bg)
    count_lbl.pack(fill="x")

    def start():
        if live and not messagebox.askyesno("确认使能", "即将使能 Piper X 并开始自动递糖。\n\n周围已清空、有人看着急停？"):
            return
        bot.start_evt.set()

    def finish():
        bot.finish_evt.set()

    btns = tk.Frame(root, bg=bg)
    btns.pack(fill="x", padx=10)
    b = dict(font=("Microsoft YaHei UI", 12, "bold"), height=2, fg="white", relief="flat")
    start_btn = tk.Button(btns, text="开始", bg="#2e7d32", command=start, **b)
    pause_btn = tk.Button(btns, text="暂停 / 继续 (空格)", bg="#ef6c00",
                          command=lambda: bot.pause_evt.clear() if bot.pause_evt.is_set() else bot.pause_evt.set(), **b)
    finish_btn = tk.Button(btns, text="回休息位并失能（结束）", bg="#1565c0", command=finish, **b)
    for i, w in enumerate((start_btn, pause_btn, finish_btn)):
        w.grid(row=0, column=i, sticky="nsew", padx=3)
        btns.columnconfigure(i, weight=1)

    sp = tk.Frame(root, bg=bg)
    sp.pack(fill="x", padx=10, pady=4)
    tk.Label(sp, text="速度倍率", font=("Microsoft YaHei UI", 10), fg=fg, bg=bg).pack(side="left")
    speed = tk.Scale(sp, from_=0.3, to=1.5, resolution=0.1, orient="horizontal", fg=fg, bg=bg, highlightthickness=0,
                     troughcolor="#444", command=lambda v: setattr(bot.arm, "speed_scale", float(v)) if bot.arm else None)
    speed.set(1.0)
    speed.pack(side="left", fill="x", expand=True, padx=8)

    cams = tk.Frame(root, bg=bg)
    cams.pack(padx=10, pady=6)
    trig_lbl = tk.Label(cams, bg="black", width=480, height=360)
    wrist_lbl = tk.Label(cams, bg="black", width=480, height=360)
    trig_lbl.grid(row=0, column=0, padx=3)
    wrist_lbl.grid(row=0, column=1, padx=3)
    tk.Label(cams, text="固定相机（触发）", fg=fg, bg=bg).grid(row=1, column=0)
    tk.Label(cams, text="爪上相机", fg=fg, bg=bg).grid(row=1, column=1)
    info = tk.Label(root, text="", font=("Consolas", 10), fg=fg, bg="#111111", anchor="w", justify="left")
    info.pack(fill="x", padx=10, pady=(0, 10))

    def show(label, bgr):
        img = ImageTk.PhotoImage(Image.fromarray(cv2.cvtColor(cv2.resize(bgr, (480, 360)), cv2.COLOR_BGR2RGB)))
        label.config(image=img, width=480, height=360)
        label.image = img

    view = {"v": None}
    if not bot.args.no_viewer:
        from sim.view import PiperView
        view["v"] = PiperView().__enter__()

    def refresh():
        label, colour = STATE_STYLE.get(bot.state, (bot.state, "#888888"))
        state_lbl.config(text=label, bg=colour)
        msg_lbl.config(text=bot.message)
        count_lbl.config(text=f"已发糖 {bot.given} 颗    没夹到 {bot.misses} 次")
        start_btn.config(state="normal" if bot.state == "WAIT_START" else "disabled")
        v = bot.vision
        if v is not None:
            if v.overlay is not None:
                frame = v.overlay.copy()
                p = v.trigger.progress(time.perf_counter())
                if p > 0 and bot.state == "IDLE":
                    cv2.rectangle(frame, (0, frame.shape[0] - 14), (int(frame.shape[1] * p), frame.shape[0]),
                                  (0, 220, 0), -1)
                show(trig_lbl, frame)
            wf = v.wrist_frame()
            if wf is not None:
                show(wrist_lbl, wf)
            xyz = "" if v.palm_xyz is None else "  手掌位置(相机坐标) x=%.2f y=%.2f z=%.2f m" % tuple(v.palm_xyz)
            info.config(text=f"触发相机 {v.fps:4.1f} fps{xyz}" + (f"   ⚠ {v.error}" if v.error else ""))
        if bot.arm is not None and view["v"] is not None and bot.arm.view_pose is not None:
            try:
                if view["v"].is_running():
                    view["v"].show(*bot.arm.view_pose)
            except Exception:  # noqa: BLE001
                pass
        root.after(50, refresh)

    def on_close():
        if bot.state in ("PICK", "DELIVER", "HOMING", "IDLE", "COOLDOWN", "PAUSED") and live:
            if not messagebox.askyesno("关闭", "关闭面板后，机械臂会先回休息位再失能（最多 30 秒）。\n确定关闭吗？"):
                return
        bot.pause_evt.clear()
        bot.finish_evt.set()
        root.destroy()

    root.bind("<Escape>", lambda e: bot.emergency_stop())
    root.bind("<space>", lambda e: bot.pause_evt.clear() if bot.pause_evt.is_set() else bot.pause_evt.set())
    root.protocol("WM_DELETE_WINDOW", on_close)
    refresh()
    root.mainloop()
    if view["v"] is not None:
        view["v"].__exit__(None, None, None)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--live", action="store_true", help="drive the real PiPER X")
    ap.add_argument("--trigger-uvc", type=int, default=None,
                    help="use this UVC camera index for hand detection instead of the D415")
    ap.add_argument("--wrist-index", type=int, default=1, help="UVC index of the gripper camera (Dabai colour)")
    ap.add_argument("--roi", type=float, nargs=4, default=None, metavar=("X0", "Y0", "X1", "Y1"),
                    help="hand-off zone in the trigger image, normalised 0..1 (default: whole image)")
    ap.add_argument("--hold-s", type=float, default=0.8, help="open palm must be held this long to trigger")
    ap.add_argument("--speed-scale", type=float, default=1.0, help="multiplies all waypoint speeds")
    ap.add_argument("--max-acc", type=float, default=120.0)
    ap.add_argument("--speed-pct", type=int, default=50)
    ap.add_argument("--max-attempts", type=int, default=3, help="grasp retries before giving up")
    ap.add_argument("--track-err-stop", type=float, default=20.0)
    ap.add_argument("--no-viewer", action="store_true")
    ap.add_argument("--waypoints", default=None, help="waypoints json (default candy/waypoints.json)")
    args = ap.parse_args()

    bot = CandyBot(args)
    bot.thread.start()
    try:
        run_panel(bot)
    finally:
        bot.finish_evt.set()
        bot.done.wait(timeout=30.0 if args.live else 3.0)
        bot.close()


if __name__ == "__main__":
    main()
