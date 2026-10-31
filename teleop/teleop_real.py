"""SO-101 leader -> REAL PiPER X teleoperation, with an always-on-top emergency control panel.

Default is a DRY RUN: the PiPER is only read (never enabled); the commands that *would* be sent
are shown in the MuJoCo viewer. Add --live to actually enable and drive the arm.

    .venv\\Scripts\\python.exe -m teleop.teleop_real                       # dry run
    .venv\\Scripts\\python.exe -m teleop.teleop_real --live --joints 1     # live, only J1 follows
    .venv\\Scripts\\python.exe -m teleop.teleop_real --live                # live, all joints + gripper

Panel keys (panel window focused): ESC = emergency stop, SPACE = pause/resume, H = home + disable.
"""

import argparse
import sys
import threading
import time
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from teleop.mapping import SafetyFilter, load_config, map_leader, save_config  # noqa: E402
from teleop.piper_driver import PiperX  # noqa: E402
from teleop.so101_reader import LeaderStream, SO101Reader  # noqa: E402
from teleop.timing import HighResTimer, Rate  # noqa: E402


def parse_joints(s: str) -> tuple[np.ndarray, bool]:
    """'1,2,3,4,6,G' -> (bool mask for J1..J6, gripper_enabled)."""
    items = {x.strip().upper() for x in s.split(",") if x.strip()}
    mask = np.array([str(i + 1) in items for i in range(6)])
    return mask, "G" in items


@dataclass
class Status:
    phase: str = "CONNECTING"  # CONNECTING ALIGNING READY FOLLOWING PAUSED HOMING STOPPED ESTOP ERROR
    message: str = ""
    diff: np.ndarray = field(default_factory=lambda: np.zeros(6))
    aligned: bool = False
    q_cmd: np.ndarray = field(default_factory=lambda: np.zeros(6))
    q_fb: np.ndarray = field(default_factory=lambda: np.zeros(6))
    g_cmd: float = 0.0
    track_err: float = 0.0
    leader_age: float = 0.0
    enabled: list = field(default_factory=lambda: [False] * 6)
    arm_status: str = ""
    loop_ms: float = 0.0


class Teleop:
    """Runs the control loop in a background thread; the UI only sets flags and reads `status`."""

    def __init__(self, args):
        self.args = args
        self.mask, self.grip_on = parse_joints(args.joints)
        self.mask[4] = False  # J5 (wrist yaw) has no SO-101 counterpart
        self.cfg = load_config()
        self.status = Status()
        self.lock = threading.Lock()
        self.max_speed = args.max_speed
        self.arm: PiperX | None = None
        self.start_evt = threading.Event()
        self.estop_evt = threading.Event()
        self.quit_evt = threading.Event()
        self.home_evt = threading.Event()
        self.pause_toggle_evt = threading.Event()
        self.calib_base_evt = threading.Event()
        self.done = threading.Event()
        self.thread = threading.Thread(target=self._run, daemon=True)

    # ------------------------------------------------------------------ UI-facing commands
    def emergency_stop(self) -> None:
        """Sent immediately from the caller's thread too, so it works even if the loop is stuck."""
        self.estop_evt.set()
        if self.args.live and self.arm is not None and self.arm.piper is not None:
            try:
                self.arm.emergency_stop()
            except Exception:  # noqa: BLE001 - the loop thread sends it again
                pass

    def _set(self, **kw):
        with self.lock:
            for k, v in kw.items():
                setattr(self.status, k, v)

    def snapshot(self) -> Status:
        with self.lock:
            s = self.status
            return Status(**{k: (v.copy() if isinstance(v, np.ndarray) else v) for k, v in vars(s).items()})

    # ------------------------------------------------------------------ control thread
    def _run(self):
        a = self.args
        reader = stream = None
        view = None
        try:
            reader = SO101Reader(a.port)
            reader.connect()
            stream = LeaderStream(reader).start()
            self.arm = arm = PiperX(allow_motion=a.live)
            arm.connect()
            time.sleep(0.5)
            q_start, g_start = arm.joints_deg(), arm.gripper_mm()
            # 'home' returns here before disabling, so start the program with the arm folded.
            q_home = q_start.copy()
            self._set(arm_status=f"fw {arm.firmware()} | {arm.status_line()}", q_fb=q_start, q_cmd=q_start)

            def leader_target():
                so, age = stream.latest()
                tgt, grip = map_leader(so, self.cfg)
                return np.where(self.mask, tgt, q_start), (grip if self.grip_on else g_start), age

            if not a.no_viewer:
                from sim.view import PiperView
                view = PiperView().__enter__()

            # ---------------- align, wait for the start button
            # The viewer shows where the SO-101 would send the arm, to help line it up.
            self._set(phase="ALIGNING", message="调整 SO-101 直到所有对齐差 < %.0f°（仿真窗口显示 SO-101 对应的目标姿态）"
                      % a.align_tol)
            while not self.start_evt.is_set():
                if self.estop_evt.is_set() or self.quit_evt.is_set():
                    self._set(phase="STOPPED", message="Stopped before start.")
                    return
                if self.calib_base_evt.is_set():
                    # The SO-101 base now points the same way as the PiPER base: choose the J1 zero so
                    # this leader reading maps exactly onto the arm's current J1, and persist it.
                    self.calib_base_evt.clear()
                    so, _ = stream.latest()
                    c = self.cfg
                    c.so_zero[0] = so["shoulder_pan"] - q_start[0] / (c.sign[0] * c.scale[0])
                    save_config(c)
                    self._set(message=f"底座已校准：SO-101 shoulder_pan {so['shoulder_pan']:.1f}° = Piper J1 "
                                      f"{q_start[0]:.1f}°（J1 零点 {c.so_zero[0]:.1f}，已保存）")
                tgt, grip, age = leader_target()
                diff = np.where(self.mask, tgt - q_start, 0.0)
                aligned = bool(np.max(np.abs(diff)) < a.align_tol)
                self._set(diff=diff, aligned=aligned, leader_age=age, q_fb=arm.joints_deg(), q_cmd=tgt,
                          phase="READY" if aligned else "ALIGNING")
                if view is not None and view.is_running():
                    view.show(np.clip(tgt, -170, 180), grip)
                time.sleep(0.05)

            # ---------------- enable
            if a.live:
                self._set(message="Enabling motors...")
                arm.enable(speed_pct=a.speed_pct)
                q_start = arm.joints_deg()  # start from the measured pose: no jump
            flt = SafetyFilter(self.cfg, start_deg=q_start, start_grip_mm=g_start,
                               max_speed_deg_s=self.max_speed, max_acc_deg_s2=a.max_acc)

            self._set(phase="FOLLOWING", message="Following the SO-101.")
            paused = homing = False
            track_bad_since = None
            last_slow = 0.0
            rate = Rate(a.hz)
            dt = 1.0 / a.hz
            with HighResTimer():
                while True:
                    if self.estop_evt.is_set():
                        if a.live:
                            arm.emergency_stop()
                        self._set(phase="ESTOP", message="EMERGENCY STOP sent. Support the arm; power-cycle to recover.")
                        return
                    if self.quit_evt.is_set():
                        self._set(phase="STOPPED", message="Stopped following. Arm holds its last position (still enabled)."
                                  if a.live else "Dry run stopped.")
                        return
                    if self.pause_toggle_evt.is_set():
                        self.pause_toggle_evt.clear()
                        paused = not paused
                        if paused:
                            flt.hold()
                        self._set(phase="PAUSED" if paused else "FOLLOWING",
                                  message="Paused: arm holds position." if paused else "Following the SO-101.")
                    if self.home_evt.is_set() and not homing:
                        homing, paused = True, False
                        flt.ramp_s, flt.t = 0.0, 0.0
                        self._set(phase="HOMING", message="Returning to the start (rest) pose at 20 deg/s...")
                    flt.max_speed = 20.0 if homing else self.max_speed

                    if homing:
                        tgt, grip = q_home, g_start
                        at_home = np.max(np.abs(flt.q - np.clip(q_home, flt.lim[:, 0], flt.lim[:, 1]))) < 0.5
                        if a.live:
                            at_home = at_home and np.max(np.abs(arm.joints_deg() - q_home)) < 3.0
                        if at_home:
                            if a.live:
                                time.sleep(0.5)
                                arm.disable()
                            self._set(phase="STOPPED", message="At rest pose. Motors disabled." if a.live
                                      else "At rest pose (dry run).")
                            return
                    elif paused:
                        tgt, grip = flt.q, flt.grip
                    else:
                        tgt, grip, age = leader_target()
                        if age > 0.2:  # leader stream stale -> hold
                            flt.hold()
                            tgt, grip = flt.q, flt.grip

                    q_cmd, g_cmd = flt.step(tgt, grip, dt)
                    if a.live:
                        arm.command(q_cmd, g_cmd, a.speed_pct)

                    q_fb = arm.joints_deg()
                    track_err = float(np.max(np.abs(q_cmd - q_fb)))
                    if a.live and track_err > a.track_err_stop:
                        track_bad_since = track_bad_since or time.perf_counter()
                        if time.perf_counter() - track_bad_since > 0.5:
                            arm.emergency_stop()
                            self._set(phase="ESTOP", message=f"Tracking error {track_err:.1f} deg for 0.5 s "
                                                            "(blocked/collision?) -> EMERGENCY STOP.")
                            return
                    else:
                        track_bad_since = None

                    if view is not None:
                        view.show(q_fb if a.live else q_cmd, arm.gripper_mm() if a.live else g_cmd)
                        if not view.is_running():
                            view = None

                    self._set(q_cmd=q_cmd, q_fb=q_fb, g_cmd=g_cmd, track_err=track_err, loop_ms=dt * 1000)
                    now = time.perf_counter()
                    if now - last_slow > 0.25:  # slower-changing info
                        last_slow = now
                        self._set(enabled=arm.enabled(), arm_status=arm.status_line(), leader_age=stream.latest()[1])
                    dt = rate.sleep()
        except Exception as e:  # noqa: BLE001
            if a.live and self.arm is not None and self.arm.piper is not None:
                self.arm.emergency_stop()
            self._set(phase="ERROR", message=f"{type(e).__name__}: {e}" + (" -> EMERGENCY STOP sent." if a.live else ""))
            raise
        finally:
            if view is not None:
                view.__exit__(None, None, None)
            if stream is not None:
                stream.stop()
            if reader is not None:
                reader.disconnect()
            if self.arm is not None:
                self.arm.disconnect()
            self.done.set()


# ---------------------------------------------------------------------- panel
PHASE_STYLE = {  # phase -> (label, colour)
    "CONNECTING": ("连接中…", "#888888"), "ALIGNING": ("对齐中：请调整 SO-101", "#d98c00"),
    "READY": ("已对齐，可以开始", "#2e8b57"), "FOLLOWING": ("跟随中", "#1e7d32"),
    "PAUSED": ("已暂停（原地保持）", "#d98c00"), "HOMING": ("回休息位中…", "#1565c0"),
    "STOPPED": ("已停止", "#555555"), "ESTOP": ("已急停", "#c62828"),
    "ERROR": ("出错（真机模式下已自动急停）", "#c62828"),
}


def run_panel(t: Teleop):
    import tkinter as tk
    from tkinter import messagebox

    live = t.args.live
    root = tk.Tk()
    root.title(("真机 LIVE" if live else "DRY RUN") + " · Piper X 紧急控制面板")
    root.attributes("-topmost", True)
    root.configure(bg="#1e1e1e")
    root.geometry("560x780+40+40")
    fg, bg = "#eeeeee", "#1e1e1e"
    big = ("Microsoft YaHei UI", 14, "bold")
    mono = ("Consolas", 12)

    tk.Label(root, text="LIVE 真机模式" if live else "DRY RUN（真机不会动）", font=big, fg="white",
             bg="#c62828" if live else "#455a64").pack(fill="x")

    estop = tk.Button(root, text="急 停\n(ESC)", font=("Microsoft YaHei UI", 30, "bold"), fg="white",
                      bg="#d32f2f", activebackground="#b71c1c", activeforeground="white", height=2,
                      command=t.emergency_stop)
    estop.pack(fill="x", padx=12, pady=12)

    phase_lbl = tk.Label(root, text="", font=big, fg="white", bg="#888888")
    phase_lbl.pack(fill="x", padx=12)
    msg_lbl = tk.Label(root, text="", font=("Microsoft YaHei UI", 10), fg=fg, bg=bg, wraplength=520, justify="left")
    msg_lbl.pack(fill="x", padx=12, pady=(4, 8))

    row = tk.Frame(root, bg=bg)
    row.pack(fill="x", padx=12)

    def start():
        s = t.snapshot()
        if not s.aligned and live:
            return
        if live and not messagebox.askyesno("确认使能", "即将使能 Piper X 电机并开始跟随。\n\n周围已清空、手在电源开关旁？"):
            return
        t.start_evt.set()

    def confirm_close():
        s = t.snapshot()
        if live and s.phase in ("FOLLOWING", "PAUSED", "HOMING"):
            if not messagebox.askyesno("关闭面板", "关闭面板会停止跟随，机械臂保持当前位置（仍使能）。\n确定吗？"):
                return
        t.quit_evt.set()
        root.after(300, root.destroy)

    btn = dict(font=("Microsoft YaHei UI", 12, "bold"), height=2, fg="white", relief="flat")
    start_btn = tk.Button(row, text="使能并开始跟随" if live else "开始 (dry run)", bg="#2e7d32", command=start, **btn)
    pause_btn = tk.Button(row, text="暂停 / 继续\n(空格)", bg="#ef6c00", command=t.pause_toggle_evt.set, **btn)
    home_btn = tk.Button(row, text="回休息位并失能\n(H)", bg="#1565c0", command=t.home_evt.set, **btn)
    quit_btn = tk.Button(row, text="停止跟随\n(保持)", bg="#546e7a", command=confirm_close, **btn)
    for i, b in enumerate((start_btn, pause_btn, home_btn, quit_btn)):
        b.grid(row=0, column=i, sticky="nsew", padx=3)
        row.columnconfigure(i, weight=1)

    def calib_base():
        if messagebox.askyesno("校准底座 (J1)", "SO-101 底座现在和 Piper 底座朝向一致了吗？\n\n"
                                               "确认后，以 SO-101 当前底座角度作为 Piper J1 的对应零点（会保存）。"):
            t.calib_base_evt.set()

    calib_btn = tk.Button(root, text="校准底座 (J1)：SO-101 当前朝向 = Piper 当前朝向", bg="#6a1b9a",
                          fg="white", relief="flat", font=("Microsoft YaHei UI", 11, "bold"), command=calib_base)
    calib_btn.pack(fill="x", padx=12, pady=(8, 0))

    sp = tk.Frame(root, bg=bg)
    sp.pack(fill="x", padx=12, pady=8)
    tk.Label(sp, text="速度上限 (°/s)", font=("Microsoft YaHei UI", 10), fg=fg, bg=bg).pack(side="left")
    speed = tk.Scale(sp, from_=5, to=t.args.max_speed, orient="horizontal", resolution=5, fg=fg, bg=bg,
                     highlightthickness=0, troughcolor="#444", command=lambda v: setattr(t, "max_speed", float(v)))
    speed.set(t.args.max_speed)
    speed.pack(side="left", fill="x", expand=True, padx=8)

    info = tk.Label(root, text="", font=mono, fg=fg, bg="#111111", justify="left", anchor="nw")
    info.pack(fill="both", expand=True, padx=12, pady=(0, 12))

    root.bind("<Escape>", lambda e: t.emergency_stop())
    root.bind("<space>", lambda e: t.pause_toggle_evt.set())
    root.bind("<KeyPress-h>", lambda e: t.home_evt.set())
    root.protocol("WM_DELETE_WINDOW", confirm_close)

    def fmt(arr, sign=False):
        return " ".join(f"{v:+7.1f}" if sign else f"{v:7.1f}" for v in arr)

    def refresh():
        s = t.snapshot()
        label, colour = PHASE_STYLE.get(s.phase, (s.phase, "#888888"))
        phase_lbl.config(text=label, bg=colour)
        msg_lbl.config(text=s.message)
        pre = s.phase in ("CONNECTING", "ALIGNING", "READY")
        start_btn.config(state="normal" if pre and (s.aligned or not live) else "disabled")
        calib_btn.config(state="normal" if s.phase in ("ALIGNING", "READY") else "disabled")
        running = s.phase in ("FOLLOWING", "PAUSED")
        pause_btn.config(state="normal" if running else "disabled")
        home_btn.config(state="normal" if running else "disabled")
        lines = ["         J1      J2      J3      J4      J5      J6"]
        if pre:
            lines.append("对齐差 " + fmt(s.diff, sign=True))
            bad = [f"J{i + 1}" for i in range(6) if abs(s.diff[i]) >= t.args.align_tol]
            lines.append(f"  需 < {t.args.align_tol:.0f}°  未对齐: {' '.join(bad) or '无'}")
        lines.append("实际   " + fmt(s.q_fb))
        lines.append("指令   " + fmt(s.q_cmd))
        lines.append("")
        lines.append(f"夹爪指令 {s.g_cmd:5.1f} mm   跟踪误差 {s.track_err:5.1f}°")
        lines.append(f"SO-101 数据延迟 {s.leader_age * 1000:5.0f} ms   循环 {s.loop_ms:4.1f} ms")
        lines.append("电机使能 " + " ".join("●" if e else "○" for e in s.enabled))
        lines.append("")
        lines.append(s.arm_status[:60])
        if len(s.arm_status) > 60:
            lines.append(s.arm_status[60:120])
        info.config(text="\n".join(lines))
        if t.done.is_set() and s.phase in ("STOPPED", "ESTOP", "ERROR"):
            estop.config(text="急 停\n(已结束)" if s.phase != "ESTOP" else "已急停")
        root.after(100, refresh)

    refresh()
    root.mainloop()


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--live", action="store_true", help="really enable and move the PiPER X")
    ap.add_argument("--port", default="COM3", help="SO-101 leader serial port")
    ap.add_argument("--joints", default="1,2,3,4,6,G",
                    help="which PiPER joints follow the leader (others hold their start position); "
                         "J5 has no leader joint and is always held")
    ap.add_argument("--hz", type=float, default=100.0)
    ap.add_argument("--max-speed", type=float, default=45.0, help="joint speed cap, deg/s")
    ap.add_argument("--max-acc", type=float, default=200.0, help="joint acceleration cap, deg/s^2")
    ap.add_argument("--speed-pct", type=int, default=50,
                    help="PiPER internal MOVE J speed percentage (its own limiter, 1-100)")
    ap.add_argument("--align-tol", type=float, default=20.0,
                    help="leader must be within this many degrees of the arm before following can start")
    ap.add_argument("--track-err-stop", type=float, default=25.0,
                    help="emergency stop if commanded vs. measured joint differs by more than this (deg) for 0.5 s")
    ap.add_argument("--no-viewer", action="store_true", help="don't open the MuJoCo viewer")
    args = ap.parse_args()

    t = Teleop(args)
    t.thread.start()
    try:
        run_panel(t)
    finally:
        t.quit_evt.set()
        t.done.wait(timeout=5.0)


if __name__ == "__main__":
    main()
