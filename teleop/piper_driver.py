"""PiPER X driver on Windows (piper_sdk + candleLight adapter via the `candle` python-can interface).

Safety model:
  * `PiperX(allow_motion=False)` (the default) can only read feedback. Every command that could make
    the arm move or go limp raises unless the object was created with `allow_motion=True`.
  * Joint limits are enforced by teleop.mapping.SafetyFilter; the SDK's own limits are PiPER (not
    PiPER X) values, so they are switched off here.

Units: joints in degrees, gripper as total opening in mm (SDK uses 0.001 deg / 0.001 mm).
"""

import time

import numpy as np
from piper_sdk import C_PiperInterface_V2

from .can_setup import CAN_BITRATE, CAN_CHANNEL, CAN_INTERFACE


def candle_channel_name() -> str:
    """piper_sdk needs a str channel; the candle interface accepts "<serial>:<channel>"."""
    import candle_api

    devs = candle_api.list_device()
    if not devs:
        raise ConnectionError(
            "candleLight adapter not accessible. Is it plugged in, and is no other program (e.g. Duet "
            "Display, another python script, a CAN tool) holding it open?")
    return f"{devs[0].serial_number}:{CAN_CHANNEL}"


class MotionNotAllowed(RuntimeError):
    pass


class PiperX:
    def __init__(self, allow_motion: bool = False):
        self.allow_motion = allow_motion
        self.piper: C_PiperInterface_V2 | None = None
        self.channel = None

    # ---------------------------------------------------------------- connection / feedback (safe)
    def connect(self, timeout: float = 3.0) -> None:
        self.channel = candle_channel_name()
        self.piper = C_PiperInterface_V2(
            can_name=self.channel, judge_flag=False, can_auto_init=False,
            start_sdk_joint_limit=False, start_sdk_gripper_limit=False)
        self.piper.CreateCanBus(self.channel, bustype=CAN_INTERFACE, expected_bitrate=CAN_BITRATE, judge_flag=False)
        # piper_init only sends *query* frames (firmware version, motor limit tables).
        self.piper.ConnectPort(piper_init=True)
        t0 = time.time()
        while time.time() - t0 < timeout:
            if self.piper.GetArmJointMsgs().Hz > 0 and self.piper.GetArmStatus().Hz > 0:
                return
            time.sleep(0.05)
        raise ConnectionError("CAN opened but no PiPER feedback received. Is the arm powered on (24 V) "
                              "and the CAN cable plugged into the arm?")

    def joints_deg(self) -> np.ndarray:
        js = self.piper.GetArmJointMsgs().joint_state
        return np.array([js.joint_1, js.joint_2, js.joint_3, js.joint_4, js.joint_5, js.joint_6]) / 1000.0

    def gripper_mm(self) -> float:
        return self.piper.GetArmGripperMsgs().gripper_state.grippers_angle / 1000.0

    def enabled(self) -> list[bool]:
        return self.piper.GetArmEnableStatus()

    def firmware(self) -> str:
        fw = self.piper.GetPiperFirmwareVersion()
        return fw if isinstance(fw, str) else "unknown"

    def status(self):
        return self.piper.GetArmStatus().arm_status

    def feedback_hz(self) -> dict:
        return {"joints": self.piper.GetArmJointMsgs().Hz, "status": self.piper.GetArmStatus().Hz,
                "gripper": self.piper.GetArmGripperMsgs().Hz}

    def status_line(self) -> str:
        s = self.status()
        errs = s.err_status
        limits = [i + 1 for i in range(6) if getattr(errs, f"joint_{i + 1}_angle_limit")]
        comm = [i + 1 for i in range(6) if getattr(errs, f"communication_status_joint_{i + 1}")]
        return (f"ctrl_mode={s.ctrl_mode} arm_status={s.arm_status} mode={s.mode_feed} "
                f"motion={s.motion_status} err_code={s.err_code} "
                f"limit_joints={limits or '-'} comm_err_joints={comm or '-'}")

    # ---------------------------------------------------------------- motion (guarded)
    def _guard(self):
        if not self.allow_motion:
            raise MotionNotAllowed("PiperX was created with allow_motion=False")

    def enable(self, timeout: float = 5.0, speed_pct: int = 30) -> None:
        """Enable all joint motors + gripper and select CAN joint (MOVE J) position-velocity mode."""
        self._guard()
        t0 = time.time()
        while not all(self.enabled()):
            self.piper.EnablePiper()
            if time.time() - t0 > timeout:
                raise TimeoutError(f"PiPER did not enable within {timeout}s: {self.enabled()}")
            time.sleep(0.05)
        self.set_joint_mode(speed_pct)
        # Enable the gripper at its current opening so it does not snap shut.
        self.piper.GripperCtrl(int(round(self.gripper_mm() * 1000)), 1000, 0x01, 0)

    def set_joint_mode(self, speed_pct: int) -> None:
        self._guard()
        # ctrl_mode 0x01 = CAN control, move_mode 0x01 = MOVE J, 0x00 = position-velocity (not MIT).
        self.piper.MotionCtrl_2(0x01, 0x01, int(np.clip(speed_pct, 1, 100)), 0x00)

    def command(self, q_deg: np.ndarray, grip_mm: float, speed_pct: int, grip_effort: int = 1000) -> None:
        """Send one joint + gripper setpoint. Call at a steady rate (e.g. 100 Hz)."""
        self._guard()
        self.set_joint_mode(speed_pct)  # re-asserted every cycle, as in the AgileX examples
        j = [int(round(v * 1000)) for v in q_deg]
        self.piper.JointCtrl(*j)
        self.piper.GripperCtrl(int(round(np.clip(grip_mm, 0, 70) * 1000)), int(grip_effort), 0x01, 0)

    def emergency_stop(self) -> None:
        """Quick stop (0x150 byte0 = 0x01). The arm stops; it does NOT cut power like ResetPiper()."""
        if self.piper is not None:
            self.piper.EmergencyStop(0x01)

    def disable(self) -> None:
        """Disable motors. WARNING: the arm goes limp and drops if it is not resting on its stops."""
        self._guard()
        t0 = time.time()
        while any(self.enabled()) and time.time() - t0 < 3.0:
            self.piper.DisablePiper()
            time.sleep(0.05)
        self.piper.GripperCtrl(0, 1000, 0x00, 0)

    def disconnect(self) -> None:
        if self.piper is None:
            return
        # piper_sdk's DisconnectPort() never joins its reader thread (its hasattr check uses the wrong
        # name), so it closes the bus while the thread is blocked inside the candle driver and the
        # process then hangs on exit. Stop and join the threads ourselves first.
        p = self.piper
        p._C_PiperInterface_V2__read_can_stop_event.set()
        p._C_PiperInterface_V2__can_monitor_stop_event.set()
        for name in ("_C_PiperInterface_V2__can_deal_th", "_C_PiperInterface_V2__can_monitor_th"):
            th = getattr(p, name, None)
            if th is not None and th.is_alive():
                th.join(timeout=2.0)  # reader uses recv(1 s), so it exits within ~1 s
        p.DisconnectPort()
        type(p)._instances.pop(self.channel, None)
        self.piper = None
