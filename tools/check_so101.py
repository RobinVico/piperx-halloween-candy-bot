"""Read-only SO-101 check: identify which calibration file matches the motors on the port,
then print joint positions. Never enables torque and never writes to the motors."""

import json
import sys
import time
from pathlib import Path

from lerobot.motors import Motor, MotorCalibration, MotorNormMode
from lerobot.motors.feetech import FeetechMotorsBus

PORT = sys.argv[1] if len(sys.argv) > 1 else "COM3"
CALIB_DIR = Path.home() / ".cache/huggingface/lerobot/calibration"
CANDIDATES = {
    "leader:my_leader": CALIB_DIR / "teleoperators/so_leader/my_leader.json",
    "follower:my_so101": CALIB_DIR / "robots/so_follower/my_so101.json",
}
NAMES = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper"]


def load(path):
    return {k: MotorCalibration(**v) for k, v in json.loads(path.read_text()).items()}


def main():
    bus = FeetechMotorsBus(
        port=PORT,
        motors={n: Motor(i + 1, "sts3215", MotorNormMode.DEGREES) for i, n in enumerate(NAMES[:-1])}
        | {"gripper": Motor(6, "sts3215", MotorNormMode.RANGE_0_100)},
    )
    bus.connect()  # pings every motor ID; raises if any is missing
    try:
        in_motor = bus.read_calibration()
        print("Calibration stored in motors:")
        for n in NAMES:
            c = in_motor[n]
            print(f"  {n:14s} id={c.id} offset={c.homing_offset:6d} min={c.range_min:5d} max={c.range_max:5d}")

        match = None
        for label, path in CANDIDATES.items():
            if not path.exists():
                continue
            ok = all(
                load(path)[n].homing_offset == in_motor[n].homing_offset
                and load(path)[n].range_min == in_motor[n].range_min
                and load(path)[n].range_max == in_motor[n].range_max
                for n in NAMES
            )
            print(f"Matches {label}: {ok}")
            if ok:
                match = path
        if match is None:
            print("No calibration file matches the motors; positions below use the motors' own values.")
            calib = in_motor
        else:
            calib = load(match)

        bus.calibration = calib
        torque = bus.sync_read("Torque_Enable", normalize=False)
        print("Torque_Enable:", torque)

        print("\nLive positions (move the arm by hand), 5 s:")
        t0 = time.time()
        while time.time() - t0 < 5:
            pos = bus.sync_read("Present_Position", num_retry=2)
            print("  " + "  ".join(f"{n[:8]}={pos[n]:7.1f}" for n in NAMES), flush=True)
            time.sleep(0.25)
    finally:
        bus.disconnect(disable_torque=False)


if __name__ == "__main__":
    main()
