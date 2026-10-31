"""Read-only PiPER X check: firmware, status, enable state and live joint feedback.
Sends only query frames (firmware / limit tables); never enables, never commands motion."""

import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from teleop.piper_driver import PiperX  # noqa: E402

SECONDS = float(sys.argv[1]) if len(sys.argv) > 1 else 5.0


def main():
    arm = PiperX(allow_motion=False)
    arm.connect()
    try:
        time.sleep(0.5)  # let the firmware query answer arrive
        print("CAN channel :", arm.channel)
        print("Firmware    :", arm.firmware())
        print("Feedback Hz :", arm.feedback_hz())
        print("Enabled     :", arm.enabled())
        print("Status      :", arm.status_line())
        print(f"\nJoint feedback for {SECONDS:.0f}s (deg; gripper mm):")
        t0 = time.time()
        while time.time() - t0 < SECONDS:
            q = arm.joints_deg()
            print("  " + " ".join(f"J{i + 1}={v:7.2f}" for i, v in enumerate(q)) + f"  G={arm.gripper_mm():5.1f}",
                  flush=True)
            time.sleep(0.5)
    finally:
        arm.disconnect()


if __name__ == "__main__":
    main()
