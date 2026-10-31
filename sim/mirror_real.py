"""Read-only: show the REAL PiPER X's joint feedback on the simulated model.

Use it before the first live run to confirm the real arm's joint directions/zero match the sim:
with the arm powered but NOT enabled, gently move one joint by hand and check the sim moves the
same joint the same way. Never enables or commands the arm.

    .venv\\Scripts\\python.exe -m sim.mirror_real
"""

import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from sim.view import PiperView  # noqa: E402
from teleop.piper_driver import PiperX  # noqa: E402


def main():
    arm = PiperX(allow_motion=False)
    arm.connect()
    print("Firmware:", arm.firmware(), "| Enabled:", arm.enabled())
    last = 0.0
    try:
        with PiperView() as view:
            while view.is_running():
                q, g = arm.joints_deg(), arm.gripper_mm()
                view.show(q, g)
                if time.time() - last > 0.5:
                    last = time.time()
                    print("REAL " + " ".join(f"J{i + 1}={v:7.2f}" for i, v in enumerate(q)) + f" G={g:5.1f}",
                          flush=True)
                time.sleep(0.02)
    finally:
        arm.disconnect()


if __name__ == "__main__":
    main()
