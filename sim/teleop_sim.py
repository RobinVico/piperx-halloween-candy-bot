"""SO-101 leader -> simulated PiPER X in the MuJoCo viewer. Touches no real PiPER hardware.

Uses exactly the same mapping, safety filter and timing as the real-arm script.

Usage (from the project root):
    .venv\\Scripts\\python.exe -m sim.teleop_sim                  # use saved/default mapping
    .venv\\Scripts\\python.exe -m sim.teleop_sim --capture-rest   # first fold the leader, then record its rest pose
"""

import argparse
import sys
import time
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from sim.view import PiperView  # noqa: E402
from teleop.mapping import SO_JOINTS, SafetyFilter, capture_rest, load_config, map_leader, save_config  # noqa: E402
from teleop.so101_reader import LeaderStream, SO101Reader  # noqa: E402
from teleop.timing import HighResTimer, Rate  # noqa: E402


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="COM3")
    ap.add_argument("--hz", type=float, default=100.0)
    ap.add_argument("--max-speed", type=float, default=60.0, help="joint speed cap, deg/s")
    ap.add_argument("--max-acc", type=float, default=300.0, help="joint acceleration cap, deg/s^2")
    ap.add_argument("--capture-rest", action="store_true",
                    help="record the leader's current pose as the PiPER zero (folded) pose")
    args = ap.parse_args()

    cfg = load_config()
    reader = SO101Reader(args.port)
    reader.connect()
    print("SO-101 leader connected (read-only).")

    if args.capture_rest:
        print("Fold the SO-101 into its rest pose (same as PiPER folded). Capturing in 3 s...")
        time.sleep(3)
        capture_rest(reader.read(), cfg)
        save_config(cfg)
        print("Saved so_zero =", np.round(cfg.so_zero, 1))

    stream = LeaderStream(reader).start()
    flt = SafetyFilter(cfg, start_deg=np.zeros(6), start_grip_mm=0.0,
                       max_speed_deg_s=args.max_speed, max_acc_deg_s2=args.max_acc)
    last_print = 0.0
    with HighResTimer(), PiperView() as view:
        print("Viewer open. Move the SO-101; close the window or Ctrl+C to stop.")
        rate = Rate(args.hz)
        dt = 1.0 / args.hz
        try:
            while view.is_running():
                so, age = stream.latest()
                if age > 0.2:
                    flt.hold()
                    target, grip_target = flt.q, flt.grip
                else:
                    target, grip_target = map_leader(so, cfg)
                q, grip = flt.step(target, grip_target, dt)
                view.show(q, grip)

                now = time.perf_counter()
                if now - last_print > 0.5:
                    last_print = now
                    so_s = " ".join(f"{n[:5]}={so[n]:6.1f}" for n in SO_JOINTS)
                    pq_s = " ".join(f"J{i + 1}={v:6.1f}" for i, v in enumerate(q))
                    print(f"SO101[{so_s}] -> PiPER[{pq_s} G={grip:4.1f}mm] dt={dt * 1000:4.1f}ms "
                          f"rej={flt.rejected} err={stream.errors}", flush=True)
                dt = rate.sleep()
        except KeyboardInterrupt:
            pass
        finally:
            stream.stop()
            reader.disconnect()
            print("Stopped.")


if __name__ == "__main__":
    main()
