"""Passive CAN probe: open the candleLight adapter at 1 Mbit/s and only *receive* for a few seconds.
Sends nothing, so the PiPER X cannot move. Prints which CAN IDs are seen and how often."""

import sys
import time
from collections import Counter
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from teleop.can_setup import CAN_BITRATE, CAN_CHANNEL, CAN_INTERFACE, list_adapters  # noqa: E402

import can  # noqa: E402

SECONDS = float(sys.argv[1]) if len(sys.argv) > 1 else 3.0

# Feedback IDs from the PiPER CAN protocol (piper_sdk can_id.py).
KNOWN = {
    0x2A1: "arm status", 0x2A2: "end pose XY", 0x2A3: "end pose Z/RX", 0x2A4: "end pose RY/RZ",
    0x2A5: "joint 1/2", 0x2A6: "joint 3/4", 0x2A7: "joint 5/6", 0x2A8: "gripper",
    0x251: "motor1 high-spd", 0x252: "motor2 high-spd", 0x253: "motor3 high-spd",
    0x254: "motor4 high-spd", 0x255: "motor5 high-spd", 0x256: "motor6 high-spd",
    0x261: "motor1 low-spd", 0x262: "motor2 low-spd", 0x263: "motor3 low-spd",
    0x264: "motor4 low-spd", 0x265: "motor5 low-spd", 0x266: "motor6 low-spd",
}


def main():
    print("Adapters:", list_adapters())
    # Normal mode, but we never call send(): the adapter only ACKs. listen_only does not work here
    # because the PiPER is the only other node — with nobody ACKing it retransmits its first frame forever.
    bus = can.Bus(interface=CAN_INTERFACE, channel=CAN_CHANNEL, bitrate=CAN_BITRATE)
    counts, last = Counter(), {}
    t0 = time.time()
    try:
        while time.time() - t0 < SECONDS:
            msg = bus.recv(0.1)
            if msg is None or msg.is_error_frame:
                continue
            counts[msg.arbitration_id] += 1
            last[msg.arbitration_id] = msg.data.hex(" ")
    finally:
        bus.shutdown()

    if not counts:
        print("No frames received. Is the PiPER X powered on and the CAN cable plugged into it?")
        return
    print(f"Received {sum(counts.values())} frames in {SECONDS:.0f}s:")
    for cid in sorted(counts):
        print(f"  0x{cid:03X} {KNOWN.get(cid, ''):16s} {counts[cid] / SECONDS:7.1f} Hz  last=[{last[cid]}]")


if __name__ == "__main__":
    main()
