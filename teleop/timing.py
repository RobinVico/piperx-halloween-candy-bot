"""Steady loop timing on Windows.

The default Windows timer tick is ~15.6 ms, so time.sleep(0.01) really sleeps 0-16 ms and the loop
stutters. timeBeginPeriod(1) raises the resolution to 1 ms for this process.
"""

import ctypes
import sys
import time


class HighResTimer:
    def __enter__(self):
        if sys.platform == "win32":
            ctypes.windll.winmm.timeBeginPeriod(1)
        return self

    def __exit__(self, *exc):
        if sys.platform == "win32":
            ctypes.windll.winmm.timeEndPeriod(1)


class Rate:
    """Fixed-rate loop helper; `sleep()` returns the measured dt of the iteration."""

    def __init__(self, hz: float):
        self.period = 1.0 / hz
        self.next_t = time.perf_counter() + self.period
        self.last = time.perf_counter()

    def sleep(self) -> float:
        remaining = self.next_t - time.perf_counter()
        if remaining > 0.002:
            time.sleep(remaining - 0.001)
        while time.perf_counter() < self.next_t:  # short spin for the last ~1 ms
            pass
        now = time.perf_counter()
        # If we fell far behind (e.g. a stall), resync instead of bursting to catch up.
        self.next_t = max(self.next_t + self.period, now)
        dt, self.last = now - self.last, now
        return dt
