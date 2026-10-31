"""Read-only SO-101 leader access: never enables torque and never writes to the motors."""

import json
import threading
import time
from pathlib import Path

from lerobot.motors import Motor, MotorCalibration, MotorNormMode
from lerobot.motors.feetech import FeetechMotorsBus

from .mapping import SO_JOINTS

DEFAULT_CALIB = Path.home() / ".cache/huggingface/lerobot/calibration/teleoperators/so_leader/my_leader.json"


class SO101Reader:
    def __init__(self, port: str = "COM3", calib_path: Path = DEFAULT_CALIB):
        calib = {k: MotorCalibration(**v) for k, v in json.loads(Path(calib_path).read_text()).items()}
        motors = {n: Motor(i + 1, "sts3215", MotorNormMode.DEGREES) for i, n in enumerate(SO_JOINTS[:-1])}
        motors["gripper"] = Motor(6, "sts3215", MotorNormMode.RANGE_0_100)
        self.bus = FeetechMotorsBus(port=port, motors=motors, calibration=calib)

    def connect(self, attempts: int = 4) -> None:
        # The first handshake after opening the port occasionally gets no status packet
        # (e.g. right after another process was killed mid-transfer), so retry a few times.
        for i in range(attempts):
            try:
                self.bus.connect()
                break
            except ConnectionError:
                if self.bus.port_handler.is_open:
                    self.bus.port_handler.closePort()
                if i == attempts - 1:
                    raise
                time.sleep(0.5)
        if not self.bus.is_calibrated:
            self.bus.disconnect(disable_torque=False)
            raise RuntimeError("Calibration in the motors does not match my_leader.json; run lerobot-calibrate.")
        torque = self.bus.sync_read("Torque_Enable", normalize=False)
        if any(torque.values()):
            print(f"WARNING: leader torque is enabled on {torque}; it will feel stiff. Not changing it.")

    def read(self) -> dict:
        return self.bus.sync_read("Present_Position", num_retry=2)

    def disconnect(self) -> None:
        self.bus.disconnect(disable_torque=False)


class LeaderStream:
    """Reads the leader in a background thread as fast as it can, so the control loop always gets
    the freshest sample without waiting on the serial bus."""

    def __init__(self, reader: SO101Reader, hz: float = 200.0):
        self.reader = reader
        self.period = 1.0 / hz
        self._lock = threading.Lock()
        self._latest: dict | None = None
        self._stamp = 0.0
        self.errors = 0
        self._stop = threading.Event()
        self._th = threading.Thread(target=self._run, daemon=True)

    def start(self) -> "LeaderStream":
        self._latest = self.reader.read()
        self._stamp = time.perf_counter()
        self._th.start()
        return self

    def _run(self):
        while not self._stop.is_set():
            t0 = time.perf_counter()
            try:
                sample = self.reader.read()
                with self._lock:
                    self._latest, self._stamp = sample, time.perf_counter()
            except Exception:  # noqa: BLE001 - keep streaming; staleness is checked by the consumer
                self.errors += 1
            time.sleep(max(0.0, self.period - (time.perf_counter() - t0)))

    def latest(self) -> tuple[dict, float]:
        """(sample, age in seconds)."""
        with self._lock:
            return dict(self._latest), time.perf_counter() - self._stamp

    def stop(self):
        self._stop.set()
        self._th.join(timeout=1.0)
