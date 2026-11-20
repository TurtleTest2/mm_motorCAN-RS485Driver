"""
MX_CAN driver - UUMOTOR MX dual brushless controller over CAN
(protocol MX_CAN-SV3.03-1, extended frames, 500 kbit/s by default).

  Host -> driver  ID 0x1801E600, 8 bytes, every 100 ms (1.5 s watchdog):
    D0  control word  bit7 M1 speed(1)/torque(0)   bit6 M2 speed/torque
                      bit3 coast   bit2 reset      bit1 M1 enable   bit0 M2 enable
    D1-2  M2 setpoint int16 (r/min, or A in torque mode)   D3  M2 accel uint8 (x10 r/min/s)
    D4-5  M1 setpoint int16                                D6  M1 accel uint8
    D7    reserved (0)

  Driver -> host  ID 0x1801E001, 8 bytes:
    D0 fault   D1 temp (deg C)   D2-3 M2 rpm int16   D4-5 M1 rpm int16   D6-7 voltage uint16 (0.1 V)

Note: the CAN status frame has NO motor current - only speed, voltage, temperature, fault.

Uses python-can, so any adapter it supports works: Waveshare/Seeed USB-CAN-A
("seeedstudio"), CANable ("slcan"), PEAK ("pcan"), Kvaser, candleLight ("gs_usb"), ...

    pip install python-can pyserial
"""

from __future__ import annotations

import struct
import threading
import time
from dataclasses import dataclass, field
from typing import Optional

from mx_serve import FAULT_CODES

CMD_ID = 0x1801E600         # host -> driver
STATUS_ID = 0x1801E001      # driver -> host
BITRATE = 500_000
SEND_INTERVAL_S = 0.100     # protocol test interval
WATCHDOG_S = 1.5

# Adapter presets: label -> (python-can interface name, default channel, extra kwargs)
ADAPTERS = {
    "Simulator": ("sim", "", {}),
    "Waveshare / Seeed USB-CAN-A (COM port)": ("seeedstudio", "COM4", {"frame_type": "EXT"}),
    "CANable / slcan (COM port)": ("slcan", "COM4", {}),
    "PEAK PCAN-USB": ("pcan", "PCAN_USBBUS1", {}),
    "Linux SocketCAN (can0, e.g. Xilume Octant)": ("socketcan", "can0", {}),
    "Kvaser": ("kvaser", "0", {}),
    "candleLight / gs_usb": ("gs_usb", "0", {}),
    "IXXAT": ("ixxat", "0", {}),
    "Vector": ("vector", "0", {}),
    "8devices USB2CAN": ("usb2can", "", {}),
    "Canalyst-II": ("canalystii", "0", {}),
}


def _i16(v: int) -> int:
    return max(-30000, min(30000, int(v)))


@dataclass
class MotorCommand:
    enabled: bool = False
    speed_mode: bool = True     # False = torque mode
    setpoint: int = 0           # r/min (speed) or A (torque)
    accel: int = 50             # protocol units: 10 r/min per s (speed) or A per s (torque)


@dataclass
class CanStatus:
    fault_code: int
    temperature_c: int
    motor2_rpm: int
    motor1_rpm: int
    voltage_v: float
    raw: bytes = b""
    timestamp: float = field(default_factory=time.monotonic)

    @property
    def fault(self) -> str:
        return FAULT_CODES.get(self.fault_code, f"Unknown fault ({self.fault_code})")

    @property
    def ok(self) -> bool:
        return self.fault_code == 0

    @classmethod
    def from_bytes(cls, d: bytes) -> "CanStatus":
        d = bytes(d).ljust(8, b"\0")
        m2, m1, v = struct.unpack(">hhH", d[2:8])
        return cls(fault_code=d[0], temperature_c=d[1], motor2_rpm=m2, motor1_rpm=m1,
                   voltage_v=v / 10.0, raw=d)


def build_command(m1: MotorCommand, m2: MotorCommand, coast: bool = False, reset: bool = False) -> bytes:
    ctrl = ((int(m1.speed_mode) << 7) | (int(m2.speed_mode) << 6) | (int(coast) << 3) |
            (int(reset) << 2) | (int(m1.enabled) << 1) | int(m2.enabled))
    acc = lambda a: max(0, min(255, int(a)))
    return struct.pack(">BhBhBB", ctrl, _i16(m2.setpoint), acc(m2.accel), _i16(m1.setpoint), acc(m1.accel), 0)


# --------------------------------------------------------------------------
# Simulated controller bus (no hardware needed)
# --------------------------------------------------------------------------
class FakeBus:
    """Behaves like a python-can Bus with one MX controller on it."""

    def __init__(self, cmd_id: int = CMD_ID, status_id: int = STATUS_ID):
        import can
        self._can = can
        self.cmd_id, self.status_id = cmd_id, status_id
        self._q: list = []
        self._lock = threading.Lock()
        self.rpm = {1: 0.0, 2: 0.0}
        self.cmd = bytes(8)
        self._t = time.monotonic()
        self._last_rx = 0.0

    def _physics(self):
        now = time.monotonic()
        dt, self._t = now - self._t, now
        ctrl = self.cmd[0]
        _, sp2, a2, sp1, a1, _ = struct.unpack(">BhBhBB", self.cmd)
        alive = now - self._last_rx < WATCHDOG_S
        for m, en, sp, a in ((1, ctrl >> 1 & 1, sp1, a1), (2, ctrl & 1, sp2, a2)):
            target = sp if (en and alive) else 0
            step = max(1, a) * 10 * dt
            cur = self.rpm[m]
            self.rpm[m] = min(target, cur + step) if target > cur else max(target, cur - step)

    def send(self, msg, timeout=None):
        if msg.arbitration_id != self.cmd_id or not msg.is_extended_id:
            return
        import random
        with self._lock:
            self._physics()
            self.cmd = bytes(msg.data).ljust(8, b"\0")
            self._last_rx = time.monotonic()
            load = (abs(self.rpm[1]) + abs(self.rpm[2])) / 400
            data = struct.pack(">BBhhH", 0, int(30 + load), int(self.rpm[2]), int(self.rpm[1]),
                               int((48.0 - load * 0.1 + random.uniform(-0.05, 0.05)) * 10))
            self._q.append(self._can.Message(arbitration_id=self.status_id, is_extended_id=True, data=data))

    def recv(self, timeout=None):
        end = time.monotonic() + (timeout or 0)
        while True:
            with self._lock:
                if self._q:
                    return self._q.pop(0)
            if time.monotonic() >= end:
                return None
            time.sleep(0.005)

    def shutdown(self):
        pass


def open_bus(interface: str, channel: str, bitrate: int = BITRATE, **extra):
    """Open a python-can bus. interface 'sim' gives the built-in simulator."""
    if interface == "sim":
        return FakeBus()
    import can
    kw = dict(extra)
    if channel != "":
        kw["channel"] = int(channel) if channel.isdigit() and interface not in ("seeedstudio", "slcan") else channel
    return can.Bus(interface=interface, bitrate=bitrate, **kw)


# --------------------------------------------------------------------------
# Driver
# --------------------------------------------------------------------------
class MXCanDriver:
    """
    Sends the command frame every 100 ms from a background thread (keeps the
    1.5 s watchdog fed) and collects status frames. Motors are 1 and 2.
    """

    def __init__(self, bus, cmd_id: int = CMD_ID, status_id: int = STATUS_ID,
                 interval: float = SEND_INTERVAL_S, max_consecutive_errors: int = 5):
        self.bus = bus
        self.cmd_id, self.status_id = cmd_id, status_id
        self.interval = interval
        self.max_consecutive_errors = max_consecutive_errors
        self.motors = {1: MotorCommand(), 2: MotorCommand()}
        self.coast = False
        self._reset_until = 0.0
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._thread: Optional[threading.Thread] = None

        self.last_status: Optional[CanStatus] = None
        self.last_error: Optional[Exception] = None
        self.consecutive_errors = 0
        self.tx_count = self.rx_count = self.status_count = 0
        self.last_tx: bytes = b""
        self.other_ids: dict[int, tuple[int, bytes, bool]] = {}   # id -> (count, last data, extended)

    # ------------------------------------------------------------ commands
    def _m(self, motor: int) -> MotorCommand:
        if motor not in (1, 2):
            raise ValueError("motor must be 1 or 2")
        return self.motors[motor]

    def enable(self, motor: int, on: bool = True):
        with self._lock:
            self._m(motor).enabled = on

    def set_speed(self, motor: int, rpm: int):
        with self._lock:
            self._m(motor).setpoint = _i16(rpm)

    def set_accel_rpm_s(self, motor: int, rpm_per_s: int):
        """Acceleration in r/min per second; the protocol carries it in steps of 10 (10-2550)."""
        with self._lock:
            self._m(motor).accel = max(1, min(255, round(rpm_per_s / 10)))

    def set_coast(self, on: bool):
        with self._lock:
            self.coast = on

    def pulse_reset(self, seconds: float = 0.5):
        """Set the reset bit for a few frames (optional factory feature: clears faults)."""
        self._reset_until = time.monotonic() + seconds

    def stop_all(self):
        with self._lock:
            for m in self.motors.values():
                m.enabled = False
                m.setpoint = 0

    def command_bytes(self) -> bytes:
        with self._lock:
            return build_command(self.motors[1], self.motors[2], self.coast,
                                 time.monotonic() < self._reset_until)

    # ------------------------------------------------------------ thread
    def start(self):
        if self._thread and self._thread.is_alive():
            return
        self._stop.clear()
        self._thread = threading.Thread(target=self._run, name="mxcan", daemon=True)
        self._thread.start()

    def close(self):
        """Send 'both disabled' a few times, stop the thread and release the adapter."""
        self.stop_all()
        self._stop.set()
        if self._thread:
            self._thread.join(timeout=2)
            self._thread = None
        for _ in range(3):
            try:
                self._send(self.command_bytes())
            except Exception:
                break
            time.sleep(0.02)
        try:
            self.bus.shutdown()
        except Exception:
            pass

    def on_error(self, exc: Exception):
        """Override / assign to get send/receive errors (runs on the driver thread)."""

    def on_status(self, status: CanStatus):
        """Override / assign to get every status frame (runs on the driver thread)."""

    def _send(self, data: bytes):
        import can
        self.bus.send(can.Message(arbitration_id=self.cmd_id, is_extended_id=True, data=data), timeout=0.05)
        self.last_tx = data
        self.tx_count += 1

    def _handle(self, msg):
        self.rx_count += 1
        if getattr(msg, "is_error_frame", False):
            return
        if msg.arbitration_id == self.status_id and msg.is_extended_id:
            st = CanStatus.from_bytes(msg.data)
            self.last_status = st
            self.status_count += 1
            self.on_status(st)
        else:
            n, _, _ = self.other_ids.get(msg.arbitration_id, (0, b"", False))
            self.other_ids[msg.arbitration_id] = (n + 1, bytes(msg.data), msg.is_extended_id)

    def _run(self):
        next_tx = time.monotonic()
        while not self._stop.is_set():
            now = time.monotonic()
            if now >= next_tx:
                next_tx = now + self.interval
                try:
                    self._send(self.command_bytes())
                    self.consecutive_errors = 0
                except Exception as exc:
                    self.last_error = exc
                    self.consecutive_errors += 1
                    self.on_error(exc)
                    if self.consecutive_errors >= self.max_consecutive_errors:
                        self.stop_all()
            try:
                msg = self.bus.recv(timeout=max(0.0, min(0.05, next_tx - time.monotonic())))
            except Exception as exc:
                self.last_error = exc
                self.on_error(exc)
                self._stop.wait(0.05)
                continue
            if msg is not None:
                self._handle(msg)


if __name__ == "__main__":
    # Self-check against the protocol's host test codes (table 2.4).
    def cmd(e1, e2, speed, sp, reset=False, coast=False):
        m = lambda en: MotorCommand(enabled=en, speed_mode=speed, setpoint=sp, accel=100)
        return build_command(m(e1), m(e2), coast, reset).hex(" ").upper()
    assert cmd(1, 1, True, 50) == "C3 00 32 64 00 32 64 00"
    assert cmd(1, 0, True, 50) == "C2 00 32 64 00 32 64 00"
    assert cmd(0, 1, True, 50) == "C1 00 32 64 00 32 64 00"
    assert cmd(1, 1, False, 20) == "03 00 14 64 00 14 64 00"
    assert cmd(0, 0, False, 0, reset=True) == "04 00 00 64 00 00 64 00"
    assert cmd(0, 0, False, 0, coast=True) == "08 00 00 64 00 00 64 00"
    st = CanStatus.from_bytes(bytes.fromhex("00 1E FF 38 01 F4 01 E0"))
    assert (st.temperature_c, st.motor2_rpm, st.motor1_rpm, st.voltage_v) == (30, -200, 500, 48.0)
    print("protocol self-check OK")
