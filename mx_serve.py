"""
MX_SERVE dual brushless driver (MXES-2kW) - RS485 / Modbus RTU host driver.

Based on "MX_SERVE-RS485 Protocol V3.03":
  * 9600 8N1 by default (configurable in the vendor tool)
  * 0x03 = read status registers 0x0000-0x0006
  * 0x10 = write control registers 0x00-0x08 (always all 9 at once)
  * >= 20 ms between frames recommended
  * Motor stops if no valid frame is received for 1.5 s -> a heartbeat
    thread re-sends the control frame continuously.

Requires: pip install pyserial

Example (Windows):
    from mx_serve import MXServeDriver, Mode

    with MXServeDriver("COM3", address=1) as drv:
        drv.set_mode(1, Mode.SPEED)
        drv.set_mode(2, Mode.SPEED)
        drv.set_accel(1, 100); drv.set_accel(2, 100)
        drv.enable(1); drv.enable(2)
        drv.set_speed(1, 50); drv.set_speed(2, 50)
        for _ in range(20):
            print(drv.get_status())
            time.sleep(0.25)
    # leaving the "with" block disables both motors and closes the port
"""

from __future__ import annotations

import enum
import struct
import threading
import time
from dataclasses import dataclass, field
from typing import Optional

# --------------------------------------------------------------------------
# Constants
# --------------------------------------------------------------------------

FUNC_READ = 0x03
FUNC_WRITE = 0x10

STATUS_START = 0x0000
STATUS_COUNT = 7

CONTROL_START = 0x0000
CONTROL_COUNT = 9

MIN_FRAME_GAP_S = 0.020       # protocol: idle >= 20 ms between frames
WATCHDOG_S = 1.5              # protocol: motor stops after 1.5 s silence

# Per the MXES-2kW V4.01 hardware manual (Table 1-1), which matches the TV4.01
# firmware. The older protocol doc V3.03 listed 17/18 as "phase short".
FAULT_CODES = {
    0: "Normal",
    1: "Controller fault",
    2: "Controller fault",
    3: "Controller fault",
    4: "Controller fault",
    5: "Over-current",
    6: "Over-voltage",
    7: "Under-voltage",
    8: "Over-temperature",
    11: "Motor 2 over-speed",
    12: "Motor 1 over-speed",
    13: "Motor 2 overload",
    14: "Motor 1 overload",
    15: "Motor 2 phase loss",
    16: "Motor 1 phase loss",
    17: "Motor 2 brake fault",
    18: "Motor 1 brake fault",
    19: "Motor 2 encoder fault",
    20: "Motor 1 encoder fault",
    21: "Motor 2 over-temperature",
    22: "Motor 1 over-temperature",
    23: "Motor 2 Hall fault",
    24: "Motor 1 Hall fault",
    25: "Motor 2 stall",
    26: "Motor 1 stall",
    27: "UART communication fault",
    28: "RS485 communication fault",
    29: "CAN communication fault",
    30: "Joystick fault",
    31: "Throttle fault",
}


class Mode(enum.IntEnum):
    TORQUE = 0b00
    SPEED = 0b01


class MXServeError(Exception):
    """Base error for this driver."""


class MXServeTimeout(MXServeError):
    """No (complete) reply from the driver."""


class MXServeCRCError(MXServeError):
    """Reply received but CRC did not match."""


class MXServeExceptionResponse(MXServeError):
    """Driver answered with an error frame (0x83 / 0x90)."""


# --------------------------------------------------------------------------
# Modbus helpers
# --------------------------------------------------------------------------

def crc16_modbus(data: bytes) -> int:
    """Modbus RTU CRC16 (poly 0xA001, init 0xFFFF). Sent low byte first."""
    crc = 0xFFFF
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = (crc >> 1) ^ 0xA001 if crc & 1 else crc >> 1
    return crc


def append_crc(payload: bytes) -> bytes:
    return payload + struct.pack("<H", crc16_modbus(payload))


def check_crc(frame: bytes) -> bool:
    if len(frame) < 3:
        return False
    return struct.unpack("<H", frame[-2:])[0] == crc16_modbus(frame[:-2])


def build_read_frame(address: int, start: int, count: int) -> bytes:
    return append_crc(struct.pack(">BBHH", address, FUNC_READ, start, count))


def build_write_frame(address: int, start: int, registers: list[int]) -> bytes:
    regs = [r & 0xFFFF for r in registers]
    header = struct.pack(">BBHHB", address, FUNC_WRITE, start, len(regs), len(regs) * 2)
    body = struct.pack(f">{len(regs)}H", *regs)
    return append_crc(header + body)


# --------------------------------------------------------------------------
# Data classes
# --------------------------------------------------------------------------

@dataclass
class MotorCommand:
    """Commanded state of one motor channel (what goes into registers 0x00-0x08)."""
    enabled: bool = False
    mode: Mode = Mode.SPEED
    flag_bit5: bool = False     # motor 2: coast   | motor 1: reset enable (optional feature)
    setpoint: int = 0           # r/min in speed mode; raw torque value in torque mode
    accel: int = 100            # r/min per s (speed mode) or A per s (torque mode)
    kp: int = 1000              # factory default
    ki: int = 2000              # factory default

    def control_byte(self) -> int:
        return (int(self.mode) << 6) | (int(self.flag_bit5) << 5) | int(self.enabled)


@dataclass
class Status:
    fault_code: int
    temperature_c: int
    voltage_v: float
    motor1_current_a: float
    motor2_current_a: float
    motor1_rpm: int
    motor2_rpm: int
    timestamp: float = field(default_factory=time.monotonic)

    @property
    def fault(self) -> str:
        return FAULT_CODES.get(self.fault_code, f"Unknown fault ({self.fault_code})")

    @property
    def ok(self) -> bool:
        return self.fault_code == 0

    @classmethod
    def from_registers(cls, regs: list[int]) -> "Status":
        def s16(v: int) -> int:
            return v - 0x10000 if v & 0x8000 else v
        return cls(
            fault_code=regs[0],
            temperature_c=s16(regs[1]),
            voltage_v=s16(regs[2]) / 10.0,
            motor2_current_a=s16(regs[3]) / 10.0,
            motor1_current_a=s16(regs[4]) / 10.0,
            motor2_rpm=s16(regs[5]),
            motor1_rpm=s16(regs[6]),
        )


# --------------------------------------------------------------------------
# Driver
# --------------------------------------------------------------------------

class MXServeDriver:
    """
    Host-side driver for one MX_SERVE dual brushless controller on an RS485 bus.

    Motors are numbered 1 and 2, matching the controller's M1/M2 outputs.

    The heartbeat thread (start()/stop(), started automatically by the context
    manager) repeatedly writes the current command and reads back status, so
    the controller's 1.5 s communication watchdog never trips while your
    program is alive. If your program crashes or the cable is unplugged, the
    controller stops the motors on its own.
    """

    def __init__(
        self,
        port: str,
        address: int = 1,
        baudrate: int = 9600,
        timeout: float = 0.1,
        heartbeat_interval: float = 0.1,
        poll_status: bool = True,
        max_consecutive_errors: int = 5,
        serial_instance=None,
    ):
        """
        port:               "COM3" on Windows, "/dev/ttyUSB0" on Linux
        address:            driver address 0x01-0x7F (set in vendor tool)
        baudrate:           must match the vendor tool setting (default 9600)
        timeout:            per-reply timeout in seconds
        heartbeat_interval: seconds between heartbeat cycles (keep << 1.5 s)
        poll_status:        heartbeat also reads status each cycle
        serial_instance:    optional pre-opened serial-like object (for tests)
        """
        if not 0x01 <= address <= 0x7F:
            raise ValueError("address must be 0x01-0x7F")
        if heartbeat_interval >= WATCHDOG_S / 2:
            raise ValueError("heartbeat_interval must be well below the 1.5 s watchdog")

        self.address = address
        self.timeout = timeout
        self.heartbeat_interval = heartbeat_interval
        self.poll_status = poll_status
        self.max_consecutive_errors = max_consecutive_errors

        if serial_instance is not None:
            self._ser = serial_instance
        else:
            import serial  # pyserial
            self._ser = serial.Serial(
                port=port,
                baudrate=baudrate,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=timeout,
            )

        self.motors = {1: MotorCommand(), 2: MotorCommand()}

        self._bus_lock = threading.Lock()     # one transaction on the wire at a time
        self._cmd_lock = threading.Lock()     # protects self.motors
        self._last_tx = 0.0
        self._hb_thread: Optional[threading.Thread] = None
        self._hb_stop = threading.Event()

        self.last_status: Optional[Status] = None
        self.last_error: Optional[Exception] = None
        self.consecutive_errors = 0

    # ---------------------------------------------------------------- context
    def __enter__(self) -> "MXServeDriver":
        self.start()
        return self

    def __exit__(self, *exc) -> None:
        self.close()

    def close(self) -> None:
        """Disable both motors, stop the heartbeat and close the port."""
        try:
            with self._cmd_lock:
                for m in self.motors.values():
                    m.enabled = False
                    m.setpoint = 0
            try:
                self.write_control()
            except MXServeError:
                pass  # controller watchdog will stop motors anyway
        finally:
            self.stop()
            self._ser.close()

    # ------------------------------------------------------------- low level
    def _wait_gap(self) -> None:
        dt = time.monotonic() - self._last_tx
        if dt < MIN_FRAME_GAP_S:
            time.sleep(MIN_FRAME_GAP_S - dt)

    def _read_exact(self, n: int) -> bytes:
        buf = b""
        deadline = time.monotonic() + self.timeout
        while len(buf) < n and time.monotonic() < deadline:
            chunk = self._ser.read(n - len(buf))
            if chunk:
                buf += chunk
        return buf

    def _transact(self, request: bytes, func: int, reply_len: int) -> bytes:
        """Send a request and return the validated reply frame."""
        with self._bus_lock:
            self._wait_gap()
            self._ser.reset_input_buffer()
            self._ser.write(request)
            self._ser.flush()
            try:
                head = self._read_exact(2)
                if len(head) < 2:
                    raise MXServeTimeout(f"no reply from address {self.address}")
                if head[0] != self.address:
                    raise MXServeError(f"reply from wrong address 0x{head[0]:02X}")

                if head[1] == (func | 0x80):
                    # error frame per spec: addr, func|0x80, 0x04, 0x00, CRC_L, CRC_H
                    rest = self._read_exact(4)
                    frame = head + rest
                    if len(frame) < 6 or not check_crc(frame):
                        raise MXServeCRCError(f"bad error frame: {frame.hex(' ')}")
                    raise MXServeExceptionResponse(
                        f"driver rejected function 0x{func:02X}, error type 0x{frame[2]:02X}"
                    )
                if head[1] != func:
                    raise MXServeError(f"unexpected function 0x{head[1]:02X}")

                rest = self._read_exact(reply_len - 2)
                frame = head + rest
                if len(frame) < reply_len:
                    raise MXServeTimeout(f"short reply: {frame.hex(' ')}")
                if not check_crc(frame):
                    raise MXServeCRCError(f"CRC mismatch: {frame.hex(' ')}")
                return frame
            finally:
                self._last_tx = time.monotonic()

    # -------------------------------------------------------------- raw ops
    def read_registers(self, start: int, count: int) -> list[int]:
        req = build_read_frame(self.address, start, count)
        frame = self._transact(req, FUNC_READ, 5 + 2 * count)
        if frame[2] != 2 * count:
            raise MXServeError(f"byte count {frame[2]} != {2 * count}")
        return list(struct.unpack(f">{count}H", frame[3:3 + 2 * count]))

    def build_control_registers(self) -> list[int]:
        with self._cmd_lock:
            m1, m2 = self.motors[1], self.motors[2]
            return [
                (m2.control_byte() << 8) | m1.control_byte(),  # 0x00 mode & enable
                m2.setpoint, m1.setpoint,                      # 0x01, 0x02
                m2.accel, m1.accel,                            # 0x03, 0x04
                m2.kp, m1.kp,                                  # 0x05, 0x06
                m2.ki, m1.ki,                                  # 0x07, 0x08
            ]

    def build_control_frame(self) -> bytes:
        return build_write_frame(self.address, CONTROL_START, self.build_control_registers())

    def write_control(self) -> None:
        """Send the full 9-register control block once."""
        self._transact(self.build_control_frame(), FUNC_WRITE, 8)

    def get_status(self) -> Status:
        """Read fault, temperature, voltage, currents and speeds in one request."""
        st = Status.from_registers(self.read_registers(STATUS_START, STATUS_COUNT))
        self.last_status = st
        return st

    # ------------------------------------------------------------ commands
    # These only update the commanded state. The heartbeat thread sends it;
    # if the heartbeat is not running, call write_control() yourself.

    def _motor(self, motor: int) -> MotorCommand:
        if motor not in (1, 2):
            raise ValueError("motor must be 1 or 2")
        return self.motors[motor]

    def set_mode(self, motor: int, mode: Mode) -> None:
        with self._cmd_lock:
            m = self._motor(motor)
            m.mode = Mode(mode)
            m.setpoint = 0  # avoid jumping when switching between speed/torque

    def enable(self, motor: int, on: bool = True) -> None:
        with self._cmd_lock:
            self._motor(motor).enabled = on

    def disable(self, motor: int) -> None:
        self.enable(motor, False)

    def set_speed(self, motor: int, rpm: int) -> None:
        """Speed setpoint in r/min (sign = direction). Motor must be in SPEED mode."""
        with self._cmd_lock:
            m = self._motor(motor)
            if m.mode != Mode.SPEED:
                raise MXServeError(f"motor {motor} is not in speed mode")
            m.setpoint = _clamp_i16(rpm)

    def set_torque(self, motor: int, value: int) -> None:
        """Torque setpoint (raw register value; units not stated in protocol doc)."""
        with self._cmd_lock:
            m = self._motor(motor)
            if m.mode != Mode.TORQUE:
                raise MXServeError(f"motor {motor} is not in torque mode")
            m.setpoint = _clamp_i16(value)

    def set_accel(self, motor: int, accel: int) -> None:
        """r/min per second (speed mode) or A per second (torque mode)."""
        with self._cmd_lock:
            self._motor(motor).accel = _clamp_u16(accel)

    def set_gains(self, motor: int, kp: Optional[int] = None, ki: Optional[int] = None) -> None:
        with self._cmd_lock:
            m = self._motor(motor)
            if kp is not None:
                m.kp = _clamp_u16(kp)
            if ki is not None:
                m.ki = _clamp_u16(ki)

    def set_coast(self, on: bool = True) -> None:
        """Motor 2 coast bit (bit5 of the high control byte)."""
        with self._cmd_lock:
            self.motors[2].flag_bit5 = on

    def set_reset(self, on: bool = True) -> None:
        """Motor 1 'reset enable' bit (optional factory feature)."""
        with self._cmd_lock:
            self.motors[1].flag_bit5 = on

    def stop_all(self) -> None:
        """Zero both setpoints and disable both motors."""
        with self._cmd_lock:
            for m in self.motors.values():
                m.setpoint = 0
                m.enabled = False

    # ------------------------------------------------------------ heartbeat
    def start(self) -> None:
        if self._hb_thread and self._hb_thread.is_alive():
            return
        self._hb_stop.clear()
        self._hb_thread = threading.Thread(target=self._heartbeat, name="mxserve-hb", daemon=True)
        self._hb_thread.start()

    def stop(self) -> None:
        self._hb_stop.set()
        if self._hb_thread:
            self._hb_thread.join(timeout=2)
            self._hb_thread = None

    @property
    def heartbeat_running(self) -> bool:
        return bool(self._hb_thread and self._hb_thread.is_alive())

    def on_error(self, exc: Exception) -> None:
        """Override or monkey-patch to log heartbeat errors."""

    def on_status(self, status: Status) -> None:
        """Override or monkey-patch to receive every polled status."""

    def _heartbeat(self) -> None:
        while not self._hb_stop.is_set():
            t0 = time.monotonic()
            try:
                self.write_control()
                if self.poll_status:
                    self.on_status(self.get_status())
                self.consecutive_errors = 0
            except MXServeError as exc:
                self.last_error = exc
                self.consecutive_errors += 1
                self.on_error(exc)
                if self.consecutive_errors >= self.max_consecutive_errors:
                    # Stop commanding motion; controller will also time out.
                    with self._cmd_lock:
                        for m in self.motors.values():
                            m.enabled = False
                            m.setpoint = 0
            self._hb_stop.wait(max(0.0, self.heartbeat_interval - (time.monotonic() - t0)))


def _clamp_i16(v: int) -> int:
    return max(-32768, min(32767, int(v)))


def _clamp_u16(v: int) -> int:
    return max(0, min(0xFFFF, int(v)))


# --------------------------------------------------------------------------
# Quick manual test:  python mx_serve.py COM3
# --------------------------------------------------------------------------
if __name__ == "__main__":
    import sys

    port = sys.argv[1] if len(sys.argv) > 1 else "COM3"
    drv = MXServeDriver(port, address=1)
    drv.on_error = lambda e: print("comm error:", e)
    try:
        drv.start()
        print("Status:", drv.get_status())
        for n in (1, 2):
            drv.set_mode(n, Mode.SPEED)
            drv.set_accel(n, 100)
            drv.enable(n)
            drv.set_speed(n, 50)
        t_end = time.monotonic() + 5
        while time.monotonic() < t_end:
            s = drv.last_status
            if s:
                print(f"{s.fault:<20} {s.voltage_v:5.1f} V  "
                      f"M1 {s.motor1_rpm:5d} rpm {s.motor1_current_a:4.1f} A  "
                      f"M2 {s.motor2_rpm:5d} rpm {s.motor2_current_a:4.1f} A")
            time.sleep(0.5)
    except KeyboardInterrupt:
        pass
    finally:
        drv.close()
        print("closed, motors disabled")
