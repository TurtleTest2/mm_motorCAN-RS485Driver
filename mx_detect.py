"""
Auto-detect an MX_SERVE controller: which COM port, baud rate and address.

SAFE: only sends READ requests (Modbus 0x03). It never writes the control
registers, so it cannot enable or move a motor.

Usage:
    python mx_detect.py               # scan every COM port
    python mx_detect.py COM5          # scan one port
    python mx_detect.py COM5 --full   # every address 1-127 (finds multiple controllers)

Strategy: first tries the factory default address 1 at every baud rate
(a few seconds). If nothing answers, it scans addresses 1-127 at every baud
(~15 s per baud rate). Close the GUI / vendor tool first - only one program
can hold a COM port.
"""

from __future__ import annotations

import sys
import threading
import time
from dataclasses import dataclass, field
from typing import Callable, Optional

from mx_serve import (FUNC_READ, MIN_FRAME_GAP_S, STATUS_COUNT, STATUS_START, Status,
                      build_read_frame, check_crc)

BAUD_RATES = [9600, 19200, 38400, 57600, 115200]     # 9600 = factory default, tried first
REPLY_LEN = 5 + 2 * STATUS_COUNT                     # addr, func, count, 14 data bytes, CRC
PROBE_TIMEOUT_S = 0.1


@dataclass
class Found:
    port: str
    baud: int
    address: int
    status: Optional[Status] = None
    rejected: bool = False          # answered with an error frame (still proves the settings)


@dataclass
class PortReport:
    port: str
    found: list[Found] = field(default_factory=list)
    error: Optional[str] = None     # could not open the port
    noise_bauds: set[int] = field(default_factory=set)   # got bytes, but no valid frame
    echo: bool = False              # adapter echoes our own request back
    cancelled: bool = False


def list_ports() -> list[tuple[str, str]]:
    from serial.tools import list_ports as lp
    return [(p.device, p.description) for p in sorted(lp.comports(), key=lambda p: p.device)]


def _find_frame(buf: bytes, address: int) -> Optional[tuple[Optional[Status], bool]]:
    """Look for a valid status reply (or error reply) from `address` in buf."""
    for i in range(len(buf)):
        if buf[i] != address:
            continue
        f = buf[i:i + REPLY_LEN]
        if len(f) == REPLY_LEN and f[1] == FUNC_READ and f[2] == 2 * STATUS_COUNT and check_crc(f):
            regs = [int.from_bytes(f[3 + 2 * k:5 + 2 * k], "big") for k in range(STATUS_COUNT)]
            return Status.from_registers(regs), False
        e = buf[i:i + 6]
        if len(e) == 6 and e[1] == FUNC_READ | 0x80 and check_crc(e):
            return None, True
    return None


def probe(ser, address: int, rep: PortReport, timeout: float = PROBE_TIMEOUT_S):
    """Send one status read to `address`. Returns (status, rejected) or None."""
    req = build_read_frame(address, STATUS_START, STATUS_COUNT)
    time.sleep(MIN_FRAME_GAP_S)
    ser.reset_input_buffer()
    ser.write(req)
    ser.flush()
    buf = b""
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        chunk = ser.read(64)
        if chunk:
            buf += chunk
            hit = _find_frame(buf.replace(req, b""), address)
            if hit:
                if req in buf:
                    rep.echo = True
                return hit
    if req in buf:
        rep.echo = True
        buf = buf.replace(req, b"")
    if buf:
        rep.noise_bauds.add(ser.baudrate)
    return None


def scan_port(port: str, bauds: list[int] = BAUD_RATES, full: bool = False,
              progress: Optional[Callable[[str], None]] = None,
              stop: Optional[threading.Event] = None,
              on_step: Optional[Callable[[int, int], None]] = None) -> PortReport:
    """`on_step(done, total)` is called after every probe (total = worst case for this port)."""
    import serial
    rep = PortReport(port)
    say = progress or (lambda msg: None)
    total = len(bauds) * 128            # phase 1 (addr 1 per baud) + phase 2 (127 per baud)
    done = 0
    say(f"{port}: opening port…")
    try:
        ser = serial.Serial(port, baudrate=bauds[0], bytesize=8, parity="N", stopbits=1,
                            timeout=0.01, write_timeout=0.5)
    except Exception as e:
        rep.error = str(e)
        return rep

    def try_addr(baud: int, addr: int) -> bool:
        nonlocal done
        try:
            hit = probe(ser, addr, rep)
        except serial.SerialTimeoutException:
            hit = None                  # adapter not accepting data; keep going so cancel still works
        done += 1
        if on_step:
            on_step(done, total)
        if hit:
            rep.found.append(Found(port, baud, addr, *hit))
            say(f"{port}: FOUND address {addr} at {baud} baud")
        return bool(hit)

    with ser:
        # Phase 1: factory-default address 1 at every baud rate (fast).
        hit_baud = None
        for b in bauds:
            if stop and stop.is_set():
                rep.cancelled = True
                return rep
            ser.baudrate = b
            say(f"{port}: {b} baud, address 1")
            if try_addr(b, 1):
                hit_baud = b
                break
        if rep.found and not full:
            return rep

        # Phase 2: all addresses. A bus runs at one baud, so once something
        # answers we only need to finish that baud rate.
        for b in ([hit_baud] if hit_baud else bauds):
            ser.baudrate = b
            for a in range(1, 128):
                if stop and stop.is_set():
                    rep.cancelled = True
                    return rep
                if any(f.address == a and f.baud == b for f in rep.found):
                    continue
                say(f"{port}: {b} baud, address {a}/127")
                if try_addr(b, a) and not full:
                    return rep
            if rep.found:
                break
    return rep


def explain(rep: PortReport) -> str:
    """Human-readable result + troubleshooting hints."""
    if rep.error:
        hint = ("close the GUI, vendor tool or any serial monitor using it"
                if "denied" in rep.error.lower() or "busy" in rep.error.lower() else "check the port name")
        return f"{rep.port}: could not open ({rep.error}) - {hint}."
    lines = []
    for f in rep.found:
        if f.status:
            s = f.status
            lines.append(f"{f.port}: FOUND  baud={f.baud}  address={f.address}   "
                         f"[{s.voltage_v:.1f} V, {s.temperature_c} °C, fault {s.fault_code}: {s.fault}]")
        else:
            lines.append(f"{f.port}: FOUND  baud={f.baud}  address={f.address}   (answered with an error frame)")
    if rep.cancelled:
        lines.append(f"{rep.port}: scan cancelled.")
    if not rep.found and not rep.cancelled:
        lines.append(f"{rep.port}: no controller answered at any baud rate / address.")
        if rep.noise_bauds:
            lines.append(f"  Received unreadable data at {sorted(rep.noise_bauds)} baud -> likely electrical noise, "
                         "a missing GND wire or missing termination.")
        else:
            lines.append("  Nothing at all came back. Check, in order:\n"
                         "   1. A/B wires swapped (most common - swapping them is harmless)\n"
                         "   2. GND connected between adapter and controller\n"
                         "   3. Controller powered on\n"
                         "   4. Controller set to RS485 control in the vendor tool\n"
                         "   5. This is the right COM port (unplug/replug the adapter and watch Device Manager)")
    if rep.echo:
        lines.append("  NOTE: the adapter echoes what it sends. mx_serve.py does not expect an echo, so "
                     "the GUI will report errors - disable echo on the adapter if it has a switch/jumper.")
    return "\n".join(lines)


def main(argv: list[str]) -> int:
    try:
        import serial  # noqa: F401
    except ImportError:
        print("pyserial is not installed:  python -m pip install pyserial")
        return 2
    full = "--full" in argv
    args = [a for a in argv if not a.startswith("--")]
    ports = args or [p for p, _ in list_ports()]
    if not ports:
        print("No COM ports found. Plug in the USB-RS485 adapter; if it still doesn't appear, "
              "install its driver (CH340, CP210x or FTDI) - check Device Manager.")
        return 1
    print("Read-only scan (motors will not move). Ctrl+C to cancel.\n")
    any_found = False
    for port in ports:
        try:
            rep = scan_port(port, full=full,
                            progress=lambda m: print(f"\r  scanning {m:<50}", end="", flush=True))
        except KeyboardInterrupt:
            print("\ncancelled")
            return 130
        print("\r" + " " * 70 + "\r" + explain(rep))
        any_found |= bool(rep.found)
    if any_found:
        print("\nUse these Port / Baud / Address values in mx_serve_gui.py.")
    return 0 if any_found else 1


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
