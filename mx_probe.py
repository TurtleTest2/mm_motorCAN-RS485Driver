#!/usr/bin/env python3
"""Probe a MX-ES / MX_SERVE dual brushless driver over RS485 (Modbus-RTU style, fn 0x03).
Usage:  python mx_probe.py COM6 --baud 115200 --addr 1   (defaults: 115200, addr 1)
        python mx_probe.py /dev/ttyUSB0
        python mx_probe.py COM5 --scan     (try every address 1-127 and common bauds)
Needs:  pip install pyserial
"""
import sys, time

def crc16(data: bytes) -> bytes:
    crc = 0xFFFF
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = (crc >> 1) ^ 0xA001 if crc & 1 else crc >> 1
    return bytes([crc & 0xFF, crc >> 8])          # CRC_L, CRC_H

def read_regs(addr, start=0, count=7):
    body = bytes([addr, 0x03, start >> 8, start & 0xFF, count >> 8, count & 0xFF])
    return body + crc16(body)

def try_once(ser, addr, count=7):
    ser.reset_input_buffer()
    ser.write(read_regs(addr, 0, count)); ser.flush()
    time.sleep(0.15)
    return ser.read(64)

def decode(resp):
    if len(resp) < 5 or crc16(resp[:-2]) != resp[-2:]:
        return None
    if resp[1] == 0x83:
        return f"error response, code {resp[2]:#x}"
    n = resp[2] // 2
    v = [int.from_bytes(resp[3+2*i:5+2*i], 'big', signed=True) for i in range(n)]
    names = ["fault", "temp C", "Vbus x0.1V", "M2 cur x0.1A", "M1 cur x0.1A", "M2 rpm", "M1 rpm"]
    return dict(zip(names, v))

if __name__ == "__main__":
    import serial
    # self-check against vendor examples
    assert read_regs(1, 0, 1).hex() == "01030000000184" + "0a"
    assert read_regs(1, 0, 7).hex() == "010300000007" + "0408"
    port = sys.argv[1]
    # optional:  --baud 115200   --addr 1   --scan
    def opt(name, default):
        return int(sys.argv[sys.argv.index(name) + 1]) if name in sys.argv else default
    scan = "--scan" in sys.argv
    bauds = [9600, 19200, 38400, 57600, 115200] if scan else [opt("--baud", 115200)]
    addrs = range(1, 128) if scan else [opt("--addr", 1)]
    for baud in bauds:
        with serial.Serial(port, baud, bytesize=8, parity='N', stopbits=1, timeout=0.2) as ser:
            for a in addrs:
                r = try_once(ser, a)
                if r:
                    print(f"baud {baud} addr {a}: raw {r.hex(' ')}  ->  {decode(r)}")
                    sys.exit(0)
                time.sleep(0.03)                  # >=20 ms idle between frames
        print(f"no reply at {baud}")
    print("No response.")
