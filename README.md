# MX dual brushless motor controller – RS485 & CAN test tools

Python drivers and Windows test GUIs for UUMOTOR MX-series dual brushless
motor controllers (e.g. MXES-1KW / MX_SERVE), over **RS485 (Modbus RTU)** or
**CAN (extended frames)**.

| File | What it is |
|---|---|
| `mx_serve.py` | RS485 / Modbus RTU driver (status read fn 0x03, control write fn 0x10, 100 ms heartbeat) |
| `mx_serve_gui.py` | RS485 test GUI: speed control, live speed / current / power / energy / voltage / temperature, CSV log, auto-detect |
| `mx_detect.py` | Read-only RS485 auto-detect (COM port, baud rate, address) |
| `mx_probe.py` | Minimal stand-alone RS485 probe |
| `mx_can.py` | CAN driver (protocol MX_CAN-SV3.03-1) using python-can |
| `mx_can_gui.py` | CAN test GUI: speed control, live data, bus monitor, statistics & fault history |
| `mx_stats.py` | Derived statistics (speed error, turns/distance, measured accel, voltage sag, link quality, faults) |

## Setup

```powershell
python -m venv .venv
.\.venv\Scripts\python.exe -m pip install -r requirements.txt
```

## Run

```powershell
# RS485
.\.venv\Scripts\python.exe mx_serve_gui.py          # pick COM port, or use Auto-detect
.\.venv\Scripts\python.exe mx_serve_gui.py --sim    # simulator, no hardware

# CAN
.\.venv\Scripts\python.exe mx_can_gui.py            # pick adapter + channel, 500 kbit/s
.\.venv\Scripts\python.exe mx_can_gui.py --sim      # simulator, no hardware

# Self-checks (frame encoding vs. the protocol's examples)
.\.venv\Scripts\python.exe mx_can.py
.\.venv\Scripts\python.exe mx_stats.py
```

### Keyboard (both GUIs, when not typing in a text box)

| Key | Action |
|---|---|
| D / A | Motor 1 speed + / − |
| → / ← | Motor 2 speed + / − |
| Space / Esc | **STOP ALL** (zero speed, disable both motors) |

"Link motors" makes any of these move both motors.

## Wiring (16-pin function connector)

| Signal | Pin |
|---|---|
| 485A / 485B | 4 / 2 |
| CANH / CANL | 10 / 12 |
| GND | 16 |

Pin 15 is battery B+ – never connect it to an adapter. The controller only
listens on the interface selected in the vendor configuration tool
(RS485 or CAN); save and power-cycle after changing it. CAN needs 120 Ω
termination at each end of the bus.

## Protocol summary

**RS485 (Modbus RTU, 8N1):** read registers 0x0000–0x0006 (fault, temperature,
voltage ×0.1 V, M2 current, M1 current ×0.1 A, M2 rpm, M1 rpm); write 9
control registers from 0x0000. ≥ 20 ms between frames, 1.5 s watchdog.

**CAN (500 kbit/s, extended):** host → driver `0x1801E600` every 100 ms
(control word, M2 setpoint, M2 accel, M1 setpoint, M1 accel);
driver → host `0x1801E001` (fault, temperature, M2 rpm, M1 rpm, voltage ×0.1 V).
The CAN status frame has no motor current.

Supported CAN adapters (via python-can): Waveshare / Seeed USB-CAN-A,
CANable (slcan), PEAK PCAN-USB, Kvaser, candleLight (gs_usb), IXXAT, Vector,
and Linux SocketCAN.

## Building Windows executables

```powershell
.\.venv\Scripts\python.exe -m pip install pyinstaller
.\.venv\Scripts\python.exe -m PyInstaller --noconfirm --onefile --windowed --name MXServeGUI --distpath dist --workpath build --specpath build --hidden-import serial.tools.list_ports mx_serve_gui.py
.\.venv\Scripts\python.exe -m PyInstaller --noconfirm --onefile --windowed --name MXCanGUI --distpath dist --workpath build --specpath build --collect-submodules can.interfaces --copy-metadata python-can --hidden-import serial.tools.list_ports mx_can_gui.py
```

## Safety

Test with the motors unloaded and a low speed limit first. Keep a hand near
STOP ALL (Space / Esc); the controller also stops the motors by itself if it
hears nothing for 1.5 s.
