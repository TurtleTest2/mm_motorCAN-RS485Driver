"""
MX_SERVE live control panel - increase/decrease motor speed while watching
live sensor data (fault, temperature, bus voltage, motor currents, speeds).

Uses mx_serve.py (same folder) for the RS485 / Modbus protocol. The driver's
heartbeat thread re-sends the control frame and polls status every 100 ms,
so the controller's 1.5 s watchdog never trips while this window is open.

Run:
    pip install pyserial
    python mx_serve_gui.py            # pick COM port in the window
    python mx_serve_gui.py --sim      # no hardware: simulated controller

Keyboard (when not typing in a text box):
    D / A        Motor 1 speed + / -
    Right / Left Motor 2 speed + / -
    Space / Esc  STOP ALL (zero speed, disable both motors)
"""

from __future__ import annotations

import csv
import struct
import sys
import threading
import time
import tkinter as tk
from collections import deque
from tkinter import filedialog, messagebox, ttk

from mx_serve import (
    FUNC_READ,
    FUNC_WRITE,
    MXServeDriver,
    Mode,
    Status,
    append_crc,
    check_crc,
)

POLL_UI_MS = 100           # how often the UI refreshes from driver state
HISTORY_S = 30             # seconds shown in the live charts
BAUD_RATES = ["9600", "19200", "38400", "57600", "115200"]
STEP_SIZES = ["1", "10", "50", "100", "250"]


# --------------------------------------------------------------------------
# Simulated controller (for --sim / "Simulator" port): answers Modbus frames
# --------------------------------------------------------------------------
class FakeSerial:
    """Minimal serial-like object that emulates an MX_SERVE controller."""

    def __init__(self, address: int = 1):
        self.address = address
        self._out = b""
        self._lock = threading.Lock()
        self.ctrl = [0, 0, 0, 100, 100, 1000, 1000, 2000, 2000]
        self.rpm = {1: 0.0, 2: 0.0}
        self._t = time.monotonic()

    def _physics(self) -> None:
        now = time.monotonic()
        dt, self._t = now - self._t, now
        reg0 = self.ctrl[0]
        for motor, en_bit, sp_idx, acc_idx in ((1, reg0 & 1, 2, 4), (2, (reg0 >> 8) & 1, 1, 3)):
            sp = self.ctrl[sp_idx]
            sp = sp - 0x10000 if sp & 0x8000 else sp
            target = sp if en_bit else 0
            step = max(1, self.ctrl[acc_idx]) * dt
            cur = self.rpm[motor]
            self.rpm[motor] = min(target, cur + step) if target > cur else max(target, cur - step)

    def _status_regs(self) -> list[int]:
        self._physics()
        import random
        cur = {m: abs(self.rpm[m]) / 200 + random.uniform(0, 0.3) for m in (1, 2)}
        vals = [0, 31 + int(sum(cur.values())), int((48.0 - sum(cur.values()) * 0.1) * 10),
                int(cur[2] * 10), int(cur[1] * 10), int(self.rpm[2]), int(self.rpm[1])]
        return [v & 0xFFFF for v in vals]

    def write(self, data: bytes) -> int:
        with self._lock:
            if not check_crc(data) or data[0] != self.address:
                return len(data)
            if data[1] == FUNC_READ:
                start, count = struct.unpack(">HH", data[2:6])
                regs = self._status_regs()[start:start + count]
                body = struct.pack(">BBB", self.address, FUNC_READ, 2 * len(regs))
                self._out += append_crc(body + struct.pack(f">{len(regs)}H", *regs))
            elif data[1] == FUNC_WRITE:
                start, count = struct.unpack(">HH", data[2:6])
                regs = struct.unpack(f">{count}H", data[7:7 + 2 * count])
                self._physics()
                self.ctrl[start:start + count] = regs
                self._out += append_crc(data[:6])
            return len(data)

    def read(self, n: int) -> bytes:
        with self._lock:
            chunk, self._out = self._out[:n], self._out[n:]
        if not chunk:
            time.sleep(0.005)
        return chunk

    def reset_input_buffer(self) -> None:
        with self._lock:
            self._out = b""

    def flush(self) -> None:
        pass

    def close(self) -> None:
        pass


# --------------------------------------------------------------------------
# Strip chart
# --------------------------------------------------------------------------
class StripChart(tk.Canvas):
    def __init__(self, master, title: str, unit: str, series: list[tuple[str, str, bool]],
                 include_zero: bool = True, min_span: float = 1.0, **kw):
        super().__init__(master, height=130, bg="#14171c", highlightthickness=0, **kw)
        self.title, self.unit = title, unit
        self.series = series                     # (key, color, dashed)
        self.include_zero = include_zero         # False -> autoscale tightly (voltage, temp)
        self.min_span = min_span                 # smallest y-range, so noise isn't magnified
        self.bind("<Configure>", lambda e: self.redraw([]))

    def redraw(self, samples) -> None:
        self.delete("all")
        w, h = self.winfo_width(), self.winfo_height()
        L, R, T, B = 52, 10, 22, 20
        self.create_text(8, 4, anchor="nw", fill="#cfd6df", text=f"{self.title} ({self.unit})",
                         font=("Segoe UI", 9, "bold"))
        x = 160
        for key, color, dashed in self.series:
            self.create_line(x, 11, x + 16, 11, fill=color, width=2, dash=(4, 3) if dashed else None)
            self.create_text(x + 20, 11, anchor="w", fill="#9aa4b0", text=key, font=("Segoe UI", 8))
            x += 30 + 7 * len(key)
        if w < 100:
            return
        now = time.monotonic()
        pts = [s for s in samples if now - s["t"] <= HISTORY_S]
        vals = [s[k] for s in pts for k, _, _ in self.series if s.get(k) is not None]
        if self.include_zero:
            vals = vals + [0]
        lo, hi = (min(vals), max(vals)) if vals else (0, 1)
        if hi - lo < self.min_span:
            mid = (hi + lo) / 2
            lo, hi = mid - self.min_span / 2, mid + self.min_span / 2
        pad = (hi - lo) * 0.1
        lo, hi = lo - pad, hi + pad

        def X(t): return L + (w - L - R) * (1 - (now - t) / HISTORY_S)
        def Y(v): return T + (h - T - B) * (1 - (v - lo) / (hi - lo))

        for i in range(5):
            v = lo + (hi - lo) * i / 4
            y = Y(v)
            self.create_line(L, y, w - R, y, fill="#262b33")
            fmt = "{:.3f}" if hi - lo < 0.5 else "{:.1f}" if hi - lo < 20 else "{:.0f}"
            self.create_text(L - 4, y, anchor="e", fill="#7d8794", text=fmt.format(v),
                             font=("Consolas", 8))
        if lo < 0 < hi:
            self.create_line(L, Y(0), w - R, Y(0), fill="#3a414b")
        for s in range(0, HISTORY_S + 1, 5):
            self.create_text(X(now - s), h - 4, anchor="s", fill="#7d8794", text=f"-{s}s" if s else "now",
                             font=("Segoe UI", 7))
        for key, color, dashed in self.series:
            coords = []
            for s in pts:
                if s.get(key) is not None:
                    coords += [X(s["t"]), Y(s[key])]
            if len(coords) >= 4:
                self.create_line(*coords, fill=color, width=2, dash=(4, 3) if dashed else None)


# --------------------------------------------------------------------------
# Main window
# --------------------------------------------------------------------------
class App(tk.Tk):
    def __init__(self, start_sim: bool = False):
        super().__init__()
        self.title("MX_SERVE Motor Control")
        # Fit the screen (laptops are often shorter than 900 px once the taskbar is counted).
        sw, sh = self.winfo_screenwidth(), self.winfo_screenheight()
        w, h = min(1100, sw - 40), min(900, sh - 100)
        self.geometry(f"{w}x{h}+{max(0, (sw - w) // 2)}+10")
        self.minsize(min(950, w), min(600, h))
        style = ttk.Style(self)
        if "clam" in style.theme_names():
            style.theme_use("clam")
        style.configure("Big.TLabel", font=("Consolas", 22, "bold"))
        style.configure("Val.TLabel", font=("Consolas", 13))
        style.configure("Stop.TButton", font=("Segoe UI", 14, "bold"), foreground="white", background="#c0392b")
        style.map("Stop.TButton", background=[("active", "#e74c3c")])

        self.drv: MXServeDriver | None = None
        self.history: deque = deque()
        self.log_writer = None
        self.log_file = None
        self.error_count = 0
        self.status_count = 0
        self._rate_t, self._rate_n, self.poll_rate = time.monotonic(), 0, 0.0
        self.energy_wh = {1: 0.0, 2: 0.0}       # integrated V*I per motor
        self._last_power: tuple[float, float, float] | None = None   # (t, P1, P2)
        self._detect_thread: threading.Thread | None = None
        self._detect_stop = threading.Event()
        self._detect_started = 0.0
        self._detect_msg, self._detect_result = "", None
        self._detect_gen = 0                       # bumps on every start/cancel; stale threads are ignored
        self._detect_step = (0, 1)                 # (probes done, total) for the current port
        self._detect_port_idx = (0, 1)             # (current port index, number of ports)

        self._build_connection_bar()
        # Packed before the body so it keeps its space when the window is short;
        # the charts shrink instead.
        sb_frame = ttk.Frame(self, relief="sunken")
        sb_frame.pack(fill="x", side="bottom")
        self.detect_bar = ttk.Progressbar(sb_frame, length=180, mode="determinate", maximum=100)
        self.statusbar = ttk.Label(sb_frame, text="Disconnected", anchor="w", padding=(8, 3),
                                   font=("Segoe UI", 10))
        self.statusbar.pack(fill="x", side="left", expand=True)
        body = ttk.Frame(self, padding=8)
        body.pack(fill="both", expand=True)
        self._build_status_panel(body)
        self.motor_vars = {}
        mf = ttk.Frame(body)
        mf.pack(fill="x", pady=6)
        for col, m in enumerate((1, 2)):
            self._build_motor_panel(mf, m).grid(row=0, column=col, sticky="nsew", padx=4)
            mf.columnconfigure(col, weight=1)
        self._build_global_bar(body)
        charts = ttk.Frame(body)
        charts.pack(fill="both", expand=True, pady=(6, 0))
        M1, M2, TOT = "#4fc3f7", "#ffb74d", "#aed581"
        self.charts = [
            StripChart(charts, "Speed", "r/min",
                       [("M1", M1, False), ("M1 set", M1, True), ("M2", M2, False), ("M2 set", M2, True)]),
            StripChart(charts, "Motor current", "A", [("M1 A", M1, False), ("M2 A", M2, False)]),
            StripChart(charts, "Power", "W", [("M1 W", M1, False), ("M2 W", M2, False), ("Total W", TOT, False)]),
            StripChart(charts, "Energy", "Wh", [("M1 Wh", M1, False), ("M2 Wh", M2, False),
                                                ("Total Wh", TOT, False)], min_span=0.01),
            StripChart(charts, "Bus voltage", "V", [("V", "#ba68c8", False)], include_zero=False),
            StripChart(charts, "Controller temp", "°C", [("°C", "#e57373", False)],
                       include_zero=False, min_span=5),
        ]
        for i, ch in enumerate(self.charts):
            ch.grid(row=i // 2, column=i % 2, sticky="nsew", padx=3, pady=3)
        for c in (0, 1):
            charts.columnconfigure(c, weight=1)
        for r in range(3):
            charts.rowconfigure(r, weight=1)


        self._bind_keys()
        self._set_controls_state(False)
        self.protocol("WM_DELETE_WINDOW", self.on_close)
        if start_sim:
            self.port_var.set("Simulator")
            self.after(200, self.connect)
        self.after(POLL_UI_MS, self._tick)

    # ------------------------------------------------------------ building
    def _build_connection_bar(self):
        bar = ttk.Frame(self, padding=(8, 8, 8, 0))
        bar.pack(fill="x")
        ttk.Label(bar, text="Port").pack(side="left")
        self.port_var = tk.StringVar()
        self.port_cb = ttk.Combobox(bar, textvariable=self.port_var, width=28)
        self.port_cb.pack(side="left", padx=(4, 2))
        ttk.Button(bar, text="↻", width=3, command=self.refresh_ports).pack(side="left", padx=(0, 10))
        ttk.Label(bar, text="Baud").pack(side="left")
        self.baud_var = tk.StringVar(value="9600")
        ttk.Combobox(bar, textvariable=self.baud_var, values=BAUD_RATES, width=8).pack(side="left", padx=(4, 10))
        ttk.Label(bar, text="Address").pack(side="left")
        self.addr_var = tk.StringVar(value="1")
        ttk.Spinbox(bar, from_=1, to=127, textvariable=self.addr_var, width=5).pack(side="left", padx=(4, 10))
        self.connect_btn = ttk.Button(bar, text="Connect", command=self.toggle_connect)
        self.connect_btn.pack(side="left", padx=4)
        self.detect_btn = ttk.Button(bar, text="Auto-detect", command=self.toggle_detect)
        self.detect_btn.pack(side="left", padx=4)
        self.log_btn = ttk.Button(bar, text="Start CSV log", command=self.toggle_log)
        self.log_btn.pack(side="right")
        self.refresh_ports()

    def _build_status_panel(self, parent):
        box = ttk.LabelFrame(parent, text="Live sensor data", padding=8)
        box.pack(fill="x")
        self.stat_vars = {}
        rows = [
            [("fault", "Fault"), ("temp", "Controller temp"), ("volt", "Bus voltage"),
             ("ptot", "Total power"), ("energy", "Energy (M1 + M2)")],
            [("m1rpm", "M1 speed"), ("m1a", "M1 current"), ("m1w", "M1 power"),
             ("m2rpm", "M2 speed"), ("m2a", "M2 current"), ("m2w", "M2 power")],
        ]
        for r, fields in enumerate(rows):
            for i, (key, label) in enumerate(fields):
                f = ttk.Frame(box)
                f.grid(row=r, column=i, padx=8, pady=(0 if r == 0 else 6, 0), sticky="w")
                head = ttk.Frame(f)
                head.pack(anchor="w", fill="x")
                ttk.Label(head, text=label, foreground="#666").pack(side="left")
                if key == "energy":
                    ttk.Button(head, text="Reset", width=6, command=self.reset_energy).pack(side="left", padx=6)
                v = tk.StringVar(value="—")
                lbl = ttk.Label(f, textvariable=v, style="Val.TLabel")
                lbl.pack(anchor="w")
                self.stat_vars[key] = (v, lbl)
                box.columnconfigure(i, weight=1)

    def _build_motor_panel(self, parent, m: int):
        box = ttk.LabelFrame(parent, text=f"Motor {m}" + ("   (keys D / A)" if m == 1 else "   (keys → / ←)"),
                             padding=10)
        v = {
            "enabled": tk.BooleanVar(value=False),
            "setpoint": tk.IntVar(value=0),
            "step": tk.StringVar(value="50"),
            "accel": tk.StringVar(value="200"),
            "actual": tk.StringVar(value="— rpm"),
        }
        self.motor_vars[m] = v
        top = ttk.Frame(box)
        top.pack(fill="x")
        v["enable_cb"] = ttk.Checkbutton(top, text="Enabled", variable=v["enabled"],
                                         command=lambda: self.on_enable(m))
        v["enable_cb"].pack(side="left")
        ttk.Label(top, textvariable=v["actual"], foreground="#555").pack(side="right")

        mid = ttk.Frame(box)
        mid.pack(fill="x", pady=8)
        v["minus"] = ttk.Button(mid, text="−", width=4, command=lambda: self.nudge(m, -1))
        v["minus"].pack(side="left")
        ttk.Label(mid, textvariable=v["setpoint"], style="Big.TLabel", width=7, anchor="center").pack(
            side="left", expand=True)
        ttk.Label(mid, text="rpm set").pack(side="left")
        v["plus"] = ttk.Button(mid, text="+", width=4, command=lambda: self.nudge(m, +1))
        v["plus"].pack(side="right")

        v["scale"] = ttk.Scale(box, from_=-self.max_rpm(), to=self.max_rpm(), orient="horizontal",
                               command=lambda val: self.on_scale(m, val))
        v["scale"].pack(fill="x")

        bot = ttk.Frame(box)
        bot.pack(fill="x", pady=(8, 0))
        ttk.Label(bot, text="Step").pack(side="left")
        ttk.Combobox(bot, textvariable=v["step"], values=STEP_SIZES, width=5).pack(side="left", padx=(4, 12))
        ttk.Label(bot, text="Accel (rpm/s)").pack(side="left")
        acc = ttk.Spinbox(bot, from_=1, to=10000, increment=50, textvariable=v["accel"], width=7,
                          command=lambda: self.on_accel(m))
        acc.pack(side="left", padx=4)
        acc.bind("<Return>", lambda e: self.on_accel(m))
        acc.bind("<FocusOut>", lambda e: self.on_accel(m))
        v["zero"] = ttk.Button(bot, text="Zero", command=lambda: self.set_setpoint(m, 0))
        v["zero"].pack(side="right")
        v["box"] = box
        return box

    def _build_global_bar(self, parent):
        bar = ttk.Frame(parent)
        bar.pack(fill="x", pady=(2, 0))
        self.stop_btn = ttk.Button(bar, text="■  STOP ALL  (Space)", style="Stop.TButton", command=self.stop_all)
        self.stop_btn.pack(side="left", ipadx=10, ipady=4)
        self.link_var = tk.BooleanVar(value=False)
        ttk.Checkbutton(bar, text="Link motors (D/A, →/← and +/− move both)",
                        variable=self.link_var).pack(side="left", padx=16)
        ttk.Label(bar, text="Speed limit ±").pack(side="left", padx=(16, 2))
        self.limit_var = tk.StringVar(value="1500")
        lim = ttk.Spinbox(bar, from_=10, to=32767, increment=100, textvariable=self.limit_var, width=7,
                          command=self.on_limit)
        lim.pack(side="left")
        lim.bind("<Return>", lambda e: self.on_limit())
        lim.bind("<FocusOut>", lambda e: self.on_limit())
        ttk.Label(bar, text="rpm").pack(side="left", padx=2)

    def _bind_keys(self):
        text_boxes = (tk.Entry, ttk.Entry, ttk.Spinbox, ttk.Combobox)

        def guard(fn):
            def h(e):
                if isinstance(self.focus_get(), text_boxes):
                    return          # arrows/keys belong to the text box being edited
                fn()
                return "break"
            return h

        # Let go of a text box (so the motor keys work again) on Enter or on a
        # click anywhere that isn't another text box.
        def release_on_click(e):
            try:
                focused = self.focus_get()
            except KeyError:        # tkinter quirk while a combobox dropdown is open
                return
            if not isinstance(e.widget, text_boxes) and isinstance(focused, text_boxes):
                self.focus_set()
        self.bind_all("<Button-1>", release_on_click, add="+")
        for cls in ("TSpinbox", "TCombobox", "TEntry"):
            self.bind_class(cls, "<Return>", lambda e: self.after_idle(self.focus_set), add="+")
        for key in ("d", "D"):      # both cases, so Caps Lock doesn't break it
            self.bind(f"<KeyPress-{key}>", guard(lambda: self.nudge(1, +1)))
        for key in ("a", "A"):
            self.bind(f"<KeyPress-{key}>", guard(lambda: self.nudge(1, -1)))
        self.bind("<Right>", guard(lambda: self.nudge(2, +1)))
        self.bind("<Left>", guard(lambda: self.nudge(2, -1)))
        self.bind("<space>", guard(self.stop_all))
        self.bind("<Escape>", self._on_escape)             # always works, even while typing

    # ---------------------------------------------------------- connection
    def refresh_ports(self):
        ports = ["Simulator"]
        try:
            from serial.tools import list_ports
            ports = [f"{p.device} - {p.description}" for p in list_ports.comports()] + ports
        except ImportError:
            pass
        self.port_cb["values"] = ports
        if not self.port_var.get() and ports:
            self.port_var.set(ports[0])

    def toggle_connect(self):
        self.disconnect() if self.drv else self.connect()

    # ---------------------------------------------------------- auto-detect
    def _detect_running(self) -> bool:
        return self.detect_btn.cget("text") != "Auto-detect"

    def cancel_detect(self):
        """Stop the scan and give the UI back immediately; the worker exits on its own."""
        self._detect_stop.set()
        self._detect_gen += 1                      # any late result from the old thread is dropped
        self._detect_msg, self._detect_result = "", None
        self.detect_bar.pack_forget()
        self.detect_btn.config(text="Auto-detect")
        self.connect_btn.state(["!disabled"])
        self.statusbar.config(text="Auto-detect cancelled - click Auto-detect to scan again")

    def toggle_detect(self):
        if self._detect_running():
            # Ignore the second half of a double-click; only a deliberate later click cancels.
            if time.monotonic() - self._detect_started > 1.0:
                self.cancel_detect()
            return
        old = self._detect_thread
        if old and old.is_alive():
            old.join(timeout=1.0)                  # let a cancelled scan release the COM port
            if old.is_alive():
                self.statusbar.config(text="Previous scan is still closing the COM port - "
                                           "try Auto-detect again in a few seconds")
                return
        try:
            import serial  # noqa: F401
            from mx_detect import list_ports
        except ImportError:
            messagebox.showerror("pyserial missing", "Install it with:\n\npython -m pip install pyserial")
            return
        if self.drv:
            self.disconnect()       # the scan needs the port to itself
        sel = self.port_var.get().split(" - ")[0].strip()
        ports = [sel] if sel and sel != "Simulator" else [p for p, _ in list_ports()]
        if not ports:
            messagebox.showinfo("Auto-detect", "No COM ports found.\n\nPlug in the USB-RS485 adapter; if it "
                                "still doesn't appear, install its driver (CH340, CP210x or FTDI).")
            return
        self._detect_stop = threading.Event()      # fresh event per scan, so an old thread can't be revived
        stop = self._detect_stop
        self._detect_gen += 1
        gen = self._detect_gen
        self._detect_started = time.monotonic()
        self._detect_msg, self._detect_result = "Auto-detect starting…", None
        self._detect_step, self._detect_port_idx = (0, 1), (0, len(ports))
        self.detect_btn.config(text="Cancel detect")
        self.connect_btn.state(["disabled"])
        self.detect_bar.config(value=0)
        self.detect_bar.pack(side="right", padx=8, pady=2, before=self.statusbar)
        self.statusbar.config(text="Auto-detect starting…")

        def work():
            reports = []

            def live(attr, value):                 # only the current scan may touch the UI state
                if gen == self._detect_gen:
                    setattr(self, attr, value)
            try:
                from mx_detect import scan_port
                for i, p in enumerate(ports):
                    if stop.is_set():
                        break
                    live("_detect_port_idx", (i, len(ports)))
                    live("_detect_step", (0, 1))
                    reports.append(scan_port(
                        p, stop=stop,
                        progress=lambda m: live("_detect_msg", f"Auto-detect: {m}"),
                        on_step=lambda d, t: live("_detect_step", (d, t))))
                live("_detect_result", reports)
            except Exception as e:                 # never leave the UI stuck on "Cancel detect"
                live("_detect_result", e)
        self._detect_thread = threading.Thread(target=work, daemon=True)
        self._detect_thread.start()

    def _poll_detect(self):
        """Runs on the UI thread from _tick: show progress, then the result."""
        if not self._detect_running():
            return
        if self._detect_result is None:
            done, total = self._detect_step
            pi, pn = self._detect_port_idx
            frac = (pi + min(done / max(total, 1), 1.0)) / max(pn, 1)
            self.detect_bar.config(value=100 * frac)
            el = time.monotonic() - self._detect_started
            ports = f"port {pi + 1}/{pn}   |   " if pn > 1 else ""
            self.statusbar.config(text=f"{self._detect_msg}   |   {ports}{done}/{total} tries   |   "
                                       f"{el:.0f} s   (read-only, motors will not move - click Cancel detect to stop)")
            return
        reports, self._detect_result, self._detect_msg = self._detect_result, None, ""
        self.detect_bar.pack_forget()
        self.detect_btn.config(text="Auto-detect")
        self.connect_btn.state(["!disabled"])
        if isinstance(reports, Exception):
            self.statusbar.config(text=f"Auto-detect failed: {reports}")
            messagebox.showerror("Auto-detect failed", f"{type(reports).__name__}: {reports}")
            return
        from mx_detect import explain
        found = [f for r in reports for f in r.found]
        if not found and any(r.cancelled for r in reports):
            self.statusbar.config(text="Auto-detect cancelled - click Auto-detect to scan again")
            return
        text = "\n\n".join(explain(r) for r in reports)
        if found:
            f = found[0]
            match = [v for v in self.port_cb["values"] if v.split(" - ")[0] == f.port]
            self.port_var.set(match[0] if match else f.port)
            self.baud_var.set(str(f.baud))
            self.addr_var.set(str(f.address))
            self.statusbar.config(text=f"Found controller: {f.port}, {f.baud} baud, address {f.address}")
            if messagebox.askyesno("Controller found", text + "\n\nSettings filled in. Connect now?"):
                self.connect()
        else:
            self.statusbar.config(text="Auto-detect: no controller found")
            messagebox.showwarning("Auto-detect", text)

    def connect(self):
        port = self.port_var.get().split(" - ")[0].strip()
        try:
            addr = int(self.addr_var.get())
            baud = int(self.baud_var.get())
            if port == "Simulator":
                self.drv = MXServeDriver("SIM", address=addr, serial_instance=FakeSerial(addr))
            else:
                self.drv = MXServeDriver(port, address=addr, baudrate=baud)
        except ImportError:
            messagebox.showerror("pyserial missing", "Install it with:\n\npip install pyserial")
            return
        except Exception as e:
            messagebox.showerror("Connect failed", str(e))
            self.drv = None
            return
        for m in (1, 2):
            self.drv.set_mode(m, Mode.SPEED)
            self.motor_vars[m]["enabled"].set(False)
            self.set_setpoint(m, 0)
            self.on_accel(m)
        self.drv.on_error = self._on_driver_error
        self.drv.on_status = self._on_driver_status
        self.error_count = self.status_count = 0
        self.history.clear()
        self.reset_energy()
        self.drv.start()
        self.connect_btn.config(text="Disconnect")
        self._set_controls_state(True)

    def disconnect(self):
        if self.drv:
            try:
                self.drv.close()          # disables both motors, then closes port
            except Exception:
                pass
        self.drv = None
        self.connect_btn.config(text="Connect")
        for m in (1, 2):
            self.motor_vars[m]["enabled"].set(False)
            self.motor_vars[m]["setpoint"].set(0)
            self.motor_vars[m]["scale"].set(0)
        self._set_controls_state(False)
        self.statusbar.config(text="Disconnected (motors disabled)")

    def _set_controls_state(self, on: bool):
        st = ["!disabled"] if on else ["disabled"]
        for m in (1, 2):
            v = self.motor_vars[m]
            for k in ("enable_cb", "minus", "plus", "scale", "zero"):
                v[k].state(st)

    # driver callbacks run on the heartbeat thread -> just count/stash
    def _on_driver_error(self, exc):
        self.error_count += 1

    def _on_driver_status(self, status: Status):
        self.status_count += 1

    # ------------------------------------------------------------ commands
    def max_rpm(self) -> int:
        try:
            return max(1, min(32767, int(float(self.limit_var.get()))))
        except (ValueError, AttributeError):
            return 1500

    def on_limit(self):
        lim = self.max_rpm()
        for m in (1, 2):
            self.motor_vars[m]["scale"].config(from_=-lim, to=lim)
            self.set_setpoint(m, self.motor_vars[m]["setpoint"].get())

    def set_setpoint(self, m: int, rpm: int):
        lim = self.max_rpm()
        rpm = max(-lim, min(lim, int(rpm)))
        v = self.motor_vars[m]
        v["setpoint"].set(rpm)
        self._scale_guard = True
        v["scale"].set(rpm)
        self._scale_guard = False
        if self.drv:
            self.drv.set_speed(m, rpm)

    def nudge(self, m: int, direction: int):
        if not self.drv:
            return
        targets = (1, 2) if self.link_var.get() else (m,)
        for t in targets:
            try:
                step = int(float(self.motor_vars[t]["step"].get()))
            except ValueError:
                step = 50
            self.set_setpoint(t, self.motor_vars[t]["setpoint"].get() + direction * step)

    def on_scale(self, m: int, val):
        if getattr(self, "_scale_guard", False) or not self.drv:
            return
        rpm = int(round(float(val) / 10) * 10)     # snap slider to 10 rpm
        if rpm != self.motor_vars[m]["setpoint"].get():
            self.set_setpoint(m, rpm)

    def on_enable(self, m: int):
        if self.drv:
            self.drv.enable(m, self.motor_vars[m]["enabled"].get())

    def on_accel(self, m: int):
        try:
            acc = max(1, int(float(self.motor_vars[m]["accel"].get())))
        except ValueError:
            acc = 200
        self.motor_vars[m]["accel"].set(str(acc))
        if self.drv:
            self.drv.set_accel(m, acc)

    def _on_escape(self, _e=None):
        if self._detect_running():
            self.cancel_detect()
        self.stop_all()

    def stop_all(self):
        for m in (1, 2):
            self.motor_vars[m]["enabled"].set(False)
            self.set_setpoint(m, 0)
        if self.drv:
            self.drv.stop_all()

    def reset_energy(self):
        self.energy_wh = {1: 0.0, 2: 0.0}
        self._last_power = None
        self.stat_vars["energy"][0].set("0.000 Wh")

    def _integrate_energy(self, t: float, p1: float, p2: float):
        """Trapezoidal V*I integration. Gaps > 1 s (link dropouts) are skipped
        rather than guessed at."""
        if self._last_power is not None:
            t0, q1, q2 = self._last_power
            dt = t - t0
            if 0 < dt <= 1.0:
                self.energy_wh[1] += (p1 + q1) / 2 * dt / 3600
                self.energy_wh[2] += (p2 + q2) / 2 * dt / 3600
        self._last_power = (t, p1, p2)

    # ---------------------------------------------------------------- log
    def toggle_log(self):
        if self.log_file:
            self.log_file.close()
            self.log_file = self.log_writer = None
            self.log_btn.config(text="Start CSV log")
            return
        path = filedialog.asksaveasfilename(defaultextension=".csv", filetypes=[("CSV", "*.csv")],
                                            initialfile=time.strftime("mxserve_%Y%m%d_%H%M%S.csv"))
        if not path:
            return
        self.log_file = open(path, "w", newline="")
        self.log_writer = csv.writer(self.log_file)
        self.log_writer.writerow(["time", "fault_code", "fault", "temp_c", "voltage_v",
                                  "m1_set_rpm", "m1_rpm", "m1_current_a", "m1_enabled",
                                  "m2_set_rpm", "m2_rpm", "m2_current_a", "m2_enabled",
                                  "m1_power_w", "m2_power_w", "total_power_w",
                                  "m1_energy_wh", "m2_energy_wh", "total_energy_wh"])
        self.log_btn.config(text="Stop CSV log ●")

    # ---------------------------------------------------------------- tick
    def _tick(self):
        try:
            self._poll_detect()
            self._update_ui()
        finally:
            self.after(POLL_UI_MS, self._tick)

    def _update_ui(self):
        drv = self.drv
        if not drv:
            return
        # Heartbeat may have force-disabled motors after repeated comm errors.
        for m in (1, 2):
            v = self.motor_vars[m]
            if v["enabled"].get() and not drv.motors[m].enabled:
                v["enabled"].set(False)
                v["setpoint"].set(drv.motors[m].setpoint)

        now = time.monotonic()
        if now - self._rate_t >= 1.0:
            self.poll_rate = (self.status_count - self._rate_n) / (now - self._rate_t)
            self._rate_t, self._rate_n = now, self.status_count

        st = drv.last_status
        fresh = st is not None and now - st.timestamp < 0.5
        if st:
            sv = self.stat_vars
            sv["fault"][0].set(f"{st.fault_code}: {st.fault}")
            sv["fault"][1].config(foreground="#1e8449" if st.ok else "#c0392b")
            sv["temp"][0].set(f"{st.temperature_c} °C")
            sv["volt"][0].set(f"{st.voltage_v:.1f} V")
            sv["m1rpm"][0].set(f"{st.motor1_rpm} rpm")
            sv["m2rpm"][0].set(f"{st.motor2_rpm} rpm")
            sv["m1a"][0].set(f"{st.motor1_current_a:.1f} A")
            sv["m2a"][0].set(f"{st.motor2_current_a:.1f} A")
            # Electrical power = bus voltage x reported motor current (negative = regen)
            p1 = st.voltage_v * st.motor1_current_a
            p2 = st.voltage_v * st.motor2_current_a
            sv["m1w"][0].set(f"{p1:.0f} W")
            sv["m2w"][0].set(f"{p2:.0f} W")
            sv["ptot"][0].set(f"{p1 + p2:.0f} W")
            for key in ("temp", "volt", "m1rpm", "m2rpm", "m1a", "m2a", "m1w", "m2w", "ptot"):
                sv[key][1].config(foreground="" if fresh else "#999")
            self.motor_vars[1]["actual"].set(f"actual {st.motor1_rpm} rpm")
            self.motor_vars[2]["actual"].set(f"actual {st.motor2_rpm} rpm")

            if not self.history or self.history[-1]["t"] != st.timestamp:
                self._integrate_energy(st.timestamp, p1, p2)
                e1, e2 = self.energy_wh[1], self.energy_wh[2]
                sv["energy"][0].set(f"{e1 + e2:.3f} Wh")
                sample = {"t": st.timestamp, "M1": st.motor1_rpm, "M2": st.motor2_rpm,
                          "M1 set": drv.motors[1].setpoint if drv.motors[1].enabled else 0,
                          "M2 set": drv.motors[2].setpoint if drv.motors[2].enabled else 0,
                          "M1 A": st.motor1_current_a, "M2 A": st.motor2_current_a,
                          "M1 W": p1, "M2 W": p2, "Total W": p1 + p2,
                          "M1 Wh": e1, "M2 Wh": e2, "Total Wh": e1 + e2,
                          "V": st.voltage_v, "°C": st.temperature_c}
                self.history.append(sample)
                if self.log_writer:
                    self.log_writer.writerow([
                        time.strftime("%Y-%m-%d %H:%M:%S"), st.fault_code, st.fault, st.temperature_c,
                        st.voltage_v, drv.motors[1].setpoint, st.motor1_rpm, st.motor1_current_a,
                        int(drv.motors[1].enabled), drv.motors[2].setpoint, st.motor2_rpm,
                        st.motor2_current_a, int(drv.motors[2].enabled),
                        round(p1, 1), round(p2, 1), round(p1 + p2, 1),
                        round(e1, 4), round(e2, 4), round(e1 + e2, 4)])
        while self.history and now - self.history[0]["t"] > HISTORY_S + 1:
            self.history.popleft()
        for ch in self.charts:
            ch.redraw(self.history)

        link = "OK" if fresh else "NO DATA"
        err = f"   last error: {drv.last_error}" if drv.last_error and not fresh else ""
        self.statusbar.config(
            text=f"Link {link}   |   {self.poll_rate:4.1f} polls/s   |   comm errors: {self.error_count}"
                 f"{'   |   LOGGING' if self.log_writer else ''}{err}")

    def on_close(self):
        self._detect_stop.set()
        self.disconnect()
        if self.log_file:
            self.log_file.close()
        self.destroy()


if __name__ == "__main__":
    App(start_sim="--sim" in sys.argv).mainloop()
