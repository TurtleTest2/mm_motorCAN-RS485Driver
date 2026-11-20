"""
MX_CAN test panel - drive both motors over CAN and watch speed, voltage,
temperature and faults live. Protocol: MX_CAN-SV3.03-1 (see mx_can.py).

Run:
    pip install python-can pyserial
    python mx_can_gui.py            # choose adapter + channel in the window
    python mx_can_gui.py --sim      # no hardware: simulated controller

Keyboard (when not typing in a text box):
    D / A        Motor 1 speed + / -
    Right / Left Motor 2 speed + / -
    Space / Esc  STOP ALL (zero speed, disable both motors)

The CAN status frame has no motor current, so (unlike the RS485 panel) there
is no current / power / energy readout.
"""

from __future__ import annotations

import csv
import sys
import threading
import time
import tkinter as tk
from collections import deque
from tkinter import filedialog, messagebox, ttk

from mx_can import ADAPTERS, BITRATE, CMD_ID, STATUS_ID, MXCanDriver, open_bus
from mx_serve_gui import HISTORY_S, STEP_SIZES, StripChart
from mx_stats import RunStats

POLL_UI_MS = 100
BITRATES = ["125000", "250000", "500000", "1000000"]


class App(tk.Tk):
    def __init__(self, start_sim: bool = False):
        super().__init__()
        self.title("MX CAN Motor Control")
        sw, sh = self.winfo_screenwidth(), self.winfo_screenheight()
        w, h = min(1100, sw - 40), min(860, sh - 100)
        self.geometry(f"{w}x{h}+{max(0, (sw - w) // 2)}+10")
        self.minsize(min(950, w), min(600, h))
        style = ttk.Style(self)
        if "clam" in style.theme_names():
            style.theme_use("clam")
        style.configure("Big.TLabel", font=("Consolas", 22, "bold"))
        style.configure("Val.TLabel", font=("Consolas", 13))
        style.configure("Mono.TLabel", font=("Consolas", 10))
        style.configure("Stop.TButton", font=("Segoe UI", 14, "bold"), foreground="white", background="#c0392b")
        style.map("Stop.TButton", background=[("active", "#e74c3c")])

        self.drv: MXCanDriver | None = None
        self.history: deque = deque()
        self.log_file = self.log_writer = None
        self.error_count = 0
        self._rate_t, self._rate_n, self.rx_rate = time.monotonic(), 0, 0.0
        self.stats = RunStats()
        self.stats_lock = threading.Lock()         # stats are fed from the driver thread (every frame)
        self._faults_shown = None
        self._last_logged = None

        self._build_connection_bar()
        sb = ttk.Frame(self, relief="sunken")
        sb.pack(fill="x", side="bottom")
        self.statusbar = ttk.Label(sb, text="Disconnected", anchor="w", padding=(8, 3), font=("Segoe UI", 10))
        self.statusbar.pack(fill="x")
        body = ttk.Frame(self, padding=8)
        body.pack(fill="both", expand=True)
        top = ttk.Frame(body)
        top.pack(fill="x")
        self._build_status_panel(top)
        self._build_monitor(top)
        self.motor_vars = {}
        mf = ttk.Frame(body)
        mf.pack(fill="x", pady=6)
        for col, m in enumerate((1, 2)):
            self._build_motor_panel(mf, m).grid(row=0, column=col, sticky="nsew", padx=4)
            mf.columnconfigure(col, weight=1)
        self._build_global_bar(body)

        tabs = ttk.Notebook(body)
        tabs.pack(fill="both", expand=True, pady=(6, 0))
        charts = ttk.Frame(tabs)
        tabs.add(charts, text="  Charts  ")
        stats_tab = ttk.Frame(tabs, padding=8)
        tabs.add(stats_tab, text="  Statistics & faults  ")
        M1, M2 = "#4fc3f7", "#ffb74d"
        self.charts = [
            StripChart(charts, "Speed", "r/min",
                       [("M1", M1, False), ("M1 set", M1, True), ("M2", M2, False), ("M2 set", M2, True)]),
            StripChart(charts, "Speed error (set − actual)", "r/min",
                       [("M1 err", M1, False), ("M2 err", M2, False)], min_span=20),
            StripChart(charts, "Bus voltage", "V", [("V", "#ba68c8", False)], include_zero=False),
            StripChart(charts, "Controller temp", "°C", [("°C", "#e57373", False)],
                       include_zero=False, min_span=5),
        ]
        for i, ch in enumerate(self.charts):
            ch.grid(row=i // 2, column=i % 2, sticky="nsew", padx=3, pady=3)
        for c in (0, 1):
            charts.columnconfigure(c, weight=1)
        for r in (0, 1):
            charts.rowconfigure(r, weight=1)
        self._build_stats_tab(stats_tab)

        self._bind_keys()
        self._set_controls_state(False)
        self.protocol("WM_DELETE_WINDOW", self.on_close)
        if start_sim:
            self.adapter_var.set("Simulator")
            self.on_adapter()
            self.after(200, self.connect)
        self.after(POLL_UI_MS, self._tick)

    # ------------------------------------------------------------ building
    def _build_connection_bar(self):
        bar = ttk.Frame(self, padding=(8, 8, 8, 0))
        bar.pack(fill="x")
        ttk.Label(bar, text="Adapter").pack(side="left")
        self.adapter_var = tk.StringVar(value=list(ADAPTERS)[1])
        cb = ttk.Combobox(bar, textvariable=self.adapter_var, values=list(ADAPTERS), width=36, state="readonly")
        cb.pack(side="left", padx=(4, 10))
        cb.bind("<<ComboboxSelected>>", lambda e: self.on_adapter())
        ttk.Label(bar, text="Channel").pack(side="left")
        self.channel_var = tk.StringVar()
        self.channel_cb = ttk.Combobox(bar, textvariable=self.channel_var, width=14)
        self.channel_cb.pack(side="left", padx=(4, 2))
        ttk.Button(bar, text="↻", width=3, command=self.refresh_channels).pack(side="left", padx=(0, 10))
        ttk.Label(bar, text="Bitrate").pack(side="left")
        self.bitrate_var = tk.StringVar(value=str(BITRATE))
        ttk.Combobox(bar, textvariable=self.bitrate_var, values=BITRATES, width=8).pack(side="left", padx=(4, 10))
        self.connect_btn = ttk.Button(bar, text="Connect", command=self.toggle_connect)
        self.connect_btn.pack(side="left", padx=4)
        self.log_btn = ttk.Button(bar, text="Start CSV log", command=self.toggle_log)
        self.log_btn.pack(side="right")

        bar2 = ttk.Frame(self, padding=(8, 4, 8, 0))
        bar2.pack(fill="x")
        ttk.Label(bar2, text="Command ID (host → driver) 0x").pack(side="left")
        self.cmd_id_var = tk.StringVar(value=f"{CMD_ID:08X}")
        ttk.Entry(bar2, textvariable=self.cmd_id_var, width=10).pack(side="left", padx=(0, 12))
        ttk.Label(bar2, text="Status ID (driver → host) 0x").pack(side="left")
        self.status_id_var = tk.StringVar(value=f"{STATUS_ID:08X}")
        ttk.Entry(bar2, textvariable=self.status_id_var, width=10).pack(side="left", padx=(0, 12))
        ttk.Label(bar2, text="(extended frames, sent every 100 ms)", foreground="#777").pack(side="left")
        self.reset_btn = ttk.Button(bar2, text="Reset fault", command=self.reset_fault)
        self.reset_btn.pack(side="right")
        self.on_adapter()

    def _build_status_panel(self, parent):
        box = ttk.LabelFrame(parent, text="Live data (status frame)", padding=8)
        box.pack(side="left", fill="both", expand=True)
        self.stat_vars = {}
        fields = [[("fault", "Fault"), ("temp", "Controller temp"), ("volt", "Bus voltage")],
                  [("m1rpm", "M1 speed"), ("m2rpm", "M2 speed"), ("rate", "Status frames/s")]]
        for r, row in enumerate(fields):
            for i, (key, label) in enumerate(row):
                f = ttk.Frame(box)
                f.grid(row=r, column=i, padx=8, pady=(0 if r == 0 else 6, 0), sticky="w")
                ttk.Label(f, text=label, foreground="#666").pack(anchor="w")
                v = tk.StringVar(value="—")
                lbl = ttk.Label(f, textvariable=v, style="Val.TLabel")
                lbl.pack(anchor="w")
                self.stat_vars[key] = (v, lbl)
                box.columnconfigure(i, weight=1)

    def _build_monitor(self, parent):
        box = ttk.LabelFrame(parent, text="Bus monitor (raw bytes)", padding=8)
        box.pack(side="left", fill="both", padx=(8, 0))
        self.mon_tx = tk.StringVar(value="TX  —")
        self.mon_rx = tk.StringVar(value="RX  —")
        self.mon_other = tk.StringVar(value="Other IDs: none")
        self.mon_counts = tk.StringVar(value="sent 0   received 0")
        for var in (self.mon_tx, self.mon_rx, self.mon_counts, self.mon_other):
            ttk.Label(box, textvariable=var, style="Mono.TLabel", anchor="w", justify="left").pack(anchor="w")

    def _build_motor_panel(self, parent, m: int):
        box = ttk.LabelFrame(parent, text=f"Motor {m}" + ("   (keys D / A)" if m == 1 else "   (keys → / ←)"),
                             padding=10)
        v = {"enabled": tk.BooleanVar(value=False), "setpoint": tk.IntVar(value=0),
             "step": tk.StringVar(value="50"), "accel": tk.StringVar(value="500"),
             "actual": tk.StringVar(value="— rpm")}
        self.motor_vars[m] = v
        top = ttk.Frame(box)
        top.pack(fill="x")
        v["enable_cb"] = ttk.Checkbutton(top, text="Enabled", variable=v["enabled"], command=lambda: self.on_enable(m))
        v["enable_cb"].pack(side="left")
        ttk.Label(top, textvariable=v["actual"], foreground="#555").pack(side="right")
        v["info"] = tk.StringVar(value="error —   accel —   turns —")
        ttk.Label(box, textvariable=v["info"], foreground="#555", font=("Consolas", 9)).pack(anchor="w", pady=(4, 0))
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
        ttk.Label(bot, text="Accel (rpm/s, 10–2550)").pack(side="left")
        acc = ttk.Spinbox(bot, from_=10, to=2550, increment=50, textvariable=v["accel"], width=6,
                          command=lambda: self.on_accel(m))
        acc.pack(side="left", padx=4)
        acc.bind("<Return>", lambda e: self.on_accel(m))
        acc.bind("<FocusOut>", lambda e: self.on_accel(m))
        v["zero"] = ttk.Button(bot, text="Zero", command=lambda: self.set_setpoint(m, 0))
        v["zero"].pack(side="right")
        return box

    def _build_stats_tab(self, parent):
        left = ttk.Frame(parent)
        left.pack(side="left", fill="both", expand=True)
        head = ttk.Frame(left)
        head.pack(fill="x")
        ttk.Label(head, text="Since connect (or last reset)", foreground="#666").pack(side="left")
        ttk.Button(head, text="Reset statistics", command=self.reset_stats).pack(side="right")

        self.mm_tree = ttk.Treeview(left, columns=("min", "avg", "max"), height=6)
        self.mm_tree.heading("#0", text="")
        self.mm_tree.column("#0", width=150, stretch=False)
        for c, t in (("min", "Min"), ("avg", "Average"), ("max", "Max")):
            self.mm_tree.heading(c, text=t)
            self.mm_tree.column(c, width=100, anchor="e")
        self._mm_rows = [("m1", "M1 speed", "rpm", 0), ("m2", "M2 speed", "rpm", 0),
                         ("err1", "M1 speed error", "rpm", 0), ("err2", "M2 speed error", "rpm", 0),
                         ("v", "Bus voltage", "V", 1), ("temp", "Controller temp", "°C", 0)]
        for key, label, _, _ in self._mm_rows:
            self.mm_tree.insert("", "end", iid=key, text=label, values=("—", "—", "—"))
        self.mm_tree.pack(fill="x", pady=(4, 8))

        grid = ttk.Frame(left)
        grid.pack(fill="x")
        self.sv2 = {}
        items = [("turns1", "M1 turns (net / total)"), ("turns2", "M2 turns (net / total)"),
                 ("dist", "Distance M1 / M2"), ("peak", "Peak accel M1 / M2"),
                 ("rest", "Resting voltage"), ("sag", "Voltage sag now / max"),
                 ("link", "Status frames (got / expected)"), ("loss", "Lost frames / dropouts"),
                 ("gap", "Worst gap between frames"), ("time", "Time connected")]
        for i, (key, label) in enumerate(items):
            r, c = divmod(i, 2)
            ttk.Label(grid, text=label, foreground="#666").grid(row=r, column=2 * c, sticky="w", padx=(0, 6), pady=1)
            self.sv2[key] = tk.StringVar(value="—")
            ttk.Label(grid, textvariable=self.sv2[key], font=("Consolas", 10)).grid(
                row=r, column=2 * c + 1, sticky="w", padx=(0, 24), pady=1)
        wheel = ttk.Frame(left)
        wheel.pack(fill="x", pady=(6, 0))
        ttk.Label(wheel, text="Wheel diameter for distance (mm, 0 = off)").pack(side="left")
        self.wheel_var = tk.StringVar(value="0")
        ttk.Entry(wheel, textvariable=self.wheel_var, width=7).pack(side="left", padx=4)

        right = ttk.LabelFrame(parent, text="Fault history", padding=6)
        right.pack(side="left", fill="both", padx=(10, 0))
        self.fault_tree = ttk.Treeview(right, columns=("time", "code", "fault", "dur"), show="headings", height=9)
        for c, t, w in (("time", "Time", 70), ("code", "Code", 45), ("fault", "Fault", 190), ("dur", "Lasted", 70)):
            self.fault_tree.heading(c, text=t)
            self.fault_tree.column(c, width=w, anchor="w" if c in ("fault", "time") else "e")
        self.fault_tree.pack(fill="both", expand=True)

    def reset_stats(self):
        with self.stats_lock:
            self.stats.reset()
        self._faults_shown = None
        self._refresh_stats()

    def _on_driver_status(self, st):
        """Driver thread, every status frame: feed the statistics so none are missed."""
        drv = self.drv
        if drv is None:
            return
        m1, m2 = drv.motors[1], drv.motors[2]
        with self.stats_lock:
            self.stats.update(st.timestamp, st.fault_code, st.fault, st.temperature_c, st.voltage_v,
                              st.motor1_rpm, st.motor2_rpm,
                              m1.setpoint if m1.enabled else 0, m2.setpoint if m2.enabled else 0)

    def _wheel_mm(self) -> float:
        try:
            return max(0.0, float(self.wheel_var.get()))
        except ValueError:
            return 0.0

    def _refresh_stats(self):
        with self.stats_lock:
            self._refresh_stats_locked()

    def _refresh_stats_locked(self):
        s = self.stats
        for key, _, unit, nd in self._mm_rows:
            self.mm_tree.item(key, values=s.mm[key].fmt(unit, nd))
        for m in (1, 2):
            self.sv2[f"turns{m}"].set(f"{s.revs[m]:+.1f} / {s.abs_revs[m]:.1f}")
            self.motor_vars[m]["info"].set(
                f"error {s.error[m]:+5.0f} rpm   accel {s.accel[m]:+6.0f} rpm/s   turns {s.revs[m]:+.1f}")
        d = self._wheel_mm()
        self.sv2["dist"].set(f"{s.distance_m(1, d):.2f} m / {s.distance_m(2, d):.2f} m" if d else "set wheel size ↓")
        self.sv2["peak"].set(f"{s.peak_accel[1]:+.0f} / {s.peak_accel[2]:+.0f} rpm/s")
        self.sv2["rest"].set(f"{s.rest_v:.1f} V" if s.rest_v is not None else "— (stop motors to learn)")
        self.sv2["sag"].set(f"{s.sag:.2f} / {s.max_sag:.2f} V")
        self.sv2["link"].set(f"{s.samples} / {s.expected_samples}")
        self.sv2["loss"].set(f"{s.loss_pct:.1f} %  /  {s.dropouts}")
        self.sv2["gap"].set(f"{s.worst_gap * 1000:.0f} ms")
        el = int(s.elapsed)
        self.sv2["time"].set(f"{el // 3600}:{el // 60 % 60:02d}:{el % 60:02d}")
        shown = [(f.wall_start, f.code, f.duration) for f in s.faults]
        if shown != self._faults_shown:            # only rebuild the list when something changed
            self._faults_shown = shown
            self.fault_tree.delete(*self.fault_tree.get_children())
            for f in reversed(s.faults):
                dur = "ongoing" if f.duration is None else f"{f.duration:.1f} s"
                self.fault_tree.insert("", "end", values=(f.wall_start, f.code, f.name, dur))

    def _build_global_bar(self, parent):
        bar = ttk.Frame(parent)
        bar.pack(fill="x", pady=(2, 0))
        ttk.Button(bar, text="■  STOP ALL  (Space)", style="Stop.TButton", command=self.stop_all).pack(
            side="left", ipadx=10, ipady=4)
        self.link_var = tk.BooleanVar(value=False)
        ttk.Checkbutton(bar, text="Link motors (D/A, →/← and +/− move both)",
                        variable=self.link_var).pack(side="left", padx=16)
        ttk.Label(bar, text="Speed limit ±").pack(side="left", padx=(16, 2))
        self.limit_var = tk.StringVar(value="1500")
        lim = ttk.Spinbox(bar, from_=10, to=30000, increment=100, textvariable=self.limit_var, width=7,
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
                    return
                fn()
                return "break"
            return h

        def release_on_click(e):
            try:
                focused = self.focus_get()
            except KeyError:
                return
            if not isinstance(e.widget, text_boxes) and isinstance(focused, text_boxes):
                self.focus_set()
        self.bind_all("<Button-1>", release_on_click, add="+")
        for cls in ("TSpinbox", "TCombobox", "TEntry"):
            self.bind_class(cls, "<Return>", lambda e: self.after_idle(self.focus_set), add="+")
        for key in ("d", "D"):
            self.bind(f"<KeyPress-{key}>", guard(lambda: self.nudge(1, +1)))
        for key in ("a", "A"):
            self.bind(f"<KeyPress-{key}>", guard(lambda: self.nudge(1, -1)))
        self.bind("<Right>", guard(lambda: self.nudge(2, +1)))
        self.bind("<Left>", guard(lambda: self.nudge(2, -1)))
        self.bind("<space>", guard(self.stop_all))
        self.bind("<Escape>", lambda e: self.stop_all())

    # ---------------------------------------------------------- connection
    def on_adapter(self):
        iface, default_ch, _ = ADAPTERS[self.adapter_var.get()]
        if iface not in ("seeedstudio", "slcan"):
            self.channel_var.set(default_ch)
        self.refresh_channels()             # COM-port adapters: picks the first COM port found
        if not self.channel_var.get():
            self.channel_var.set(default_ch)

    def refresh_channels(self):
        iface = ADAPTERS[self.adapter_var.get()][0]
        values = []
        if iface in ("seeedstudio", "slcan"):
            try:
                from serial.tools import list_ports
                values = [p.device for p in list_ports.comports()]
            except ImportError:
                pass
            if values and self.channel_var.get() not in values:
                self.channel_var.set(values[0])
        elif iface == "socketcan":
            values = ["can0", "can1"]
        elif iface == "pcan":
            values = [f"PCAN_USBBUS{i}" for i in range(1, 9)]
        elif iface not in ("sim", "usb2can"):
            values = [str(i) for i in range(4)]
        self.channel_cb["values"] = values

    def toggle_connect(self):
        self.disconnect() if self.drv else self.connect()

    def connect(self):
        label = self.adapter_var.get()
        iface, _, extra = ADAPTERS[label]
        try:
            cmd_id = int(self.cmd_id_var.get(), 16)
            status_id = int(self.status_id_var.get(), 16)
            bitrate = int(self.bitrate_var.get())
        except ValueError:
            messagebox.showerror("Bad setting", "IDs must be hex (e.g. 1801E600) and bitrate a number.")
            return
        self.statusbar.config(text=f"Opening {label} {self.channel_var.get()}…")
        self.update_idletasks()
        try:
            if iface == "sim":
                from mx_can import FakeBus
                bus = FakeBus(cmd_id, status_id)
            else:
                bus = open_bus(iface, self.channel_var.get().strip(), bitrate, **extra)
        except ImportError as e:
            messagebox.showerror("Missing package", f"{e}\n\nInstall with:\n\npip install python-can pyserial")
            self.statusbar.config(text="Disconnected")
            return
        except Exception as e:
            messagebox.showerror("Could not open CAN adapter",
                                 f"{type(e).__name__}: {e}\n\nCheck the adapter type, channel (COM port / "
                                 "PCAN_USBBUS1 / 0) and that its driver is installed and no other "
                                 "program (e.g. the adapter's own software) has it open.")
            self.statusbar.config(text="Disconnected")
            return
        self.reset_stats()
        self.drv = MXCanDriver(bus, cmd_id, status_id)
        self.drv.on_error = self._on_driver_error
        self.drv.on_status = self._on_driver_status
        for m in (1, 2):
            self.motor_vars[m]["enabled"].set(False)
            self.set_setpoint(m, 0)
            self.on_accel(m)
        self.error_count = 0
        self.history.clear()
        self._rate_t, self._rate_n = time.monotonic(), 0
        self.drv.start()
        self.connect_btn.config(text="Disconnect")
        self._set_controls_state(True)

    def disconnect(self):
        if self.drv:
            try:
                self.drv.close()            # sends "both disabled", then releases the adapter
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
            for k in ("enable_cb", "minus", "plus", "scale", "zero"):
                self.motor_vars[m][k].state(st)
        self.reset_btn.state(st)

    def _on_driver_error(self, exc):
        self.error_count += 1

    # ------------------------------------------------------------ commands
    def max_rpm(self) -> int:
        try:
            return max(1, min(30000, int(float(self.limit_var.get()))))
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
        for t in ((1, 2) if self.link_var.get() else (m,)):
            try:
                step = int(float(self.motor_vars[t]["step"].get()))
            except ValueError:
                step = 50
            self.set_setpoint(t, self.motor_vars[t]["setpoint"].get() + direction * step)

    def on_scale(self, m: int, val):
        if getattr(self, "_scale_guard", False) or not self.drv:
            return
        rpm = int(round(float(val) / 10) * 10)
        if rpm != self.motor_vars[m]["setpoint"].get():
            self.set_setpoint(m, rpm)

    def on_enable(self, m: int):
        if self.drv:
            self.drv.enable(m, self.motor_vars[m]["enabled"].get())

    def on_accel(self, m: int):
        try:
            acc = int(float(self.motor_vars[m]["accel"].get()))
        except ValueError:
            acc = 500
        acc = max(10, min(2550, round(acc / 10) * 10))     # protocol carries 10 rpm/s steps
        self.motor_vars[m]["accel"].set(str(acc))
        if self.drv:
            self.drv.set_accel_rpm_s(m, acc)

    def reset_fault(self):
        if self.drv:
            self.drv.pulse_reset()

    def stop_all(self):
        for m in (1, 2):
            self.motor_vars[m]["enabled"].set(False)
            self.set_setpoint(m, 0)
        if self.drv:
            self.drv.stop_all()

    # ---------------------------------------------------------------- log
    def toggle_log(self):
        if self.log_file:
            self.log_file.close()
            self.log_file = self.log_writer = None
            self.log_btn.config(text="Start CSV log")
            return
        path = filedialog.asksaveasfilename(defaultextension=".csv", filetypes=[("CSV", "*.csv")],
                                            initialfile=time.strftime("mxcan_%Y%m%d_%H%M%S.csv"))
        if not path:
            return
        self.log_file = open(path, "w", newline="")
        self.log_writer = csv.writer(self.log_file)
        self.log_writer.writerow(["time", "fault_code", "fault", "temp_c", "voltage_v",
                                  "m1_set_rpm", "m1_rpm", "m1_enabled", "m2_set_rpm", "m2_rpm", "m2_enabled",
                                  "m1_error_rpm", "m2_error_rpm", "m1_accel_rpm_s", "m2_accel_rpm_s",
                                  "m1_turns", "m2_turns", "m1_distance_m", "m2_distance_m",
                                  "voltage_sag_v", "raw_status"])
        self.log_btn.config(text="Stop CSV log ●")

    # ---------------------------------------------------------------- tick
    def _tick(self):
        try:
            self._update_ui()
        finally:
            self.after(POLL_UI_MS, self._tick)

    def _update_ui(self):
        drv = self.drv
        if not drv:
            return
        for m in (1, 2):                         # driver may force-disable after repeated send errors
            v = self.motor_vars[m]
            if v["enabled"].get() and not drv.motors[m].enabled:
                v["enabled"].set(False)
                v["setpoint"].set(drv.motors[m].setpoint)
        now = time.monotonic()
        if now - self._rate_t >= 1.0:
            self.rx_rate = (drv.status_count - self._rate_n) / (now - self._rate_t)
            self._rate_t, self._rate_n = now, drv.status_count

        hexs = lambda b: " ".join(f"{x:02X}" for x in b) if b else "—"
        self.mon_tx.set(f"TX {drv.cmd_id:08X}  {hexs(drv.last_tx)}")
        st = drv.last_status
        self.mon_rx.set(f"RX {drv.status_id:08X}  {hexs(st.raw) if st else '— (nothing yet)'}")
        self.mon_counts.set(f"sent {drv.tx_count}   received {drv.rx_count}   send errors {self.error_count}")
        others = sorted(drv.other_ids.items(), key=lambda kv: -kv[1][0])[:3]
        self.mon_other.set("Other IDs: " + ("none" if not others else "\n  ".join(
            f"{i:08X}{'' if ext else ' (std)'} ×{n}  {hexs(d)}" for i, (n, d, ext) in others)))

        fresh = st is not None and now - st.timestamp < 0.5
        sv = self.stat_vars
        sv["rate"][0].set(f"{self.rx_rate:4.1f} /s")
        if st:
            sv["fault"][0].set(f"{st.fault_code}: {st.fault}")
            sv["fault"][1].config(foreground="#1e8449" if st.ok else "#c0392b")
            sv["temp"][0].set(f"{st.temperature_c} °C")
            sv["volt"][0].set(f"{st.voltage_v:.1f} V")
            sv["m1rpm"][0].set(f"{st.motor1_rpm} rpm")
            sv["m2rpm"][0].set(f"{st.motor2_rpm} rpm")
            for key in ("temp", "volt", "m1rpm", "m2rpm"):
                sv[key][1].config(foreground="" if fresh else "#999")
            self.motor_vars[1]["actual"].set(f"actual {st.motor1_rpm} rpm")
            self.motor_vars[2]["actual"].set(f"actual {st.motor2_rpm} rpm")
            if not self.history or self.history[-1]["t"] != st.timestamp:
                m1, m2 = drv.motors[1], drv.motors[2]
                set1 = m1.setpoint if m1.enabled else 0
                set2 = m2.setpoint if m2.enabled else 0
                s = self.stats
                self.history.append({"t": st.timestamp, "M1": st.motor1_rpm, "M2": st.motor2_rpm,
                                     "M1 set": set1, "M2 set": set2,
                                     "M1 err": set1 - st.motor1_rpm, "M2 err": set2 - st.motor2_rpm,
                                     "V": st.voltage_v, "°C": st.temperature_c})
                if self.log_writer:
                    d = self._wheel_mm()
                    with self.stats_lock:
                        self.log_writer.writerow([
                            time.strftime("%Y-%m-%d %H:%M:%S"), st.fault_code, st.fault, st.temperature_c,
                            st.voltage_v, m1.setpoint, st.motor1_rpm, int(m1.enabled),
                            m2.setpoint, st.motor2_rpm, int(m2.enabled),
                            round(s.error[1]), round(s.error[2]), round(s.accel[1]), round(s.accel[2]),
                            round(s.revs[1], 3), round(s.revs[2], 3),
                            round(s.distance_m(1, d), 3), round(s.distance_m(2, d), 3),
                            round(s.sag, 2), hexs(st.raw)])
                self._refresh_stats()
        while self.history and now - self.history[0]["t"] > HISTORY_S + 1:
            self.history.popleft()
        for ch in self.charts:
            ch.redraw(self.history)

        if fresh:
            link = "Link OK"
        elif drv.tx_count and not drv.rx_count:
            link = "NO REPLY - sending, but nothing received (check CANH/CANL, 120 Ω termination, bitrate)"
        elif drv.rx_count and not drv.status_count:
            link = "Receiving other IDs but no status frame - check the Status ID"
        else:
            link = "NO DATA"
        err = f"   |   last error: {drv.last_error}" if drv.last_error and not fresh else ""
        self.statusbar.config(text=f"{link}   |   {self.rx_rate:4.1f} status frames/s"
                                   f"{'   |   LOGGING' if self.log_writer else ''}{err}")

    def on_close(self):
        self.disconnect()
        if self.log_file:
            self.log_file.close()
        self.destroy()


if __name__ == "__main__":
    App(start_sim="--sim" in sys.argv).mainloop()
