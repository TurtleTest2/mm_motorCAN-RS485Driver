"""
Derived statistics computed from the controller's status samples
(speed, voltage, temperature, fault) - no current measurement needed.

    stats = RunStats()
    stats.update(t, fault_code, fault_name, temp_c, voltage_v, rpm1, rpm2, set1, set2)

All times are time.monotonic() seconds.
"""

from __future__ import annotations

import math
import time
from collections import deque
from dataclasses import dataclass
from typing import Optional

EXPECTED_PERIOD_S = 0.100     # controller answers every command frame (100 ms)
DROPOUT_GAP_S = 0.25          # a gap longer than this counts as a dropout
MAX_INTEGRATE_GAP_S = 1.0     # don't integrate revolutions across longer gaps
ACCEL_WINDOW_S = 0.5          # measured acceleration = rpm change over this window
IDLE_RPM = 5                  # both motors below this (and commanded 0) = "at rest" for voltage sag


@dataclass
class MinMaxAvg:
    lo: float = math.inf
    hi: float = -math.inf
    total: float = 0.0
    n: int = 0

    def add(self, v: float):
        self.lo, self.hi = min(self.lo, v), max(self.hi, v)
        self.total += v
        self.n += 1

    @property
    def avg(self) -> Optional[float]:
        return self.total / self.n if self.n else None

    def fmt(self, unit: str, nd: int = 0) -> tuple[str, str, str]:
        if not self.n:
            return "—", "—", "—"
        f = lambda v: f"{v:.{nd}f} {unit}"
        return f(self.lo), f(self.avg), f(self.hi)


@dataclass
class FaultEvent:
    wall_start: str               # local clock time, for the log
    t_start: float
    code: int
    name: str
    t_end: Optional[float] = None

    @property
    def duration(self) -> Optional[float]:
        return None if self.t_end is None else self.t_end - self.t_start


class RunStats:
    def __init__(self):
        self.reset()

    def reset(self):
        self.t_first: Optional[float] = None
        self.t_last: Optional[float] = None
        self.samples = 0
        # link quality
        self.worst_gap = 0.0
        self.dropouts = 0
        self.missed = 0
        # revolutions (signed = net position, abs = total turned)
        self.revs = {1: 0.0, 2: 0.0}
        self.abs_revs = {1: 0.0, 2: 0.0}
        self._last_rpm = {1: 0.0, 2: 0.0}
        # measured acceleration
        self._accel_buf: deque = deque()
        self.accel = {1: 0.0, 2: 0.0}
        self.peak_accel = {1: 0.0, 2: 0.0}
        # speed error (setpoint - actual)
        self.error = {1: 0.0, 2: 0.0}
        # min / avg / max
        self.mm = {k: MinMaxAvg() for k in ("m1", "m2", "v", "temp", "err1", "err2")}
        # voltage sag vs. resting voltage
        self.rest_v: Optional[float] = None
        self.sag = 0.0
        self.max_sag = 0.0
        # faults
        self.faults: list[FaultEvent] = []
        self.fault_code = 0

    # ------------------------------------------------------------------
    def update(self, t: float, fault_code: int, fault_name: str, temp_c: float, voltage_v: float,
               rpm1: float, rpm2: float, set1: float, set2: float):
        """set1/set2 = commanded speed (0 when that motor is disabled)."""
        rpm = {1: rpm1, 2: rpm2}

        # link quality + revolutions (both need the previous sample)
        if self.t_last is not None:
            gap = t - self.t_last
            if gap <= 0:
                return                      # same sample twice
            self.worst_gap = max(self.worst_gap, gap)
            # frames that should have arrived inside this gap (tolerant of ~100 ms timer jitter)
            self.missed += max(0, int(round(gap / EXPECTED_PERIOD_S)) - 1)
            if gap > DROPOUT_GAP_S:
                self.dropouts += 1
            if gap <= MAX_INTEGRATE_GAP_S:
                for m in (1, 2):
                    turns = (rpm[m] + self._last_rpm[m]) / 2 / 60 * gap      # trapezoid
                    self.revs[m] += turns
                    self.abs_revs[m] += abs(turns)
        else:
            self.t_first = t
        self.t_last = t
        self.samples += 1
        self._last_rpm = dict(rpm)

        # measured acceleration over a short window (smooths the 1-rpm steps)
        self._accel_buf.append((t, rpm1, rpm2))
        while len(self._accel_buf) > 2 and t - self._accel_buf[0][0] > ACCEL_WINDOW_S:
            self._accel_buf.popleft()
        t0, a1, a2 = self._accel_buf[0]
        if t - t0 >= 0.2:
            self.accel = {1: (rpm1 - a1) / (t - t0), 2: (rpm2 - a2) / (t - t0)}
            for m in (1, 2):
                if abs(self.accel[m]) > abs(self.peak_accel[m]):
                    self.peak_accel[m] = self.accel[m]

        # speed error
        self.error = {1: set1 - rpm1, 2: set2 - rpm2}

        for k, v in (("m1", rpm1), ("m2", rpm2), ("v", voltage_v), ("temp", temp_c),
                     ("err1", self.error[1]), ("err2", self.error[2])):
            self.mm[k].add(v)

        # voltage sag: learn the resting voltage while nothing is moving
        if abs(rpm1) < IDLE_RPM and abs(rpm2) < IDLE_RPM and set1 == 0 and set2 == 0:
            self.rest_v = voltage_v if self.rest_v is None else 0.9 * self.rest_v + 0.1 * voltage_v
        if self.rest_v is not None:
            self.sag = max(0.0, self.rest_v - voltage_v)
            self.max_sag = max(self.max_sag, self.sag)

        # fault history: one event per fault, closed when the code changes
        if fault_code != self.fault_code:
            if self.faults and self.faults[-1].t_end is None:
                self.faults[-1].t_end = t
            if fault_code != 0:
                self.faults.append(FaultEvent(time.strftime("%H:%M:%S"), t, fault_code, fault_name))
            self.fault_code = fault_code

    # ------------------------------------------------------------------
    @property
    def elapsed(self) -> float:
        return 0.0 if self.t_first is None else self.t_last - self.t_first

    @property
    def expected_samples(self) -> int:
        return self.samples + self.missed

    @property
    def loss_pct(self) -> float:
        exp = self.expected_samples
        return 100.0 * self.missed / exp if exp else 0.0

    def distance_m(self, motor: int, wheel_diameter_mm: float) -> float:
        return self.revs[motor] * math.pi * wheel_diameter_mm / 1000.0


if __name__ == "__main__":
    s = RunStats()
    # 10 s at 10 Hz: M1 ramps 0->600 rpm at 300 rpm/s then holds; M2 holds -120 rpm.
    v_rest = 48.0
    for i in range(101):
        t = i * 0.1
        r1 = min(600.0, 300.0 * t)
        s.update(t, 0, "Normal", 30, v_rest - r1 / 600, r1, -120.0 if i else 0.0,
                 600 if i else 0, -120 if i else 0)          # first sample = at rest
    assert abs(s.revs[2] - (-120 / 60 * 9.95)) < 1e-9, s.revs          # 10 s, first 0.1 s ramps from 0
    expect1 = (300 * 2 ** 2 / 2 + 600 * 8) / 60                         # ramp area + hold area, in turns
    assert abs(s.revs[1] - expect1) < 0.05, (s.revs[1], expect1)
    assert abs(s.peak_accel[1] - 300) < 1, s.peak_accel
    assert s.error[1] == 0 and s.mm["err1"].hi == 570                  # 600 set vs 30 actual at t=0.1
    assert abs(s.max_sag - 1.0) < 0.01 and s.loss_pct == 0 and s.dropouts == 0
    # fault appears, clears; then a 0.5 s gap
    s.update(10.1, 25, "Motor 2 stall", 30, 47, 600, 0, 600, -120)
    s.update(10.2, 0, "Normal", 30, 47, 600, 0, 600, -120)
    s.update(10.7, 0, "Normal", 30, 47, 600, 0, 600, -120)
    assert len(s.faults) == 1 and abs(s.faults[0].duration - 0.1) < 1e-9
    assert s.dropouts == 1 and abs(s.worst_gap - 0.5) < 1e-9 and s.loss_pct > 0
    assert abs(s.distance_m(1, 100) - s.revs[1] * math.pi * 0.1) < 1e-9
    print("stats self-check OK:", {m: round(r, 2) for m, r in s.revs.items()}, "turns,",
          f"loss {s.loss_pct:.1f}%")
