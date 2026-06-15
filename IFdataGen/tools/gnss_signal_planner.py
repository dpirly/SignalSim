#!/usr/bin/env python3
"""GNSS frequency planning GUI for SignalSim IF data configurations.

The tool is intentionally self-contained and uses only tkinter.  It follows the
same working idea as the LabSat 3 Wideband frequency setup page: choose GNSS
signals, inspect the RF coverage, then decide how many RF bands are required.
"""

from __future__ import annotations

import json
import calendar
import math
import re
import tkinter as tk
from dataclasses import dataclass
from pathlib import Path
from tkinter import filedialog, messagebox, ttk
from typing import Dict, Iterable, List, Sequence


@dataclass(frozen=True)
class Signal:
    key: str
    constellation: str
    name: str
    band: str
    center_mhz: float
    bandwidth_mhz: float
    group_hint: str
    note: str = ""
    range_low_mhz: float | None = None
    range_high_mhz: float | None = None

    @property
    def low_mhz(self) -> float:
        if self.range_low_mhz is not None:
            return self.range_low_mhz - self.bandwidth_mhz / 2.0
        return self.center_mhz - self.bandwidth_mhz / 2.0

    @property
    def high_mhz(self) -> float:
        if self.range_high_mhz is not None:
            return self.range_high_mhz + self.bandwidth_mhz / 2.0
        return self.center_mhz + self.bandwidth_mhz / 2.0

    @property
    def display_center(self) -> str:
        if self.range_low_mhz is None:
            return f"{self.center_mhz:.3f}"
        return f"{self.range_low_mhz:.3f}..{self.range_high_mhz:.3f}"


@dataclass
class BandPlan:
    name: str
    signals: List[Signal]

    @property
    def low_mhz(self) -> float:
        return min(s.low_mhz for s in self.signals)

    @property
    def high_mhz(self) -> float:
        return max(s.high_mhz for s in self.signals)

    @property
    def center_mhz(self) -> float:
        return (self.low_mhz + self.high_mhz) / 2.0

    @property
    def span_mhz(self) -> float:
        return self.high_mhz - self.low_mhz


SIGNALS: Sequence[Signal] = (
    Signal("gps_l1ca", "GPS", "L1 C/A", "L1", 1575.420, 2.046, "L1"),
    Signal("gps_l1c", "GPS", "L1C", "L1", 1575.420, 4.092, "L1"),
    Signal("gps_l2c", "GPS", "L2C", "L2", 1227.600, 2.046, "L2"),
    Signal("gps_l5", "GPS", "L5", "L5", 1176.450, 20.460, "L5"),
    Signal("gal_e1", "Galileo", "E1", "E1", 1575.420, 4.092, "L1"),
    Signal("gal_e5a", "Galileo", "E5a", "E5a", 1176.450, 20.460, "L5"),
    Signal("gal_e5b", "Galileo", "E5b", "E5b", 1207.140, 20.460, "L2"),
    Signal("gal_e6", "Galileo", "E6", "E6", 1278.750, 40.920, "E6"),
    Signal("bds_b1i", "BeiDou", "B1I", "B1", 1561.098, 4.092, "L1"),
    Signal("bds_b1c", "BeiDou", "B1C", "B1", 1575.420, 4.092, "L1"),
    Signal("bds_b2i", "BeiDou", "B2I", "B2", 1207.140, 4.092, "L2"),
    Signal("bds_b2a", "BeiDou", "B2a", "B2", 1176.450, 20.460, "L5"),
    Signal("bds_b2b", "BeiDou", "B2b", "B2", 1207.140, 20.460, "L2"),
    Signal("bds_b3i", "BeiDou", "B3I", "B3", 1268.520, 20.460, "E6"),
    Signal(
        "glo_g1",
        "GLONASS",
        "G1/L1OF",
        "G1",
        1601.719,
        1.000,
        "L1",
        "FDMA k=-7..+6 full range",
        1598.0625,
        1605.3750,
    ),
    Signal(
        "glo_g2",
        "GLONASS",
        "G2/L2OF",
        "G2",
        1245.781,
        0.800,
        "L2",
        "FDMA k=-7..+6 full range",
        1242.9375,
        1248.6250,
    ),
)

CONSTELLATION_ORDER = ("GPS", "Galileo", "BeiDou", "GLONASS")
GROUP_ORDER = {"L1": 0, "L2": 1, "L5": 2, "E6": 3}
SIGNAL_ORDER = {signal.key: idx for idx, signal in enumerate(SIGNALS)}
COLORS = {
    "GPS": "#2b7de9",
    "Galileo": "#8e44ad",
    "BeiDou": "#1f9d55",
    "GLONASS": "#c0392b",
}

SYSTEM_EXPORT_NAMES = {
    "GPS": "GPS",
    "Galileo": "Galileo",
    "BeiDou": "BDS",
    "GLONASS": "GLONASS",
}

SIGNAL_EXPORT_NAMES = {
    "gps_l1ca": "L1CA",
    "gps_l1c": "L1C",
    "gps_l2c": "L2C",
    "gps_l5": "L5",
    "gal_e1": "E1",
    "gal_e5a": "E5a",
    "gal_e5b": "E5b",
    "gal_e6": "E6",
    "bds_b1i": "B1I",
    "bds_b1c": "B1C",
    "bds_b2i": "B2I",
    "bds_b2a": "B2a",
    "bds_b2b": "B2b",
    "bds_b3i": "B3I",
    "glo_g1": "G1",
    "glo_g2": "G2",
}
SIGNAL_IMPORT_KEYS = {
    (SYSTEM_EXPORT_NAMES[signal.constellation], SIGNAL_EXPORT_NAMES[signal.key]): signal.key
    for signal in SIGNALS
}

CONFIG_DIR = Path(__file__).resolve().parents[1] / "configs"


def round_up(value: float, step: float) -> float:
    return math.ceil(value / step - 1e-9) * step


def plan_ls4_sample_rates(required_mhz: Sequence[float], step_mhz: float) -> tuple[float, List[float], List[int]]:
    if not required_mhz:
        return 0.0, [], []

    min_base = round_up(max(required_mhz), step_mhz)
    # Searching to 4x the largest required rate is enough to expose useful
    # integer dividers without encouraging unrealistic high base rates.
    max_base = round_up(max(required_mhz) * 4.0, step_mhz)
    step_count = max(0, int(round((max_base - min_base) / step_mhz)))
    best: tuple[float, float, List[float], List[int]] | None = None

    for idx in range(step_count + 1):
        base = round(min_base + idx * step_mhz, 6)
        path_rates: List[float] = []
        dividers: List[int] = []
        valid = True
        for required in required_mhz:
            divider = max(1, int(math.floor(base / required + 1e-9)))
            if divider <= 0:
                valid = False
                break
            rate = base / divider
            if rate + 1e-9 < required:
                valid = False
                break
            path_rates.append(rate)
            dividers.append(divider)
        if not valid:
            continue
        total_rate = sum(path_rates)
        candidate = (total_rate, base, path_rates, dividers)
        if best is None or candidate[0] < best[0] - 1e-9 or (
            abs(candidate[0] - best[0]) <= 1e-9 and candidate[1] < best[1]
        ):
            best = candidate

    if best is None:
        return min_base, [min_base for _ in required_mhz], [1 for _ in required_mhz]
    return best[1], best[2], best[3]


class ToolTip:
    def __init__(self, widget: tk.Widget, text: str) -> None:
        self.widget = widget
        self.text = text
        self.window: tk.Toplevel | None = None
        widget.bind("<Enter>", self._show)
        widget.bind("<Leave>", self._hide)
        widget.bind("<ButtonPress>", self._hide)

    def _show(self, _event: tk.Event) -> None:
        if self.window is not None:
            return
        x = self.widget.winfo_rootx() + 18
        y = self.widget.winfo_rooty() + self.widget.winfo_height() + 4
        self.window = tk.Toplevel(self.widget)
        self.window.wm_overrideredirect(True)
        self.window.wm_geometry(f"+{x}+{y}")
        label = ttk.Label(
            self.window,
            text=self.text,
            justify=tk.LEFT,
            background="#ffffe8",
            relief=tk.SOLID,
            borderwidth=1,
            padding=(6, 4),
        )
        label.pack()

    def _hide(self, _event: tk.Event | None = None) -> None:
        if self.window is not None:
            self.window.destroy()
            self.window = None


def group_signals(signals: Iterable[Signal], max_bands: int) -> List[BandPlan]:
    grouped: Dict[str, List[Signal]] = {}
    for signal in signals:
        grouped.setdefault(signal.group_hint, []).append(signal)

    bands = [
        BandPlan(name, sorted(items, key=lambda s: SIGNAL_ORDER[s.key]))
        for name, items in grouped.items()
    ]
    bands.sort(key=lambda b: (GROUP_ORDER.get(b.name, 99), b.low_mhz))

    if max_bands <= 0 or len(bands) <= max_bands:
        return bands

    while len(bands) > max_bands:
        best_idx = 0
        best_span = None
        for idx in range(len(bands) - 1):
            low = min(bands[idx].low_mhz, bands[idx + 1].low_mhz)
            high = max(bands[idx].high_mhz, bands[idx + 1].high_mhz)
            span = high - low
            if best_span is None or span < best_span:
                best_span = span
                best_idx = idx
        merged = BandPlan(
            f"{bands[best_idx].name}+{bands[best_idx + 1].name}",
            bands[best_idx].signals + bands[best_idx + 1].signals,
        )
        bands[best_idx : best_idx + 2] = [merged]
    return bands


class GnssPlanner(tk.Tk):
    def __init__(self) -> None:
        super().__init__()
        self.title("GNSS Signal Frequency Planner")
        width = min(1600, max(1280, self.winfo_screenwidth() - 80))
        height = min(1040, max(960, self.winfo_screenheight() - 90))
        self.geometry(f"{width}x{height}")
        self.minsize(1280, 760)

        self.signal_vars: Dict[str, tk.BooleanVar] = {}
        self.max_bands = 3
        self.usable_ratio = tk.DoubleVar(value=0.80)
        self.quant_bits = tk.IntVar(value=3)
        self.duration_s = tk.DoubleVar(value=180.0)
        self.round_step = tk.DoubleVar(value=0.5)
        self.year = tk.IntVar(value=2021)
        self.month = tk.IntVar(value=6)
        self.day = tk.IntVar(value=19)
        self.hour = tk.IntVar(value=0)
        self.minute = tk.IntVar(value=0)
        self.second = tk.IntVar(value=0)
        self.date_text = tk.StringVar(value="")
        self.longitude = tk.DoubleVar(value=116.471006)
        self.latitude = tk.DoubleVar(value=40.019450)
        self.altitude = tk.DoubleVar(value=20.0)
        self.output_format = tk.StringVar(value="LS3W")
        self.channels_used_text = tk.StringVar(value="0")
        self.parameter_spins: List[tuple[ttk.Spinbox, tk.Variable, float, float, float]] = []
        self.current_bands: List[BandPlan] = []
        self.current_band_fs_mhz: List[float] = []
        self.current_band_bw_div: List[int] = []
        self.current_sample_freq_mhz = 0.0

        self._build_ui()
        self._sync_date_text()
        self.update_view()

    def _build_ui(self) -> None:
        self.columnconfigure(1, weight=1)
        self.rowconfigure(0, weight=1)

        left = ttk.Frame(self, padding=10)
        left.grid(row=0, column=0, sticky="ns")
        left.rowconfigure(2, weight=1)

        ttk.Label(left, text="Signal Selection", font=("TkDefaultFont", 11, "bold")).grid(
            row=0, column=0, sticky="w"
        )
        ttk.Button(left, text="Clear", command=self._clear_selection).grid(
            row=1, column=0, sticky="ew", pady=(8, 8)
        )

        selector = ttk.Frame(left)
        selector.grid(row=2, column=0, sticky="nsew")
        selector.columnconfigure(0, weight=1)
        self._build_signal_selector(selector)

        setup = ttk.LabelFrame(left, text="Planning Parameters", padding=8)
        setup.grid(row=3, column=0, sticky="ew", pady=(10, 0))
        setup.columnconfigure(0, weight=1)
        setup.columnconfigure(1, weight=1)

        output_frame = ttk.LabelFrame(setup, text="Output", padding=6)
        output_frame.grid(row=0, column=0, sticky="new", padx=(0, 6))
        scenario_frame = ttk.LabelFrame(setup, text="Scenario", padding=6)
        scenario_frame.grid(row=0, column=1, sticky="new", padx=(6, 0))

        ttk.Label(output_frame, text="Channels used").grid(row=0, column=0, sticky="w", pady=2)
        ttk.Label(output_frame, textvariable=self.channels_used_text).grid(row=0, column=1, sticky="e", padx=(8, 0), pady=2)
        self._spin(output_frame, "Usable BW ratio", self.usable_ratio, 0.50, 0.95, 1, 0.01)
        self._spin(output_frame, "Quant bits", self.quant_bits, 1, 4, 2)
        self._spin(output_frame, "Duration s", self.duration_s, 1.0, 3600.0, 3, 1.0)
        self._spin(output_frame, "Round Fs MHz", self.round_step, 0.1, 10.0, 4, 0.1)
        ttk.Label(output_frame, text="Output format").grid(row=5, column=0, sticky="w", pady=2)
        format_box = ttk.Combobox(
            output_frame,
            textvariable=self.output_format,
            values=("LS3W", "WAVE", "LS4"),
            state="readonly",
            width=8,
        )
        format_box.grid(row=5, column=1, sticky="e", padx=(8, 0), pady=2)
        format_box.bind("<<ComboboxSelected>>", lambda _event: self.update_view())

        self._date_control(scenario_frame, 0)
        self._time_control(scenario_frame, 1)
        self._spin(scenario_frame, "Longitude deg", self.longitude, -180.0, 180.0, 2, 0.000001)
        self._spin(scenario_frame, "Latitude deg", self.latitude, -90.0, 90.0, 3, 0.000001)
        self._spin(scenario_frame, "Altitude m", self.altitude, -1000.0, 20000.0, 4, 1.0)

        ttk.Button(left, text="Load Config JSON", command=self._load_config_json).grid(
            row=4, column=0, sticky="ew", pady=(10, 0)
        )
        ttk.Button(left, text="Generate Config JSON", command=self._save_config_json).grid(
            row=5, column=0, sticky="ew", pady=(6, 0)
        )

        right = ttk.Frame(self, padding=(0, 10, 10, 10))
        right.grid(row=0, column=1, sticky="nsew")
        right.columnconfigure(0, weight=1)
        right.rowconfigure(3, weight=1)

        title = ttk.Label(
            right,
            text="LabSat-style RF channel planner",
            font=("TkDefaultFont", 12, "bold"),
        )
        title.grid(row=0, column=0, sticky="w")
        note = ttk.Label(
            right,
            text=(
                "Bandwidth values describe useful planning bandwidth, not an absolute spectral mask. "
                "Recommended Fs = required RF span / usable bandwidth ratio."
            ),
        )
        note.grid(row=1, column=0, sticky="w", pady=(2, 8))

        chart_frame = ttk.Frame(right)
        chart_frame.grid(row=2, column=0, sticky="ew", pady=(0, 8))
        for idx in range(3):
            chart_frame.columnconfigure(idx, weight=1, uniform="channel_chart")
        self.channel_canvases: List[tk.Canvas] = []
        for idx in range(3):
            canvas = tk.Canvas(
                chart_frame,
                height=210,
                bg="white",
                highlightthickness=1,
                highlightbackground="#b0b0b0",
            )
            canvas.grid(row=0, column=idx, sticky="ew", padx=(0 if idx == 0 else 6, 0))
            canvas.bind("<Configure>", lambda _event: self.update_view())
            self.channel_canvases.append(canvas)

        tables = ttk.PanedWindow(right, orient=tk.VERTICAL)
        tables.grid(row=3, column=0, sticky="nsew")

        band_frame = ttk.LabelFrame(tables, text="Channel Summary", padding=6)
        signal_frame = ttk.LabelFrame(tables, text="Selected Signals", padding=6)
        tables.add(band_frame, weight=1)
        tables.add(signal_frame, weight=1)

        self.band_tree = ttk.Treeview(
            band_frame,
            columns=("channel", "center", "range", "span", "fs", "rate", "signals"),
            show="headings",
            height=6,
        )
        self._setup_tree(
            self.band_tree,
            {
                "channel": ("Channel", 70),
                "center": ("Center MHz", 100),
                "range": ("RF range MHz", 180),
                "span": ("Span MHz", 80),
                "fs": ("Fs MHz", 80),
                "rate": ("Data MB/s", 95),
                "signals": ("Signals", 430),
            },
        )
        self.band_tree.pack(fill="both", expand=True)

        self.signal_tree = ttk.Treeview(
            signal_frame,
            columns=("constellation", "signal", "center", "bw", "edge", "offset", "note"),
            show="headings",
            height=9,
        )
        self._setup_tree(
            self.signal_tree,
            {
                "constellation": ("Constellation", 95),
                "signal": ("Signal", 90),
                "center": ("Center MHz", 110),
                "bw": ("BW MHz", 80),
                "edge": ("RF edges MHz", 170),
                "offset": ("IF offset MHz", 105),
                "note": ("Note", 240),
            },
        )
        self.signal_tree.pack(fill="both", expand=True)

        self.status = ttk.Label(right, text="", anchor="w")
        self.status.grid(row=4, column=0, sticky="ew", pady=(8, 0))

    def _spin(
        self,
        parent: ttk.Frame,
        label: str,
        var: tk.Variable,
        low: float,
        high: float,
        row: int,
        inc: float = 1.0,
    ) -> None:
        ttk.Label(parent, text=label).grid(row=row, column=0, sticky="w", pady=2)
        spin = ttk.Spinbox(
            parent,
            textvariable=var,
            from_=low,
            to=high,
            increment=inc,
            width=self._spin_width(low, high, inc),
        )
        spin.configure(command=lambda: self._commit_spin(spin, var, low, high, inc, update=True))
        spin.grid(row=row, column=1, sticky="e", padx=(8, 0), pady=2)
        spin.bind("<Return>", lambda _event: self._commit_spin(spin, var, low, high, inc, update=True))
        spin.bind("<FocusOut>", lambda _event: self._commit_spin(spin, var, low, high, inc, update=True))
        self.parameter_spins.append((spin, var, low, high, inc))

    def _date_control(self, parent: ttk.Frame, row: int) -> None:
        ttk.Label(parent, text="UTC date").grid(row=row, column=0, sticky="w", pady=2)
        frame = ttk.Frame(parent)
        frame.grid(row=row, column=1, sticky="e", padx=(8, 0), pady=2)
        ttk.Label(frame, textvariable=self.date_text, width=10).grid(row=0, column=0, sticky="e")
        ttk.Button(frame, text="...", width=3, command=self._show_calendar).grid(row=0, column=1, padx=(4, 0))

    def _time_control(self, parent: ttk.Frame, row: int) -> None:
        ttk.Label(parent, text="UTC time").grid(row=row, column=0, sticky="w", pady=2)
        frame = ttk.Frame(parent)
        frame.grid(row=row, column=1, sticky="e", padx=(8, 0), pady=2)
        self._time_spin(frame, self.hour, 0, 23, 0)
        ttk.Label(frame, text=":").grid(row=0, column=1)
        self._time_spin(frame, self.minute, 0, 59, 2)
        ttk.Label(frame, text=":").grid(row=0, column=3)
        self._time_spin(frame, self.second, 0, 59, 4)

    def _time_spin(self, parent: ttk.Frame, var: tk.IntVar, low: int, high: int, column: int) -> None:
        spin = ttk.Spinbox(parent, textvariable=var, from_=low, to=high, increment=1, width=2)
        spin.configure(command=lambda: self._commit_spin(spin, var, low, high, 1, update=True))
        spin.grid(row=0, column=column)
        spin.bind("<Return>", lambda _event: self._commit_spin(spin, var, low, high, 1, update=True))
        spin.bind("<FocusOut>", lambda _event: self._commit_spin(spin, var, low, high, 1, update=True))
        self.parameter_spins.append((spin, var, low, high, 1))

    def _sync_date_text(self) -> None:
        self._clamp_date()
        self.date_text.set(f"{self.year.get():04d}-{self.month.get():02d}-{self.day.get():02d}")

    def _clamp_date(self) -> None:
        year = min(max(int(self.year.get()), 1980), 2099)
        month = min(max(int(self.month.get()), 1), 12)
        max_day = calendar.monthrange(year, month)[1]
        day = min(max(int(self.day.get()), 1), max_day)
        self.year.set(year)
        self.month.set(month)
        self.day.set(day)

    def _show_calendar(self) -> None:
        self._sync_date_text()
        popup = tk.Toplevel(self)
        popup.title("Select UTC Date")
        popup.resizable(False, False)
        popup.transient(self)
        popup.grab_set()

        view_year = tk.IntVar(value=self.year.get())
        view_month = tk.IntVar(value=self.month.get())
        title = tk.StringVar()
        body = ttk.Frame(popup, padding=8)
        body.grid(row=0, column=0)

        def redraw() -> None:
            for child in body.winfo_children():
                child.destroy()
            year = view_year.get()
            month = view_month.get()
            title.set(f"{year:04d}-{month:02d}")
            ttk.Button(body, text="<", width=3, command=lambda: shift_month(-1)).grid(row=0, column=0)
            ttk.Label(body, textvariable=title, width=12, anchor="center").grid(row=0, column=1, columnspan=5)
            ttk.Button(body, text=">", width=3, command=lambda: shift_month(1)).grid(row=0, column=6)
            for col, name in enumerate(("Mon", "Tue", "Wed", "Thu", "Fri", "Sat", "Sun")):
                ttk.Label(body, text=name, width=4, anchor="center").grid(row=1, column=col, pady=(6, 2))
            for r, week in enumerate(calendar.monthcalendar(year, month), start=2):
                for c, day in enumerate(week):
                    if day == 0:
                        ttk.Label(body, text="", width=4).grid(row=r, column=c)
                        continue
                    ttk.Button(
                        body,
                        text=str(day),
                        width=4,
                        command=lambda d=day: choose_day(d),
                    ).grid(row=r, column=c, padx=1, pady=1)

        def shift_month(delta: int) -> None:
            year = view_year.get()
            month = view_month.get() + delta
            if month < 1:
                year -= 1
                month = 12
            elif month > 12:
                year += 1
                month = 1
            year = min(max(year, 1980), 2099)
            view_year.set(year)
            view_month.set(month)
            redraw()

        def choose_day(day: int) -> None:
            self.year.set(view_year.get())
            self.month.set(view_month.get())
            self.day.set(day)
            self._sync_date_text()
            popup.destroy()
            self.update_view()

        redraw()

    def _commit_spin(
        self,
        spin: ttk.Spinbox,
        var: tk.Variable,
        low: float,
        high: float,
        inc: float,
        update: bool,
    ) -> None:
        raw = spin.get().strip()
        try:
            value = float(raw)
        except ValueError:
            value = low
        value = min(max(value, low), high)
        if type(var) is tk.IntVar:
            value = int(round(value))
            value = int(min(max(value, int(low)), int(high)))
            var.set(value)
        else:
            decimals = self._decimal_places(inc)
            value = round(value, decimals)
            var.set(value)
        if update:
            self.update_view()

    def _decimal_places(self, inc: float) -> int:
        if inc >= 1:
            return 0
        text = f"{inc:.12f}".rstrip("0")
        if "." not in text:
            return 0
        return len(text.split(".", 1)[1])

    def _spin_width(self, low: float, high: float, inc: float) -> int:
        decimals = self._decimal_places(inc)
        if decimals >= 6:
            return 12
        low_text = f"{low:.{decimals}f}" if decimals else f"{int(low)}"
        high_text = f"{high:.{decimals}f}" if decimals else f"{int(high)}"
        return max(8, min(16, max(len(low_text), len(high_text)) + 1))

    def _validate_parameters(self) -> None:
        if not hasattr(self, "parameter_spins"):
            return
        for spin, var, low, high, inc in self.parameter_spins:
            self._commit_spin(spin, var, low, high, inc, update=False)
        self._sync_date_text()

    def _build_signal_selector(self, parent: ttk.Frame) -> None:
        canvas = tk.Canvas(parent, width=390, highlightthickness=0)
        scrollbar = ttk.Scrollbar(parent, orient=tk.VERTICAL, command=canvas.yview)
        content = ttk.Frame(canvas)
        content.columnconfigure(0, weight=1)
        content_window = canvas.create_window((0, 0), window=content, anchor="nw")
        content.bind(
            "<Configure>",
            lambda _event: canvas.configure(scrollregion=canvas.bbox("all")),
        )
        canvas.bind("<Configure>", lambda event: canvas.itemconfigure(content_window, width=event.width))
        canvas.configure(yscrollcommand=scrollbar.set)
        canvas.grid(row=0, column=0, sticky="nsew")
        scrollbar.grid(row=0, column=1, sticky="ns")
        parent.rowconfigure(0, weight=1)
        parent.columnconfigure(0, weight=1)
        canvas.bind_all("<MouseWheel>", lambda event: canvas.yview_scroll(int(-event.delta / 120), "units"))

        by_constellation: Dict[str, List[Signal]] = {}
        for signal in SIGNALS:
            by_constellation.setdefault(signal.constellation, []).append(signal)

        row = 0
        for constellation in CONSTELLATION_ORDER:
            frame = ttk.LabelFrame(content, text=constellation, padding=6)
            frame.grid(row=row, column=0, sticky="ew", pady=4)
            frame.columnconfigure(0, weight=1)
            frame.columnconfigure(1, weight=1)
            row += 1
            signals = sorted(by_constellation.get(constellation, []), key=lambda s: (s.band, s.name))
            by_band: Dict[str, List[Signal]] = {}
            for signal in signals:
                by_band.setdefault(signal.band, []).append(signal)
            band_items = sorted(by_band.items(), key=lambda item: (GROUP_ORDER.get(item[0], 99), item[0]))
            for band_idx, (band, band_signals) in enumerate(band_items):
                column = band_idx % 2
                band_row = band_idx // 2
                band_frame = ttk.Frame(frame)
                band_frame.grid(
                    row=band_row,
                    column=column,
                    sticky="new",
                    padx=(0 if column == 0 else 12, 6 if column == 0 else 0),
                    pady=(0, 8),
                )
                band_frame.columnconfigure(0, weight=1)
                ttk.Label(band_frame, text=band, font=("TkDefaultFont", 9, "bold")).grid(
                    row=0, column=0, sticky="w", pady=(0, 2)
                )
                for signal in band_signals:
                    var = tk.BooleanVar(value=False)
                    self.signal_vars[signal.key] = var
                    cb = ttk.Checkbutton(
                        band_frame,
                        text=signal.name,
                        variable=var,
                        command=self.update_view,
                    )
                    cb.grid(row=band_frame.grid_size()[1], column=0, sticky="w")
                    ToolTip(cb, self._signal_tooltip(signal))

    def _signal_tooltip(self, signal: Signal) -> str:
        lines = [
            f"{signal.constellation} {signal.name}",
            f"Band: {signal.band}",
            f"Center: {signal.display_center} MHz",
            f"Bandwidth: {signal.bandwidth_mhz:.3f} MHz",
            f"RF edges: {signal.low_mhz:.3f} .. {signal.high_mhz:.3f} MHz",
            f"Default channel group: {signal.group_hint}",
        ]
        if signal.note:
            lines.append(f"Note: {signal.note}")
        return "\n".join(lines)

    def _setup_tree(self, tree: ttk.Treeview, columns: Dict[str, tuple[str, int]]) -> None:
        for column, (heading, width) in columns.items():
            tree.heading(column, text=heading)
            tree.column(column, width=width, anchor="w", stretch=True)

    def _selected_signals(self) -> List[Signal]:
        return [s for s in SIGNALS if self.signal_vars.get(s.key, tk.BooleanVar()).get()]

    def _clear_selection(self) -> None:
        for var in self.signal_vars.values():
            var.set(False)
        self.update_view()

    def update_view(self) -> None:
        if not hasattr(self, "band_tree"):
            return
        self._validate_parameters()
        selected = self._selected_signals()
        bands = group_signals(selected, self.max_bands) if selected else []
        self.current_bands = bands
        self.channels_used_text.set(str(len(bands)))

        for tree in (self.band_tree, self.signal_tree):
            tree.delete(*tree.get_children())

        usable = max(0.01, float(self.usable_ratio.get()))
        quant_bits = max(1, int(self.quant_bits.get()))
        duration_s = max(0.0, float(self.duration_s.get()))
        step = max(0.01, float(self.round_step.get()))
        fmt = self.output_format.get().upper()

        fs_values: List[float] = []
        band_by_signal: Dict[str, BandPlan] = {}
        for idx, band in enumerate(bands, start=1):
            fs_mhz = round_up(band.span_mhz / usable, step)
            fs_values.append(fs_mhz)
        packed_sample_freq_mhz = max(fs_values) if fs_values else 0.0
        if fmt == "LS4":
            base_sample_freq_mhz, output_fs_values, bw_divs = plan_ls4_sample_rates(fs_values, step)
        else:
            base_sample_freq_mhz = packed_sample_freq_mhz
            output_fs_values = [packed_sample_freq_mhz for _ in fs_values]
            bw_divs = [1 for _ in fs_values]
        self.current_band_fs_mhz = output_fs_values
        self.current_band_bw_div = bw_divs
        self.current_sample_freq_mhz = base_sample_freq_mhz

        for idx, band in enumerate(bands, start=1):
            output_fs_mhz = output_fs_values[idx - 1]
            data_mbs = output_fs_mhz * 1_000_000.0 * 2.0 * quant_bits / 8.0 / 1_000_000.0
            for signal in band.signals:
                band_by_signal[signal.key] = band
            signal_names = ", ".join(f"{s.constellation} {s.name}" for s in band.signals)
            self.band_tree.insert(
                "",
                "end",
                values=(
                    f"CH{idx}",
                    f"{band.center_mhz:.3f}",
                    f"{band.low_mhz:.3f}..{band.high_mhz:.3f}",
                    f"{band.span_mhz:.3f}",
                    f"{output_fs_mhz:.3f}",
                    f"{data_mbs:.1f}",
                    signal_names,
                ),
            )

        total_sample_rate_mhz = sum(output_fs_values)
        total_rate = total_sample_rate_mhz * 1_000_000.0 * 2.0 * quant_bits / 8.0 / 1_000_000.0

        for signal in selected:
            band = band_by_signal.get(signal.key)
            offset = signal.center_mhz - band.center_mhz if band else 0.0
            self.signal_tree.insert(
                "",
                "end",
                values=(
                    signal.constellation,
                    signal.name,
                    signal.display_center,
                    f"{signal.bandwidth_mhz:.3f}",
                    f"{signal.low_mhz:.3f}..{signal.high_mhz:.3f}",
                    f"{offset:+.3f}" if band else "",
                    signal.note,
                ),
            )

        size_gb = total_rate * duration_s / 1000.0
        self.status.config(
            text=(
                f"Selected {len(selected)} signals, {len(bands)} RF channels, "
                f"{fmt} base Fs {base_sample_freq_mhz:.3f} MHz, estimated raw IQ rate {total_rate:.1f} MB/s, "
                f"{duration_s:.0f}s file {size_gb:.2f} GB."
            )
        )
        self._draw_charts(bands)

    def _save_config_json(self) -> None:
        self._validate_parameters()
        self.update_view()
        if not self.current_bands:
            messagebox.showerror("Generate Config JSON", "Select at least one signal first.")
            return

        config = self._build_config_json()
        initial_name = self._default_config_filename()
        path = filedialog.asksaveasfilename(
            title="Save SignalSim config",
            initialdir=str(CONFIG_DIR),
            initialfile=initial_name,
            defaultextension=".json",
            filetypes=(("JSON files", "*.json"), ("All files", "*.*")),
        )
        if not path:
            return
        with open(path, "w", encoding="utf-8") as fp:
            json.dump(config, fp, indent=2)
            fp.write("\n")
        messagebox.showinfo("Generate Config JSON", f"Saved:\n{path}")

    def _load_config_json(self) -> None:
        path = filedialog.askopenfilename(
            title="Load SignalSim config",
            initialdir=str(CONFIG_DIR),
            filetypes=(("JSON files", "*.json"), ("All files", "*.*")),
        )
        if not path:
            return
        try:
            with open(path, "r", encoding="utf-8") as fp:
                config = json.load(fp)
            unsupported = self._apply_config_json(config)
        except Exception as exc:
            messagebox.showerror("Load Config JSON", f"Failed to load config:\n{exc}")
            return

        if unsupported:
            messagebox.showwarning(
                "Load Config JSON",
                "Loaded supported fields, but these signals are not available in this tool:\n"
                + "\n".join(unsupported),
            )

    def _apply_config_json(self, config: Dict[str, object]) -> List[str]:
        output = self._dict_value(config, "output")
        trajectory = self._dict_value(config, "trajectory")
        time_cfg = self._dict_value(config, "time")

        fmt = str(output.get("format", "LS3W")).upper()
        if fmt not in ("LS3W", "WAVE", "LS4"):
            raise ValueError(f"unsupported output.format: {fmt}")
        self.output_format.set(fmt)

        if "quantBits" in output:
            self.quant_bits.set(int(output["quantBits"]))

        self.year.set(int(time_cfg.get("year", self.year.get())))
        self.month.set(int(time_cfg.get("month", self.month.get())))
        self.day.set(int(time_cfg.get("day", self.day.get())))
        self.hour.set(int(time_cfg.get("hour", self.hour.get())))
        self.minute.set(int(time_cfg.get("minute", self.minute.get())))
        self.second.set(int(time_cfg.get("second", self.second.get())))

        init_position = trajectory.get("initPosition", {})
        if isinstance(init_position, dict):
            self.longitude.set(float(init_position.get("longitude", self.longitude.get())))
            self.latitude.set(float(init_position.get("latitude", self.latitude.get())))
            self.altitude.set(float(init_position.get("altitude", self.altitude.get())))

        trajectory_list = trajectory.get("trajectoryList", [])
        if isinstance(trajectory_list, list) and trajectory_list:
            first_item = trajectory_list[0]
            if isinstance(first_item, dict) and "time" in first_item:
                self.duration_s.set(float(first_item["time"]))

        sample_freq = float(output.get("sampleFreq", 0.0))
        paths = output.get("ls4Paths" if fmt == "LS4" else "ls3wPaths", [])
        if not paths:
            paths = output.get("paths", [])
        if not isinstance(paths, list):
            raise ValueError("output path list must be a list")

        ratio_values: List[float] = []
        selected_keys = set()
        unsupported: List[str] = []
        for path_item in paths:
            if not isinstance(path_item, dict):
                continue
            bandwidth = float(path_item.get("bandwidth", 0.0))
            path_sample_freq = float(path_item.get("sampleFreq", sample_freq))
            if bandwidth > 0.0 and path_sample_freq > 0.0:
                ratio_values.append(bandwidth / path_sample_freq)
            system_select = path_item.get("systemSelect", [])
            if not isinstance(system_select, list):
                continue
            for item in system_select:
                if not isinstance(item, dict) or not item.get("enable", True):
                    continue
                system = str(item.get("system", ""))
                signal = str(item.get("signal", ""))
                key = SIGNAL_IMPORT_KEYS.get((system, signal))
                if key is None:
                    unsupported.append(f"{system} {signal}".strip())
                else:
                    selected_keys.add(key)

        if ratio_values:
            self.usable_ratio.set(min(max(sum(ratio_values) / len(ratio_values), 0.50), 0.95))

        for key, var in self.signal_vars.items():
            var.set(key in selected_keys)

        self._validate_parameters()
        self.update_view()
        return sorted(set(unsupported))

    def _dict_value(self, config: Dict[str, object], key: str) -> Dict[str, object]:
        value = config.get(key)
        if not isinstance(value, dict):
            raise ValueError(f"{key} must be an object")
        return value

    def _build_config_json(self) -> Dict[str, object]:
        fmt = self.output_format.get().upper()
        duration_s = float(self.duration_s.get())
        usable = max(0.01, float(self.usable_ratio.get()))
        quant_bits = max(1, int(self.quant_bits.get()))
        signal_part = self._signal_name_part()
        packed_sample_freq_mhz = self.current_sample_freq_mhz
        sample_freq_mhz = packed_sample_freq_mhz
        fs_part = self._mhz_name(sample_freq_mhz)
        channel_part = f"{len(self.current_bands)}ch"
        file_ext = "wave" if fmt == "WAVE" else fmt
        output_name = f"{signal_part}_{channel_part}_{fs_part}_{duration_s:g}s_{quant_bits}bit.{file_ext}"

        paths = []
        for idx, band in enumerate(self.current_bands):
            path_sample_freq_mhz = self.current_band_fs_mhz[idx] if fmt == "LS4" else sample_freq_mhz
            path = {
                "centerFreq": round(band.center_mhz, 6),
                "bandwidth": round(path_sample_freq_mhz * usable, 6),
                "systemSelect": [
                    {
                        "system": SYSTEM_EXPORT_NAMES[signal.constellation],
                        "signal": SIGNAL_EXPORT_NAMES[signal.key],
                        "enable": True,
                    }
                    for signal in band.signals
                ],
            }
            if fmt == "LS4":
                path["sampleFreq"] = round(path_sample_freq_mhz, 6)
            paths.append(path)

        path_key = "ls4Paths" if fmt == "LS4" else "ls3wPaths"

        return {
            "version": 1.0,
            "description": f"{fmt} output: {signal_part.replace('_', ' ')} in {len(paths)} RF channels",
            "time": {
                "type": "UTC",
                "year": int(self.year.get()),
                "month": int(self.month.get()),
                "day": int(self.day.get()),
                "hour": int(self.hour.get()),
                "minute": int(self.minute.get()),
                "second": int(self.second.get()),
            },
            "trajectory": {
                "name": self._config_text(signal_part).lower(),
                "initPosition": {
                    "type": "LLA",
                    "format": "d",
                    "longitude": float(self.longitude.get()),
                    "latitude": float(self.latitude.get()),
                    "altitude": float(self.altitude.get()),
                },
                "initVelocity": {
                    "type": "SCU",
                    "speed": 0,
                    "course": 0,
                },
                "trajectoryList": [
                    {
                        "type": "Const",
                        "time": duration_s,
                    }
                ],
            },
            "ephemeris": {
                "type": "RINEX",
                "name": "../EphData/BRDC00IGS_R_20211700000_01D_MN.rnx",
            },
            "output": {
                "type": "IFdata",
                "format": fmt,
                "sampleFreq": sample_freq_mhz,
                "name": output_name,
                "quantBits": quant_bits,
                path_key: paths,
                "config": {
                    "elevationMask": 3,
                },
            },
            "power": {
                "noiseFloor": -172,
                "initPower": {
                    "unit": "dBHz",
                    "value": 60,
                },
                "elevationAdjust": False,
            },
        }

    def _default_config_filename(self) -> str:
        fmt = self.output_format.get().upper()
        signal_part = self._signal_name_part()
        fs_part = self._mhz_name(self.current_sample_freq_mhz)
        return f"{fmt}_{signal_part}_{len(self.current_bands)}ch_{fs_part}.json"

    def _signal_name_part(self) -> str:
        selected = self._selected_signals()
        parts = []
        for signal in selected:
            system = SYSTEM_EXPORT_NAMES[signal.constellation]
            parts.append(f"{system}_{SIGNAL_EXPORT_NAMES[signal.key]}")
        text = "_".join(parts) if parts else "GNSS"
        return self._config_text(text)

    def _config_text(self, text: str) -> str:
        return re.sub(r"_+", "_", re.sub(r"[^A-Za-z0-9]+", "_", text)).strip("_")

    def _mhz_name(self, value: float) -> str:
        text = f"{value:g}".replace(".", "p")
        return f"{text}m"

    def _draw_charts(self, bands: List[BandPlan]) -> None:
        for idx, canvas in enumerate(self.channel_canvases):
            band = bands[idx] if idx < len(bands) else None
            self._draw_channel_chart(canvas, idx + 1, band)

    def _draw_channel_chart(self, canvas: tk.Canvas, channel_idx: int, band: BandPlan | None) -> None:
        canvas.delete("all")
        width = max(1, canvas.winfo_width())
        height = max(1, canvas.winfo_height())

        left_px = 48
        right_px = width - 18
        top_px = 34
        bottom_px = height - 30
        canvas.create_text(
            left_px,
            16,
            text=f"CH{channel_idx}",
            anchor="w",
            font=("TkDefaultFont", 10, "bold"),
            fill="#222222",
        )

        if band is None:
            canvas.create_rectangle(left_px, top_px, right_px, bottom_px, outline="#dddddd", fill="#fafafa")
            canvas.create_text(width / 2, height / 2, text="unused", fill="#999999", font=("TkDefaultFont", 10))
            return

        low = band.low_mhz
        high = band.high_mhz
        margin_mhz = max(1.0, (high - low) * 0.12)
        low -= margin_mhz
        high += margin_mhz
        span = max(1e-6, high - low)

        def x_at(freq_mhz: float) -> float:
            return left_px + (freq_mhz - low) / span * (right_px - left_px)

        canvas.create_rectangle(left_px, top_px, right_px, bottom_px, outline="#dddddd", fill="#fafafa")
        canvas.create_text(
            right_px,
            16,
            text=f"{band.center_mhz:.3f} MHz / {band.span_mhz:.3f} MHz",
            anchor="e",
            font=("TkDefaultFont", 8),
            fill="#333333",
        )
        canvas.create_line(left_px, bottom_px, right_px, bottom_px, fill="#333333")
        tick_step = self._choose_tick_step(span)
        first_tick = math.ceil(low / tick_step) * tick_step
        tick = first_tick
        while tick <= high:
            x = x_at(tick)
            canvas.create_line(x, bottom_px, x, bottom_px + 5, fill="#333333")
            canvas.create_text(x, bottom_px + 16, text=f"{tick:g}", font=("TkDefaultFont", 8))
            canvas.create_line(x, top_px, x, bottom_px, fill="#eeeeee")
            tick += tick_step
        canvas.create_text(left_px, bottom_px + 16, text="MHz", anchor="w", fill="#555555", font=("TkDefaultFont", 8))

        canvas.create_rectangle(
            x_at(band.low_mhz),
            top_px + 8,
            x_at(band.high_mhz),
            top_px + 28,
            outline="#444444",
            fill="#f0f0f0",
        )
        signal_top = top_px + 40
        signal_bottom = bottom_px - 8
        lane_height = max(12, min(22, (signal_bottom - signal_top) / max(1, len(band.signals))))
        for sig_idx, signal in enumerate(band.signals):
            y = signal_top + sig_idx * lane_height
            color = COLORS.get(signal.constellation, "#666666")
            canvas.create_rectangle(
                x_at(signal.low_mhz),
                y,
                x_at(signal.high_mhz),
                y + lane_height - 4,
                outline=color,
                fill=color,
                stipple="gray25",
            )
            label_x = min(max(x_at(signal.low_mhz) + 4, left_px + 4), right_px - 105)
            canvas.create_text(
                label_x,
                y + (lane_height - 4) / 2,
                text=f"{signal.constellation[:3]} {signal.name}",
                anchor="w",
                font=("TkDefaultFont", 8),
                fill="#222222",
            )

    def _choose_tick_step(self, span_mhz: float) -> float:
        for step in (1, 2, 5, 10, 20, 50, 100):
            if span_mhz / step <= 12:
                return float(step)
        return 200.0


def main() -> None:
    app = GnssPlanner()
    app.mainloop()


if __name__ == "__main__":
    main()
