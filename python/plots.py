"""Modular plotting panels for the breath monitor.

Each panel is a self-contained object with `setup(ax)` and `update(state)`.
To add a new chart: implement a Panel subclass and append it to LAYOUT below.
To remove one: delete its tuple from LAYOUT. Nothing else needs to change.

Every panel renders all radar targets (3 slots); the `selected` slot is
highlighted and `state.show_target` controls which targets are drawn.
"""

import numpy as np
import matplotlib
import matplotlib.patches as patches
import matplotlib.widgets as widgets

import analysis


def scipy_gaussian(arr, sigma=1.8):
    try:
        from scipy.ndimage import gaussian_filter
        return gaussian_filter(arr, sigma=sigma)
    except Exception:
        return arr


THEME = {
    "fig_bg": "#eef1f5",
    "panel_bg": "#fbfcfe",
    "title": "#2c3e50",
    "grid": 0.5,
}


class Panel:
    """Base class. Subclasses may override setup() and update()."""

    def setup(self, ax):
        self.ax = ax

    def update(self, state):
        pass


class HeaderPanel(Panel):
    """Top status bar: title + live system/algorithm badges."""

    def setup(self, ax):
        self.ax = ax
        ax.set_xticks([])
        ax.set_yticks([])
        ax.patch.set_visible(True)
        ax.set_facecolor("#e2e8f0")
        self.title = ax.text(0.01, 0.5, "HLK-LD2450 Breath Monitor",
                             fontsize=16, fontweight='bold', va='center',
                             color=THEME["title"], transform=ax.transAxes)
        self.status = ax.text(0.99, 0.5, "", fontsize=11, family='monospace',
                              ha='right', va='center', transform=ax.transAxes)

    def update(self, state):
        with state.lock:
            visible = state.visible
            in_zone = state.in_zone
            bpm = state.bpm
            quality = state.quality
            apnea = state.apnea
        tracking = "TRACKING" if in_zone and visible else ("VISIBLE" if visible else "NO TARGET")
        self.status.set_text(
            f"STATE: {tracking}    PY BPM: {bpm:>4.1f}    QUALITY: {quality:>5.1f}%"
            f"    APNEA: {'DETECTED' if apnea else 'OK'}"
        )
        self.status.set_color("#c0392b" if apnea else "#27ae60")


class WavePlot(Panel):
    """Radial distance over time for every target."""

    def setup(self, ax):
        self.ax = ax
        ax.set_facecolor(THEME["panel_bg"])
        ax.set_title("Distance (radial) vs time", fontsize=11, color=THEME["title"], loc='left')
        self.lines = [
            ax.plot([], [], lw=1.0, color=analysis.TARGET_COLORS[i],
                    label=f"Target {i}") [0]
            for i in range(3)
        ]
        self.trend, = ax.plot([], [], lw=1.5, color="#d95f02", ls="--",
                              label="Baseline (sel)")
        ax.set_ylabel("Distance (mm)")
        ax.legend(loc="upper right", fontsize=8)
        ax.grid(True, alpha=THEME["grid"], linestyle='--')

    def update(self, state):
        with state.lock:
            if not state.times:
                return
            t = list(state.times)
            sel = state.selected
            series = [list(state.depth[i]) for i in range(3)]
            present = [list(state.present[i]) for i in range(3)]
        for i, line in enumerate(self.lines):
            d = series[i]
            line.set_data(t[:len(d)], d)
            line.set_visible(state.show_target[i])
            line.set_linewidth(2.2 if i == sel else 0.8)
            line.set_alpha(1.0 if (i == sel or any(present[i])) else 0.2)
        if sel is not None and series[sel]:
            d = series[sel]
            k = max(1, int(len(d) * analysis.TREND_TAIL))
            avg = sum(d[-k:]) / k
            self.trend.set_data(t[:len(d)], [avg] * len(d))
            self.trend.set_visible(True)
        else:
            self.trend.set_visible(False)
        self.ax.set_xlim(max(0, t[0]), max(t[-1], 1))
        alld = [v for s in series for v in s]
        if alld:
            self.ax.set_ylim(min(alld) - 10, max(alld) + 10)


class BreathPlot(Panel):
    """Detrended breath signal (AC) for every target; detection on selected."""

    def setup(self, ax):
        self.ax = ax
        ax.set_facecolor(THEME["panel_bg"])
        ax.set_title("Breath signal (AC, detrended)", fontsize=11, color=THEME["title"], loc='left')
        self.lines = [
            ax.plot([], [], lw=1.6, color=analysis.TARGET_COLORS[i],
                    label=f"Target {i} AC") [0]
            for i in range(3)
        ]
        ax.axhline(0, color="grey", lw=1.0, linestyle='--')
        ax.set_ylabel("Amplitude (mm)")
        ax.set_xlabel("Time (s)")
        ax.legend(loc="upper right", fontsize=8)
        ax.grid(True, alpha=THEME["grid"], linestyle='--')

    def update(self, state):
        with state.lock:
            if not state.times:
                return
            t = list(state.times)
            sel = state.selected
            series = [list(state.ac[i]) for i in range(3)]
            present = [list(state.present[i]) for i in range(3)]
            detected = state.detected
        for i, line in enumerate(self.lines):
            a = series[i]
            line.set_data(t[:len(a)], a)
            line.set_visible(state.show_target[i])
            line.set_linewidth(2.2 if i == sel else 0.8)
            line.set_alpha(1.0 if (i == sel or any(present[i])) else 0.2)
            if i == sel:
                line.set_color(analysis.SELECTED_COLOR if detected else "#adb5bd")
            else:
                line.set_color(analysis.TARGET_COLORS[i])
        amax = 2.0
        for a in series:
            if a:
                amax = max(amax, max(abs(v) for v in a) * 1.3)
        self.ax.set_ylim(-amax, amax)
        self.ax.set_xlim(max(0, t[0]), max(t[-1], 1))


class HeatmapPlot(Panel):
    """Spatial distribution: density of all targets + a marker per target."""

    def setup(self, ax):
        self.ax = ax
        self.img = ax.imshow(
            np.zeros((100, 100)), origin="lower", cmap="inferno", aspect="auto",
            extent=[analysis.LATERAL_MIN, analysis.LATERAL_MAX,
                    analysis.DEPTH_MIN, analysis.DEPTH_MAX], interpolation='nearest'
        )
        ax.set_title("Target spatial distribution", fontsize=11, color=THEME["title"], loc='left')
        ax.set_xlabel("Lateral Axis (mm)")
        ax.set_ylabel("Depth Axis (mm)")
        ax.set_xlim(analysis.LATERAL_MIN, analysis.LATERAL_MAX)
        ax.set_ylim(analysis.DEPTH_MIN, analysis.DEPTH_MAX)
        ax.grid(True, alpha=0.4, linestyle=':', color='white')

        ax.axvline(analysis.CENTER_LAT, color='white', lw=1.5, alpha=0.6, linestyle='-.')
        ax.plot(analysis.CENTER_LAT, 0, marker='^', color='white', markersize=10, clip_on=False)
        ax.text(analysis.CENTER_LAT, 40, 'RADAR TX/RX', ha='center', color='white',
                fontsize=10, fontweight='bold')

        zone = patches.Polygon(
            analysis.build_zone_patch(), closed=True,
            facecolor='none', edgecolor='#00ffff', lw=2.5,
            linestyle='--', alpha=0.9, label='Detection Zone'
        )
        ax.add_patch(zone)
        ax.legend(loc="upper left", facecolor='black', labelcolor='white', fontsize=8)

        self.markers = [
            ax.plot([], [], marker='o', markersize=14, ls='',
                    color=analysis.TARGET_COLORS[i],
                    markeredgecolor='white', markeredgewidth=2) [0]
            for i in range(3)
        ]
        self.ax.figure.colorbar(self.img, ax=ax, fraction=0.046, pad=0.04,
                                label='Position Density')

    def update(self, state):
        with state.lock:
            if not state.depth[0]:
                return
            sel = state.selected
            lat_all, dep_all = [], []
            markers = []
            for i in range(3):
                lat = list(state.lateral[i])
                dep = list(state.depth[i])
                pres = list(state.present[i])
                for la, de, pr in zip(lat, dep, pres):
                    if pr:
                        lat_all.append(la)
                        dep_all.append(de)
                if pres and pres[-1]:
                    markers.append((i, lat[-1], dep[-1]))
        if lat_all:
            lat_a = np.clip(lat_all, analysis.LATERAL_MIN, analysis.LATERAL_MAX)
            dep_a = np.clip(dep_all, analysis.DEPTH_MIN, analysis.DEPTH_MAX)
            heat_raw, _, _ = np.histogram2d(
                lat_a, dep_a, bins=100,
                range=[[analysis.LATERAL_MIN, analysis.LATERAL_MAX],
                       [analysis.DEPTH_MIN, analysis.DEPTH_MAX]]
            )
            heat = scipy_gaussian(heat_raw, sigma=1.8)
            self.img.set_data(heat.T)
            self.img.set_clim(0, max(0.1, np.max(heat)))
        else:
            self.img.set_data(np.zeros((100, 100)))
        for i, mk in enumerate(self.markers):
            found = [m for m in markers if m[0] == i]
            if found and state.show_target[i]:
                _, la, de = found[0]
                mk.set_data([la], [de])
                mk.set_color(analysis.SELECTED_COLOR if i == sel else analysis.TARGET_COLORS[i])
                mk.set_visible(True)
            else:
                mk.set_visible(False)


class BarPlot(Panel):
    """Radial distance (mm) of every target — the monitored one is highlighted."""

    def setup(self, ax):
        self.ax = ax
        ax.set_facecolor(THEME["panel_bg"])
        ax.set_title("Target range (radial distance)", fontsize=11, color=THEME["title"], loc='left')
        self.bars = ax.barh([f"T{i}" for i in range(3)], [0, 0, 0],
                            color=analysis.TARGET_COLORS)
        ax.set_xlim(0, analysis.DEPTH_MAX)
        ax.set_xlabel("Radial distance (mm)")
        ax.grid(True, axis="x", alpha=THEME["grid"], linestyle='--')

    def update(self, state):
        with state.lock:
            sel = state.selected
            depths = [list(state.depth[i]) for i in range(3)]
            show = list(state.show_target)
        for i, bar in enumerate(self.bars):
            d = depths[i]
            bar.set_width(d[-1] if d else 0)
            bar.set_visible(show[i])
            bar.set_color(analysis.SELECTED_COLOR if i == sel else analysis.TARGET_COLORS[i])


class StatsPanel(Panel):
    """Text panel: system state, algorithm metrics, and per-target summary."""

    def setup(self, ax):
        self.ax = ax
        ax.axis('off')
        ax.set_facecolor(THEME["panel_bg"])
        self.txt = ax.text(
            0.03, 0.97, "", fontsize=10, family='monospace', va='top',
            transform=ax.transAxes,
            bbox=dict(boxstyle="round,pad=0.6", fc="#ffffff", ec="#d0d7de", alpha=1.0)
        )

    def update(self, state):
        with state.lock:
            sel = state.selected
            visible = state.visible
            in_zone = state.in_zone
            stationary = state.stationary
            bpm = state.bpm
            detected = state.detected
            apnea = state.apnea
            quality = state.quality
            dropped = state.dropped
            latest = list(state.latest)

        tracking = "TRACKING" if in_zone and visible else ("VISIBLE" if visible else "NO TARGET")
        lines = [
            f"SYSTEM : {tracking:<10} | STAT: {'YES' if stationary else 'NO ':>3} | APNEA: {'DETECTED' if apnea else 'OK'}",
            f"PY BPM (FFT) = {bpm:>4.1f} | QUALITY = {quality:>5.1f}% | DROPPED = {dropped}",
            "--- targets ---",
        ]
        for i, (x, y, spd, res, pres, depth) in enumerate(latest):
            mark = "<<" if i == sel else "  "
            status = f"depth={depth:>5.0f} lat={x:>5} spd={spd:>4}" if pres else "empty"
            lines.append(f" T{i}{mark} {status}")
        self.txt.set_text("\n".join(lines))
        self.txt.set_bbox(dict(boxstyle="round,pad=0.6",
                               fc="#ffe3e3" if apnea else "#ffffff",
                               ec="#d0d7de", alpha=1.0))


class LogPanel(Panel):
    """Scrolling, titled panel showing how each ESP32 message was parsed."""

    def setup(self, ax):
        self.ax = ax
        ax.axis('off')
        ax.set_facecolor("#f6f8fa")
        ax.text(0.03, 1.0, "ЛОГ ПАРСИНГА ESP32", fontsize=10, fontweight='bold',
                va='top', color=THEME["title"], transform=ax.transAxes)
        self.txt = ax.text(0.03, 0.90, "", fontsize=8, family='monospace',
                           va='top', transform=ax.transAxes,
                           bbox=dict(boxstyle="round,pad=0.5", fc="#f6f8fa",
                                     ec="#d0d7de", alpha=1.0))

    def update(self, state):
        self.txt.set_text(state.recent_log(16))


class TargetSelector(Panel):
    """Checkbox control box: toggle drawing of each target, plus an 'All' master.

    Lives in its own axes (placed by the app outside the chart grid) and writes
    the per-target visibility into `state.show_target`. Every Panel already
    reads that flag, so no other code needs to know about the widget.
    """

    LABELS = ["Target 0", "Target 1", "Target 2", "All"]

    def __init__(self, state):
        self.state = state

    def setup(self, ax):
        self.ax = ax
        ax.set_xticks([])
        ax.set_yticks([])
        ax.set_facecolor("#e9eef5")
        for spine in ax.spines.values():
            spine.set_edgecolor("#bcc6d4")
            spine.set_linewidth(1.0)
        with self.state.lock:
            all_on = all(self.state.show_target)
        self.check = widgets.CheckButtons(ax, self.LABELS, actives=[all_on] * 4)
        self._busy = False
        self.check.on_clicked(self._on_click)

    def _on_click(self, label):
        if self._busy:
            return
        with self.state.lock:
            show = list(self.state.show_target)
        if label == "All":
            on = self.check.get_status()[3]
            show = [on, on, on]
        else:
            idx = self.LABELS.index(label)
            show[idx] = self.check.get_status()[idx]
        with self.state.lock:
            self.state.show_target = show
        all_on = all(show)
        if self.check.get_status()[3] != all_on:
            self._busy = True
            self.check.set_active(3)
            self._busy = False


# Grid layout: (row, col, PanelClass). Add/remove rows here.
LAYOUT = [
    (0, 0, WavePlot),
    (1, 0, BreathPlot),
    (2, 0, HeatmapPlot),
    (0, 1, StatsPanel),
    (1, 1, BarPlot),
    (2, 1, LogPanel),
]
