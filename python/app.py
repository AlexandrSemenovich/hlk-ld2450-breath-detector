"""Application glue: serial reader, figure construction, animation loop.

This module wires the pieces together but contains no analysis or drawing
logic of its own — swap `protocol.decode_line`, `state.MonitorState`, or the
`plots.LAYOUT` list to change behaviour without touching this file.
"""

import os
import sys
import time
import threading

import matplotlib
_backend = os.environ.get("MPLBACKEND")
if _backend:
    matplotlib.use(_backend)
else:
    matplotlib.use("TkAgg")

import matplotlib.pyplot as plt
import matplotlib.animation as animation
import serial

import protocol
import state as state_mod
import plots


try:
    import serial.tools.list_ports
    _HAVE_LIST_PORTS = True
except Exception:  # pragma: no cover
    _HAVE_LIST_PORTS = False


def find_port():
    if not _HAVE_LIST_PORTS:
        return None
    ports = list(serial.tools.list_ports.comports())
    for p in ports:
        desc = (p.description or "").lower()
        if any(k in desc for k in ("usb", "uart", "cp210", "ch340", "esp", "serial")):
            return p.device
    return ports[0].device if ports else None


class MonitorApp:
    def __init__(self, port, baud):
        self.state = state_mod.MonitorState()
        self.ser = serial.Serial(port, baud, timeout=0.05)
        self._stop = False
        self._sync()
        self.thread = threading.Thread(target=self.reader, daemon=True)
        self.thread.start()

    def _sync(self):
        """Wait for the ESP 'ready' banner so we don't parse startup text."""
        self.ser.reset_input_buffer()
        t0 = time.time()
        while time.time() - t0 < 5:
            line = self.ser.readline().decode(errors="ignore").strip()
            if "ready" in line:
                return

    def reader(self):
        while not self._stop:
            try:
                line = self.ser.readline().decode(errors="ignore").strip()
            except (serial.SerialException, OSError):
                break
            if not line:
                continue
            raw = protocol.decode_line(line)
            if raw is None:
                tag = "PARSE FAIL" if line.startswith("R") else "INFO"
                self.state.append_log(f"{tag}: {line[:80]}")
                continue
            self.state.ingest(raw)

    def build_figure(self):
        fig = plt.figure(figsize=(15, 11))
        fig.patch.set_facecolor(plots.THEME["fig_bg"])

        # Top status bar (full width).
        self.header = plots.HeaderPanel()
        self.header.setup(fig.add_axes([0.0, 0.93, 1.0, 0.07]))

        # Main chart grid: signals + spatial on the left, analysis on the right.
        gs = fig.add_gridspec(
            3, 2, left=0.06, right=0.97, top=0.91, bottom=0.30,
            height_ratios=[2, 1, 1.2], width_ratios=[1.5, 1.0],
            hspace=0.35, wspace=0.25,
        )
        self.panels = []
        for (r, c, cls) in plots.LAYOUT:
            ax = fig.add_subplot(gs[r, c])
            p = cls()
            p.setup(ax)
            self.panels.append(p)

        # Target-visibility control box in the bottom-left margin.
        ctrl_ax = fig.add_axes([0.06, 0.04, 0.24, 0.18])
        self.selector = plots.TargetSelector(self.state)
        self.selector.setup(ctrl_ax)
        fig.text(0.06, 0.235, "ПОКАЗЫВАЕМЫЕ ЦЕЛИ",
                 fontsize=10, fontweight='bold', color=plots.THEME["title"])

        return fig

    def _update(self, _):
        self.state.analyze()
        self.header.update(self.state)
        for p in self.panels:
            p.update(self.state)

    def run(self):
        self.fig = self.build_figure()
        self.ani = animation.FuncAnimation(
            self.fig, self._update, interval=50, blit=False, cache_frame_data=False
        )
        plt.show()

    def stop(self):
        self._stop = True
