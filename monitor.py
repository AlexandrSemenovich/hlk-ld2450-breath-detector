#!/usr/bin/env python3
"""Live breath monitor for the HLK-LD2450 ESP32 firmware.

Reads the CSV stream printed by the firmware on the USB UART:
    bpm,amp_mm,quality,dist_mm
and plots, in real time:
  - the breath waveform (distance, mm) with a moving trend line
  - the current breaths/min, amplitude and quality

Requirements:
    pip install pyserial matplotlib

Usage:
    python monitor.py                      # auto-detect port at 115200
    python monitor.py COM3                 # explicit port
    python monitor.py /dev/ttyUSB0 921600  # custom baud
"""
import sys
import re
import time
import collections

import matplotlib
matplotlib.use("TkAgg")   # stable backend on Windows

import serial
import matplotlib.pyplot as plt
import matplotlib.animation as animation


def find_port():
    """Best-effort auto-detection of a likely ESP32 port."""
    import serial.tools.list_ports
    ports = list(serial.tools.list_ports.comports())
    for p in ports:
        desc = (p.description or "").lower()
        if any(k in desc for k in ("usb", "uart", "cp210", "ch340", "esp", "serial")):
            return p.device
    return ports[0].device if ports else None


CSV_RE = re.compile(r"^\s*([-0-9.]+),([-0-9.]+),(\d+),([-0-9.]+)\s*$")

WINDOW_S = 20          # seconds shown on the chart
MIN_SAMPLES = 5
READ_CHUNK = 200       # max lines drained per animation frame


class Monitor:
    def __init__(self, port, baud):
        # timeout=0 => purely non-blocking reads, so the GUI timer never stalls.
        self.ser = serial.Serial(port, baud, timeout=0)
        self.times = collections.deque()
        self.dist = collections.deque()
        self.bpm = self.amp = self.q = 0.0
        self.dist_v = 0.0
        self.start = time.time()
        self._sync()

    def _sync(self):
        """Drain boot text until we see the CSV header line."""
        t0 = time.time()
        while time.time() - t0 < 5:
            line = self.ser.readline().decode(errors="ignore").strip()
            if line.startswith("bpm,"):
                return

    def poll(self):
        """Read whatever is buffered right now (non-blocking)."""
        n = min(self.ser.in_waiting or 0, READ_CHUNK)
        for _ in range(n):
            line = self.ser.readline().decode(errors="ignore").strip()
            if not line:
                break
            m = CSV_RE.match(line)
            if not m:
                continue
            bpm, amp, q, dist = (float(m.group(1)), float(m.group(2)),
                                 float(m.group(3)), float(m.group(4)))
            self.bpm, self.amp, self.q, self.dist_v = bpm, amp, q, dist
            now = time.time() - self.start
            self.times.append(now)
            self.dist.append(dist)
            while self.times and self.times[0] < now - WINDOW_S:
                self.times.popleft()
                self.dist.popleft()

    def trend(self):
        """Trailing-average baseline for the distance signal."""
        if len(self.dist) < MIN_SAMPLES:
            return []
        n = max(MIN_SAMPLES, len(self.dist) // 4)
        tail = list(self.dist)[-n:]
        avg = sum(tail) / len(tail)
        return [avg] * len(self.times)


def main():
    port = sys.argv[1] if len(sys.argv) > 1 else find_port()
    baud = int(sys.argv[2]) if len(sys.argv) > 2 else 115200
    if not port:
        print("No serial port found. Pass it explicitly: python monitor.py COM3")
        return

    print(f"Connecting to {port} @ {baud} ...")
    mon = Monitor(port, baud)
    print(f"Connected. Streaming from {port}...")

    fig, (ax_wave, ax_bar) = plt.subplots(
        2, 1, figsize=(11, 6), gridspec_kw={"height_ratios": [3, 1]},
        sharex=False)
    fig.suptitle("HLK-LD2450 Breath Monitor")

    line_dist, = ax_wave.plot([], [], lw=1.2, color="#2c7fb8", label="distance (mm)")
    line_trend, = ax_wave.plot([], [], lw=1.0, color="#d95f02", ls="--", label="baseline")
    txt_stat = ax_wave.text(0.01, 0.95, "", transform=ax_wave.transAxes,
                            va="top", fontsize=10,
                            bbox=dict(boxstyle="round", fc="white", alpha=0.8))
    ax_wave.set_ylabel("distance (mm)")
    ax_wave.set_xlabel("time (s)")
    ax_wave.legend(loc="upper right", fontsize=8)
    ax_wave.grid(True, alpha=0.3)
    ax_wave.set_xlim(0, 1)
    ax_wave.set_ylim(0, 1)

    bar = ax_bar.barh(["bpm"], [0], color="#41ab5d")
    ax_bar.set_xlim(0, 40)
    ax_bar.set_xlabel("breaths / min")
    ax_bar.grid(True, axis="x", alpha=0.3)

    def update(_):
        try:
            mon.poll()
        except serial.SerialException:
            plt.close(fig)
            return line_dist, line_trend, txt_stat, bar
        if not mon.times:
            return line_dist, line_trend, txt_stat, bar
        t = list(mon.times)
        d = list(mon.dist)
        tr = mon.trend()
        line_dist.set_data(t, d)
        line_trend.set_data(t, tr)
        ax_wave.set_xlim(max(0, t[0]), max(t[-1], 1))
        lo = min(d + tr) - 5
        hi = max(d + tr) + 5
        ax_wave.set_ylim(lo, hi)

        txt_stat.set_text(
            f"BPM: {mon.bpm:5.1f}   amp: {mon.amp:5.1f} mm   "
            f"quality: {int(mon.q):3d}%   dist: {mon.dist_v:6.1f} mm")

        bar[0].set_width(mon.bpm)
        return line_dist, line_trend, txt_stat, bar

    ani = animation.FuncAnimation(fig, update, interval=100,
                                  blit=False, cache_frame_data=False)
    plt.show()


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        pass
