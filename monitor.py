#!/usr/bin/env python3
"""Live breath monitor for the HLK-LD2450 ESP32 firmware."""

import sys
import re
import time
import collections

import matplotlib
matplotlib.use("TkAgg")

import serial
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import matplotlib.patches as patches

import numpy as np

def find_port():
    """Best-effort auto-detection of a likely ESP32 port."""
    import serial.tools.list_ports
    ports = list(serial.tools.list_ports.comports())
    for p in ports:
        desc = (p.description or "").lower()
        if any(k in desc for k in ("usb", "uart", "cp210", "ch340", "esp", "serial")):
            return p.device
    return ports[0].device if ports else None


D_RE = re.compile(r"^D([-0-9.]+),([-0-9.]+),([-0-9.]+),([-0-9.]+),([01]),([01]),([01])$")
S_RE = re.compile(r"^S([-0-9.]+),([-0-9.]+),(\d+),([01]),([01]),([01])$")

WINDOW_S = 20
TREND_TAIL = 0.25
READ_CHUNK = 500

ZONE_R_MIN = 800.0
ZONE_R_MAX = 1200.0
ZONE_SIDE_MAX = 150.0
ZONE_EDGE_POINTS = 120


def build_zone_patch():
    xs = np.linspace(-ZONE_SIDE_MAX, ZONE_SIDE_MAX, ZONE_EDGE_POINTS)
    inner = np.sqrt(np.maximum(0.0, ZONE_R_MIN**2 - xs**2))
    outer = np.sqrt(np.maximum(0.0, ZONE_R_MAX**2 - xs**2))
    coords = np.column_stack([
        np.concatenate([xs, xs[::-1]]),
        np.concatenate([inner, outer[::-1]])
    ])
    return coords


class Monitor:
    def __init__(self, port, baud):
        self.ser = serial.Serial(port, baud, timeout=0.02)
        self.times = collections.deque()
        self.dist = collections.deque()
        self.ac = collections.deque()
        self.depth = collections.deque()
        self.lateral = collections.deque()
        self.bpm = self.amp = self.q = 0.0
        self.visible = False
        self.in_zone = False
        self.stationary = False
        self.last_valid_breath_time = None
        self.apnea_detected = False
        self.start = time.time()
        self._sync()

    def _sync(self):
        self.ser.reset_input_buffer()
        t0 = time.time()
        while time.time() - t0 < 5:
            line = self.ser.readline().decode(errors="ignore").strip()
            if "ready" in line:
                return

    def poll(self):
        for _ in range(READ_CHUNK):
            try:
                line = self.ser.readline().decode(errors="ignore").strip()
            except (serial.SerialException, OSError):
                raise
            except Exception:
                break
            if not line:
                break

            m = D_RE.match(line)
            if m:
                try:
                    dist = float(m.group(1))
                    acv = float(m.group(2))
                    depth = float(m.group(3))
                    lateral = float(m.group(4))
                    self.visible = (m.group(5) == "1")
                    self.in_zone = (m.group(6) == "1")
                    self.stationary = (m.group(7) == "1")
                except ValueError:
                    continue

                now = time.time() - self.start
                self.times.append(now)
                self.dist.append(dist)
                self.ac.append(acv)
                self.depth.append(max(0.0, depth))
                self.lateral.append(max(-400.0, min(400.0, lateral)))

                while self.times and self.times[0] < now - WINDOW_S:
                    self.times.popleft()
                    self.dist.popleft()
                    self.ac.popleft()
                    self.depth.popleft()
                    self.lateral.popleft()
                continue

            s = S_RE.match(line)
            if s:
                try:
                    self.bpm = float(s.group(1))
                    self.amp = float(s.group(2))
                    self.q = float(s.group(3))
                    self.visible = (s.group(4) == "1")
                    self.in_zone = (s.group(5) == "1")
                    self.stationary = (s.group(6) == "1")
                except ValueError:
                    continue

    def analyze_breath_fft(self):
        if not self.times or not self.ac:
            return 0.0, False, False, 0.0

        ts = np.array(self.times, dtype=float)
        signal = np.array(self.ac, dtype=float)
        if signal.size < 2 or ts.size < 2:
            return 0.0, False, False, 0.0

        n = min(ts.size, signal.size)
        ts = ts[-n:]
        signal = signal[-n:]

        duration = float(ts[-1] - ts[0])
        if duration < 5.0:
            return 0.0, False, False, 0.0

        signal = signal - np.mean(signal)
        if np.max(np.abs(signal)) < 1e-6:
            return 0.0, False, False, 0.0

        window = np.hanning(signal.size)
        signal_win = signal * window

        dt = np.diff(ts)
        dt = dt[dt > 0.0]
        fs = 1.0 / np.median(dt) if dt.size else signal.size / max(duration, 1e-6)

        if fs <= 0.0:
            return 0.0, False, False, 0.0

        spectrum = np.fft.rfft(signal_win)
        freqs = np.fft.rfftfreq(signal_win.size, d=1.0 / fs)
        mag = np.abs(spectrum)
        if np.max(mag) > 0.0:
            mag = mag / np.max(mag)

        band_mask = (freqs >= 0.1) & (freqs <= 0.5)
        if not np.any(band_mask):
            return 0.0, False, False, 0.0

        band_mag = mag[band_mask]
        peak_idx = int(np.argmax(band_mag))
        peak_freq = float(freqs[band_mask][peak_idx])
        peak_amp = float(band_mag[peak_idx])

        nonzero = band_mag[band_mag > 0.0]
        noise_floor = float(np.median(nonzero)) if nonzero.size else 0.0
        adaptive_threshold = max(0.15, 3.0 * max(noise_floor, 1e-3))

        bpm = peak_freq * 60.0
        breathing_detected = peak_amp > adaptive_threshold and 5.0 < bpm < 40.0

        filtered_mag = mag.copy()
        filtered_mag[~band_mask] = 0.0
        energy_total = float(np.sum(mag ** 2))
        energy_breath = float(np.sum(filtered_mag[band_mask] ** 2))
        quality = 100.0 * energy_breath / energy_total if energy_total > 0.0 else 0.0
        quality = float(np.clip(quality, 0.0, 100.0))

        current_time = float(ts[-1])
        if breathing_detected:
            self.last_valid_breath_time = current_time
            self.apnea_detected = False
        elif self.last_valid_breath_time is None:
            self.apnea_detected = current_time > 15.0
        else:
            self.apnea_detected = (current_time - self.last_valid_breath_time) > 15.0

        if not breathing_detected:
            bpm = 0.0

        return bpm, breathing_detected, self.apnea_detected, quality

    def baseline(self):
        d = list(self.dist)
        if not d:
            return []
        k = max(1, int(len(d) * TREND_TAIL))
        avg = sum(d[-k:]) / k
        return [avg] * len(d)


def main():
    port = sys.argv[1] if len(sys.argv) > 1 else find_port()
    baud = int(sys.argv[2]) if len(sys.argv) > 2 else 921600
    if not port:
        print("No serial port found. Pass it explicitly: python monitor.py COM3")
        return

    print(f"Connecting to {port} @ {baud} ...")
    mon = Monitor(port, baud)
    print(f"Connected. Streaming from {port}...")

    fig, ((ax_wave, ax_breath), (ax_heat, ax_bar)) = plt.subplots(
        2, 2, figsize=(14, 10), gridspec_kw={"height_ratios": [2, 1], "width_ratios": [1.5, 1]},
        sharex=False)
    fig.suptitle("HLK-LD2450 Breath Monitor (real-time)")

    line_dist, = ax_wave.plot([], [], lw=0.8, color="#2c7fb8", label="distance (mm)")
    line_trend, = ax_wave.plot([], [], lw=1.0, color="#d95f02", ls="--", label="baseline")
    ax_wave.set_ylabel("distance (mm)")
    ax_wave.legend(loc="upper right", fontsize=8)
    ax_wave.grid(True, alpha=0.3)

    line_ac, = ax_breath.plot([], [], lw=1.8, color="#2ca25f", label="breath (AC, mm)")
    breath_fill = None
    ax_breath.axhline(0, color="grey", lw=0.6)
    ax_breath.set_ylabel("breath (mm)")
    ax_breath.set_xlabel("time (s)")
    ax_breath.legend(loc="upper right", fontsize=8)
    ax_breath.grid(True, alpha=0.3)

    heat_img = ax_heat.imshow(np.zeros((40, 40)), origin="lower",
                              cmap="inferno", aspect="equal",
                              extent=[-400.0, 400.0, 0.0, 1500.0])
    ax_heat.set_title("Movement heatmap (lateral ←→, depth forward)")
    ax_heat.set_xlabel("lateral mm (sideways)")
    ax_heat.set_ylabel("depth mm (forward from sensor)")
    ax_heat.set_xlim(-400.0, 400.0)
    ax_heat.set_ylim(0.0, 1500.0)
    ax_heat.grid(True, alpha=0.2)

    zone_patch = patches.Polygon(build_zone_patch(), closed=True,
                                facecolor='none', edgecolor='cyan', lw=1.4,
                                linestyle='--', alpha=0.8)
    ax_heat.add_patch(zone_patch)

    heat_marker, = ax_heat.plot([], [], marker='o', markersize=14,
                                markerfacecolor='cyan', markeredgecolor='white',
                                markeredgewidth=2, linestyle='')

    bar = ax_bar.barh(["bpm"], [0], color="#41ab5d")
    ax_bar.set_xlim(0, 40)
    ax_bar.set_xlabel("breaths / min")
    ax_bar.grid(True, axis="x", alpha=0.3)

    txt_stat = fig.text(0.01, 0.97, "", fontsize=9.5,
                        bbox=dict(boxstyle="round", fc="white", alpha=0.85))

    def update(_):
        nonlocal breath_fill
        if breath_fill is not None:
            breath_fill.remove()
            breath_fill = None

        try:
            mon.poll()
            py_bpm, detected, apnea, quality = mon.analyze_breath_fft()
        except (serial.SerialException, OSError):
            plt.close(fig)
            return line_dist, line_trend, line_ac, bar

        try:
            if not mon.times:
                line_dist.set_data([], [])
                line_trend.set_data([], [])
                line_ac.set_data([], [])
                txt_stat.set_text("No data yet")
                bar[0].set_width(0)
                return line_dist, line_trend, line_ac, bar

            t = list(mon.times)
            d = list(mon.dist)
            a = list(mon.ac)
            depth = list(mon.depth)
            lateral = list(mon.lateral)
            tr = mon.baseline()

            line_dist.set_data(t, d)
            line_trend.set_data(t, tr)
            line_ac.set_data(t, a)

            # === Wave plots limits ===
            ax_wave.set_xlim(max(0, t[0]), max(t[-1], 1))
            if d and tr:
                lo = min(min(d), min(tr)) - 5
                hi = max(max(d), max(tr)) + 5
                ax_wave.set_ylim(lo, hi)

            if a:
                amax = max(2.0, max(abs(v) for v in a) * 1.2)
                ax_breath.set_ylim(-amax, amax)
            ax_breath.set_xlim(max(0, t[0]), max(t[-1], 1))

            line_ac.set_color("#2ca25f" if detected else "#888888")
            ax_breath.set_title("breath (AC, mm) " + ("[BREATHING]" if detected else "[idle]"))

            if a and t:
                breath_fill = ax_breath.fill_between(t, a, 0, color="#2ca25f", alpha=0.12)

            # === Heatmap + Marker ===
            if depth and lateral:
                depth_arr = np.clip(depth, 0.0, 1500.0)
                lateral_arr = np.clip(lateral, -400.0, 400.0)

                heat, _, _ = np.histogram2d(
                    lateral_arr, depth_arr, bins=40,
                    range=[[-400.0, 400.0], [0.0, 1500.0]])

                heat_img.set_data(heat.T)
                heat_img.set_clim(0, max(1.0, np.max(heat)))

                last_lat = lateral_arr[-1]
                last_dep = depth_arr[-1]

                heat_marker.set_data([last_lat], [last_dep])
                heat_marker.set_markerfacecolor('cyan' if mon.in_zone else 'yellow')

                pos_text = f"Depth={last_dep:.0f} mm, Lat={last_lat:.0f} mm"
                if last_dep < 50:
                    pos_text += " [BOTTOM EDGE - side detection]"
            else:
                heat_img.set_data(np.zeros((40, 40)))
                heat_marker.set_data([], [])
                pos_text = "Pos: no data"

            # Status text
            radar_state = "YES" if mon.visible else "NO"
            zone_state = "YES" if mon.in_zone else "NO"
            stationary_state = "YES" if mon.stationary else "NO"
            tracking_state = "TRACKING" if mon.in_zone and mon.visible else (
                "VISIBLE" if mon.visible else "NO TARGET")

            txt_stat.set_text(
                f"State: {tracking_state} | Stationary: {stationary_state}\n"
                f"FW BPM: {mon.bpm:5.1f} | PY FFT BPM: {py_bpm:5.1f}\n"
                f"{pos_text} | Breathing: {'YES' if detected else 'NO'} | "
                f"Quality: {quality:5.1f}% | Apnea: {'YES' if apnea else 'NO'}"
            )

            bar[0].set_width(mon.bpm if mon.bpm > 0 else 0)

        except Exception as e:
            txt_stat.set_text(f"Monitor error\n{e}")
            print("update error:", e, file=sys.stderr)

        return line_dist, line_trend, line_ac, bar

    ani = animation.FuncAnimation(fig, update, interval=30,
                                  blit=False, cache_frame_data=False)
    plt.show()


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        pass