#!/usr/bin/env python3
"""Live breath monitor for the HLK-LD2450 ESP32 firmware.

PC-centric analysis: the ESP32 only forwards raw radar target data (plus a
lightweight firmware BPM estimate for reference). All heavy lifting — band-pass
filtering, FFT, SNR/quality and apnea detection — happens here in Python.
"""

import sys
import os
import re
import time
import threading
import collections

import matplotlib
_backend = os.environ.get("MPLBACKEND")
if _backend:
    matplotlib.use(_backend)
else:
    matplotlib.use("TkAgg")

import serial
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import matplotlib.patches as patches
import numpy as np
from scipy.signal import butter, filtfilt

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


D_RE = re.compile(r"^D([-0-9.]+),([-0-9.]+),([-0-9.]+),([-0-9.]+),([01]),([01]),([01]),(\d+),(\d+)$")
S_RE = re.compile(r"^S([-0-9.]+),([-0-9.]+),(\d+),([01]),([01]),([01]),(\d+),(\d+)$")

WINDOW_S = 30
TREND_TAIL = 0.25
READ_CHUNK = 200

# ==================== НАСТРОЙКИ КАРТЫ ====================
LATERAL_MIN = -600       # Лимит влево (мм)
LATERAL_MAX = 600        # Лимит вправо (мм)
CENTER_LAT = 0
DEPTH_MIN = 0
DEPTH_MAX = 2600         # Глубина обнаружения (до ~2.5 м)
# =======================================================

ZONE_R_MIN = 600.0
ZONE_R_MAX = 2500.0
ZONE_SIDE_MAX = 500.0
ZONE_EDGE_POINTS = 200

# Breath band (Hz) -> 7.2 .. 30 breaths/min, with margin.
BAND_LO_HZ = 0.12
BAND_HI_HZ = 0.5
MIN_BPM = 5.0
MAX_BPM = 40.0
APNEA_S = 15.0
SNR_THRESHOLD = 3.0


def build_zone_patch():
    """Математически точное построение зоны относительно центра радара.

    lateral = x (поперечная ось), depth = radial (sqrt(x^2+y^2)).
    """
    xs = np.linspace(-ZONE_SIDE_MAX, ZONE_SIDE_MAX, ZONE_EDGE_POINTS)
    inner = np.sqrt(np.maximum(0.0, ZONE_R_MIN ** 2 - xs ** 2))
    outer = np.sqrt(np.maximum(0.0, ZONE_R_MAX ** 2 - xs ** 2))
    coords = np.column_stack([
        np.concatenate([xs, xs[::-1]]) + CENTER_LAT,
        np.concatenate([inner, outer[::-1]])
    ])
    return coords


def detect_breath(ts_ms, ac):
    """Pure PC-side breath detector (no I/O).

    Resamples `ac` to a uniform grid using radar `ts_ms`, applies a Butterworth
    band-pass, then an FFT peak search in the breathing band. Returns
    (bpm, detected, quality, snr). bpm is 0.0 when breathing is not detected.
    """
    ts = np.asarray(ts_ms, dtype=float)
    signal = np.asarray(ac, dtype=float)
    if ts.size < 2 or signal.size < 2:
        return 0.0, False, 0.0, 0.0

    t0 = ts[0]
    tt = (ts - t0) / 1000.0
    duration = float(tt[-1] - tt[0])
    if duration < 5.0:
        return 0.0, False, 0.0, 0.0

    dt = np.diff(ts)
    dt = dt[dt > 0]
    fs = 1000.0 / float(np.median(dt)) if dt.size else 10.0
    if fs <= 0.0:
        return 0.0, False, 0.0, 0.0

    t_uni = np.arange(0.0, duration, 1.0 / fs)
    if t_uni.size < 8:
        return 0.0, False, 0.0, 0.0
    sig_u = np.interp(t_uni, tt, signal)
    sig_u = sig_u - np.mean(sig_u)

    nyq = fs / 2.0
    lo = max(BAND_LO_HZ / nyq, 1e-4)
    hi = min(BAND_HI_HZ / nyq, 0.99)
    b, a = butter(2, [lo, hi], btype="band")
    filt = filtfilt(b, a, sig_u)

    spectrum = np.fft.rfft(filt * np.hanning(filt.size))
    freqs = np.fft.rfftfreq(filt.size, d=1.0 / fs)
    mag = np.abs(spectrum)
    if np.max(mag) > 0.0:
        mag = mag / np.max(mag)

    band_mask = (freqs >= BAND_LO_HZ) & (freqs <= BAND_HI_HZ)
    if not np.any(band_mask):
        return 0.0, False, 0.0, 0.0

    band_freqs = freqs[band_mask]
    band_mag = mag[band_mask]
    peak_idx = int(np.argmax(band_mag))
    peak_freq = float(band_freqs[peak_idx])
    peak_amp = float(band_mag[peak_idx])

    no_peak = np.ones(band_mag.size, dtype=bool)
    no_peak[max(0, peak_idx - 2):peak_idx + 3] = False
    rest = band_mag[no_peak]
    noise_floor = float(np.median(rest)) if rest.size else band_mag.min()
    snr = peak_amp / max(noise_floor, 1e-3)

    bpm = peak_freq * 60.0
    detected = (snr > SNR_THRESHOLD) and (MIN_BPM < bpm < MAX_BPM)
    quality = float(np.clip(100.0 * snr / (snr + 3.0), 0.0, 100.0))

    if not detected:
        bpm = 0.0
    return bpm, detected, quality, snr


class Monitor:
    def __init__(self, port, baud):
        self.ser = serial.Serial(port, baud, timeout=0.05)
        self.lock = threading.Lock()
        self.times = collections.deque(maxlen=2000)
        self.dist = collections.deque(maxlen=2000)
        self.ac = collections.deque(maxlen=2000)
        self.depth = collections.deque(maxlen=2000)
        self.lateral = collections.deque(maxlen=2000)
        self.tsms = collections.deque(maxlen=2000)
        self.bpm = self.amp = self.q = 0.0
        self.visible = False
        self.in_zone = False
        self.stationary = False
        self.last_valid_breath_time = None
        self.apnea_detected = False
        self.start = time.time()
        self._stop = False
        self.last_frame_id = 0
        self.dropped = 0
        self._sync()
        self.thread = threading.Thread(target=self.reader, daemon=True)
        self.thread.start()

    def _sync(self):
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
            self._handle(line)

    def _handle(self, line):
        m = D_RE.match(line)
        if m:
            try:
                dist = float(m.group(1))
                acv = float(m.group(2))
                depth_val = float(m.group(3))
                lateral_val = float(m.group(4))
                ts_ms = int(m.group(8))
                frame_id = int(m.group(9))

                # Zero-Order-Hold защита от выбросов/багов координат.
                if lateral_val > LATERAL_MAX or lateral_val < LATERAL_MIN or depth_val < DEPTH_MIN:
                    lateral_val = self.lateral[-1] if self.lateral else CENTER_LAT
                    depth_val = self.depth[-1] if self.depth else 0.0

                visible = (m.group(5) == "1")
                in_zone = (m.group(6) == "1")
                stationary = (m.group(7) == "1")
            except ValueError:
                return

            with self.lock:
                now = time.time() - self.start
                # Детект пропущенных кадров.
                if self.last_frame_id and frame_id != self.last_frame_id + 1:
                    self.dropped += (frame_id - self.last_frame_id - 1)
                self.last_frame_id = frame_id

                self.times.append(now)
                self.dist.append(dist)
                self.ac.append(acv)
                self.depth.append(max(0.0, depth_val))
                self.lateral.append(lateral_val)
                self.tsms.append(ts_ms)
                self.visible = visible
                self.in_zone = in_zone
                self.stationary = stationary

                while self.times and self.times[0] < now - WINDOW_S:
                    for dq in (self.times, self.dist, self.ac, self.depth, self.lateral, self.tsms):
                        dq.popleft()
            return

        s = S_RE.match(line)
        if s:
            try:
                with self.lock:
                    self.bpm = float(s.group(1))
                    self.amp = float(s.group(2))
                    self.q = float(s.group(3))
                    self.visible = (s.group(4) == "1")
                    self.in_zone = (s.group(5) == "1")
                    self.stationary = (s.group(6) == "1")
            except ValueError:
                return

    def poll(self):
        """Deprecated: data is now collected by the reader thread."""
        pass

    def analyze_breath_fft(self):
        with self.lock:
            if not self.in_zone or len(self.ac) < 2 or len(self.tsms) < 2:
                self.last_valid_breath_time = None
                self.apnea_detected = False
                return 0.0, False, False, 0.0
            ts = list(self.tsms)
            ac = list(self.ac)

        bpm, detected, quality, snr = detect_breath(ts, ac)

        current_time = time.time() - self.start
        if detected:
            self.last_valid_breath_time = current_time
            self.apnea_detected = False
        elif self.last_valid_breath_time is None:
            self.apnea_detected = current_time > APNEA_S
        else:
            self.apnea_detected = (current_time - self.last_valid_breath_time) > APNEA_S

        return bpm, detected, self.apnea_detected, quality

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
        print("No serial port found.")
        return

    print(f"Connecting to {port} @ {baud} ...")
    mon = Monitor(port, baud)
    print("Connected.")

    fig, ((ax_wave, ax_breath), (ax_heat, ax_bar)) = plt.subplots(
        2, 2, figsize=(15, 11), gridspec_kw={"height_ratios": [2, 1], "width_ratios": [1.5, 1]}
    )
    fig.suptitle("HLK-LD2450 Breath Monitor — Scientific View", fontsize=14, fontweight='bold')

    line_dist, = ax_wave.plot([], [], lw=1.2, color="#2c7fb8", label="Distance (mm)")
    line_trend, = ax_wave.plot([], [], lw=1.5, color="#d95f02", ls="--", label="Baseline")
    ax_wave.set_ylabel("Distance (mm)")
    ax_wave.legend(loc="upper right", fontsize=10)
    ax_wave.grid(True, alpha=0.5, linestyle='--')

    line_ac, = ax_breath.plot([], [], lw=2.0, color="#2ca25f", label="Breath Signal (AC)")
    breath_fill = None
    ax_breath.axhline(0, color="grey", lw=1.0, linestyle='--')
    ax_breath.set_ylabel("Amplitude (mm)")
    ax_breath.set_xlabel("Time (s)")
    ax_breath.legend(loc="upper right", fontsize=10)
    ax_breath.grid(True, alpha=0.5, linestyle='--')

    heat_img = ax_heat.imshow(
        np.zeros((100, 100)), origin="lower", cmap="inferno", aspect="auto",
        extent=[LATERAL_MIN, LATERAL_MAX, DEPTH_MIN, DEPTH_MAX], interpolation='nearest'
    )
    cbar = fig.colorbar(heat_img, ax=ax_heat, fraction=0.046, pad=0.04)
    cbar.set_label('Position Density', rotation=270, labelpad=15)

    ax_heat.set_title("Target Spatial Distribution", fontsize=12)
    ax_heat.set_xlabel("Lateral Axis (mm)")
    ax_heat.set_ylabel("Depth Axis (mm)")
    ax_heat.set_xlim(LATERAL_MIN, LATERAL_MAX)
    ax_heat.set_ylim(DEPTH_MIN, DEPTH_MAX)
    ax_heat.grid(True, alpha=0.4, linestyle=':', color='white')

    ax_heat.axvline(CENTER_LAT, color='white', lw=1.5, alpha=0.6, linestyle='-.')
    ax_heat.plot(CENTER_LAT, 0, marker='^', color='white', markersize=10, clip_on=False)
    ax_heat.text(CENTER_LAT, 40, 'RADAR TX/RX', ha='center', color='white', fontsize=10, fontweight='bold')

    zone_patch = patches.Polygon(
        build_zone_patch(), closed=True,
        facecolor='none', edgecolor='#00ffff', lw=2.5,
        linestyle='--', alpha=0.9, label='Detection Zone'
    )
    ax_heat.add_patch(zone_patch)
    ax_heat.legend(loc="upper left", facecolor='black', labelcolor='white')

    heat_marker, = ax_heat.plot([], [], marker='o', markersize=16,
                                markerfacecolor='#00ff00', markeredgecolor='white', markeredgewidth=2)

    bar = ax_bar.barh(["BPM (FFT)"], [0], color="#41ab5d")
    ax_bar.set_xlim(0, 40)
    ax_bar.set_xlabel("Breaths / Min")
    ax_bar.grid(True, axis="x", alpha=0.5, linestyle='--')

    txt_stat = fig.text(0.02, 0.96, "", fontsize=11, family='monospace',
                        bbox=dict(boxstyle="round,pad=0.5", fc="#f8f9fa", ec="#dee2e6", alpha=0.95))

    plt.tight_layout(rect=[0, 0, 1, 0.95])

    def update(_):
        nonlocal breath_fill
        if breath_fill is not None:
            breath_fill.remove()
            breath_fill = None

        mon.poll()
        py_bpm, detected, apnea, quality = mon.analyze_breath_fft()

        with mon.lock:
            if not mon.times:
                return line_dist, line_trend, line_ac, bar, heat_img, heat_marker
            t = list(mon.times)
            d = list(mon.dist)
            a = list(mon.ac)
            depth = list(mon.depth)
            lateral = list(mon.lateral)
            in_zone = mon.in_zone
            visible = mon.visible
            stationary = mon.stationary
            fw_bpm = mon.bpm
            dropped = mon.dropped

        tr = mon.baseline()

        line_dist.set_data(t, d)
        line_trend.set_data(t, tr)
        line_ac.set_data(t, a)

        ax_wave.set_xlim(max(0, t[0]), max(t[-1], 1))
        if d and tr:
            y_min = min(min(d), min(tr)) - 10
            y_max = max(max(d), max(tr)) + 10
            if y_max > y_min:
                ax_wave.set_ylim(y_min, y_max)

        if a:
            amax = max(2.0, max(abs(v) for v in a) * 1.3)
            ax_breath.set_ylim(-amax, amax)
        ax_breath.set_xlim(max(0, t[0]), max(t[-1], 1))

        line_ac.set_color("#2ca25f" if detected else "#adb5bd")
        if a and t:
            breath_fill = ax_breath.fill_between(t, a, 0, color="#2ca25f" if detected else "#adb5bd", alpha=0.15)

        if depth and lateral:
            depth_arr = np.clip(depth, DEPTH_MIN, DEPTH_MAX)
            lateral_arr = np.clip(lateral, LATERAL_MIN, LATERAL_MAX)
            heat_raw, _, _ = np.histogram2d(
                lateral_arr, depth_arr, bins=100,
                range=[[LATERAL_MIN, LATERAL_MAX], [DEPTH_MIN, DEPTH_MAX]]
            )
            heat_smooth = scipy_gaussian(heat_raw, sigma=1.8)
            heat_img.set_data(heat_smooth.T)
            vmax = max(0.1, np.max(heat_smooth))
            heat_img.set_clim(0, vmax)
            heat_marker.set_data([lateral_arr[-1]], [depth_arr[-1]])
            heat_marker.set_markerfacecolor('#00ff00' if in_zone else '#ffae00')
        else:
            heat_img.set_data(np.zeros((100, 100)))
            heat_marker.set_data([], [])

        tracking_state = "TRACKING" if in_zone and visible else ("VISIBLE" if visible else "NO TARGET")
        bg_color = "#ffe3e3" if apnea else "#f8f9fa"
        txt_stat.set_bbox(dict(boxstyle="round,pad=0.5", fc=bg_color, ec="#dee2e6", alpha=0.95))

        txt_stat.set_text(
            f"SYSTEM STATE : {tracking_state:<10} | STATIONARY: {'YES' if stationary else 'NO':<3} | APNEA: {'DETECTED' if apnea else 'OK'}\n"
            f"ALGORITHM    : FW BPM = {fw_bpm:>4.1f} | PY BPM (FFT) = {py_bpm:>4.1f} \n"
            f"SIGNAL METRIC: BREATHING = {'YES' if detected else 'NO':<3} | QUALITY = {quality:>5.1f}% | DROPPED = {dropped}"
        )

        bar[0].set_width(py_bpm if detected else 0)

        return line_dist, line_trend, line_ac, bar, heat_img, heat_marker

    ani = animation.FuncAnimation(fig, update, interval=50, blit=False, cache_frame_data=False)
    plt.show()


def scipy_gaussian(arr, sigma=1.8):
    try:
        from scipy.ndimage import gaussian_filter
        return gaussian_filter(arr, sigma=sigma)
    except Exception:
        return arr


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        pass
    finally:
        plt.close()
