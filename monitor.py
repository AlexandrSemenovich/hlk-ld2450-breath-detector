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
import scipy.ndimage  # Для сглаживания тепловой карты

def find_port():
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

# ==================== НАСТРОЙКИ КАРТЫ ====================
LATERAL_MIN = -600       # Лимит влево (мм)
LATERAL_MAX = 600        # Лимит вправо (мм)
CENTER_LAT = 0            # Физический центр радара теперь строго 0
DEPTH_MIN = 0
DEPTH_MAX = 1600          # Глубина обнаружения (до 3 метров)
# =======================================================

ZONE_R_MIN = 600.0
ZONE_R_MAX = 1200.0
ZONE_SIDE_MAX = 150.0
ZONE_EDGE_POINTS = 200

def build_zone_patch():
    """Математически точное построение зоны относительно центра радара"""
    xs = np.linspace(-ZONE_SIDE_MAX, ZONE_SIDE_MAX, ZONE_EDGE_POINTS)
    inner = np.sqrt(np.maximum(0.0, ZONE_R_MIN**2 - xs**2))
    outer = np.sqrt(np.maximum(0.0, ZONE_R_MAX**2 - xs**2))
    coords = np.column_stack([
        np.concatenate([xs, xs[::-1]]) + CENTER_LAT,
        np.concatenate([inner, outer[::-1]])
    ])
    return coords


class Monitor:
    def __init__(self, port, baud):
        self.ser = serial.Serial(port, baud, timeout=0.02)
        self.times = collections.deque(maxlen=1000)
        self.dist = collections.deque(maxlen=1000)
        self.ac = collections.deque(maxlen=1000)
        self.depth = collections.deque(maxlen=1000)
        self.lateral = collections.deque(maxlen=1000)
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
                    depth_val = float(m.group(3))
                    lateral_val = float(m.group(4))
                    
                    # Если латераль равна глубине (баг) или выходит за границы,
                    # используем последнюю известную достоверную координату (Zero-Order Hold).
                    # не ломает временной ряд и не телепортирует объект ложно.
                    if abs(lateral_val - depth_val) < 1e-3 or lateral_val > LATERAL_MAX or lateral_val < LATERAL_MIN:
                        lateral_val = self.lateral[-1] if self.lateral else CENTER_LAT
                        depth_val = self.depth[-1] if self.depth else 0.0

                    self.visible = (m.group(5) == "1")
                    self.in_zone = (m.group(6) == "1")
                    self.stationary = (m.group(7) == "1")
                except ValueError:
                    continue

                now = time.time() - self.start
                self.times.append(now)
                self.dist.append(dist)
                self.ac.append(acv)
                self.depth.append(max(0.0, depth_val))
                self.lateral.append(lateral_val)

                while self.times and self.times[0] < now - WINDOW_S:
                    for dq in (self.times, self.dist, self.ac, self.depth, self.lateral):
                        dq.popleft()
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
        if not self.in_zone:
            self.last_valid_breath_time = None
            self.apnea_detected = False
            return 0.0, False, False, 0.0

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
        print("No serial port found.")
        return

    print(f"Connecting to {port} @ {baud} ...")
    mon = Monitor(port, baud)
    print("Connected.")

    fig, ((ax_wave, ax_breath), (ax_heat, ax_bar)) = plt.subplots(
        2, 2, figsize=(15, 11), gridspec_kw={"height_ratios": [2, 1], "width_ratios": [1.5, 1]}
    )
    fig.suptitle("HLK-LD2450 Breath Monitor — Scientific View", fontsize=14, fontweight='bold')

    # Оформление графиков сигналов
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

    # ==================== НАУЧНАЯ ТЕПЛОВАЯ КАРТА ====================
    # Изменен aspect="auto" для растягивания по ширине окна
    heat_img = ax_heat.imshow(
        np.zeros((100, 100)), origin="lower", cmap="inferno", aspect="auto",
        extent=[LATERAL_MIN, LATERAL_MAX, DEPTH_MIN, DEPTH_MAX], interpolation='nearest'
    )
    
    # Добавление цветовой шкалы (colorbar) для строгой оценки
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

    # Гистограмма ЧДД
    bar = ax_bar.barh(["BPM (FFT)"], [0], color="#41ab5d")
    ax_bar.set_xlim(0, 40)
    ax_bar.set_xlabel("Breaths / Min")
    ax_bar.grid(True, axis="x", alpha=0.5, linestyle='--')

    txt_stat = fig.text(0.02, 0.96, "", fontsize=11, family='monospace',
                        bbox=dict(boxstyle="round,pad=0.5", fc="#f8f9fa", ec="#dee2e6", alpha=0.95))
    
    plt.tight_layout(rect=[0, 0, 1, 0.95]) # Оптимизация отступов

    def update(_):
        nonlocal breath_fill
        if breath_fill is not None:
            breath_fill.remove()
            breath_fill = None

        mon.poll()
        py_bpm, detected, apnea, quality = mon.analyze_breath_fft()

        if not mon.times:
            return line_dist, line_trend, line_ac, bar, heat_img, heat_marker

        t = list(mon.times)
        d = list(mon.dist)
        a = list(mon.ac)
        depth = list(mon.depth)
        lateral = list(mon.lateral)
        tr = mon.baseline()

        # Обновление графиков
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

        # Вычисление сглаженной тепловой карты
        if depth and lateral:
            depth_arr = np.clip(depth, DEPTH_MIN, DEPTH_MAX)
            lateral_arr = np.clip(lateral, LATERAL_MIN, LATERAL_MAX)

            # Создаем 2D гистограмму (матрица 100x100)
            heat_raw, _, _ = np.histogram2d(
                lateral_arr, depth_arr, bins=100,
                range=[[LATERAL_MIN, LATERAL_MAX], [DEPTH_MIN, DEPTH_MAX]]
            )
            
            # Применяем Гауссово размытие для визуальной эстетики (настоящий Heatmap)
            heat_smooth = scipy.ndimage.gaussian_filter(heat_raw, sigma=1.8)

            heat_img.set_data(heat_smooth.T)
            vmax = max(0.1, np.max(heat_smooth))
            heat_img.set_clim(0, vmax)

            last_lat = lateral_arr[-1]
            last_dep = depth_arr[-1]
            heat_marker.set_data([last_lat], [last_dep])
            heat_marker.set_markerfacecolor('#00ff00' if mon.in_zone else '#ffae00')
        else:
            heat_img.set_data(np.zeros((100, 100)))
            heat_marker.set_data([], [])

        # Информационная панель
        tracking_state = "TRACKING" if mon.in_zone and mon.visible else ("VISIBLE" if mon.visible else "NO TARGET")
        bg_color = "#ffe3e3" if apnea else "#f8f9fa"
        txt_stat.set_bbox(dict(boxstyle="round,pad=0.5", fc=bg_color, ec="#dee2e6", alpha=0.95))
        
        txt_stat.set_text(
            f"SYSTEM STATE : {tracking_state:<10} | STATIONARY: {'YES' if mon.stationary else 'NO':<3} | APNEA: {'DETECTED ⚠️' if apnea else 'OK'}\n"
            f"ALGORITHM    : FW BPM = {mon.bpm:>4.1f} | PY BPM (FFT) = {py_bpm:>4.1f} \n"
            f"SIGNAL METRIC: BREATHING = {'YES' if detected else 'NO':<3} | QUALITY = {quality:>5.1f}%"
        )

        bar[0].set_width(py_bpm if detected else 0)

        return line_dist, line_trend, line_ac, bar, heat_img, heat_marker

    ani = animation.FuncAnimation(fig, update, interval=33, blit=False, cache_frame_data=False)
    plt.show()

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        pass
    finally:
        plt.close()