import sys
import time
import serial
import numpy as np
from collections import deque

from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout,
    QHBoxLayout, QLabel, QPushButton, QComboBox,
    QSpinBox, QGroupBox, QFormLayout
)
from PyQt5.QtCore import QTimer
from PyQt5.QtGui import QFont

import matplotlib.pyplot as plt
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.colors import LinearSegmentedColormap


class LD2450Monitor(QMainWindow):
    def __init__(self):
        super().__init__()

        self.setWindowTitle("LD2450 — RAW + Target0 + Heatmap (fading trails)")
        self.resize(1350, 850)

        # ---- Heatmap grid (fixed) ----
        self.max_range = 6000  # mm
        self.bins_x = 90
        self.bins_y = 90

        # Heat buffer with fade
        self.heat = np.zeros((self.bins_y, self.bins_x), dtype=np.float32)
        self.last_ts_ms = None

        # Trail points (time-window)
        self.trail_points = deque()
        self.history_max = 5000

        # Kernel for “blur” at each point (so trails fade smoothly)
        self.kernel_r = 4  # radius in bins
        self.kernel = self._make_gaussian_kernel(size=2 * self.kernel_r + 1, sigma=1.6)

        self.init_ui()

        self.timer = QTimer()
        self.timer.timeout.connect(self.read_serial)
        self.timer.setInterval(10)

    # ---------------- UI ----------------

    def init_ui(self):
        central = QWidget()
        self.setCentralWidget(central)
        main_layout = QHBoxLayout(central)

        # ----- Left panel -----
        left = QWidget()
        left_layout = QVBoxLayout(left)

        # Connection group
        port_group = QGroupBox("Подключение")
        form = QFormLayout()

        self.port_combo = QComboBox()
        self.port_combo.addItems(["COM3", "COM4", "COM5", "COM6", "/dev/ttyUSB0"])
        form.addRow("Порт:", self.port_combo)

        self.baud = QSpinBox()
        self.baud.setRange(9600, 2000000)
        self.baud.setSingleStep(115200)
        self.baud.setValue(921600)
        form.addRow("Скорость:", self.baud)

        self.btn_connect = QPushButton("Подключиться")
        self.btn_connect.clicked.connect(self.toggle_connection)
        form.addRow(self.btn_connect)

        port_group.setLayout(form)
        left_layout.addWidget(port_group)

        # Settings group
        settings_group = QGroupBox("Настройки визуализации")
        sform = QFormLayout()

        self.fade_time_ms = QSpinBox()
        self.fade_time_ms.setRange(200, 20000)
        self.fade_time_ms.setSingleStep(100)
        self.fade_time_ms.setValue(3000)
        sform.addRow("Следы исчезают через (ms):", self.fade_time_ms)

        self.point_intensity = QSpinBox()
        self.point_intensity.setRange(10, 200)
        self.point_intensity.setSingleStep(5)
        self.point_intensity.setValue(80)
        sform.addRow("Интенсивность (контраст):", self.point_intensity)

        self.trail_time_ms = QSpinBox()
        self.trail_time_ms.setRange(200, 20000)
        self.trail_time_ms.setSingleStep(100)
        self.trail_time_ms.setValue(2500)
        sform.addRow("Длина трейла (time window, ms):", self.trail_time_ms)

        self.trail_points_max = QSpinBox()
        self.trail_points_max.setRange(50, 5000)
        self.trail_points_max.setSingleStep(50)
        self.trail_points_max.setValue(1200)
        sform.addRow("Макс. точек в трейле:", self.trail_points_max)

        settings_group.setLayout(sform)
        left_layout.addWidget(settings_group)

        # Parsed target0 group
        parsed_group = QGroupBox("Target 0 + Timestamp + Frame ID")
        parsed_layout = QVBoxLayout()

        self.data_label = QLabel("Ожидание данных...")
        self.data_label.setFont(QFont("Consolas", 11))
        self.data_label.setStyleSheet("""
            QLabel {
                background-color: #eef3f7;
                border: 1px solid #cfd8dc;
                border-radius: 6px;
                padding: 10px;
                color: #263238;
            }
        """)
        self.data_label.setWordWrap(True)
        parsed_layout.addWidget(self.data_label)

        parsed_group.setLayout(parsed_layout)
        left_layout.addWidget(parsed_group)

        # RAW group
        raw_group = QGroupBox("RAW строка из COM порта (как пришло)")
        raw_layout = QVBoxLayout()

        self.raw_label = QLabel("Ожидание RAW строки...")
        self.raw_label.setFont(QFont("Consolas", 10))
        self.raw_label.setStyleSheet("""
            QLabel {
                background-color: #111;
                color: #00FF88;
                border: 1px solid #333;
                border-radius: 6px;
                padding: 10px;
            }
        """)
        self.raw_label.setWordWrap(True)
        raw_layout.addWidget(self.raw_label)

        raw_group.setLayout(raw_layout)
        left_layout.addWidget(raw_group)

        # Stats
        self.stats_label = QLabel("Статистика:\n—")
        self.stats_label.setFont(QFont("Consolas", 11))
        self.stats_label.setStyleSheet("""
            QLabel {
                background-color: #eef3f7;
                border: 1px solid #cfd8dc;
                border-radius: 6px;
                padding: 10px;
            }
        """)
        left_layout.addWidget(self.stats_label)

        left_layout.addStretch()

        btn_clear = QPushButton("Очистить карту")
        btn_clear.clicked.connect(self.clear_map)
        left_layout.addWidget(btn_clear)

        main_layout.addWidget(left, stretch=1)

        # ----- Right: plot -----
        self.figure = plt.figure(figsize=(9, 9), facecolor="white")
        self.canvas = FigureCanvas(self.figure)
        self.ax = self.figure.add_subplot(111)

        self.ax.set_facecolor("#ffffff")
        self.ax.grid(True, alpha=0.25)

        # fixed axis limits (map MUST NOT move)
        self.ax.set_xlim(-self.max_range, self.max_range)
        self.ax.set_ylim(0, self.max_range)
        self.ax.set_aspect("equal", adjustable="box")
        self.ax.set_autoscale_on(False)

        # colormap (light)
        colors = ['#ffffff', '#dbeafe', '#93c5fd', '#60a5fa',
                  '#34d399', '#fde047', '#fb923c', '#ef4444']
        self.cmap = LinearSegmentedColormap.from_list("light_heat", colors)

        self.ax.set_xlabel("X (mm)")
        self.ax.set_ylabel("Y (mm)")
        self.ax.set_title("Heatmap (fading trails) — Target0 only", pad=15)

        self.heat_img = self.ax.imshow(
            self.heat,
            origin="lower",
            cmap=self.cmap,
            extent=[-self.max_range, self.max_range, 0, self.max_range],
            interpolation="bilinear",
            vmin=0,
            vmax=1
        )

        # Trail line + current point
        self.trail_line, = self.ax.plot([], [], color="#1976d2", lw=2, alpha=0.85, zorder=4)
        self.current_point = self.ax.scatter(
            [], [], c="#d32f2f", s=130, edgecolors="black", linewidths=1.2, zorder=6
        )

        main_layout.addWidget(self.canvas, stretch=4)

        self.canvas.draw_idle()

    # ---------------- Helpers ----------------

    def _make_gaussian_kernel(self, size=9, sigma=1.5):
        r = size // 2
        ax = np.arange(-r, r + 1, dtype=np.float32)
        xx, yy = np.meshgrid(ax, ax)
        k = np.exp(-(xx * xx + yy * yy) / (2.0 * sigma * sigma))
        k /= np.max(k) if np.max(k) > 0 else 1.0
        return k.astype(np.float32)

    def _mm_to_bin(self, x_mm, y_mm):
        # Clamp to map range; returns integer indices (iy, ix) in [0..bins-1]
        x_mm = float(x_mm)
        y_mm = float(y_mm)

        x_mm = max(-self.max_range, min(self.max_range, x_mm))
        y_mm = max(0.0, min(self.max_range, y_mm))

        # x range: [-max_range, +max_range]
        t_x = (x_mm + self.max_range) / (2.0 * self.max_range)  # 0..1
        # y range: [0, max_range]
        t_y = (y_mm) / self.max_range  # 0..1

        ix = int(t_x * (self.bins_x - 1) + 0.5)
        iy = int(t_y * (self.bins_y - 1) + 0.5)

        ix = max(0, min(self.bins_x - 1, ix))
        iy = max(0, min(self.bins_y - 1, iy))
        return iy, ix

    def _add_kernel_to_heat(self, iy, ix, add_value):
        # Add gaussian kernel around (iy, ix) safely (with clipping)
        r = self.kernel_r
        k = self.kernel
        kh, kw = k.shape

        y0 = iy - r
        y1 = iy + r + 1
        x0 = ix - r
        x1 = ix + r + 1

        # Clip to heat bounds
        yy0 = max(0, y0)
        yy1 = min(self.bins_y, y1)
        xx0 = max(0, x0)
        xx1 = min(self.bins_x, x1)

        # Corresponding kernel slice
        ky0 = yy0 - y0
        ky1 = ky0 + (yy1 - yy0)
        kx0 = xx0 - x0
        kx1 = kx0 + (xx1 - xx0)

        if yy0 < yy1 and xx0 < xx1:
            self.heat[yy0:yy1, xx0:xx1] += k[ky0:ky1, kx0:kx1] * add_value

    def _format_ts(self, ts_ms):
        # Timestamp in ms "как пришло". Additionally show readable only if it looks like Unix epoch.
        try:
            ts_ms = int(ts_ms)
        except:
            return "—"

        if ts_ms <= 0:
            return f"{ts_ms} ms"

        # If looks like Unix epoch ms (rough heuristic)
        if ts_ms > 10**12:
            dt = time.localtime(ts_ms / 1000.0)
            readable = time.strftime("%H:%M:%S", dt)
            ms_part = ts_ms % 1000
            return f"{ts_ms} ms (Local: {readable}.{ms_part:03d})"
        else:
            return f"{ts_ms} ms (как в пакете)"

    # ---------------- Serial ----------------

    def toggle_connection(self):
        if hasattr(self, "ser") and self.ser and self.ser.is_open:
            self.ser.close()
            self.timer.stop()
            self.btn_connect.setText("Подключиться")
            return

        try:
            self.ser = serial.Serial(
                self.port_combo.currentText(),
                self.baud.value(),
                timeout=0.05
            )
            self.timer.start()
            self.btn_connect.setText("Отключиться")
            print("✅ Подключено")
        except Exception as e:
            print("❌ Ошибка подключения:", e)

    def read_serial(self):
        if not hasattr(self, "ser") or not self.ser.is_open:
            return

        try:
            raw_line = self.ser.readline().decode("ascii", errors="ignore").strip()
            if not raw_line.startswith("R"):
                return

            parts = raw_line[1:].split(",")
            if len(parts) < 14:
                return

            # Target0 fields
            x0 = int(parts[0])
            y0 = int(parts[1])
            spd0 = int(parts[2])
            res0 = int(parts[3])

            # Timestamp and frame id
            frame_id = parts[13]
            try:
                ts_ms = int(parts[12])
            except:
                ts_ms = -1

            # ---- Update heat fading ----
            self._update_heat_and_trail(x0, y0, spd0, res0, ts_ms)

            # ---- Update panels ----
            self.update_parsed_panel(x0, y0, spd0, res0, ts_ms, frame_id)
            self.update_raw_panel(raw_line)

        except Exception:
            # Keep UI alive; ignore broken lines
            return

    def _update_heat_and_trail(self, x0, y0, spd0, res0, ts_ms):
        # decay by real time between frames (using ts_ms if valid)
        tau_ms = max(1, int(self.fade_time_ms.value()))
        if self.last_ts_ms is not None and ts_ms is not None and ts_ms > 0 and self.last_ts_ms > 0:
            dt = ts_ms - self.last_ts_ms
            if dt < 0:
                dt = 0
            decay = float(np.exp(-dt / tau_ms))
        else:
            # fallback decay per update if timestamp is missing
            decay = float(np.exp(-10.0 / tau_ms))  # ~10 ms assumed between polls

        self.heat *= decay

        # Add heat at current point
        iy, ix = self._mm_to_bin(x0, y0)
        add_value = float(self.point_intensity.value()) / 80.0  # scale
        self._add_kernel_to_heat(iy, ix, add_value)

        self.last_ts_ms = ts_ms if (ts_ms is not None and ts_ms > 0) else self.last_ts_ms

        # Update heat image scaling
        vmax_now = float(np.max(self.heat))
        if vmax_now < 1e-6:
            vmax_now = 1.0

        # intensity spinbox also affects contrast range
        contrast = float(self.point_intensity.value()) / 100.0
        vmax = max(1e-3, vmax_now * max(0.5, contrast))
        self.heat_img.set_data(self.heat)
        self.heat_img.set_clim(0.0, vmax)

        # ---- Trail line with fading via time window ----
        self.trail_points.append((x0, y0, ts_ms))
        while len(self.trail_points) > self.history_max:
            self.trail_points.popleft()

        # time-window prune if ts_ms is valid
        points_list = list(self.trail_points)
        if ts_ms is not None and ts_ms > 0:
            cutoff = ts_ms - int(self.trail_time_ms.value())
            pruned = [(px, py, pts_ts) for (px, py, pts_ts) in points_list if (pts_ts is not None and pts_ts > 0 and pts_ts >= cutoff)]
            # If all pruned due to missing ts, keep last N
            if len(pruned) == 0:
                pruned = points_list[-int(self.trail_points_max.value()):]
            points_list = pruned
        else:
            points_list = points_list[-int(self.trail_points_max.value()):]

        xs = [p[0] for p in points_list]
        ys = [p[1] for p in points_list]

        self.trail_line.set_data(xs, ys)
        self.current_point.set_offsets(np.array([[x0, y0]], dtype=np.float64))

        # ---- Stats ----
        self._update_stats(points_list)

        self.canvas.draw_idle()

    def _update_stats(self, points_list):
        if len(points_list) < 2:
            self.stats_label.setText(f"Статистика:\nТочек в трейле: {len(points_list)}")
            return

        xs = np.array([p[0] for p in points_list], dtype=np.float64)
        ys = np.array([p[1] for p in points_list], dtype=np.float64)

        dist_mm = np.sqrt(np.diff(xs) ** 2 + np.diff(ys) ** 2).sum()
        dist_m = dist_mm / 1000.0
        y_max = float(np.max(ys)) if len(ys) else 0.0

        # show heat energy for debugging
        heat_sum = float(np.sum(self.heat))

        self.stats_label.setText(
            "Статистика:\n"
            f"Точек в трейле: {len(points_list)}\n"
            f"Макс. Y: {y_max:.0f} мм\n"
            f"Пройдено ≈ {dist_m:.2f} м\n"
            f"Heat sum: {heat_sum:.1f}"
        )

    # ---------------- Panels ----------------

    def update_parsed_panel(self, x, y, spd, res, ts_ms, frame_id):
        self.data_label.setText(
            "TARGET 0\n"
            "--------------------------------\n"
            f"X: {x:6d} мм\n"
            f"Y: {y:6d} мм\n"
            f"Скорость: {spd:6d} см/с\n"
            f"Разрешение: {res:6d} мм\n\n"
            f"Timestamp ts_ms: {self._format_ts(ts_ms)}\n"
            f"Frame ID: {frame_id}"
        )

    def update_raw_panel(self, raw_line):
        # Show raw exactly as received (already .strip() removes ending \r\n)
        self.raw_label.setText(raw_line)

    # ---------------- Clear ----------------

    def clear_map(self):
        # Must NOT call ax.clear(). Map should be fixed.
        self.heat[:] = 0.0
        self.heat_img.set_data(self.heat)
        self.heat_img.set_clim(0.0, 1.0)

        self.trail_points.clear()
        self.trail_line.set_data([], [])
        self.current_point.set_offsets(np.array([[np.nan, np.nan]], dtype=np.float64))

        self.stats_label.setText("Статистика:\n—")
        self.canvas.draw_idle()

    def closeEvent(self, event):
        try:
            if hasattr(self, "ser") and self.ser and self.ser.is_open:
                self.ser.close()
        finally:
            event.accept()


if __name__ == "__main__":
    app = QApplication(sys.argv)
    app.setStyle("Fusion")

    app.setStyleSheet("""
        QMainWindow { background-color: #f5f7fa; }

        QGroupBox {
            font-weight: bold;
            border: 1px solid #cfd8dc;
            border-radius: 8px;
            margin-top: 12px;
            background: white;
        }
        QGroupBox::title {
            subcontrol-origin: margin;
            left: 10px;
            padding: 0 5px 0 5px;
        }

        QPushButton {
            background-color: #1976d2;
            color: white;
            border-radius: 6px;
            padding: 6px;
        }
        QPushButton:hover {
            background-color: #1565c0;
        }
        QComboBox, QSpinBox { padding: 4px; }
    """)

    window = LD2450Monitor()
    window.show()
    sys.exit(app.exec_())