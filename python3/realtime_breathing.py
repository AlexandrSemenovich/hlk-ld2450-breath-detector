import sys
import time
import math
import numpy as np
import serial
from collections import deque
from enum import Enum

# DSP импорты
from scipy.signal import butter, filtfilt, hilbert, detrend

# PySide6 & PyQtGraph импорты
from PySide6.QtCore import QThread, Signal, Slot, QTimer
from PySide6.QtWidgets import (QApplication, QMainWindow, QVBoxLayout, QHBoxLayout, 
                               QWidget, QLabel, QPushButton, QMessageBox)
import pyqtgraph as pg

# --- Настройки ---
BAUD_RATE = 921600
BUFFER_SIZE = 500  # Буфер ~45 секунд при FS ≈ 11.2 Гц
FS_NOMINAL = 11.2

# НОВЫЕ РАЗМЕРЫ ЗОНЫ ДЫХАНИЯ (Ширина 500мм, Глубина 700мм)
TARGET_ZONE_X_MIN = -250
TARGET_ZONE_X_MAX = 250
TARGET_ZONE_Y_MIN = 800
TARGET_ZONE_Y_MAX = 1500


class AppState(Enum):
    IDLE = 1
    CALIBRATING = 2
    REVIEWING = 3
    READY = 4
    RUNNING = 5


class SerialReaderWorker(QThread):
    frame_received = Signal(list, int)

    def __init__(self, port):
        super().__init__()
        self.port = port
        self.running = True

    def run(self):
        try:
            ser = serial.Serial(self.port, BAUD_RATE, timeout=0.5)
            ser.reset_input_buffer()
            while self.running:
                line = ser.readline().decode('utf-8', errors='ignore').strip()
                if not line.startswith('R'): 
                    continue
                
                parts = line[1:].split(',')
                if len(parts) < 14: 
                    continue
                try:
                    targets = [{'x': int(parts[0+i*4]), 'y': int(parts[1+i*4]), 
                                'speed': int(parts[2+i*4]), 'res': int(parts[3+i*4])} for i in range(3)]
                    ts_ms = int(parts[12])
                    self.frame_received.emit(targets, ts_ms)
                except (ValueError, IndexError):
                    continue
            ser.close()
        except Exception as e:
            print(f"Ошибка COM-порта ({self.port}): {e}")

    def stop(self):
        self.running = False
        self.wait()


def extract_strict_breathing_extrema(y_filt, times_rel, envelope, min_prominence, fs):
    """ЖЕСТКИЙ АЛГОРИТМ ПОИСКА ВДОХОВ И ВЫДОХОВ (Гильберт)"""
    if len(y_filt) < 20:
        return np.array([], dtype=int), np.array([], dtype=int)

    analytic_signal = hilbert(y_filt)
    wrapped_phase = np.angle(analytic_signal)
    
    phase_unwrap = np.unwrap(wrapped_phase)
    dphase = np.gradient(phase_unwrap)

    raw_peak_candidates = []
    for i in range(1, len(wrapped_phase) - 1):
        if (wrapped_phase[i-1] < 0 and wrapped_phase[i] >= 0) and dphase[i] > 0:
            win = range(max(0, i - 3), min(len(y_filt), i + 4))
            best_idx = win[0] + np.argmax(y_filt[win])
            if envelope[best_idx] >= min_prominence:
                raw_peak_candidates.append(best_idx)

    phase_diff = np.diff(wrapped_phase)
    raw_trough_candidates = []
    for i in range(len(phase_diff)):
        if phase_diff[i] < -np.pi * 1.5 and dphase[i] > 0:
            win = range(max(0, i - 3), min(len(y_filt), i + 4))
            best_idx = win[0] + np.argmin(y_filt[win])
            if envelope[best_idx] >= min_prominence:
                raw_trough_candidates.append(best_idx)

    min_samples_dist = max(1, int(1.2 * fs))
    
    def filter_refractory(candidates, is_max=True):
        if not candidates:
            return []
        filtered = []
        candidates = sorted(list(set(candidates)))
        for c in candidates:
            if not filtered:
                filtered.append(c)
            else:
                if (c - filtered[-1]) < min_samples_dist:
                    if is_max and y_filt[c] > y_filt[filtered[-1]]:
                        filtered[-1] = c
                    elif not is_max and y_filt[c] < y_filt[filtered[-1]]:
                        filtered[-1] = c
                else:
                    filtered.append(c)
        return filtered

    peaks = filter_refractory(raw_peak_candidates, is_max=True)
    troughs = filter_refractory(raw_trough_candidates, is_max=False)

    events = []
    for p in peaks: events.append((p, 1, y_filt[p]))
    for t in troughs: events.append((t, -1, y_filt[t]))
    events.sort(key=lambda x: x[0])

    final_peaks = []
    final_troughs = []
    last_type = None
    last_event = None

    for ev in events:
        idx, ev_type, val = ev
        if last_type is None:
            last_type = ev_type
            last_event = ev
        elif ev_type == last_type:
            if ev_type == 1:
                if val > last_event[2]: last_event = ev
            else:
                if val < last_event[2]: last_event = ev
        else:
            if last_event[1] == 1: final_peaks.append(last_event[0])
            else: final_troughs.append(last_event[0])
            last_type = ev_type
            last_event = ev

    if last_event is not None:
        if last_event[1] == 1: final_peaks.append(last_event[0])
        else: final_troughs.append(last_event[0])

    return np.array(final_peaks, dtype=int), np.array(final_troughs, dtype=int)


class RespirationMonitor(QMainWindow):
    def __init__(self, com_port='COM3'):
        super().__init__()
        self.setWindowTitle("LD2450 Pro Respiratory Monitor — Medical DSP")
        self.resize(1400, 800)

        # Состояние системы
        self.state = AppState.IDLE
        self.calib_prominence = 10.0
        self._temp_calib_amp = 0.0

        # Буферы
        self.raw_y_buffer = deque(maxlen=BUFFER_SIZE)
        self.time_buffer = deque(maxlen=BUFFER_SIZE)
        self.pos_x_buffer = deque(maxlen=50) # Траектория за 5 секунд
        self.pos_y_buffer = deque(maxlen=50)
        self.latest_all_targets = []

        # Переменные стабильного трекинга цели
        self.last_raw_x = None
        self.last_raw_y = None
        self.ema_x = None  # Сглаженный X для отрисовки маркера
        self.ema_y = None  # Сглаженный Y для отрисовки маркера
        self.lost_frames = 0
        self.max_tracking_jump = 400  # Максимальный Евклидов прыжок (2D расстояние)
        self.current_slot = -1

        self.setup_filter(FS_NOMINAL)
        self.init_ui()

        self.serial_thread = SerialReaderWorker(port=com_port)
        self.serial_thread.frame_received.connect(self.process_new_frame)
        self.serial_thread.start()

        self.ui_timer = QTimer()
        self.ui_timer.timeout.connect(self.update_dsp_and_plots)
        self.ui_timer.start(200)

    def setup_filter(self, fs):
        self.fs = fs
        nyquist = 0.5 * self.fs
        lowcut = 0.1 / nyquist
        highcut = min(0.5 / nyquist, 0.99)
        self.b, self.a = butter(2, [lowcut, highcut], btype='band')

    def init_ui(self):
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        main_layout = QVBoxLayout(central_widget)

        # --- Кнопки управления ---
        control_layout = QHBoxLayout()
        self.btn_reset = QPushButton("⏹ Сброс")
        self.btn_start_calib = QPushButton("⚙️ Калибровка (3 вдоха)")
        self.btn_stop_calib = QPushButton("⏸ Проверить калибровку")
        self.btn_approve = QPushButton("✅ Подтвердить")
        self.btn_reject = QPushButton("❌ Перекалибровать")
        self.btn_start_exp = QPushButton("▶️ Запуск мониторинга")
        
        self.btn_reset.clicked.connect(self.set_idle)
        self.btn_start_calib.clicked.connect(self.start_calibration)
        self.btn_stop_calib.clicked.connect(self.review_calibration)
        self.btn_approve.clicked.connect(self.approve_calibration)
        self.btn_reject.clicked.connect(self.start_calibration)
        self.btn_start_exp.clicked.connect(self.start_experiment)
        
        for btn in [self.btn_reset, self.btn_start_calib, self.btn_stop_calib, 
                    self.btn_approve, self.btn_reject, self.btn_start_exp]:
            btn.setFixedHeight(38)
            btn.setStyleSheet("font-size: 13px; font-weight: bold;")
            control_layout.addWidget(btn)
            
        main_layout.addLayout(control_layout)

        # --- Информационная панель метрик ---
        panel_layout = QHBoxLayout()
        self.status_label = QLabel("Статус: ОЖИДАНИЕ")
        self.status_label.setStyleSheet("font-size: 13px; font-weight: bold; color: #FFA500;")
        self.zone_label = QLabel("Позиция: Поиск цели...")
        self.zone_label.setStyleSheet("font-size: 13px; font-weight: bold; color: #888888; margin-left: 10px;")
        self.sqi_label = QLabel("SQI: --")
        self.sqi_label.setStyleSheet("font-size: 13px; font-weight: bold; color: #888888; margin-left: 10px;")
        self.ie_label = QLabel("I/E Ratio: --")
        self.ie_label.setStyleSheet("font-size: 15px; font-weight: bold; color: #FFD700; margin-left: 15px;")
        self.bpm_label = QLabel("BPM: --")
        self.bpm_label.setStyleSheet("font-size: 18px; font-weight: bold; color: #00FF00; margin-left: 15px;")
        
        panel_layout.addWidget(self.status_label)
        panel_layout.addWidget(self.zone_label)
        panel_layout.addWidget(self.sqi_label)
        panel_layout.addStretch()
        panel_layout.addWidget(self.ie_label)
        panel_layout.addWidget(self.bpm_label)
        main_layout.addLayout(panel_layout)

        # --- Графики PyQtGraph ---
        pg.setConfigOptions(antialias=True)
        self.graph_view = pg.GraphicsLayoutWidget()
        main_layout.addWidget(self.graph_view)

        # === ЛЕВАЯ КОЛОНКА: ДЫХАТЕЛЬНЫЕ ГРАФИКИ ===
        self.p1 = self.graph_view.addPlot(row=0, col=0, title="Сырой сигнал дальности Y (мм)")
        self.p1.showGrid(x=True, y=True)
        self.raw_curve = self.p1.plot(pen=pg.mkPen(color='w', width=1.2))
        
        self.p2 = self.graph_view.addPlot(row=1, col=0, title="Дыхание (Синий), Огибающая (Желтый) и Жесткие пики")
        self.p2.showGrid(x=True, y=True)
        self.filtered_curve = self.p2.plot(pen=pg.mkPen(color='#1E90FF', width=2))
        self.envelope_curve = self.p2.plot(pen=pg.mkPen(color='#FFD700', width=1.5, style=pg.QtCore.Qt.DashLine))
        self.peaks_scatter = pg.ScatterPlotItem(size=10, pen=pg.mkPen('r'), brush=pg.mkBrush('r'), symbol='t1')
        self.troughs_scatter = pg.ScatterPlotItem(size=10, pen=pg.mkPen('g'), brush=pg.mkBrush('g'), symbol='t')
        self.p2.addItem(self.peaks_scatter)
        self.p2.addItem(self.troughs_scatter)

        # === ПРАВАЯ КОЛОНКА: 2D КАРТА ПОЗИЦИОНИРОВАНИЯ ===
        self.p_radar = self.graph_view.addPlot(row=0, col=1, rowspan=2, title="2D Карта (Сглаженный Трекинг 24/7)")
        self.p_radar.setXRange(-1000, 1000)
        self.p_radar.setYRange(0, 2500)
        self.p_radar.setLabel('bottom', "Смещение X (мм)")
        self.p_radar.setLabel('left', "Дистанция Y (мм)")
        self.p_radar.showGrid(x=True, y=True)

        # Зона Дыхания
        opt_box_x = [TARGET_ZONE_X_MIN, TARGET_ZONE_X_MAX, TARGET_ZONE_X_MAX, TARGET_ZONE_X_MIN, TARGET_ZONE_X_MIN]
        opt_box_y = [TARGET_ZONE_Y_MIN, TARGET_ZONE_Y_MIN, TARGET_ZONE_Y_MAX, TARGET_ZONE_Y_MAX, TARGET_ZONE_Y_MIN]
        self.p_radar.plot(opt_box_x, opt_box_y, pen=pg.mkPen(color='#00FF00', width=2, style=pg.QtCore.Qt.DashLine))

        opt_text = pg.TextItem(f"ЗОНА {TARGET_ZONE_X_MAX*2}x{TARGET_ZONE_Y_MAX-TARGET_ZONE_Y_MIN} мм", color=(0, 255, 0), anchor=(0.5, 0.5))
        opt_text.setPos(0, (TARGET_ZONE_Y_MIN + TARGET_ZONE_Y_MAX) / 2)
        self.p_radar.addItem(opt_text)

        # Радар (0,0)
        self.p_radar.plot([0], [0], pen=None, symbol='s', symbolSize=12, symbolBrush='m')

        # Траектория
        self.trail_curve = self.p_radar.plot(pen=pg.mkPen(color='#00FFFF', width=2, style=pg.QtCore.Qt.DotLine))
        self.tracked_scatter = pg.ScatterPlotItem(size=16, pen=pg.mkPen('c'), brush=pg.mkBrush('c'), symbol='o')
        self.p_radar.addItem(self.tracked_scatter)

        # Прочие объекты
        self.other_scatter = pg.ScatterPlotItem(size=10, pen=pg.mkPen('#FFA500'), brush=pg.mkBrush('#FFA500'), symbol='x')
        self.p_radar.addItem(self.other_scatter)

        self.update_ui_state()

    # --- Управление FSM ---
    def set_idle(self):
        self.state = AppState.IDLE
        self.status_label.setText("Статус: ОЖИДАНИЕ. Встаньте в зеленую зону.")
        self.bpm_label.setText("BPM: --")
        self.ie_label.setText("I/E Ratio: --")
        self.sqi_label.setText("SQI: --")
        self.calib_prominence = 0
        self.raw_y_buffer.clear()
        self.time_buffer.clear()
        self.update_ui_state()

    def start_calibration(self):
        self.raw_y_buffer.clear()
        self.time_buffer.clear()
        self.state = AppState.CALIBRATING
        self.status_label.setText("Статус: КАЛИБРОВКА. Сделайте 3 спокойных глубоких вдоха!")
        self.update_ui_state()

    def review_calibration(self):
        if len(self.raw_y_buffer) < 80:
            QMessageBox.warning(self, "Ошибка", "Недостаточно данных. Подождите не менее 10 секунд.")
            return
        
        self.state = AppState.REVIEWING
        raw_y = detrend(np.array(self.raw_y_buffer))
        times_rel = np.array(self.time_buffer) - self.time_buffer[0]
        y_filt = filtfilt(self.b, self.a, raw_y)
        envelope = np.abs(hilbert(y_filt))
        
        peaks, _ = extract_strict_breathing_extrema(y_filt, times_rel, envelope, 2.0, self.fs)
        
        self.status_label.setText(f"ОСМОТР: Найдено {len(peaks)} вдохов. Проверьте правильность пиков.")
        if len(y_filt) > 0:
            self._temp_calib_amp = np.ptp(y_filt)
        
        self.update_ui_state()

    def approve_calibration(self):
        self.calib_prominence = max(5.0, self._temp_calib_amp * 0.35)
        self.state = AppState.READY
        self.status_label.setText("Статус: ОТКАЛИБРОВАНО. Готово к работе.")
        self.update_ui_state()

    def start_experiment(self):
        self.raw_y_buffer.clear()
        self.time_buffer.clear()
        self.state = AppState.RUNNING
        self.status_label.setText("Статус: ИДЕТ МОНИТОРИНГ...")
        self.update_ui_state()

    def update_ui_state(self):
        self.btn_reset.setVisible(True)
        self.btn_start_calib.setVisible(self.state in [AppState.IDLE, AppState.READY])
        self.btn_stop_calib.setVisible(self.state == AppState.CALIBRATING)
        self.btn_approve.setVisible(self.state == AppState.REVIEWING)
        self.btn_reject.setVisible(self.state == AppState.REVIEWING)
        self.btn_start_exp.setVisible(self.state == AppState.READY)

    @Slot(list, int)
    def process_new_frame(self, targets, ts_ms):
        """Интеллектуальный 2D трекинг со сглаживанием координат (EMA)"""
        best_target = None
        min_dist = float('inf')
        self.latest_all_targets = targets

        for idx, t in enumerate(targets):
            if t['res'] > 0:
                if self.last_raw_x is None:
                    # Если цели нет, берем самую ближнюю к радару (по Y)
                    if t['y'] < min_dist:
                        min_dist = t['y']
                        best_target = t
                        self.current_slot = idx
                else:
                    # Трекинг по 2D дистанции (Евклидово расстояние)
                    dist = math.hypot(t['x'] - self.last_raw_x, t['y'] - self.last_raw_y)
                    if dist < min_dist and dist < self.max_tracking_jump:
                        min_dist = dist
                        best_target = t
                        self.current_slot = idx

        if best_target is not None:
            self.lost_frames = 0
            raw_x = best_target['x']
            raw_y = best_target['y']
            
            self.last_raw_x = raw_x
            self.last_raw_y = raw_y

            # Амортизатор маркера (EMA): чем меньше ALPHA, тем плавнее движение маркера на UI
            ALPHA = 0.1 
            if self.ema_x is None:
                self.ema_x = float(raw_x)
                self.ema_y = float(raw_y)
            else:
                self.ema_x = raw_x * ALPHA + self.ema_x * (1.0 - ALPHA)
                self.ema_y = raw_y * ALPHA + self.ema_y * (1.0 - ALPHA)

            self.pos_x_buffer.append(self.ema_x)
            self.pos_y_buffer.append(self.ema_y)

            # В буфер дыхания мы пишем СТРОГО СЫРЫЕ данные (чтобы не убить фильтром дыхание!)
            if self.state in [AppState.CALIBRATING, AppState.RUNNING]:
                self.raw_y_buffer.append(raw_y)
                self.time_buffer.append(ts_ms / 1000.0)
        else:
            self.lost_frames += 1
            # Если цель исчезла дольше чем на ~1.5 секунды (15 кадров), сбрасываем захват
            if self.lost_frames > 15:
                self.last_raw_x = None
                self.last_raw_y = None
                self.ema_x = None
                self.ema_y = None
                
            # Экстраполяция при потере кадра (чтобы графики не останавливались)
            if self.state in [AppState.CALIBRATING, AppState.RUNNING] and len(self.raw_y_buffer) > 0:
                self.raw_y_buffer.append(self.raw_y_buffer[-1])
                if len(self.time_buffer) > 1:
                    dt = self.time_buffer[-1] - self.time_buffer[-2]
                    self.time_buffer.append(self.time_buffer[-1] + dt)
                else:
                    self.time_buffer.append(time.time())

    def update_dsp_and_plots(self):
        # 1. ОБНОВЛЕНИЕ 2D КАРТЫ ПОЗИЦИОНИРОВАНИЯ
        if len(self.pos_x_buffer) > 0:
            cur_x = self.pos_x_buffer[-1]
            cur_y = self.pos_y_buffer[-1]
            
            self.trail_curve.setData(list(self.pos_x_buffer), list(self.pos_y_buffer))
            self.tracked_scatter.setData(x=[cur_x], y=[cur_y])
            
            in_zone = (TARGET_ZONE_X_MIN <= cur_x <= TARGET_ZONE_X_MAX) and \
                      (TARGET_ZONE_Y_MIN <= cur_y <= TARGET_ZONE_Y_MAX)
            
            if in_zone:
                zone_str = "В ЗОНЕ"
                zone_color = "#00FF00"
            else:
                zone_str = f"ВНЕ ЗОНЫ (X:{int(cur_x)}, Y:{int(cur_y)})"
                zone_color = "#FFA500"
            self.zone_label.setText(f"Позиция: {zone_str}")
            self.zone_label.setStyleSheet(f"font-size: 13px; font-weight: bold; color: {zone_color};")
        else:
            self.tracked_scatter.clear()
            self.trail_curve.clear()
            self.zone_label.setText("Позиция: ИЩУ ЦЕЛЬ...")
            self.zone_label.setStyleSheet("font-size: 13px; font-weight: bold; color: #FF4500;")

        # Отрисовка остальных целей
        other_x, other_y = [], []
        for idx, t in enumerate(self.latest_all_targets):
            if t['res'] > 0 and idx != self.current_slot:
                other_x.append(t['x'])
                other_y.append(t['y'])

        if len(other_x) > 0:
            self.other_scatter.setData(x=other_x, y=other_y)
        else:
            self.other_scatter.clear()

        # 2. ОБРАБОТКА ДЫХАТЕЛЬНОГО СИГНАЛА
        if len(self.raw_y_buffer) < 60:
            return

        raw_y = np.array(self.raw_y_buffer)
        times = np.array(self.time_buffer)
        times_rel = times - times[0]
        
        dt_mean = np.mean(np.diff(times_rel))
        if dt_mean <= 0: return

        fs_actual = 1.0 / dt_mean
        if abs(fs_actual - self.fs) > 0.5:
            self.setup_filter(fs_actual)

        raw_y_detrend = detrend(raw_y)
        y_filtered = filtfilt(self.b, self.a, raw_y_detrend)

        analytic_signal = hilbert(y_filtered)
        amplitude_envelope = np.abs(analytic_signal)

        min_amp = (self.calib_prominence * 0.3) if self.state == AppState.RUNNING else (np.ptp(y_filtered) * 0.15)
        min_amp = max(2.0, min_amp)

        peaks, troughs = extract_strict_breathing_extrema(y_filtered, times_rel, amplitude_envelope, min_amp, self.fs)

        ie_ratios = []
        for p in peaks:
            tr_before = troughs[troughs < p]
            tr_after = troughs[troughs > p]
            if len(tr_before) > 0 and len(tr_after) > 0:
                ti = times_rel[p] - times_rel[tr_before[-1]]
                te = times_rel[tr_after[0]] - times_rel[p]
                if ti > 0.2 and te > 0.2: ie_ratios.append(ti / te)

        avg_ie_ratio = np.median(ie_ratios) if len(ie_ratios) > 0 else 0.0

        env_tail = amplitude_envelope[-min(100, len(amplitude_envelope)):]
        env_mean = np.mean(env_tail)
        cv = np.std(env_tail) / (env_mean + 1e-6)

        if env_mean < (self.calib_prominence * 0.3):
            sqi_str = "СЛАБЫЙ СИГНАЛ"
            sqi_color = "#FF4500"
        elif cv > 1.2:
            sqi_str = "ДВИЖЕНИЕ / ШУМ"
            sqi_color = "#FFA500"
        else:
            sqi_str = "ОТЛИЧНО"
            sqi_color = "#00FF00"

        bpm_peaks = 0.0
        if len(peaks) >= 2:
            avg_period = np.median(np.diff(times_rel[peaks]))
            if avg_period > 0: bpm_peaks = 60.0 / avg_period

        self.raw_curve.setData(times_rel, raw_y)
        self.filtered_curve.setData(times_rel, y_filtered)
        self.envelope_curve.setData(times_rel, amplitude_envelope)

        if len(peaks) > 0: self.peaks_scatter.setData(x=times_rel[peaks], y=y_filtered[peaks])
        else: self.peaks_scatter.clear()

        if len(troughs) > 0: self.troughs_scatter.setData(x=times_rel[troughs], y=y_filtered[troughs])
        else: self.troughs_scatter.clear()

        if self.state in [AppState.CALIBRATING, AppState.RUNNING]:
            self.sqi_label.setText(f"SQI: {sqi_str}")
            self.sqi_label.setStyleSheet(f"font-size: 13px; font-weight: bold; color: {sqi_color};")
            
            if avg_ie_ratio > 0: self.ie_label.setText(f"I/E Ratio: 1 : {1.0 / avg_ie_ratio:.1f}")
            else: self.ie_label.setText("I/E Ratio: --")

            if self.state == AppState.RUNNING: self.bpm_label.setText(f"BPM: {bpm_peaks:.1f}")

    def closeEvent(self, event):
        self.serial_thread.stop()
        event.accept()


if __name__ == "__main__":
    app = QApplication(sys.argv)
    app.setStyle('Fusion')
    
    monitor = RespirationMonitor(com_port='COM3') 
    monitor.show()
    sys.exit(app.exec())