import sys
import numpy as np
import serial
from collections import deque
from enum import Enum
# Импортируем всё необходимое для DSP
from scipy.signal import butter, filtfilt, hilbert, detrend, find_peaks
from PySide6.QtCore import QThread, Signal, Slot, QTimer
from PySide6.QtWidgets import (QApplication, QMainWindow, QVBoxLayout, QHBoxLayout, 
                               QWidget, QLabel, QPushButton, QMessageBox)
import pyqtgraph as pg

# --- Настройки ---
BAUD_RATE = 921600
BUFFER_SIZE = 500  # Увеличим буфер, чтобы точно влезли 3 глубоких вдоха (~45 секунд)
FS_NOMINAL = 11.2

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
                if not line.startswith('R'): continue
                
                parts = line[1:].split(',')
                if len(parts) < 14: continue
                try:
                    targets = [{'x': int(parts[0+i*4]), 'y': int(parts[1+i*4]), 
                                'speed': int(parts[2+i*4]), 'res': int(parts[3+i*4])} for i in range(3)]
                    ts_ms = int(parts[12])
                    self.frame_received.emit(targets, ts_ms)
                except (ValueError, IndexError):
                    continue
            ser.close()
        except Exception as e:
            print(f"Ошибка COM-порта: {e}")

    def stop(self):
        self.running = False
        self.wait()


class RespirationMonitor(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("LD2450 Pro Monitor - Строгая валидация")
        self.resize(1100, 750)

        # Состояние системы
        self.state = AppState.IDLE
        self.calib_prominence = 0
        
        self.raw_y_buffer = deque(maxlen=BUFFER_SIZE)
        self.time_buffer = deque(maxlen=BUFFER_SIZE)
        
        self.last_tracked_y = None
        self.max_tracking_jump = 300
        self.current_slot = -1

        self.setup_filter(FS_NOMINAL)
        self.init_ui()

        self.serial_thread = SerialReaderWorker(port='COM3') # Замените на нужный порт
        self.serial_thread.frame_received.connect(self.process_new_frame)
        self.serial_thread.start()

        self.ui_timer = QTimer()
        self.ui_timer.timeout.connect(self.update_dsp_and_plots)
        self.ui_timer.start(200)

    def setup_filter(self, fs):
        self.fs = fs
        nyquist = 0.5 * self.fs
        highcut = 0.5 if 0.5 < nyquist else nyquist - 0.01
        self.b, self.a = butter(2, [0.1 / nyquist, highcut / nyquist], btype='band')

    def init_ui(self):
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        main_layout = QVBoxLayout(central_widget)

        # --- Динамическая панель кнопок ---
        control_layout = QHBoxLayout()
        
        self.btn_reset = QPushButton("⏹ Сброс системы")
        self.btn_start_calib = QPushButton("⚙️ Начать калибровку (3 вдоха)")
        self.btn_stop_calib = QPushButton("⏸ Остановить и проверить график")
        self.btn_approve = QPushButton("✅ Подтверждаю (Сохранить)")
        self.btn_reject = QPushButton("❌ Ошибка (Перекалибровать)")
        self.btn_start_exp = QPushButton("▶️ Запуск эксперимента")
        
        self.btn_reset.clicked.connect(self.set_idle)
        self.btn_start_calib.clicked.connect(self.start_calibration)
        self.btn_stop_calib.clicked.connect(self.review_calibration)
        self.btn_approve.clicked.connect(self.approve_calibration)
        self.btn_reject.clicked.connect(self.start_calibration)
        self.btn_start_exp.clicked.connect(self.start_experiment)
        
        # Стилизация и добавление в layout
        for btn in [self.btn_reset, self.btn_start_calib, self.btn_stop_calib, 
                    self.btn_approve, self.btn_reject, self.btn_start_exp]:
            btn.setFixedHeight(40)
            btn.setStyleSheet("font-size: 14px; font-weight: bold;")
            control_layout.addWidget(btn)
            
        main_layout.addLayout(control_layout)

        # --- Информационная панель ---
        panel_layout = QHBoxLayout()
        self.status_label = QLabel("Статус: ОЖИДАНИЕ")
        self.status_label.setStyleSheet("font-size: 14px; font-weight: bold; color: #FFA500;")
        
        self.prominence_label = QLabel("Порог: --")
        self.prominence_label.setStyleSheet("font-size: 14px; font-weight: bold; color: #00BFFF;")
        
        self.bpm_label = QLabel("BPM: --")
        self.bpm_label.setStyleSheet("font-size: 24px; font-weight: bold; color: #00FF00; margin-left: 30px;")
        
        panel_layout.addWidget(self.status_label)
        panel_layout.addWidget(self.prominence_label)
        panel_layout.addStretch()
        panel_layout.addWidget(self.bpm_label)
        main_layout.addLayout(panel_layout)

        # --- Графики ---
        pg.setConfigOptions(antialias=True)
        self.graph_view = pg.GraphicsLayoutWidget()
        main_layout.addWidget(self.graph_view)

        self.p1 = self.graph_view.addPlot(title="Сырой сигнал Y (мм)")
        self.p1.showGrid(x=True, y=True)
        self.raw_curve = self.p1.plot(pen=pg.mkPen(color='d', width=1.5))
        
        self.graph_view.nextRow()

        self.p2 = self.graph_view.addPlot(title="DSP Анализ (Вдохи/Выдохи)")
        self.p2.showGrid(x=True, y=True)
        self.filtered_curve = self.p2.plot(pen=pg.mkPen(color='b', width=2))
        
        self.peaks_scatter = pg.ScatterPlotItem(size=12, pen=pg.mkPen('r'), brush=pg.mkBrush('r'), symbol='t1')
        self.troughs_scatter = pg.ScatterPlotItem(size=12, pen=pg.mkPen('g'), brush=pg.mkBrush('g'), symbol='t')
        self.p2.addItem(self.peaks_scatter)
        self.p2.addItem(self.troughs_scatter)
        
        self.update_ui_state()

    # --- Управление состояниями (FSM) ---
    def set_idle(self):
        self.state = AppState.IDLE
        self.status_label.setText("Статус: ОЖИДАНИЕ. Требуется калибровка.")
        self.bpm_label.setText("BPM: --")
        self.prominence_label.setText("Порог: --")
        self.calib_prominence = 0
        self.update_ui_state()

    def start_calibration(self):
        self.raw_y_buffer.clear()
        self.time_buffer.clear()
        self.last_tracked_y = None
        self.state = AppState.CALIBRATING
        self.status_label.setText("Статус: ИДЕТ КАЛИБРОВКА. Сделайте 3 глубоких вдоха и выдоха!")
        self.update_ui_state()

    def review_calibration(self):
        if len(self.raw_y_buffer) < 100:
            QMessageBox.warning(self, "Ошибка", "Слишком мало данных. Подождите хотя бы 10 секунд.")
            return
        
        # Переключаем состояние (таймер перестанет обновлять график, график "заморозится")
        self.state = AppState.REVIEWING
        
        # Делаем разовый финальный расчет для текущего замороженного окна
        raw_y = np.array(self.raw_y_buffer)
        y_filt = filtfilt(self.b, self.a, raw_y)
        min_dist_samples = int(1.5 * self.fs)
        
        # Во время ревью ищем ВСЕ пики (без порога), чтобы пользователь оценил качество
        peaks, _ = find_peaks(y_filt, distance=min_dist_samples)
        troughs, _ = find_peaks(-y_filt, distance=min_dist_samples)
        
        num_breaths = len(peaks)
        self.status_label.setText(f"ОСМОТР: Найдено вдохов: {num_breaths}. График соответствует реальности?")
        
        # Сохраняем амплитуду для расчета порога, если пользователь нажмет "Подтверждаю"
        self._temp_calib_amp = np.max(y_filt) - np.min(y_filt) if len(y_filt) > 0 else 0
        
        self.update_ui_state()

    def approve_calibration(self):
        # Рассчитываем и фиксируем порог отсечки шума
        self.calib_prominence = self._temp_calib_amp * 0.35  # 35% от максимального вдоха
        self.prominence_label.setText(f"Порог: {self.calib_prominence:.1f} мм")
        
        self.state = AppState.READY
        self.status_label.setText("Статус: СИСТЕМА ОТКАЛИБРОВАНА И ГОТОВА.")
        self.update_ui_state()

    def start_experiment(self):
        # Очищаем буферы от калибровочных данных для чистого эксперимента
        self.raw_y_buffer.clear()
        self.time_buffer.clear()
        self.state = AppState.RUNNING
        self.status_label.setText("Статус: ЗАПИСЬ ЭКСПЕРИМЕНТА ИДЕТ...")
        self.update_ui_state()

    def update_ui_state(self):
        """Скрывает и показывает кнопки в зависимости от логики"""
        self.btn_reset.setVisible(True)
        self.btn_start_calib.setVisible(self.state in [AppState.IDLE, AppState.READY])
        self.btn_stop_calib.setVisible(self.state == AppState.CALIBRATING)
        self.btn_approve.setVisible(self.state == AppState.REVIEWING)
        self.btn_reject.setVisible(self.state == AppState.REVIEWING)
        self.btn_start_exp.setVisible(self.state == AppState.READY)

    @Slot(list, int)
    def process_new_frame(self, targets, ts_ms):
        # Не пишем новые данные в буфер, если мы заморозили график для проверки
        if self.state in [AppState.IDLE, AppState.REVIEWING, AppState.READY]:
            return 
            
        best_target = None
        min_delta_y = float('inf')

        for idx, t in enumerate(targets):
            if t['res'] > 0:
                if self.last_tracked_y is None:
                    best_target = t
                    self.current_slot = idx
                    break
                else:
                    delta_y = abs(t['y'] - self.last_tracked_y)
                    if delta_y < min_delta_y and delta_y < self.max_tracking_jump:
                        min_delta_y = delta_y
                        best_target = t
                        self.current_slot = idx

        if best_target is not None:
            self.last_tracked_y = best_target['y']
            self.raw_y_buffer.append(best_target['y'])
            self.time_buffer.append(ts_ms / 1000.0)
        else:
            if len(self.raw_y_buffer) > 0:
                self.raw_y_buffer.append(self.raw_y_buffer[-1])
                if len(self.time_buffer) > 1:
                    dt = self.time_buffer[-1] - self.time_buffer[-2]
                    self.time_buffer.append(self.time_buffer[-1] + dt)
                else:
                    self.time_buffer.append(time.time())

    # def update_dsp_and_plots(self):
    #     # Обновляем графики только в активных режимах сбора
    #     if self.state not in [AppState.CALIBRATING, AppState.RUNNING] or len(self.raw_y_buffer) < 50:
    #         return

    #     raw_y = np.array(self.raw_y_buffer)
    #     times = np.array(self.time_buffer)
    #     times_rel = times - times[0]

    #     dt_mean = np.mean(np.diff(times_rel))
    #     if dt_mean > 0:
    #         fs_actual = 1.0 / dt_mean
    #         if abs(fs_actual - self.fs) > 0.5:
    #             self.setup_filter(fs_actual)

    #     self.raw_curve.setData(times_rel, raw_y)

    #     try:
    #         y_filtered = filtfilt(self.b, self.a, raw_y)
    #         self.filtered_curve.setData(times_rel, y_filtered)

    #         min_dist_samples = int(1.5 * self.fs)
            
    #         # Если это рабочий эксперимент, применяем строгий порог из калибровки
    #         if self.state == AppState.RUNNING:
    #             peaks, _ = find_peaks(y_filtered, distance=min_dist_samples, prominence=self.calib_prominence)
    #             troughs, _ = find_peaks(-y_filtered, distance=min_dist_samples, prominence=self.calib_prominence)
    #         else:
    #             # В режиме калибровки просто ищем все физические пики
    #             peaks, _ = find_peaks(y_filtered, distance=min_dist_samples)
    #             troughs, _ = find_peaks(-y_filtered, distance=min_dist_samples)

    #         if len(peaks) > 0: self.peaks_scatter.setData(x=times_rel[peaks], y=y_filtered[peaks])
    #         else: self.peaks_scatter.clear()
                
    #         if len(troughs) > 0: self.troughs_scatter.setData(x=times_rel[troughs], y=y_filtered[troughs])
    #         else: self.troughs_scatter.clear()

    #         if len(peaks) > 1 and self.state == AppState.RUNNING:
    #             avg_period = np.median(np.diff(times_rel[peaks]))
    #             if avg_period > 0:
    #                 self.bpm_label.setText(f"BPM: {60.0 / avg_period:.1f}")
    #     except Exception:
    #         pass

    def update_dsp_and_plots(self):
        if len(self.raw_y_buffer) < 50:
            return

        # 1. Снимаем тренд (удаляем DC-смещение)
        raw_y = detrend(np.array(self.raw_y_buffer))
        times = np.array(self.time_buffer)
        times_rel = times - times[0]

        # 2. Фильтрация
        y_filtered = filtfilt(self.b, self.a, raw_y)

        # 3. АНАЛИЗ ГИЛЬБЕРТА (Магия точности)
        analytic_signal = hilbert(y_filtered)
        instantaneous_phase = np.unwrap(np.angle(analytic_signal))
        
        # Мгновенная частота (в Гц)
        # Вычисляем производную фазы
        inst_freq = (np.diff(instantaneous_phase) / (2.0 * np.pi)) / np.mean(np.diff(times_rel))
        
        # 4. Расчет BPM (среднее за последние 5 секунд)
        # Вместо поиска пиков считаем среднюю частоту в фазовом пространстве
        current_bpm = np.mean(inst_freq[-50:]) * 60.0

        # Отрисовка
        self.filtered_curve.setData(times_rel, y_filtered)
        self.bpm_label.setText(f"BPM: {current_bpm:.2f}")

    def closeEvent(self, event):
        self.serial_thread.stop()
        event.accept()

if __name__ == "__main__":
    app = QApplication(sys.argv)
    app.setStyle('Fusion')
    monitor = RespirationMonitor()
    monitor.show()
    sys.exit(app.exec())