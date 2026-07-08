import numpy as np
import matplotlib.pyplot as plt
from scipy.signal import butter, filtfilt, find_peaks

# 1. Загрузка данных
filename = 'output_2026-07-08_12-36-59.txt'

times_ms = []
y_coords = []

with open(filename, 'r') as f:
    for line in f:
        line = line.strip()
        if line.startswith('R'):
            parts = line[1:].split(',')
            if len(parts) >= 14:
                # Извлекаем Y-координату цели 0 (индекс 1) и время (индекс 12)
                y_val = int(parts[1])
                ts_ms = int(parts[12])
                
                # Игнорируем нули, если радар терял цель на долю секунды
                if y_val != 0: 
                    y_coords.append(y_val)
                    times_ms.append(ts_ms)

# Переводим время в секунды от начала записи
times_sec = (np.array(times_ms) - times_ms[0]) / 1000.0
y_data = np.array(y_coords)

# 2. Вычисление частоты дискретизации (Fs)
dt = np.mean(np.diff(times_sec))
fs = 1.0 / dt
print(f"Средняя частота дискретизации: {fs:.2f} Гц")

# 3. Настройка полосового фильтра Баттерворта
# Нормальное дыхание: 12-20 вдохов в минуту -> 0.2 - 0.33 Гц
# Расширяем окно для надежности: от 0.1 Гц до 0.5 Гц
lowcut = 0.1
highcut = 0.5
nyquist = 0.5 * fs
low = lowcut / nyquist
high = highcut / nyquist

# Фильтр 2-го порядка
b, a = butter(2, [low, high], btype='band')

# Применяем фильтр с нулевым фазовым сдвигом (filtfilt)
# Это уберет постоянную составляющую (Detrend) и высокочастотный шум
y_filtered = filtfilt(b, a, y_data)

# 4. Поиск вдохов (пиков) и выдохов (впадин)
# distance: минимальное расстояние между вдохами в отсчетах (например, 1.5 секунды)
min_distance_samples = int(1.5 * fs) 
peaks, _ = find_peaks(y_filtered, distance=min_distance_samples)
troughs, _ = find_peaks(-y_filtered, distance=min_distance_samples)

# 5. Визуализация
plt.figure(figsize=(14, 7))

plt.subplot(2, 1, 1)
plt.plot(times_sec, y_data, label='Сырая Y-координата (мм)', color='lightgray')
plt.title('Сырые данные с LD2450')
plt.xlabel('Время (с)')
plt.ylabel('Дистанция (мм)')
plt.legend()
plt.grid(True)

plt.subplot(2, 1, 2)
plt.plot(times_sec, y_filtered, label='Отфильтрованный сигнал (0.1 - 0.5 Гц)', color='blue')
plt.plot(times_sec[peaks], y_filtered[peaks], "r^", markersize=8, label='Вдох')
plt.plot(times_sec[troughs], y_filtered[troughs], "gv", markersize=8, label='Выдох')

# Расчет мгновенной частоты дыхания по первым 5 пикам
if len(peaks) > 1:
    avg_breath_period = np.mean(np.diff(times_sec[peaks]))
    bpm = 60.0 / avg_breath_period
    plt.title(f'Выделенное дыхание (Средняя частота: {bpm:.1f} вдохов/мин)')
else:
    plt.title('Выделенное дыхание')

plt.xlabel('Время (с)')
plt.ylabel('Амплитуда (мм)')
plt.legend()
plt.grid(True)

plt.tight_layout()
plt.show()