"""Pure analysis functions for the breath monitor (no I/O, no GUI).

Everything here is side-effect free except `Detrender` (stateful signal
detrender). Keep this module free of matplotlib / serial imports so it stays
fast, testable and reusable from the PC-side pipeline.
"""

import math

import numpy as np
from scipy.signal import butter, filtfilt

# ==================== НАСТРОЙКИ КАРТЫ ====================
LATERAL_MIN = -600       # Лимит влево (мм)
LATERAL_MAX = 600        # Лимит вправо (мм)
CENTER_LAT = 0
DEPTH_MIN = 0
DEPTH_MAX = 2600         # Глубина обнаружения (до ~2.5 м)
# =======================================================

# Параметры выбора цели (зеркалят бывший ESP32 firmware).
VALID_R_MIN = 100.0
VALID_R_MAX = 6000.0
ZONE_R_MIN = 600.0
ZONE_R_MAX = 2500.0
ZONE_SIDE_MAX = 500.0
STATIONARY_SPEED_THRESHOLD = 80.0   # cm/s

ZONE_EDGE_POINTS = 200
TREND_TAIL = 0.25

# Цвета для отрисовки каждой из 3 целей (t0, t1, t2) и выделенной цели.
TARGET_COLORS = ("#2c7fb8", "#e6550d", "#756bb1")
SELECTED_COLOR = "#31a354"

# Breath band (Hz) -> 7.2 .. 30 breaths/min, with margin.
BAND_LO_HZ = 0.12
BAND_HI_HZ = 0.5
MIN_BPM = 5.0
MAX_BPM = 40.0
APNEA_S = 15.0
SNR_THRESHOLD = 3.0


def radial_of(x, y):
    return math.hypot(x, y)


def pick_target(targets, locked_index):
    """Выбор цели целиком на ПК.

    Приоритет:
      1) удерживать ранее выбранную цель, если она ещё в зоне и стационарна
      2) ближайшая стационарная цель в зоне
      3) fallback: ближайшая присутствующая цель (для прицеливания)
    `targets` — список (x, y, speed, res); res == 0 => слот пустой.
    Возвращает индекс цели или None.
    """
    present = [(i, radial_of(t[0], t[1])) for i, t in enumerate(targets) if t[3] != 0]
    if not present:
        return None

    if locked_index is not None:
        for i, r in present:
            if i != locked_index:
                continue
            t = targets[i]
            if (ZONE_R_MIN <= r <= ZONE_R_MAX and abs(t[0]) <= ZONE_SIDE_MAX
                    and abs(t[2]) < STATIONARY_SPEED_THRESHOLD):
                return i

    best = None
    best_r = 1e12
    for i, r in present:
        t = targets[i]
        if not (ZONE_R_MIN <= r <= ZONE_R_MAX and abs(t[0]) <= ZONE_SIDE_MAX):
            continue
        if abs(t[2]) >= STATIONARY_SPEED_THRESHOLD:
            continue
        if r < best_r:
            best_r = r
            best = i
    if best is not None:
        return best

    # Fallback: ближайшая присутствующая цель (прицеливание/отладка).
    best = None
    best_r = 1e12
    for i, r in present:
        if r < best_r:
            best_r = r
            best = i
    return best


class Detrender:
    """Детренд сигнала дистанции (зеркалит бывший firmware breath_detector).

    lp = EMA низких частот; ac = lp - trend, trend = медленный high-pass.
    """

    def __init__(self, lp_alpha=0.15, trend_tau_ms=6000.0):
        self.lp_alpha = lp_alpha
        self.trend_tau = trend_tau_ms
        self.lp = None
        self.trend = None
        self.last_ms = None

    def push(self, dist_mm, now_ms):
        if self.lp is None:
            self.lp = self.trend = dist_mm
            self.last_ms = now_ms
            return 0.0
        dms = now_ms - self.last_ms
        if dms > 500:
            dms = 500
        self.last_ms = now_ms
        dt = dms
        self.lp = self.lp_alpha * dist_mm + (1.0 - self.lp_alpha) * self.lp
        a = 1.0 - math.exp(-dt / self.trend_tau)
        self.trend += a * (self.lp - self.trend)
        return self.lp - self.trend

    def reset(self):
        self.lp = self.trend = None
        self.last_ms = None


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
