"""Thread-safe application state for the breath monitor.

Holds rolling history for ALL radar targets (3 slots), the current target
selection, the latest detection result, and a ring of human-readable parse
log lines (shown in the log panel). The serial reader thread calls `ingest`;
the animation calls `analyze` and reads the buffers under the same lock.

History is stored per target slot so every chart can render each target
individually; the `selected` slot is the one used for breath detection.
"""

import time
import threading
import collections

import analysis

WINDOW_S = 30          # seconds of history kept for plotting/analysis
MAXLEN = 2000          # hard cap on buffer length
LOG_MAX = 400          # ring size for the parse log
NSLOTS = 3


class MonitorState:
    def __init__(self):
        self.lock = threading.RLock()
        self.times = collections.deque(maxlen=MAXLEN)   # PC clock, shared
        self.tsms = collections.deque(maxlen=MAXLEN)    # radar ts_ms, shared

        # Per-target-slot rolling history.
        self.depth = [collections.deque(maxlen=MAXLEN) for _ in range(NSLOTS)]
        self.lateral = [collections.deque(maxlen=MAXLEN) for _ in range(NSLOTS)]
        self.dist = [collections.deque(maxlen=MAXLEN) for _ in range(NSLOTS)]
        self.ac = [collections.deque(maxlen=MAXLEN) for _ in range(NSLOTS)]
        self.present = [collections.deque(maxlen=MAXLEN) for _ in range(NSLOTS)]
        self.detrenders = [analysis.Detrender() for _ in range(NSLOTS)]

        self.selected = None
        self.latest = [(0, 0, 0, 0, False, 0.0) for _ in range(NSLOTS)]

        # Per-target draw visibility, toggled from the GUI checkboxes.
        self.show_target = [True, True, True]

        self.visible = False
        self.in_zone = False
        self.stationary = False
        self.bpm = 0.0
        self.quality = 0.0
        self.detected = False
        self.apnea = False
        self.dropped = 0

        self.last_frame_id = 0
        self.last_valid_breath_time = None

        self.log_lines = collections.deque(maxlen=LOG_MAX)
        self.start = time.time()

    # ---- parse log -------------------------------------------------------
    def append_log(self, msg):
        with self.lock:
            self.log_lines.append(msg)

    def recent_log(self, n=12):
        with self.lock:
            return "\n".join(list(self.log_lines)[-n:])

    # ---- ingest one raw frame -------------------------------------------
    def ingest(self, raw):
        with self.lock:
            sel = analysis.pick_target(
                [(t.x, t.y, t.speed, t.res) for t in raw.targets], self.selected
            )
            self.selected = sel

            if self.last_frame_id and raw.frame_id != self.last_frame_id + 1:
                self.dropped += (raw.frame_id - self.last_frame_id - 1)
            self.last_frame_id = raw.frame_id

            now = time.time() - self.start
            self.times.append(now)
            self.tsms.append(raw.ts_ms)

            for i, t in enumerate(raw.targets):
                if t.res != 0:
                    depth = analysis.radial_of(t.x, t.y)
                    lateral = float(t.x)
                    stationary = abs(t.speed) < analysis.STATIONARY_SPEED_THRESHOLD
                    in_zone = (analysis.ZONE_R_MIN <= depth <= analysis.ZONE_R_MAX
                               and abs(t.x) <= analysis.ZONE_SIDE_MAX)

                    # Zero-Order-Hold защита от выбросов координат.
                    if depth < analysis.DEPTH_MIN or lateral < analysis.LATERAL_MIN or lateral > analysis.LATERAL_MAX:
                        depth = self.depth[i][-1] if self.depth[i] else 0.0
                        lateral = self.lateral[i][-1] if self.lateral[i] else 0.0

                    if in_zone and stationary:
                        ac = self.detrenders[i].push(depth, raw.ts_ms)
                    else:
                        self.detrenders[i].reset()
                        ac = 0.0

                    self.depth[i].append(depth)
                    self.lateral[i].append(lateral)
                    self.dist[i].append(depth)
                    self.ac[i].append(ac)
                    self.present[i].append(True)
                    self.latest[i] = (t.x, t.y, t.speed, t.res, True, depth)
                else:
                    self.detrenders[i].reset()
                    self.depth[i].append(0.0)
                    self.lateral[i].append(self.lateral[i][-1] if self.lateral[i] else 0.0)
                    self.dist[i].append(0.0)
                    self.ac[i].append(0.0)
                    self.present[i].append(False)
                    self.latest[i] = (0, 0, 0, 0, False, 0.0)

            if sel is None:
                self.visible = False
                self.in_zone = False
                self.stationary = False
            else:
                t = raw.targets[sel]
                self.visible = True
                self.in_zone = (analysis.ZONE_R_MIN <= analysis.radial_of(t.x, t.y) <= analysis.ZONE_R_MAX
                                and abs(t.x) <= analysis.ZONE_SIDE_MAX)
                self.stationary = abs(t.speed) < analysis.STATIONARY_SPEED_THRESHOLD

            desc = " ".join(
                f"t{i}:(" + (f"{t.x},{t.y},spd{t.speed},res{t.res}" if t.res else "empty") + ")"
                for i, t in enumerate(raw.targets)
            )
            if sel is None:
                self.append_log(f"[#{raw.frame_id} ts={raw.ts_ms}] {desc} -> NO TARGET")
            else:
                self.append_log(f"[#{raw.frame_id} ts={raw.ts_ms}] {desc} -> sel={sel}")

            self._trim(now)

    def _trim(self, now):
        while self.times and self.times[0] < now - WINDOW_S:
            for dq in (self.times, self.tsms):
                dq.popleft()
            for i in range(NSLOTS):
                for dq in (self.depth[i], self.lateral[i], self.dist[i],
                           self.ac[i], self.present[i]):
                    dq.popleft()

    # ---- run FFT breath detection on the current window -----------------
    def analyze(self):
        with self.lock:
            sel = self.selected
            if sel is None or len(self.ac[sel]) < 2 or len(self.tsms) < 2:
                self.detected = False
                self.apnea = False
                self.quality = 0.0
                return 0.0, False, False, 0.0
            ts = list(self.tsms)
            ac = list(self.ac[sel])

        bpm, detected, quality, snr = analysis.detect_breath(ts, ac)

        current = time.time() - self.start
        with self.lock:
            if detected:
                self.last_valid_breath_time = current
                self.apnea = False
            elif self.last_valid_breath_time is None:
                self.apnea = current > analysis.APNEA_S
            else:
                self.apnea = (current - self.last_valid_breath_time) > analysis.APNEA_S

            self.bpm = bpm
            self.detected = detected
            self.quality = quality
            return bpm, detected, self.apnea, quality
