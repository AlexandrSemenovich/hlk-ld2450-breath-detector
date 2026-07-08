"""Protocol decoder for the ESP32 -> PC stream.

This module is the ONLY place that knows the wire format. To support a new
firmware protocol, rewrite `decode_line` (or add a sibling function) — nothing
else in the app imports the regex or the field layout directly.

Wire format (one line per radar frame):
    R<x0>,<y0>,<spd0>,<res0>,<x1>,<y1>,<spd1>,<res1>,
      <x2>,<y2>,<spd2>,<res2>,<ts_ms>,<frame_id>
"""

import re
from dataclasses import dataclass

# 12 signed int fields (3 targets x (x, y, speed, distance_res)) + ts_ms + frame_id
_R_RE = re.compile(r"^R" + ",".join([r"([-0-9]+)"] * 12) + r",(\d+),(\d+)$")


@dataclass
class Target:
    x: int       # lateral, mm (signed)
    y: int       # longitudinal, mm (signed)
    speed: int   # radial speed, cm/s (signed)
    res: int     # distance gate resolution, mm (0 => slot empty)

    @property
    def present(self) -> bool:
        return self.res != 0


@dataclass
class RawFrame:
    targets: list          # 3 Target objects
    ts_ms: int             # millis() on the ESP
    frame_id: int          # monotonic frame counter


def decode_line(line: str):
    """Decode one serial line into a RawFrame, or return None if it is not a
    valid R-frame (control banners, noise, partial lines, ...)."""
    m = _R_RE.match(line)
    if not m:
        return None
    vals = [int(m.group(k)) for k in range(1, 13)]
    targets = [
        Target(vals[0], vals[1], vals[2], vals[3]),
        Target(vals[4], vals[5], vals[6], vals[7]),
        Target(vals[8], vals[9], vals[10], vals[11]),
    ]
    return RawFrame(targets, int(m.group(13)), int(m.group(14)))
