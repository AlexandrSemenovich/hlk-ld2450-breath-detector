# HLK-LD2450 Detector

A contactless human **breath-rate monitor** built on the 24 GHz mmWave radar
module **HLK-LD2450** and an **ESP32**.

The ESP32 runs a tiny *transparent forwarder* firmware: it parses the raw radar
frames over UART and relays the up-to-three tracked targets to a PC as a compact
text stream. All breathing analysis (target selection, detrending,
band-pass filtering, FFT, BPM / SNR estimation, apnea detection) happens on the
PC in Python, where it is easy to visualise, log, and tune.

```
HLK-LD2450 ──UART(256000)──> ESP32 ──USB CDC(921600)──> python/breath_monitor.py
   (radar frames)   parse + relay      "R" lines          (analysis + live plot)
```

## Features

- Minimal, dependency-free ESP32 firmware (Arduino / PlatformIO).
- Accurate LD2450 wire-frame parser (header `AA FF 03 00` … tail `55 CC`),
  signed-magnitude decoding per the HLK datasheet.
- Robust text protocol with a frame id so the PC can detect dropped frames.
- Reference PC tool with live matplotlib visualisation, CSV logging, and a
  breathing-rate / SNR / apnea estimator.
- Pure, unit-tested analysis functions (`pick_target`, `detect_breath`).

> **Disclaimer:** this is an experimental hobby project, **not a medical
> device**. Do not use it for health, safety, or clinical decisions.

## Hardware

| Part      | Notes                                                        |
|-----------|-------------------------------------------------------------|
| HLK-LD2450 | 24 GHz mmWave radar, 5 V supply, 3.3 V UART IO, 256000 8N1 |
| ESP32      | Any ESP32 dev board (tested on a generic `esp32dev`)        |

Wire the radar to the ESP32 (crossed TX/RX):

| LD2450 | ESP32        | Signal            |
|--------|--------------|-------------------|
| 5V     | 5V           | power             |
| GND    | GND          | ground            |
| TX     | GPIO32 (RX)  | radar → ESP32     |
| RX     | GPIO33 (TX)  | ESP32 → radar     |

Mount the radar 1.5–2 m above the subject, antenna pointing at the monitored
area. The default zone (configurable in `python/breath_monitor.py`) is a ring
`600 mm ≤ radial ≤ 2500 mm` with `|x| ≤ 500 mm`.

## Firmware build & flash

Requires [PlatformIO](https://platformio.org/).

```bash
pio run              # build
pio run -t upload    # flash (set upload port in platformio.ini or via --upload-port)
pio device monitor -b 921600
```

On boot the firmware prints a one-line protocol banner, then streams one `R`
line per radar frame (~10 Hz).

## Serial protocol

One ASCII line per radar frame, newline terminated:

```
R<x0>,<y0>,<spd0>,<res0>,<x1>,<y1>,<spd1>,<res1>,<x2>,<y2>,<spd2>,<res2>,<ts_ms>,<frame_id>
```

| Field     | Type        | Unit  | Meaning                                            |
|-----------|-------------|-------|----------------------------------------------------|
| `x`, `y`  | int16 (s.m.)| mm    | lateral / longitudinal target coordinate          |
| `spd`     | int16 (s.m.)| cm/s  | radial speed (sign = direction)                    |
| `res`     | uint16      | mm    | distance-gate size; **`0` ⇒ slot empty**           |
| `ts_ms`   | uint32      | ms    | `millis()` timestamp on the ESP32                  |
| `frame_id`| uint32      | —     | monotonic counter (detects dropped frames)         |

`s.m.` = signed-magnitude, sign bit = bit15 (1 → positive). Three target slots
are always emitted; an empty slot has `res == 0` (and `x = y = spd = 0`).

Coordinate convention: `x` is lateral, `y` is the forward (longitudinal) axis.
The breath axis is `radial = sqrt(x² + y²)`.

## PC tool

All PC code lives in [`python/`](python/).

```bash
cd python
python -m venv .venv && source .venv/bin/activate   # optional
pip install -r requirements.txt

# auto-detect the serial port, or pass it explicitly:
python breath_monitor.py            # auto-detect
python breath_monitor.py COM3 921600
python breath_monitor.py /dev/ttyUSB0
```

What it does:

1. Reads the serial port in a background thread into a thread-safe buffer.
2. `pick_target` selects the most stationary in-zone target each frame.
3. Detrends the radial distance (slow EMA high-pass) → AC breathing signal.
4. Resamples to a uniform grid using `ts_ms`, then band-pass filters
   (Butterworth, 0.12–0.5 Hz ≈ 7–30 breaths/min).
5. FFT → dominant peak in band → `BPM = f · 60`; SNR vs. the rest of the band.
6. Flags apnea when no confident peak is found for > 15 s.
7. Live plot + CSV logging (`--out session.csv`) for offline analysis.

Override the matplotlib backend if needed:

```bash
MPLBACKEND=QtAgg python breath_monitor.py /dev/ttyUSB0
```

## Project layout

```
src/
  main.cpp          # ESP32 firmware: read radar, relay targets over USB CDC
  ld2450_parser.h/.cpp  # LD2450 wire-frame parser
  radar_bridge.h    # serial relay ("R" protocol) + banner
python/
  breath_monitor.py # PC tool: decode, analyse, plot, log
  requirements.txt
  tests/            # unit tests for protocol + analysis
doc/
  006000_HLK-LD2450-Instruction-Manual.pdf
```

## Limitations

The LD2450 is a *moving-target* tracker; its distance resolution is coarse
(`distance_res` ≈ 320 mm per the datasheet), and breath amplitude (~5–15 mm)
is comparable to tracker noise, so expect a low SNR. For stable results: track
only a stationary in-zone target, use an aggressive band-pass, and gate results
on the SNR / quality estimate before trusting the BPM.

## License

[MIT](LICENSE) — see `LICENSE`.
