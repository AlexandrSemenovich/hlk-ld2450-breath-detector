#!/usr/bin/env python3
"""Entry point for the HLK-LD2450 breath monitor.

All logic lives in the sibling modules (protocol / analysis / state / plots /
app). Run with:  python monitor.py [PORT] [BAUD]
"""

import sys

import matplotlib.pyplot as plt

from app import MonitorApp, find_port


def main():
    port = sys.argv[1] if len(sys.argv) > 1 else find_port()
    baud = int(sys.argv[2]) if len(sys.argv) > 2 else 921600
    if not port:
        print("No serial port found.")
        return

    print(f"Connecting to {port} @ {baud} ...")
    app = MonitorApp(port, baud)
    print("Connected.")
    app.run()


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        pass
    finally:
        plt.close("all")
