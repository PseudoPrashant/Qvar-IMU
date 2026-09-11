#!/usr/bin/env python3
"""
Standalone Raw Q1 Electrode Waveform Plotter
============================================
Lightweight visualizer dedicated purely to plotting raw Q1 electrode telemetry
streaming over COM7 (or viewing recorded CSV waveforms).

Features:
  - Live Serial Mode (default): Streams and plots raw Q1 samples from COM7 in real time.
  - CSV File Mode: Plots recorded sessions from CSV files (e.g. data/Glasses.csv).
  - Monotonic auto-expansion (no bouncing/shrinking).
  - Premium dark-mode oscilloscope theme with live HUD (LSB, mV, Min, Max, Peak-to-Peak).

Usage:
  python plot_raw_q1.py                    # Live streaming from COM7
  python plot_raw_q1.py --port COM7        # Specify serial port
  python plot_raw_q1.py data/Glasses.csv   # View recorded CSV waveform
  python plot_raw_q1.py --window 500       # Adjust rolling window size (samples)
  python plot_raw_q1.py --ylim -35000 5000 # Lock Y-axis limits
"""

import argparse
import collections
import csv
import os
import re
import sys
import threading
import time

try:
    import matplotlib
    import matplotlib.animation as animation
    import matplotlib.pyplot as plt
except ImportError:
    print("[!] Error: 'matplotlib' is required. Run: pip install matplotlib")
    sys.exit(1)

# Regex to extract Q1 raw sample from ESP32 serial stream
# Matches: [IMU QVAR RAW] Q1=1240 Q2=... or Q1=-52
RE_RAW_Q1 = re.compile(r"\[IMU QVAR RAW\]\s+Q1=(?:NA:)?(-?\d+)")


class RawQ1SerialReader:
    """Reads serial line stream in a background thread and extracts Q1 raw samples."""

    def __init__(self, port, baud, max_samples=2000):
        self.port = port
        self.baud = baud
        self.max_samples = max_samples
        self.lock = threading.Lock()
        self.running = True
        self.connected = False
        self.status = "Connecting..."
        self.sample_idx = 0

        # Buffers
        self.indices = collections.deque(maxlen=max_samples)
        self.values = collections.deque(maxlen=max_samples)

        self.thread = threading.Thread(target=self._worker, daemon=True)
        self.thread.start()

    def _worker(self):
        try:
            import serial
        except ImportError:
            self.status = "pyserial not installed"
            return

        try:
            ser = serial.Serial(self.port, self.baud, timeout=0.1)
            self.connected = True
            self.status = f"Connected ({self.port} @ {self.baud})"
        except Exception as e:
            self.status = f"Port error: {e}"
            return

        while self.running:
            try:
                line_bytes = ser.readline()
                if not line_bytes:
                    continue
                line = line_bytes.decode("utf-8", errors="replace").strip()
                if not line:
                    continue

                m = RE_RAW_Q1.search(line)
                if m:
                    val = int(m.group(1))
                    with self.lock:
                        self.sample_idx += 1
                        self.indices.append(self.sample_idx)
                        self.values.append(val)
            except Exception as e:
                self.status = f"Read error: {e}"
                time.sleep(0.05)

        ser.close()
        self.connected = False
        self.status = "Disconnected"

    def get_data(self):
        with self.lock:
            return list(self.indices), list(self.values), self.status, self.connected

    def stop(self):
        self.running = False


def plot_live(port, baud, window_size, ylim=None):
    """Run real-time dark-theme rolling waveform plot of raw Q1 data."""
    reader = RawQ1SerialReader(port, baud, max_samples=max(2000, window_size * 2))

    # Styling
    plt.style.use("dark_background")
    fig, ax = plt.subplots(figsize=(11, 6))
    fig.canvas.manager.set_window_title(f"QVAR Raw Q1 Stream - {port}")
    fig.patch.set_facecolor("#0b0f19")
    ax.set_facecolor("#111827")

    # Trace line: vibrant cyan with subtle glow
    (line,) = ax.plot([], [], color="#00e5ff", linewidth=2.0, label="Q1 Raw (LSB)")

    # Zero baseline reference line
    ax.axhline(0, color="#64748b", linestyle="-", alpha=0.4, linewidth=0.8, label="Zero Baseline")

    # Grid and styling
    ax.grid(True, linestyle="--", alpha=0.25, color="#475569")
    ax.set_xlabel("Sample Index (100 Hz = 10 ms / sample)", color="#94a3b8", fontsize=11, labelpad=8)
    ax.set_ylabel("Raw Amplitude (LSB)", color="#94a3b8", fontsize=11, labelpad=8)
    ax.tick_params(colors="#64748b", labelsize=10)
    for spine in ax.spines.values():
        spine.set_color("#334155")
    ax.legend(loc="upper right", facecolor="#1e293b", edgecolor="#475569", fontsize=9)

    # HUD Texts
    title_text = ax.text(
        0.02, 0.94, f"Q1 RAW SENSOR STREAM ({port})",
        transform=ax.transAxes, color="#f8fafc",
        fontsize=13, weight="bold"
    )
    hud_text = ax.text(
        0.02, 0.86, "Waiting for telemetry...",
        transform=ax.transAxes, color="#38bdf8",
        fontsize=10, family="monospace"
    )
    status_text = ax.text(
        0.98, 0.94, reader.status,
        transform=ax.transAxes, color="#81c784",
        fontsize=10, horizontalalignment="right", family="monospace"
    )

    y_min_hist = -500.0
    y_max_hist = 500.0

    def update(frame):
        nonlocal y_min_hist, y_max_hist
        xs, ys, status, connected = reader.get_data()

        status_text.set_text(status)
        status_text.set_color("#4ade80" if connected else "#f87171")

        if not xs:
            return line, hud_text, status_text

        # Slice rolling window
        disp_xs = xs[-window_size:]
        disp_ys = ys[-window_size:]

        line.set_data(disp_xs, disp_ys)

        # X-axis limits
        min_x = disp_xs[0]
        max_x = disp_xs[-1]
        ax.set_xlim(min_x, max(max_x, min_x + 10))

        # Y-axis auto-scaling: monotonic expansion (only zoom out, never zoom in)
        curr_min = min(disp_ys)
        curr_max = max(disp_ys)
        y_min_hist = min(y_min_hist, curr_min)
        y_max_hist = max(y_max_hist, curr_max)

        if ylim is not None:
            ax.set_ylim(ylim[0], ylim[1])
        else:
            span = y_max_hist - y_min_hist
            pad = max(150.0, span * 0.15)
            ax.set_ylim(y_min_hist - pad, y_max_hist + pad)

        # HUD Calculation (ST AN5755 gain: 78 LSB / mV)
        latest_val = disp_ys[-1]
        latest_mv = latest_val / 78.0
        p2p = curr_max - curr_min
        p2p_mv = p2p / 78.0

        hud_text.set_text(
            f"Current  : {latest_val:+6d} LSB ({latest_mv:+6.1f} mV)\n"
            f"Min / Max: [{curr_min:+6d}, {curr_max:+6d}] LSB\n"
            f"Peak-Peak: {p2p:6d} LSB ({p2p_mv:5.1f} mV)"
        )

        return line, hud_text, status_text, title_text

    ani = animation.FuncAnimation(fig, update, interval=25, blit=False, cache_frame_data=False)

    try:
        plt.tight_layout()
        plt.show()
    finally:
        reader.stop()


def plot_csv_file(csv_path):
    """Plot an entire recorded CSV session with pan & zoom tools."""
    if not os.path.exists(csv_path):
        print(f"[!] Error: File '{csv_path}' does not exist.")
        sys.exit(1)

    print(f"[*] Reading telemetry from: {csv_path}...")
    xs = []
    ys = []

    with open(csv_path, "r", encoding="utf-8") as f:
        reader = csv.DictReader(f)
        for row in reader:
            try:
                idx = int(row.get("sample_index", len(xs) + 1))
                q1 = int(row["q1_raw"])
                xs.append(idx)
                ys.append(q1)
            except (ValueError, KeyError):
                continue

    if not xs:
        print("[!] Error: No valid Q1 samples found in file.")
        sys.exit(1)

    print(f"[+] Loaded {len(xs):,} samples.")

    plt.style.use("dark_background")
    fig, ax = plt.subplots(figsize=(12, 6))
    fig.canvas.manager.set_window_title(f"QVAR CSV Viewer - {os.path.basename(csv_path)}")
    fig.patch.set_facecolor("#0b0f19")
    ax.set_facecolor("#111827")

    # Plot raw Q1 waveform
    ax.plot(xs, ys, color="#00e5ff", linewidth=1.5, label="Q1 Raw (LSB)")
    ax.axhline(0, color="#64748b", linestyle="-", alpha=0.4, linewidth=0.8, label="Zero Baseline")

    # Grid & axes
    ax.grid(True, linestyle="--", alpha=0.25, color="#475569")
    ax.set_title(f"QVAR Q1 Raw Waveform: {os.path.basename(csv_path)} ({len(xs):,} samples)", color="#f8fafc", fontsize=13, weight="bold", pad=12)
    ax.set_xlabel("Sample Index (100 Hz)", color="#94a3b8", fontsize=11, labelpad=8)
    ax.set_ylabel("Raw Amplitude (LSB)", color="#94a3b8", fontsize=11, labelpad=8)
    ax.tick_params(colors="#64748b", labelsize=10)
    for spine in ax.spines.values():
        spine.set_color("#334155")
    ax.legend(loc="upper right", facecolor="#1e293b", edgecolor="#475569")

    # Summary box
    q1_min = min(ys)
    q1_max = max(ys)
    span = q1_max - q1_min
    summary_text = (
        f"Samples   : {len(xs):,}\n"
        f"Min LSB   : {q1_min:+d} ({q1_min/78.0:+.1f} mV)\n"
        f"Max LSB   : {q1_max:+d} ({q1_max/78.0:+.1f} mV)\n"
        f"Peak-Peak : {span:d} LSB ({span/78.0:.1f} mV)"
    )
    ax.text(
        0.02, 0.95, summary_text,
        transform=ax.transAxes, color="#e2e8f0",
        fontsize=10, family="monospace",
        verticalalignment="top",
        bbox=dict(boxstyle="round,pad=0.5", facecolor="#1e293b", alpha=0.8, edgecolor="#475569")
    )

    plt.tight_layout()
    print("[*] Displaying interactive plot (use toolbar to zoom and pan)...")
    plt.show()


def main():
    parser = argparse.ArgumentParser(
        description="Standalone Raw Q1 Electrode Waveform Plotter (Live COM streaming or CSV viewer)"
    )
    parser.add_argument(
        "file",
        nargs="?",
        default=None,
        help="Optional path to a CSV telemetry file (e.g. data/Glasses.csv) to view static recording.",
    )
    parser.add_argument(
        "--port",
        default="COM7",
        help="Serial port for live streaming (default: COM7)",
    )
    parser.add_argument(
        "--baud",
        type=int,
        default=115200,
        help="Baud rate (default: 115200)",
    )
    parser.add_argument(
        "--window",
        type=int,
        default=300,
        help="Rolling window size in samples for live plot (default: 300 samples = 3 sec @ 100 Hz)",
    )
    parser.add_argument(
        "--ylim",
        nargs=2,
        type=float,
        default=None,
        metavar=("MIN", "MAX"),
        help="Lock Y-axis limits to fixed range (e.g. --ylim -35000 5000)",
    )
    parser.add_argument(
        "--test",
        action="store_true",
        help="Quick validation mode (does not block on GUI loop)",
    )

    args = parser.parse_args()

    # If test mode on CSV:
    if args.test and args.file:
        print(f"[*] Validating CSV load on '{args.file}'...")
        with open(args.file, "r") as f:
            reader = csv.DictReader(f)
            first_row = next(reader, None)
            assert first_row and "q1_raw" in first_row, "Missing q1_raw column"
        print("[+] Test passed successfully.")
        return

    # Decide Mode
    if args.file:
        plot_csv_file(args.file)
    else:
        plot_live(args.port, args.baud, args.window, args.ylim)


if __name__ == "__main__":
    main()
