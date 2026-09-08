#!/usr/bin/env python3
"""
===============================================================================
ISM330BX QVAR Real-Time Serial Plotter
===============================================================================
Reads serial telemetry output from ESP32 / FreeRTOS ISM330BX firmware,
renders a real-time rolling waveform with threshold guides, baseline tracking,
and touch / wear event detection HUD, and automatically archives plotted data
to CSV files in the project's 'data' directory.

Usage:
  python plot_qvar.py                               # Auto-logs to project/data/
  python plot_qvar.py --port COM7 --window 250
  python plot_qvar.py --mock                        # Offline synthetic test mode
  python plot_qvar.py --record custom_output.csv    # Custom output file
  python plot_qvar.py --no-record                   # Disable CSV saving
===============================================================================
"""

import argparse
import collections
import csv
from datetime import datetime
import math
import os
import random
import re
import signal
import sys
import threading
import time

try:
    import serial
    import serial.tools.list_ports
except ImportError:
    serial = None

import numpy as np
import matplotlib
import matplotlib.animation as animation
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D

# Default storage directory
PROJECT_ROOT = os.path.dirname(os.path.abspath(__file__))
DEFAULT_DATA_DIR = os.path.join(PROJECT_ROOT, "data")

# -----------------------------------------------------------------------------
# Telemetry Regex Patterns
# -----------------------------------------------------------------------------
# Examples:
#   [IMU QVAR RAW] Q1=1240 Q2=NA:0
#   [IMU QVAR RAW] Q1=NA:0 Q2=840
#   [IMU QVAR RAW] Q1=-50 Q2=1120
RE_RAW_SAMPLE = re.compile(r"\[IMU QVAR RAW\]\s+Q1=(NA:)?(-?\d+)\s+Q2=(NA:)?(-?\d+)")

# Examples:
#   [IMU QVAR] Q1 button baseline=1200 press_th=900 release=350 peak=1300
RE_BUTTON_BASELINE = re.compile(
    r"\[IMU QVAR\]\s+Q1 button baseline=(-?\d+)\s+press_th=(-?\d+)\s+release=(-?\d+)(?:\s+peak=(-?\d+))?"
)

# Examples:
#   [IMU QVAR] Q2 wear baseline=800 on_th=700 off_th=250
RE_WEAR_BASELINE = re.compile(
    r"\[IMU QVAR\]\s+Q2 wear baseline=(-?\d+)\s+on_th=(-?\d+)\s+off_th=(-?\d+)"
)

# Events:
#   [IMU QVAR] Q1 BUTTON SINGLE
#   [IMU QVAR] Q1 BUTTON HOLD
#   [IMU QVAR] Q1 BUTTON HOLD RELEASE
#   [IMU QVAR] Q2 WEAR ON
#   [IMU QVAR] Q2 WEAR OFF
RE_EVENT = re.compile(
    r"\[IMU QVAR\]\s+(Q1 BUTTON SINGLE|Q1 BUTTON HOLD RELEASE|Q1 BUTTON HOLD|Q2 WEAR ON|Q2 WEAR OFF)"
)


class TelemetryReceiver:
    """Handles serial ingestion or synthetic mock generation in a background thread."""

    def __init__(self, port, baud, is_mock=False, csv_writer=None, csv_file=None, verbose=False):
        self.port = port
        self.baud = baud
        self.is_mock = is_mock
        self.csv_writer = csv_writer
        self.csv_file = csv_file
        self.verbose = verbose

        self.running = False
        self.thread = None
        self.lock = threading.Lock()

        # Data buffers: stores (timestamp, sample_idx, q1, q1_valid, q2, q2_valid)
        self.samples = collections.deque()
        self.events = collections.deque()  # (timestamp, sample_idx, event_name, value)
        self.pending_event = ""

        # Firmware states
        self.q1_baseline = None
        self.q1_press_th = None
        self.q1_rel_th = None
        self.q2_baseline = None
        self.q2_on_th = None
        self.q2_off_th = None

        self.sample_counter = 0
        self.status_message = "Initializing..."
        self.connected = False

    def start(self):
        self.running = True
        self.thread = threading.Thread(target=self._run, daemon=True)
        self.thread.start()

    def stop(self):
        self.running = False
        if self.thread and self.thread.is_alive():
            self.thread.join(timeout=1.0)

    def _run(self):
        if self.is_mock:
            self._run_mock()
        else:
            self._run_serial()

    def _run_serial(self):
        if serial is None:
            self.status_message = "Error: 'pyserial' is not installed."
            print(f"[ERROR] {self.status_message}")
            return

        print(f"[*] Opening serial port {self.port} at {self.baud} baud...")
        try:
            ser = serial.Serial(self.port, self.baud, timeout=0.5)
            self.connected = True
            self.status_message = f"Connected to {self.port} ({self.baud} baud)"
            print(f"[+] Successfully connected to {self.port}.")
        except serial.SerialException as exc:
            self.connected = False
            msg = str(exc)
            if "PermissionError" in msg or "Access is denied" in msg:
                self.status_message = (
                    f"Access denied to {self.port}. Is 'idf.py monitor' running? (Ctrl + ] to exit)"
                )
            else:
                self.status_message = f"Failed to open {self.port}: {exc}"
            print(f"\n[!] SERIAL PORT ERROR: {self.status_message}\n")
            return

        while self.running:
            try:
                line_bytes = ser.readline()
                if not line_bytes:
                    continue
                line = line_bytes.decode("utf-8", errors="replace").strip()
                if not line:
                    continue

                if self.verbose:
                    print(f"RAW: {line}")

                self._parse_line(line)

            except Exception as e:
                self.status_message = f"Serial read error: {e}"
                time.sleep(0.1)

        ser.close()
        self.connected = False
        self.status_message = "Disconnected."

    def _parse_line(self, line):
        now = time.time()

        # 1. Match Raw Sample
        m_raw = RE_RAW_SAMPLE.search(line)
        if m_raw:
            q1_na, q1_str, q2_na, q2_str = m_raw.groups()
            q1_val = int(q1_str)
            q2_val = int(q2_str)
            q1_valid = q1_na is None
            q2_valid = q2_na is None

            with self.lock:
                self.sample_counter += 1
                idx = self.sample_counter
                item = (now, idx, q1_val, q1_valid, q2_val, q2_valid)
                self.samples.append(item)

                if self.csv_writer:
                    iso_time = datetime.fromtimestamp(now).strftime("%Y-%m-%dT%H:%M:%S.%f")[:-3]
                    q1_base = self.q1_baseline if self.q1_baseline is not None else ""
                    q2_base = self.q2_baseline if self.q2_baseline is not None else ""
                    evt_str = self.pending_event
                    self.pending_event = ""  # Consume attached event

                    self.csv_writer.writerow([
                        iso_time,
                        f"{now:.4f}",
                        idx,
                        q1_val,
                        int(q1_valid),
                        q2_val,
                        int(q2_valid),
                        q1_base,
                        q2_base,
                        evt_str,
                    ])
                    if self.csv_file:
                        self.csv_file.flush()

            return

        # 2. Match Button Baseline & Thresholds
        m_btn = RE_BUTTON_BASELINE.search(line)
        if m_btn:
            base, press, rel, _ = m_btn.groups()
            with self.lock:
                self.q1_baseline = int(base)
                self.q1_press_th = int(press)
                self.q1_rel_th = int(rel)
            print(f"[Q1 CONFIG] Baseline={base}, PressTh={press}, RelTh={rel}")
            return

        # 3. Match Wear Baseline & Thresholds
        m_wear = RE_WEAR_BASELINE.search(line)
        if m_wear:
            base, on_th, off_th = m_wear.groups()
            with self.lock:
                self.q2_baseline = int(base)
                self.q2_on_th = int(on_th)
                self.q2_off_th = int(off_th)
            print(f"[Q2 CONFIG] Baseline={base}, OnTh={on_th}, OffTh={off_th}")
            return

        # 4. Match Events
        m_evt = RE_EVENT.search(line)
        if m_evt:
            evt_text = m_evt.group(1)
            with self.lock:
                latest_val = self.samples[-1][2] if self.samples else 0
                self.events.append((now, self.sample_counter, evt_text, latest_val))
                self.pending_event = evt_text
            print(f"*** EVENT TRIGGERED: {evt_text} ***")
            return

    def _run_mock(self):
        """Simulate realistic QVAR readings at 40 Hz (25 ms interval)."""
        self.connected = True
        self.status_message = "MOCK MODE (Simulation @ 40Hz)"
        print("[*] Running in mock simulation mode...")

        q1_base = 1200
        q1_press_delta = 950
        q1_rel_delta = 350

        with self.lock:
            self.q1_baseline = q1_base
            self.q1_press_th = q1_press_delta
            self.q1_rel_th = q1_rel_delta
            self.q2_baseline = 800
            self.q2_on_th = 700
            self.q2_off_th = 250

        t = 0.0
        btn_active = False
        btn_hold_counter = 0

        while self.running:
            start_t = time.time()
            t += 0.025

            # Base signal with slight environmental wobble and random noise
            noise = random.gauss(0, 15)
            drift = 50 * math.sin(2 * math.pi * 0.1 * t)
            q1 = q1_base + drift + noise
            q2 = 800 + noise * 0.5

            mock_evt = ""
            # Random touch simulation
            if not btn_active and random.random() < 0.015:
                btn_active = True
                btn_hold_counter = random.choice([8, 30])  # Short tap or long press

            if btn_active:
                btn_hold_counter -= 1
                q1 += 1400  # Jump during touch
                if btn_hold_counter <= 0:
                    btn_active = False
                    mock_evt = "Q1 BUTTON SINGLE" if random.random() > 0.3 else "Q1 BUTTON HOLD RELEASE"
                    with self.lock:
                        self.events.append((start_t, self.sample_counter, mock_evt, q1))
                    print(f"*** MOCK EVENT: {mock_evt} ***")

            with self.lock:
                self.sample_counter += 1
                item = (start_t, self.sample_counter, int(q1), True, int(q2), False)
                self.samples.append(item)

                if self.csv_writer:
                    iso_time = datetime.fromtimestamp(start_t).strftime("%Y-%m-%dT%H:%M:%S.%f")[:-3]
                    self.csv_writer.writerow([
                        iso_time,
                        f"{start_t:.4f}",
                        self.sample_counter,
                        int(q1),
                        1,
                        int(q2),
                        0,
                        self.q1_baseline,
                        self.q2_baseline,
                        mock_evt,
                    ])
                    if self.csv_file:
                        self.csv_file.flush()

            elapsed = time.time() - start_t
            sleep_time = max(0.001, 0.025 - elapsed)
            time.sleep(sleep_time)


class RealtimeQvarPlotter:
    """Modern dark-themed real-time matplotlib dashboard for QVAR."""

    def __init__(self, receiver, window_size=200):
        self.receiver = receiver
        self.window_size = window_size

        # Rolling display deques
        self.xs = collections.deque(maxlen=window_size)
        self.q1_ys = collections.deque(maxlen=window_size)
        self.q2_ys = collections.deque(maxlen=window_size)

        # Setup figure and dark aesthetics
        plt.style.use("dark_background")
        self.fig, self.ax = plt.subplots(figsize=(11, 6), dpi=100)
        self.fig.canvas.manager.set_window_title("ISM330BX QVAR Live Monitor")

        # Visual styling
        self.fig.patch.set_facecolor("#121417")
        self.ax.set_facecolor("#1a1d24")
        self.ax.grid(True, linestyle="--", alpha=0.25, color="#5a6578")

        # Plot curves
        (self.line_q1,) = self.ax.plot(
            [], [], color="#00e5ff", linewidth=2.0, label="Q1 Electrode (Button)", zorder=3
        )
        (self.line_q2,) = self.ax.plot(
            [], [], color="#ff9100", linewidth=1.8, linestyle="--", label="Q2 Electrode (Wear)", zorder=2
        )

        # Baseline & threshold guides (horizontal lines)
        self.hl_baseline = self.ax.axhline(0, color="#76ff03", linestyle=":", alpha=0.0, label="Q1 Baseline")
        self.hl_press_th = self.ax.axhline(0, color="#ff1744", linestyle="--", alpha=0.0, label="Press Threshold")
        self.hl_rel_th = self.ax.axhline(0, color="#ffd600", linestyle="-.", alpha=0.0, label="Release Threshold")

        # Event scatter points
        self.event_scatter = self.ax.scatter([], [], color="#ffea00", s=90, marker="o", zorder=5, label="Events")
        self.event_markers = collections.deque(maxlen=20)  # (x, y, text, timestamp)

        # Labels & title
        self.ax.set_title("ISM330BX QVAR Real-Time Telemetry", fontsize=14, fontweight="bold", color="#f0f3f8", pad=12)
        self.ax.set_xlabel("Sample Index", fontsize=10, color="#b0bec5")
        self.ax.set_ylabel("QVAR Amplitude (LSB) [78 LSB = 1 mV]", fontsize=10, color="#b0bec5")
        self.ax.tick_params(colors="#90a4ae")

        # Legend
        self.legend = self.ax.legend(loc="upper right", framealpha=0.6, facecolor="#1f2430", edgecolor="#37474f")

        # Status text overlays
        self.info_text = self.fig.text(
            0.02,
            0.02,
            "Connecting...",
            fontsize=9,
            color="#81c784",
            fontfamily="monospace",
        )
        self.event_hud = self.fig.text(
            0.70,
            0.02,
            "Latest Event: None",
            fontsize=9,
            color="#ffd54f",
            fontfamily="monospace",
        )

        self.last_event_str = "None"
        self.last_event_time = 0

    def update(self, frame):
        # Pull latest samples safely
        with self.receiver.lock:
            # Drain new samples
            while self.receiver.samples:
                now, idx, q1, q1_valid, q2, q2_valid = self.receiver.samples.popleft()
                self.xs.append(idx)
                self.q1_ys.append(q1 if q1_valid else float("nan"))
                self.q2_ys.append(q2 if q2_valid else float("nan"))

            # Drain new events
            while self.receiver.events:
                evt_t, evt_idx, evt_name, evt_val = self.receiver.events.popleft()
                self.event_markers.append((evt_idx, evt_val, evt_name, evt_t))
                self.last_event_str = f"{evt_name} (sample #{evt_idx})"
                self.last_event_time = evt_t

            # Capture states
            base = self.receiver.q1_baseline
            press_th = self.receiver.q1_press_th
            rel_th = self.receiver.q1_rel_th
            status = self.receiver.status_message
            connected = self.receiver.connected

        if not self.xs:
            self.info_text.set_text(status)
            return self.line_q1, self.line_q2

        # Update curve data
        self.line_q1.set_data(self.xs, self.q1_ys)
        self.line_q2.set_data(self.xs, self.q2_ys)

        # Update thresholds if available
        if base is not None:
            self.hl_baseline.set_ydata([base, base])
            self.hl_baseline.set_alpha(0.85)

            if press_th is not None:
                th_val = base + press_th if press_th < 2000 else press_th
                self.hl_press_th.set_ydata([th_val, th_val])
                self.hl_press_th.set_alpha(0.75)

            if rel_th is not None:
                rel_val = base + rel_th if rel_th < 2000 else rel_th
                self.hl_rel_th.set_ydata([rel_val, rel_val])
                self.hl_rel_th.set_alpha(0.75)

        # Update event markers inside current window
        min_x = self.xs[0]
        max_x = self.xs[-1]
        active_evts = [e for e in self.event_markers if min_x <= e[0] <= max_x]
        if active_evts:
            ev_x = [e[0] for e in active_evts]
            ev_y = [e[1] for e in active_evts]
            self.event_scatter.set_offsets(list(zip(ev_x, ev_y)))
        else:
            self.event_scatter.set_offsets(np.empty((0, 2)))

        # Adjust axes
        self.ax.set_xlim(min_x, max(max_x, min_x + 10))

        # Dynamic auto-scaling with padding
        valid_vals = [v for v in self.q1_ys if not math.isnan(v)]
        if self.q2_ys:
            valid_vals.extend([v for v in self.q2_ys if not math.isnan(v)])
        if base is not None:
            valid_vals.append(base)

        if valid_vals:
            y_min = min(valid_vals)
            y_max = max(valid_vals)
            pad = max(50, (y_max - y_min) * 0.2)
            self.ax.set_ylim(y_min - pad, y_max + pad)

        # Format info HUD text (using ST AN5755 Section 4.3 Gain: 78 LSB/mV)
        if self.q1_ys and not math.isnan(self.q1_ys[-1]):
            latest_lsb = self.q1_ys[-1]
            latest_mv = latest_lsb / 78.0
            latest_q1_str = f"{latest_lsb:.0f} LSB ({latest_mv:.1f} mV)"
        else:
            latest_q1_str = "N/A"

        if base is not None:
            base_mv = base / 78.0
            latest_base_str = f"{base} LSB ({base_mv:.1f} mV)"
        else:
            latest_base_str = "Learning..."

        status_color = "#81c784" if connected else "#e57373"
        self.info_text.set_color(status_color)
        self.info_text.set_text(
            f"[{status}] | #{max_x} | Q1: {latest_q1_str} | Base: {latest_base_str}"
        )

        # Event HUD
        time_since_evt = time.time() - self.last_event_time
        if self.last_event_time > 0 and time_since_evt < 4.0:
            self.event_hud.set_text(f"Event: {self.last_event_str}")
            self.event_hud.set_color("#ffeb3b")
        else:
            self.event_hud.set_text("Event: Idle")
            self.event_hud.set_color("#78909c")

        return (
            self.line_q1,
            self.line_q2,
            self.hl_baseline,
            self.hl_press_th,
            self.hl_rel_th,
            self.event_scatter,
            self.info_text,
            self.event_hud,
        )

    def show(self):
        ani = animation.FuncAnimation(
            self.fig, self.update, interval=30, blit=False, cache_frame_data=False
        )
        try:
            plt.tight_layout(rect=[0, 0.05, 1, 0.98])
            plt.show()
        except KeyboardInterrupt:
            plt.close(self.fig)


def parse_args():
    parser = argparse.ArgumentParser(
        description="Real-time graphical visualizer for ISM330BX QVAR serial telemetry with auto-CSV archiving."
    )
    parser.add_argument(
        "-p", "--port", default="COM7", help="Serial COM port (default: COM7)."
    )
    parser.add_argument(
        "-b", "--baud", type=int, default=115200, help="Serial baud rate (default: 115200)."
    )
    parser.add_argument(
        "-w",
        "--window",
        type=int,
        default=250,
        help="Number of samples in the rolling window (default: 250).",
    )
    parser.add_argument(
        "-r",
        "--record",
        type=str,
        default=None,
        help="Custom CSV file path. If omitted, files are automatically saved to project/data/.",
    )
    parser.add_argument(
        "--data-dir",
        type=str,
        default=DEFAULT_DATA_DIR,
        help=f"Target directory for automatic CSV records (default: {DEFAULT_DATA_DIR}).",
    )
    parser.add_argument(
        "--no-record",
        action="store_true",
        help="Disable automatic CSV recording.",
    )
    parser.add_argument(
        "--mock",
        action="store_true",
        help="Simulate realistic QVAR data stream without hardware.",
    )
    parser.add_argument(
        "-v",
        "--verbose",
        action="store_true",
        help="Echo all raw serial lines to stdout.",
    )
    return parser.parse_args()


def main():
    args = parse_args()

    # Clean signal handling for Ctrl+C
    def _sigint_handler(signum, frame):
        print("\n[*] Ctrl+C received, closing plotter...")
        plt.close("all")

    try:
        signal.signal(signal.SIGINT, _sigint_handler)
    except (ValueError, AttributeError):
        pass

    csv_file = None
    csv_writer = None
    resolved_csv_path = None

    if not args.no_record:
        if args.record:
            resolved_csv_path = args.record
        else:
            os.makedirs(args.data_dir, exist_ok=True)
            timestamp_str = datetime.now().strftime("%Y%m%d_%H%M%S")
            resolved_csv_path = os.path.join(args.data_dir, f"qvar_telemetry_{timestamp_str}.csv")

        print(f"[*] Archiving QVAR telemetry to: {resolved_csv_path}")
        parent_dir = os.path.dirname(os.path.abspath(resolved_csv_path))
        os.makedirs(parent_dir, exist_ok=True)
        csv_file = open(resolved_csv_path, mode="w", newline="", encoding="utf-8")
        csv_writer = csv.writer(csv_file)
        csv_writer.writerow([
            "iso_time",
            "epoch_seconds",
            "sample_index",
            "q1_raw",
            "q1_valid",
            "q2_raw",
            "q2_valid",
            "q1_baseline",
            "q2_baseline",
            "event",
        ])
        csv_file.flush()

    # Detect available ports if not in mock mode
    if not args.mock and serial is not None:
        available = [p.device for p in serial.tools.list_ports.comports()]
        if args.port not in available:
            print(f"[!] Warning: Port {args.port} was not found in available system ports: {available}")
            print(f"[!] If running in ESP-IDF, remember to close 'idf.py monitor' first.")

    receiver = TelemetryReceiver(
        port=args.port,
        baud=args.baud,
        is_mock=args.mock,
        csv_writer=csv_writer,
        csv_file=csv_file,
        verbose=args.verbose,
    )

    plotter = RealtimeQvarPlotter(receiver, window_size=args.window)

    try:
        receiver.start()
        plotter.show()
    finally:
        print("\n[*] Shutting down serial receiver...")
        receiver.stop()
        if csv_file:
            csv_file.flush()
            csv_file.close()
            print(f"[+] Recording saved to: {resolved_csv_path}")
        print("[+] Done.")


if __name__ == "__main__":
    main()
