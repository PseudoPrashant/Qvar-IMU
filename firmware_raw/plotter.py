#!/usr/bin/env python3
r"""
200 Hz Raw & 4-Sample Peak-to-Peak Activity Envelope Plotter & CSV Logger
=========================================================================
Real-time oscilloscope visualizer for ISM330BX QVAR telemetry streaming at 200 Hz.
Displays:
  - Cyan Trace: Raw Q1 Signal (showing high-resolution 200 Hz AC waveform)
  - Amber Gold Trace: 4-Sample Peak-to-Peak Activity Envelope (solid unipolar touch pulse)

Features:
  - Live Serial Streaming: Reads from COM7 (or user-specified port) in real time.
  - Dual-Trace Oscilloscope: Displays raw and envelope signals side-by-side.
  - CSV Logging: Records both raw and envelope channels with ISO timestamps.
  - Monotonic Auto-Expansion: Smooth, non-jittery Y-axis scaling.
  - CSV Viewer Mode: Can open and plot any existing recorded CSV session.

Usage:
  python plotter.py                    # Stream live from COM7 & log to data/
  python plotter.py --port COM7        # Specify serial port
  python plotter.py data/session.csv   # Plot recorded CSV waveform
  python plotter.py --window 500       # Adjust display window size (default: 500 samples = 2 sec @ 250 Hz)
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

# Regex to extract Q1 raw sample and optional Q1_ACT envelope from ESP32 serial stream
# Matches: [IMU QVAR RAW] Q1=1240 Q1_ACT=1180 Q2=... or Q1=-52
RE_TELEMETRY = re.compile(r"\[IMU QVAR RAW\]\s+Q1=(?:NA:)?(-?\d+)(?:\s+Q1_ACT=(?:NA:)?(-?\d+))?")
RE_TAP = re.compile(r"\[IMU QVAR TAP\]\s+#(\d+)\s+dur=(\d+)\s*ms\s+peak=(\d+)(?:\s+LSB)?(?:\s+base=([\d\.]+))?")
DEFAULT_DATA_DIR = r"C:\Users\prash\OneDrive\Desktop\IMU\IMU\firmware_raw\data"


class DualQ1SerialReader:
    """Reads serial telemetry, extracts raw & activity Q1 samples, and logs to CSV."""

    def __init__(self, port, baud, max_samples=3000, data_dir=None, csv_filename=None):
        self.port = port
        self.baud = baud
        self.max_samples = max_samples
        self.data_dir = data_dir or DEFAULT_DATA_DIR
        self.lock = threading.Lock()
        self.running = True
        self.connected = False
        self.status = "Connecting..."
        self.sample_idx = 0
        self.has_act = False

        # CSV Logging Setup
        os.makedirs(self.data_dir, exist_ok=True)
        if not csv_filename:
            ts = time.strftime("%Y%m%d_%H%M%S")
            csv_filename = f"q1_env200_{ts}.csv"
        self.csv_path = os.path.join(self.data_dir, csv_filename)
        self.csv_file = None
        self.csv_writer = None
        try:
            self.csv_file = open(self.csv_path, "w", newline="", encoding="utf-8")
            self.csv_writer = csv.writer(self.csv_file)
            self.csv_writer.writerow([
                "iso_time", "epoch_seconds", "sample_index",
                "q1_raw", "q1_voltage_mv", "q1_activity", "q1_activity_mv"
            ])
            self.csv_file.flush()
            print(f"[+] Recording dual-channel 200 Hz telemetry to CSV: {self.csv_path}")
        except Exception as e:
            print(f"[!] Warning: Failed to initialize CSV logging: {e}")

        # Data Deques
        self.indices = collections.deque(maxlen=max_samples)
        self.raw_vals = collections.deque(maxlen=max_samples)
        self.act_vals = collections.deque(maxlen=max_samples)
        self.tap_events = collections.deque(maxlen=100)
        self.latest_tap_info = None
        self.latest_tap_time = 0.0

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

                m_tap = RE_TAP.search(line)
                if m_tap:
                    t_id = int(m_tap.group(1))
                    t_dur = int(m_tap.group(2))
                    t_peak = int(m_tap.group(3))
                    with self.lock:
                        self.tap_events.append((self.sample_idx, t_id, t_dur, t_peak))
                        self.latest_tap_info = (t_id, t_dur, t_peak)
                        self.latest_tap_time = time.time()

                m = RE_TELEMETRY.search(line)
                if m:
                    raw_v = int(m.group(1))
                    act_v = int(m.group(2)) if m.group(2) is not None else abs(raw_v)
                    if m.group(2) is not None:
                        self.has_act = True

                    now_epoch = time.time()
                    iso_now = (
                        time.strftime("%Y-%m-%dT%H:%M:%S", time.localtime(now_epoch))
                        + f".{int((now_epoch % 1) * 1000):03d}"
                    )
                    raw_mv = round(raw_v / 78.0, 2)
                    act_mv = round(act_v / 78.0, 2)

                    with self.lock:
                        self.sample_idx += 1
                        cur_idx = self.sample_idx
                        self.indices.append(cur_idx)
                        self.raw_vals.append(raw_v)
                        self.act_vals.append(act_v)

                    # Save to CSV
                    if self.csv_writer:
                        try:
                            self.csv_writer.writerow([
                                iso_now, f"{now_epoch:.4f}", cur_idx,
                                raw_v, raw_mv, act_v, act_mv
                            ])
                            if cur_idx % 25 == 0:
                                self.csv_file.flush()
                        except Exception:
                            pass
            except Exception as e:
                self.status = f"Read error: {e}"
                time.sleep(0.02)

        ser.close()
        self.connected = False
        self.status = "Disconnected"

    def get_data(self):
        with self.lock:
            return (
                list(self.indices),
                list(self.raw_vals),
                list(self.act_vals),
                self.status,
                self.connected,
                self.csv_path,
                self.sample_idx,
                self.has_act,
                list(self.tap_events),
                self.latest_tap_info,
                self.latest_tap_time,
            )

    def stop(self):
        self.running = False
        if self.csv_file:
            try:
                self.csv_file.flush()
                self.csv_file.close()
                print(f"[+] Saved {self.sample_idx} samples to: {self.csv_path}")
            except Exception:
                pass
            self.csv_file = None


def plot_live(port, baud, window_size, ylim=None, data_dir=None, csv_filename=None):
    """Run real-time dark-theme dual-trace waveform plot."""
    reader = DualQ1SerialReader(
        port, baud, max_samples=max(3000, window_size * 2),
        data_dir=data_dir, csv_filename=csv_filename
    )

    # Styling
    plt.style.use("dark_background")
    fig, ax = plt.subplots(figsize=(12, 6.5))
    fig.canvas.manager.set_window_title(f"QVAR 200 Hz Envelope Extractor - {port}")
    fig.patch.set_facecolor("#0b0f19")
    ax.set_facecolor("#111827")

    # Trace 1: Raw Q1 in semi-transparent cyan
    (line_raw,) = ax.plot([], [], color="#00e5ff", alpha=0.45, linewidth=1.2, label="Q1 Raw (200 Hz AC Wave)")

    # Trace 2: 4-Sample Peak-to-Peak Activity Envelope in Amber Gold
    (line_act,) = ax.plot([], [], color="#fbbf24", linewidth=2.4, label="Q1 Activity Envelope (4-Sample P2P)")

    # Zero baseline reference
    ax.axhline(0, color="#64748b", linestyle="-", alpha=0.4, linewidth=0.8, label="Zero Baseline")

    # Band-Pass State Machine Thresholds
    thresh_lower = ax.axhline(3500, color="#fbbf24", linestyle="--", alpha=0.7, linewidth=1.0, label="Lower Floor (3500 LSB)")
    thresh_upper = ax.axhline(5500, color="#f87171", linestyle="--", alpha=0.7, linewidth=1.0, label="Upper Ceiling (5500 LSB)")
    (line_tap_markers,) = ax.plot([], [], "o", color="#facc15", markersize=9, markeredgecolor="#ffffff", markeredgewidth=1.5, label="Detected Taps")

    # Grid and styling
    ax.grid(True, linestyle="--", alpha=0.25, color="#475569")
    ax.set_xlabel("Sample Index (200 Hz = 5 ms / sample)", color="#94a3b8", fontsize=11, labelpad=8)
    ax.set_ylabel("Amplitude (LSB)", color="#94a3b8", fontsize=11, labelpad=8)
    ax.tick_params(colors="#64748b", labelsize=10)
    for spine in ax.spines.values():
        spine.set_color("#334155")
    ax.legend(loc="upper right", facecolor="#1e293b", edgecolor="#475569", fontsize=9)

    # HUD Texts
    title_text = ax.text(
        0.02, 0.94, f"QVAR 240 HZ ENVELOPE EXTRACTOR ({port})",
        transform=ax.transAxes, color="#f8fafc",
        fontsize=12, weight="bold"
    )
    hud_text = ax.text(
        0.02, 0.79, "Waiting for telemetry...",
        transform=ax.transAxes, color="#38bdf8",
        fontsize=9.5, family="monospace"
    )
    status_text = ax.text(
        0.98, 0.94, reader.status,
        transform=ax.transAxes, color="#81c784",
        fontsize=10, horizontalalignment="right", family="monospace"
    )

    y_min_hist = -500.0
    y_max_hist = 5000.0

    def update(frame):
        nonlocal y_min_hist, y_max_hist
        xs, r_ys, a_ys, status, connected, csv_path, total_samples, has_act, tap_evs, last_tap, last_tap_time = reader.get_data()

        status_text.set_text(status)
        status_text.set_color("#4ade80" if connected else "#f87171")

        if not xs:
            return line_raw, line_act, hud_text, status_text

        # Slice rolling window
        disp_xs = xs[-window_size:]
        disp_raw = r_ys[-window_size:]
        disp_act = a_ys[-window_size:]

        line_raw.set_data(disp_xs, disp_raw)
        line_act.set_data(disp_xs, disp_act)

        # X-axis limits
        min_x = disp_xs[0]
        max_x = disp_xs[-1]
        ax.set_xlim(min_x, max(max_x, min_x + 10))

        # Y-axis auto-scaling
        all_ys = disp_raw + disp_act
        curr_min = min(all_ys)
        curr_max = max(all_ys)
        y_min_hist = min(y_min_hist, curr_min)
        y_max_hist = max(y_max_hist, curr_max)

        if ylim is not None:
            ax.set_ylim(ylim[0], ylim[1])
        else:
            span = y_max_hist - y_min_hist
            pad = max(200.0, span * 0.12)
            ax.set_ylim(y_min_hist - pad, y_max_hist + pad)

        # Plot confirmed tap events within visible window
        if tap_evs:
            visible_taps = [t for t in tap_evs if min_x <= t[0] <= max_x]
            if visible_taps:
                t_xs = [t[0] for t in visible_taps]
                t_ys = [t[3] for t in visible_taps]
                line_tap_markers.set_data(t_xs, t_ys)
            else:
                line_tap_markers.set_data([], [])
        else:
            line_tap_markers.set_data([], [])

        # HUD calculation matching the firmware State Machine
        latest_raw = disp_raw[-1]
        latest_act = disp_act[-1]
        raw_mv = latest_raw / 78.0
        act_mv = latest_act / 78.0

        now = time.time()
        if last_tap and (now - last_tap_time) < 1.5:
            t_id, t_dur, t_peak = last_tap
            state_str = f"*** TAP CONFIRMED #{t_id} (dur={t_dur} ms, peak={t_peak} LSB) ***"
            status_color = "#facc15"  # bright yellow
        elif latest_act > 5500:
            state_str = "REJECTED (ABOVE 5500 LSB UPPER CEILING - KILL-SWITCH)"
            status_color = "#f87171"  # red
        elif latest_act > 3500:
            state_str = "IN-BAND CANDIDATE EVENT (3500 - 5500 LSB)"
            status_color = "#38bdf8"  # cyan
        else:
            state_str = "IDLE (BELOW 3500 LSB NOISE FLOOR)"
            status_color = "#94a3b8"  # gray

        csv_basename = os.path.basename(csv_path) if csv_path else "None"

        hud_text.set_text(
            f"Raw Q1   : {latest_raw:+6d} LSB ({raw_mv:+6.1f} mV)\n"
            f"Activity : {latest_act:6d} LSB ({act_mv:5.1f} mV) [5-Sample P2P Envelope]\n"
            f"Status   : {state_str}\n"
            f"Firmware Taps: {len(tap_evs)} | Logging: data/{csv_basename}"
        )
        hud_text.set_color(status_color)

        return line_raw, line_act, line_tap_markers, hud_text, status_text, title_text

    ani = animation.FuncAnimation(fig, update, interval=25, blit=False, cache_frame_data=False)

    try:
        plt.tight_layout()
        plt.show()
    finally:
        reader.stop()


def plot_csv_file(csv_path):
    """Plot an entire recorded CSV session with dual-channel support."""
    if not os.path.exists(csv_path):
        print(f"[!] Error: File '{csv_path}' does not exist.")
        sys.exit(1)

    print(f"[*] Reading telemetry from: {csv_path}...")
    xs = []
    raw_ys = []
    act_ys = []
    has_act = False

    with open(csv_path, "r", encoding="utf-8") as f:
        reader = csv.DictReader(f)
        for row in reader:
            try:
                idx = int(row.get("sample_index", len(xs) + 1))
                q_raw = int(float(row.get("q1_raw", 0)))
                xs.append(idx)
                raw_ys.append(q_raw)
                if "q1_activity" in row and row["q1_activity"] != "":
                    act_ys.append(int(float(row["q1_activity"])))
                    has_act = True
                else:
                    act_ys.append(abs(q_raw))
            except (ValueError, KeyError):
                continue

    if not xs:
        print("[!] Error: No valid samples found in file.")
        sys.exit(1)

    print(f"[+] Loaded {len(xs):,} samples (Has Activity Envelope: {has_act}).")

    plt.style.use("dark_background")
    fig, ax = plt.subplots(figsize=(12, 6.5))
    fig.canvas.manager.set_window_title(f"QVAR CSV Viewer - {os.path.basename(csv_path)}")
    fig.patch.set_facecolor("#0b0f19")
    ax.set_facecolor("#111827")

    # Plot raw waveform
    ax.plot(xs, raw_ys, color="#00e5ff", alpha=0.4 if has_act else 1.0,
            linewidth=1.2 if has_act else 1.8, label="Q1 Raw")

    # Plot activity envelope if available
    if has_act:
        ax.plot(xs, act_ys, color="#fbbf24", linewidth=2.0, label="Q1 Activity Envelope (P2P)")

    ax.axhline(0, color="#64748b", linestyle="-", alpha=0.4, linewidth=0.8, label="Zero Baseline")
    ax.axhline(3000, color="#f87171", linestyle="--", alpha=0.6, linewidth=1.0, label="Touch Threshold (3000 LSB)")
    ax.grid(True, linestyle="--", alpha=0.25, color="#475569")
    ax.set_xlabel("Sample Index (200 Hz = 5 ms / sample)", color="#94a3b8", fontsize=11)
    ax.set_ylabel("Amplitude (LSB)", color="#94a3b8", fontsize=11)
    ax.legend(loc="upper right", facecolor="#1e293b", edgecolor="#475569")
    plt.tight_layout()
    plt.show()


def main():
    parser = argparse.ArgumentParser(description="Live 200 Hz Dual-Trace Oscilloscope & CSV Logger for ISM330BX QVAR")
    parser.add_argument("csv_file", nargs="?", default=None, help="Path to CSV file to view")
    parser.add_argument("--port", default="COM7", help="Serial port (default: COM7)")
    parser.add_argument("--baud", type=int, default=115200, help="Baud rate (default: 115200)")
    parser.add_argument("--window", type=int, default=500, help="Display window in samples (default: 500 = 2 sec @ 250 Hz)")
    parser.add_argument("--ylim", type=float, nargs=2, default=None, help="Fixed Y limits: --ylim MIN MAX")
    parser.add_argument("--csv-name", default=None, help="Custom CSV filename")
    args = parser.parse_args()

    if args.csv_file:
        plot_csv_file(args.csv_file)
    else:
        plot_live(
            port=args.port,
            baud=args.baud,
            window_size=args.window,
            ylim=args.ylim,
            csv_filename=args.csv_name
        )


if __name__ == "__main__":
    main()
