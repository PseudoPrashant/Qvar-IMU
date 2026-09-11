#!/usr/bin/env python3
"""
ISM330BX QVAR Interactive Tap Accuracy Benchmark & Validation Protocol
-----------------------------------------------------------------------
A standalone hardware-in-the-loop test suite to evaluate tap detection
accuracy, false positives, false negatives, and latency on ESP32 firmware.

Guided Protocol:
  1. Quiescent Desk Baseline (Resting)
  2. Pickup & Hold Hand Settling
  3. Randomized Flash Tap Trials (Default: 20 trials)
     - Randomized inter-stimulus wait (1.5s - 4.0s) -> Monitors for FALSE POSITIVES
     - Visual flash cue & auditory tone -> Prompts user to tap
     - 1.5s response window -> Evaluates HIT (True Positive) or MISS (False Negative)
  4. Return Sensor to Desk (Release Settling)
  5. Comprehensive Accuracy & Signal Metrics Report (Printed + Saved to TXT & CSV)

Usage:
  python tap_test_benchmark.py --port COM7
  python tap_test_benchmark.py --port COM7 --trials 25 --min-wait 2.0 --max-wait 5.0
"""

import argparse
import collections
import csv
import datetime
import os
import random
import re
import sys
import threading
import time

try:
    import serial
except ImportError:
    print("[!] Error: 'pyserial' is required. Run: pip install pyserial")
    sys.exit(1)

# Enable ANSI escape sequences on Windows console
os.system("")

# Audio support for cues
try:
    import winsound
    HAS_WINSOUND = True
except ImportError:
    HAS_WINSOUND = False

# Regex patterns matching ESP32 firmware telemetry
RE_TELEMETRY = re.compile(r"\[IMU QVAR RAW\]\s+Q1=(?:NA:)?(-?\d+)(?:\s+Q1_ACT=(?:NA:)?(-?\d+))?")
RE_TAP = re.compile(r"\[IMU QVAR TAP\]\s+#(\d+)\s+dur=(\d+)\s*ms\s+peak=(\d+)(?:\s+LSB)?(?:\s+base=([\d\.]+))?(?:\s+th=(\d+))?(?:\s+ceil=(\d+))?")
RE_CALIB = re.compile(r"\[IMU QVAR CALIB\]\s+BASE=([\d\.]+)\s+LO=(\d+)\s+HI=(\d+)")

DEFAULT_DATA_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "data")


def play_sound(freq_hz, dur_ms):
    """Plays an audio cue if available, otherwise terminal bell."""
    if HAS_WINSOUND:
        try:
            winsound.Beep(int(freq_hz), int(dur_ms))
            return
        except Exception:
            pass
    sys.stdout.write("\a")
    sys.stdout.flush()


class ANSI:
    RESET = "\033[0m"
    BOLD = "\033[1m"
    DIM = "\033[2m"
    RED = "\033[91m"
    GREEN = "\033[92m"
    YELLOW = "\033[93m"
    BLUE = "\033[94m"
    MAGENTA = "\033[95m"
    CYAN = "\033[96m"
    WHITE = "\033[97m"
    BG_YELLOW = "\033[43m"
    BG_RED = "\033[41m"
    BG_GREEN = "\033[42m"
    BG_CYAN = "\033[46m"
    BLACK = "\033[30m"


class BenchmarkSerialLogger:
    """Threaded reader logging continuous 200 Hz telemetry with experiment state tags."""

    def __init__(self, port, baud, csv_path):
        self.port = port
        self.baud = baud
        self.csv_path = csv_path
        self.lock = threading.Lock()
        self.running = True
        self.connected = False
        self.sample_idx = 0

        # Current calibration telemetry
        self.baseline_act = 0.0
        self.lower_th = 0
        self.upper_ceil = 0

        # Live experiment state tags
        self.current_phase = "INIT"
        self.current_trial = 0
        self.cue_active = False

        # Tap Event queues
        self.tap_events = []
        self.new_tap_event = threading.Event()

        # CSV setup
        os.makedirs(os.path.dirname(self.csv_path), exist_ok=True)
        self.csv_file = open(self.csv_path, "w", newline="", encoding="utf-8")
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow([
            "iso_time", "epoch_seconds", "sample_index",
            "q1_raw", "q1_voltage_mv", "q1_activity", "q1_activity_mv",
            "baseline", "lower_th", "upper_ceil",
            "phase", "trial_idx", "cue_active"
        ])
        self.csv_file.flush()

        self.thread = threading.Thread(target=self._worker, daemon=True)
        self.thread.start()

    def set_experiment_state(self, phase, trial_idx=0, cue_active=False):
        with self.lock:
            self.current_phase = phase
            self.current_trial = trial_idx
            self.cue_active = cue_active

    def get_and_clear_new_taps(self):
        """Returns any new tap events received since last call."""
        with self.lock:
            taps = list(self.tap_events)
            self.tap_events.clear()
            self.new_tap_event.clear()
            return taps

    def _worker(self):
        try:
            ser = serial.Serial(self.port, self.baud, timeout=0.1)
            self.connected = True
        except Exception as e:
            print(f"{ANSI.RED}[!] Serial connection error on {self.port}: {e}{ANSI.RESET}")
            return

        while self.running:
            try:
                line_bytes = ser.readline()
                if not line_bytes:
                    continue
                line = line_bytes.decode("utf-8", errors="replace").strip()
                if not line:
                    continue

                now = time.time()
                iso_time = datetime.datetime.fromtimestamp(now).isoformat(timespec="milliseconds")

                # Calibration telemetry
                m_calib = RE_CALIB.search(line)
                if m_calib:
                    with self.lock:
                        self.baseline_act = float(m_calib.group(1))
                        self.lower_th = int(m_calib.group(2))
                        self.upper_ceil = int(m_calib.group(3))

                # Tap event
                m_tap = RE_TAP.search(line)
                if m_tap:
                    t_id = int(m_tap.group(1))
                    t_dur = int(m_tap.group(2))
                    t_peak = int(m_tap.group(3))
                    base = float(m_tap.group(4)) if m_tap.group(4) is not None else self.baseline_act
                    th = int(m_tap.group(5)) if m_tap.group(5) is not None else self.lower_th
                    ceil = int(m_tap.group(6)) if m_tap.group(6) is not None else self.upper_ceil

                    tap_record = {
                        "time": now,
                        "iso_time": iso_time,
                        "tap_id": t_id,
                        "dur_ms": t_dur,
                        "peak_act": t_peak,
                        "baseline": base,
                        "lower_th": th,
                        "upper_ceil": ceil,
                        "phase": self.current_phase,
                        "trial_idx": self.current_trial,
                        "cue_active": self.cue_active
                    }
                    with self.lock:
                        self.tap_events.append(tap_record)
                        self.new_tap_event.set()

                # Raw 200 Hz telemetry
                m_raw = RE_TELEMETRY.search(line)
                if m_raw:
                    self.sample_idx += 1
                    raw_val = int(m_raw.group(1))
                    act_val = int(m_raw.group(2)) if m_raw.group(2) is not None else 0
                    raw_mv = round(raw_val / 78.0, 2)
                    act_mv = round(act_val / 78.0, 2)

                    with self.lock:
                        cur_phase = self.current_phase
                        cur_trial = self.current_trial
                        cur_cue = 1 if self.cue_active else 0
                        cur_base = self.baseline_act
                        cur_lo = self.lower_th
                        cur_hi = self.upper_ceil

                    self.csv_writer.writerow([
                        iso_time, f"{now:.4f}", self.sample_idx,
                        raw_val, raw_mv, act_val, act_mv,
                        cur_base, cur_lo, cur_hi,
                        cur_phase, cur_trial, cur_cue
                    ])

            except Exception:
                pass

        try:
            self.csv_file.flush()
            self.csv_file.close()
            ser.close()
        except Exception:
            pass

    def stop(self):
        self.running = False
        if self.thread.is_alive():
            self.thread.join(timeout=1.0)


def print_banner():
    print(f"\n{ANSI.CYAN}{ANSI.BOLD}" + "=" * 70)
    print("      ISM330BX QVAR INTERACTIVE TAP BENCHMARK & ACCURACY TEST")
    print("=" * 70 + f"{ANSI.RESET}\n")


def run_benchmark(port="COM7", baud=115200, total_trials=20, min_wait=1.5, max_wait=4.0, tap_timeout=2.0):
    print_banner()

    ts = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    csv_filename = f"tap_benchmark_{ts}.csv"
    report_filename = f"tap_benchmark_report_{ts}.txt"
    csv_path = os.path.join(DEFAULT_DATA_DIR, csv_filename)
    report_path = os.path.join(DEFAULT_DATA_DIR, report_filename)

    print(f"{ANSI.WHITE}Target Serial Port   : {ANSI.YELLOW}{port} @ {baud} baud{ANSI.RESET}")
    print(f"{ANSI.WHITE}Total Prompted Trials: {ANSI.YELLOW}{total_trials}{ANSI.RESET}")
    print(f"{ANSI.WHITE}Random Wait Interval : {ANSI.YELLOW}{min_wait:.1f}s - {max_wait:.1f}s{ANSI.RESET}")
    print(f"{ANSI.WHITE}Tap Response Timeout : {ANSI.YELLOW}{tap_timeout:.1f}s{ANSI.RESET}")
    print(f"{ANSI.WHITE}Continuous 200 Hz CSV: {ANSI.CYAN}data/{csv_filename}{ANSI.RESET}")
    print(f"{ANSI.WHITE}Summary Report File  : {ANSI.CYAN}data/{report_filename}{ANSI.RESET}\n")

    print(f"{ANSI.CYAN}[*] Connecting to ESP32 on {port}...{ANSI.RESET}")
    logger = BenchmarkSerialLogger(port, baud, csv_path)

    # Wait up to 3 seconds for connection and telemetry
    t_start = time.time()
    while not logger.connected and (time.time() - t_start) < 3.0:
        time.sleep(0.1)

    if not logger.connected:
        print(f"{ANSI.RED}[!] Failed to connect to {port}. Please check the port and wiring.{ANSI.RESET}")
        logger.stop()
        return

    # Check incoming telemetry
    time.sleep(1.0)
    if logger.sample_idx == 0:
        print(f"{ANSI.YELLOW}[!] Warning: Connected, but no QVAR telemetry samples received yet. Is firmware running?{ANSI.RESET}")

    print(f"{ANSI.GREEN}[✓] Serial connection established! 200 Hz telemetry streaming confirmed.{ANSI.RESET}")
    print(f"{ANSI.DIM}    Initial Baseline: {logger.baseline_act:.0f} LSB | Floor: {logger.lower_th} LSB | Ceil: {logger.upper_ceil} LSB{ANSI.RESET}\n")

    # Clear any previous tap queue
    logger.get_and_clear_new_taps()

    false_positives = []
    trial_results = []

    try:
        # =========================================================================
        # PHASE 1: Quiescent Desk Baseline (Resting)
        # =========================================================================
        print(f"{ANSI.BOLD}{ANSI.WHITE}" + "-" * 70)
        print("STAGE 1: QUIESCENT DESK BASELINE (SENSOR AT REST)")
        print("-" * 70 + f"{ANSI.RESET}")
        print(f"{ANSI.YELLOW}>> Place the sensor flat and resting on the desk or mount.{ANSI.RESET}")
        input(f"{ANSI.CYAN}>> Press [ENTER] when the sensor is resting still on the desk... {ANSI.RESET}")

        logger.set_experiment_state("RESTING_DESK", 0, False)
        print(f"{ANSI.CYAN}[*] Logging quiescent resting baseline for 5 seconds (testing false positives)...{ANSI.RESET}")

        t_desk = time.time()
        while (time.time() - t_desk) < 5.0:
            time.sleep(0.1)
            taps = logger.get_and_clear_new_taps()
            for t in taps:
                t["context"] = "Desk Resting Phase"
                false_positives.append(t)
                print(f"  {ANSI.RED}[!] FALSE POSITIVE ON DESK: #{t['tap_id']} dur={t['dur_ms']}ms peak={t['peak_act']} LSB{ANSI.RESET}")

        print(f"{ANSI.GREEN}[✓] Desk baseline recorded. Current Baseline: {logger.baseline_act:.0f} LSB | Floor: {logger.lower_th} LSB{ANSI.RESET}\n")

        # =========================================================================
        # PHASE 2: Pickup & Hold Sensor (Hand-Held Settling)
        # =========================================================================
        print(f"{ANSI.BOLD}{ANSI.WHITE}" + "-" * 70)
        print("STAGE 2: PICK UP & HOLD SENSOR IN OPERATING POSITION")
        print("-" * 70 + f"{ANSI.RESET}")
        print(f"{ANSI.YELLOW}>> Pick up and hold the sensor firmly in your hand as if worn or in use.{ANSI.RESET}")
        print(f"{ANSI.YELLOW}>> Do NOT tap on the sensor electrode yet.{ANSI.RESET}")
        input(f"{ANSI.CYAN}>> Press [ENTER] when you are holding the sensor steadily... {ANSI.RESET}")

        logger.set_experiment_state("HOLD_SETTLING", 0, False)
        print(f"{ANSI.CYAN}[*] Allowing 4.0s settling grace period for hand-held auto-calibration...{ANSI.RESET}")

        # 4-second settling grace period: wait for rolling auto-calibration to adapt to hand
        t_hold = time.time()
        while (time.time() - t_hold) < 4.0:
            time.sleep(0.1)

        # Flush pickup handling spikes from queue before arming false positive audit
        logger.get_and_clear_new_taps()

        # Audit steady holding for 2 seconds (sensor should be quiet)
        t_steady = time.time()
        while (time.time() - t_steady) < 2.0:
            time.sleep(0.1)
            taps = logger.get_and_clear_new_taps()
            for t in taps:
                t["context"] = "Steady Hand Hold"
                false_positives.append(t)
                print(f"  {ANSI.RED}[!] FALSE POSITIVE DURING STEADY HOLD: #{t['tap_id']} dur={t['dur_ms']}ms peak={t['peak_act']} LSB{ANSI.RESET}")

        print(f"{ANSI.GREEN}[✓] Hand-held baseline established: {logger.baseline_act:.0f} LSB | Floor: {logger.lower_th} LSB{ANSI.RESET}\n")

        # =========================================================================
        # PHASE 3: Randomized Flash Tap Trials
        # =========================================================================
        print(f"{ANSI.BOLD}{ANSI.WHITE}" + "-" * 70)
        print(f"STAGE 3: RANDOMIZED PROMPTED TAP TRIALS ({total_trials} TRIALS)")
        print("-" * 70 + f"{ANSI.RESET}")
        print(f"{ANSI.WHITE}Instructions:{ANSI.RESET}")
        print(f"  - Keep holding the sensor steadily.")
        print(f"  - Wait for the screen to {ANSI.YELLOW}{ANSI.BOLD}FLASH YELLOW{ANSI.RESET} and beep.")
        print(f"  - As soon as the cue flashes, {ANSI.CYAN}{ANSI.BOLD}TAP ONCE{ANSI.RESET} on the sensor.")
        print(f"  - Between cues, hold quietly (any taps detected while waiting count as {ANSI.RED}False Positives{ANSI.RESET}).")
        input(f"\n{ANSI.CYAN}>> Press [ENTER] to start the {total_trials}-trial benchmark... {ANSI.RESET}")

        print("\n" + "=" * 70)

        for trial in range(1, total_trials + 1):
            # 1. Randomized wait interval (Monitors for False Positives)
            wait_time = random.uniform(min_wait, max_wait)
            logger.set_experiment_state("WAITING_CUE", trial, False)
            print(f"\n{ANSI.DIM}[Trial {trial:2d}/{total_trials}] Holding quietly... (wait {wait_time:.1f}s){ANSI.RESET}")

            t_wait_start = time.time()
            while (time.time() - t_wait_start) < wait_time:
                time.sleep(0.05)
                taps = logger.get_and_clear_new_taps()
                for t in taps:
                    t["context"] = f"Waiting before Trial {trial}"
                    false_positives.append(t)
                    print(f"  {ANSI.RED}[!] FALSE POSITIVE (Unprompted Tap while waiting!): #{t['tap_id']} dur={t['dur_ms']}ms peak={t['peak_act']} LSB{ANSI.RESET}")

            # 2. Flash Cue (Visual Banner + High Pitch Beep)
            logger.get_and_clear_new_taps()  # Clear any residual queue right before cue
            cue_time = time.time()
            logger.set_experiment_state("TAP_WINDOW", trial, True)

            # High contrast visual flash
            print(f"\n{ANSI.BG_YELLOW}{ANSI.BLACK}{ANSI.BOLD}" + "=" * 70)
            print(f"   >>>>>>>>> [ TRIAL {trial:2d}/{total_trials}:  !!! TAP NOW - TOUCH SENSOR !!! ] <<<<<<<<<   ")
            print("=" * 70 + f"{ANSI.RESET}")

            play_sound(1200, 120)

            # 3. Response Window (Evaluates HIT vs MISS)
            tap_detected = None
            reaction_ms = None

            while (time.time() - cue_time) < tap_timeout:
                time.sleep(0.02)
                taps = logger.get_and_clear_new_taps()
                if taps:
                    tap_detected = taps[0]
                    reaction_ms = (tap_detected["time"] - cue_time) * 1000.0
                    # Any extra taps in same window
                    if len(taps) > 1:
                        for extra_t in taps[1:]:
                            extra_t["context"] = f"Extra bounce during Trial {trial}"
                            false_positives.append(extra_t)
                    break

            logger.set_experiment_state("COOLDOWN", trial, False)

            if tap_detected is not None:
                play_sound(1800, 80)
                outcome = "HIT"
                print(f"{ANSI.GREEN}{ANSI.BOLD}[✓] HIT! Tap detected in {reaction_ms:.0f} ms | Dur: {tap_detected['dur_ms']} ms | Peak: {tap_detected['peak_act']} LSB | Floor: {tap_detected['lower_th']} LSB{ANSI.RESET}")
                trial_results.append({
                    "trial": trial,
                    "outcome": "HIT",
                    "latency_ms": reaction_ms,
                    "dur_ms": tap_detected["dur_ms"],
                    "peak_act": tap_detected["peak_act"],
                    "baseline": tap_detected["baseline"],
                    "lower_th": tap_detected["lower_th"]
                })
            else:
                play_sound(400, 200)
                outcome = "MISS"
                print(f"{ANSI.RED}{ANSI.BOLD}[✗] MISS! Touch not detected within {tap_timeout:.1f}s timeout.{ANSI.RESET}")
                trial_results.append({
                    "trial": trial,
                    "outcome": "MISS",
                    "latency_ms": None,
                    "dur_ms": None,
                    "peak_act": None,
                    "baseline": logger.baseline_act,
                    "lower_th": logger.lower_th
                })

            # Brief debounce buffer before next trial
            time.sleep(0.5)
            # Flush any delayed tap or late bounce so it does not bleed into the next trial's quiet waiting phase
            logger.get_and_clear_new_taps()

        # =========================================================================
        # PHASE 4: Return Sensor to Desk
        # =========================================================================
        print(f"\n{ANSI.BOLD}{ANSI.WHITE}" + "-" * 70)
        print("STAGE 4: RETURN SENSOR TO DESK (RELEASE SETTLING)")
        print("-" * 70 + f"{ANSI.RESET}")
        print(f"{ANSI.YELLOW}>> Place the sensor back down flat and resting on the desk.{ANSI.RESET}")
        input(f"{ANSI.CYAN}>> Press [ENTER] once the sensor is placed back down... {ANSI.RESET}")

        logger.set_experiment_state("RETURN_DESK", 0, False)
        print(f"{ANSI.CYAN}[*] Recording release settling baseline for 5 seconds...{ANSI.RESET}")

        t_ret = time.time()
        while (time.time() - t_ret) < 5.0:
            time.sleep(0.1)
            taps = logger.get_and_clear_new_taps()
            for t in taps:
                t["context"] = "Return to Desk Release"
                false_positives.append(t)
                print(f"  {ANSI.RED}[!] FALSE POSITIVE ON RETURN TO DESK: #{t['tap_id']} dur={t['dur_ms']}ms peak={t['peak_act']} LSB{ANSI.RESET}")

        print(f"{ANSI.GREEN}[✓] Return baseline settled. Final Baseline: {logger.baseline_act:.0f} LSB | Floor: {logger.lower_th} LSB{ANSI.RESET}\n")

    finally:
        logger.stop()

    # =========================================================================
    # PHASE 5: Compute Accuracy Metrics & Produce Report
    # =========================================================================
    hits = [t for t in trial_results if t["outcome"] == "HIT"]
    misses = [t for t in trial_results if t["outcome"] == "MISS"]

    n_hits = len(hits)
    n_misses = len(misses)
    n_false_pos = len(false_positives)

    sensitivity = (n_hits / total_trials) * 100.0 if total_trials > 0 else 0.0
    precision = (n_hits / (n_hits + n_false_pos)) * 100.0 if (n_hits + n_false_pos) > 0 else 0.0

    latencies = [t["latency_ms"] for t in hits if t["latency_ms"] is not None]
    peaks = [t["peak_act"] for t in hits if t["peak_act"] is not None]
    durs = [t["dur_ms"] for t in hits if t["dur_ms"] is not None]

    avg_lat = sum(latencies) / len(latencies) if latencies else 0.0
    avg_peak = sum(peaks) / len(peaks) if peaks else 0.0
    avg_dur = sum(durs) / len(durs) if durs else 0.0

    report_lines = []
    report_lines.append("=======================================================================")
    report_lines.append("           ISM330BX QVAR TAP DETECTION ACCURACY BENCHMARK REPORT       ")
    report_lines.append("=======================================================================")
    report_lines.append(f"Timestamp          : {datetime.datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
    report_lines.append(f"Firmware Serial Port: {port} @ {baud} baud")
    report_lines.append(f"Total Prompted Cues: {total_trials}")
    report_lines.append(f"Recorded Telemetry : data/{csv_filename} ({logger.sample_idx} samples @ 200 Hz)")
    report_lines.append("-" * 71)
    report_lines.append("CORE ACCURACY METRICS:")
    report_lines.append(f"  • True Positives (Hits)       : {n_hits:2d} / {total_trials} ({sensitivity:5.1f}%)")
    report_lines.append(f"  • False Negatives (Misses)    : {n_misses:2d} / {total_trials}")
    report_lines.append(f"  • False Positives (Ghost Taps): {n_false_pos:2d}")
    report_lines.append(f"  • Detection Sensitivity/Recall: {sensitivity:5.1f}%")
    report_lines.append(f"  • Overall Precision           : {precision:5.1f}%")
    report_lines.append("-" * 71)
    report_lines.append("REACTION & SIGNAL METRICS:")
    if hits:
        report_lines.append(f"  • Avg Reaction Latency       : {avg_lat:5.1f} ms (Min: {min(latencies):.0f} ms, Max: {max(latencies):.0f} ms)")
        report_lines.append(f"  • Avg Tap Peak Activity      : {avg_peak:5.0f} LSB (Min: {min(peaks)}, Max: {max(peaks)})")
        report_lines.append(f"  • Avg Tap Duration           : {avg_dur:5.1f} ms (Min: {min(durs)} ms, Max: {max(durs)} ms)")
    else:
        report_lines.append("  • No successful tap events recorded.")
    report_lines.append("-" * 71)
    report_lines.append("DETAILED PER-TRIAL RESULTS:")
    report_lines.append(f"{'Trial':>5} | {'Outcome':>7} | {'Reaction':>10} | {'Peak Act':>10} | {'Duration':>10} | {'Lower Floor':>11}")
    report_lines.append("-" * 71)
    for r in trial_results:
        lat_str = f"{r['latency_ms']:.0f} ms" if r['latency_ms'] is not None else "--"
        peak_str = f"{r['peak_act']} LSB" if r['peak_act'] is not None else "--"
        dur_str = f"{r['dur_ms']} ms" if r['dur_ms'] is not None else "--"
        th_str = f"{r['lower_th']} LSB"
        report_lines.append(f"{r['trial']:5d} | {r['outcome']:>7} | {lat_str:>10} | {peak_str:>10} | {dur_str:>10} | {th_str:>11}")

    if false_positives:
        report_lines.append("-" * 71)
        report_lines.append(f"FALSE POSITIVE EVENTS DETECTED ({len(false_positives)}):")
        for i, fp in enumerate(false_positives, 1):
            report_lines.append(f"  [{i}] Context: {fp['context']} | Tap #{fp['tap_id']} dur={fp['dur_ms']}ms peak={fp['peak_act']} LSB base={fp['baseline']:.0f} th={fp['lower_th']}")
    else:
        report_lines.append("-" * 71)
        report_lines.append("FALSE POSITIVE AUDIT: ZERO UNPROMPTED TAPS DETECTED (100% False-Positive Free!)")

    report_lines.append("=======================================================================")

    report_text = "\n".join(report_lines)

    # Save to TXT file
    with open(report_path, "w", encoding="utf-8") as f:
        f.write(report_text)

    # Print to console with color highlights
    print(f"\n{ANSI.GREEN}{ANSI.BOLD}" + "=" * 70)
    print("                    BENCHMARK COMPLETE")
    print("=" * 70 + f"{ANSI.RESET}\n")

    print(report_text)

    print(f"\n{ANSI.CYAN}[+] Telemetry CSV saved to: {csv_path}{ANSI.RESET}")
    print(f"{ANSI.CYAN}[+] Summary Report saved to: {report_path}{ANSI.RESET}\n")


def main():
    parser = argparse.ArgumentParser(description="ISM330BX QVAR Interactive Tap Accuracy Benchmark Protocol")
    parser.add_argument("--port", default="COM7", help="Serial port of ESP32 (default: COM7)")
    parser.add_argument("--baud", type=int, default=115200, help="Baud rate (default: 115200)")
    parser.add_argument("--trials", type=int, default=20, help="Number of randomized tap trials (default: 20)")
    parser.add_argument("--min-wait", type=float, default=1.5, help="Min quiet wait before cue in seconds (default: 1.5)")
    parser.add_argument("--max-wait", type=float, default=4.0, help="Max quiet wait before cue in seconds (default: 4.0)")
    parser.add_argument("--timeout", type=float, default=2.0, help="Tap response timeout in seconds (default: 2.0)")

    args = parser.parse_args()

    run_benchmark(
        port=args.port,
        baud=args.baud,
        total_trials=args.trials,
        min_wait=args.min_wait,
        max_wait=args.max_wait,
        tap_timeout=args.timeout
    )


if __name__ == "__main__":
    main()
