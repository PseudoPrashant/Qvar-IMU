#!/usr/bin/env python3
"""
Dual CSV Waveform & Metrics Comparison Visualizer
=================================================
Compares two QVAR telemetry CSV files side-by-side and overlaid with interactive
dark-mode waveform plots, amplitude distribution histograms, and automated SNR metrics.

Usage:
  python compare_plot.py file1.csv file2.csv
  python compare_plot.py data/20-taps-235-wearing.csv data/20-taps-730-wearing.csv
  python compare_plot.py data/noise-235-wearing.csv data/noise-730-wearing.csv
"""

import argparse
import csv
import os
import sys
import numpy as np

try:
    import matplotlib.pyplot as plt
    from matplotlib.gridspec import GridSpec
except ImportError:
    print("[!] Error: 'matplotlib' is required. Run: pip install matplotlib")
    sys.exit(1)


def load_csv(path):
    """Load QVAR CSV file and extract samples, indices, and time."""
    if not os.path.exists(path):
        print(f"[!] Error: File not found: {path}")
        sys.exit(1)

    indices = []
    values = []
    with open(path, "r", encoding="utf-8") as f:
        reader = csv.DictReader(f)
        for row in reader:
            try:
                idx = int(row.get("sample_index", len(indices) + 1))
                val = float(row["q1_raw"])
                indices.append(idx)
                values.append(val)
            except (ValueError, KeyError):
                continue

    if not values:
        print(f"[!] Error: No valid 'q1_raw' samples found in {path}")
        sys.exit(1)

    return np.array(indices), np.array(values)


def compute_metrics(arr):
    """Compute key signal statistics (LSB and mV)."""
    mean_val = np.mean(arr)
    median_val = np.median(arr)
    std_val = np.std(arr)
    min_val = np.min(arr)
    max_val = np.max(arr)
    p2p_val = max_val - min_val
    clipped_neg = int(np.sum(arr <= -32700))
    clipped_pos = int(np.sum(arr >= 32700))
    p99 = np.percentile(np.abs(arr - median_val), 99)

    return {
        "count": len(arr),
        "duration_s": len(arr) * 0.01,
        "mean_lsb": mean_val,
        "mean_mv": mean_val / 78.0,
        "median_lsb": median_val,
        "median_mv": median_val / 78.0,
        "std_lsb": std_val,
        "std_mv": std_val / 78.0,
        "min_lsb": min_val,
        "min_mv": min_val / 78.0,
        "max_lsb": max_val,
        "max_mv": max_val / 78.0,
        "p2p_lsb": p2p_val,
        "p2p_mv": p2p_val / 78.0,
        "clipped": clipped_neg + clipped_pos,
        "p99_dev_lsb": p99,
        "p99_dev_mv": p99 / 78.0,
    }


def compare_plots(file1, file2, title=None):
    """Render comprehensive comparison dashboard."""
    idx1, val1 = load_csv(file1)
    idx2, val2 = load_csv(file2)

    m1 = compute_metrics(val1)
    m2 = compute_metrics(val2)

    label1 = os.path.basename(file1)
    label2 = os.path.basename(file2)

    # Time in seconds (assuming 100 Hz = 10 ms / sample)
    t1 = np.arange(len(val1)) * 0.01
    t2 = np.arange(len(val2)) * 0.01

    plt.style.use("dark_background")
    fig = plt.figure(figsize=(15, 8.5))
    fig.patch.set_facecolor("#0b0f19")
    fig.canvas.manager.set_window_title(f"QVAR Comparison: {label1} vs {label2}")

    gs = GridSpec(2, 2, height_ratios=[1.3, 1.0], width_ratios=[1.2, 0.8], figure=fig)
    ax_wave = fig.add_subplot(gs[0, :])
    ax_hist = fig.add_subplot(gs[1, 0])
    ax_table = fig.add_subplot(gs[1, 1])

    c1 = "#00e5ff"  # Vibrant Cyan
    c2 = "#fb7185"  # Vibrant Coral/Rose

    # -------------------------------------------------------------
    # 1. Top Panel: Overlaid Waveform Plot
    # -------------------------------------------------------------
    ax_wave.set_facecolor("#111827")
    ax_wave.axhline(0, color="#64748b", linestyle="-", alpha=0.4, linewidth=0.8)
    line1, = ax_wave.plot(t1, val1, color=c1, alpha=0.85, linewidth=1.4, label=f"File 1: {label1}")
    line2, = ax_wave.plot(t2, val2, color=c2, alpha=0.80, linewidth=1.4, label=f"File 2: {label2}")

    # Mark saturation lines
    ax_wave.axhline(-32704, color="#ef4444", linestyle="--", alpha=0.5, linewidth=1.0, label="ADC Saturation (-32,704)")

    ax_wave.set_title(title or f"QVAR Telemetry Comparison: {label1} (Cyan) vs {label2} (Rose)",
                      color="#f8fafc", fontsize=13, weight="bold", pad=10)
    ax_wave.set_xlabel("Time (seconds @ 100 Hz)", color="#94a3b8", fontsize=11, labelpad=6)
    ax_wave.set_ylabel("Raw Amplitude (LSB)", color="#94a3b8", fontsize=11, labelpad=6)
    ax_wave.grid(True, linestyle="--", alpha=0.25, color="#475569")
    ax_wave.tick_params(colors="#64748b", labelsize=10)
    for s in ax_wave.spines.values():
        s.set_color("#334155")
    ax_wave.legend(loc="upper right", facecolor="#1e293b", edgecolor="#475569", fontsize=9)

    # Right Y-axis for Millivolts (78 LSB / mV)
    ax_wave_mv = ax_wave.secondary_yaxis("right", functions=(lambda x: x / 78.0, lambda x: x * 78.0))
    ax_wave_mv.set_ylabel("Equivalent Voltage (mV)", color="#94a3b8", fontsize=10, labelpad=8)
    ax_wave_mv.tick_params(colors="#64748b", labelsize=9)

    # -------------------------------------------------------------
    # 2. Bottom Left Panel: Amplitude Distribution Histogram
    # -------------------------------------------------------------
    ax_hist.set_facecolor("#111827")
    bins = np.linspace(min(m1["min_lsb"], m2["min_lsb"]), max(m1["max_lsb"], m2["max_lsb"]), 120)
    ax_hist.hist(val1, bins=bins, color=c1, alpha=0.55, density=True, label=label1)
    ax_hist.hist(val2, bins=bins, color=c2, alpha=0.55, density=True, label=label2)
    ax_hist.axvline(0, color="#64748b", linestyle="-", alpha=0.4, linewidth=0.8)

    ax_hist.set_title("Amplitude Probability Distribution", color="#f8fafc", fontsize=11, weight="bold", pad=8)
    ax_hist.set_xlabel("Amplitude (LSB)", color="#94a3b8", fontsize=10)
    ax_hist.set_ylabel("Probability Density", color="#94a3b8", fontsize=10)
    ax_hist.grid(True, linestyle="--", alpha=0.25, color="#475569")
    ax_hist.tick_params(colors="#64748b", labelsize=9)
    for s in ax_hist.spines.values():
        s.set_color("#334155")
    ax_hist.legend(loc="upper right", facecolor="#1e293b", edgecolor="#475569", fontsize=8)

    # -------------------------------------------------------------
    # 3. Bottom Right Panel: Formatted Comparison Table
    # -------------------------------------------------------------
    ax_table.set_facecolor("#111827")
    ax_table.axis("off")

    table_data = [
        ["Metric", label1[:18], label2[:18]],
        ["Samples", f"{m1['count']:,}", f"{m2['count']:,}"],
        ["Duration", f"{m1['duration_s']:.1f} s", f"{m2['duration_s']:.1f} s"],
        ["DC Offset", f"{m1['mean_mv']:+.2f} mV", f"{m2['mean_mv']:+.2f} mV"],
        ["Noise RMS", f"{m1['std_mv']:.2f} mV", f"{m2['std_mv']:.2f} mV"],
        ["Peak-to-Peak", f"{m1['p2p_mv']:.1f} mV", f"{m2['p2p_mv']:.1f} mV"],
        ["Min Peak", f"{m1['min_mv']:+.1f} mV", f"{m2['min_mv']:+.1f} mV"],
        ["Max Peak", f"{m1['max_mv']:+.1f} mV", f"{m2['max_mv']:+.1f} mV"],
        ["99% Dev", f"{m1['p99_dev_mv']:.1f} mV", f"{m2['p99_dev_mv']:.1f} mV"],
        ["Clipped Samples", f"{m1['clipped']}", f"{m2['clipped']}"],
    ]

    col_widths = [0.38, 0.31, 0.31]
    tab = ax_table.table(
        cellText=table_data,
        loc="center",
        cellLoc="center",
        colWidths=col_widths,
    )
    tab.auto_set_font_size(False)
    tab.set_fontsize(9.5)
    tab.scale(1.0, 1.35)

    # Table styling
    for (r, c), cell in tab.get_celld().items():
        cell.set_edgecolor("#334155")
        if r == 0:
            cell.set_facecolor("#1e293b")
            cell.set_text_props(weight="bold", color="#f8fafc")
        elif c == 1:
            cell.set_facecolor("#111827")
            cell.set_text_props(color=c1)
        elif c == 2:
            cell.set_facecolor("#111827")
            cell.set_text_props(color=c2)
        else:
            cell.set_facecolor("#111827")
            cell.set_text_props(color="#94a3b8")

    plt.tight_layout()
    print(f"[*] Displaying comparison plot for:\n    1) {file1}\n    2) {file2}")
    plt.show()


def main():
    parser = argparse.ArgumentParser(
        description="Dual CSV Waveform & Metrics Comparison Visualizer"
    )
    parser.add_argument(
        "file1",
        nargs="?",
        default=r"C:\Users\prash\OneDrive\Desktop\IMU\IMU\firmware_raw\data\20-taps-235-wearing.csv",
        help="Path to first CSV file",
    )
    parser.add_argument(
        "file2",
        nargs="?",
        default=r"C:\Users\prash\OneDrive\Desktop\IMU\IMU\firmware_raw\data\20-taps-730-wearing.csv",
        help="Path to second CSV file",
    )
    parser.add_argument(
        "--test",
        action="store_true",
        help="Validation test mode (does not block on GUI window)",
    )

    args = parser.parse_args()

    if args.test:
        print("[*] Running test load on input files...")
        i1, v1 = load_csv(args.file1)
        i2, v2 = load_csv(args.file2)
        assert len(v1) > 0 and len(v2) > 0, "Empty dataset loaded"
        print(f"[+] Test passed: loaded {len(v1)} and {len(v2)} samples.")
        return

    compare_plots(args.file1, args.file2)


if __name__ == "__main__":
    main()
