#!/usr/bin/env python3
"""
QVAR Spacebar Keyboard Controller
=================================
Listens to real-time QVAR electrode telemetry from the ESP32 over serial (COM7),
and injects a native Windows Spacebar keypress on every detected peak.

Zero external dependencies required (uses built-in ctypes and standard pyserial).

Usage:
    python qvar_keyboard.py
    python qvar_keyboard.py --port COM7
    python qvar_keyboard.py --key space
    python qvar_keyboard.py --key enter
"""

import argparse
import ctypes
import os
import re
import sys
import time

try:
    import serial
    import serial.tools.list_ports
except ImportError:
    print("[!] Error: 'pyserial' is not installed. Run: pip install pyserial")
    sys.exit(1)

# Windows Virtual-Key Codes
VK_MAP = {
    "space": 0x20,
    "enter": 0x0D,
    "return": 0x0D,
    "up": 0x26,
    "down": 0x28,
    "left": 0x25,
    "right": 0x27,
    "tab": 0x09,
    "escape": 0x1B,
    "esc": 0x1B,
}

KEYEVENTF_KEYUP = 0x0002

# Telemetry regex patterns
RE_PEAK = re.compile(r"\[IMU QVAR\]\s+Q1 PEAK(?:\s+\(depth=(-?\d+)\s+LSB\s*/\s*([0-9.]+)\s*mV\))?")
RE_EVENT = re.compile(r"\[IMU QVAR\]\s+(Q1 PEAK|Q1 BUTTON TAP|Q1 BUTTON SINGLE|Q1 BUTTON DOUBLE|Q1 BUTTON TRIPLE)")
RE_BASELINE = re.compile(r"\[IMU QVAR\]\s+Q1 (?:button )?baseline=(-?\d+)")


def press_key(vk_code):
    """Press and release a virtual key using Windows user32.dll with native speed."""
    ctypes.windll.user32.keybd_event(vk_code, 0, 0, 0)
    time.sleep(0.02)  # 20 ms hold so games/applications register the keypress reliably
    ctypes.windll.user32.keybd_event(vk_code, 0, KEYEVENTF_KEYUP, 0)


def auto_detect_com_port():
    """Attempt to auto-detect an ESP32 serial port if available."""
    ports = list(serial.tools.list_ports.comports())
    for p in ports:
        desc = (p.description or "").lower()
        if "cp210" in desc or "ch340" in desc or "uart" in desc or "usb-to-uart" in desc:
            return p.device
    return "COM7"


def main():
    parser = argparse.ArgumentParser(
        description="Translate QVAR electrode peaks into real-time keyboard presses."
    )
    parser.add_argument(
        "--port",
        default="COM7",
        help="Serial port of the ESP32 (default: COM7)",
    )
    parser.add_argument(
        "--baud",
        type=int,
        default=115200,
        help="Baud rate (default: 115200)",
    )
    parser.add_argument(
        "--key",
        default="space",
        choices=list(VK_MAP.keys()),
        help="Key to trigger on each peak (default: space)",
    )
    parser.add_argument(
        "--verbose",
        action="store_true",
        help="Print raw serial stream",
    )
    args = parser.parse_args()

    vk_code = VK_MAP[args.key.lower()]
    key_name = args.key.upper()

    print("=" * 65)
    print("       QVAR REAL-TIME KEYBOARD CONTROLLER")
    print("=" * 65)
    print(f"  Target Port : {args.port}")
    print(f"  Baud Rate   : {args.baud}")
    print(f"  Mapped Key  : [{key_name}] (Virtual Key Code: 0x{vk_code:02X})")
    print("=" * 65)
    print("  TIP: You can test this right now with:")
    print("       - Chrome Dino Game : Open chrome://dino and start tapping!")
    print("       - YouTube / Media  : Tap to Pause/Play")
    print("       - Text Document    : Tap to insert spaces")
    print("=" * 65)

    try:
        ser = serial.Serial(args.port, args.baud, timeout=0.1)
    except serial.SerialException as e:
        print(f"\n[!] Failed to open serial port {args.port}: {e}")
        print("[!] Tip: If 'plot_qvar.py' is currently open, close it first so the port is free.")
        sys.exit(1)

    print(f"[+] Connected to {args.port}. Listening for electrode peaks...\n")

    tap_count = 0
    current_baseline = "N/A"

    try:
        while True:
            line_bytes = ser.readline()
            if not line_bytes:
                continue

            line = line_bytes.decode("utf-8", errors="replace").strip()
            if not line:
                continue

            if args.verbose:
                print(f"[RAW] {line}")

            # Track baseline updates
            m_base = RE_BASELINE.search(line)
            if m_base:
                current_baseline = m_base.group(1)

            # Match peak or event
            m_peak = RE_PEAK.search(line)
            m_event = RE_EVENT.search(line)

            if m_peak or m_event:
                tap_count += 1
                depth_str = ""
                if m_peak and m_peak.group(1):
                    depth_lsb = m_peak.group(1)
                    depth_mv = m_peak.group(2)
                    depth_str = f" | Depth: {depth_lsb} LSB ({depth_mv} mV)"

                # Immediately trigger native Windows keystroke
                press_key(vk_code)

                timestamp = time.strftime("%H:%M:%S")
                print(f"[{timestamp}] [PEAK #{tap_count:03d}] -> KEYBOARD: [{key_name}] PRESSED!{depth_str} (Base: {current_baseline})")

    except KeyboardInterrupt:
        print("\n[*] Stopping QVAR Keyboard Controller...")
    finally:
        ser.close()
        print("[+] Serial port closed. Goodbye!")


if __name__ == "__main__":
    main()
