# ISM330BX Pure Raw Telemetry Firmware

A standalone, minimal ESP-IDF firmware dedicated purely to streaming completely unprocessed 16-bit ADC samples directly from the ISM330BX QVAR register.

## Key Characteristics:
- **Zero Filtering (Neither Hardware nor Software)**:
  - Hardware HPF: DISABLED (`filter.hpf = 0`)
  - Hardware LPF: DISABLED (`filter.lpf = 0`)
  - NO FIR notch / moving average filter.
  - NO dynamic baseline tracking.
  - NO peak detection / thresholding.
  - NO button / hold / wear state machines.
- **Pure 100 Hz Streaming**:
  - Sampled every 10 ms (100 Hz) via FreeRTOS `vTaskDelayUntil`.
  - Directly reads registers `0x3A` / `0x3B` (`ISM330BX_AH_QVAR_OUT_L` / `H`).
- **Telemetry Format**:
  - `[IMU QVAR RAW] Q1=<raw_lsb> Q2=NA (<voltage> mV) #<sample_count>`
  - Fully compatible with `plot_raw_q1.py`.

## Build & Flash:
```powershell
# 1. Activate ESP-IDF
. C:\esp\v6.1\esp-idf\export.ps1

# 2. Navigate to firmware_raw directory
cd C:\Users\prash\OneDrive\Desktop\IMU\IMU\firmware_raw

# 3. Build
idf.py build

# 4. Flash to COM7
idf.py -p COM7 flash
```
