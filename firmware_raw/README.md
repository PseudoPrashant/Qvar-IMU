# ISM330BX Pure Raw Telemetry Firmware & Visualizer

A standalone, minimal ESP-IDF firmware and Python real-time visualizer dedicated purely to streaming and recording completely unprocessed 16-bit ADC samples directly from the ISM330BX QVAR register.

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

---

## Real-Time Visualizer & CSV Data Logger (`plotter.py`):
Located directly in this folder, `plotter.py` connects to COM7, renders a dark-mode real-time oscilloscope, and **automatically records every sample into CSV format** inside `firmware_raw\data\`:

```powershell
# Live plot + automatic CSV recording in data/
python plotter.py

# Specify serial port or custom CSV filename
python plotter.py --port COM7 --csv-name my_session.csv

# View any previously recorded CSV file
python plotter.py data\raw_q1_YYYYMMDD_HHMMSS.csv
```

### CSV Schema:
Files stored in `firmware_raw\data\raw_q1_<timestamp>.csv`:
`iso_time,epoch_seconds,sample_index,q1_raw,q1_voltage_mv`

---

## Build & Flash Firmware:
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
