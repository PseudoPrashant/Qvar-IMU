# ISM330BX ESP-IDF / FreeRTOS Project

This directory contains the ESP32 / FreeRTOS port of the ISM330BX IMU driver and QVAR electrode polling engine.

## Directory Structure

- `CMakeLists.txt`: ESP-IDF root project definition.
- `plot_qvar.py`: Python real-time GUI visualizer and logger for COM serial telemetry.
- `main/`: ESP-IDF component containing:
  - `main.c`: Application entry point (`app_main`), initializes I2C master (`GPIO 21/22`) and runs the QVAR polling loop.
  - `imu.c`: Low-level IMU driver over I2C (`i2c_master_write_to_device` / `i2c_master_write_read_device`).
  - `imu.h`: IMU driver header declarations and types.
  - `imu_internal.h`: Internal hooks between IMU core and feature modules.
  - `ism330bx_reg.c` / `ism330bx_reg.h`: Official ST ISM330BX register driver.
  - `qvar.c`: QVAR electrode state machine and polling engine ported to FreeRTOS (`xTaskGetTickCount`, `vTaskDelay`).
  - `qvar.h`: QVAR API declarations, data structures, and configuration macros.
  - `qvar_config.h`: QVAR application profiles. Active configuration is set to `QVAR_APP_CONFIG_BUTTON_Q1_ONLY`.

## QVAR Polling Configuration

The polling logic in `qvar.c` replaces STM32 HAL timing functions with FreeRTOS equivalents:
- `HAL_GetTick()` &rarr; `(xTaskGetTickCount() * portTICK_PERIOD_MS)`
- `HAL_Delay(...)` &rarr; `vTaskDelay(pdMS_TO_TICKS(...))`
- Active Profile in `qvar_config.h`:
  ```c
  #define QVAR_APP_ACTIVE_CONFIG QVAR_APP_CONFIG_BUTTON_Q1_ONLY
  ```
- **Tuned Input Impedance ($Z_{in}$)**:
  - Configured to `IMU_QVAR_ZIN_235_MOHM` ($235\text{ M}\Omega$) in `QVAR_CONFIG_BUTTON_Q1_ONLY` ([qvar.h](file:///c:/Users/prash/OneDrive/Desktop/IMU/IMU/project/main/qvar.h)) per ST AN5755 Section 5.3.1 (Page 19, line 54: `CTRL7 = 0xB0`), attenuating radiated airborne electric field noise by an additional $\approx 22\%$ over $300\text{ M}\Omega$.
- **Hardware High-Pass Filter (HPF)**:
  - Enabled `.hpfEnable = 1u` in `QVAR_CONFIG_BUTTON_Q1_ONLY` ([qvar.h](file:///c:/Users/prash/OneDrive/Desktop/IMU/IMU/project/main/qvar.h)) per ST AN5755 Section 5.1.4 to eliminate floating-electrode DC static charge accumulation and recenter the baseline around $0\text{ LSB}$.
- **10-Sample Moving Average FIR Comb Filter (50 Hz Notch)**:
  - Circular 10-sample rolling average ($100\text{ ms}$ window at 100 Hz) in `qvar.c` per ST AN5755 Section 5.1.6. Places an exact mathematical null notch at $50\text{ Hz}$ mains hum while slashing group delay by $50\%$ down to just $45\text{ ms}$.
- **Deterministic 10 ms (100 Hz) Polling Loop**:
  - Implemented non-drifting periodic task pacing via `vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(10))` in [main.c](file:///c:/Users/prash/OneDrive/Desktop/IMU/IMU/project/main/main.c), doubling data throughput and temporal resolution over 50 Hz.
- **Unified Single-Tap & Double-Tap Event Detection**:
  - Implemented multi-tap state machine in `qvar.c` and `qvar_config.h` at 100 Hz:
    - `qvar1ButtonDoubleTapWindowMs = 350 ms`: Evaluates whether a second tap arrives within 350 ms (real user double-tap intervals measured at $184\text{ to }222\text{ ms}$).
    - **In-Gesture Trough Detection**: Recognizes the $+13,000\text{ to }+18,800\text{ LSB}$ upward inflection between double taps (`rise >= 8000 LSB` or `delta <= peak * 0.55`) to cleanly segment sub-taps without waiting for a full return to $0\text{ LSB}$.
    - `buttonEventCooldownMs = 50 ms`: Snappy re-arming between consecutive taps.
    - Timing parameters: `touchConfirmSamples = 3` ($30\text{ ms}$), `releaseConfirmSamples = 2` ($20\text{ ms}$), `max_press = 400 ms`, `hold_time = 800 ms`.
    - **Dual Events Emitted**:
      - `[IMU QVAR] Q1 BUTTON SINGLE (peak=%ld LSB / %.1f mV, dur=%lu ms)` (when no 2nd tap follows within 350 ms).
      - `[IMU QVAR] Q1 BUTTON DOUBLE (interval=%lu ms, peak=%ld LSB / %.1f mV)` (when 2 taps complete within 350 ms).
      - `[IMU QVAR] Q1 BUTTON HOLD` and `HOLD RELEASE` (for sustained touches $\ge 800\text{ ms}$).
    - **Empirically Validated**: Verified with $10/10$ ($100\%$) Single Taps on `data/latest_single-taps-100hz.csv`, $10/10$ ($100\%$) Double Taps on `data/latest_double-taps-100Hz.csv`, and $0$ false triggers on `data/noise after all changes.csv`.
- **Periodic Baseline Heartbeat**:
  - Firmware broadcasts `[IMU QVAR] Q1 button baseline=...` every 5 seconds so serial monitors connecting after boot immediately acquire active baseline and threshold guides.
- **Physical Voltage Readout**:
  - Scaled via ST AN5755 Section 4.3 constant ($1\text{ mV} = 78\text{ LSB}$) displayed in real time on the live plotter HUD.

## Build & Flash (ESP-IDF)
In PowerShell, activate the ESP-IDF environment before running `idf.py`:
```powershell
# 1. Load ESP-IDF environment
. C:\esp\v6.1\esp-idf\export.ps1

# 2. Build firmware
idf.py build

# 3. Flash to COM7
idf.py -p COM7 flash
```

## Real-Time Telemetry & Plotting (`plot_qvar.py`)

A real-time Python graphical monitor is provided to visualize the QVAR electrode data streaming over serial:

### Features
- **Rolling Waveform**: Real-time plot of Q1 (Button) and Q2 (Wear) electrode channels.
- **Monotonic Y-Axis Auto-Expansion (Zoom-Out Only)**: The Y-axis scale smoothly expands to accommodate large tap excursions without jumping or contracting when the tap scrolls out of view. Press `'r'` at any time on the plot window to reset the scale.
- **Dynamic Baseline & Thresholds**: Automatically extracts and draws horizontal guides for baseline learning, press thresholds, and release thresholds.
- **Event Detection HUD**: Real-time visual markers for button taps (`SINGLE`), hold events (`HOLD`), and wear sensing.
- **Automatic CSV Archiving**: Automatically saves every plotted session into timestamped CSV files inside `project/data/` (`qvar_telemetry_YYYYMMDD_HHMMSS.csv`).
- **Offline Mock Simulation**: Built-in mock generator (`--mock`) for testing without hardware connected.

### Requirements
Ensure dependencies are installed:
```powershell
pip install pyserial matplotlib numpy
```

### Running the Plotter
> **Important**: On Windows, serial COM ports cannot be shared across multiple programs. If `idf.py monitor` is running, exit it first (`Ctrl + ]`) before running the plotter.

1. **Default Run (Connects to COM7, auto-archives to `project/data/`)**:
   ```powershell
   python plot_qvar.py
   ```
2. **Offline Simulation Mode (no hardware needed)**:
   ```powershell
   python plot_qvar.py --mock
   ```
3. **Custom Output CSV or Disable Recording**:
   ```powershell
   # Custom file
   python plot_qvar.py --record custom_test.csv

   # Run visualizer without writing CSV files
   python plot_qvar.py --no-record
   ```

### CSV Telemetry Schema
Stored in `project/data/` with the following columns:
| Column | Description |
| :--- | :--- |
| `iso_time` | ISO-8601 formatted timestamp (`YYYY-MM-DDTHH:MM:SS.mmm`) |
| `epoch_seconds` | High precision epoch seconds |
| `sample_index` | Continuous sample index |
| `q1_raw` | Raw ADC value of Q1 electrode |
| `q1_valid` | 1 if valid, 0 if disabled/NA |
| `q2_raw` | Raw ADC value of Q2 electrode |
| `q2_valid` | 1 if valid, 0 if disabled/NA |
| `q1_baseline` | Active learned baseline for Q1 |
| `q2_baseline` | Active learned baseline for Q2 |
| `event` | Triggered event label (`Q1 BUTTON SINGLE`, `HOLD`, etc.) |


