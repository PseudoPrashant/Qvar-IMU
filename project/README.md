# ISM330BX ESP-IDF / FreeRTOS Project

This directory contains the ESP32 / FreeRTOS port of the ISM330BX IMU driver and QVAR electrode polling engine.

## Directory Structure

- `CMakeLists.txt`: ESP-IDF root project definition.
- `plot_qvar.py`: Python real-time GUI visualizer and logger for COM serial telemetry.
- `plot_raw_q1.py`: Standalone Python script dedicated to plotting raw Q1 electrode data (both live from COM7 and from CSV files).
- `qvar_keyboard.py`: Standalone Python script that maps each detected electrode peak to a native Windows keyboard press (Spacebar by default).
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
  - Configured to `IMU_QVAR_ZIN_2400_MOHM` ($2,400\text{ M}\Omega = 2.4\text{ G}\Omega$) in `QVAR_CONFIG_BUTTON_Q1_ONLY` ([qvar.h](file:///c:/Users/prash/OneDrive/Desktop/IMU/IMU/project/main/qvar.h)) per ST AN5755 Section 5.3.1 (`CTRL7 = 0x80`), providing the maximum possible physical charge-to-voltage conversion gain in silicon ($\approx 3.29\times$ higher than $730\text{ M}\Omega$, and $\approx 10.2\times$ higher than $235\text{ M}\Omega$) for ultra-sensitive touch and proximity detection.
- **Hardware High-Pass Filter (HPF)**:
  - Enabled `.hpfEnable = 1u` in `QVAR_CONFIG_BUTTON_Q1_ONLY` ([qvar.h](file:///c:/Users/prash/OneDrive/Desktop/IMU/IMU/project/main/qvar.h)) per ST AN5755 Section 5.1.4 to eliminate floating-electrode DC static charge accumulation and recenter the baseline around $0\text{ LSB}$.
- **4-Sample Sliding Window Peak-to-Peak Envelope Extractor**:
  - Computes $\text{Activity}[n] = \max(x[n..n-3]) - \min(x[n..n-3])$ over a 4-sample sliding window in `qvar.c`. Spans exactly $20.0\text{ ms}$ ($1$ full cycle of $50\text{ Hz}$ mains powerline hum at $200\text{ Hz}$), converting oscillatory AC touch bursts into a solid, unipolar pulse ($>15,000\text{ LSB}$) with zero phase ambiguity and zero zero-crossing dropouts.
- **200 Hz (5 ms) Polling Loop with 240 Hz Sensor ODR**:
  - Sensor accelerometer / QVAR clock operates at $240\text{ Hz}$ (`ISM330BX_XL_ODR_AT_240Hz`), and FreeRTOS task pacing is configured to $5\text{ ms}$ (`CONFIG_FREERTOS_HZ=1000`, `pdMS_TO_TICKS(5)`). Every polled sample is guaranteed to be a freshly converted reading.
- **Bipolar Peak Detector ($\pm 200\text{ LSB}$ Maximum Sensitivity Threshold)**:
  - Configured specifically for maximum sensitivity, detecting peaks on the slightest feather touch whenever the waveform crosses $\pm 200\text{ LSB}$ ($\approx 2.56\text{ mV}$):
    - **Negative Plunge Peak**: Detects turnaround minimums when $V[n-1] \le V[n-2]$ and $V[n-1] < V[n]$, with plunge $\le -200\text{ LSB}$ (or $\Delta \le -200\text{ LSB}$).
    - **Positive Crest Peak**: Detects turnaround maximums when $V[n-1] \ge V[n-2]$ and $V[n-1] > V[n]$, with crest $\ge +200\text{ LSB}$ (or $\Delta \ge +200\text{ LSB}$).
    - **Hardware Input Impedance**: Configured to $Z_{in} = 2,400\text{ M}\Omega$ for maximum physical analog charge pickup.
    - **Refractory Gating**: Enforces $\ge 70\text{ ms}$ refractory separation between consecutive triggers.
    - **Instant Zero-Latency Emission**:
      ```
      [IMU QVAR] Q1 PEAK (NEG val=-320 delta=-318 LSB / -4.1 mV)
      [IMU QVAR] Q1 PEAK (POS val=+280 delta=+278 LSB / +3.6 mV)
      ```
    - **Visual Alignment**: In `plot_raw_q1.py`, subtle dashed reference lines display at $+200$ and $-200\text{ LSB}$, with yellow markers dropping directly on confirmed peaks. Users can adjust the visualization threshold at any time via `python plot_raw_q1.py --threshold <LSB>`.
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
| `event` | Triggered event label (`Q1 PEAK`, `Q1 BUTTON SINGLE`, `HOLD`, etc.) |

---

## Real-Time Keyboard Controller (`qvar_keyboard.py`)

A standalone host-side Python controller that listens to the QVAR telemetry stream over serial (`COM7`) and triggers native Windows keystrokes (Spacebar by default) on every detected electrode peak.

### Quick Start
```powershell
# Default: Spacebar on COM7
python qvar_keyboard.py

# Custom Key (e.g. Enter, Up, Down)
python qvar_keyboard.py --key enter
python qvar_keyboard.py --key up

# Specify COM port
python qvar_keyboard.py --port COM7
```

### Supported Keys
`space` (default), `enter`, `up`, `down`, `left`, `right`, `tab`, `escape`

### Example Uses
- 🦖 **Chrome Dino Game**: Open `chrome://dino` in your browser and tap your electrode to jump!
- ⏯️ **Media Control**: Tap to pause/play YouTube or Spotify.
- 📑 **Slide Presentations**: Tap to advance slides in PowerPoint / PDF presentation mode.

---

## Standalone Raw Q1 Plotter (`plot_raw_q1.py`)

A focused, lightweight plotting script for the **Q1 electrode** supporting both live serial streaming and recorded CSV visualization.

### Quick Start
```powershell
# 1. Live streaming from COM7 (rolling oscilloscope view with monotonic auto-expansion)
python plot_raw_q1.py

# 2. Lock Y-axis limits (e.g. -35000 to +5000 LSB)
python plot_raw_q1.py --ylim -35000 5000

# 3. View any recorded CSV file (e.g. data/Glasses.csv)
python plot_raw_q1.py data/Glasses.csv

# 4. Adjust rolling window size (default: 300 samples)
python plot_raw_q1.py --window 500
```
