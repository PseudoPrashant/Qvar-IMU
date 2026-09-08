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

## Real-Time Telemetry & Plotting (`plot_qvar.py`)

A real-time Python graphical monitor is provided to visualize the QVAR electrode data streaming over serial:

### Features
- **Rolling Waveform**: Real-time plot of Q1 (Button) and Q2 (Wear) electrode channels.
- **Dynamic Baseline & Thresholds**: Automatically extracts and draws horizontal guides for baseline learning, press thresholds, and release thresholds.
- **Event Detection HUD**: Real-time visual markers for button taps (`SINGLE`), hold events (`HOLD`), and wear sensing.
- **CSV Data Recording**: Optional recording of raw samples with timestamps.
- **Offline Mock Simulation**: Built-in mock generator (`--mock`) for testing without hardware connected.

### Requirements
Ensure dependencies are installed:
```powershell
pip install pyserial matplotlib numpy
```

### Running the Plotter
> **Important**: On Windows, serial COM ports cannot be shared across multiple programs. If `idf.py monitor` is running, exit it first (`Ctrl + ]`) before running the plotter.

1. **Connect to COM7 (default)**:
   ```powershell
   python plot_qvar.py --port COM7
   ```
2. **Custom Window Size & Logging**:
   ```powershell
   python plot_qvar.py --port COM7 --window 300 --record qvar_log.csv
   ```
3. **Offline Mock Simulation Mode**:
   ```powershell
   python plot_qvar.py --mock
   ```

