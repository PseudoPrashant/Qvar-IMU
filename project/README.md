# ISM330BX ESP-IDF / FreeRTOS Project

This directory contains the ESP32 / FreeRTOS port of the ISM330BX IMU driver and QVAR electrode polling engine.

## Directory Structure

- `CMakeLists.txt`: ESP-IDF root project definition.
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
