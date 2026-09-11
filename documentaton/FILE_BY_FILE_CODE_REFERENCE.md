# File-by-File Technical Code Reference
## Complete Codebase Breakdown for All ISM330BX QVAR Firmware

**Target Platform:** STMicroelectronics ISM330BX IMU + Espressif ESP32  
**Framework:** ESP-IDF (FreeRTOS)  
**Repository:** [PseudoPrashant/Qvar-IMU](file:///c:/Users/prash/OneDrive/Desktop/IMU/IMU)  

This document provides an exhaustive, file-by-file technical explanation of every source file, header, build script, and Python companion tool across all three firmware directories:
1. **[`dual qvar/`](#1-dual-qvar-differential-firmware)** (Differential Dual-Pad Tap & Direction Detection)
2. **[`firmware_raw/`](#2-firmware_raw-single-ended-200-hz-stream--auto-calibration)** (Single-Ended 200 Hz Stream & Rolling Auto-Calibration)
3. **[`project/`](#3-project-modular-wearable-imu-framework)** (Modular Multi-Feature Wearable Framework)

---

## 1. `dual qvar/` (Differential Firmware)

```text
dual qvar/
├── CMakeLists.txt              # Root build configuration
├── sdkconfig                   # FreeRTOS & hardware settings
├── plotter.py                  # Real-time dual-trace oscilloscope
├── tap_test_benchmark.py       # Automated 20-trial accuracy benchmark
├── compare_plot.py             # Side-by-side CSV comparator
└── main/
    ├── CMakeLists.txt          # Component definition & source list
    ├── imu.h                   # Public IMU driver API
    ├── imu.c                   # Hardware initialization & differential QVAR config
    ├── main.c                  # 200 Hz FreeRTOS task, envelope, auto-calib & tap state machine
    ├── ism330bx_reg.h          # STMicroelectronics official register bitfields
    └── ism330bx_reg.c          # STMicroelectronics register access functions
```

---

### File 1.1: `dual qvar/main/main.c`

#### Purpose & Role:
The main application entry point and real-time execution engine. It initializes the I2C master peripheral, configures the ISM330BX sensor, and executes a strictly paced 200 Hz (5 ms) FreeRTOS polling loop. Inside the loop, it computes the 4-sample peak-to-peak envelope, tracks ambient noise using a rolling quantile buffer, dynamically calculates lower contact thresholds and upper ceilings, runs the tap state machine, identifies which pad was tapped (Pad 1 vs Pad 2), and pulses the blue LED on GPIO 2.

#### Key Data Structures:
```c
/* 1. Sliding Window Peak-to-Peak Envelope Extractor */
#define ENVELOPE_WINDOW_SIZE  4u  /* 4 samples * 5 ms = 20 ms = 1 full cycle of 50 Hz */

typedef struct {
    int16_t window[ENVELOPE_WINDOW_SIZE]; /* Circular sample buffer */
    uint8_t count;                        /* Filled count (up to 4) */
    uint8_t head;                         /* Circular insertion pointer */
} qvar_envelope_extractor_t;

/* 2. Robust Tap Detector & Auto-Calibration State Machine */
typedef struct {
    uint16_t block_mins[ROLLING_NUM_BLOCKS]; /* Ring buffer storing minimum of each 100 ms block */
    uint16_t cur_block_min;                  /* Running minimum of the active block */
    uint16_t cur_block_max;                  /* Running maximum of the active block */
    uint8_t  sample_in_block;                /* Counter from 0 to 19 */
    uint8_t  block_idx;                      /* Circular ring buffer index (0 to 9) */
    uint8_t  blocks_filled;                  /* Blocks populated so far (up to 10) */
    float    baseline_act;                   /* Current learned noise floor baseline */
    int32_t  lower_threshold;                /* Dynamically calculated lower tap threshold */
    int32_t  upper_ceiling;                  /* Dynamically calculated upper kill-switch ceiling */

    tap_state_t state;                       /* TAP_STATE_IDLE, TAP_STATE_EVENT, TAP_STATE_LOCKOUT */
    uint8_t in_event;                        /* 1 if candidate tap event is currently open */
    uint8_t event_valid;                     /* 1 if tap is valid; set to 0 if ceiling breached */
    uint32_t start_idx;                      /* Sample index when threshold was crossed */
    uint32_t last_above_idx;                 /* Last sample index activity remained above threshold */
    int32_t peak_act;                        /* Maximum activity reached during tap */
    uint16_t below_counter;                  /* Bridge timer counting samples below threshold */
    uint16_t lockout_timer;                  /* Cooldown timer preventing double clicks */
    uint32_t tap_count;                      /* Total confirmed taps */
    uint32_t last_tap_dur_ms;                /* Duration of last confirmed tap in ms */
    int16_t  initial_raw_spike;              /* Sign of initial raw sample (determines Pad 1 vs Pad 2) */
} robust_tap_detector_t;
```

#### Key Functions:
1. `qvar_envelope_extractor_update(env, sample)`:
   - Inserts the new raw sample into the circular window.
   - Computes `max_val - min_val` across the 4 stored samples.
   - **Why it matters:** Spanning 20.0 ms, it captures both the positive crest and negative trough of the 50 Hz room hum, producing a stable unipolar activity value with zero phase-dropouts.
2. `robust_tap_detector_recalc_thresholds(det)`:
   - Computes $T_{\text{lower}} = \text{Baseline} + 500 + 200 + 0.28 \cdot \text{Baseline}$ (clamped at minimum 1,000 LSB).
   - Computes $T_{\text{upper}} = T_{\text{lower}} + 4000 + 0.25 \cdot \text{Baseline}$ (clamped at maximum 10,000 LSB).
3. `robust_tap_detector_update(det, sample_idx, raw_val, act_val)`:
   - Updates the 100 ms block minimum and tracks the 2nd lowest block minimum (`min2`) across 1.0 second.
   - When activity crosses $T_{\text{lower}}$, opens the tap candidate and saves `initial_raw_spike = raw_val`.
   - If activity exceeds $T_{\text{upper}}$, permanently invalidates the candidate tap.
   - When signal stays below $T_{\text{lower}}$ for 50 ms (10 samples), checks duration:
     - If duration $< 20\text{ ms}$ (`4u`), rejects as sharp noise spike!
     - If duration $> 300\text{ ms}$ (`60u`), rejects as sustained hold!
     - If $20\text{ ms} \le \text{Duration} \le 300\text{ ms}$, confirms tap!
   - Direction check:
     - `initial_raw_spike < 0` $\implies$ Returns `1` (**Pad 1 Q+**).
     - `initial_raw_spike >= 0` $\implies$ Returns `2` (**Pad 2 Q-**).
4. `app_main()`:
   - Initializes I2C master at 400 kHz on GPIO 21 (SDA) and GPIO 22 (SCL).
   - Configures GPIO 2 as an output for the LED indicator.
   - Calls `imu_init()` and `imu_raw_qvar_start(RAW_QVAR_ZIN)`.
   - Loops forever using `vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(5))`.

---

### File 1.2: `dual qvar/main/imu.h`

#### Purpose & Role:
Public header exposing the IMU hardware abstraction interface to `main.c`.

#### Key Declarations:
```c
#include "ism330bx_reg.h"

int imu_init(void);                                         /* Initializes I2C & verifies WHO_AM_I */
int imu_raw_qvar_start(ism330bx_ah_qvar_zin_t zin);         /* Configures differential QVAR AFE */
int imu_raw_qvar_read(int16_t *raw_out);                    /* Reads latest 16-bit raw QVAR conversion */
stmdev_ctx_t *imu_get_ctx(void);                            /* Returns ST driver context pointer */
```

---

### File 1.3: `dual qvar/main/imu.c`

#### Purpose & Role:
Implements low-level register communication with the ISM330BX over the ESP32 I2C peripheral and configures the silicon analog front-end for **differential sensing**.

#### Step-by-Step Execution:
1. `imu_platform_write()` / `imu_platform_read()`:
   - Bridges the official ST driver callbacks to ESP-IDF's `i2c_master_write_to_device()` and `i2c_master_write_read_device()`.
2. `imu_init()`:
   - Queries register `WHO_AM_I` (`0x0F`) and confirms value `0x71` (`ISM330BX_ID`).
   - Enables auto-increment on multi-byte transfers (`ism330bx_auto_increment_set`) and block data update (`ism330bx_block_data_update_set`).
3. `imu_raw_qvar_start(zin)`:
   - **Crucial Differential Configuration**:
     ```c
     // Enables BOTH Q1 and Q2 sensing paths for differential operation
     ism330bx_ah_qvar_mode_t mode = {0};
     mode.ah_qvar1_en = 1u;
     mode.ah_qvar2_en = 1u; // Enables Q2 pin as inverting differential partner
     mode.swaps = 0u;
     ism330bx_ah_qvar_mode_set(&sImuCtx, mode);
     ```
   - Sets input impedance to $300\text{ M}\Omega$ (`ism330bx_ah_qvar_zin_set(&sImuCtx, zin)`).
   - Enables the hardware high-pass filter (`filter.hpf = 1u`) to eliminate baseline drift.
   - Powers up the accelerometer core at $240\text{ Hz}$ ODR (`ISM330BX_XL_ODR_AT_240Hz`), which clocks the QVAR analog front-end.
4. `imu_raw_qvar_read(raw_out)`:
   - Calls `ism330bx_ah_qvar_raw_get()` to fetch the latest signed 16-bit differential conversion from registers `AH_QVAR_OUT_L` and `AH_QVAR_OUT_H`.

---

### File 1.4: `dual qvar/plotter.py`

#### Purpose & Role:
A real-time Python graphical monitor built on Matplotlib. It connects to the ESP32 over serial (e.g. `COM7` at 115,200 baud), displays the live dual-trace oscilloscope, animates dynamic auto-calibration threshold lines, places visual markers on confirmed taps, and automatically logs data to timestamped CSV files.

#### Key Architecture:
- `DualQ1SerialReader`: Runs a background thread reading serial lines. Uses regex:
  - `RE_TELEMETRY`: Parses `[IMU QVAR RAW] Q1=-14 Q1_ACT=480 #100`.
  - `RE_CALIB`: Parses `[IMU QVAR CALIB] BASE=480 LO=1314 HI=5434`.
  - `RE_TAP`: Parses `[IMU QVAR TAP] #1 dur=55 ms peak=2840 LSB`.
- `OscilloscopeUI`:
  - **Cyan Trace**: Raw differential signal showing 200 Hz oscillations.
  - **Amber Gold Trace**: Smooth 4-sample peak-to-peak activity envelope.
  - **Yellow Line**: Live lower contact threshold ($T_{\text{lower}}$) moving dynamically.
  - **Red Line**: Live upper ceiling kill-switch ($T_{\text{upper}}$).
  - **Yellow Dots**: Placed on confirmed tap peaks.
  - **Monotonic Y-Axis Scaling**: Automatically scales upward during large taps without jumpy vertical snapping. Press `'r'` on the window to reset the scale.

---

### File 1.5: `dual qvar/tap_test_benchmark.py`

#### Purpose & Role:
An automated hardware-in-the-loop accuracy benchmarking harness. It guides the user through randomized prompt trials to evaluate detection sensitivity, precision, false alarm rate, and human reaction latency.

#### Test Execution Protocol:
1. **Stage 1 (Quiescent Desk Baseline, 5s)**: Sensor sits undisturbed on the desk. Audits for resting false positives.
2. **Stage 2 (Hand Pickup & Settling, 6s)**: User picks up the sensor. Verifies auto-calibration adapts to hand-held baseline without false triggering.
3. **Stage 3 (20 Randomized Flash Trials)**:
   - Quiet wait between trials is randomized between $1.5\text{ s}$ and $4.0\text{ s}$ (audits for steady holding false alarms).
   - Audio beep and visual prompt cue the user to tap.
   - $2.0\text{ s}$ response window records True Positives (Hits) with latency or flags False Negatives (Misses).
4. **Stage 4 (Return to Desk, 5s)**: Verifies release settling stability.
5. **Stage 5 (Summary Report)**: Calculates Sensitivity ($\%$) and Precision ($\%$) and saves a summary text report and tagged continuous CSV file to `data/`.

---

### File 1.6: `dual qvar/compare_plot.py`

#### Purpose & Role:
A standalone analysis tool that compares two recorded CSV sessions side-by-side.
- Generates side-by-side and overlaid time-domain waveform comparisons.
- Plots amplitude distribution histograms to compare noise floor spread.
- Computes Signal-to-Noise Ratio (SNR), Mean, Median, Standard Deviation, and Peak-to-Peak values.

---

### Files 1.7 & 1.8: `dual qvar/CMakeLists.txt` & `main/CMakeLists.txt`

#### Purpose & Role:
Build configuration files for the ESP-IDF build system:
- Root `CMakeLists.txt`: Defines the project name: `project(dual_qvar)`.
- `main/CMakeLists.txt`: Declares component source files (`main.c`, `imu.c`, `ism330bx_reg.c`), include directories (`.`), and dependencies (`driver`, `freertos`).

---

### File 1.9: `dual qvar/sdkconfig`

#### Purpose & Role:
The Kconfig system configuration file for the ESP32 firmware:
- `CONFIG_FREERTOS_HZ=1000`: Sets the FreeRTOS tick rate to $1\text{ kHz}$ ($1\text{ ms}$ tick resolution), which enables precise $5.0\text{ ms}$ ($200\text{ Hz}$) periodic polling.
- `CONFIG_ESP_CONSOLE_UART_BAUDRATE=115200`: Configures serial monitor communication baud rate.
- Sets target chip to standard ESP32 dual-core Xtensa LX6.

---

## 2. `firmware_raw/` (Single-Ended 200 Hz Stream & Auto-Calibration)

```text
firmware_raw/
├── CMakeLists.txt              # Root build configuration
├── sdkconfig                   # Target ESP32, CONFIG_FREERTOS_HZ=1000
├── plotter.py                  # Real-time oscilloscope visualizer
├── tap_test_benchmark.py       # Automated hardware benchmarking harness
├── compare_plot.py             # CSV comparison script
└── main/
    ├── CMakeLists.txt          # Component declaration
    ├── imu.h                   # IMU driver interface
    ├── imu.c                   # Single-ended Q1 hardware setup
    ├── main.c                  # 200 Hz raw streamer & rolling auto-calibration engine
    ├── ism330bx_reg.h          # ST register definitions
    └── ism330bx_reg.c          # ST register read/write functions
```

---

### File 2.1: `firmware_raw/main/main.c`

#### Purpose & Role:
Pioneered the high-speed 200 Hz sampling architecture, the 4-sample peak-to-peak envelope extractor, and the rolling-window auto-calibration engine for a **single-ended** QVAR electrode pad.

#### Differences from `dual qvar/main/main.c`:
1. **Single-Ended Calibration Values**:
   - `CALIB_LOWER_OFFSET = 1800L` (higher threshold offset for single-ended amplitude swings).
   - `CALIB_LOWER_BUFFER = 400L` (dedicated noise headroom buffer against single-ended 50 Hz hum beating).
   - `CALIB_MIN_LOWER_TH = 2500L` (safety floor clamp for single-ended mode).
   - `CALIB_MAX_UPPER_CEIL = 25000L` (higher ceiling clamp).
2. **Tap Duration Gate**:
   - `TAP_MIN_DUR_SAMPLES = 5u` ($25\text{ ms}$).
3. **No Direction Check**:
   - Because it operates on a single pad, it confirms taps without polarity direction checks (`initial_raw_spike`).

---

### File 2.2: `firmware_raw/main/imu.c`

#### Purpose & Role:
Initializes the ISM330BX in **single-ended mode**:
```c
// Select ONLY Q1 electrode input (single-ended)
ism330bx_ah_qvar_mode_t mode = {0};
mode.ah_qvar1_en = 1u;
mode.ah_qvar2_en = 0u; // Q2 is disabled
mode.swaps = 0u;
ism330bx_ah_qvar_mode_set(&sImuCtx, mode);

// Configures impedance to 730 MOhm or 235 MOhm
ism330bx_ah_qvar_zin_set(&sImuCtx, zin);
```

---

### File 2.3: `firmware_raw/plotter.py` & `tap_test_benchmark.py`
Identical in functionality to the tools in `dual qvar/`, but configured with default file paths pointing to `firmware_raw/data/`.

---

## 3. `project/` (Modular Wearable IMU Framework)

```text
project/
├── CMakeLists.txt              # Root build configuration
├── sdkconfig                   # Target ESP32, CONFIG_FREERTOS_HZ=1000
├── plot_qvar.py                # Dual-channel live GUI visualizer & CSV archiver
├── plot_raw_q1.py              # Lightweight raw Q1 visualizer
├── qvar_keyboard.py            # Real-time Windows HID controller (Spacebar on tap)
├── compare_plot.py             # CSV comparison script
└── main/
    ├── CMakeLists.txt          # Component definition & source list
    ├── imu.h                   # Public IMU API
    ├── imu.c                   # Low-level I2C & IMU sensor core driver
    ├── imu_internal.h          # Internal decoupling hooks
    ├── qvar.h                  # Public QVAR API, structures, and profiles
    ├── qvar.c                  # Wear detection & button state machines
    ├── qvar_config.h           # Active configuration profiles & tuning constants
    ├── main.c                  # App entry point & FreeRTOS task pacing
    ├── ism330bx_reg.h          # ST register definitions
    └── ism330bx_reg.c          # ST register functions
```

---

### File 3.1: `project/main/main.c`

#### Purpose & Role:
Application entry point. Initializes the I2C master driver, calls `imu_init()`, and spawns a periodic 200 Hz FreeRTOS task that delegates execution to `imu_qvar_app_task()` in `qvar.c`.

```c
void app_main(void) {
    i2c_master_init();
    if (imu_init() != 0) return;

    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xPeriod = pdMS_TO_TICKS(5); /* 5 ms = 200 Hz */

    while (1) {
        imu_qvar_app_task();
        vTaskDelayUntil(&xLastWakeTime, xPeriod);
    }
}
```

---

### File 3.2: `project/main/qvar.h`

#### Purpose & Role:
Declares data types, enums, configuration profiles, and function prototypes for the multi-channel QVAR subsystem.

#### Key Enums & Structs:
- `imu_qvar_use_t`: `IMU_QVAR_USE_DISABLED`, `IMU_QVAR_USE_RAW`, `IMU_QVAR_USE_WEAR`, `IMU_QVAR_USE_BUTTON`.
- `imu_qvar_zin_t`: `IMU_QVAR_ZIN_2400_MOHM`, `IMU_QVAR_ZIN_730_MOHM`, `IMU_QVAR_ZIN_300_MOHM`, `IMU_QVAR_ZIN_235_MOHM`.
- `imu_qvar_config_t`: Holds feature flags, impedance, filter enables, and electrode role assignments.
- `imu_qvar_raw_t`: Container holding latest raw samples for `qvar1` and `qvar2` along with validity flags.

---

### File 3.3: `project/main/qvar_config.h`

#### Purpose & Role:
Central configuration header storing application profiles and tuning macros:
- `QVAR_APP_ACTIVE_CONFIG`: Active profile set to `QVAR_APP_CONFIG_BUTTON_Q1_ONLY`.
- Predefined profile definitions:
  - `QVAR_CONFIG_DISABLED`: Accelerometer/gyroscope only; QVAR powered down.
  - `QVAR_CONFIG_RAW_BOTH`: Reads Q1 and Q2 without state machine filtering.
  - `QVAR_CONFIG_WEAR_Q2_BUTTON_Q1`: Wear detection on Q2 + Button tap/hold on Q1.
  - `QVAR_CONFIG_BUTTON_Q1_ONLY`: Single button on Q1 with ultra-high $2.4\text{ G}\Omega$ impedance.
- Wear tuning constants: `QVAR_APP_WEAR_AVG_BUFFER_MAX_SAMPLES = 8`, `QVAR_APP_WEAR_CONFIRM_MS = 150`, `QVAR_APP_REMOVE_CONFIRM_MS = 250`.
- Button tuning constants: Press threshold ($3,300\text{ LSB}$), release threshold ($2,650\text{ LSB}$), minimum duration ($10\text{ ms}$), maximum duration ($250\text{ ms}$).

---

### File 3.4: `project/main/qvar.c`

#### Purpose & Role:
The complete multi-channel QVAR sensing engine. Contains the state machines for both wear detection (Q2) and button gestures (Q1).

#### Key Subsystems:
1. **Wear Detection State Machine (`imu_app_qvar_wear_channel_t`)**:
   - Maintains an 8-sample circular buffer.
   - Computes rolling average: $\text{Avg} = \frac{1}{N}\sum Q2$.
   - Computes wear delta: $\Delta = \text{Avg} - \text{Baseline}$.
   - If $\Delta \ge \text{Threshold}_{\text{on}}$ for $150\text{ ms}$, transitions to `WORN` state and emits `[IMU QVAR] Q2 WEAR ON`.
   - If $\Delta \le \text{Threshold}_{\text{off}}$ for $250\text{ ms}$, transitions to `REMOVED` state and emits `[IMU QVAR] Q2 WEAR OFF`.
2. **Button Tap & Hold State Machine (`imu_app_qvar_button_channel_t`)**:
   - Tracks baseline using an accumulator during the first $500\text{ ms}$.
   - Evaluates contact threshold ($3,300\text{ LSB}$) and release threshold ($2,650\text{ LSB}$).
   - Rebound crest tracking: Analyzes signal trajectory after peak.
   - Duration gating: If contact $>250\text{ ms}$, transitions to `HOLD` event.
   - Bipolarity verification: Rejects unipolar electrostatic discharges by enforcing $V_{\min} \le -500\text{ LSB}$ AND $V_{\max} \ge +500\text{ LSB}$.
   - Squelch lockout: Enters $250\text{ ms}$ lockout if chattering occurs.
3. **Periodic Baseline Heartbeat**:
   - Emits `[IMU QVAR] Q1 button baseline=...` every 5 seconds so connected hosts can automatically discover active thresholds.

---

### File 3.5: `project/main/imu.c` & `imu.h`

#### Purpose & Role:
Provides the sensor abstraction layer over I2C. Connects ST register calls to ESP-IDF I2C driver functions. Queries `WHO_AM_I` (`0x71`), configures auto-increment, and provides accessors for accelerometer and gyroscope data.

---

### File 3.6: `project/main/imu_internal.h`

#### Purpose & Role:
Internal decoupling header defining private hooks between `imu.c` and `qvar.c` without polluting the public `imu.h` API.

---

### File 3.7: `project/plot_qvar.py`

#### Purpose & Role:
A Python visualizer for the `project/` firmware:
- Streams Q1 and Q2 channels simultaneously.
- Renders threshold guides, baseline tracking lines, and wear detection status.
- Implements **monotonic auto-expansion** (Y-axis expands to accommodate large spikes without bouncing).
- Automatically records all data to `project/data/qvar_telemetry_YYYYMMDD_HHMMSS.csv`.

---

### File 3.8: `project/plot_raw_q1.py`

#### Purpose & Role:
A lightweight, fast oscilloscope visualizer dedicated purely to viewing raw Q1 waveform data from either live serial or pre-recorded CSV files.

---

### File 3.9: `project/qvar_keyboard.py`

#### Purpose & Role:
A real-time host-side Windows keyboard controller:
- Connects to serial port `COM7`.
- Uses Python's built-in `ctypes` to call `ctypes.windll.user32.keybd_event()`.
- On every confirmed tap, injects a native Windows keystroke (Spacebar by default, or Enter, Up, Down).
- Requires zero third-party keyboard drivers; runs with zero latency.
- Allows using the sensor electrode to control the Chrome Dinosaur Game (`chrome://dino`), pause/play YouTube/Spotify, or advance slides.

---

## 4. Shared Component: STMicroelectronics Register Driver

Present in all three firmware implementations under `main/`:
- **`ism330bx_reg.h`**: STMicroelectronics official C header containing 160+ register definitions, bitfield unions, and data type enums for every feature of the ISM330BX (accel, gyro, QVAR, FIFO, embedded functions, interrupts).
- **`ism330bx_reg.c`**: STMicroelectronics official driver implementation providing register read/write helper functions (e.g. `ism330bx_device_id_get()`, `ism330bx_ah_qvar_mode_set()`, `ism330bx_ah_qvar_raw_get()`).

---

## 5. Summary Table: Every File at a Glance

| Directory | File | Language | Primary Responsibility |
| :--- | :--- | :---: | :--- |
| **`dual qvar/`** | `main/main.c` | C | 200 Hz FreeRTOS loop, P2P envelope, auto-calib, plateau filter, Pad 1 vs Pad 2 direction logic, LED pulse. |
| **`dual qvar/`** | `main/imu.c` | C | I2C transport, device ID verification, hardware differential AFE mode setup (`ah_qvar1_en=1`, `ah_qvar2_en=1`, 300M Zin, HPF, 240 Hz ODR). |
| **`dual qvar/`** | `main/imu.h` | C Header | Public declarations for IMU initialization and raw QVAR reading. |
| **`dual qvar/`** | `plotter.py` | Python | Real-time dual-trace oscilloscope with live dynamic threshold animation and CSV logging. |
| **`dual qvar/`** | `tap_test_benchmark.py` | Python | Automated 20-trial hardware-in-the-loop accuracy benchmark harness. |
| **`dual qvar/`** | `compare_plot.py` | Python | Side-by-side CSV comparator, SNR calculation, and histogram visualizer. |
| **`dual qvar/`** | `CMakeLists.txt` | CMake | ESP-IDF project definition. |
| **`dual qvar/`** | `main/CMakeLists.txt` | CMake | Component source registration and dependency declaration. |
| **`dual qvar/`** | `sdkconfig` | Kconfig | ESP32 configuration (FreeRTOS 1000 Hz tick rate, console baud rate 115200). |
| **`firmware_raw/`**| `main/main.c` | C | Single-ended 200 Hz raw stream, rolling-window auto-calibration, and time-gated tap state machine. |
| **`firmware_raw/`**| `main/imu.c` | C | Single-ended QVAR hardware initialization (`ah_qvar1_en=1`, `ah_qvar2_en=0`). |
| **`firmware_raw/`**| `main/imu.h` | C Header | Public driver declarations. |
| **`firmware_raw/`**| `plotter.py` | Python | Dual-trace oscilloscope for single-ended data. |
| **`firmware_raw/`**| `tap_test_benchmark.py` | Python | 20-trial accuracy benchmark harness for single-ended firmware. |
| **`project/`** | `main/main.c` | C | Application entry point delegating to FreeRTOS QVAR task. |
| **`project/`** | `main/qvar.c` | C | Wear detection algorithm (Q2) and button tap/hold state machine (Q1). |
| **`project/`** | `main/qvar.h` | C Header | QVAR application API, enums, structs, and profile definitions. |
| **`project/`** | `main/qvar_config.h`| C Header | Active profile selection (`QVAR_CONFIG_BUTTON_Q1_ONLY`) and tuning parameters. |
| **`project/`** | `main/imu.c` | C | I2C driver and basic IMU sensor core initialization. |
| **`project/`** | `main/imu.h` | C Header | Public IMU interface. |
| **`project/`** | `main/imu_internal.h`| C Header | Private hooks between IMU core and QVAR modules. |
| **`project/`** | `plot_qvar.py` | Python | Dual-channel real-time visualizer with wear and button HUD indicators. |
| **`project/`** | `plot_raw_q1.py` | Python | Lightweight raw Q1 waveform viewer. |
| **`project/`** | `qvar_keyboard.py` | Python | Windows HID bridge converting tap peaks into native Spacebar keypresses. |
| **`Shared`** | `ism330bx_reg.h` | C Header | ST official register map and bitfield unions (160+ registers). |
| **`Shared`** | `ism330bx_reg.c` | C | ST official register read/write access functions. |
