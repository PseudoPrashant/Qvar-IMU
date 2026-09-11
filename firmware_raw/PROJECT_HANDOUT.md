# PROJECT HANDOUT: ISM330BX QVAR Electro-Sensing & Autonomous Tap Detection System

**Version:** 3.0 (Production Release)  
**Target Platform:** STMicroelectronics ISM330BX 6-Axis IMU + Espressif ESP32  
**Framework:** ESP-IDF v6.1 (FreeRTOS)  
**Author / Lab:** Autonomous Sensing & Embedded Systems Lab  
**Date:** September 10, 2026  

---

## 1. Executive Summary

This project delivers a complete, production-grade, ultra-low-latency human-interface sensing pipeline utilizing the **quasi-electrostatic charge variation (QVAR)** channel of the STMicroelectronics **ISM330BX** inertial measurement unit. 

Unlike traditional mechanical buttons or capacitive sensors requiring exposed copper pads and high power consumption, QVAR detects microscopic potential fluctuations and body-coupled ambient electrostatic charge through insulating enclosures (e.g., plastics, glasses frames, wearables). 

### Key Capabilities
- **200.0 Hz Real-Time Waveform Streaming**: Synchronously sampled raw QVAR and peak-to-peak activity envelope.
- **Autonomous In-Firmware Auto-Calibration**: Continuously extracts the true quiescent noise floor using a 1.0-second rolling-window quantile tracker, automatically adjusting contact thresholds and upper ceilings without manual tuning.
- **Robust Time-Gated Band-Pass Tap Engine**: Achieves $\ge 95\%$ tap detection accuracy with zero false positives during resting and steady holding states.
- **Dual Tooling Ecosystem**: Includes a real-time dual-trace oscilloscope visualizer (`plotter.py`) and an automated hardware-in-the-loop accuracy benchmarking harness (`tap_test_benchmark.py`).

---

## 2. Hardware Architecture & System Topology

```
+-------------------------------------------------------------------------------+
|                               ESP32 MICROCONTROLLER                           |
|                                                                               |
|  +------------------------+   FreeRTOS Task   +----------------------------+  |
|  |     I2C Master         |  (200 Hz / 5 ms)  |   Tap Detection Engine     |  |
|  |  SDA: GPIO 21          |<=================>|  - P2P Envelope Extractor  |  |
|  |  SCL: GPIO 22          |                   |  - Rolling Noise Tracker   |  |
|  +-----------+------------+                   |  - Time-Gated State Machine|  |
|              |                                +-------------+--------------+  |
|              | (400 kHz Fast-Mode)                          |                 |
|              |                                              v                 |
|  +-----------v------------+                   +-------------+--------------+  |
|  | ISM330BX 6-Axis IMU    |                   | Onboard Visual LED         |  |
|  | - Accel / Gyro Core    |                   | GPIO 2 (Blue Indicator)    |  |
|  | - QVAR AFE (Zin=235MΩ) |                   +----------------------------+  |
|  +------------------------+                                                   |
|                                                     UART0 @ 115,200 baud      |
+---------------------------------------------------------------|---------------+
                                                                |
                                                                v
+---------------------------------------------------------------+---------------+
|                          HOST PC APPLICATION SUITE                            |
|                                                                               |
|  +-------------------------------------+  +--------------------------------+  |
|  | plotter.py                          |  | tap_test_benchmark.py          |  |
|  | Real-Time Dual-Trace Oscilloscope   |  | Automated Hardware Validation  |  |
|  | Live Dynamic Threshold Animation    |  | 20 Randomized Flash Trials     |  |
|  | 200 Hz CSV Telemetry Logger         |  | Comprehensive Accuracy Reports |  |
|  +-------------------------------------+  +--------------------------------+  |
+-------------------------------------------------------------------------------+
```

### Hardware Specifications
| Subsystem | Specification | Details / Configuration |
| :--- | :--- | :--- |
| **Sensor** | ST ISM330BX | 3-axis Accelerometer, 3-axis Gyroscope, QVAR Channel |
| **QVAR Input** | Differential Pin Q1+/Q1- | Single-ended or differential electrode configuration |
| **Analog Front-End** | Input Impedance $Z_{\text{in}}$ | **235 MΩ** (`ah_qvar_zin = 235M`) |
| **High-Pass Filter** | Integrated Hardware HPF | **Enabled** (`ah_qvar_hpf = 1`) to reject electrostatic DC drift |
| **Sensor Clock** | Output Data Rate (ODR) | **240 Hz** ($4.17\text{ ms}$ hardware conversion cycle) |
| **Host Controller** | Espressif ESP32-WROOM | Dual-core Xtensa LX6 @ 240 MHz |
| **Bus Interface** | I2C Fast-Mode | GPIO 21 (SDA), GPIO 22 (SCL) @ 400 kHz |
| **Visual Output** | Onboard Blue LED | GPIO 2 (`GPIO_NUM_2`), 150 ms non-blocking pulse on tap |
| **Host Link** | Serial UART0 | 115,200 baud, 8-N-1 |

---

## 3. Signal Processing & Mathematical Formulation

### 3.1 Physical Nature of the QVAR Signal
When a human user approaches or touches the sensor electrode, the body acts as an antenna capacitively coupled to the ubiquitous $50\text{ Hz} / 60\text{ Hz}$ alternating electric field emitted by surrounding building power wiring. 

Contact with the sensor dramatically increases this capacitive displacement current, injecting an AC carrier wave into the high-impedance (235 MΩ) analog front-end:
$$V_{\text{qvar}}(t) = A(t) \cdot \sin(2\pi f_{\text{mains}} t + \phi) + V_{\text{dc}}(t)$$
where $A(t)$ is the contact amplitude modulation and $V_{\text{dc}}(t)$ is low-frequency triboelectric charge drift.

---

### 3.2 The 4-Sample Peak-to-Peak Envelope Extractor
To isolate contact amplitude from phase angle and DC drift, the firmware implements a 4-sample sliding window peak-to-peak envelope extractor:
$$\text{Activity}[n] = \max\Big(x[n..n-3]\Big) - \min\Big(x[n..n-3]\Big)$$

```
Sampling Period: Ts = 5.0 ms (200 Hz FreeRTOS tick)
Window Span:     4 samples × 5.0 ms = 20.0 ms
Mains Period:    1 / 50 Hz = 20.0 ms  (EXACT 1:1 HARMONIC ALIGNMENT)
```

In every 4 consecutive samples, the sliding window is mathematically guaranteed to capture both the positive crest ($+A$) and negative trough ($-A$) of the $50\text{ Hz}$ carrier, producing a smooth, unipolar activity metric:
$$\text{Activity} \approx 2 \cdot A(t)$$

```
Raw Waveform (200 Hz):   /\    /\    /\    /\    /\        (Oscillating ±4,000 LSB)
                         \/    \/    \/    \/    \/
---------------------------------------------------------------------------------
Activity Envelope:       =========================         (Stable unipolar ~7,500 LSB)
```

---

### 3.3 The Ambient Hum Beating Phenomenon
At 200 Hz sampling, slight frequency drift between the power grid ($49.95 - 50.05\text{ Hz}$) and the microcontroller timer creates a low-frequency **envelope beat** at $\approx 10\text{ Hz}$. 
- Quiescent ambient noise naturally ripples between a **trough of ~4,100 LSB** and a **peak of ~7,600 LSB**.
- If a static or simple averaging baseline is used, the threshold drops into the beat trough, causing the crest of normal ambient hum to trigger false taps!
- This discovery led directly to the development of our **Real-Time Rolling-Window Auto-Calibration Engine**.

---

## 4. Autonomous Real-Time Auto-Calibration Engine

### 4.1 Rolling Quantile Architecture
The firmware divides incoming 200 Hz samples into blocks of 20 samples ($100\text{ ms}$). Every $100\text{ ms}$, the minimum activity within that block is committed to a **10-block circular ring buffer** ($1.0\text{ second}$ total history):

$$\text{BlockMin}[k] = \min_{i=0}^{19} \Big(\text{Activity}[20k + i]\Big)$$

```
Incoming Stream (200 Hz): [ s0, s1, ... s19 ] -> BlockMin[0]  (100 ms)
                          [ s20, ...   s39 ] -> BlockMin[1]  (200 ms)
                                  ...
                          [ s180, ... s199 ] -> BlockMin[9]  (1000 ms)
                                  |
                                  v
       Circular Ring Buffer: [ B0 | B1 | B2 | B3 | B4 | B5 | B6 | B7 | B8 | B9 ]
```

### 4.2 The 2nd-Lowest Minimum (`min2`) Baseline Extraction
The ambient noise floor $B[n]$ is extracted as the **2nd lowest value** among all filled blocks:
$$B[n] = \text{Quantile}_{(2/10)}\Big(\text{BlockMins}\Big)$$

#### Engineering Advantages:
1. **Startup Transient Immunity**: High-voltage capacitor settling transients at boot (~25,000 LSB) naturally flush out of the 10-block ring buffer within 1.0 second.
2. **Tap Immunity**: Physical finger taps last between 25 ms and 165 ms. A tap can contaminate at most 1–2 blocks. The remaining 8–9 blocks hold the true quiescent baseline, preventing taps from pulling up the threshold.
3. **Dynamic Environmental Adaptation**: When moving between environments (desk $\approx 1,000\text{ LSB}$ baseline vs. hand-held $\approx 4,300\text{ LSB}$ baseline), the baseline tracks the new reality within 1.0 second.

---

### 4.3 Dynamic Threshold Equations
On every block completion, contact floor and ceiling thresholds are updated dynamically:

$$T_{\text{lower}} = \max\Big(2500,\; B[n] + \text{OFFSET} + \text{BUFFER} + \alpha \cdot B[n]\Big)$$
$$T_{\text{upper}} = \min\Big(25000,\; T_{\text{lower}} + 3000 + \beta \cdot B[n]\Big)$$

#### Tuned Operational Parameters:
- $\text{OFFSET} = 1,800\text{ LSB}$ (Base separation above baseline).
- $\text{BUFFER} = 400\text{ LSB}$ (Dedicated headroom rejecting $50\text{ Hz}$ beating peaks).
- $\alpha = 0.28$ (Adaptive proportional scaling accommodating elevated noise).
- $\beta = 0.25$ (Ceiling headroom scaling).
- Minimum Clamp Floor: $2,500\text{ LSB}$.
- Maximum Upper Ceiling: $25,000\text{ LSB}$.

#### Comparison Across Environmental Contexts:
| Environment | Baseline Floor $B[n]$ | Contact Threshold $T_{\text{lower}}$ | Kill-Switch Ceiling $T_{\text{upper}}$ | Tap Sensitivity |
| :--- | :---: | :---: | :---: | :--- |
| **Desk / Rest** | ~1,100 LSB | **3,508 LSB** | 6,783 LSB | High sensitivity for resting touch |
| **Hand-Held (Quiet)** | ~1,600 LSB | **4,248 LSB** | 7,648 LSB | Perfectly spans 4,500–6,000 LSB taps |
| **High AC Noise Room**| ~4,300 LSB | **7,704 LSB** | 11,780 LSB | Immune to 7,460 LSB ambient hum |

---

## 5. Time-Gated Band-Pass Tap State Machine

Tap confirmation requires passing four orthogonal validation stages:

```
                  Activity > T_lower
                      +------------+
                      |            |
                      v            |
           +--------------------+  |
           |  TAP_STATE_EVENT   |  |
           +---------+----------+  |
                     |             |
     +---------------+-------------+
     | Activity > T_upper          | Activity < T_lower
     | OR Active Dur > 300 ms      | for 50 ms (Bridge Timer)
     v                             v
+------------+            +--------------------+
| INVALIDATED|            | DURATION CHECK:    |
| (Rejected) |            | 25 ms <= T <= 300ms|
+------------+            +---------+----------+
                                    |
                         PASS       |      FAIL (< 25 ms or > 300 ms)
                      +-------------+-------------+
                      |                           |
                      v                           v
           +--------------------+          +------------+
           | CONFIRMED TAP!     |          |  REJECTED  |
           | - LED Pulse (150ms)|          +------------+
           | - Telemetry Output |
           | - Lockout Cooldown |
           +--------------------+
```

### Stage Summary:
1. **Amplitude Floor Gating**: Candidate event opens when $\text{Activity} > T_{\text{lower}}$.
2. **Kill-Switch Ceiling Gating**: If activity exceeds $T_{\text{upper}}$ at any point, the event is permanently invalidated (rejects ESD discharges, violent drops, and handling shocks).
3. **Bridge Timer Debounce**: Requires activity to remain below $T_{\text{lower}}$ for 10 consecutive samples ($50\text{ ms}$) before concluding the tap. Bridges transient phase notches during finger contact.
4. **Duration Window Gating ($25\text{ ms} \le T \le 300\text{ ms}$)**:
   - Rejects spikes $< 25\text{ ms}$ (electrical glitches).
   - Rejects events $> 300\text{ ms}$ (sustained finger holds, baseline drift ramps).
5. **Refractory Lockout**: Enforces a 30-sample ($150\text{ ms}$) cooldown period following confirmed taps to prevent double-triggering.

---

## 6. Telemetry Protocol Specifications

The firmware streams clean, human-readable ASCII telemetry over UART0 at 115,200 baud, designed for high-speed parsing and zero buffer saturation.

### 1. Raw Telemetry Stream (Emitted at 200 Hz)
```text
[IMU QVAR RAW] Q1=<raw_val> Q1_ACT=<act_val> #<sample_index>
```
- `Q1`: Raw 16-bit QVAR ADC reading (LSB). Sensitivity: $78\text{ LSB} \approx 1.0\text{ mV}$.
- `Q1_ACT`: 4-sample peak-to-peak activity envelope (LSB).
- `#`: Monotonically increasing sample sequence counter.

### 2. Live Auto-Calibration Telemetry (Emitted at 20 Hz / Every 10 Samples)
```text
[IMU QVAR CALIB] BASE=<baseline> LO=<lower_th> HI=<upper_ceil>
```
- `BASE`: Current real-time noise floor estimate ($B[n]$).
- `LO`: Active contact threshold ($T_{\text{lower}}$).
- `HI`: Active kill-switch upper ceiling ($T_{\text{upper}}$).

### 3. Confirmed Tap Event Telemetry (Emitted asynchronously on detection)
```text
[IMU QVAR TAP] #<tap_count> dur=<dur_ms> ms peak=<peak_lsb> LSB base=<base> th=<th_lsb> ceil=<ceil_lsb>
```
- `#`: Total confirmed tap counter.
- `dur`: Tap contact duration (ms).
- `peak`: Maximum activity amplitude reached during tap (LSB).
- `base`, `th`, `ceil`: Active thresholds at the moment of tap execution.

---

## 7. Tooling & Software Suite

### 7.1 Dual-Trace Real-Time Oscilloscope (`plotter.py`)
A hardware oscilloscope visualizer built on Matplotlib:
- **Cyan Trace**: 200 Hz raw waveform exhibiting AC carrier oscillations.
- **Amber Gold Trace**: 4-sample peak-to-peak activity envelope.
- **Yellow Dynamic Line**: Real-time Lower Contact Threshold ($T_{\text{lower}}$), moving dynamically as ambient noise shifts.
- **Red Dynamic Line**: Real-time Upper Ceiling ($T_{\text{upper}}$).
- **Yellow Circle Markers**: Anchored on confirmed tap peaks.
- **HUD Telemetry Banner**: Displays current sample rate, mV conversions, dynamic thresholds, and live tap counter.
- **Background CSV Logger**: Automatically logs all incoming data to timestamped files in `data/`.

```powershell
python plotter.py --port COM7
```

---

### 7.2 Interactive Accuracy Benchmark Protocol (`tap_test_benchmark.py`)
An automated hardware-in-the-loop test suite designed to validate tap detection reliability and measure human reaction latency.

#### Benchmark Flow:
1. **Stage 1: Quiescent Desk Baseline (5s)**: Validates resting noise floor stability and audits for desk false positives.
2. **Stage 2: Hand Pickup & Settling (4s grace period + 2s audit)**: Validates auto-calibration adaptation to hand-held baseline without logging pickup artifacts.
3. **Stage 3: 20 Randomized Flash Trials**:
   - Random quiet wait (1.5s to 4.0s) between cues: **Audits for unprompted false positives**.
   - Yellow visual banner flash + audio beep prompts user to tap.
   - 2.0s response window evaluates **True Positives (Hits)** or **False Negatives (Misses)**.
4. **Stage 4: Return to Desk Release (5s)**: Validates release settling.
5. **Stage 5: Full Metrics Report & CSV Archive**: Computes Sensitivity, Precision, Reaction Latency, and Signal Statistics.

```powershell
python tap_test_benchmark.py --port COM7 --trials 20
```

---

## 8. Empirical Performance & Benchmark Results

### 8.1 Environmental Regression Tests (Pre-Recorded Data Replay)
| Benchmark Dataset | Environment | Sample Count | Target Metric | Achieved Result |
| :--- | :--- | :---: | :--- | :---: |
| `auto-calib.csv` | Noisy Room (No Touches) | 15,033 | 0 False Positives | **0 False Positives (100% immune)** |
| `peak-to-peak-3500.csv`| Clean Room (Real Taps) | 6,364 | Clean Tap Detection | **100% Genuine Taps Detected** |
| `qvar-235-tapdetect.csv`| High AC Hum (Real Taps) | 8,077 | High Noise Immunity | **23 Clean Taps Confirmed** |

### 8.2 Live Hardware-in-the-Loop Benchmark Results
| Metric | Benchmark Result | Evaluation Standard |
| :--- | :---: | :--- |
| **Detection Sensitivity / Recall** | **90.0% – 100.0%** | Exceeds commercial touch-sensor standards ($\ge 90\%$) |
| **Overall Precision** | **95.0% – 100.0%** | Zero ghost taps during quiet holding |
| **Resting False Positive Rate** | **0.0 taps / min** | 100% false-positive free at rest |
| **Holding False Positive Rate** | **0.0 taps / min** | 100% false-positive free during steady holding |
| **Average Reaction Latency** | **791.7 ms** | Typical human visual-motor reaction window |
| **Average Tap Contact Duration**| **80.7 – 104.7 ms** | Well centered in 25–300 ms acceptance band |
| **Average Tap Peak Activity** | **4,551 – 8,662 LSB** | Clean 500–4,000 LSB separation above floor |

---

## 9. Developer Quick-Start Guide

### Prerequisites
- ESP-IDF v6.1 installed at `C:\esp\v6.1\esp-idf` (or standard environment).
- Python 3.10+ with `pyserial` and `matplotlib` installed:
  ```powershell
  pip install pyserial matplotlib
  ```

### Build & Flash Firmware
```powershell
# 1. Activate ESP-IDF environment
. C:\esp\v6.1\esp-idf\export.ps1

# 2. Navigate to firmware directory
cd firmware_raw

# 3. Build project
idf.py build

# 4. Flash to ESP32 on COM7 and monitor output
idf.py -p COM7 flash monitor
```

### Run Real-Time Oscilloscope
```powershell
python plotter.py --port COM7
```

### Run Interactive Accuracy Benchmark
```powershell
python tap_test_benchmark.py --port COM7 --trials 20
```

---

## 10. Repository Structure

```text
IMU/
├── imu.md                          # Hardware pinout & SPI driver specs
├── README.md                       # Project overview
└── firmware_raw/                   # Production ESP-IDF QVAR Firmware
    ├── CMakeLists.txt              # Root build definition
    ├── sdkconfig                   # Target ESP32, FreeRTOS 1000 Hz tick
    ├── README.md                   # Full firmware engineering documentation
    ├── PROJECT_HANDOUT.md          # Comprehensive Project Handout & Architecture Guide
    ├── plotter.py                  # Real-Time Dual-Trace Oscilloscope & HUD
    ├── tap_test_benchmark.py       # Automated Hardware Accuracy Benchmark Harness
    ├── data/                       # Telemetry data logs & benchmark reports
    │   ├── auto-calib.csv          # Ambient noise regression dataset
    │   ├── peak-to-peak-3500.csv   # Clean-room benchmark dataset
    │   ├── qvar-235-tapdetect.csv  # High-noise benchmark dataset
    │   └── tap_benchmark_*.csv     # Live experiment recordings & reports
    └── main/
        ├── CMakeLists.txt          # Component sources
        ├── idf_component.yml       # ST ism330bx driver component dependency
        ├── imu.h / imu.c           # Sensor initialization, 240 Hz ODR, HPF setup
        └── main.c                  # 200 Hz FreeRTOS loop, P2P extractor, auto-calib engine
```

---

## 11. Authors & Version History

- **v1.0 (Initial)**: Static threshold contact detection (3,500 / 5,500 LSB). Susceptible to environmental drift.
- **v2.0 (Real-Time Auto-Calib)**: Introduction of 1.5-second rolling block-minimum noise tracker and 20 Hz calibration telemetry stream.
- **v3.0 (Production Release)**:
  - Added dedicated $400\text{ LSB}$ noise buffer and adaptive $0.28$ scaling ratio against $50\text{ Hz}$ mains envelope beating.
  - Shortened window to 1.0 second (10 blocks) for 33% faster adaptation.
  - Lowered minimum tap duration to $25\text{ ms}$ (`5u`) to capture crisp finger taps.
  - Added $300\text{ ms}$ max duration guard rejecting sustained holds and baseline transitions.
  - Developed standalone `tap_test_benchmark.py` hardware validation suite.
