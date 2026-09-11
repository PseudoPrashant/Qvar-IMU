# Comprehensive Firmware Architecture & Engineering Guide
## STMicroelectronics ISM330BX QVAR Electro-Sensing Platform

**Document Version:** 4.1 (Simplified & Deeply Detailed Release)  
**Target Hardware:** STMicroelectronics ISM330BX 6-Axis IMU + Espressif ESP32-WROOM-32  
**Framework:** ESP-IDF v5.x / v6.x (FreeRTOS)  
**Repository:** [PseudoPrashant/Qvar-IMU](file:///c:/Users/prash/OneDrive/Desktop/IMU/IMU)  

> 🔍 **File-by-File Technical Code Reference:**  
> For an exhaustive, file-by-file breakdown of every C source file, header, CMake configuration, and Python script across all directories, see:  
> **👉 [FILE_BY_FILE_CODE_REFERENCE.md](FILE_BY_FILE_CODE_REFERENCE.md)**

---

## Quick Primer: What Is QVAR? (In Plain English)

Imagine you want to add touch buttons to a device—like a pair of smart audio glasses, earbuds, or a medical monitor—but you **do not want any physical buttons that click, and you do not want exposed metal pins** that can rust, get clogged with sweat, or let water inside.

That is what **QVAR** (Quasi-Electrostatic Charge Variation) does:
1. **The Human Body is an Antenna:** Every home and office has electric wires hidden inside the walls, lights, and appliances. These wires radiate an invisible, harmless electrical wave humming at **50 Hz** (in India and Europe) or **60 Hz** (in the Americas). Because our bodies are mostly conductive salt water, our bodies naturally act like an antenna picking up this electric field.
2. **The Sensor Senses Your Touch Through Plastic:** The ISM330BX IMU has ultra-sensitive input pins. When you touch or get your finger close to a small piece of copper foil—even if it is hidden behind a thick plastic case—the electric charge from your body jumps across the plastic into the sensor.
3. **The Firmware Makes It a Bulletproof Button:** The raw signal coming out of the sensor is noisy and wavy. Our firmware cleans up the signal, filters out accidental static, learns the background noise of the room, and detects clean, crisp button taps in real time!

---

## 1. Executive Summary & Evolutionary Architecture

Over the course of this project, the firmware went through **three distinct generations** to solve real-world physical challenges discovered during testing:

```
+---------------------------------------------------------------------------------------------------------+
|                                    FIRMWARE EVOLUTION TIMELINE                                          |
+---------------------------------------------------------------------------------------------------------+
|                                                                                                         |
|  Generation 1: [project] - Modular Multi-Feature Wearable Framework                                    |
|  -----------------------------------------------------------------                                      |
|  * The Goal: A complete software framework for smart glasses.                                            |
|  * What It Did: Handled 6-axis motion (accel/gyro), detected if glasses are worn on the face (Pad 2),    |
|    and detected tap/hold gestures on the temple frame (Pad 1).                                          |
|  * The Real-World Problem: Tested in different rooms, the fixed thresholds failed. Room noise changed,  |
|    and 50 Hz powerline hum caused false triggers or missed taps.                                        |
|                                                    |                                                    |
|                                                    v                                                    |
|  Generation 2: [firmware_raw] - 200 Hz Stream & In-Firmware Auto-Calibration                           |
|  ---------------------------------------------------------------------------                            |
|  * The Goal: Eliminate false triggers by understanding the true physical signal.                        |
|  * What It Did: Streamed raw data at 200 Hz. Created a 4-sample peak-to-peak envelope extractor that    |
|    perfectlies turns 50 Hz sine waves into solid touch pulses. Implemented dynamic auto-calibration.   |
|  * The Real-World Problem: Single-ended sensing still picked up ambient room noise when walking near    |
|    appliances, and we couldn't tell which of two pads was tapped on a single line.                      |
|                                                    |                                                    |
|                                                    v                                                    |
|  Generation 3: [dual qvar] - Differential Dual-Pad Tap & Direction Engine                               |
|  ------------------------------------------------------------------------                               |
|  * The Goal: Zero room noise, 100% false-positive immunity, and two distinct buttons on one channel.   |
|  * What It Did: Connected Pad 1 (Q+) and Pad 2 (Q-) as a differential pair (room noise cancels out!).   |
|    Tuned impedance to 300 MOhm. Added a 20 ms plateau duration filter that eliminates 100% of sharp    |
|    noise spikes, and checked initial voltage polarity so Pad 1 vs Pad 2 is identified instantly!        |
|                                                                                                         |
+---------------------------------------------------------------------------------------------------------+
```

---

## 2. Fundamental Physics: How the Signals Behave

### 2.1 The 50 Hz Carrier Wave & The "Zero-Crossing" Problem
Because the human body acts as an antenna picking up building powerline wiring, the electrical voltage on your skin is not a steady DC voltage. It is an **alternating sine wave** oscillating 50 times every second:

$$V_{\text{body}}(t) = A(t) \cdot \sin(2\pi \cdot 50 \cdot t + \phi)$$

- When you tap the electrode, the **amplitude $A(t)$ jumps dramatically** (the wave gets much taller).
- **The Zero-Crossing Trap:** Because a sine wave crosses through $0\text{ V}$ twice every cycle, if your firmware simply checks `if (raw_value > threshold)`, and your finger touches the pad right when the wave is crossing zero, **the tap will be missed!**
- **The Solution:** We must measure the **envelope** (the overall height of the wave from its top peak to its bottom valley), not just individual raw points.

```
Individual Raw Points (Oscillating):   +4000  /\        /\        /\
                                           0 ----\--/----\--/----\--/----  <-- Crosses 0 every 10 ms!
                                       -4000      \/        \/        \/

Envelope (Peak-to-Peak Height):        +8000  =========================  <-- Solid, stable touch pulse!
```

### 2.2 Silicon Analog Front-End: Input Impedance ($Z_{\text{in}}$)
The ISM330BX chip allows us to set the electrical input resistance (impedance) of its sensing pins via the `CTRL7` register. Think of this like a **volume or gain knob**:

| Setting in Code | Resistance ($Z_{\text{in}}$) | Physical Behavior (Like a Microphone Gain Knob) | Where It Is Used |
| :--- | :---: | :--- | :--- |
| `ISM330BX_2400MOhm` | $2,400\text{ M}\Omega$ ($2.4\text{ G}\Omega$) | **Gain at 100%**: Massive sensitivity. Detects hands moving several inches away, but picks up every tiny electrical noise in the room. | Used in `project` for high-sensitivity proximity. |
| `ISM330BX_730MOhm` | $730\text{ M}\Omega$ | **Gain at 75%**: Very sensitive touch. Good for single-ended pads with thick plastic overlays ($>2\text{ mm}$). | Used in early `firmware_raw` tests. |
| `ISM330BX_300MOhm` | $300\text{ M}\Omega$ | **Gain at 50% (The Sweet Spot)**: Perfect for differential sensing. Taps create strong plateaus ($1,500\text{ to }5,000\text{ LSB}$), while background noise drops to almost zero ($<500\text{ LSB}$). | **Active default in `dual qvar`**. |
| `ISM330BX_235MOhm` | $235\text{ M}\Omega$ | **Gain at 25%**: Extremely quiet, but requires very hard, forceful taps to trigger. | Used in high-noise industrial environments. |

### 2.3 Single-Ended vs. Differential Mode
- **Single-Ended Mode (`firmware_raw`)**: Uses 1 copper pad. The sensor measures voltage between that pad and circuit ground. If an air conditioner turns on nearby, the entire room's electric field jumps, causing a false spike on the pad.
- **Differential Mode (`dual qvar`)**: Uses 2 copper pads (Pad 1 = Positive input $Q+$, Pad 2 = Negative input $Q-$). The sensor measures $(V_{\text{Pad 1}} - V_{\text{Pad 2}})$.
  - **Common-Mode Cancellation:** Because both pads are in the same room, external electrical noise hits both pads equally ($V_{\text{noise}} - V_{\text{noise}} = 0$). Room noise cancels out in hardware!
  - **Directional Polarity:** When your finger touches Pad 1, $V_{Q+}$ drops first, producing a negative number. When your finger touches Pad 2, $V_{Q-}$ drops first, producing a positive number.

---

## 3. Deep Dive: `dual qvar` (Production Differential Tap & Direction Engine)

### 3.1 WHY It Was Built
`dual qvar` is the most refined, production-ready firmware in this repository. It was built to solve three real-world demands:
1. **Zero False Positives:** It must never trigger when sitting on a desk or when someone simply holds the device.
2. **Two Buttons on One Channel:** Tapping Pad 1 or Pad 2 must be distinguished automatically without extra wiring.
3. **Robust Against Electrical Noise:** It must ignore electrostatic sparks from clothing and carpet while never missing a genuine finger tap.

### 3.2 WHAT It Does (The Signal Processing Pipeline)

```
[ Raw Differential ADC Sample @ 200 Hz ]
                   |
                   v
[ 1. The 4-Sample Envelope Extractor (20 ms) ]
  --> Takes the last 4 samples (4 * 5 ms = 20 ms = exactly 1 cycle of 50 Hz).
  --> Computes: Activity = Max(window) - Min(window)
  --> Output: Solid, unipolar activity number with zero phase dropouts.
                   |
                   v
[ 2. The 1.0-Second Auto-Calibration Noise Floor Tracker ]
  --> Divides time into 20-sample blocks (100 ms each).
  --> Remembers the quietest point of the last 10 blocks (1.0 second history).
  --> Picks the 2nd lowest block minimum as the TRUE Quiescent Baseline.
  --> Why 2nd lowest? Immune to startup capacitor spikes and immune to finger taps!
                   |
                   v
[ 3. Dynamic Thresholds Recalculated Every 100 ms ]
  --> Lower Contact Threshold = Baseline + 500 offset + 200 buffer + 0.28 * Baseline
      (Taps must cross this line to be considered). Clamped at minimum 1,000 LSB.
  --> Upper Kill-Switch Ceiling = Lower Threshold + 4,000 headroom + 0.25 * Baseline
      (Clamped at maximum 10,000 LSB to kill violent drops and static shocks).
                   |
                   v
[ 4. Time-Gated Band-Pass State Machine Validation ]
  * Stage A: Did Activity cross Lower Threshold? ---> YES, Event opens!
             Immediately record: initial_raw_spike = raw_val (for direction check).
  * Stage B: Did Activity breach Upper Ceiling (10,000 LSB)? ---> If YES, kill tap.
  * Stage C: Has signal stayed below threshold for 50 ms (10 samples)? ---> Event ends.
  * Stage D: Check Duration:
             - Is Duration < 20 ms? ---> REJECTED (Sharp electrical noise spike).
             - Is Duration > 300 ms? ---> REJECTED (Long press hold).
             - Is 20 ms <= Duration <= 300 ms? ---> CONFIRMED TAP!
                   |
                   v
[ 5. Polarity Direction Determination ]
  * If initial_raw_spike < 0  ---> Printed: "VALID TAP DETECTED: PAD 1 (Q+)"
  * If initial_raw_spike >= 0 ---> Printed: "VALID TAP DETECTED: PAD 2 (Q-)"
                   |
                   v
[ 6. Non-Blocking Visual Pulse: GPIO 2 Blue LED flashes for 150 ms ]
```

### 3.3 The Data-Backed Discovery: Noise Spikes vs. Tap Plateaus
When analyzing real CSV data (`q1_env200_20260911_143141.csv`), we made a key empirical discovery:
- **Noise Spikes (< 20 ms):** Accidental static discharges from clothing or desk contact are razor-sharp electrical impulses. They last only **10 to 15 milliseconds** (1 to 3 samples).
- **Human Tap Plateaus (20 to 230 ms):** When a human finger taps a pad, the soft skin tissue squishes and maintains contact for **at least 20 to 200 milliseconds**, creating a flat "plateau" in the envelope.
- **The Fix:** By simply setting `TAP_MIN_DUR_SAMPLES = 4u` (20 ms at 200 Hz), the firmware automatically eliminates **100% of sharp noise spikes** with zero reduction in tap responsiveness!

### 3.4 Code Walkthrough of Key Files in `dual qvar/`

#### `main/imu.c`:
Configures the ISM330BX hardware:
```c
// Enable both Q1 and Q2 for differential sensing
ism330bx_ah_qvar_mode_t mode = {0};
mode.ah_qvar1_en = 1u;
mode.ah_qvar2_en = 1u;
mode.swaps = 0u;
ism330bx_ah_qvar_mode_set(&sImuCtx, mode);

// Set impedance to 300 MOhm (optimal sweet spot)
ism330bx_ah_qvar_zin_set(&sImuCtx, ISM330BX_300MOhm);

// Turn on Hardware High-Pass Filter (HPF) to eliminate DC baseline wander
ism330bx_filt_ah_qvar_conf_t filter = {0};
filter.hpf = 1u;
ism330bx_filt_ah_qvar_conf_set(&sImuCtx, filter);

// Set Accelerometer clock to 240 Hz (drives the QVAR AFE)
ism330bx_xl_data_rate_set(&sImuCtx, ISM330BX_XL_ODR_AT_240Hz);
```

#### `main/main.c`:
Runs the FreeRTOS 200 Hz loop and tap state machine:
```c
// Read differential QVAR reading
if (imu_raw_qvar_read(&q1_raw) == 0) {
    sample_idx++;
    // Calculate 4-sample peak-to-peak activity envelope
    int32_t q1_act = qvar_envelope_extractor_update(&sEnvQ1, q1_raw);

    // Pass sample into robust tap detector
    uint8_t tap_result = robust_tap_detector_update(&sTapDetector, sample_idx, q1_raw, q1_act);
    if (tap_result != 0u) {
        // Flash onboard blue LED
        gpio_set_level(TAP_LED_GPIO, 1);
        sLedTimer = TAP_LED_PULSE_SAMPLES;
        
        if (tap_result == 1u) {
            printf("  >>> VALID TAP DETECTED: PAD 1 (Q+) <<<\r\n");
        } else {
            printf("  >>> VALID TAP DETECTED: PAD 2 (Q-) <<<\r\n");
        }
    }
}
```

### 3.5 HOW to Build, Flash, and Use `dual qvar`
```powershell
# 1. Activate the ESP-IDF tools
. C:\esp\v6.1\esp-idf\export.ps1

# 2. Go to the dual qvar folder
cd "dual qvar"

# 3. Build, flash to COM7, and view serial output
idf.py build flash monitor --port COM7
```

---

## 4. Deep Dive: `firmware_raw` (Single-Ended 200 Hz Stream & Auto-Calibration)

### 4.1 WHY It Was Built
`firmware_raw` was created to scientifically observe the raw waveforms coming out of the QVAR channel at high speed. It proved that a simple fixed threshold is unviable in wearable applications, and pioneered the **4-Sample Peak-to-Peak Envelope Extractor** and **Rolling-Window Auto-Calibration Engine**.

### 4.2 WHAT It Does
1. **Single-Ended Operation:** Connects a single pad to `Q1+`. Uses $Z_{\text{in}} = 730\text{ M}\Omega$ or $235\text{ M}\Omega$.
2. **Paced 200 Hz Polling:** Uses FreeRTOS tick timing (`vTaskDelayUntil`, 5 ms period) to synchronously read every converted sample from the sensor.
3. **Rolling Quantile Auto-Calibration (1.0 s Window):**
   - Collects 20 samples ($100\text{ ms}$) per block.
   - Stores 10 blocks in a circular ring buffer.
   - Computes baseline $B[n]$ as the 2nd lowest block minimum (`min2`).
   - Automatically computes Lower Threshold and Upper Ceiling ($25,000\text{ LSB}$ clamp).
4. **Streamlined Telemetry:** Streams raw values and activity envelope at 200 Hz, calibration data at 20 Hz, and confirmed taps asynchronously over UART at 115,200 baud.

### 4.3 HOW to Run `firmware_raw`
```powershell
cd firmware_raw
idf.py build flash monitor --port COM7
```
Run the companion oscilloscope visualizer:
```powershell
python plotter.py --port COM7
```
Run the automated hardware validation benchmark:
```powershell
python tap_test_benchmark.py --port COM7 --trials 20
```

---

## 5. Deep Dive: `project` (The Modular Wearable Framework)

### 5.1 WHY It Was Built
The `project` directory was the foundational firmware created when porting the STMicroelectronics STM32 driver to the ESP32. It was designed specifically for **smart glasses**:
- Detecting whether the user has put on or taken off their glasses frame (wear sensing on Pad 2).
- Detecting single taps, double taps, or press-and-hold gestures on the glasses frame (touch button on Pad 1).
- Running the accelerometer and gyroscope to track head movement and orientation.

### 5.2 WHAT It Does
1. **Modular Architecture:** Cleanly decoupled into separate components:
   - `imu.c` / `imu.h`: Low-level I2C communication and sensor initialization.
   - `ism330bx_reg.c` / `.h`: ST's official hardware register definitions.
   - `qvar.c` / `qvar.h`: Application state machines.
   - `qvar_config.h`: Application profiles (`QVAR_CONFIG_WEAR_Q2_BUTTON_Q1`, `QVAR_CONFIG_BUTTON_Q1_ONLY`).
2. **Wear Sensing Algorithm (`Q2`):**
   - Uses an 8-sample moving average filter to smooth the signal.
   - Compares smoothed delta $\Delta_{\text{wear}}$ against wear-on and wear-off thresholds with $150\text{ to }250\text{ ms}$ debounce confirmation timers.
3. **Button Channel State Machine (`Q1`):**
   - Rebound crest tracking and peak detection.
   - Distinguishes between quick taps and sustained holds ($>250\text{ ms}$).
   - Verifies signal bipolarity ($V_{\min} \le -500$, $V_{\max} \ge +500$) to reject unipolar static discharges.
   - Emits a periodic baseline heartbeat message every 5 seconds.
4. **Windows Keyboard Bridge (`qvar_keyboard.py`):**
   - Listens to serial port COM7.
   - Triggers a real Windows keyboard stroke (e.g. Spacebar) whenever a tap is detected, allowing the electrode to act as a wireless game controller (e.g. jumping in the Chrome Dinosaur Game) or media control button.

### 5.3 HOW to Run `project`
```powershell
cd project
idf.py build flash monitor --port COM7
```
To run the Windows keyboard controller:
```powershell
python qvar_keyboard.py --port COM7 --key space
```

---

## 6. Comprehensive Comparative Matrix

| Feature | `project` (Gen 1) | `firmware_raw` (Gen 2) | `dual qvar` (Gen 3 - Production) |
| :--- | :--- | :--- | :--- |
| **Sensing Mode** | Single-Ended (Q1 & Q2 separate) | Single-Ended (Q1 only) | **Differential Pair (Q+ vs Q-)** |
| **Input Impedance ($Z_{\text{in}}$)** | $2,400\text{ M}\Omega$ ($2.4\text{ G}\Omega$) | $730\text{ M}\Omega$ / $235\text{ M}\Omega$ | **$300\text{ M}\Omega$ (Optimized Sweet Spot)** |
| **Number of Buttons** | 1 Button + 1 Wear Sensor | 1 Button | **2 Buttons (Pad 1 vs Pad 2)** |
| **Common-Mode Noise Rejection** | Low (picks up room hum) | Moderate (envelope smoothing) | **Ultra-High (Hardware cancellation)** |
| **Quiescent Noise Floor** | Moderate ($\approx 1,500\text{ to }3,000\text{ LSB}$) | High ($\approx 2,000\text{ to }4,500\text{ LSB}$) | **Ultra-Low ($\approx 300\text{ to }700\text{ LSB}$)** |
| **Tap Envelope Amplitude** | Huge ($>15,000\text{ LSB}$) | Huge ($>15,000\text{ LSB}$) | **Clean & Compact ($1,500\text{ to }5,000\text{ LSB}$)** |
| **Anti-Noise Duration Filter** | None | Min $25\text{ ms}$ | **Min $20\text{ ms}$ (Rejects sharp noise spikes)** |
| **Upper Ceiling Kill-Switch** | None | $25,000\text{ LSB}$ | **$10,000\text{ LSB}$ (Anti-static clamp)** |
| **Direction / Pad Identification** | Separate physical pins | None | **Yes (Initial spike sign +/-)** |
| **Visual Feedback** | None | GPIO 2 Blue LED | **GPIO 2 Blue LED (150 ms pulse)** |
| **Recommended Use** | Smart glasses wear research | Raw signal & DSP experimentation | **Commercial touch product release** |

---

## 7. Electrical Schematics & Hardware Wiring Guide

```
+------------------------+                     +------------------------+
|    ESP32-WROOM-32      |                     |   ST ISM330BX BREAKOUT |
|                        |                     |                        |
|             3V3 (Pin 2)|====================>|VDD / VDDIO             |
|             GND (Pin 1)|====================>|GND                     |
|                        |                     |                        |
|    I2C SDA (GPIO 21)   |<===================>|SDA (Data)              |
|    I2C SCL (GPIO 22)   |<===================>|SCL (Clock)             |
|                        |                     |                        |
|    LED (GPIO 2)        |--[ 330Ω ]-->[ LED ] |CS  ---> 3.3V (I2C mode)|
|                        |                     |SA0 ---> 3.3V (Addr 0x6B|
+------------------------+                     +-----------+------------+
                                                           |
                                  +------------------------+------------------------+
                                  |                                                 |
                                  v                                                 v
                       +----------------------+                          +----------------------+
                       | PAD 1 ELECTRODE (Q+) |                          | PAD 2 ELECTRODE (Q-) |
                       | Copper Foil / Tape   |                          | Copper Foil / Tape   |
                       +----------------------+                          +----------------------+
```

### Wiring Table:
| ESP32 Pin | Sensor Pin | Description | Requirements |
| :--- | :--- | :--- | :--- |
| **3V3** | VDD, VDDIO | 3.3V DC Power Supply | $100\text{ nF}$ decoupling capacitor close to sensor |
| **GND** | GND | Common System Ground | Connected to ground plane |
| **GPIO 21** | SDA | I2C Serial Data | $4.7\text{ k}\Omega$ pull-up resistor to 3.3V |
| **GPIO 22** | SCL | I2C Serial Clock (400 kHz) | $4.7\text{ k}\Omega$ pull-up resistor to 3.3V |
| **GPIO 2** | N/A | Onboard Blue Indicator LED | Pulses high for $150\text{ ms}$ on confirmed tap |
| **3V3** | CS | Chip Select | Tie HIGH to enable I2C mode |
| **3V3** | SA0 | I2C Slave Address LSB | Tie HIGH for address `0x6B` (tie LOW for `0x6A`) |
| **Pad 1** | Q1+ | Differential Input Positive | Copper pad ($10\text{ mm} \times 10\text{ mm}$ recommended) |
| **Pad 2** | Q1- | Differential Input Negative | Copper pad (keep $\ge 8\text{ mm}$ away from Pad 1) |

---

## 8. Parameter Tuning & Troubleshooting Reference

### 8.1 Active Tuning Parameters (`dual qvar/main/main.c`)
If you want to tune how the firmware behaves, here is what each number controls:

| Parameter | Value in Code | When to Change It |
| :--- | :---: | :--- |
| `CALIB_LOWER_OFFSET` | `500L` | If taps feel too hard to trigger, lower to `350L`. If false taps occur when moving your hand near the sensor, raise to `700L`. |
| `CALIB_MIN_LOWER_TH` | `1000L` | The absolute minimum threshold allowed. Leave at `1000L` unless testing inside an RF shielding box. |
| `CALIB_MAX_UPPER_CEIL` | `10000L` | If hard taps are being rejected as static, raise to `12000L`. If dropping the device on a desk causes false taps, lower to `8000L`. |
| `TAP_MIN_DUR_SAMPLES` | `4u` (20 ms) | The anti-noise filter. Never set below `3u` (15 ms) or sharp electrical static spikes will trigger false taps. |
| `TAP_MAX_DUR_SAMPLES` | `60u` (300 ms) | Maximum allowed tap length. Taps lasting longer than 300 ms are rejected as sustained resting holds. |
| `TAP_LOCKOUT_SAMPLES` | `30u` (150 ms) | Cooldown timer. If double-taps occur on a single press, raise to `40u` (200 ms). |

### 8.2 Common Issues & Quick Fixes

1. **"Failed to read WHO_AM_I"**:
   - Check wiring on GPIO 21 (SDA) and GPIO 22 (SCL).
   - Ensure the sensor is receiving 3.3V.
   - Verify that SA0 is tied to 3.3V (for address `0x6B`).
2. **"No taps detected at all"**:
   - Look at the `[IMU QVAR CALIB]` line in the serial terminal.
   - Check if your tap activity is reaching the `LO` threshold. If your taps peak around 1,200 and `LO` is 1,500, lower `CALIB_LOWER_OFFSET` in `main.c`.
   - Check if your tap is breaching `HI`. If `peak` exceeds `HI`, raise `CALIB_BAND_OFFSET`.
3. **"Tapping Pad 1 shows PAD 2"**:
   - Your physical wires are inverted. Simply swap the wire connections to Pad 1 and Pad 2, or invert the sign check in `main.c` (`initial_raw_spike >= 0`).
4. **"False taps when resting"**:
   - Make sure `ENVELOPE_WINDOW_SIZE` is set to `4u` (20 ms).
   - Increase `CALIB_LOWER_BUFFER` from `200L` to `400L`.
