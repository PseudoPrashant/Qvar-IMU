# ISM330BX 200 Hz Raw & 4-Stage Robust Tap Detector Firmware

A dedicated ESP-IDF firmware for the ISM330BX QVAR electro-sensing channel, streaming **raw 200 Hz waveform data**, the **4-Sample Peak-to-Peak Activity Envelope**, and **4-Stage Robust Tap Events** in real time over serial.

## Mathematical Architecture (The 200 Hz Robust Tap Engine)
1. **Sensor Accelerometer / QVAR Clock**:
   - Hardware ODR set to **240 Hz** (`ISM330BX_XL_ODR_AT_240Hz` in `CTRL1`), converting a new sample every $4.17\text{ ms}$.
   - Hardware HPF = ON (`ah_qvar_hpf = 1`).
2. **Firmware Polling & FreeRTOS Pacing**:
   - Paced strictly at **$5.0\text{ ms}$ ($200\text{ Hz}$)** using `vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(5))`.
   - Enabled by FreeRTOS tick rate `CONFIG_FREERTOS_HZ=1000` (1 ms tick resolution).
   - Because $5.0\text{ ms} > 4.17\text{ ms}$, every polled sample is guaranteed to be a freshly updated hardware conversion with zero duplicate reads.
3. **4-Sample Sliding Window Peak-to-Peak Envelope**:
   $$\text{Activity}[n] = \max(x[n..n-3]) - \min(x[n..n-3])$$
   - Exactly $4 \times 5.0\text{ ms} = 20.0\text{ ms}$, matching **one full cycle** of $50\text{ Hz}$ mains powerline hum ($1/50 = 20.0\text{ ms}$).
   - In every 4 consecutive samples, the window captures both the crest ($+A$) and trough ($-A$) of the injected AC carrier wave, producing a stable unipolar pulse.
4. **4-Stage Robust Tap Detection State Machine**:
   - **Contact Threshold**: $T_{\text{press}} = 3,300\text{ LSB}$ ($pprox 42.3\text{ mV}$)
   - **Release Threshold**: $T_{\text{release}} = 2,650\text{ LSB}$ ($pprox 34.0\text{ mV}$)
   - **Duration Gating**: $10\text{ ms} \le T \le 250\text{ ms}$ (2 to 50 samples; rejects long holds/disturbances).
   - **Bipolarity Verification**: Rejects unipolar electrostatic DC discharges by enforcing $V_{\min} \le -500\text{ LSB}$ AND $V_{\max} \ge +500\text{ LSB}$ across the contact window.
   - **Refractory Cooldown & Disturbance Squelch**: $125\text{ ms}$ cooldown after valid taps. Enters a $250\text{ ms}$ squelch lockout on chattering or DC blasts until $75\text{ ms}$ of continuous quiet baseline is observed.
5. **Streamlined High-Speed Telemetry**:
   - Telemetry format: `[IMU QVAR RAW] Q1=%d Q1_ACT=%ld #%lu\r\n`
   - Tap event format: `[IMU QVAR TAP] #%lu dur=%lu ms peak=%ld LSB\r\n`
   - Keeps payload compact to prevent UART buffer saturation at 115,200 baud, guaranteeing full 200.0 Hz streaming throughput.

## Project Structure
```text
firmware_raw/
├── CMakeLists.txt          # ESP-IDF project definition
├── sdkconfig               # Target ESP32, CONFIG_FREERTOS_HZ=1000
├── plotter.py              # Real-time Dual-Trace Oscilloscope & Tap Event Visualizer
└── main/
    ├── CMakeLists.txt      # Component sources & includes
    ├── idf_component.yml   # ST ism330bx component dependency
    ├── imu.h / imu.c       # IMU initialization at 240 Hz ODR & HPF
    └── main.c              # 200 Hz loop, P2P envelope extractor, 4-stage tap detector
```

## How to Build and Flash
```powershell
. C:\esp\v6.1\esp-idf\export.ps1
cd firmware_raw
idf.py -p COM7 flash
```

## How to Run the Real-Time Dual-Trace Oscilloscope & Tap Visualizer
```powershell
python plotter.py --port COM7
```
- **Cyan Trace**: Raw 200 Hz Q1 signal (showing AC carrier oscillations).
- **Amber Gold Trace**: 4-Sample Peak-to-Peak Activity Envelope.
- **Yellow Circle Markers**: Dropped on confirmed tap events at the exact peak amplitude.
- **Dashed Red Line**: Tap detection reference threshold (`3,300 LSB`).
- **HUD Banner**: Real-time sample rate, live tap counter, and flashing `*** TAP DETECTED #N ***` notifications.
- **Data Logging**: Automatically creates a timestamped CSV in `firmware_raw/data/` (e.g. `q1_env200_YYYYMMDD_HHMMSS.csv`).

# ISM330BX 200 Hz Raw & 4-Stage Robust Tap Detector Firmware

A dedicated ESP-IDF firmware for the ISM330BX QVAR electro-sensing channel, streaming **raw 200 Hz waveform data**, the **4-Sample Peak-to-Peak Activity Envelope**, and **4-Stage Robust Tap Events** in real time over serial.

## Mathematical Architecture (The 200 Hz Robust Tap Engine)
1. **Sensor Accelerometer / QVAR Clock**:
   - Hardware ODR set to **240 Hz** (`ISM330BX_XL_ODR_AT_240Hz` in `CTRL1`), converting a new sample every $4.17\text{ ms}$.
   - Hardware HPF = ON (`ah_qvar_hpf = 1`).
2. **Firmware Polling & FreeRTOS Pacing**:
   - Paced strictly at **$5.0\text{ ms}$ ($200\text{ Hz}$)** using `vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(5))`.
   - Enabled by FreeRTOS tick rate `CONFIG_FREERTOS_HZ=1000` (1 ms tick resolution).
   - Because $5.0\text{ ms} > 4.17\text{ ms}$, every polled sample is guaranteed to be a freshly updated hardware conversion with zero duplicate reads.
3. **4-Sample Sliding Window Peak-to-Peak Envelope**:
   $$\text{Activity}[n] = \max(x[n..n-3]) - \min(x[n..n-3])$$
   - Exactly $4 \times 5.0\text{ ms} = 20.0\text{ ms}$, matching **one full cycle** of $50\text{ Hz}$ mains powerline hum ($1/50 = 20.0\text{ ms}$).
   - In every 4 consecutive samples, the window captures both the crest ($+A$) and trough ($-A$) of the injected AC carrier wave, producing a stable unipolar pulse.
4. **4-Stage Robust Tap Detection State Machine**:
   - **Contact Threshold**: $T_{\text{press}} = 3,300\text{ LSB}$ ($ pprox 42.3\text{ mV}$)
   - **Release Threshold**: $T_{\text{release}} = 2,650\text{ LSB}$ ($ pprox 34.0\text{ mV}$)
   - **Duration Gating**: $10\text{ ms} \le T \le 250\text{ ms}$ (2 to 50 samples; rejects long holds/disturbances).
   - **Bipolarity Verification**: Rejects unipolar electrostatic DC discharges by enforcing $V_{\min} \le -500\text{ LSB}$ AND $V_{\max} \ge +500\text{ LSB}$ across the contact window.
   - **Refractory Cooldown & Disturbance Squelch**: $125\text{ ms}$ cooldown after valid taps. Enters a $250\text{ ms}$ squelch lockout on chattering or DC blasts until $75\text{ ms}$ of continuous quiet baseline is observed.
5. **Streamlined High-Speed Telemetry**:
   - Telemetry format: `[IMU QVAR RAW] Q1=%d Q1_ACT=%ld #%lu\r\n`
   - Tap event format: `[IMU QVAR TAP] #%lu dur=%lu ms peak=%ld LSB\r\n`
   - Keeps payload compact to prevent UART buffer saturation at 115,200 baud, guaranteeing full 200.0 Hz streaming throughput.

## Project Structure
```text
firmware_raw/
├── CMakeLists.txt          # ESP-IDF project definition
├── sdkconfig               # Target ESP32, CONFIG_FREERTOS_HZ=1000
├── plotter.py              # Real-time Dual-Trace Oscilloscope & Tap Event Visualizer
└── main/
    ├── CMakeLists.txt      # Component sources & includes
    ├── idf_component.yml   # ST ism330bx component dependency
    ├── imu.h / imu.c       # IMU initialization at 240 Hz ODR & HPF
    └── main.c              # 200 Hz loop, P2P envelope extractor, 4-stage tap detector
```

## How to Build and Flash
```powershell
. C:\esp\v6.1\esp-idf\export.ps1
cd firmware_raw
idf.py -p COM7 flash
```

## How to Run the Real-Time Dual-Trace Oscilloscope & Tap Visualizer
```powershell
python plotter.py --port COM7
```
- **Cyan Trace**: Raw 200 Hz Q1 signal (showing AC carrier oscillations).
- **Amber Gold Trace**: 4-Sample Peak-to-Peak Activity Envelope.
- **Yellow Circle Markers**: Dropped on confirmed tap events at the exact peak amplitude.
- **Dashed Red Line**: Tap detection reference threshold (`3,300 LSB`).
- **HUD Banner**: Real-time sample rate, live tap counter, and flashing `*** TAP DETECTED #N ***` notifications.
- **Data Logging**: Automatically creates a timestamped CSV in `firmware_raw/data/` (e.g. `q1_env200_YYYYMMDD_HHMMSS.csv`).

---

## Time-Gated Band-Pass State Machine with In-Firmware Auto-Calibration

The firmware features an autonomous in-firmware auto-calibration engine that dynamically computes the **Lower Threshold (Floor)** and **Upper Ceiling (Kill-Switch)** based on real-time ambient noise statistics, eliminating manual tuning across different environments, grounding setups, or user contact conditions.

### Real-Time Rolling-Window Auto-Calibration Architecture

1. **Continuous Rolling-Window Noise Floor Tracking (1.0-Second Window)**:
   - Divides incoming samples into blocks of 20 samples ($100\text{ ms}$ @ $200\text{ Hz}$).
   - Continuously computes the minimum activity within the active block.
   - Every $100\text{ ms}$, commits the block minimum to a circular ring buffer of 10 blocks ($1.0\text{ second}$ rolling memory for fast adaptation).
   - Extracts the real-time noise floor $B[n]$ as the **2nd lowest block minimum** across the ring buffer:
     - **Immune to Startup Transients**: Initial power-on capacitor settling artifacts ($\approx 25,000\text{ LSB}$) automatically flush out of the window as live data arrives.
     - **Immune to Taps**: Finger taps last only $40\text{ to }165\text{ ms}$ (affecting at most 1–2 blocks), leaving the remaining 8–9 blocks tracking the true quiescent floor.
     - **Fast Dynamic Responsiveness**: When ambient noise changes (e.g. user touches glasses, changes grounding, or moves rooms), the baseline adjusts to the new floor within $\approx 1.0\text{ second}$.

2. **Dynamic Threshold Formulations with Dedicated Noise Headroom Buffer**:
   $$T_{\text{lower}} = \max\Big(2500,\; B + 1,800 + 400 + 0.28 \cdot B\Big)$$
   $$T_{\text{upper}} = \min\Big(25000,\; T_{\text{lower}} + 3,000 + 0.25 \cdot B\Big)$$
   - `CALIB_LOWER_OFFSET = 1800L` (base separation from baseline).
   - `CALIB_LOWER_BUFFER = 400L` (dedicated noise headroom buffer eliminating false positives from 50 Hz powerline hum envelope beating).
   - `CALIB_LOWER_RATIO = 0.28f` (proportional scaling tuned to preserve sensitivity for gentle taps).
   - In low-noise environments ($B \approx 1,000\text{ LSB}$): $T_{\text{lower}} \approx 3,480\text{ LSB}$, $T_{\text{upper}} \approx 6,730\text{ LSB}$.
   - In high-noise environments ($B \approx 4,300\text{ LSB}$): $T_{\text{lower}} \approx 7,704\text{ LSB}$, $T_{\text{upper}} \approx 11,780\text{ LSB}$.

3. **20 Hz Live Calibration Telemetry**:
   - Every 10 samples ($50\text{ ms}$ / $20\text{ Hz}$), firmware streams live calibration values:
     `[IMU QVAR CALIB] BASE=... LO=... HI=...`
   - `plotter.py` visualizes the yellow lower floor and red upper ceiling horizontal lines moving and adapting in real time on the oscilloscope.

4. **Time-Gated Band-Pass Execution Flow with Min & Max Duration Guard**:
   - When $\text{Activity} > T_{\text{lower}}$, opens candidate Tap Event (`Valid = true`).
   - If $\text{Activity} > T_{\text{upper}}$ or event duration exceeds $300\text{ ms}$ (`TAP_MAX_DUR_SAMPLES = 60u`), permanently sets `Valid = false` (kill-switch rejecting heavy handling, ESD, prolonged touch holds, and baseline transition ramps).
   - When $\text{Activity} < T_{\text{lower}}$ for 10 consecutive samples ($50\text{ ms}$ Bridge Timer), the event closes.
   - If `Valid == true` and active duration satisfies $25\text{ ms} \le T \le 300\text{ ms}$ ($5 \le \text{samples} \le 60$), emits confirmed tap event:
     `[IMU QVAR TAP] #N dur=... ms peak=... LSB base=... th=... ceil=...`
   - Enforces $150\text{ ms}$ lockout refractory period.

### Onboard LED Visual Tap Indicator (GPIO 2):
Whenever a valid tap event is confirmed, the firmware pulses the onboard blue LED on `GPIO 2` (`GPIO_NUM_2`) for $150\text{ ms}$ (30 samples @ 200 Hz). The pulse is handled non-blockingly within the periodic sampling loop with zero jitter or latency to QVAR sensor reading.

---

## Interactive Tap Accuracy Benchmark Protocol (`tap_test_benchmark.py`)

A standalone guided hardware-in-the-loop validation script to quantify tap detection accuracy, false positives, false negatives, and human reaction latency across randomized trials.

### Running the Benchmark:
```powershell
python tap_test_benchmark.py --port COM7
```
Optional flags:
- `--trials 20`: Number of randomized prompt trials (default: 20).
- `--min-wait 1.5`: Minimum quiet wait before cue in seconds (default: 1.5).
- `--max-wait 4.0`: Maximum quiet wait before cue in seconds (default: 4.0).
- `--timeout 1.5`: Maximum allowed response window to tap after cue in seconds (default: 1.5).

### Test Flow:
1. **Quiescent Desk Baseline**: 5-second resting log to verify resting noise floor and audit for desk false positives.
2. **Hand Pickup & Hold Settling**: 3-second hold settling time to allow auto-calibration to adapt to the hand-held baseline.
3. **Randomized Tap Trials (20 Trials)**:
   - Randomized quiet wait (1.5s - 4.0s) between trials: Continuously audits for unprompted **False Positives**.
   - Visual flash banner + auditory tone prompts the user to tap.
   - 1.5s response window: Detects **True Positives (Hits)** with latency, peak activity, and duration; or flags **False Negatives (Misses)**.
4. **Return Sensor to Desk**: 5-second release settling log to audit release stability.
5. **Accuracy Report & Exports**:
   - Prints full metrics table to console (Hits, Misses, False Positives, Sensitivity/Recall, Precision, Reaction Latency).
   - Generates summary report: `data/tap_benchmark_report_YYYYMMDD_HHMMSS.txt`
   - Generates phase-tagged 200 Hz continuous CSV: `data/tap_benchmark_YYYYMMDD_HHMMSS.csv`

