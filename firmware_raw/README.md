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

---

## Time-Gated Band-Pass State Machine Tap Detection Engine

The firmware includes a time-gated, band-pass state machine designed to detect finger taps while completely rejecting environmental disturbances and handling movement.

### Five Tuned Parameters (at 200 Hz / 5 ms per sample):
1. **Lower Threshold (`3,500 LSB`)**: The floor required to clear background noise and open a candidate Tap Event.
2. **Upper Ceiling (`5,500 LSB`)**: The kill-switch. If Activity breaks $5,500	ext{ LSB}$ at any point while an event is open, it is permanently flagged **Invalid** (handling transients / movement).
3. **Bridge Timer (`50 ms` / 10 samples)**: Prevents phase troughs from splitting a single tap. If Activity dips below $3,500	ext{ LSB}$, the event remains open for up to $50	ext{ ms}$ to bridge.
4. **Minimum Duration (`40 ms` / 8 samples)**: Rejects microscopic static pops. If total event duration is $< 40	ext{ ms}$, it is deleted.
5. **Lockout Cooldown (`150 ms` / 30 samples)**: Enforces a $150	ext{ ms}$ refractory period after tap confirmation to suppress release chatter.

### Execution Flow:
- When $	ext{Activity} > 3,500	ext{ LSB}$, opens Tap Event (`Valid = true`).
- If $	ext{Activity} > 5,500	ext{ LSB}$ at any point, permanently sets `Valid = false`.
- When $	ext{Activity} < 3,500	ext{ LSB}$ for 10 consecutive samples ($50	ext{ ms}$), the event closes.
- If `Valid == true` and active duration $\ge 40	ext{ ms}$, emits `[IMU QVAR TAP] #N dur=... peak=...`.
- Enforces $150	ext{ ms}$ lockout before next event can begin.

### Onboard LED Visual Tap Indicator (GPIO 2):
Whenever a valid tap event is confirmed, the firmware pulses the onboard blue LED on `GPIO 2` (`GPIO_NUM_2`) for $150	ext{ ms}$ (30 samples @ 200 Hz). The pulse is handled non-blockingly within the periodic sampling loop with zero jitter or latency to QVAR sensor reading.
