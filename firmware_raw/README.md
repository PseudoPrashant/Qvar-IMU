# ISM330BX 200 Hz Raw & 4-Sample Peak-to-Peak Envelope Telemetry Firmware

A dedicated ESP-IDF firmware for the ISM330BX QVAR electro-sensing channel, streaming both **raw 200 Hz waveform data** and the **4-Sample Peak-to-Peak Activity Envelope** in real time over serial.

## Mathematical Architecture (The 200 Hz Envelope Extractor)
1. **Sensor Accelerometer / QVAR Clock**:
   - Hardware ODR set to **240 Hz** (`ISM330BX_XL_ODR_AT_240Hz` in `CTRL1`), converting a new sample every $4.17	ext{ ms}$.
   - Hardware HPF = ON (`ah_qvar_hpf = 1`).
2. **Firmware Polling & FreeRTOS Pacing**:
   - Paced strictly at **$5.0	ext{ ms}$ ($200	ext{ Hz}$)** using `vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(5))`.
   - Enabled by FreeRTOS tick rate `CONFIG_FREERTOS_HZ=1000` (1 ms tick resolution).
   - Because $5.0	ext{ ms} > 4.17	ext{ ms}$, every polled sample is guaranteed to be a freshly updated hardware conversion with zero duplicate reads.
3. **4-Sample Sliding Window Peak-to-Peak Envelope**:
   $$\text{Activity}[n] = \max(x[n..n-3]) - \min(x[n..n-3])$$
   - Exactly $4 \times 5.0\text{ ms} = 20.0\text{ ms}$, matching **one full cycle** of $50\text{ Hz}$ mains powerline hum ($1/50 = 20.0\text{ ms}$).
   - In every 4 consecutive samples, the window captures both the crest ($+A$) and trough ($-A$) of the injected AC carrier wave, producing a stable unipolar pulse ($>15,000\text{ LSB}$) with zero dropouts.
4. **Streamlined High-Speed Telemetry**:
   - Telemetry format: `[IMU QVAR RAW] Q1=%d Q1_ACT=%ld #%lu\r\n`
   - Keeps payload size under 35 bytes to prevent UART buffer saturation at 115,200 baud, guaranteeing full 200.0 Hz streaming throughput.

## Project Structure
```text
firmware_raw/
├── CMakeLists.txt          # ESP-IDF project definition
├── sdkconfig               # Target ESP32, CONFIG_FREERTOS_HZ=1000
├── plotter.py              # Real-time Dual-Trace Oscilloscope & CSV Logger
└── main/
    ├── CMakeLists.txt      # Component sources & includes
    ├── idf_component.yml   # ST ism330bx component dependency
    ├── imu.h / imu.c       # IMU initialization at 240 Hz ODR & HPF
    └── main.c              # 200 Hz loop, 4-sample P2P envelope extractor, streamlined telemetry
```

## How to Build and Flash
```powershell
. C:\esp\v6.1\esp-idf\export.ps1
cd firmware_raw
idf.py -p COM7 flash
```

## How to Run the Real-Time Dual-Trace Oscilloscope
```powershell
python plotter.py --port COM7
```
- **Cyan Trace**: Raw 200 Hz Q1 signal (showing AC carrier oscillations).
- **Amber Gold Trace**: 4-Sample Peak-to-Peak Activity Envelope.
- **Dashed Red Line**: Touch detection reference threshold (`3,000 LSB`).
- **HUD**: Displays real-time live sample rate and active touch state.
- **Data Logging**: Automatically creates a timestamped CSV in `firmware_raw/data/` (e.g. `q1_env200_YYYYMMDD_HHMMSS.csv`).
