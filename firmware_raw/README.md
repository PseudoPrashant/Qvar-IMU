# ISM330BX 240 Hz Raw & 5-Sample Peak-to-Peak Envelope Telemetry Firmware

A dedicated ESP-IDF firmware for the ISM330BX QVAR electro-sensing channel, streaming both **raw 240 Hz AC waveform data** and the **5-Sample Peak-to-Peak Activity Envelope** in real time over serial.

## Mathematical Architecture (The 240 Hz Envelope Extractor)
1. **Sensor Accelerometer / QVAR Clock**:
   - Configured to **240 Hz** (`ISM330BX_XL_ODR_AT_240Hz` in `CTRL1`).
   - Hardware HPF = ON (`ah_qvar_hpf = 1`).
2. **Pacing**:
   - Paced at **4 ms (250 Hz)** with `CONFIG_FREERTOS_HZ=1000`.
3. **The 5-Sample Peak-to-Peak Window**:
   $$\text{Activity}[n] = \max(x[n..n-4]) - \min(x[n..n-4])$$
   - Spans exactly **$5 \times 4\text{ ms} = 20.0\text{ ms}$**—the exact full-period duration of a $50\text{ Hz}$ AC powerline wave ($1/50 = 20.0\text{ ms}$) and $>1$ full period of $60\text{ Hz}$ ($16.7\text{ ms}$).
   - **Phase Invariance**: Every 5 consecutive samples are mathematically guaranteed to capture both the positive apex ($+A$) and negative valley ($-A$) of the injected AC wave.
   - **Unipolar Solid Output**: Always positive ($\ge 0$). Transforms alternating, chaotic touch oscillations into a massive, solid, unipolar pulse ($> 14,000\text{ LSB}$) with zero dropouts and zero zero-crossing dips.

## Dual-Channel Telemetry Format
```text
[IMU QVAR RAW] Q1=<raw_lsb> Q1_ACT=<activity_lsb> Q2=NA (<raw_mv> mV, act: <act_mv> mV) #<sample_count>
```

## Live Dual-Trace Oscilloscope & CSV Logger
Run `plotter.py` to visualize both signals simultaneously:
```powershell
python plotter.py --port COM7
```
- **Cyan Trace**: Raw Q1 signal (240 Hz AC wave).
- **Amber Gold Trace**: 5-Sample Peak-to-Peak Activity Envelope.
- **Red Dashed Line**: Touch detection threshold guide (default: $3,500\text{ LSB}$).
- **Automatic CSV Logging**: Records both `q1_raw` and `q1_activity` columns to timestamped CSV files in `data/`.

## Build & Flash
```powershell
# 1. Activate ESP-IDF
. C:\esp\v6.1\esp-idf\export.ps1

# 2. Navigate to firmware_raw directory
cd C:\Users\prash\OneDrive\Desktop\IMU\IMU\firmware_raw

# 3. Build
idf.py build

# 4. Flash to COM7
idf.py -p COM7 flash
```
