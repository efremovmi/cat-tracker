# Cat Activity Tracker (XIAO nRF52840 sence + LIS2DH12)

A cat activity tracker project based on XIAO and the LIS2DH12 accelerometer.
The firmware tracks activity, steps (estimated), distance, jumps, stores hourly statistics, and sends data over BLE.

The project is inspired by nihilism and follows several goals:
- Minimalism
- High autonomy
- Cat safety
- Avoiding an excessive number of unnecessary features
- Enjoying the process
- Helping study the cat's health

## What the system does

- Runs in two sensor modes:
  - `WAKE`: low power, waiting for motion
  - `ACTIVE`: reading samples and motion analysis
- Builds movement segments and filters noise.
- Estimates:
  - `steps` (via stride length)
  - `meters` (via segment duration and speed estimation)
  - `activePoints`
  - `jumps`
- Saves state to Flash (`LittleFS`) and keeps hourly buckets.
- Returns stats via BLE command `GET_STATS`.

## Runtime logic (process)

1. **Device startup**
- Initializes LED, battery ADC, filesystem, accelerometer, and BLE.
- Loads saved state or starts with default parameters.

2. **WAKE mode**
- LIS2DH12 runs at lower frequency and generates `INT1` on activity.
- On IRQ trigger, the device switches to `ACTIVE`.

3. **ACTIVE mode**
- Reads acceleration samples.
- Computes dynamic component (`dyn`) relative to filtered gravity.
- Confirms movement segment start (`pending start` -> `movement ongoing`).
- Counts jumps using `zDyn` with hysteresis and cooldown.
- Finishes segment on silence (`MOTION_END_GAP_MS`) or by `MAX_MOVEMENT_SEGMENT_MS`.

4. **Segment metrics calculation**
- Applies segment quality filters (minimum duration / motion density).
- Estimates speed from average `avgDyn`.
- Calculates meters and steps.
- Writes data to current hourly bucket and total counters.

5. **Background tasks**
- Periodic battery measurement via divider + MOSFET switch.
- Periodic state save to Flash.
- BLE command handling (`TIME_SYNC`, `GET_STATS`, `RESET_STATS`, `DISCONNECT`, `DEBUG_CHANGE`).

## Wiring (single SVG diagram)

Component wiring diagram (SVG)
![Wiring diagram](https://raw.githubusercontent.com/efremovmi/cat-tracker/main/schema.svg)

The diagram is made as one sheet: controller, accelerometer, battery, divider, and MOSFET switch node.

## Components table (BOM)

| Component | Qty | Note |
|---|---:|---|
| Seeed XIAO (3.3V, BLE-compatible) | 1 | Main controller |
| LIS2DH12 breakout/module | 1 | Accelerometer, I2C, INT1 |
| CR2032 battery | 1 | Power |
| CR2032 holder | 1 | Power |
| IRLML6344 (N-MOSFET) | 1 | Switch for battery measurement node |
| 1M resistor (R1) | 1 | Divider top resistor |
| 1M resistor (R2) | 1 | Divider bottom resistor |
| 100k resistor | 1 | Series resistor to Gate |
| 1M resistor | 1 | Gate pull-down to GND |
| Wires/assembly | 1 set | Connections |

## Firmware file

- Main firmware: `./cat-tracker.ino`
- BLE desktop client:
- Files with 3 models: `./3d models`
- Project photos: `./images`

## Notes

- Current firmware coefficients are tuned for a cat (not a human).
- The most honest metric is activity.
- For correct hourly statistics after BLE connection, send `TIME_SYNC:<unix_ts>`.

## Plans

- Collect statistics and build metrics
- Measure differences when the cat is played with vs. not given attention
