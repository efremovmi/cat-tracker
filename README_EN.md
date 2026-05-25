# Cat Activity Tracker (XIAO + LIS2DH12)

Cat activity tracker firmware based on a Seeed XIAO board and the LIS2DH12 accelerometer.
The device estimates activity, steps, distance, jump count, stores hourly stats, and exposes data over BLE.

## What The System Does

- Runs in two sensor modes:
  - `WAKE`: low-power idle mode
  - `ACTIVE`: accelerometer sampling and motion analysis
- Builds movement segments and filters noise.
- Calculates:
  - `steps` (estimated from stride length)
  - `meters` (estimated from segment duration + motion intensity)
  - `activePoints`
  - `jumps`
- Stores persistent state in Flash (`LittleFS`) with hourly buckets.
- Returns stats over BLE (`GET_STATS`).

## Runtime Flow

1. **Boot**
- Initialize LED, battery ADC control, filesystem, accelerometer, and BLE.
- Load saved state or start from defaults.

2. **WAKE Mode**
- LIS2DH12 runs in low-power mode and raises `INT1` on activity.
- On interrupt, firmware switches to `ACTIVE`.

3. **ACTIVE Mode**
- Read acceleration samples.
- Compute dynamic component (`dyn`) against filtered gravity baseline.
- Confirm segment start (`pending start` -> `movement ongoing`).
- Count jumps using `zDyn` threshold + hysteresis + cooldown.
- Finish segment on silence (`MOTION_END_GAP_MS`) or max segment time.

4. **Metrics Update**
- Validate segment quality filters.
- Estimate speed from average dynamics.
- Compute distance, steps, activity points, jumps.
- Save to current hourly bucket and total counters.

5. **Background Tasks**
- Battery measurement through divider + MOSFET switch.
- Periodic Flash save.
- BLE commands: `TIME_SYNC`, `GET_STATS`, `RESET_STATS`, `DISCONNECT`, `DEBUG_CHANGE`.

## Wiring (Text Specification)

### XIAO <-> LIS2DH12

| From | To | Notes |
|---|---|---|
| `XIAO 3V3` | `LIS2DH12 VCC` | Sensor power |
| `XIAO GND` | `LIS2DH12 GND` | Common ground |
| `XIAO D4` | `LIS2DH12 SDA` | I2C data |
| `XIAO D5` | `LIS2DH12 SCL` | I2C clock |
| `XIAO D2` | `LIS2DH12 INT1` | Sensor interrupt |
| `XIAO 3V3` | `LIS2DH12 CS` | Force I2C mode |
| `XIAO GND` | `LIS2DH12 SA0` | I2C address = `0x18` |

### Battery + Measurement Node

| From | To | Notes |
|---|---|---|
| `CR2032 +` | `System power rail` | Main supply |
| `CR2032 -` | `GND` | Common ground |
| `CR2032 +` | `R1 (1M)` | Divider top resistor |
| `R1 (other pin)` | `VBAT_SENSE` | Sense node |
| `VBAT_SENSE` | `XIAO A0` | ADC input |
| `VBAT_SENSE` | `R2 (1M)` | Divider bottom resistor |
| `R2 (other pin)` | `IRLML6344 DRAIN (pin 3)` | Divider low-side switching |
| `IRLML6344 SOURCE (pin 2)` | `GND` | MOSFET source |
| `XIAO D1` | `100k` | Gate drive series resistor |
| `100k (other pin)` | `IRLML6344 GATE (pin 1)` | Gate control |
| `IRLML6344 GATE (pin 1)` | `1M` | Gate pull-down |
| `1M (other pin)` | `GND` | Keep MOSFET off by default |

## Bill Of Materials (BOM)

| Component | Qty | Notes |
|---|---:|---|
| Seeed XIAO (3.3V, BLE-capable) | 1 | Main controller |
| LIS2DH12 breakout/module | 1 | Accelerometer, I2C + INT1 |
| CR2032 battery | 1 | Power source |
| CR2032 holder | 1 | Optional depending on assembly |
| IRLML6344 (N-MOSFET) | 1 | Divider ground switch |
| Resistor `1M` (R1) | 1 | Divider top |
| Resistor `1M` (R2) | 1 | Divider bottom |
| Resistor `100k` | 1 | Gate series resistor |
| Resistor `1M` | 1 | Gate pull-down |
| Wires / assembly materials | 1 set | Interconnects |

## Firmware File

- Main sketch: `cat-tracket-final.ino`

## Notes

- Current motion constants are tuned for a cat profile.
- For correct wall-clock hourly buckets, send BLE command: `TIME_SYNC:<unix_ts>`.
- Verify IRLML6344 pinout against the exact package/datasheet before soldering.
