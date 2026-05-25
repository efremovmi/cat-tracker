#include <ArduinoBLE.h>
#include <Wire.h>

#include "mbed.h"
#include "FlashIAPBlockDevice.h"
#include "LittleFileSystem.h"

#include <stdio.h>
#include <stdarg.h>
#include <string.h>
#include <math.h>

using namespace mbed;

// ===================== Settings =====================

// ----- Debug -----
bool debugLogEnabled = true;

// ----- LED indication -----
static const bool HAS_RGB_LED = true;
static const uint32_t BLE_WAIT_BLINK_INTERVAL_MS = 10UL * 1000UL;
static const uint32_t LED_BLINK_ON_MS = 120UL;
static const uint32_t LED_BLINK_OFF_MS = 120UL;
static const uint8_t  DISCONNECT_RED_BLINKS = 5;

// ----- BLE -----
static const char* DEVICE_NAME  = "PET_TRACKER";
static const char* SERVICE_UUID = "19B10000-E8F2-537E-4F6C-D104768A1214";
static const char* RX_UUID      = "19B10001-E8F2-537E-4F6C-D104768A1214";
static const char* TX_UUID      = "19B10002-E8F2-537E-4F6C-D104768A1214";

static const uint32_t BLE_ADVERTISE_WINDOW_MS = 30UL * 1000UL;
static const uint16_t BLE_ADV_INTERVAL_UNITS  = 3200;

// ----- Storage -----
static const uint32_t SAVE_INTERVAL_MS = 60UL * 60UL * 1000UL;
static const uint32_t BATTERY_MEASURE_INTERVAL_MS = 60UL * 60UL * 1000UL;
static const size_t   MAX_HOURLY_BUCKETS = 48;
static const uint32_t PERSIST_MAGIC = 0x50535450;
static const uint32_t PERSIST_VERSION = 20;

static const uint32_t FLASH_FS_BASE = 0x80000;
static const uint32_t FLASH_FS_SIZE = 64 * 1024;
static const char* STATS_FILE_PATH = "/littlefs/pet_steps.bin";

// ----- Battery measurement -----
static const int BATTERY_ADC_PIN     = A0;
static const int BATTERY_MEAS_EN_PIN = D1;

static const float BAT_R1_OHMS = 1000000.0f;
static const float BAT_R2_OHMS = 1000000.0f;

static const int   ADC_MAX_COUNTS      = 4095;
static const float ADC_REFERENCE_VOLTS = 3.0f;

static const float CR2032_FULL_V  = 3.3f;
static const float CR2032_EMPTY_V = 2.0f;

// ----- LIS2DH12 -----
static const uint8_t LIS_ADDR     = 0x18;
static const int     LIS_INT1_PIN = D2;

// ----- Activity score -----
static const float SEG_MOTION_PER_ACTIVE_POINT = 100.0f;

// ----- Human model -----
static const float DEFAULT_STRIDE_METERS = 0.26f;

// ----- Speed estimation from motion intensity -----
static const float DEFAULT_MIN_SPEED_MPS = 0.35f;
static const float DEFAULT_MAX_SPEED_MPS = 1.80f;
static const float DEFAULT_DYN_SPEED_MIN = 18.0f;
static const float DEFAULT_DYN_SPEED_MAX = 95.0f;

// ----- Power / sensor state machine -----
static const uint32_t ACTIVE_WINDOW_MS = 1500;
static const uint32_t QUIET_RETURN_MS  = 5000; //  когда идти спать

// ----- Motion detection tuning -----
static const uint8_t ACTIVE_ODR_25HZ_CTRL1 = 0x3F;
static const uint8_t WAKE_ODR_10HZ_CTRL1   = 0x2F;

static const int32_t GRAV_LP_SHIFT = 4; // Отвечает за то, как быстро оценка гравитации подстраивается.
static const int32_t BASE_LP_SHIFT = 7; // Как быстро подстраивается “база шума”.
static const int32_t MOTION_MIN_RAW = 12; // абсолютно нижний порог
static const int32_t MOTION_MARGIN  = 5; // насколько порог выше фонового base
static const int32_t STRONG_MOTION_EXTRA = 4; // Продление active-окна.
static const int32_t JUMP_Z_DYN_THRESHOLD = 52; // порог прыжка по Z
static const int32_t JUMP_Z_DYN_HYSTERESIS = 8; // защита от повторного срабатывания на пике
static const uint32_t JUMP_COUNT_COOLDOWN_MS = 450; // минимальный интервал между засчитанными прыжками

// Если нет motion-образцов дольше этого интервала, считаем сегмент движения завершённым
static const uint32_t MOTION_END_GAP_MS = 2500;

// Минимальная длительность сегмента
static const uint32_t MIN_MOVEMENT_SEGMENT_MS = 1000; // 1 секунда
// Максимальная длительность сегмента
static const uint32_t MAX_MOVEMENT_SEGMENT_MS = 60UL * 1000UL; // 1 минута

// ===== Защита от шума / ударов =====
// Нужно столько подряд motion-сэмплов, чтобы начать сегмент
static const uint8_t MOTION_START_CONFIRM_COUNT = 2;

// Если между motion-сэмплами слишком большая пауза — сбрасываем кандидат на старт
static const uint32_t MOTION_START_CONFIRM_GAP_MS = 800;

// Минимум "настоящих" motion-сэмплов в сегменте
static const uint32_t MIN_SEGMENT_MOTION_SAMPLES = 4;

// Минимальная доля motion-сэмплов среди всех сэмплов сегмента
static const float MIN_SEGMENT_MOTION_RATIO = 0.10f;

// Минимальный средний dyn сегмента, чтобы отсечь слабый шум
static const float MIN_SEGMENT_AVG_DYN = 12.0f;

// ===================== REGISTERS LIS2DH12 =====================

static const uint8_t REG_WHO_AM_I       = 0x0F;
static const uint8_t REG_CTRL_REG1      = 0x20;
static const uint8_t REG_CTRL_REG2      = 0x21;
static const uint8_t REG_CTRL_REG3      = 0x22;
static const uint8_t REG_CTRL_REG4      = 0x23;
static const uint8_t REG_CTRL_REG5      = 0x24;
static const uint8_t REG_CTRL_REG6      = 0x25;
static const uint8_t REG_REFERENCE      = 0x26;
static const uint8_t REG_STATUS_REG     = 0x27;
static const uint8_t REG_OUT_X_L        = 0x28;
static const uint8_t REG_INT1_CFG       = 0x30;
static const uint8_t REG_INT1_SRC       = 0x31;
static const uint8_t REG_INT1_THS       = 0x32;
static const uint8_t REG_INT1_DURATION  = 0x33;

// ===================== BLE =====================

BLEService petService(SERVICE_UUID);
BLEStringCharacteristic rxChar(RX_UUID, BLEWrite, 120);
BLEStringCharacteristic txChar(TX_UUID, BLERead | BLENotify, 1024);

// ===================== FILESYSTEM =====================

FlashIAPBlockDevice flashBD(FLASH_FS_BASE, FLASH_FS_SIZE);
mbed::LittleFileSystem fs("littlefs");
bool fsReady = false;

// ===================== TYPES =====================

struct HourBucket {
  uint32_t hourKey;
  uint32_t steps;
  float meters;
  float activePoints;
  uint32_t jumps;
};

struct PersistedState {
  uint32_t magic;
  uint32_t version;
  uint32_t timeSynced;
  int64_t  unixTimeBase;
  uint32_t millisAtSync;

  uint8_t debugLogEnabled;

  float strideMeters;
  float minSpeedMps;
  float maxSpeedMps;
  float dynSpeedMin;
  float dynSpeedMax;

  uint32_t totalSteps;
  float totalMeters;
  float totalActivePoints;
  uint32_t totalJumps;

  float batteryVoltage;
  int32_t batteryPercent;
  uint32_t lastBatteryMeasureMillis;

  uint32_t bucketCount;
  HourBucket buckets[MAX_HOURLY_BUCKETS];
};

struct Sample3 {
  int16_t x;
  int16_t y;
  int16_t z;
};

enum SensorMode {
  SENSOR_WAKE = 0,
  SENSOR_ACTIVE = 1
};

struct MotionDetectorState {
  bool initialized = false;

  int32_t gx = 0;
  int32_t gy = 0;
  int32_t gz = 0;

  int32_t base = 0;

  bool movementOngoing = false;
  uint32_t movementStartMs = 0;
  uint32_t lastMotionMs = 0;

  uint64_t segmentDynSum = 0;
  uint32_t segmentSampleCount = 0;
  uint32_t segmentMotionSampleCount = 0;
  uint32_t segmentMotionJump = 0;
  int32_t segmentDynPeak = 0;
  int32_t lastDynZ = 0;
  bool jumpPeakLatched = false;
  uint32_t lastJumpCountMs = 0;

  uint8_t pendingStartCount = 0;
  uint32_t pendingStartFirstMs = 0;
  uint32_t pendingStartLastMs = 0;
};

// ===================== GLOBALS =====================

PersistedState state;
MotionDetectorState motionDet;

BLEDevice currentCentral;

bool bleStarted = false;
bool bleAdvertising = false;
bool clientConnected = false;
bool disconnectRequested = false;
bool dirty = false;

volatile bool lisIntFlag = false;
volatile uint32_t lisIntCount = 0;
volatile bool saveTimerFlag = false;

uint32_t bootMillis = 0;
uint32_t activeUntilMs = 0;
uint32_t lastStrongMotionMs = 0;

SensorMode sensorMode = SENSOR_WAKE;

LowPowerTimeout saveTimer;

uint32_t lastBleWaitBlinkMillis = 0;
bool pendingDisconnectRedBlinkSequence = false;

// ===================== LOG =====================

void logLine(const String& s) {
  if (debugLogEnabled) Serial.println(s);
}

// ===================== SPEED =====================

float clampFloat(float v, float lo, float hi) {
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}

float estimateSpeedFromAvgDyn(float avgDyn) {
  float dynMin = state.dynSpeedMin;
  float dynMax = state.dynSpeedMax;

  if (dynMax <= dynMin) {
    return state.minSpeedMps;
  }

  float t = (avgDyn - dynMin) / (dynMax - dynMin);
  t = clampFloat(t, 0.0f, 1.0f);

  float speed = state.minSpeedMps + t * (state.maxSpeedMps - state.minSpeedMps);
  return speed;
}

// ===================== LED =====================

void ledOffAll() {
  if (!HAS_RGB_LED) return;

  pinMode(LED_RED, OUTPUT);
  pinMode(LED_GREEN, OUTPUT);
  pinMode(LED_BLUE, OUTPUT);

  digitalWrite(LED_RED, HIGH);
  digitalWrite(LED_GREEN, HIGH);
  digitalWrite(LED_BLUE, HIGH);
}

void blinkGreenOnce() {
  if (!HAS_RGB_LED) return;

  ledOffAll();
  digitalWrite(LED_GREEN, LOW);
  delay(LED_BLINK_ON_MS);
  digitalWrite(LED_GREEN, HIGH);
}

void blinkRedSequence(uint8_t times) {
  if (!HAS_RGB_LED) return;

  ledOffAll();
  for (uint8_t i = 0; i < times; i++) {
    digitalWrite(LED_RED, LOW);
    delay(LED_BLINK_ON_MS);
    digitalWrite(LED_RED, HIGH);
    if (i + 1 < times) {
      delay(LED_BLINK_OFF_MS);
    }
  }
}

// ===================== TIME/STATS =====================

uint32_t getBootHourKey() {
  return (uint32_t)(millis() / 3600000UL);
}

uint32_t getCurrentHourKey() {
  if (state.timeSynced) {
    int64_t nowUnix = state.unixTimeBase + ((int64_t)(millis() - state.millisAtSync) / 1000LL);
    if (nowUnix < 0) nowUnix = 0;
    return (uint32_t)(nowUnix / 3600LL);
  }
  return getBootHourKey();
}

String formatHourLabel(uint32_t hourKey) {
  return state.timeSynced ? ("UNIX_HOUR_" + String(hourKey))
                          : ("BOOT_HOUR_" + String(hourKey));
}

void clearStatsInRam() {
  memset(state.buckets, 0, sizeof(state.buckets));
  state.bucketCount = 0;
  state.totalSteps = 0;
  state.totalMeters = 0.0f;
  state.totalActivePoints = 0.0f;
  state.totalJumps = 0;
  dirty = true;
  logLine("[STATS] Cleared");
}

void addMovementToCurrentHour(float metersToAdd, float activePointsToAdd, uint32_t jumpsToAdd) {
  if (metersToAdd <= 0.0f && activePointsToAdd <= 0.0f && jumpsToAdd == 0) return;

  uint32_t stepsToAdd = (uint32_t)roundf(metersToAdd / state.strideMeters);
  const uint32_t hourKey = getCurrentHourKey();

  state.totalMeters += metersToAdd;
  state.totalSteps += stepsToAdd;
  state.totalActivePoints += activePointsToAdd;
  state.totalJumps += jumpsToAdd;

  for (uint32_t i = 0; i < state.bucketCount; i++) {
    if (state.buckets[i].hourKey == hourKey) {
      state.buckets[i].meters += metersToAdd;
      state.buckets[i].steps += stepsToAdd;
      state.buckets[i].activePoints += activePointsToAdd;
      state.buckets[i].jumps += jumpsToAdd;
      dirty = true;

      logLine("[STATS] +" + String(metersToAdd, 2) + " m, +" +
              String(stepsToAdd) + " steps, +" +
              String(activePointsToAdd, 2) + " active points, +" +
              String(jumpsToAdd) + " jumps in " + formatHourLabel(hourKey));
      return;
    }
  }

  if (state.bucketCount < MAX_HOURLY_BUCKETS) {
    state.buckets[state.bucketCount].hourKey = hourKey;
    state.buckets[state.bucketCount].steps = stepsToAdd;
    state.buckets[state.bucketCount].meters = metersToAdd;
    state.buckets[state.bucketCount].activePoints = activePointsToAdd;
    state.buckets[state.bucketCount].jumps = jumpsToAdd;
    state.bucketCount++;
  } else {
    memmove(&state.buckets[0], &state.buckets[1], sizeof(HourBucket) * (MAX_HOURLY_BUCKETS - 1));
    state.buckets[MAX_HOURLY_BUCKETS - 1].hourKey = hourKey;
    state.buckets[MAX_HOURLY_BUCKETS - 1].steps = stepsToAdd;
    state.buckets[MAX_HOURLY_BUCKETS - 1].meters = metersToAdd;
    state.buckets[MAX_HOURLY_BUCKETS - 1].activePoints = activePointsToAdd;
    state.buckets[MAX_HOURLY_BUCKETS - 1].jumps = jumpsToAdd;
  }

  dirty = true;
  logLine("[STATS] New bucket " + formatHourLabel(hourKey) +
          ", meters=" + String(metersToAdd, 2) +
          ", steps=" + String(stepsToAdd) +
          ", activePoints=" + String(activePointsToAdd, 2) +
          ", jumps=" + String(jumpsToAdd));
}

String buildStatsPayload() {
  static const size_t MAX_STATS_PAYLOAD_LEN = 900;
  char out[MAX_STATS_PAYLOAD_LEN];
  out[0] = '\0';
  size_t used = 0;

  auto appendRaw = [&](const char* s) -> bool {
    size_t n = strlen(s);
    if (used + n >= MAX_STATS_PAYLOAD_LEN) return false;
    memcpy(out + used, s, n);
    used += n;
    out[used] = '\0';
    return true;
  };

  auto appendf = [&](const char* fmt, ...) -> bool {
    if (used >= MAX_STATS_PAYLOAD_LEN - 1) return false;
    va_list args;
    va_start(args, fmt);
    int written = vsnprintf(out + used, MAX_STATS_PAYLOAD_LEN - used, fmt, args);
    va_end(args);
    if (written < 0) return false;
    if ((size_t)written >= (MAX_STATS_PAYLOAD_LEN - used)) {
      used = MAX_STATS_PAYLOAD_LEN - 1;
      out[used] = '\0';
      return false;
    }
    used += (size_t)written;
    return true;
  };

  updateBatteryInRam();

  bool ok = true;
  ok &= appendRaw("OK:STATS\n");
  ok &= appendf("TIME_MODE=%s\n", state.timeSynced ? "SYNCED" : "BOOT_RELATIVE");
  ok &= appendf("TOTAL_STEPS=%lu\n", (unsigned long)state.totalSteps);
  ok &= appendf("TOTAL_METERS=%.2f\n", (double)state.totalMeters);
  ok &= appendf("TOTAL_ACTIVE_POINTS=%.2f\n", (double)state.totalActivePoints);
  ok &= appendf("TOTAL_JUMPS=%lu\n", (unsigned long)state.totalJumps);
  ok &= appendf("SEGMENT_JUMPS=%lu\n", (unsigned long)motionDet.segmentMotionJump);
  ok &= appendf("BATTERY_V=%.3f\n", (double)state.batteryVoltage);
  ok &= appendf("BATTERY_PERCENT=%ld\n", (long)state.batteryPercent);
  ok &= appendf("SENSOR_MODE=%d\n", (int)sensorMode);
  ok &= appendf("BUCKETS=%lu\n", (unsigned long)state.bucketCount);

  for (uint32_t i = 0; i < state.bucketCount; i++) {
    if (!ok) break;
    String hourLabel = formatHourLabel(state.buckets[i].hourKey);
    ok &= appendf("%s:STEPS=%lu,METERS=%.2f,ACTIVE_POINTS=%.2f,JUMPS=%lu\n",
                  hourLabel.c_str(),
                  (unsigned long)state.buckets[i].steps,
                  (double)state.buckets[i].meters,
                  (double)state.buckets[i].activePoints,
                  (unsigned long)state.buckets[i].jumps);
  }

  if (!ok) {
    if (used + strlen("TRUNCATED=1\n") < MAX_STATS_PAYLOAD_LEN) {
      appendRaw("TRUNCATED=1\n");
    } else if (MAX_STATS_PAYLOAD_LEN >= 2) {
      out[MAX_STATS_PAYLOAD_LEN - 2] = '\n';
      out[MAX_STATS_PAYLOAD_LEN - 1] = '\0';
    }
  }

  return String(out);
}

// ===================== FLASH =====================

bool initFilesystem() {
  int err = fs.mount(&flashBD);
  if (err) {
    err = fs.reformat(&flashBD);
    if (err) {
      logLine("[FS] Reformat failed, code=" + String(err));
      return false;
    }
    err = fs.mount(&flashBD);
    if (err) {
      logLine("[FS] Mount after format failed, code=" + String(err));
      return false;
    }
  }
  return true;
}

bool saveStateToFlash() {
  if (!fsReady) {
    logLine("[FS] save skipped: FS not ready");
    return false;
  }

  FILE* f = fopen(STATS_FILE_PATH, "wb");
  if (!f) {
    logLine("[FS] Failed to open file for write");
    return false;
  }

  size_t written = fwrite(&state, 1, sizeof(state), f);
  fclose(f);

  if (written != sizeof(state)) {
    logLine("[FS] Incomplete write");
    return false;
  }

  dirty = false;
  return true;
}

bool loadStateFromFlash() {
  if (!fsReady) return false;

  FILE* f = fopen(STATS_FILE_PATH, "rb");
  if (!f) {
    logLine("[FS] No saved state");
    return false;
  }

  PersistedState loaded;
  size_t rd = fread(&loaded, 1, sizeof(loaded), f);
  fclose(f);

  if (rd != sizeof(loaded)) {
    logLine("[FS] Saved state size mismatch");
    return false;
  }

  if (loaded.magic != PERSIST_MAGIC || loaded.version != PERSIST_VERSION) {
    logLine("[FS] Version mismatch, start fresh");
    return false;
  }

  state = loaded;
  if (state.bucketCount > MAX_HOURLY_BUCKETS) {
    state.bucketCount = MAX_HOURLY_BUCKETS;
  }

  debugLogEnabled = (state.debugLogEnabled != 0);

  return true;
}

void initFreshState() {
  memset(&state, 0, sizeof(state));
  state.magic = PERSIST_MAGIC;
  state.version = PERSIST_VERSION;
  state.debugLogEnabled = 1;
  state.strideMeters = DEFAULT_STRIDE_METERS;
  state.minSpeedMps = DEFAULT_MIN_SPEED_MPS;
  state.maxSpeedMps = DEFAULT_MAX_SPEED_MPS;
  state.dynSpeedMin = DEFAULT_DYN_SPEED_MIN;
  state.dynSpeedMax = DEFAULT_DYN_SPEED_MAX;
  state.totalActivePoints = 0.0f;
  state.totalJumps = 0;
  state.batteryVoltage = 0.0f;
  state.batteryPercent = 0;
  state.lastBatteryMeasureMillis = 0;

  debugLogEnabled = true;
  dirty = false;
}

// ===================== BATTERY =====================

int batteryPercentFromVoltage(float v) {
  if (v >= CR2032_FULL_V) return 100;
  if (v <= CR2032_EMPTY_V) return 0;

  float p = (v - CR2032_EMPTY_V) / (CR2032_FULL_V - CR2032_EMPTY_V);
  int percent = (int)(p * 100.0f + 0.5f);

  if (percent < 0) percent = 0;
  if (percent > 100) percent = 100;
  return percent;
}

float readBatteryVoltageOnce() {
  digitalWrite(BATTERY_MEAS_EN_PIN, HIGH);

  analogRead(BATTERY_ADC_PIN);
  analogRead(BATTERY_ADC_PIN);

  uint32_t sum = 0;
  const int samples = 8;
  for (int i = 0; i < samples; i++) {
    sum += analogRead(BATTERY_ADC_PIN);
  }

  digitalWrite(BATTERY_MEAS_EN_PIN, LOW);

  float raw = (float)sum / (float)samples;
  float vAdc = (raw / (float)ADC_MAX_COUNTS) * ADC_REFERENCE_VOLTS;
  float vBat = vAdc * ((BAT_R1_OHMS + BAT_R2_OHMS) / BAT_R2_OHMS);

  return vBat;
}

void updateBatteryInRam() {
  float vBat = readBatteryVoltageOnce();
  int pct = batteryPercentFromVoltage(vBat);

  state.batteryVoltage = vBat;
  state.batteryPercent = pct;
  state.lastBatteryMeasureMillis = millis();
  dirty = true;

}

void periodicBatteryMeasureIfNeeded() {
  uint32_t now = millis();

  if (state.lastBatteryMeasureMillis == 0 ||
      (now - state.lastBatteryMeasureMillis) >= BATTERY_MEASURE_INTERVAL_MS) {
    updateBatteryInRam();
  }
}

// ===================== LIS LOW LEVEL =====================

bool lisWriteReg(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(LIS_ADDR);
  Wire.write(reg);
  Wire.write(value);
  return Wire.endTransmission() == 0;
}

bool lisReadReg(uint8_t reg, uint8_t &value) {
  Wire.beginTransmission(LIS_ADDR);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;

  if (Wire.requestFrom((int)LIS_ADDR, 1) != 1) return false;
  value = Wire.read();
  return true;
}

bool lisReadRegs(uint8_t reg, uint8_t* dst, size_t len) {
  Wire.beginTransmission(LIS_ADDR);
  Wire.write(reg | 0x80);
  if (Wire.endTransmission(false) != 0) return false;

  size_t got = Wire.requestFrom((int)LIS_ADDR, (int)len);
  if (got != len) return false;

  for (size_t i = 0; i < len; i++) dst[i] = Wire.read();
  return true;
}

// ===================== TIMER / ISR =====================

void onSaveTimerISR() {
  saveTimerFlag = true;
}

void armSaveTimer() {
  saveTimer.detach();
  saveTimer.attach(&onSaveTimerISR, std::chrono::milliseconds(SAVE_INTERVAL_MS));
}

void lisIntISR() {
  lisIntFlag = true;
  lisIntCount++;
}

// ===================== MOTION DETECTOR =====================

void resetMotionDetector() {
  motionDet = MotionDetectorState{};
}

int32_t dynFromSampleAndUpdateFilters(const Sample3& s) {
  const int32_t x = s.x;
  const int32_t y = s.y;
  const int32_t z = s.z;

  if (!motionDet.initialized) {
    motionDet.gx = x;
    motionDet.gy = y;
    motionDet.gz = z;
    motionDet.initialized = true;
  }

  motionDet.gx += (x - motionDet.gx) >> GRAV_LP_SHIFT;
  motionDet.gy += (y - motionDet.gy) >> GRAV_LP_SHIFT;
  motionDet.gz += (z - motionDet.gz) >> GRAV_LP_SHIFT;

  const int32_t dx = x - motionDet.gx;
  const int32_t dy = y - motionDet.gy;
  const int32_t dz = z - motionDet.gz;

  const int32_t adx = abs(dx);
  const int32_t ady = abs(dy);
  const int32_t adz = abs(dz);
  motionDet.lastDynZ = adz;

  const int32_t dyn = max(adx, max(ady, adz));

  motionDet.base += (dyn - motionDet.base) >> BASE_LP_SHIFT;
  return dyn;
}

int32_t getMotionThreshold() {
  return (motionDet.base + MOTION_MARGIN > MOTION_MIN_RAW)
           ? (motionDet.base + MOTION_MARGIN)
           : MOTION_MIN_RAW;
}

void updateSegmentJumpFromZDyn() {
  if (motionDet.lastDynZ >= JUMP_Z_DYN_THRESHOLD) {
    if (!motionDet.jumpPeakLatched) {
      uint32_t now = millis();
      if (motionDet.lastJumpCountMs == 0 ||
          (uint32_t)(now - motionDet.lastJumpCountMs) >= JUMP_COUNT_COOLDOWN_MS) {
        motionDet.segmentMotionJump++;
        motionDet.lastJumpCountMs = now;
      }
      motionDet.jumpPeakLatched = true;
    }
    return;
  }

  if (motionDet.lastDynZ <= (JUMP_Z_DYN_THRESHOLD - JUMP_Z_DYN_HYSTERESIS)) {
    motionDet.jumpPeakLatched = false;
  }
}

void restartMovementSegmentNow(uint32_t now, int32_t dyn) {
  motionDet.movementOngoing = true;
  motionDet.movementStartMs = now;
  motionDet.lastMotionMs = now;

  motionDet.segmentDynSum = (uint64_t)dyn;
  motionDet.segmentSampleCount = 1;
  motionDet.segmentMotionSampleCount = 1;
  motionDet.segmentMotionJump = 0;
  motionDet.segmentDynPeak = dyn;
  motionDet.jumpPeakLatched = false;
  updateSegmentJumpFromZDyn();

  motionDet.pendingStartCount = 0;
  motionDet.pendingStartFirstMs = 0;
  motionDet.pendingStartLastMs = 0;

}

void startConfirmedMovementSegment(uint32_t now) {
  motionDet.movementOngoing = true;
  motionDet.movementStartMs = motionDet.pendingStartFirstMs ? motionDet.pendingStartFirstMs : now;
  motionDet.lastMotionMs = now;
  motionDet.segmentDynSum = 0;
  motionDet.segmentSampleCount = 0;
  motionDet.segmentMotionSampleCount = 0;
  motionDet.segmentMotionJump = 0;
  motionDet.segmentDynPeak = 0;
  motionDet.jumpPeakLatched = false;

  motionDet.pendingStartCount = 0;
  motionDet.pendingStartFirstMs = 0;
  motionDet.pendingStartLastMs = 0;

}

void resetPendingStart() {
  motionDet.pendingStartCount = 0;
  motionDet.pendingStartFirstMs = 0;
  motionDet.pendingStartLastMs = 0;
}

void finalizeMovementSegment(uint32_t segmentEndMs) {
  if (!motionDet.movementOngoing) return;
  if (segmentEndMs <= motionDet.movementStartMs) {
    uint32_t jumpsToCredit = motionDet.segmentMotionJump;
    motionDet.movementOngoing = false;
    addMovementToCurrentHour(0.0f, 0.0f, jumpsToCredit);
    if (jumpsToCredit > 0) {
      logLine("[MOTION] Jumps credited outside segment: +" + String(jumpsToCredit));
    }
    motionDet.segmentDynSum = 0;
    motionDet.segmentSampleCount = 0;
    motionDet.segmentMotionSampleCount = 0;
    motionDet.segmentMotionJump = 0;
    motionDet.segmentDynPeak = 0;
    motionDet.jumpPeakLatched = false;
    return;
  }

  uint32_t durationMs = segmentEndMs - motionDet.movementStartMs;
  motionDet.movementOngoing = false;

  if (durationMs < MIN_MOVEMENT_SEGMENT_MS ||
      motionDet.segmentSampleCount == 0 ||
      motionDet.segmentMotionSampleCount < MIN_SEGMENT_MOTION_SAMPLES) {
    uint32_t jumpsToCredit = motionDet.segmentMotionJump;
    logLine("[MOTION] Segment rejected: too short or too few motion samples");
    addMovementToCurrentHour(0.0f, 0.0f, jumpsToCredit);
    if (jumpsToCredit > 0) {
      logLine("[MOTION] Jumps credited outside segment: +" + String(jumpsToCredit));
    }
    motionDet.segmentDynSum = 0;
    motionDet.segmentSampleCount = 0;
    motionDet.segmentMotionSampleCount = 0;
    motionDet.segmentMotionJump = 0;
    motionDet.segmentDynPeak = 0;
    motionDet.jumpPeakLatched = false;
    return;
  }

  float motionRatio = (float)motionDet.segmentMotionSampleCount / (float)motionDet.segmentSampleCount;
  float avgDyn = (float)motionDet.segmentDynSum / (float)motionDet.segmentMotionSampleCount;

  if (motionRatio < MIN_SEGMENT_MOTION_RATIO) {
    uint32_t jumpsToCredit = motionDet.segmentMotionJump;
    logLine("[MOTION] Segment rejected: low motion ratio = " + String(motionRatio, 3));
    addMovementToCurrentHour(0.0f, 0.0f, jumpsToCredit);
    if (jumpsToCredit > 0) {
      logLine("[MOTION] Jumps credited outside segment: +" + String(jumpsToCredit));
    }
    motionDet.segmentDynSum = 0;
    motionDet.segmentSampleCount = 0;
    motionDet.segmentMotionSampleCount = 0;
    motionDet.segmentMotionJump = 0;
    motionDet.segmentDynPeak = 0;
    motionDet.jumpPeakLatched = false;
    return;
  }

  if (avgDyn < MIN_SEGMENT_AVG_DYN) {
    uint32_t jumpsToCredit = motionDet.segmentMotionJump;
    logLine("[MOTION] Segment rejected: low avgDyn = " + String(avgDyn, 2));
    addMovementToCurrentHour(0.0f, 0.0f, jumpsToCredit);
    if (jumpsToCredit > 0) {
      logLine("[MOTION] Jumps credited outside segment: +" + String(jumpsToCredit));
    }
    motionDet.segmentDynSum = 0;
    motionDet.segmentSampleCount = 0;
    motionDet.segmentMotionSampleCount = 0;
    motionDet.segmentMotionJump = 0;
    motionDet.segmentDynPeak = 0;
    motionDet.jumpPeakLatched = false;
    return;
  }

  float durationSec = durationMs / 1000.0f;
  float speedMps = estimateSpeedFromAvgDyn(avgDyn);
  float meters = durationSec * speedMps;
  float activePoints = (float)motionDet.segmentMotionSampleCount / SEG_MOTION_PER_ACTIVE_POINT;

  const int32_t metersCm = (int32_t)lroundf(meters * 100.0f);

  logLine("[MOTION] Segment finished: durMs=" + String(durationMs) +
          ", metersCm=" + String(metersCm) +
          ", jumps=" + String(motionDet.segmentMotionJump));

  addMovementToCurrentHour(meters, activePoints, motionDet.segmentMotionJump);

  motionDet.segmentDynSum = 0;
  motionDet.segmentSampleCount = 0;
  motionDet.segmentMotionSampleCount = 0;
  motionDet.segmentMotionJump = 0;
  motionDet.segmentDynPeak = 0;
  motionDet.jumpPeakLatched = false;
}

// ===================== LIS MODES =====================

bool lisSetWakeMode() {
  if (!lisWriteReg(REG_CTRL_REG1, 0x00)) return false;
  if (!lisWriteReg(REG_CTRL_REG1, WAKE_ODR_10HZ_CTRL1)) return false;
  if (!lisWriteReg(REG_CTRL_REG2, 0x09)) return false;
  if (!lisWriteReg(REG_CTRL_REG3, 0x40)) return false;
  if (!lisWriteReg(REG_CTRL_REG4, 0x00)) return false;
  if (!lisWriteReg(REG_CTRL_REG5, 0x08)) return false;
  if (!lisWriteReg(REG_INT1_CFG, 0x2A)) return false;
  if (!lisWriteReg(REG_INT1_THS, 12)) return false;
  if (!lisWriteReg(REG_INT1_DURATION, 1)) return false;

  uint8_t tmp = 0;
  lisReadReg(REG_REFERENCE, tmp);
  lisReadReg(REG_INT1_SRC, tmp);

  sensorMode = SENSOR_WAKE;
  return true;
}

bool lisSetActiveMode() {
  if (!lisWriteReg(REG_CTRL_REG1, 0x00)) return false;
  if (!lisWriteReg(REG_CTRL_REG1, ACTIVE_ODR_25HZ_CTRL1)) return false;
  if (!lisWriteReg(REG_CTRL_REG2, 0x00)) return false;
  if (!lisWriteReg(REG_CTRL_REG3, 0x10)) return false;
  if (!lisWriteReg(REG_CTRL_REG4, 0x00)) return false;
  if (!lisWriteReg(REG_CTRL_REG5, 0x00)) return false;
  if (!lisWriteReg(REG_INT1_CFG, 0x00)) return false;

  sensorMode = SENSOR_ACTIVE;
  activeUntilMs = millis() + ACTIVE_WINDOW_MS;
  lastStrongMotionMs = millis();
  resetMotionDetector();

  return true;
}

bool initAccelerometer() {
  Wire.begin();
  delay(5);

  uint8_t who = 0;
  if (!lisReadReg(REG_WHO_AM_I, who)) {
    logLine("[ACC] WHO_AM_I read failed");
    return false;
  }

  if (who != 0x33) {
    logLine("[ACC] Unexpected WHO_AM_I");
    return false;
  }

  pinMode(LIS_INT1_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(LIS_INT1_PIN), lisIntISR, RISING);

  if (!lisSetWakeMode()) {
    logLine("[ACC] Failed to enter WAKE mode");
    return false;
  }

  return true;
}

// ===================== SAMPLE READ =====================

bool lisReadLatestSample(Sample3& s) {
  uint8_t raw[6];
  if (!lisReadRegs(REG_OUT_X_L, raw, sizeof(raw))) return false;

  int16_t x = (int16_t)((raw[1] << 8) | raw[0]);
  int16_t y = (int16_t)((raw[3] << 8) | raw[2]);
  int16_t z = (int16_t)((raw[5] << 8) | raw[4]);

  s.x = x >> 8;
  s.y = y >> 8;
  s.z = z >> 8;
  return true;
}

// ===================== SENSOR EVENT HANDLERS =====================

void processWakeInterrupt() {
  uint8_t src = 0;
  lisReadReg(REG_INT1_SRC, src);
  if (src & 0x40) {
    lisSetActiveMode();
  }
}

void processActiveDataReadyInterrupt() {
  Sample3 s;
  if (!lisReadLatestSample(s)) {
    logLine("[ACC] sample read failed");
    return;
  }

  int32_t dyn = dynFromSampleAndUpdateFilters(s);
  int32_t thr = getMotionThreshold();
  uint32_t now = millis();

  bool motionDetected = (dyn >= thr);

  if (motionDet.movementOngoing) {
    // Жесткое разбиение слишком длинного сегмента
    if ((now - motionDet.movementStartMs) >= MAX_MOVEMENT_SEGMENT_MS) {
      logLine("[MOTION] Segment split: max duration reached");
      finalizeMovementSegment(now);

      // Если на текущем сэмпле движение всё ещё есть — сразу стартуем новый сегмент
      if (motionDetected) {
        restartMovementSegmentNow(now, dyn);

        lastStrongMotionMs = now;
        activeUntilMs = now + ACTIVE_WINDOW_MS;
      }

      logLine("[MOTION] dyn=" + String(dyn) +
              ", zDyn=" + String(motionDet.lastDynZ) +
              ", thr=" + String(thr) +
              ", jumpThr=" + String(JUMP_Z_DYN_THRESHOLD) +
              ", active=" + String(motionDet.movementOngoing ? "1" : "0") +
              ", pending=" + String(motionDet.pendingStartCount) +
              ", segSamples=" + String(motionDet.segmentSampleCount) +
              ", segMotion=" + String(motionDet.segmentMotionSampleCount));
      return;
    }

    motionDet.segmentSampleCount++;
    updateSegmentJumpFromZDyn();

    if (motionDetected) {
      motionDet.lastMotionMs = now;
      motionDet.segmentMotionSampleCount++;
      motionDet.segmentDynSum += (uint64_t)dyn;

      if (dyn > motionDet.segmentDynPeak) {
        motionDet.segmentDynPeak = dyn;
      }

      lastStrongMotionMs = now;
      activeUntilMs = now + ACTIVE_WINDOW_MS;
    } else {
      if ((now - motionDet.lastMotionMs) >= MOTION_END_GAP_MS) {
        finalizeMovementSegment(motionDet.lastMotionMs);
      }

      if (dyn > (thr + STRONG_MOTION_EXTRA)) {
        lastStrongMotionMs = now;
      }
    }
  } else {
    if (motionDetected) {
      if (motionDet.pendingStartCount == 0 ||
          (now - motionDet.pendingStartLastMs) > MOTION_START_CONFIRM_GAP_MS) {
        motionDet.pendingStartCount = 1;
        motionDet.pendingStartFirstMs = now;
        motionDet.pendingStartLastMs = now;
      } else {
        motionDet.pendingStartCount++;
        motionDet.pendingStartLastMs = now;
      }

      if (motionDet.pendingStartCount >= MOTION_START_CONFIRM_COUNT) {
        startConfirmedMovementSegment(now);

        // Сразу учитываем текущий сэмпл как первый motion-сэмпл сегмента
        motionDet.segmentSampleCount = 1;
        motionDet.segmentMotionSampleCount = 1;
        motionDet.segmentDynSum = (uint64_t)dyn;
        motionDet.segmentMotionJump = 0;
        motionDet.segmentDynPeak = dyn;
        motionDet.lastMotionMs = now;
        motionDet.jumpPeakLatched = false;
        updateSegmentJumpFromZDyn();

        lastStrongMotionMs = now;
        activeUntilMs = now + ACTIVE_WINDOW_MS;
      }
    } else {
      if (motionDet.pendingStartCount > 0 &&
          (now - motionDet.pendingStartLastMs) > MOTION_START_CONFIRM_GAP_MS) {
        resetPendingStart();
      }

      if (dyn > (thr + STRONG_MOTION_EXTRA)) {
        lastStrongMotionMs = now;
      }
    }
  }

  logLine("[MOTION] dyn=" + String(dyn) +
          ", zDyn=" + String(motionDet.lastDynZ) +
          ", thr=" + String(thr) +
          ", jumpThr=" + String(JUMP_Z_DYN_THRESHOLD) +
          ", active=" + String(motionDet.movementOngoing ? "1" : "0") +
          ", pending=" + String(motionDet.pendingStartCount) +
          ", segSamples=" + String(motionDet.segmentSampleCount) +
          ", segMotion=" + String(motionDet.segmentMotionSampleCount));
}

void processLisInterrupts() {
  if (!lisIntFlag) return;

  noInterrupts();
  lisIntFlag = false;
  uint32_t count = lisIntCount;
  lisIntCount = 0;
  interrupts();

  if (count == 0) return;

  logLine("[ACC] IRQ count=" + String(count) + ", mode=" + String((int)sensorMode));

  if (sensorMode == SENSOR_WAKE) {
    processWakeInterrupt();
  } else {
    processActiveDataReadyInterrupt();
  }
}

void maybeReturnToWakeMode() {
  if (sensorMode != SENSOR_ACTIVE) return;

  const uint32_t now = millis();

  if (motionDet.movementOngoing &&
      (now - motionDet.lastMotionMs) >= MOTION_END_GAP_MS) {
    finalizeMovementSegment(motionDet.lastMotionMs);
  }

  if (motionDet.pendingStartCount > 0 &&
      (now - motionDet.pendingStartLastMs) > MOTION_START_CONFIRM_GAP_MS) {
    resetPendingStart();
  }

  if (now >= activeUntilMs && (now - lastStrongMotionMs) >= QUIET_RETURN_MS) {
    if (motionDet.movementOngoing) {
      finalizeMovementSegment(motionDet.lastMotionMs);
    }

    resetPendingStart();

    logLine("[ACC] Quiet timeout, back to WAKE mode");
    lisSetWakeMode();
  }
}

// ===================== BLE =====================

void processBleWaitLed() {
  if (!HAS_RGB_LED) return;
  if (!bleStarted || clientConnected || !bleAdvertising) return;

  uint32_t now = millis();
  if (now - lastBleWaitBlinkMillis >= BLE_WAIT_BLINK_INTERVAL_MS) {
    lastBleWaitBlinkMillis = now;
    blinkGreenOnce();
  }
}

bool startBleAdvertising() {
  if (bleStarted) return true;

  if (!BLE.begin()) {
    logLine("[BLE] BLE.begin() failed");
    return false;
  }

  BLE.setLocalName(DEVICE_NAME);
  BLE.setDeviceName(DEVICE_NAME);
  BLE.setAdvertisedService(petService);
  BLE.setAdvertisingInterval(BLE_ADV_INTERVAL_UNITS);

  petService.addCharacteristic(rxChar);
  petService.addCharacteristic(txChar);
  BLE.addService(petService);

  txChar.writeValue("READY");
  rxChar.writeValue("");

  BLE.advertise();

  bleStarted = true;
  bleAdvertising = true;

  return true;
}

void stopBleCompletely() {
  if (!bleStarted) return;

  BLE.stopAdvertise();
  BLE.end();

  bleStarted = false;
  bleAdvertising = false;
  clientConnected = false;
}

void sendResponse(const String& msg) {
  txChar.writeValue(msg);
}

void handleCommand(const String& cmdRaw) {
  String cmd = cmdRaw;
  cmd.trim();

  if (cmd.startsWith("TIME_SYNC:")) {
    String value = cmd.substring(String("TIME_SYNC:").length());
    int64_t unixTs = value.toInt();

    if (unixTs > 0) {
      state.timeSynced = 1;
      state.unixTimeBase = unixTs;
      state.millisAtSync = millis();
      dirty = true;
      sendResponse("OK:TIME_SYNC");
    } else {
      sendResponse("ERR:BAD_TIME_SYNC");
    }
    return;
  }

  if (cmd == "GET_STATS") {
    sendResponse(buildStatsPayload());
    return;
  }

  if (cmd == "RESET_STATS") {
    clearStatsInRam();
    saveStateToFlash();
    sendResponse("OK:RESET_DONE");
    return;
  }

  if (cmd == "DISCONNECT") {
    sendResponse("OK:DISCONNECTING");
    disconnectRequested = true;
    return;
  }

  if (cmd == "DEBUG_CHANGE") {
    debugLogEnabled = !debugLogEnabled;
    state.debugLogEnabled = !state.debugLogEnabled;
    dirty = true;
    saveStateToFlash();
    txChar.writeValue("OK:DEBUG_CHANGED");
    logLine("[SYS] Debug logging changed");
    return;
  }

  sendResponse("ERR:UNKNOWN_COMMAND");
}

void serviceBle() {
  if (!bleStarted) return;

  BLE.poll();

  if (!clientConnected) {
    BLEDevice central = BLE.central();
    if (central) {
      currentCentral = central;
      clientConnected = true;
    }
  }

  if (!clientConnected) return;

  if (!currentCentral.connected()) {
    clientConnected = false;
    stopBleCompletely();
    pendingDisconnectRedBlinkSequence = true;
    return;
  }

  if (rxChar.written()) {
    String cmd = rxChar.value();
    handleCommand(cmd);
  }

  if (disconnectRequested) {
    disconnectRequested = false;
    stopBleCompletely();
    pendingDisconnectRedBlinkSequence = true;
  }
}

// ===================== PERIODIC SAVE =====================

void processSaveTimer() {
  if (!saveTimerFlag) return;
  saveTimerFlag = false;

  if (dirty) {
    saveStateToFlash();
  }

  armSaveTimer();
}

// ===================== SETUP / LOOP =====================

void setup() {
  Serial.begin(115200);
  delay(1000);

  if (HAS_RGB_LED) {
    pinMode(LED_RED, OUTPUT);
    pinMode(LED_GREEN, OUTPUT);
    pinMode(LED_BLUE, OUTPUT);
    ledOffAll();
  }

  bootMillis = millis();

  pinMode(BATTERY_MEAS_EN_PIN, OUTPUT);
  digitalWrite(BATTERY_MEAS_EN_PIN, LOW);
  analogReadResolution(12);

  initFreshState();

  fsReady = initFilesystem();
  if (fsReady && !loadStateFromFlash()) {
    logLine("[FS] using fresh state");
  }

  updateBatteryInRam();

  if (!initAccelerometer()) {
    logLine("[ACC] init failed");
  }

  if (!startBleAdvertising()) {
    logLine("[BLE] start failed");
  }

  armSaveTimer();

}

void processDisconnectLedSequence() {
  if (!pendingDisconnectRedBlinkSequence) return;
  pendingDisconnectRedBlinkSequence = false;

  blinkRedSequence(DISCONNECT_RED_BLINKS);
}

void loop() {
  if (bleStarted) {
    serviceBle();
  }

  if (bleStarted && !clientConnected && bleAdvertising) {
    if (millis() - bootMillis >= BLE_ADVERTISE_WINDOW_MS) {
      stopBleCompletely();
      pendingDisconnectRedBlinkSequence = true;
    }
  }

  processBleWaitLed();
  processDisconnectLedSequence();

  processLisInterrupts();
  maybeReturnToWakeMode();

  periodicBatteryMeasureIfNeeded();
  processSaveTimer();

  sleep();
}
