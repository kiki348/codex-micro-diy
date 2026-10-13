#include <Arduino.h>

#include <math.h>
#include <stdlib.h>
#include <string.h>

#include <SPIFFS.h>

#include "driver/gpio.h"
#include "driver/rmt.h"
#include "esp_err.h"
#include "esp_sleep.h"

#include "CodexMicroBle.h"
#include "BoardLightingGeometry.h"
#include "EncoderModeGesture.h"

extern bool fsReady;  // defined in CodexMicroBle.cpp

namespace {
EncoderModeGesture encoderModeGesture;


constexpr gpio_num_t kKeyLedPin = GPIO_NUM_12;
constexpr gpio_num_t kAmbientLedPin = GPIO_NUM_13;
constexpr gpio_num_t kFiveVoltEnablePin = GPIO_NUM_14;
constexpr size_t kKeyLedCount = 6;
constexpr size_t kAmbientLedCount = 8;
static_assert(sizeof(spatialLighting::keyPoints) / sizeof(spatialLighting::Point) == kKeyLedCount,
              "PCB key LED geometry must cover the complete strip");
static_assert(sizeof(spatialLighting::ambientPoints) / sizeof(spatialLighting::Point) == kAmbientLedCount,
              "PCB ambient LED geometry must cover the complete strip");

constexpr uint8_t kShiftClockPin = 17;
constexpr uint8_t kParallelLoadPin = 7;
constexpr uint8_t kClockEnablePin = 18;
constexpr uint8_t kSerialDataPin = 9;

constexpr uint8_t kEncoderAPin = 4;
constexpr uint8_t kEncoderBPin = 5;
constexpr uint8_t kEncoderPressPin = 6;
constexpr uint8_t kJoystickXPin = 8;
constexpr uint8_t kJoystickYPin = 10;
constexpr uint8_t kBatterySensePin = 2;
constexpr uint8_t kChargeCompletePin = 15;
constexpr uint8_t kChargingPin = 16;
constexpr uint32_t kCpuFrequencyMhz = 160;

// Preserve the input diagnostic's 5 ms raw scan and 25 ms host-visible
// timing. The final firmware requires a state to remain unchanged for the
// full 25 ms before emitting an edge.
constexpr uint32_t kButtonScanIntervalUs = 5000;
constexpr uint32_t kButtonDebounceMs = 25;
constexpr uint32_t kJoystickScanIntervalUs = 20000;
constexpr uint8_t kJoystickAdcSamples = 8;
constexpr uint16_t kJoystickCenterSamples = 160;
constexpr uint16_t kJoystickAdcMaximum = 4095;
constexpr float kJoystickDeadzone = 0.03f;
constexpr float kJoystickDistanceReportStep = 0.005f;
constexpr float kJoystickAngleReportStep = 1.0f / 360.0f;
constexpr uint32_t kBatteryUpdateIntervalMs = 30000;
constexpr uint8_t kBatteryAdcSamples = 16;
// User-measured calibration point: VBATT=3.88 V and BATTEST=1.92 V.
constexpr float kBatteryVoltageScale = 3.88f / 1.92f;

// Raw bits after reading U15.D7..D0 then U16.D7..D0. U16 is a 74HC165:
// its Q7 stream is H,G,F,E,D,C,B,A, so the PCB's SW1=E and SW2=F land at
// raw bits 4 and 5 respectively. Keep the physical SW1/SW2 order intact.
constexpr uint8_t kRawBitBySwitch[12] = {
    4, 5, 6, 7, 0, 1, 2, 3, 8, 9, 10, 11,
};

struct KeyBinding {
  const char* key;
  int8_t agent;
};

struct BatteryCurvePoint {
  uint16_t millivolts;
  uint8_t percentage;
};

// Coarse 1S lithium-ion open-circuit estimate. Interpolation happens only
// between adjacent points instead of treating the full voltage range as a
// single linear percentage scale. Actual percentage remains load-, cell-,
// temperature-, and calibration-dependent.
constexpr BatteryCurvePoint kBatteryCurve[] = {
    {3300, 0},
    {3500, 5},
    {3700, 20},
    {3800, 45},
    {3900, 65},
    {4000, 80},
    {4100, 90},
    {4200, 100},
};

// Physical switch number order: SW1..SW12.
constexpr KeyBinding kKeyBindings[12] = {
    {"AG00", 0},
    {"AG01", 1},
    {"AG05", 5},
    {"AG04", 4},
    {"AG03", 3},
    {"AG02", 2},
    {"ACT09", -1},
    {"ACT08", -1},
    {"ACT07", -1},
    {"ACT06", -1},
    {"ACT12", -1},
    {"ACT10", -1},
};

constexpr rmt_channel_t kKeyLedChannel = RMT_CHANNEL_0;
constexpr rmt_channel_t kAmbientLedChannel = RMT_CHANNEL_1;
constexpr uint8_t kRmtClockDivider = 2;
constexpr uint32_t kResetTimeUs = 100;
constexpr size_t kBitsPerPixel = 24;

constexpr uint32_t kFrameIntervalUs = 4167;  // Approximately 240 Hz.
constexpr uint32_t kMonitorSnapshotIntervalMs = 1000;
constexpr uint32_t kDefaultBreathPeriodMs = 2000;
constexpr uint32_t kDefaultAmbientLapMs = 2000;
constexpr uint32_t kMinimumEffectPeriodMs = 200;
constexpr uint32_t kMaximumEffectPeriodMs = 30000;
// Local conversion for the protocol's normalized speed. This keeps the
// desktop's current s=0.4 command at one 2000 ms animation cycle.
constexpr float kProtocolSpeedScaleMs = 800.0f;
constexpr uint8_t kMaximumBrightnessPercent = 100;
constexpr size_t kAmbientFlowLitCount = 4;

// Power gesture: two short presses completed within one second, followed by
// a third press started within two seconds and held for three seconds.
constexpr uint32_t kPowerShortPressMaximumMs = 500;
constexpr uint32_t kPowerDoublePressWindowMs = 1000;
constexpr uint32_t kPowerHoldStartWindowMs = 2000;
constexpr uint32_t kPowerOffCommitMs = 3000;
constexpr uint32_t kPowerOffFinishAnimationMs = 800;
constexpr uint32_t kPowerOnHoldMs = 3000;
constexpr uint32_t kPowerOnFrameIntervalMs = 16;
constexpr uint32_t kBleResetChordHoldMs = 3000;
constexpr uint8_t kBleResetConfirmFlashCount = 2;
constexpr uint32_t kBleResetConfirmOnMs = 180;
constexpr uint32_t kBleResetConfirmOffMs = 120;
constexpr uint32_t kBleSearchLapMs = 1200;
constexpr uint32_t kBleConnectedBreathMs = 1800;
// Ignore brief USB/BLE transport gaps so an enumeration glitch cannot replace
// an active host animation with the search/connected indicator sequence.
constexpr uint32_t kHostDisconnectDebounceMs = 750;
// The hardware notes limit the perimeter LEDs to 35% maximum brightness.
constexpr uint8_t kSystemIndicatorMaximum = 89;
constexpr uint16_t kAgent0SwitchMask = 1U << 0;

static_assert(kAmbientLedCount >= 2,
              "Power-off animation requires at least two ambient LEDs");
static_assert(kFiveVoltEnablePin != kKeyLedPin &&
                  kFiveVoltEnablePin != kAmbientLedPin,
              "5V enable GPIO must not share an LED data pin");

// UI order: SW1, SW2, SW6, SW5, SW4, SW3.
// Serial LED order: SW2, SW1, SW3, SW4, SW5, SW6.
constexpr size_t kPhysicalKeyToSerial[kKeyLedCount] = {1, 0, 5, 4, 3, 2};

struct LedTiming {
  uint16_t t0HighTicks;
  uint16_t t0LowTicks;
  uint16_t t1HighTicks;
  uint16_t t1LowTicks;
};

// YO-WS2812B-3528 test profile: standard 800 kHz WS2812B timing.
constexpr LedTiming kKeyLedTiming = {
    16,  // 400 ns
    34,  // 850 ns
    32,  // 800 ns
    18,  // 450 ns
};

// Confirmed 4020-1815S timing rounded to 25 ns RMT ticks.
constexpr LedTiming kAmbientLedTiming = {
    12,  // 300 ns
    24,  // 600 ns
    24,  // 600 ns
    12,  // 300 ns
};

struct Rgb {
  uint8_t red;
  uint8_t green;
  uint8_t blue;
};

enum class KeyEffect : uint8_t {
  Off = 0,
  Solid = 1,
  Snake = 2,
  Rainbow = 3,
  Breathe = 4,
  Gradient = 5,
  ShallowBreathe = 6,
};

enum class AmbientEffect : uint8_t {
  Off = 0,
  Solid = 1,
  Snake = 2,
  Rainbow = 3,
  Breathe = 4,
  Gradient = 5,
  ShallowBreathe = 6,
};

struct KeySetting {
  Rgb color;
  KeyEffect effect;
  float brightness;
  float speed;
  float magic;
};

struct AmbientSetting {
  Rgb colorA;
  Rgb colorB;
  AmbientEffect effect;
  float brightness;
  float speed;
  float magic;
};

KeySetting keySettings[kKeyLedCount] = {
    {{255, 255, 255}, KeyEffect::Solid, 1.0f, -1.0f, 1.0f},
    {{255, 255, 255}, KeyEffect::Solid, 1.0f, -1.0f, 1.0f},
    {{255, 255, 255}, KeyEffect::Solid, 1.0f, -1.0f, 1.0f},
    {{255, 255, 255}, KeyEffect::Solid, 1.0f, -1.0f, 1.0f},
    {{255, 255, 255}, KeyEffect::Solid, 1.0f, -1.0f, 1.0f},
    {{255, 255, 255}, KeyEffect::Solid, 1.0f, -1.0f, 1.0f},
};

AmbientSetting ambientSetting = {
    {0, 180, 255},
    {140, 0, 255},
    AmbientEffect::Off,
    1.0f,
    -1.0f,
    1.0f,
};

uint8_t globalBrightnessPercent = 100;
uint32_t breathPeriodMs = kDefaultBreathPeriodMs;
uint32_t ambientLapMs = kDefaultAmbientLapMs;

rmt_item32_t keyRmtItems[kKeyLedCount * kBitsPerPixel];
rmt_item32_t ambientRmtItems[kAmbientLedCount * kBitsPerPixel];
Rgb keyFrame[kKeyLedCount] = {};
Rgb ambientFrame[kAmbientLedCount] = {};

char serialLine[160];
size_t serialLineLength = 0;
uint32_t lastFrameUs = 0;
uint32_t lastMonitorSnapshotMs = 0;

CodexMicroBle codex;
void dispatchEncoderGesture(EncoderModeGesture::Event event) {
  if (event == EncoderModeGesture::Event::Toggle) codex.toggleCodexMode();
  else if (event == EncoderModeGesture::Event::Click) {
    codex.sendKey("ENC", 1);
    codex.sendKey("ENC", 0);
  }
}
CodexMicroState bleState;
uint32_t appliedThreadsRevision = 0;
uint32_t appliedAmbientRevision = 0;

struct JoystickPosition {
  float angle = 0.0f;
  float distance = 0.0f;
};

portMUX_TYPE encoderMux = portMUX_INITIALIZER_UNLOCKED;
volatile uint8_t encoderPreviousState = 0;
volatile int8_t encoderQuarterSteps = 0;
volatile int16_t encoderPendingSteps = 0;
bool encoderInterruptsAttached = false;

uint16_t latestKeyRaw = 0xFFFF;
uint16_t keyCandidateMask = 0;
uint16_t keyStableMask = 0;
uint32_t keyPressCounts[12] = {};
uint32_t rawLowCounts[16] = {};
uint32_t buttonScanCount = 0;
uint32_t buttonScanLastUs = 0;
uint32_t buttonScanMaxGapUs = 0;
uint32_t buttonScanOver25ms = 0;
uint32_t layerIndicatorStartedMs = 0;
uint32_t layerIndicatorColor = 0;
bool layerIndicatorActive = false;
uint32_t keyReleaseCounts[12] = {};
uint32_t keyCandidateSinceMs[12] = {};
bool encoderPressCandidate = false;
bool encoderPressStable = false;
uint32_t encoderPressCandidateSinceMs = 0;

enum class PowerOffState : uint8_t {
  Idle,
  HoldCountdown,
  Committed,
};

enum class StartupAmbientState : uint8_t {
  None,
  WakeHold,
  WakeAccepted,
  BleSearching,
  BleConnectedBreath,
};

PowerOffState powerOffState = PowerOffState::Idle;
StartupAmbientState startupAmbientState = StartupAmbientState::None;
bool startupHostWasConnected = false;
bool startupHostDisconnectPending = false;
uint32_t startupHostDisconnectedSinceMs = 0;
uint8_t powerShortPressCount = 0;
uint32_t powerFirstPressStartedMs = 0;
uint32_t powerSecondReleaseMs = 0;
uint32_t powerCurrentPressStartedMs = 0;
uint32_t powerOffHoldStartedMs = 0;
uint32_t powerOffCommittedMs = 0;
uint32_t startupAmbientStartedMs = 0;
bool powerGestureConsumesEncoderPress = false;
bool suppressEncoderReleaseAfterWake = false;
bool suppressAgent0ReleaseAfterWake = false;
bool clearBleBondsOnStartup = false;

uint16_t joystickX = 0;
uint16_t joystickY = 0;
uint16_t joystickCenterX = 2048;
uint16_t joystickCenterY = 2048;
JoystickPosition joystickPosition;
JoystickPosition lastReportedJoystickPosition;
bool joystickReportValid = false;

uint32_t lastButtonScanUs = 0;
uint32_t lastJoystickScanUs = 0;
uint32_t lastBatteryUpdateMs = 0;

[[noreturn]] void enterDeepSleepAfterRelease();

uint8_t clampByte(int value) {
  if (value < 0) {
    return 0;
  }
  if (value > 255) {
    return 255;
  }
  return static_cast<uint8_t>(value);
}

Rgb scaleSystemColor(const Rgb& color, float scale) {
  const float limited = constrain(scale, 0.0f, 1.0f);
  return {
      clampByte(static_cast<int>(lroundf(color.red * limited))),
      clampByte(static_cast<int>(lroundf(color.green * limited))),
      clampByte(static_cast<int>(lroundf(color.blue * limited))),
  };
}

bool initializeRmtChannel(gpio_num_t pin, rmt_channel_t channel) {
  rmt_config_t config = RMT_DEFAULT_CONFIG_TX(pin, channel);
  config.clk_div = kRmtClockDivider;
  config.tx_config.loop_en = false;
  config.tx_config.carrier_en = false;
  config.tx_config.idle_output_en = true;
  config.tx_config.idle_level = RMT_IDLE_LEVEL_LOW;

  esp_err_t result = rmt_config(&config);
  if (result != ESP_OK) {
    Serial.printf("ERR RMT_CONFIG GPIO%d %s\n",
                  static_cast<int>(pin), esp_err_to_name(result));
    return false;
  }

  result = rmt_driver_install(channel, 0, 0);
  if (result != ESP_OK) {
    Serial.printf("ERR RMT_INSTALL GPIO%d %s\n",
                  static_cast<int>(pin), esp_err_to_name(result));
    return false;
  }
  return true;
}

void appendByte(uint8_t value, size_t& itemIndex,
                const LedTiming& timing, rmt_item32_t* rmtItems) {
  for (int bit = 7; bit >= 0; --bit) {
    const bool one = (value & (1U << bit)) != 0;
    rmt_item32_t& item = rmtItems[itemIndex++];
    item.level0 = 1;
    item.duration0 = one ? timing.t1HighTicks : timing.t0HighTicks;
    item.level1 = 0;
    item.duration1 = one ? timing.t1LowTicks : timing.t0LowTicks;
  }
}

bool sendFrame(rmt_channel_t channel, const Rgb* pixels, size_t ledCount,
               const LedTiming& timing, rmt_item32_t* rmtItems) {
  size_t itemIndex = 0;
  for (size_t pixel = 0; pixel < ledCount; ++pixel) {
    appendByte(pixels[pixel].green, itemIndex, timing, rmtItems);
    appendByte(pixels[pixel].red, itemIndex, timing, rmtItems);
    appendByte(pixels[pixel].blue, itemIndex, timing, rmtItems);
  }

  const esp_err_t result =
      rmt_write_items(channel, rmtItems, itemIndex, true);
  if (result != ESP_OK) {
    Serial.printf("ERR RMT_WRITE %d %s\n",
                  static_cast<int>(channel), esp_err_to_name(result));
    return false;
  }
  delayMicroseconds(kResetTimeUs);
  return true;
}

uint16_t readShiftRegisters() {
  digitalWrite(kClockEnablePin, HIGH);
  digitalWrite(kShiftClockPin, LOW);

  digitalWrite(kParallelLoadPin, LOW);
  delayMicroseconds(5);
  digitalWrite(kParallelLoadPin, HIGH);
  delayMicroseconds(5);

  digitalWrite(kClockEnablePin, LOW);
  delayMicroseconds(2);

  uint16_t raw = 0;
  for (uint8_t index = 0; index < 16; ++index) {
    raw = static_cast<uint16_t>(
        (raw << 1) | (digitalRead(kSerialDataPin) == HIGH ? 1U : 0U));

    digitalWrite(kShiftClockPin, HIGH);
    delayMicroseconds(2);
    digitalWrite(kShiftClockPin, LOW);
    delayMicroseconds(2);
  }

  digitalWrite(kClockEnablePin, HIGH);
  return raw;
}

uint16_t rawToSwitchMask(uint16_t raw) {
  uint16_t pressed = 0;
  for (uint8_t switchIndex = 0; switchIndex < 12; ++switchIndex) {
    const uint16_t rawBit =
        static_cast<uint16_t>(1U << kRawBitBySwitch[switchIndex]);
    if ((raw & rawBit) == 0) {
      pressed |= static_cast<uint16_t>(1U << switchIndex);
    }
  }
  return pressed;
}

int8_t IRAM_ATTR quadratureDelta(uint8_t transition) {
  switch (transition) {
    case 0b0010:
    case 0b1011:
    case 0b1101:
    case 0b0100:
      return 1;
    case 0b0001:
    case 0b0111:
    case 0b1110:
    case 0b1000:
      return -1;
    default:
      return 0;
  }
}

void IRAM_ATTR onEncoderChange() {
  const uint8_t currentState = static_cast<uint8_t>(
      (gpio_get_level(static_cast<gpio_num_t>(kEncoderAPin)) << 1) |
      gpio_get_level(static_cast<gpio_num_t>(kEncoderBPin)));

  portENTER_CRITICAL_ISR(&encoderMux);
  if (currentState != encoderPreviousState) {
    const uint8_t transition = static_cast<uint8_t>(
        (encoderPreviousState << 2) | currentState);
    const int8_t delta = quadratureDelta(transition);
    encoderPreviousState = currentState;

    if (delta == 0) {
      encoderQuarterSteps = 0;
    } else {
      encoderQuarterSteps =
          static_cast<int8_t>(encoderQuarterSteps + delta);
      if (encoderQuarterSteps >= 4) {
        ++encoderPendingSteps;
        encoderQuarterSteps = 0;
      } else if (encoderQuarterSteps <= -4) {
        --encoderPendingSteps;
        encoderQuarterSteps = 0;
      }
    }
  }
  portEXIT_CRITICAL_ISR(&encoderMux);
}

int16_t consumeEncoderSteps() {
  portENTER_CRITICAL(&encoderMux);
  const int16_t steps = encoderPendingSteps;
  encoderPendingSteps = 0;
  portEXIT_CRITICAL(&encoderMux);
  return steps;
}

uint16_t readAveragedAdc(uint8_t pin) {
  uint32_t sum = 0;
  for (uint8_t index = 0; index < kJoystickAdcSamples; ++index) {
    sum += analogRead(pin);
  }
  return static_cast<uint16_t>(sum / kJoystickAdcSamples);
}

void sampleJoystick() {
  joystickX = readAveragedAdc(kJoystickXPin);
  joystickY = readAveragedAdc(kJoystickYPin);
}

void calibrateJoystickCenter() {
  uint32_t xSum = 0;
  uint32_t ySum = 0;
  for (uint16_t index = 0; index < kJoystickCenterSamples; ++index) {
    xSum += analogRead(kJoystickXPin);
    ySum += analogRead(kJoystickYPin);
    delay(2);
  }
  joystickCenterX =
      static_cast<uint16_t>(xSum / kJoystickCenterSamples);
  joystickCenterY =
      static_cast<uint16_t>(ySum / kJoystickCenterSamples);
  sampleJoystick();
  Serial.printf(
      "INPUT joystick center x=%u y=%u deadzone=%.1f%%\n",
      joystickCenterX, joystickCenterY, kJoystickDeadzone * 100.0f);
}

float normalizeJoystickAxis(uint16_t value, uint16_t center) {
  const int32_t delta = static_cast<int32_t>(value) - center;
  const uint16_t span =
      delta >= 0 ? static_cast<uint16_t>(kJoystickAdcMaximum - center)
                 : center;
  if (span == 0) {
    return 0.0f;
  }
  const float normalized =
      static_cast<float>(delta) / static_cast<float>(span);
  return constrain(normalized, -1.0f, 1.0f);
}

JoystickPosition calculateJoystickPosition() {
  // Final assembled hardware test: increasing X is physical right and
  // increasing Y is physical down. These signs also match the Codex angle
  // convention: right=0.00, down=0.25, left=0.50, up=0.75.
  const float normalizedX =
      normalizeJoystickAxis(joystickX, joystickCenterX);
  const float normalizedY =
      normalizeJoystickAxis(joystickY, joystickCenterY);
  const float rawDistance =
      sqrtf(normalizedX * normalizedX + normalizedY * normalizedY);
  if (rawDistance <= kJoystickDeadzone) {
    return {};
  }

  constexpr float kTwoPi = 6.28318530717958647692f;
  float angle = atan2f(normalizedY, normalizedX) / kTwoPi;
  if (angle < 0.0f) {
    angle += 1.0f;
  }
  const float clampedDistance = constrain(rawDistance, 0.0f, 1.0f);
  const float distance =
      (clampedDistance - kJoystickDeadzone) /
      (1.0f - kJoystickDeadzone);
  JoystickPosition position;
  position.angle = angle;
  position.distance = constrain(distance, 0.0f, 1.0f);
  return position;
}

float circularAngleDifference(float first, float second) {
  float difference = fabsf(first - second);
  if (difference > 0.5f) {
    difference = 1.0f - difference;
  }
  return difference;
}

bool joystickPositionNeedsReport(const JoystickPosition& next) {
  if (!joystickReportValid) {
    return true;
  }
  const bool nextCentered = next.distance == 0.0f;
  const bool previousCentered =
      lastReportedJoystickPosition.distance == 0.0f;
  if (nextCentered || previousCentered) {
    return nextCentered != previousCentered;
  }
  return fabsf(next.distance - lastReportedJoystickPosition.distance) >=
             kJoystickDistanceReportStep ||
         circularAngleDifference(
             next.angle, lastReportedJoystickPosition.angle) >=
             kJoystickAngleReportStep;
}

void sendKeyEdge(uint8_t switchIndex, bool pressed) {
  const KeyBinding& binding = kKeyBindings[switchIndex];
  codex.sendKey(binding.key, pressed ? 1 : 0, binding.agent);
  Serial.printf("INPUT SW%u %s %s\n",
                static_cast<unsigned>(switchIndex + 1),
                binding.key,
                pressed ? "pressed" : "released");
}

void resetPowerGestureSequence() {
  powerShortPressCount = 0;
  powerFirstPressStartedMs = 0;
  powerSecondReleaseMs = 0;
}

void commitPowerOff(uint32_t committedMs) {
  powerOffState = PowerOffState::Committed;
  powerOffCommittedMs = committedMs;
  Serial.println("POWER shutdown committed; finishing animation");
}

bool onEncoderPowerGestureEdge(bool pressed, uint32_t nowMs) {
  if (pressed) {
    powerCurrentPressStartedMs = nowMs;

    if (powerOffState != PowerOffState::Idle) {
      return powerGestureConsumesEncoderPress;
    }

    if (powerShortPressCount == 1 &&
        nowMs - powerFirstPressStartedMs >
            kPowerDoublePressWindowMs) {
      resetPowerGestureSequence();
    }

    if (powerShortPressCount == 2) {
      if (nowMs - powerSecondReleaseMs <=
          kPowerHoldStartWindowMs) {
        powerOffState = PowerOffState::HoldCountdown;
        powerOffHoldStartedMs = nowMs;
        startupAmbientState = StartupAmbientState::None;
        powerGestureConsumesEncoderPress = true;
        resetPowerGestureSequence();
        Serial.println(
            "POWER shutdown hold started; release before commit to cancel");
        return true;
      }
      resetPowerGestureSequence();
    }
    return false;
  }

  if (powerOffState == PowerOffState::HoldCountdown) {
    if (nowMs - powerOffHoldStartedMs >= kPowerOffCommitMs) {
      // A release observed exactly on or after the LED-2 threshold cannot
      // cancel shutdown, even if this debounced edge runs before poll().
      commitPowerOff(powerOffHoldStartedMs + kPowerOffCommitMs);
      return true;
    }
    powerOffState = PowerOffState::Idle;
    resetPowerGestureSequence();
    powerGestureConsumesEncoderPress = false;
    Serial.println("POWER shutdown hold cancelled");
    return true;
  }

  if (powerOffState == PowerOffState::Committed) {
    return powerGestureConsumesEncoderPress;
  }

  const uint32_t pressDurationMs =
      nowMs - powerCurrentPressStartedMs;
  if (pressDurationMs > kPowerShortPressMaximumMs) {
    resetPowerGestureSequence();
    return false;
  }

  if (powerShortPressCount == 0) {
    powerShortPressCount = 1;
    powerFirstPressStartedMs = powerCurrentPressStartedMs;
    return false;
  }

  if (powerShortPressCount == 1 &&
      nowMs - powerFirstPressStartedMs <=
          kPowerDoublePressWindowMs) {
    powerShortPressCount = 2;
    powerSecondReleaseMs = nowMs;
    Serial.println(
        "POWER double short press accepted; hold within two seconds");
    return false;
  }

  powerShortPressCount = 1;
  powerFirstPressStartedMs = powerCurrentPressStartedMs;
  powerSecondReleaseMs = 0;
  return false;
}

void pollPowerGesture(uint32_t nowMs) {
  if (powerOffState == PowerOffState::HoldCountdown) {
    if (nowMs - powerOffHoldStartedMs >= kPowerOffCommitMs) {
      // At this instant serial LED 2 has fully faded out. From now on the
      // shutdown is deliberately irreversible; LED 1 still gets its own
      // finishing fade before the chip enters Deep-sleep.
      commitPowerOff(powerOffHoldStartedMs + kPowerOffCommitMs);
    }
    return;
  }

  if (powerOffState == PowerOffState::Committed) {
    if (nowMs - powerOffCommittedMs >=
        kPowerOffFinishAnimationMs) {
      enterDeepSleepAfterRelease();
    }
    return;
  }

  if (powerShortPressCount == 1 &&
      nowMs - powerFirstPressStartedMs >
          kPowerDoublePressWindowMs) {
    resetPowerGestureSequence();
  } else if (powerShortPressCount == 2 &&
             nowMs - powerSecondReleaseMs >
                 kPowerHoldStartWindowMs) {
    resetPowerGestureSequence();
  }
}

void updateDebouncedButtons(uint32_t nowMs) {
  // Measure actual scan cadence, including time spent in RPC/file/USB work.
  // The nominal 5 ms scheduler period alone cannot prove a brief press was read.
  const uint32_t scanNowUs = micros();
  if (buttonScanCount != 0) {
    const uint32_t gapUs = scanNowUs - buttonScanLastUs;
    if (gapUs > buttonScanMaxGapUs) buttonScanMaxGapUs = gapUs;
    if (gapUs > 25000) ++buttonScanOver25ms;
  }
  buttonScanLastUs = scanNowUs;
  ++buttonScanCount;
  const uint16_t previousRaw = latestKeyRaw;
  latestKeyRaw = readShiftRegisters();
  for (uint8_t bit = 0; bit < 16; ++bit) {
    if ((previousRaw & (1U << bit)) && !(latestKeyRaw & (1U << bit)))
      ++rawLowCounts[bit];
  }
  const uint16_t currentMask = rawToSwitchMask(latestKeyRaw);

  for (uint8_t switchIndex = 0; switchIndex < 12; ++switchIndex) {
    const uint16_t bit = static_cast<uint16_t>(1U << switchIndex);
    const bool current = (currentMask & bit) != 0;
    const bool candidate = (keyCandidateMask & bit) != 0;
    const bool stable = (keyStableMask & bit) != 0;

    if (current != candidate) {
      if (current) {
        keyCandidateMask |= bit;
      } else {
        keyCandidateMask &= static_cast<uint16_t>(~bit);
      }
      keyCandidateSinceMs[switchIndex] = nowMs;
    } else if (candidate != stable &&
               nowMs - keyCandidateSinceMs[switchIndex] >=
                   kButtonDebounceMs) {
      if (candidate) {
        ++keyPressCounts[switchIndex];
        keyStableMask |= bit;
      } else {
        ++keyReleaseCounts[switchIndex];
        keyStableMask &= static_cast<uint16_t>(~bit);
      }
      bool consumedByWakeChord = false;
      if (switchIndex == 0 && suppressAgent0ReleaseAfterWake) {
        consumedByWakeChord = true;
        if (!candidate) {
          suppressAgent0ReleaseAfterWake = false;
          Serial.println("POWER Agent 0 wake-chord release consumed locally");
        }
      }
      if (bleState.connected && !consumedByWakeChord) {
        sendKeyEdge(switchIndex, candidate);
      }
    }
  }

  const bool pressCurrent =
      digitalRead(kEncoderPressPin) == LOW;
  if (pressCurrent != encoderPressCandidate) {
    encoderPressCandidate = pressCurrent;
    encoderPressCandidateSinceMs = nowMs;
  } else if (encoderPressCandidate != encoderPressStable &&
             nowMs - encoderPressCandidateSinceMs >=
                 kButtonDebounceMs) {
    encoderPressStable = encoderPressCandidate;
    bool consumedByPowerGesture = false;
    if (suppressEncoderReleaseAfterWake) {
      consumedByPowerGesture = true;
      if (!encoderPressStable) {
        suppressEncoderReleaseAfterWake = false;
        Serial.println("POWER wake-button release consumed locally");
      }
    } else {
      consumedByPowerGesture =
          onEncoderPowerGestureEdge(encoderPressStable, nowMs);
    }
    dispatchEncoderGesture(encoderModeGesture.edge(
        encoderPressStable, nowMs, consumedByPowerGesture || !bleState.connected));
    Serial.printf("INPUT encoder %s\n",
                  encoderPressStable ? "pressed" : "released");
  }
  if (!bleState.connected || powerOffState != PowerOffState::Idle ||
      suppressEncoderReleaseAfterWake) encoderModeGesture.cancel();
  else dispatchEncoderGesture(encoderModeGesture.poll(nowMs));
}

void updateEncoderEvents() {
  const int16_t pendingSteps = consumeEncoderSteps();
  if (pendingSteps == 0) {
    return;
  }

  // The full-board diagnostic showed positive/A-leading steps are physical
  // left and negative steps are physical right. The Codex desktop action was
  // then tested and requires the protocol key names to be swapped: physical
  // left sends ENC_CW and physical right sends ENC_CC.
  const bool left = pendingSteps > 0;
  const uint16_t count = static_cast<uint16_t>(
      left ? pendingSteps : -pendingSteps);
  const char* key = left ? "ENC_CW" : "ENC_CC";
  if (bleState.connected) {
    for (uint16_t index = 0; index < count; ++index) {
      codex.sendKey(key, 2);
    }
  }
  Serial.printf("INPUT encoder %s steps=%u\n",
                left ? "left" : "right", count);
}

void updateJoystickPosition() {
  sampleJoystick();
  joystickPosition = calculateJoystickPosition();
  if (!bleState.connected) {
    joystickReportValid = false;
    return;
  }
  if (!joystickPositionNeedsReport(joystickPosition)) {
    return;
  }

  codex.sendJoystick(joystickPosition.angle, joystickPosition.distance);
  lastReportedJoystickPosition = joystickPosition;
  joystickReportValid = true;
  Serial.printf(
      "INPUT joystick angle=%.4f distance=%.4f x=%u y=%u\n",
      joystickPosition.angle, joystickPosition.distance,
      joystickX, joystickY);
}

uint32_t readBatterySenseMillivolts() {
  // Discard the first conversion after a long idle period, then average a
  // short burst. The PCB's 100 nF capacitor supports the 100K/100K divider
  // while the ADC sampling capacitor is being charged.
  analogReadMilliVolts(kBatterySensePin);
  uint32_t totalMillivolts = 0;
  for (uint8_t sample = 0; sample < kBatteryAdcSamples; ++sample) {
    totalMillivolts += analogReadMilliVolts(kBatterySensePin);
  }
  return totalMillivolts / kBatteryAdcSamples;
}

uint8_t estimateBatteryPercentage(uint32_t batteryMillivolts) {
  if (batteryMillivolts <= kBatteryCurve[0].millivolts) {
    return kBatteryCurve[0].percentage;
  }
  constexpr size_t pointCount =
      sizeof(kBatteryCurve) / sizeof(kBatteryCurve[0]);
  for (size_t index = 1; index < pointCount; ++index) {
    const BatteryCurvePoint& lower = kBatteryCurve[index - 1];
    const BatteryCurvePoint& upper = kBatteryCurve[index];
    if (batteryMillivolts > upper.millivolts) {
      continue;
    }
    const uint32_t voltageSpan = upper.millivolts - lower.millivolts;
    const uint32_t voltageOffset = batteryMillivolts - lower.millivolts;
    const uint32_t percentageSpan = upper.percentage - lower.percentage;
    return static_cast<uint8_t>(
        lower.percentage +
        (voltageOffset * percentageSpan + voltageSpan / 2) / voltageSpan);
  }
  return kBatteryCurve[pointCount - 1].percentage;
}

void updateBattery(bool force = false) {
  const uint32_t nowMs = millis();
  if (!force && lastBatteryUpdateMs != 0 &&
      nowMs - lastBatteryUpdateMs < kBatteryUpdateIntervalMs) {
    return;
  }
  lastBatteryUpdateMs = nowMs;

  const bool chargeComplete = digitalRead(kChargeCompletePin) == LOW;
  const bool chargingSignal = digitalRead(kChargingPin) == LOW;
  const bool charging = chargingSignal && !chargeComplete;
  const uint32_t senseMillivolts = readBatterySenseMillivolts();
  const uint32_t batteryMillivolts = static_cast<uint32_t>(
      lroundf(static_cast<float>(senseMillivolts) * kBatteryVoltageScale));
  const uint8_t percentage = chargeComplete
                                 ? 100
                                 : estimateBatteryPercentage(batteryMillivolts);

  // This updates the cached device.status response for both transports and,
  // when BLE is enabled and connected, the standard HID battery level.
  codex.setBattery(percentage, charging);
  Serial.printf(
      "BATTERY battery_mv=%lu sense_mv=%lu percent=%u charging=%u full=%u\n",
      static_cast<unsigned long>(batteryMillivolts),
      static_cast<unsigned long>(senseMillivolts), percentage,
      charging ? 1U : 0U, chargeComplete ? 1U : 0U);
}

void initializeBatteryMonitoring() {
  // STBY and CHRG have confirmed external 10K pull-ups and are active LOW.
  pinMode(kChargeCompletePin, INPUT);
  pinMode(kChargingPin, INPUT);
  pinMode(kBatterySensePin, INPUT);
  analogReadResolution(12);
  analogSetPinAttenuation(kBatterySensePin, ADC_11db);
}

void initializeShiftRegisterInputs() {
  pinMode(kSerialDataPin, INPUT);
  digitalWrite(kShiftClockPin, LOW);
  pinMode(kShiftClockPin, OUTPUT);
  digitalWrite(kParallelLoadPin, HIGH);
  pinMode(kParallelLoadPin, OUTPUT);
  digitalWrite(kClockEnablePin, HIGH);
  pinMode(kClockEnablePin, OUTPUT);
}

void initializeInputs() {
  initializeShiftRegisterInputs();

  // All three EC11 signals have confirmed external 10K pull-ups.
  pinMode(kEncoderAPin, INPUT);
  pinMode(kEncoderBPin, INPUT);
  pinMode(kEncoderPressPin, INPUT);

  pinMode(kJoystickXPin, INPUT);
  pinMode(kJoystickYPin, INPUT);
  analogReadResolution(12);
  analogSetPinAttenuation(kJoystickXPin, ADC_11db);
  analogSetPinAttenuation(kJoystickYPin, ADC_11db);

  latestKeyRaw = readShiftRegisters();
  keyCandidateMask = rawToSwitchMask(latestKeyRaw);
  keyStableMask = keyCandidateMask;
  const uint32_t nowMs = millis();
  for (uint32_t& candidateSince : keyCandidateSinceMs) {
    candidateSince = nowMs;
  }
  if (suppressAgent0ReleaseAfterWake &&
      (keyStableMask & kAgent0SwitchMask) == 0) {
    // Agent 0 may have been released between wake acceptance and input
    // initialization, leaving no release edge to consume later.
    suppressAgent0ReleaseAfterWake = false;
  }

  encoderPressCandidate =
      digitalRead(kEncoderPressPin) == LOW;
  encoderPressStable = encoderPressCandidate;
  encoderPressCandidateSinceMs = nowMs;
  if (suppressEncoderReleaseAfterWake && !encoderPressStable) {
    // The user may release between the three-second threshold and input
    // initialization. In that case there is no release edge left to consume.
    suppressEncoderReleaseAfterWake = false;
  }

  calibrateJoystickCenter();
  joystickPosition = calculateJoystickPosition();
  joystickReportValid = false;

  encoderPreviousState = static_cast<uint8_t>(
      ((digitalRead(kEncoderAPin) == HIGH ? 1U : 0U) << 1) |
      (digitalRead(kEncoderBPin) == HIGH ? 1U : 0U));
  attachInterrupt(digitalPinToInterrupt(kEncoderAPin),
                  onEncoderChange, CHANGE);
  attachInterrupt(digitalPinToInterrupt(kEncoderBPin),
                  onEncoderChange, CHANGE);
  encoderInterruptsAttached = true;

  const uint32_t nowUs = micros();
  lastButtonScanUs = nowUs;
  lastJoystickScanUs = nowUs;
  Serial.printf("INPUT ready raw=0x%04X keymask=0x%03X\n",
                latestKeyRaw, keyStableMask);
}

void pollInputs() {
  updateEncoderEvents();

  const uint32_t nowUs = micros();
  if (static_cast<uint32_t>(nowUs - lastButtonScanUs) >=
      kButtonScanIntervalUs) {
    lastButtonScanUs = nowUs;
    updateDebouncedButtons(millis());
  }
  if (static_cast<uint32_t>(nowUs - lastJoystickScanUs) >=
      kJoystickScanIntervalUs) {
    lastJoystickScanUs = nowUs;
    updateJoystickPosition();
  }
  pollPowerGesture(millis());
}

void clearFrame(Rgb* pixels, size_t count) {
  for (size_t index = 0; index < count; ++index) {
    pixels[index] = {0, 0, 0};
  }
}

Rgb scaleColor(const Rgb& color, float effectScale) {
  float scale =
      effectScale * static_cast<float>(globalBrightnessPercent) / 100.0f;
  if (scale < 0.0f) {
    scale = 0.0f;
  }
  if (scale > 1.0f) {
    scale = 1.0f;
  }
  return {
      clampByte(static_cast<int>(lroundf(color.red * scale))),
      clampByte(static_cast<int>(lroundf(color.green * scale))),
      clampByte(static_cast<int>(lroundf(color.blue * scale))),
  };
}

Rgb blendColor(const Rgb& first, const Rgb& second, float amount) {
  if (amount < 0.0f) {
    amount = 0.0f;
  }
  if (amount > 1.0f) {
    amount = 1.0f;
  }
  const float inverse = 1.0f - amount;
  return {
      clampByte(static_cast<int>(
          lroundf(first.red * inverse + second.red * amount))),
      clampByte(static_cast<int>(
          lroundf(first.green * inverse + second.green * amount))),
      clampByte(static_cast<int>(
          lroundf(first.blue * inverse + second.blue * amount))),
  };
}

Rgb hueShift(const Rgb& color, float hueOffset) {
  const float red = color.red / 255.0f;
  const float green = color.green / 255.0f;
  const float blue = color.blue / 255.0f;
  const float maximum = max(red, max(green, blue));
  const float minimum = min(red, min(green, blue));
  const float delta = maximum - minimum;

  float hue = 0.0f;
  if (delta > 0.0001f) {
    if (maximum == red) {
      hue = 60.0f * fmodf((green - blue) / delta, 6.0f);
    } else if (maximum == green) {
      hue = 60.0f * (((blue - red) / delta) + 2.0f);
    } else {
      hue = 60.0f * (((red - green) / delta) + 4.0f);
    }
  }
  if (hue < 0.0f) {
    hue += 360.0f;
  }

  hue = fmodf(hue + hueOffset, 360.0f);
  const float saturation = maximum <= 0.0001f ? 0.0f : delta / maximum;
  const float chroma = maximum * saturation;
  const float x = chroma *
                  (1.0f - fabsf(fmodf(hue / 60.0f, 2.0f) - 1.0f));
  const float match = maximum - chroma;

  float shiftedRed = 0.0f;
  float shiftedGreen = 0.0f;
  float shiftedBlue = 0.0f;
  if (hue < 60.0f) {
    shiftedRed = chroma;
    shiftedGreen = x;
  } else if (hue < 120.0f) {
    shiftedRed = x;
    shiftedGreen = chroma;
  } else if (hue < 180.0f) {
    shiftedGreen = chroma;
    shiftedBlue = x;
  } else if (hue < 240.0f) {
    shiftedGreen = x;
    shiftedBlue = chroma;
  } else if (hue < 300.0f) {
    shiftedRed = x;
    shiftedBlue = chroma;
  } else {
    shiftedRed = chroma;
    shiftedBlue = x;
  }

  return {
      clampByte(static_cast<int>(lroundf((shiftedRed + match) * 255.0f))),
      clampByte(static_cast<int>(lroundf((shiftedGreen + match) * 255.0f))),
      clampByte(static_cast<int>(lroundf((shiftedBlue + match) * 255.0f))),
  };
}

uint32_t animationPeriodMs(float speed, uint32_t fallbackPeriodMs) {
  if (speed < 0.0f) {
    return fallbackPeriodMs;
  }
  if (speed <= 0.0001f) {
    return 0;
  }

  uint32_t period = static_cast<uint32_t>(
      lroundf(kProtocolSpeedScaleMs / speed));
  if (period < kMinimumEffectPeriodMs) {
    period = kMinimumEffectPeriodMs;
  }
  if (period > kMaximumEffectPeriodMs) {
    period = kMaximumEffectPeriodMs;
  }
  return period;
}

float animationProgress(uint32_t nowMs, float speed,
                        uint32_t fallbackPeriodMs) {
  const uint32_t period = animationPeriodMs(speed, fallbackPeriodMs);
  if (period == 0) {
    return 0.0f;
  }
  return static_cast<float>(nowMs % period) /
         static_cast<float>(period);
}

float breathingScale(uint32_t nowMs, float speed) {
  return sinf(
      PI * animationProgress(nowMs, speed, breathPeriodMs));
}

float snakeScale(uint32_t nowMs, float speed, size_t index,
                 size_t ledCount) {
  const float progress =
      animationProgress(nowMs, speed, kDefaultAmbientLapMs);
  const float head = progress * static_cast<float>(ledCount);
  float distance = static_cast<float>(index) - head;
  while (distance < 0.0f) {
    distance += static_cast<float>(ledCount);
  }
  const float litLength = max(2.0f, ledCount / 2.0f);
  return distance < litLength ? 1.0f - distance / litLength : 0.0f;
}

bool systemLightingOwnsKeyFrame() {
  return powerOffState != PowerOffState::Idle ||
         startupAmbientState != StartupAmbientState::None;
}

bool renderSystemAmbientFrame(uint32_t nowMs) {
  const float systemMaximum =
      static_cast<float>(kSystemIndicatorMaximum) / 255.0f;

  if (powerOffState == PowerOffState::HoldCountdown) {
    const uint32_t elapsedMs = nowMs - powerOffHoldStartedMs;
    const size_t countdownLedCount = kAmbientLedCount - 1;
    const float completedSegments =
        constrain(static_cast<float>(elapsedMs) /
                      static_cast<float>(kPowerOffCommitMs),
                  0.0f, 1.0f) *
        static_cast<float>(countdownLedCount);

    // LED 1 remains red until after the irreversible threshold. LEDs 8 down
    // through 2 fade one at a time, with LED 2 reaching black at 3 seconds.
    ambientFrame[0] =
        scaleSystemColor({255, 0, 0}, systemMaximum);
    for (size_t segment = 0; segment < countdownLedCount; ++segment) {
      const size_t index = kAmbientLedCount - 1 - segment;
      const float segmentProgress =
          constrain(completedSegments - static_cast<float>(segment),
                    0.0f, 1.0f);
      ambientFrame[index] = scaleSystemColor(
          {255, 0, 0}, systemMaximum * (1.0f - segmentProgress));
    }
    return true;
  }

  if (powerOffState == PowerOffState::Committed) {
    const float progress = constrain(
        static_cast<float>(nowMs - powerOffCommittedMs) /
            static_cast<float>(kPowerOffFinishAnimationMs),
        0.0f, 1.0f);
    ambientFrame[0] =
        scaleSystemColor({255, 0, 0}, systemMaximum * (1.0f - progress));
    return true;
  }

  if (startupAmbientState == StartupAmbientState::WakeHold) {
    const float progress = constrain(
        static_cast<float>(nowMs - startupAmbientStartedMs) /
            static_cast<float>(kPowerOnHoldMs),
        0.0f, 1.0f);
    const Rgb white = scaleSystemColor(
        {255, 255, 255}, systemMaximum * progress);
    for (Rgb& pixel : ambientFrame) {
      pixel = white;
    }
    return true;
  }

  if (startupAmbientState == StartupAmbientState::WakeAccepted) {
    const Rgb green = scaleSystemColor(
        {0, 255, 0}, systemMaximum);
    for (Rgb& pixel : ambientFrame) {
      pixel = green;
    }
    return true;
  }

  if (startupAmbientState == StartupAmbientState::BleSearching) {
    const float head =
        static_cast<float>((nowMs - startupAmbientStartedMs) %
                           kBleSearchLapMs) /
        static_cast<float>(kBleSearchLapMs) *
        static_cast<float>(kAmbientLedCount);
    constexpr float kTailLength = 3.0f;
    for (size_t index = 0; index < kAmbientLedCount; ++index) {
      float distance = head - static_cast<float>(index);
      while (distance < 0.0f) {
        distance += static_cast<float>(kAmbientLedCount);
      }
      while (distance >= static_cast<float>(kAmbientLedCount)) {
        distance -= static_cast<float>(kAmbientLedCount);
      }
      if (distance < kTailLength) {
        ambientFrame[index] = scaleSystemColor(
            {0, 0, 255}, systemMaximum *
                             (1.0f - distance / kTailLength));
      }
    }
    return true;
  }

  if (startupAmbientState ==
      StartupAmbientState::BleConnectedBreath) {
    const float progress = constrain(
        static_cast<float>(nowMs - startupAmbientStartedMs) /
            static_cast<float>(kBleConnectedBreathMs),
        0.0f, 1.0f);
    const float scale = sinf(PI * progress);
    const Rgb green = scaleSystemColor(
        {0, 255, 0}, systemMaximum * scale);
    for (Rgb& pixel : ambientFrame) {
      pixel = green;
    }
    return true;
  }

  return false;
}

void renderKeyFrame(uint32_t nowMs) {
  clearFrame(keyFrame, kKeyLedCount);

  if (systemLightingOwnsKeyFrame()) {
    return;
  }

  for (size_t physicalIndex = 0; physicalIndex < kKeyLedCount;
       ++physicalIndex) {
    const KeySetting& setting = keySettings[physicalIndex];
    Rgb color = setting.color;
    float effectScale = 0.0f;
    switch (setting.effect) {
      case KeyEffect::Off:
        break;
      case KeyEffect::Solid:
        effectScale = 1.0f;
        break;
      case KeyEffect::Snake:
        effectScale = snakeScale(
            nowMs, setting.speed, physicalIndex, kKeyLedCount);
        break;
      case KeyEffect::Rainbow: {
        const float phase =
            animationProgress(nowMs, setting.speed, breathPeriodMs);
        color = hueShift(
            {255, 0, 0},
            360.0f * spatialLighting::rainbowTurns(
                spatialLighting::keyPoints[kPhysicalKeyToSerial[physicalIndex]],
                setting.magic, phase));
        effectScale = 1.0f;
        break;
      }
      case KeyEffect::Breathe:
        effectScale = breathingScale(nowMs, setting.speed);
        break;
      case KeyEffect::Gradient: {
        color = hueShift(setting.color, 360.0f * spatialLighting::gradientTurns(
            spatialLighting::keyPoints[kPhysicalKeyToSerial[physicalIndex]],
            setting.magic));
        effectScale = 1.0f;
        break;
      }
      case KeyEffect::ShallowBreathe:
        effectScale =
            0.5f + 0.5f * breathingScale(nowMs, setting.speed);
        break;
    }
    keyFrame[kPhysicalKeyToSerial[physicalIndex]] =
        scaleColor(color, effectScale * setting.brightness);
  }
}

void renderAmbientFrame(uint32_t nowMs) {
  clearFrame(ambientFrame, kAmbientLedCount);

  if (renderSystemAmbientFrame(nowMs)) {
    return;
  }

  if (ambientSetting.effect == AmbientEffect::Off) {
    return;
  }

  if (ambientSetting.effect == AmbientEffect::Solid) {
    const Rgb color =
        scaleColor(ambientSetting.colorA, ambientSetting.brightness);
    for (size_t index = 0; index < kAmbientLedCount; ++index) {
      ambientFrame[index] = color;
    }
    return;
  }

  if (ambientSetting.effect == AmbientEffect::Breathe) {
    const Rgb color = scaleColor(
        ambientSetting.colorA,
        ambientSetting.brightness *
            breathingScale(nowMs, ambientSetting.speed));
    for (size_t index = 0; index < kAmbientLedCount; ++index) {
      ambientFrame[index] = color;
    }
    return;
  }

  if (ambientSetting.effect == AmbientEffect::ShallowBreathe) {
    const float scale =
        0.5f +
        0.5f * breathingScale(nowMs, ambientSetting.speed);
    const Rgb color = scaleColor(
        ambientSetting.colorA, ambientSetting.brightness * scale);
    for (size_t index = 0; index < kAmbientLedCount; ++index) {
      ambientFrame[index] = color;
    }
    return;
  }

  const float progress = animationProgress(
      nowMs, ambientSetting.speed, ambientLapMs);

  if (ambientSetting.effect == AmbientEffect::Rainbow) {
    for (size_t index = 0; index < kAmbientLedCount; ++index) {
      const float hue = 360.0f * spatialLighting::rainbowTurns(
          spatialLighting::ambientPoints[index], ambientSetting.magic, progress);
      ambientFrame[index] =
          scaleColor(hueShift({255, 0, 0}, hue),
                     ambientSetting.brightness);
    }
    return;
  }

  if (ambientSetting.effect == AmbientEffect::Gradient) {
    for (size_t index = 0; index < kAmbientLedCount; ++index) {
      ambientFrame[index] =
          scaleColor(
              hueShift(ambientSetting.colorA,
                       360.0f * spatialLighting::gradientTurns(
                           spatialLighting::ambientPoints[index], ambientSetting.magic)),
              ambientSetting.brightness);
    }
    return;
  }

  Rgb first = ambientSetting.colorA;
  Rgb second = ambientSetting.colorB;

  // A four-pixel window moves around the ring. During each one-pixel step,
  // the old tail fades out while the new head fades in. The two middle
  // pixels stay at full brightness, so movement is continuous without
  // creating a fifth lit pixel.
  const float segmentPosition = progress * kAmbientLedCount;
  const float wholeStep = floorf(segmentPosition);
  const float stepFraction = segmentPosition - wholeStep;
  const size_t segmentStart =
      static_cast<size_t>(wholeStep) % kAmbientLedCount;

  for (size_t offset = 0; offset < kAmbientFlowLitCount; ++offset) {
    const size_t index = (segmentStart + offset) % kAmbientLedCount;
    float edgeScale = 1.0f;
    if (offset == 0) {
      edgeScale = 1.0f - stepFraction;
    } else if (offset == kAmbientFlowLitCount - 1) {
      edgeScale = stepFraction;
    }

    // Moving the gradient with the fractional step keeps each physical
    // pixel's color continuous when the four-pixel window advances.
    const float amount =
        (static_cast<float>(offset) - stepFraction) /
        static_cast<float>(kAmbientFlowLitCount - 2);
    ambientFrame[index] =
        scaleColor(blendColor(first, second, amount),
                   edgeScale * ambientSetting.brightness);
  }
}

float clampBrightness(float brightness) {
  if (brightness < 0.0f) {
    return 0.0f;
  }
  if (brightness > 1.0f) {
    return 1.0f;
  }
  return brightness;
}

Rgb unpackRgb(uint32_t packed) {
  return {
      static_cast<uint8_t>((packed >> 16) & 0xFF),
      static_cast<uint8_t>((packed >> 8) & 0xFF),
      static_cast<uint8_t>(packed & 0xFF),
  };
}

KeyEffect keyEffectFromBle(uint8_t effect, float brightness) {
  if (brightness <= 0.0001f || effect == 0) {
    return KeyEffect::Off;
  }
  return effect <= 6 ? static_cast<KeyEffect>(effect)
                     : KeyEffect::Solid;
}

AmbientEffect ambientEffectFromBle(uint8_t effect,
                                   float brightness) {
  if (brightness <= 0.0001f || effect == 0) {
    return AmbientEffect::Off;
  }
  return effect <= 6 ? static_cast<AmbientEffect>(effect)
                     : AmbientEffect::Solid;
}

void applyBleThreads(const CodexMicroState& state) {
  for (size_t index = 0; index < kKeyLedCount; ++index) {
    const ThreadLight& source = state.threads[index];
    const float brightness = clampBrightness(source.brightness);
    keySettings[index] = {
        unpackRgb(source.color),
        keyEffectFromBle(source.effect, brightness),
        brightness,
        clampBrightness(source.speed),
        clampBrightness(source.magic),
    };
  }
}

void applyBleAmbient(const CodexMicroState& state) {
  uint32_t color = state.ambient.color;
  float brightness = clampBrightness(state.ambient.brightness);
  uint8_t effect = state.ambient.effect;
  float speed = clampBrightness(state.ambient.speed);
  float magic = clampBrightness(state.ambient.magic);

  const int syncThread = state.ambientSyncThread;
  if (syncThread >= 0 &&
      syncThread < static_cast<int>(state.threads.size())) {
    const ThreadLight& source = state.threads[syncThread];
    color = source.color;
    brightness = clampBrightness(source.brightness);
    effect = source.effect;
    speed = clampBrightness(source.speed);
    magic = clampBrightness(source.magic);
  }

  const Rgb unpacked = unpackRgb(color);
  ambientSetting = {
      unpacked,
      unpacked,
      ambientEffectFromBle(effect, brightness),
      brightness,
      speed,
      magic,
  };
}

void pollBleLighting() {
  const CodexMicroState latest = codex.snapshot();
  const bool threadsChanged =
      latest.threadsRevision != appliedThreadsRevision;
  const bool ambientChanged =
      latest.ambientRevision != appliedAmbientRevision;
  if (threadsChanged) {
    applyBleThreads(latest);
  }
  if (threadsChanged || ambientChanged) {
    applyBleAmbient(latest);
  }

  appliedThreadsRevision = latest.threadsRevision;
  appliedAmbientRevision = latest.ambientRevision;
  bleState = latest;
}

void updateStartupAmbientState(uint32_t nowMs) {
  const bool hostConnected = bleState.connected;

#if CODEX_BLE_ENABLED
  if (!hostConnected && startupHostWasConnected) {
    startupHostDisconnectPending = true;
    startupHostDisconnectedSinceMs = nowMs;
    Serial.println("POWER host disconnect pending debounce");
  }

  if (hostConnected) {
    if (startupHostDisconnectPending) {
      startupHostDisconnectPending = false;
      Serial.println("POWER transient host disconnect ignored");
    }
  } else if (startupHostDisconnectPending &&
             nowMs - startupHostDisconnectedSinceMs >=
                 kHostDisconnectDebounceMs) {
    startupHostDisconnectPending = false;
    startupAmbientState = StartupAmbientState::BleSearching;
    startupAmbientStartedMs = nowMs;
    Serial.println(
        "POWER host disconnected after debounce; BLE search animation restarted");
  }
#endif

  if (startupAmbientState == StartupAmbientState::BleSearching &&
      hostConnected) {
    startupAmbientState = StartupAmbientState::BleConnectedBreath;
    startupAmbientStartedMs = nowMs;
    Serial.println("POWER host connected; green breath started");
  }

  if (startupAmbientState ==
          StartupAmbientState::BleConnectedBreath &&
      nowMs - startupAmbientStartedMs >=
          kBleConnectedBreathMs) {
    startupAmbientState = StartupAmbientState::None;
    Serial.println("POWER startup animation complete");
  }

  startupHostWasConnected = hostConnected;
}

bool parseInteger(const char* token, int& value) {
  if (token == nullptr || *token == '\0') {
    return false;
  }
  char* end = nullptr;
  const long parsed = strtol(token, &end, 10);
  if (end == token || *end != '\0') {
    return false;
  }
  value = static_cast<int>(parsed);
  return true;
}

bool parseRgb(char*& savePointer, Rgb& color) {
  int red = 0;
  int green = 0;
  int blue = 0;
  if (!parseInteger(strtok_r(nullptr, " ", &savePointer), red) ||
      !parseInteger(strtok_r(nullptr, " ", &savePointer), green) ||
      !parseInteger(strtok_r(nullptr, " ", &savePointer), blue)) {
    return false;
  }
  if (red < 0 || red > 255 || green < 0 || green > 255 ||
      blue < 0 || blue > 255) {
    return false;
  }
  color = {
      static_cast<uint8_t>(red),
      static_cast<uint8_t>(green),
      static_cast<uint8_t>(blue),
  };
  return true;
}

bool parseKeyEffect(const char* token, KeyEffect& effect) {
  if (token == nullptr) {
    return false;
  }
  if (strcmp(token, "OFF") == 0) {
    effect = KeyEffect::Off;
    return true;
  }
  if (strcmp(token, "SOLID") == 0) {
    effect = KeyEffect::Solid;
    return true;
  }
  if (strcmp(token, "BREATHE") == 0) {
    effect = KeyEffect::Breathe;
    return true;
  }
  if (strcmp(token, "SNAKE") == 0) {
    effect = KeyEffect::Snake;
    return true;
  }
  if (strcmp(token, "RAINBOW") == 0) {
    effect = KeyEffect::Rainbow;
    return true;
  }
  if (strcmp(token, "GRADIENT") == 0) {
    effect = KeyEffect::Gradient;
    return true;
  }
  if (strcmp(token, "SHALLOW_BREATHE") == 0) {
    effect = KeyEffect::ShallowBreathe;
    return true;
  }
  return false;
}

bool parseAmbientEffect(const char* token, AmbientEffect& effect) {
  if (token == nullptr) {
    return false;
  }
  if (strcmp(token, "OFF") == 0) {
    effect = AmbientEffect::Off;
    return true;
  }
  if (strcmp(token, "SOLID") == 0) {
    effect = AmbientEffect::Solid;
    return true;
  }
  if (strcmp(token, "BREATHE") == 0) {
    effect = AmbientEffect::Breathe;
    return true;
  }
  if (strcmp(token, "FLOW") == 0) {
    effect = AmbientEffect::Snake;
    return true;
  }
  if (strcmp(token, "COLOR_FLOW") == 0) {
    effect = AmbientEffect::Rainbow;
    return true;
  }
  if (strcmp(token, "SNAKE") == 0) {
    effect = AmbientEffect::Snake;
    return true;
  }
  if (strcmp(token, "RAINBOW") == 0) {
    effect = AmbientEffect::Rainbow;
    return true;
  }
  if (strcmp(token, "GRADIENT") == 0) {
    effect = AmbientEffect::Gradient;
    return true;
  }
  if (strcmp(token, "SHALLOW_BREATHE") == 0) {
    effect = AmbientEffect::ShallowBreathe;
    return true;
  }
  return false;
}

void processSerialCommand(char* line) {
  char* savePointer = nullptr;
  const char* command = strtok_r(line, " ", &savePointer);
  if (command == nullptr) {
    return;
  }

  if (strcmp(command, "PING") == 0) {
    Serial.println("PONG LEDCTRL 1");
    return;
  }

  if (strcmp(command, "BRIGHTNESS") == 0) {
    int percent = 0;
    if (!parseInteger(strtok_r(nullptr, " ", &savePointer), percent) ||
        percent < 0 || percent > kMaximumBrightnessPercent) {
      Serial.println("ERR BRIGHTNESS RANGE_0_100");
      return;
    }
    globalBrightnessPercent = static_cast<uint8_t>(percent);
    Serial.printf("OK BRIGHTNESS %d\n", percent);
    return;
  }

  if (strcmp(command, "SPEED") == 0) {
    int requestedBreathMs = 0;
    int requestedFlowMs = 0;
    if (!parseInteger(
            strtok_r(nullptr, " ", &savePointer), requestedBreathMs) ||
        !parseInteger(
            strtok_r(nullptr, " ", &savePointer), requestedFlowMs) ||
        requestedBreathMs <
            static_cast<int>(kMinimumEffectPeriodMs) ||
        requestedBreathMs >
            static_cast<int>(kMaximumEffectPeriodMs) ||
        requestedFlowMs <
            static_cast<int>(kMinimumEffectPeriodMs) ||
        requestedFlowMs >
            static_cast<int>(kMaximumEffectPeriodMs)) {
      Serial.println("ERR SPEED RANGE_200_30000_MS");
      return;
    }
    breathPeriodMs = static_cast<uint32_t>(requestedBreathMs);
    ambientLapMs = static_cast<uint32_t>(requestedFlowMs);
    Serial.printf("OK SPEED %lu %lu\n",
                  static_cast<unsigned long>(breathPeriodMs),
                  static_cast<unsigned long>(ambientLapMs));
    return;
  }

  if (strcmp(command, "KEY") == 0) {
    int index = 0;
    KeyEffect effect = KeyEffect::Off;
    Rgb color = {};
    if (!parseInteger(strtok_r(nullptr, " ", &savePointer), index) ||
        index < 0 || index >= static_cast<int>(kKeyLedCount) ||
        !parseKeyEffect(strtok_r(nullptr, " ", &savePointer), effect) ||
        !parseRgb(savePointer, color)) {
      Serial.println("ERR KEY FORMAT");
      return;
    }
    keySettings[index] = {color, effect, 1.0f, -1.0f, 1.0f};
    Serial.printf("OK KEY %d\n", index);
    return;
  }

  if (strcmp(command, "AMBIENT") == 0) {
    AmbientEffect effect = AmbientEffect::Off;
    Rgb colorA = {};
    Rgb colorB = {};
    if (!parseAmbientEffect(
            strtok_r(nullptr, " ", &savePointer), effect) ||
        !parseRgb(savePointer, colorA) ||
        !parseRgb(savePointer, colorB)) {
      Serial.println("ERR AMBIENT FORMAT");
      return;
    }
    ambientSetting = {colorA, colorB, effect, 1.0f, -1.0f, 1.0f};
    Serial.println("OK AMBIENT");
    return;
  }

  if (strcmp(command, "ALL_OFF") == 0) {
    for (size_t index = 0; index < kKeyLedCount; ++index) {
      keySettings[index].effect = KeyEffect::Off;
    }
    ambientSetting.effect = AmbientEffect::Off;
    Serial.println("OK ALL_OFF");
    return;
  }

  Serial.println("ERR UNKNOWN_COMMAND");
}

void pollSerial() {
  while (Serial.available() > 0) {
    const char character = static_cast<char>(Serial.read());
    if (character == '\r') {
      continue;
    }
    if (character == '\n') {
      serialLine[serialLineLength] = '\0';
      processSerialCommand(serialLine);
      serialLineLength = 0;
      continue;
    }
    if (serialLineLength + 1 < sizeof(serialLine)) {
      serialLine[serialLineLength++] = character;
    } else {
      serialLineLength = 0;
      Serial.println("ERR LINE_TOO_LONG");
    }
  }
}

void renderAndSend(uint32_t nowMs) {
  renderKeyFrame(nowMs);
  renderAmbientFrame(nowMs);
  if (layerIndicatorActive) {
    if (nowMs - layerIndicatorStartedMs >= 800) layerIndicatorActive = false;
    else if (powerOffState == PowerOffState::Idle) {
      const Rgb color = scaleSystemColor({
        static_cast<uint8_t>(layerIndicatorColor >> 16),
        static_cast<uint8_t>(layerIndicatorColor >> 8),
        static_cast<uint8_t>(layerIndicatorColor)}, 0.5f);
      for (Rgb& pixel : ambientFrame) pixel = color;
    }
  }
  const bool keyOk =
      sendFrame(kKeyLedChannel, keyFrame, kKeyLedCount,
                kKeyLedTiming, keyRmtItems);
  const bool ambientOk =
      sendFrame(kAmbientLedChannel, ambientFrame, kAmbientLedCount,
                kAmbientLedTiming, ambientRmtItems);
  if (!keyOk || !ambientOk) {
    Serial.println("ERR LED_REFRESH");
  }
}

void sendAllLedsOff() {
  clearFrame(keyFrame, kKeyLedCount);
  clearFrame(ambientFrame, kAmbientLedCount);
  const bool keyOk =
      sendFrame(kKeyLedChannel, keyFrame, kKeyLedCount,
                kKeyLedTiming, keyRmtItems);
  const bool ambientOk =
      sendFrame(kAmbientLedChannel, ambientFrame, kAmbientLedCount,
                kAmbientLedTiming, ambientRmtItems);
  if (!keyOk || !ambientOk) {
    Serial.println("ERR LED_POWER_OFF");
  }
}

void playBleBondResetConfirmation() {
  clearFrame(keyFrame, kKeyLedCount);
  const float systemMaximum =
      static_cast<float>(kSystemIndicatorMaximum) / 255.0f;
  const Rgb red = scaleSystemColor({255, 0, 0}, systemMaximum);

  Serial.println("POWER BLE bond reset confirmation: two red flashes");
  for (uint8_t flash = 0; flash < kBleResetConfirmFlashCount; ++flash) {
    for (Rgb& pixel : ambientFrame) {
      pixel = red;
    }
    const bool keyOk =
        sendFrame(kKeyLedChannel, keyFrame, kKeyLedCount,
                  kKeyLedTiming, keyRmtItems);
    const bool ambientOk =
        sendFrame(kAmbientLedChannel, ambientFrame, kAmbientLedCount,
                  kAmbientLedTiming, ambientRmtItems);
    if (!keyOk || !ambientOk) {
      Serial.println("ERR BLE_RESET_CONFIRM_LED");
    }
    delay(kBleResetConfirmOnMs);
    sendAllLedsOff();
    delay(kBleResetConfirmOffMs);
  }
}

bool enableFiveVoltRail() {
  // GPIO14 may still be holding the LOW level retained through Deep-sleep.
  // Program the active configuration first so releasing the hold cannot
  // produce an unwanted pulse on the TPS22918 ON input.
  gpio_deep_sleep_hold_dis();
  esp_err_t result = gpio_set_direction(kFiveVoltEnablePin,
                                        GPIO_MODE_OUTPUT);
  if (result != ESP_OK) {
    Serial.printf("ERR 5V_ENABLE_DIRECTION %s\n",
                  esp_err_to_name(result));
    return false;
  }
  result = gpio_set_level(kFiveVoltEnablePin, 1);
  if (result != ESP_OK) {
    Serial.printf("ERR 5V_ENABLE_LEVEL %s\n",
                  esp_err_to_name(result));
    return false;
  }
  result = gpio_hold_dis(kFiveVoltEnablePin);
  if (result != ESP_OK) {
    Serial.printf("ERR 5V_ENABLE_RELEASE_HOLD %s\n",
                  esp_err_to_name(result));
    return false;
  }
  return true;
}

bool disableFiveVoltRailForDeepSleep() {
  esp_err_t result = gpio_set_level(kFiveVoltEnablePin, 0);
  if (result != ESP_OK) {
    Serial.printf("ERR 5V_DISABLE_LEVEL %s\n",
                  esp_err_to_name(result));
    return false;
  }
  result = gpio_hold_en(kFiveVoltEnablePin);
  if (result != ESP_OK) {
    Serial.printf("ERR 5V_DISABLE_HOLD %s\n",
                  esp_err_to_name(result));
    return false;
  }
  gpio_deep_sleep_hold_en();
  return true;
}

void waitForEncoderRelease() {
  while (true) {
    while (digitalRead(kEncoderPressPin) == LOW) {
      delay(10);
    }

    const uint32_t releasedSinceMs = millis();
    while (digitalRead(kEncoderPressPin) == HIGH &&
           millis() - releasedSinceMs < kButtonDebounceMs) {
      delay(1);
    }
    if (digitalRead(kEncoderPressPin) == HIGH) {
      return;
    }
  }
}

[[noreturn]] void enterDeepSleepAfterRelease() {
  // ext0 is level-triggered, not edge-triggered. Entering Deep-sleep while
  // PE10 is still LOW would wake the chip immediately, so finish the visible
  // animation, switch every LED off, then wait for a stable release.
  sendAllLedsOff();
  if (!disableFiveVoltRailForDeepSleep()) {
    Serial.println("ERR 5V_DISABLE_ABORTING_DEEP_SLEEP");
    while (true) {
      delay(1000);
    }
  }
  waitForEncoderRelease();
  if (encoderInterruptsAttached) {
    detachInterrupt(digitalPinToInterrupt(kEncoderAPin));
    detachInterrupt(digitalPinToInterrupt(kEncoderBPin));
    encoderInterruptsAttached = false;
  }

  const esp_err_t wakeResult = esp_sleep_enable_ext0_wakeup(
      static_cast<gpio_num_t>(kEncoderPressPin), 0);
  if (wakeResult != ESP_OK) {
    Serial.printf("ERR DEEP_SLEEP_WAKE %s\n",
                  esp_err_to_name(wakeResult));
    while (true) {
      delay(1000);
    }
  }

  Serial.println("POWER 5V off; entering Deep-sleep; hold encoder for three seconds to wake");
  Serial.flush();
  esp_deep_sleep_start();
  while (true) {
  }
}

bool confirmDeepSleepWakeHold() {
  startupAmbientState = StartupAmbientState::WakeHold;
  startupAmbientStartedMs = millis();
  uint32_t lastRenderedMs = startupAmbientStartedMs -
                            kPowerOnFrameIntervalMs;
  bool agent0ChordActive = false;
  uint32_t agent0HoldStartedMs = 0;

  while (digitalRead(kEncoderPressPin) == LOW) {
    const uint32_t nowMs = millis();
    if (nowMs - lastRenderedMs >= kPowerOnFrameIntervalMs) {
      lastRenderedMs = nowMs;
      renderAndSend(nowMs);
    }

    const bool agent0Pressed =
        (rawToSwitchMask(readShiftRegisters()) & kAgent0SwitchMask) != 0;
    const bool beforeNormalWakeThreshold =
        nowMs - startupAmbientStartedMs < kPowerOnHoldMs;
    if (agent0Pressed && !agent0ChordActive &&
        beforeNormalWakeThreshold) {
      agent0ChordActive = true;
      agent0HoldStartedMs = nowMs;
      suppressAgent0ReleaseAfterWake = true;
      Serial.println(
          "POWER Agent 0 wake chord armed; hold both for three seconds");
    } else if (!agent0Pressed && agent0ChordActive) {
      agent0ChordActive = false;
      Serial.println("POWER Agent 0 wake chord cancelled");
    }

    const bool resetChordComplete =
        agent0ChordActive &&
        nowMs - agent0HoldStartedMs >= kBleResetChordHoldMs;
    const bool normalWakeComplete =
        nowMs - startupAmbientStartedMs >= kPowerOnHoldMs;
    if (resetChordComplete ||
        (normalWakeComplete && !agent0ChordActive)) {
      // Three seconds is the normal acceptance point. A completed BLE reset
      // chord first confirms the destructive pairing reset with two perimeter
      // red flashes; both paths then show the normal green wake confirmation.
      // Button releases are consumed locally so the host never receives an
      // unmatched startup-chord release report.
      clearBleBondsOnStartup = resetChordComplete;
      if (resetChordComplete) {
        playBleBondResetConfirmation();
      }
      startupAmbientState = StartupAmbientState::WakeAccepted;
      suppressEncoderReleaseAfterWake = true;
      renderAndSend(nowMs);
      Serial.println(resetChordComplete
                         ? "POWER BLE bond reset chord accepted; starting system"
                         : "POWER three-second wake accepted; starting system immediately");
      return true;
    }
    delay(1);
  }

  if (millis() - startupAmbientStartedMs >= kPowerOnHoldMs) {
    // The normal wake hold was already satisfied, but the user released while
    // an unfinished Agent 0 chord was delaying startup. Boot normally without
    // deleting any pairing information.
    startupAmbientState = StartupAmbientState::WakeAccepted;
    suppressEncoderReleaseAfterWake = true;
    renderAndSend(millis());
    Serial.println(
        "POWER incomplete BLE reset chord; continuing normal startup");
    return true;
  }

  Serial.println("POWER wake hold cancelled; returning to Deep-sleep");
  startupAmbientState = StartupAmbientState::None;
  sendAllLedsOff();
  return false;
}

void emitReadOnlyMonitorSnapshot(uint32_t nowMs) {
#if CODEX_VERBOSE_DIAGNOSTICS
  if (nowMs - lastMonitorSnapshotMs < kMonitorSnapshotIntervalMs) {
    return;
  }
  lastMonitorSnapshotMs = nowMs;

  const CodexMicroState state = codex.snapshot();
  StaticJsonDocument<1024> document;
  document["t"] = nowMs;
  JsonArray threads = document.createNestedArray("threads");
  for (size_t index = 0; index < state.threads.size(); ++index) {
    const ThreadLight& source = state.threads[index];
    JsonObject target = threads.createNestedObject();
    target["id"] = index;
    target["c"] = source.color;
    target["b"] = source.brightness;
    target["e"] = source.effect;
    target["s"] = source.speed;
    target["sk"] = source.syncKeysLighting ? 1 : 0;
    target["sa"] = source.syncAmbientLighting ? 1 : 0;
  }
  JsonObject ambient = document.createNestedObject("ambient");
  ambient["c"] = state.ambient.color;
  ambient["b"] = state.ambient.brightness;
  ambient["e"] = state.ambient.effect;
  ambient["s"] = state.ambient.speed;
  ambient["m"] = state.ambient.magic;
  ambient["sync"] = state.ambientSyncThread;

  Serial.print("MON STATE ");
  serializeJson(document, Serial);
  Serial.println();
#else
  (void)nowMs;
#endif
}

}  // namespace

// Read-only RPC diagnostics: observes scanner state, never injects an input.
void codexMicroShowLayerIndicator(uint32_t color) {
  layerIndicatorColor = color;
  layerIndicatorStartedMs = millis();
  layerIndicatorActive = true;
}

void codexMicroInputSnapshot(JsonObject out) {
  out["raw"] = latestKeyRaw;
  out["scan_count"] = buttonScanCount;
  out["scan_max_gap_us"] = buttonScanMaxGapUs;
  out["scan_over_25ms"] = buttonScanOver25ms;
  out["scan_age_us"] = buttonScanCount == 0 ? 0 : micros() - buttonScanLastUs;
  out["stable_mask"] = keyStableMask;
  out["suppress_agent0"] = suppressAgent0ReleaseAfterWake;
  out["host_connected"] = bleState.connected;
  out["layer_indicator_active"] = layerIndicatorActive;
  JsonArray presses = out.createNestedArray("presses");
  JsonArray rawLows = out.createNestedArray("raw_lows");
  for (size_t i = 0; i < 16; ++i) rawLows.add(rawLowCounts[i]);
  JsonArray releases = out.createNestedArray("releases");
  for (size_t i = 0; i < 12; ++i) {
    presses.add(keyPressCounts[i]);
    releases.add(keyReleaseCounts[i]);
  }
}

// Public bridge for the HID service's Work axis internal brightness keys. The
// renderer keeps the actual value in its existing device-global setting so
// keycode actions and serial/RPC brightness controls stay in one state.
void codexMicroAdjustGlobalBrightness(int delta) {
  globalBrightnessPercent = static_cast<uint8_t>(constrain(
      static_cast<int>(globalBrightnessPercent) + delta, 0, 100));
}

void setup() {
  const bool cpuFrequencyReady =
      setCpuFrequencyMhz(kCpuFrequencyMhz);
  const esp_sleep_wakeup_cause_t wakeCause =
      esp_sleep_get_wakeup_cause();
  const bool wakingFromPowerButton =
      wakeCause == ESP_SLEEP_WAKEUP_EXT0;

  Serial.begin(115200);
  Serial.printf("POWER CPU frequency requested=%lu actual=%lu result=%s\n",
                static_cast<unsigned long>(kCpuFrequencyMhz),
                static_cast<unsigned long>(getCpuFrequencyMhz()),
                cpuFrequencyReady ? "ok" : "failed");
  const bool fiveVoltReady = enableFiveVoltRail();
  if (!fiveVoltReady) {
    Serial.println("ERR 5V_POWER_INIT");
    return;
  }
  if (!wakingFromPowerButton) {
    delay(1000);
  }

  // PE10 has a confirmed external 10K pull-up. Configure it before checking
  // an ext0 wake so a short release returns directly to Deep-sleep.
  pinMode(kEncoderPressPin, INPUT);

  const bool keyReady =
      initializeRmtChannel(kKeyLedPin, kKeyLedChannel);
  const bool ambientReady =
      initializeRmtChannel(kAmbientLedPin, kAmbientLedChannel);
  if (!keyReady || !ambientReady) {
    Serial.println("ERR RMT_INIT");
    return;
  }

  if (wakingFromPowerButton) {
    initializeShiftRegisterInputs();
    Serial.println("POWER Deep-sleep wake detected; verifying three-second hold");
    if (!confirmDeepSleepWakeHold()) {
      enterDeepSleepAfterRelease();
    }
  }

  initializeInputs();
  initializeBatteryMonitoring();
  updateBattery(true);

#if CODEX_BLE_ENABLED
  // A normal reset does not pass through the Deep-sleep wake animation. Enter
  // the BLE-search state before the first LED frame so a watchdog, brownout,
  // or USB-power restart cannot leave the perimeter showing the previous
  // full-ring frame while no host is connected.
  if (!wakingFromPowerButton) {
    startupAmbientState = StartupAmbientState::BleSearching;
    startupAmbientStartedMs = millis();
  }
#endif

  renderAndSend(millis());
  lastFrameUs = micros();
  codex.begin(clearBleBondsOnStartup);

#if CODEX_BLE_ENABLED
  // BLE advertising is started by codex.begin() on every boot. Keep the
  // system lighting in the matching search state until either USB or BLE is
  // actually connected; updateStartupAmbientState() then starts the green
  // connected breath.
  startupAmbientState = StartupAmbientState::BleSearching;
  startupAmbientStartedMs = millis();
  renderAndSend(startupAmbientStartedMs);
  lastFrameUs = micros();
  if (wakingFromPowerButton) {
    Serial.println("POWER wake accepted; BLE search animation started");
  } else {
    Serial.println("POWER reset startup; BLE search animation started");
  }
#else
  if (wakingFromPowerButton) {
    startupAmbientState = StartupAmbientState::None;
    renderAndSend(millis());
    lastFrameUs = micros();
    Serial.println(
        "POWER wake accepted; BLE disabled, normal program started");
  }
#endif
  Serial.println("READY LEDCTRL 1");
}

void loop() {
  // (Variant B) SPIFFS mount intentionally disabled while investigating USB
  // stability. Nothing to do here.
  const uint32_t loopNow = millis();

  pollSerial();
  codex.poll();
  codex.flushPendingAmbientPersist();
  pollBleLighting();
  pollInputs();
  updateBattery();
  updateStartupAmbientState(loopNow);
  emitReadOnlyMonitorSnapshot(loopNow);

  const uint32_t nowUs = micros();
  if (nowUs - lastFrameUs >= kFrameIntervalUs) {
    lastFrameUs += kFrameIntervalUs;
    renderAndSend(loopNow);
  }

  delayMicroseconds(100);
}
