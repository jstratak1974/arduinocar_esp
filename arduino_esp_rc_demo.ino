/*
  Arduino ESP-Like Stability Controller for an Educational RC/Robot Car
  ----------------------------------------------------------------------

  This sketch demonstrates the basic feedback idea behind Electronic
  Stability Control (ESC), also commonly called ESP:

    1. Estimate what yaw rate the driver requests from steering and speed.
    2. Measure or simulate the car's actual yaw rate.
    3. Compare requested and measured yaw.
    4. Reduce total drive torque and selectively reduce the left or right
       motor command to create a corrective yaw moment.

  The target platform is a LOW-SPEED model car with:
    - Arduino Uno/Nano-compatible board
    - two independently driven rear/side DC motors
    - optional front steering servo
    - steering and throttle potentiometers
    - physical mode: MPU-6050 IMU and two wheel-speed pulse sensors
    - bench mode: software vehicle model and yaw-disturbance potentiometer

  IMPORTANT SAFETY LIMITATION
  ---------------------------
  This is an educational model-car controller. It is NOT automotive-grade
  software, ABS, traction control, or a road-vehicle ESP implementation.
  Never connect it to a real vehicle's CAN bus, brakes, steering, throttle,
  ABS modulator, or powertrain. Test with the model raised off the floor first,
  then only at walking speed in a clear indoor area.

  Author: John Stratakis
  Website: https://med-physics.com
  Version: 1.0.0
*/

#include <Arduino.h>
#include <Servo.h>
#include <Wire.h>


// -----------------------------------------------------------------------------
// OPERATING MODE
// -----------------------------------------------------------------------------

// true  = runs without an IMU or wheel sensors. A2 injects a yaw disturbance.
// false = uses MPU-6050 yaw rate and wheel-speed sensors on D2/D3.
constexpr bool BENCH_SIMULATION_MODE = true;

// Enable the physical steering-servo output. Set false if the platform uses
// differential steering only or if no steering servo is connected.
constexpr bool USE_STEERING_SERVO = true;


// -----------------------------------------------------------------------------
// PIN ASSIGNMENTS — Arduino Uno/Nano
// -----------------------------------------------------------------------------

constexpr uint8_t PIN_WHEEL_LEFT = 2;       // Interrupt input, physical mode
constexpr uint8_t PIN_WHEEL_RIGHT = 3;      // Interrupt input, physical mode
constexpr uint8_t PIN_ARM_SWITCH = 4;       // Active LOW; connect to GND to arm
constexpr uint8_t PIN_MOTOR_LEFT_PWM = 5;   // PWM
constexpr uint8_t PIN_MOTOR_RIGHT_PWM = 6;  // PWM
constexpr uint8_t PIN_MOTOR_LEFT_DIR = 7;
constexpr uint8_t PIN_MOTOR_RIGHT_DIR = 8;
constexpr uint8_t PIN_STEERING_SERVO = 9;
constexpr uint8_t PIN_LED_ESP_ACTIVE = 10;
constexpr uint8_t PIN_LED_LEFT_CORRECTION = 11;
constexpr uint8_t PIN_LED_RIGHT_CORRECTION = 12;
constexpr uint8_t PIN_LED_FAULT = 13;

constexpr uint8_t PIN_STEERING_INPUT = A0;
constexpr uint8_t PIN_THROTTLE_INPUT = A1;
constexpr uint8_t PIN_SIM_YAW_DISTURBANCE = A2;


// -----------------------------------------------------------------------------
// VEHICLE AND SENSOR PARAMETERS — MEASURE AND TUNE FOR YOUR MODEL
// -----------------------------------------------------------------------------

constexpr float GRAVITY_MPS2 = 9.80665f;
constexpr float WHEELBASE_M = 0.260f;
constexpr float TRACK_WIDTH_M = 0.180f;
constexpr float WHEEL_RADIUS_M = 0.0325f;
constexpr uint16_t WHEEL_PULSES_PER_REV = 20;

// The controller is disabled below this speed because yaw prediction becomes
// noisy and unnecessary. This is a model-car value, not a road-car threshold.
constexpr float MIN_ESP_SPEED_MPS = 0.35f;
constexpr float MAX_DESIRED_YAW_RAD_S = 3.50f;
constexpr float ASSUMED_FRICTION_COEFFICIENT = 0.70f;

// Steering calibration. Positive steering angle means left turn / positive yaw.
constexpr int STEERING_ADC_LEFT = 100;
constexpr int STEERING_ADC_CENTER = 512;
constexpr int STEERING_ADC_RIGHT = 924;
constexpr int STEERING_ADC_DEADBAND = 8;
constexpr float MAX_STEERING_ANGLE_DEG = 25.0f;

// Servo calibration. Change STEERING_SERVO_REVERSED if steering moves opposite
// to the input. Keep the endpoints within the safe mechanical range.
constexpr int STEERING_SERVO_CENTER_DEG = 90;
constexpr int STEERING_SERVO_RANGE_DEG = 35;
constexpr bool STEERING_SERVO_REVERSED = false;

// Throttle calibration. The model supports forward drive only.
constexpr int THROTTLE_ADC_MIN = 40;
constexpr int THROTTLE_ADC_MAX = 980;
constexpr uint8_t THROTTLE_PWM_DEADBAND = 12;

constexpr bool LEFT_MOTOR_DIRECTION_REVERSED = false;
constexpr bool RIGHT_MOTOR_DIRECTION_REVERSED = false;


// -----------------------------------------------------------------------------
// CONTROLLER PARAMETERS
// -----------------------------------------------------------------------------

constexpr uint32_t CONTROL_INTERVAL_US = 20000UL;   // 50 Hz
constexpr uint32_t WHEEL_UPDATE_INTERVAL_MS = 100;  // 10 Hz speed update
constexpr uint32_t TELEMETRY_INTERVAL_MS = 100;     // 10 Hz serial output

// First-order filter coefficients: 0 = no new information, 1 = no filtering.
constexpr float YAW_FILTER_ALPHA = 0.28f;
constexpr float LATERAL_ACCEL_FILTER_ALPHA = 0.20f;
constexpr float WHEEL_SPEED_FILTER_ALPHA = 0.35f;
constexpr float DERIVATIVE_FILTER_ALPHA = 0.18f;

// Activation threshold includes a fixed term and a term proportional to the
// requested yaw rate. Hysteresis prevents rapid on/off switching.
constexpr float YAW_ERROR_BASE_ON_RAD_S = 0.22f;
constexpr float YAW_ERROR_RELATIVE_ON = 0.18f;
constexpr float YAW_ERROR_OFF_RATIO = 0.58f;
constexpr uint32_t ESP_MIN_ACTIVE_TIME_MS = 180;

// PD controller converts yaw-rate error into a 0..1 selective torque cut.
// Start conservatively. Increase KP slowly during low-speed testing.
constexpr float ESP_KP = 0.78f;
constexpr float ESP_KD = 0.025f;
constexpr float MAX_SELECTIVE_TORQUE_CUT = 0.68f;
constexpr float MAX_GLOBAL_TORQUE_CUT = 0.38f;
constexpr float GLOBAL_CUT_FULL_ERROR_RAD_S = 1.20f;

// Limit actuator command changes to prevent abrupt model-car motion.
constexpr float PWM_SLEW_PER_SECOND = 520.0f;

// Reject electrical bounce/noise on wheel-pulse inputs.
constexpr uint32_t WHEEL_PULSE_DEBOUNCE_US = 400UL;


// -----------------------------------------------------------------------------
// BENCH SIMULATION PARAMETERS
// -----------------------------------------------------------------------------

constexpr float SIM_MAX_SPEED_MPS = 2.40f;
constexpr float SIM_SPEED_TIME_CONSTANT_S = 0.55f;
constexpr float SIM_YAW_TIME_CONSTANT_S = 0.24f;
constexpr float SIM_MAX_EXTERNAL_YAW_DISTURBANCE_RAD_S = 1.80f;
constexpr float SIM_DIFFERENTIAL_YAW_GAIN_RAD_S = 2.20f;


// -----------------------------------------------------------------------------
// MPU-6050 REGISTERS AND SCALE FACTORS
// -----------------------------------------------------------------------------

constexpr uint8_t MPU_ADDRESS_LOW = 0x68;
constexpr uint8_t MPU_ADDRESS_HIGH = 0x69;
constexpr uint8_t MPU_REG_SMPLRT_DIV = 0x19;
constexpr uint8_t MPU_REG_CONFIG = 0x1A;
constexpr uint8_t MPU_REG_GYRO_CONFIG = 0x1B;
constexpr uint8_t MPU_REG_ACCEL_CONFIG = 0x1C;
constexpr uint8_t MPU_REG_ACCEL_XOUT_H = 0x3B;
constexpr uint8_t MPU_REG_PWR_MGMT_1 = 0x6B;
constexpr uint8_t MPU_REG_WHO_AM_I = 0x75;

// ±250 degrees/s and ±2 g full-scale settings.
constexpr float MPU_GYRO_LSB_PER_DEG_S = 131.0f;
constexpr float MPU_ACCEL_LSB_PER_G = 16384.0f;
constexpr uint16_t GYRO_CALIBRATION_SAMPLES = 1200;
constexpr uint8_t MAX_CONSECUTIVE_IMU_FAILURES = 8;


// -----------------------------------------------------------------------------
// TYPES AND GLOBAL STATE
// -----------------------------------------------------------------------------

enum class StabilityState : uint8_t {
  STABLE,
  UNDERSTEER,
  OVERSTEER,
  SPIN_RECOVERY
};

enum class FaultCode : uint8_t {
  NONE,
  IMU_NOT_FOUND,
  IMU_CONFIGURATION_FAILED,
  IMU_CALIBRATION_FAILED,
  IMU_READ_FAILED
};

struct RawImuData {
  int16_t accelX;
  int16_t accelY;
  int16_t accelZ;
  int16_t temperature;
  int16_t gyroX;
  int16_t gyroY;
  int16_t gyroZ;
};

struct VehicleSignals {
  float leftSpeedMps;
  float rightSpeedMps;
  float vehicleSpeedMps;
  float yawRateRadS;
  float lateralAccelMps2;
  float wheelSlipRatio;
};

struct ControlSignals {
  float steeringAngleDeg;
  float desiredYawRateRadS;
  float yawErrorRadS;
  float yawErrorDerivativeRadS2;
  float correctionDemand;
  float globalTorqueCut;
  uint8_t requestedThrottlePwm;
  uint8_t leftMotorPwm;
  uint8_t rightMotorPwm;
  bool espActive;
  bool leftCorrection;
  bool rightCorrection;
  StabilityState stabilityState;
};

Servo steeringServo;

volatile uint32_t leftWheelPulseCount = 0;
volatile uint32_t rightWheelPulseCount = 0;
volatile uint32_t leftWheelLastPulseUs = 0;
volatile uint32_t rightWheelLastPulseUs = 0;

uint8_t mpuAddress = MPU_ADDRESS_LOW;
float gyroZBiasLsb = 0.0f;
uint8_t consecutiveImuFailures = 0;

VehicleSignals vehicle = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
ControlSignals control = {
  0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
  0, 0, 0, false, false, false, StabilityState::STABLE
};

FaultCode faultCode = FaultCode::NONE;
bool waitingForDisarmedStart = true;
bool telemetryEnabled = true;
bool telemetryHeaderPrinted = false;
bool physicalSensorsReady = false;
bool wheelInterruptsAttached = false;

uint32_t lastControlUs = 0;
uint32_t lastWheelUpdateMs = 0;
uint32_t lastTelemetryMs = 0;
uint32_t espActivatedMs = 0;

float filteredYawRateRadS = 0.0f;
float filteredLateralAccelMps2 = 0.0f;
float filteredYawErrorDerivative = 0.0f;
float previousYawErrorRadS = 0.0f;
float appliedLeftPwm = 0.0f;
float appliedRightPwm = 0.0f;

// Bench model state.
float simulatedSpeedMps = 0.0f;
float simulatedYawRateRadS = 0.0f;


// -----------------------------------------------------------------------------
// SMALL UTILITY FUNCTIONS
// -----------------------------------------------------------------------------

float clampFloat(float value, float minimum, float maximum) {
  if (value < minimum) return minimum;
  if (value > maximum) return maximum;
  return value;
}

float lowPass(float previous, float input, float alpha) {
  return previous + alpha * (input - previous);
}

float moveToward(float current, float target, float maximumStep) {
  if (target > current + maximumStep) return current + maximumStep;
  if (target < current - maximumStep) return current - maximumStep;
  return target;
}

bool isArmed() {
  return digitalRead(PIN_ARM_SWITCH) == LOW;
}

const __FlashStringHelper* stabilityStateName(StabilityState state) {
  switch (state) {
    case StabilityState::STABLE:
      return F("STABLE");
    case StabilityState::UNDERSTEER:
      return F("UNDERSTEER");
    case StabilityState::OVERSTEER:
      return F("OVERSTEER");
    case StabilityState::SPIN_RECOVERY:
      return F("SPIN_RECOVERY");
  }
  return F("UNKNOWN");
}

const __FlashStringHelper* faultName(FaultCode code) {
  switch (code) {
    case FaultCode::NONE:
      return F("NONE");
    case FaultCode::IMU_NOT_FOUND:
      return F("IMU_NOT_FOUND");
    case FaultCode::IMU_CONFIGURATION_FAILED:
      return F("IMU_CONFIGURATION_FAILED");
    case FaultCode::IMU_CALIBRATION_FAILED:
      return F("IMU_CALIBRATION_FAILED");
    case FaultCode::IMU_READ_FAILED:
      return F("IMU_READ_FAILED");
  }
  return F("UNKNOWN");
}


// -----------------------------------------------------------------------------
// WHEEL-SPEED INTERRUPTS AND SPEED ESTIMATION
// -----------------------------------------------------------------------------

void onLeftWheelPulse() {
  const uint32_t nowUs = micros();
  if (nowUs - leftWheelLastPulseUs >= WHEEL_PULSE_DEBOUNCE_US) {
    ++leftWheelPulseCount;
    leftWheelLastPulseUs = nowUs;
  }
}

void onRightWheelPulse() {
  const uint32_t nowUs = micros();
  if (nowUs - rightWheelLastPulseUs >= WHEEL_PULSE_DEBOUNCE_US) {
    ++rightWheelPulseCount;
    rightWheelLastPulseUs = nowUs;
  }
}

void updatePhysicalWheelSpeeds(uint32_t nowMs) {
  const uint32_t elapsedMs = nowMs - lastWheelUpdateMs;
  if (elapsedMs < WHEEL_UPDATE_INTERVAL_MS) return;

  noInterrupts();
  const uint32_t leftPulses = leftWheelPulseCount;
  const uint32_t rightPulses = rightWheelPulseCount;
  leftWheelPulseCount = 0;
  rightWheelPulseCount = 0;
  interrupts();

  const float elapsedSeconds = elapsedMs * 0.001f;
  const float wheelCircumferenceM = 2.0f * PI * WHEEL_RADIUS_M;
  const float leftRawMps =
      (leftPulses / static_cast<float>(WHEEL_PULSES_PER_REV)) *
      wheelCircumferenceM / elapsedSeconds;
  const float rightRawMps =
      (rightPulses / static_cast<float>(WHEEL_PULSES_PER_REV)) *
      wheelCircumferenceM / elapsedSeconds;

  vehicle.leftSpeedMps = lowPass(
      vehicle.leftSpeedMps, leftRawMps, WHEEL_SPEED_FILTER_ALPHA);
  vehicle.rightSpeedMps = lowPass(
      vehicle.rightSpeedMps, rightRawMps, WHEEL_SPEED_FILTER_ALPHA);
  vehicle.vehicleSpeedMps =
      0.5f * (vehicle.leftSpeedMps + vehicle.rightSpeedMps);

  const float denominator = max(vehicle.vehicleSpeedMps, 0.10f);
  vehicle.wheelSlipRatio =
      fabsf(vehicle.leftSpeedMps - vehicle.rightSpeedMps) / denominator;

  lastWheelUpdateMs = nowMs;
}


// -----------------------------------------------------------------------------
// MPU-6050 LOW-LEVEL DRIVER — NO THIRD-PARTY SENSOR LIBRARY REQUIRED
// -----------------------------------------------------------------------------

bool mpuWriteRegister(uint8_t registerAddress, uint8_t value) {
  Wire.beginTransmission(mpuAddress);
  Wire.write(registerAddress);
  Wire.write(value);
  return Wire.endTransmission(true) == 0;
}

bool mpuReadRegister(uint8_t registerAddress, uint8_t& value) {
  Wire.beginTransmission(mpuAddress);
  Wire.write(registerAddress);
  if (Wire.endTransmission(false) != 0) return false;

  const uint8_t received = Wire.requestFrom(
      mpuAddress, static_cast<uint8_t>(1), static_cast<uint8_t>(true));
  if (received != 1 || Wire.available() < 1) return false;
  value = Wire.read();
  return true;
}

bool mpuAddressResponds(uint8_t address) {
  Wire.beginTransmission(address);
  return Wire.endTransmission(true) == 0;
}

bool readMpuRaw(RawImuData& data) {
  Wire.beginTransmission(mpuAddress);
  Wire.write(MPU_REG_ACCEL_XOUT_H);
  if (Wire.endTransmission(false) != 0) return false;

  const uint8_t received = Wire.requestFrom(
      mpuAddress, static_cast<uint8_t>(14), static_cast<uint8_t>(true));
  if (received != 14 || Wire.available() < 14) return false;

  data.accelX = static_cast<int16_t>((Wire.read() << 8) | Wire.read());
  data.accelY = static_cast<int16_t>((Wire.read() << 8) | Wire.read());
  data.accelZ = static_cast<int16_t>((Wire.read() << 8) | Wire.read());
  data.temperature = static_cast<int16_t>((Wire.read() << 8) | Wire.read());
  data.gyroX = static_cast<int16_t>((Wire.read() << 8) | Wire.read());
  data.gyroY = static_cast<int16_t>((Wire.read() << 8) | Wire.read());
  data.gyroZ = static_cast<int16_t>((Wire.read() << 8) | Wire.read());
  return true;
}

bool initializeMpu() {
  if (mpuAddressResponds(MPU_ADDRESS_LOW)) {
    mpuAddress = MPU_ADDRESS_LOW;
  } else if (mpuAddressResponds(MPU_ADDRESS_HIGH)) {
    mpuAddress = MPU_ADDRESS_HIGH;
  } else {
    faultCode = FaultCode::IMU_NOT_FOUND;
    return false;
  }

  uint8_t identity = 0;
  if (!mpuReadRegister(MPU_REG_WHO_AM_I, identity)) {
    faultCode = FaultCode::IMU_NOT_FOUND;
    return false;
  }
  if (identity != 0x68 && identity != 0x69) {
    faultCode = FaultCode::IMU_NOT_FOUND;
    return false;
  }

  // Clock source: X-axis gyroscope PLL; wake device.
  if (!mpuWriteRegister(MPU_REG_PWR_MGMT_1, 0x01)) {
    faultCode = FaultCode::IMU_CONFIGURATION_FAILED;
    return false;
  }
  delay(100);

  // 1 kHz internal sample / (1 + 4) = 200 Hz output.
  const bool configured =
      mpuWriteRegister(MPU_REG_SMPLRT_DIV, 0x04) &&
      mpuWriteRegister(MPU_REG_CONFIG, 0x03) &&       // DLPF around 42–44 Hz
      mpuWriteRegister(MPU_REG_GYRO_CONFIG, 0x00) &&  // ±250 deg/s
      mpuWriteRegister(MPU_REG_ACCEL_CONFIG, 0x00);   // ±2 g

  if (!configured) {
    faultCode = FaultCode::IMU_CONFIGURATION_FAILED;
    return false;
  }
  return true;
}

bool calibrateGyro() {
  if (isArmed()) {
    Serial.println(F("Calibration refused: disarm the model first."));
    return false;
  }

  Serial.println(F("Keep the model completely still: calibrating gyro..."));
  double sum = 0.0;
  double sumSquares = 0.0;
  RawImuData raw;

  for (uint16_t sample = 0; sample < GYRO_CALIBRATION_SAMPLES; ++sample) {
    if (!readMpuRaw(raw)) {
      faultCode = FaultCode::IMU_CALIBRATION_FAILED;
      return false;
    }
    const double value = raw.gyroZ;
    sum += value;
    sumSquares += value * value;
    delay(2);
  }

  gyroZBiasLsb = static_cast<float>(sum / GYRO_CALIBRATION_SAMPLES);
  const double meanSquare = sumSquares / GYRO_CALIBRATION_SAMPLES;
  const double variance = max(0.0, meanSquare - gyroZBiasLsb * gyroZBiasLsb);
  const float standardDeviationDegS =
      static_cast<float>(sqrt(variance) / MPU_GYRO_LSB_PER_DEG_S);

  Serial.print(F("Gyro Z bias [LSB]: "));
  Serial.println(gyroZBiasLsb, 3);
  Serial.print(F("Stationary gyro noise SD [deg/s]: "));
  Serial.println(standardDeviationDegS, 3);

  if (standardDeviationDegS > 2.5f) {
    Serial.println(F("Calibration failed: model moved or sensor is too noisy."));
    faultCode = FaultCode::IMU_CALIBRATION_FAILED;
    return false;
  }

  faultCode = FaultCode::NONE;
  Serial.println(F("Gyro calibration complete."));
  return true;
}

bool updatePhysicalImu() {
  RawImuData raw;
  if (!readMpuRaw(raw)) {
    if (consecutiveImuFailures < 255) ++consecutiveImuFailures;
    if (consecutiveImuFailures >= MAX_CONSECUTIVE_IMU_FAILURES) {
      faultCode = FaultCode::IMU_READ_FAILED;
    }
    return false;
  }

  consecutiveImuFailures = 0;
  const float yawRateDegS =
      (raw.gyroZ - gyroZBiasLsb) / MPU_GYRO_LSB_PER_DEG_S;
  const float yawRateRadS = yawRateDegS * DEG_TO_RAD;
  const float lateralAccelMps2 =
      (raw.accelY / MPU_ACCEL_LSB_PER_G) * GRAVITY_MPS2;

  filteredYawRateRadS = lowPass(
      filteredYawRateRadS, yawRateRadS, YAW_FILTER_ALPHA);
  filteredLateralAccelMps2 = lowPass(
      filteredLateralAccelMps2,
      lateralAccelMps2,
      LATERAL_ACCEL_FILTER_ALPHA);

  vehicle.yawRateRadS = filteredYawRateRadS;
  vehicle.lateralAccelMps2 = filteredLateralAccelMps2;
  return true;
}


// -----------------------------------------------------------------------------
// DRIVER INPUTS AND STEERING OUTPUT
// -----------------------------------------------------------------------------

float readSteeringAngleDeg() {
  const int raw = analogRead(PIN_STEERING_INPUT);
  const int difference = raw - STEERING_ADC_CENTER;
  if (abs(difference) <= STEERING_ADC_DEADBAND) return 0.0f;

  float normalized = 0.0f;
  if (raw < STEERING_ADC_CENTER) {
    normalized =
        (STEERING_ADC_CENTER - raw) /
        static_cast<float>(STEERING_ADC_CENTER - STEERING_ADC_LEFT);
    normalized = clampFloat(normalized, 0.0f, 1.0f);
  } else {
    normalized =
        -(raw - STEERING_ADC_CENTER) /
        static_cast<float>(STEERING_ADC_RIGHT - STEERING_ADC_CENTER);
    normalized = clampFloat(normalized, -1.0f, 0.0f);
  }
  return normalized * MAX_STEERING_ANGLE_DEG;
}

uint8_t readThrottlePwm() {
  const int raw = analogRead(PIN_THROTTLE_INPUT);
  if (raw <= THROTTLE_ADC_MIN) return 0;
  const int constrainedRaw = constrain(raw, THROTTLE_ADC_MIN, THROTTLE_ADC_MAX);
  const long mapped = map(
      constrainedRaw, THROTTLE_ADC_MIN, THROTTLE_ADC_MAX, 0, 255);
  if (mapped < THROTTLE_PWM_DEADBAND) return 0;
  return static_cast<uint8_t>(constrain(mapped, 0L, 255L));
}

void updateSteeringServo(float steeringAngleDeg) {
  if (!USE_STEERING_SERVO) return;
  float normalized = steeringAngleDeg / MAX_STEERING_ANGLE_DEG;
  normalized = clampFloat(normalized, -1.0f, 1.0f);
  if (STEERING_SERVO_REVERSED) normalized = -normalized;
  const int command = STEERING_SERVO_CENTER_DEG +
      static_cast<int>(roundf(normalized * STEERING_SERVO_RANGE_DEG));
  steeringServo.write(constrain(command, 0, 180));
}


// -----------------------------------------------------------------------------
// BENCH VEHICLE MODEL
// -----------------------------------------------------------------------------

void updateBenchSimulation(float dtSeconds, uint8_t requestedThrottlePwm,
                           float steeringAngleDeg, bool armed) {
  const float throttleFraction =
      armed ? requestedThrottlePwm / 255.0f : 0.0f;
  const float targetSpeedMps = throttleFraction * SIM_MAX_SPEED_MPS;
  const float speedAlpha = clampFloat(
      dtSeconds / max(SIM_SPEED_TIME_CONSTANT_S, dtSeconds), 0.0f, 1.0f);
  simulatedSpeedMps += speedAlpha * (targetSpeedMps - simulatedSpeedMps);

  const float steeringRad = steeringAngleDeg * DEG_TO_RAD;
  float kinematicYawRadS = 0.0f;
  if (fabsf(WHEELBASE_M) > 0.001f) {
    kinematicYawRadS =
        (simulatedSpeedMps / WHEELBASE_M) * tanf(steeringRad);
  }

  // A2 center means no disturbance. Moving it either side injects a positive
  // or negative yaw-rate disturbance. During a turn, one direction can mimic
  // understeer and the other oversteer; during straight travel it mimics a spin
  // or side disturbance.
  const int disturbanceAdc = analogRead(PIN_SIM_YAW_DISTURBANCE);
  const float disturbanceNormalized =
      (disturbanceAdc - 512) / 511.0f;
  const float externalYawDisturbance = clampFloat(
      disturbanceNormalized, -1.0f, 1.0f) *
      SIM_MAX_EXTERNAL_YAW_DISTURBANCE_RAD_S;

  // Differential motor command creates a corrective yaw tendency.
  const float differentialCommand =
      (appliedRightPwm - appliedLeftPwm) / 255.0f;
  const float controllerYaw =
      differentialCommand * SIM_DIFFERENTIAL_YAW_GAIN_RAD_S;

  const float targetYawRate =
      kinematicYawRadS + externalYawDisturbance + controllerYaw;
  const float yawAlpha = clampFloat(
      dtSeconds / max(SIM_YAW_TIME_CONSTANT_S, dtSeconds), 0.0f, 1.0f);
  simulatedYawRateRadS +=
      yawAlpha * (targetYawRate - simulatedYawRateRadS);

  vehicle.vehicleSpeedMps = max(0.0f, simulatedSpeedMps);
  vehicle.yawRateRadS = simulatedYawRateRadS;
  vehicle.lateralAccelMps2 =
      vehicle.vehicleSpeedMps * vehicle.yawRateRadS;

  const float halfTrackSpeedDifference =
      0.5f * TRACK_WIDTH_M * vehicle.yawRateRadS;
  vehicle.leftSpeedMps = max(
      0.0f, vehicle.vehicleSpeedMps - halfTrackSpeedDifference);
  vehicle.rightSpeedMps = max(
      0.0f, vehicle.vehicleSpeedMps + halfTrackSpeedDifference);
  const float denominator = max(vehicle.vehicleSpeedMps, 0.10f);
  vehicle.wheelSlipRatio =
      fabsf(vehicle.leftSpeedMps - vehicle.rightSpeedMps) / denominator;
}


// -----------------------------------------------------------------------------
// ESP-LIKE REFERENCE MODEL AND STABILITY CLASSIFICATION
// -----------------------------------------------------------------------------

float calculateDesiredYawRate(float speedMps, float steeringAngleDeg) {
  if (speedMps < 0.01f) return 0.0f;

  const float steeringRad = steeringAngleDeg * DEG_TO_RAD;
  float desiredYaw = (speedMps / WHEELBASE_M) * tanf(steeringRad);

  // Simplified friction-circle limit: |yaw rate| <= mu*g/v.
  const float frictionLimitedYaw =
      (ASSUMED_FRICTION_COEFFICIENT * GRAVITY_MPS2) / max(speedMps, 0.20f);
  const float absoluteLimit = min(
      frictionLimitedYaw, MAX_DESIRED_YAW_RAD_S);
  desiredYaw = clampFloat(desiredYaw, -absoluteLimit, absoluteLimit);
  return desiredYaw;
}

StabilityState classifyStability(float desiredYaw, float measuredYaw,
                                 float threshold) {
  const float desiredMagnitude = fabsf(desiredYaw);
  const float measuredMagnitude = fabsf(measuredYaw);

  if (desiredMagnitude < 0.10f) {
    return measuredMagnitude > threshold
        ? StabilityState::SPIN_RECOVERY
        : StabilityState::STABLE;
  }

  if (desiredYaw * measuredYaw < -0.02f) {
    return StabilityState::SPIN_RECOVERY;
  }
  if (measuredMagnitude < max(0.0f, desiredMagnitude - threshold)) {
    return StabilityState::UNDERSTEER;
  }
  if (measuredMagnitude > desiredMagnitude + threshold) {
    return StabilityState::OVERSTEER;
  }
  return StabilityState::STABLE;
}


// -----------------------------------------------------------------------------
// MOTOR OUTPUT AND FAIL-SAFE HANDLING
// -----------------------------------------------------------------------------

void writeOneMotor(uint8_t pwmPin, uint8_t directionPin, uint8_t pwm,
                   bool directionReversed) {
  digitalWrite(directionPin, directionReversed ? LOW : HIGH);
  analogWrite(pwmPin, pwm);
}

void writeMotorOutputs(uint8_t leftPwm, uint8_t rightPwm) {
  writeOneMotor(
      PIN_MOTOR_LEFT_PWM, PIN_MOTOR_LEFT_DIR,
      leftPwm, LEFT_MOTOR_DIRECTION_REVERSED);
  writeOneMotor(
      PIN_MOTOR_RIGHT_PWM, PIN_MOTOR_RIGHT_DIR,
      rightPwm, RIGHT_MOTOR_DIRECTION_REVERSED);
}

void stopMotorsImmediately() {
  appliedLeftPwm = 0.0f;
  appliedRightPwm = 0.0f;
  control.leftMotorPwm = 0;
  control.rightMotorPwm = 0;
  writeMotorOutputs(0, 0);
}

void updateIndicatorLeds() {
  digitalWrite(PIN_LED_ESP_ACTIVE, control.espActive ? HIGH : LOW);
  digitalWrite(
      PIN_LED_LEFT_CORRECTION, control.leftCorrection ? HIGH : LOW);
  digitalWrite(
      PIN_LED_RIGHT_CORRECTION, control.rightCorrection ? HIGH : LOW);
  digitalWrite(PIN_LED_FAULT, faultCode != FaultCode::NONE ? HIGH : LOW);
}

void resetControllerState() {
  control.desiredYawRateRadS = 0.0f;
  control.yawErrorRadS = 0.0f;
  control.yawErrorDerivativeRadS2 = 0.0f;
  control.correctionDemand = 0.0f;
  control.globalTorqueCut = 0.0f;
  control.espActive = false;
  control.leftCorrection = false;
  control.rightCorrection = false;
  control.stabilityState = StabilityState::STABLE;
  previousYawErrorRadS = 0.0f;
  filteredYawErrorDerivative = 0.0f;
}

bool completePhysicalSensorStartup() {
  if (BENCH_SIMULATION_MODE) return true;
  if (isArmed()) return false;
  if (!calibrateGyro()) return false;

  if (!wheelInterruptsAttached) {
    attachInterrupt(
        digitalPinToInterrupt(PIN_WHEEL_LEFT), onLeftWheelPulse, FALLING);
    attachInterrupt(
        digitalPinToInterrupt(PIN_WHEEL_RIGHT), onRightWheelPulse, FALLING);
    wheelInterruptsAttached = true;
  }
  physicalSensorsReady = true;
  return true;
}


// -----------------------------------------------------------------------------
// MAIN CONTROL STEP
// -----------------------------------------------------------------------------

void runControlStep(float dtSeconds) {
  const uint32_t nowMs = millis();
  const bool armed = isArmed();

  control.steeringAngleDeg = readSteeringAngleDeg();
  control.requestedThrottlePwm = readThrottlePwm();
  updateSteeringServo(control.steeringAngleDeg);

  if (waitingForDisarmedStart) {
    if (!armed) {
      if (!BENCH_SIMULATION_MODE && !physicalSensorsReady) {
        stopMotorsImmediately();
        if (!completePhysicalSensorStartup()) {
          updateIndicatorLeds();
          return;
        }
      }
      waitingForDisarmedStart = false;
      Serial.println(F("Safe start confirmed. ARM switch may now be enabled."));
    } else {
      stopMotorsImmediately();
      resetControllerState();
      updateIndicatorLeds();
      return;
    }
  }

  if (BENCH_SIMULATION_MODE) {
    updateBenchSimulation(
        dtSeconds,
        control.requestedThrottlePwm,
        control.steeringAngleDeg,
        armed);
  } else {
    updatePhysicalWheelSpeeds(nowMs);
    updatePhysicalImu();
  }

  if (faultCode != FaultCode::NONE) {
    stopMotorsImmediately();
    resetControllerState();
    updateIndicatorLeds();
    return;
  }

  if (!armed) {
    stopMotorsImmediately();
    resetControllerState();
    updateIndicatorLeds();
    return;
  }

  control.desiredYawRateRadS = calculateDesiredYawRate(
      vehicle.vehicleSpeedMps, control.steeringAngleDeg);
  control.yawErrorRadS =
      control.desiredYawRateRadS - vehicle.yawRateRadS;

  const float rawDerivative =
      (control.yawErrorRadS - previousYawErrorRadS) /
      max(dtSeconds, 0.001f);
  filteredYawErrorDerivative = lowPass(
      filteredYawErrorDerivative,
      rawDerivative,
      DERIVATIVE_FILTER_ALPHA);
  control.yawErrorDerivativeRadS2 = filteredYawErrorDerivative;
  previousYawErrorRadS = control.yawErrorRadS;

  const float activationThreshold =
      YAW_ERROR_BASE_ON_RAD_S +
      YAW_ERROR_RELATIVE_ON * fabsf(control.desiredYawRateRadS);
  const float releaseThreshold =
      activationThreshold * YAW_ERROR_OFF_RATIO;
  const bool controllerEligible =
      vehicle.vehicleSpeedMps >= MIN_ESP_SPEED_MPS &&
      control.requestedThrottlePwm > 0;

  if (!controllerEligible) {
    control.espActive = false;
  } else if (!control.espActive &&
             fabsf(control.yawErrorRadS) > activationThreshold) {
    control.espActive = true;
    espActivatedMs = nowMs;
  } else if (control.espActive &&
             nowMs - espActivatedMs >= ESP_MIN_ACTIVE_TIME_MS &&
             fabsf(control.yawErrorRadS) < releaseThreshold) {
    control.espActive = false;
  }

  control.stabilityState = classifyStability(
      control.desiredYawRateRadS,
      vehicle.yawRateRadS,
      activationThreshold);

  float targetLeftPwm = control.requestedThrottlePwm;
  float targetRightPwm = control.requestedThrottlePwm;
  control.correctionDemand = 0.0f;
  control.globalTorqueCut = 0.0f;
  control.leftCorrection = false;
  control.rightCorrection = false;

  if (control.espActive) {
    const float proportional = ESP_KP * fabsf(control.yawErrorRadS);
    const float derivative = ESP_KD * fabsf(control.yawErrorDerivativeRadS2);
    control.correctionDemand = clampFloat(
        proportional + derivative,
        0.0f,
        MAX_SELECTIVE_TORQUE_CUT);

    const float severeError = max(
        0.0f, fabsf(control.yawErrorRadS) - activationThreshold);
    control.globalTorqueCut = clampFloat(
        severeError / GLOBAL_CUT_FULL_ERROR_RAD_S,
        0.0f,
        1.0f) * MAX_GLOBAL_TORQUE_CUT;

    const float reducedBase =
        control.requestedThrottlePwm * (1.0f - control.globalTorqueCut);
    targetLeftPwm = reducedBase;
    targetRightPwm = reducedBase;
    const float selectiveReduction =
        control.requestedThrottlePwm * control.correctionDemand;

    if (control.yawErrorRadS > 0.0f) {
      // Need more positive/left yaw: reduce left-side drive torque.
      targetLeftPwm -= selectiveReduction;
      control.leftCorrection = true;
    } else {
      // Need more negative/right yaw: reduce right-side drive torque.
      targetRightPwm -= selectiveReduction;
      control.rightCorrection = true;
    }
  }

  targetLeftPwm = clampFloat(targetLeftPwm, 0.0f, 255.0f);
  targetRightPwm = clampFloat(targetRightPwm, 0.0f, 255.0f);

  const float maximumPwmStep = PWM_SLEW_PER_SECOND * dtSeconds;
  appliedLeftPwm = moveToward(
      appliedLeftPwm, targetLeftPwm, maximumPwmStep);
  appliedRightPwm = moveToward(
      appliedRightPwm, targetRightPwm, maximumPwmStep);

  control.leftMotorPwm = static_cast<uint8_t>(
      constrain(static_cast<int>(roundf(appliedLeftPwm)), 0, 255));
  control.rightMotorPwm = static_cast<uint8_t>(
      constrain(static_cast<int>(roundf(appliedRightPwm)), 0, 255));

  writeMotorOutputs(control.leftMotorPwm, control.rightMotorPwm);
  updateIndicatorLeds();
}


// -----------------------------------------------------------------------------
// SERIAL TELEMETRY AND COMMANDS
// -----------------------------------------------------------------------------

void printTelemetryHeader() {
  Serial.println(F(
      "ms,mode,armed,speed_mps,left_speed_mps,right_speed_mps,"
      "steering_deg,desired_yaw_deg_s,measured_yaw_deg_s,yaw_error_deg_s,"
      "lateral_accel_mps2,slip_ratio,esp_active,state,throttle_pwm,"
      "left_pwm,right_pwm,correction,global_cut,fault"));
  telemetryHeaderPrinted = true;
}

void printTelemetry() {
  if (!telemetryEnabled) return;
  if (!telemetryHeaderPrinted) printTelemetryHeader();

  Serial.print(millis());
  Serial.print(',');
  Serial.print(BENCH_SIMULATION_MODE ? F("SIM") : F("PHYSICAL"));
  Serial.print(',');
  Serial.print(isArmed() ? 1 : 0);
  Serial.print(',');
  Serial.print(vehicle.vehicleSpeedMps, 4);
  Serial.print(',');
  Serial.print(vehicle.leftSpeedMps, 4);
  Serial.print(',');
  Serial.print(vehicle.rightSpeedMps, 4);
  Serial.print(',');
  Serial.print(control.steeringAngleDeg, 3);
  Serial.print(',');
  Serial.print(control.desiredYawRateRadS * RAD_TO_DEG, 3);
  Serial.print(',');
  Serial.print(vehicle.yawRateRadS * RAD_TO_DEG, 3);
  Serial.print(',');
  Serial.print(control.yawErrorRadS * RAD_TO_DEG, 3);
  Serial.print(',');
  Serial.print(vehicle.lateralAccelMps2, 4);
  Serial.print(',');
  Serial.print(vehicle.wheelSlipRatio, 4);
  Serial.print(',');
  Serial.print(control.espActive ? 1 : 0);
  Serial.print(',');
  Serial.print(stabilityStateName(control.stabilityState));
  Serial.print(',');
  Serial.print(control.requestedThrottlePwm);
  Serial.print(',');
  Serial.print(control.leftMotorPwm);
  Serial.print(',');
  Serial.print(control.rightMotorPwm);
  Serial.print(',');
  Serial.print(control.correctionDemand, 4);
  Serial.print(',');
  Serial.print(control.globalTorqueCut, 4);
  Serial.print(',');
  Serial.println(faultName(faultCode));
}

void printConfiguration() {
  Serial.println(F("--- ESP-like model-car configuration ---"));
  Serial.print(F("Mode: "));
  Serial.println(BENCH_SIMULATION_MODE ? F("BENCH SIMULATION") : F("PHYSICAL SENSORS"));
  Serial.print(F("Wheelbase [m]: "));
  Serial.println(WHEELBASE_M, 4);
  Serial.print(F("Wheel radius [m]: "));
  Serial.println(WHEEL_RADIUS_M, 4);
  Serial.print(F("Pulses/revolution: "));
  Serial.println(WHEEL_PULSES_PER_REV);
  Serial.print(F("Minimum ESP speed [m/s]: "));
  Serial.println(MIN_ESP_SPEED_MPS, 3);
  Serial.print(F("ESP KP: "));
  Serial.println(ESP_KP, 4);
  Serial.print(F("ESP KD: "));
  Serial.println(ESP_KD, 4);
  Serial.println(F("Positive yaw/steering = left/CCW."));
}

void printSerialHelp() {
  Serial.println(F("Serial commands:"));
  Serial.println(F("  h = show this help"));
  Serial.println(F("  p = print configuration"));
  Serial.println(F("  t = toggle CSV telemetry"));
  Serial.println(F("  c = recalibrate MPU-6050 gyro while disarmed (physical mode)"));
}

void handleSerialCommands() {
  while (Serial.available() > 0) {
    const char command = static_cast<char>(Serial.read());
    if (command == 'h' || command == 'H') {
      printSerialHelp();
    } else if (command == 'p' || command == 'P') {
      printConfiguration();
    } else if (command == 't' || command == 'T') {
      telemetryEnabled = !telemetryEnabled;
      telemetryHeaderPrinted = false;
      Serial.print(F("Telemetry "));
      Serial.println(telemetryEnabled ? F("enabled.") : F("disabled."));
    } else if (command == 'c' || command == 'C') {
      if (BENCH_SIMULATION_MODE) {
        Serial.println(F("Gyro calibration is not used in bench simulation mode."));
      } else if (isArmed()) {
        Serial.println(F("Calibration refused: disarm the model first."));
      } else {
        stopMotorsImmediately();
        calibrateGyro();
      }
    }
  }
}


// -----------------------------------------------------------------------------
// ARDUINO SETUP AND LOOP
// -----------------------------------------------------------------------------

void setup() {
  pinMode(PIN_ARM_SWITCH, INPUT_PULLUP);
  pinMode(PIN_WHEEL_LEFT, INPUT_PULLUP);
  pinMode(PIN_WHEEL_RIGHT, INPUT_PULLUP);

  pinMode(PIN_MOTOR_LEFT_PWM, OUTPUT);
  pinMode(PIN_MOTOR_RIGHT_PWM, OUTPUT);
  pinMode(PIN_MOTOR_LEFT_DIR, OUTPUT);
  pinMode(PIN_MOTOR_RIGHT_DIR, OUTPUT);
  pinMode(PIN_LED_ESP_ACTIVE, OUTPUT);
  pinMode(PIN_LED_LEFT_CORRECTION, OUTPUT);
  pinMode(PIN_LED_RIGHT_CORRECTION, OUTPUT);
  pinMode(PIN_LED_FAULT, OUTPUT);

  stopMotorsImmediately();
  updateIndicatorLeds();

  if (USE_STEERING_SERVO) {
    steeringServo.attach(PIN_STEERING_SERVO);
    steeringServo.write(STEERING_SERVO_CENTER_DEG);
  }

  Serial.begin(115200);
  delay(300);
  Serial.println();
  Serial.println(F("Arduino ESP-like model-car controller v1.0.0"));
  Serial.println(F("EDUCATIONAL MODEL ONLY — NOT FOR A ROAD VEHICLE"));
  printConfiguration();
  printSerialHelp();

  if (isArmed()) {
    Serial.println(F("SAFE START: move ARM switch to OFF before enabling motors."));
    waitingForDisarmedStart = true;
  } else {
    waitingForDisarmedStart = false;
  }

  if (!BENCH_SIMULATION_MODE) {
    Wire.begin();
    Wire.setClock(400000UL);
#if defined(WIRE_HAS_TIMEOUT)
    Wire.setWireTimeout(3000, true);
#endif

    if (!initializeMpu()) {
      Serial.print(F("MPU initialization failed: "));
      Serial.println(faultName(faultCode));
    } else if (!isArmed()) {
      if (!completePhysicalSensorStartup()) {
        Serial.print(F("MPU calibration failed: "));
        Serial.println(faultName(faultCode));
      }
    } else {
      Serial.println(F(
          "Physical sensors initialized; gyro calibration will begin after disarming."));
    }
  } else {
    faultCode = FaultCode::NONE;
    Serial.println(F("Bench mode: A2 controls synthetic yaw disturbance."));
  }

  lastControlUs = micros();
  lastWheelUpdateMs = millis();
  lastTelemetryMs = millis();
  updateIndicatorLeds();
}

void loop() {
  handleSerialCommands();

  const uint32_t nowUs = micros();
  if (nowUs - lastControlUs >= CONTROL_INTERVAL_US) {
    float dtSeconds = (nowUs - lastControlUs) * 1.0e-6f;
    lastControlUs = nowUs;
    dtSeconds = clampFloat(dtSeconds, 0.005f, 0.100f);
    runControlStep(dtSeconds);
  }

  const uint32_t nowMs = millis();
  if (nowMs - lastTelemetryMs >= TELEMETRY_INTERVAL_MS) {
    lastTelemetryMs = nowMs;
    printTelemetry();
  }
}
