#include "rotation.h"

#include <Arduino.h>
#include <Wire.h>
#include <math.h>
#include "config.h"

// USB/Wi-Fi logging and stop command handling live in turns.ino.
void logPrint(const String &text);
void logPrintln(const String &text);
bool abortRequested();

namespace {
constexpr int LEFT_IN1 = 25, LEFT_IN2 = 26;
constexpr int RIGHT_IN1 = 14, RIGHT_IN2 = 27, MOTOR_EN = 23;
constexpr int MPU_ADDR = 0x68;
constexpr float GYRO_DEADBAND_DPS = 0.5f;

constexpr float turnKp = 15.9f;
constexpr float turnKi = 0.0f;
constexpr float turnKd = 1.3f;
// Keep enough torque to overcome drivetrain stiction near the target.
constexpr int minOutput = 95;
constexpr int maxTurnSpeed = 150;
constexpr float angleToleranceDeg = 1.0f;
constexpr int stableTicksNeeded = 10;
constexpr unsigned long turnTimeoutMs = 2000;

float gyroZBias = 0.0f;
float headingDeg = 0;
bool mpuReady = false;

void driveMotor(int in1, int in2, int speed) {
  speed = constrain(speed, -255, 255);
  if (speed > 0) {
    analogWrite(in1, speed);
    analogWrite(in2, 0);
  } else if (speed < 0) {
    analogWrite(in1, 0);
    analogWrite(in2, -speed);
  } else {
    analogWrite(in1, 0);
    analogWrite(in2, 0);
  }
}

void drive(int left, int right) {
  driveMotor(LEFT_IN1, LEFT_IN2, left);
  driveMotor(RIGHT_IN1, RIGHT_IN2, right);
}

bool readMPU6050(uint8_t reg, uint8_t *data, uint8_t count) {
  const unsigned long started = millis();
  const char *firstFailure = "none";
  uint8_t firstDetail = 0;
  for (int attempt = 1; attempt <= MPU_I2C_MAX_ATTEMPTS; ++attempt) {
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(reg);
    const uint8_t status = Wire.endTransmission(false);
    if (status == 0) {
      const uint8_t received = Wire.requestFrom((uint8_t)MPU_ADDR, count);
      if (received == count) {
        for (uint8_t i = 0; i < count; ++i) data[i] = Wire.read();
        if (attempt > 1) {
          char line[150];
          snprintf(line, sizeof(line),
                   "MPU RECOVERED | rotation reg=0x%02X | attempt=%d/%d | elapsed=%lums | first=%s %u",
                   reg, attempt, MPU_I2C_MAX_ATTEMPTS, millis() - started,
                   firstFailure, firstDetail);
          logPrintln(line);
        }
        return true;
      }
      if (attempt == 1) {
        firstFailure = "short read bytes";
        firstDetail = received;
      }
      if (attempt == MPU_I2C_MAX_ATTEMPTS ||
          millis() - started >= MPU_I2C_RETRY_BUDGET_MS) {
        char line[150];
        snprintf(line, sizeof(line),
                 "MPU FAULT | rotation MPU6050 0x68 | read reg=0x%02X | received=%u/%u bytes | attempts=%d | elapsed=%lums",
                 reg, received, count, attempt, millis() - started);
        logPrintln(line);
        return false;
      }
    } else {
      if (attempt == 1) {
        firstFailure = "select I2C code";
        firstDetail = status;
      }
      if (attempt == MPU_I2C_MAX_ATTEMPTS ||
          millis() - started >= MPU_I2C_RETRY_BUDGET_MS) {
        char line[140];
        snprintf(line, sizeof(line),
                 "MPU FAULT | rotation MPU6050 0x68 | select reg=0x%02X | I2C=%u | attempts=%d | elapsed=%lums",
                 reg, status, attempt, millis() - started);
        logPrintln(line);
        return false;
      }
    }
    // Keep the turn moving through a brief recovered read, but stop if
    // the sensor remains unavailable long enough to lose heading control.
    if (millis() - started >= MPU_RETRY_MOTOR_HOLD_MS) stopRotation();
    delay(5);
  }
  return false;
}

bool writeMPU6050(uint8_t reg, uint8_t value) {
  const unsigned long started = millis();
  for (int attempt = 1; attempt <= MPU_I2C_MAX_ATTEMPTS; ++attempt) {
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(reg);
    Wire.write(value);
    const uint8_t status = Wire.endTransmission();
    if (status == 0) return true;
    if (attempt == MPU_I2C_MAX_ATTEMPTS ||
        millis() - started >= MPU_I2C_RETRY_BUDGET_MS) {
      char line[140];
      snprintf(line, sizeof(line),
               "MPU FAULT | rotation MPU6050 0x68 | write reg=0x%02X | I2C=%u | attempts=%d | elapsed=%lums",
               reg, status, attempt, millis() - started);
      logPrintln(line);
      return false;
    }
    delay(5);
  }
  return false;
}

bool readGyroZ(float &rate) {
  uint8_t data[2];
  if (!readMPU6050(0x47, data, 2)) return false;
  int16_t raw = static_cast<int16_t>(
      (static_cast<uint16_t>(data[0]) << 8) | data[1]);
  rate = raw / 65.5f;
  return true;
}

bool initMPU() {
  uint8_t identity;
  if (!readMPU6050(0x75, &identity, 1)) return false;
  if (identity != 0x68) {
    char line[100];
    snprintf(line, sizeof(line),
             "MPU FAULT | sensor=rotation MPU6050 0x68 | WHO_AM_I=0x%02X expected=0x68",
             identity);
    logPrintln(line);
    return false;
  }
  if (!writeMPU6050(0x6B, 0x01)) return false;
  delay(100);
  if (!writeMPU6050(0x1A, 0x06) ||
      !writeMPU6050(0x1B, 0x08)) return false;
  delay(100);

  logPrintln("Calibrating gyro bias - keep the robot completely still...");
  float sum = 0.0f;
  for (int i = 0; i < 200; ++i) {
    float rate;
    if (!readGyroZ(rate)) {
      logPrintln("MPU FAULT | rotation gyro bias calibration interrupted");
      return false;
    }
    sum += rate;
    delay(10);
  }
  gyroZBias = sum / 200.0f;
  return true;
}
} // namespace

void stopRotation() {
  drive(0, 0);
}

bool beginRotation() {
  Wire.begin(21, 22);
  pinMode(MOTOR_EN, OUTPUT);
  digitalWrite(MOTOR_EN, HIGH);
  pinMode(LEFT_IN1, OUTPUT);
  pinMode(LEFT_IN2, OUTPUT);
  pinMode(RIGHT_IN1, OUTPUT);
  pinMode(RIGHT_IN2, OUTPUT);
  stopRotation();

  headingDeg = 0;
  mpuReady = initMPU();
  logPrintln(mpuReady ? "MPU initialization OK."
                     : "MPU initialization FAILED.");
  return mpuReady;
}

bool turnDegrees(float relativeAngle) {
  if (!mpuReady) {
    stopRotation();
    logPrintln("REFUSED: MPU initialization failed.");
    return false;
  }

  const float startHeading = headingDeg;
  char turnLog[160];
  snprintf(turnLog, sizeof(turnLog), "TURN | %s %.1fdeg",
           relativeAngle < 0 ? "RIGHT" : "LEFT", fabsf(relativeAngle));
  logPrintln(turnLog);

  const float targetHeading = headingDeg + relativeAngle;
  float integral = 0, prevError = 0;
  unsigned long lastMicros = micros();
  unsigned long startMs = millis();
  int stableTicks = 0;
  bool completed = false;

  while (true) {
    if (abortRequested()) {
      logPrintln("Turn aborted - stopping.");
      break;
    }
    if (millis() - startMs > turnTimeoutMs) {
      logPrintln("Turn timeout - stopping.");
      break;
    }

    if (micros() - lastMicros < 5000) continue;

    float gyroZ;
    if (!readGyroZ(gyroZ)) {
      mpuReady = false;
      logPrintln("MPU read failed after retries - stopping.");
      break;
    }
    const unsigned long sampleMicros = micros();
    float dt = (sampleMicros - lastMicros) / 1000000.0f;
    lastMicros = sampleMicros;
    // Do not integrate a long gap as one gyro sample.
    if (dt > 0.1f) dt = 0.0f;
    float appliedGyroZ = -(gyroZ - gyroZBias);
    if (fabs(appliedGyroZ) < GYRO_DEADBAND_DPS) {
      appliedGyroZ = 0.0f;
    }
    headingDeg += appliedGyroZ * dt;
    const float turned = headingDeg - startHeading;
    if ((relativeAngle > 0 && turned < -5.0f) ||
        (relativeAngle < 0 && turned > 5.0f)) {
      stopRotation();
      logPrintln("TURN FAULT | gyro angle moving opposite the requested direction");
      break;
    }

    float error = targetHeading - headingDeg;
    if (fabs(error) <= angleToleranceDeg) {
      stopRotation();
      ++stableTicks;
      if (stableTicks >= stableTicksNeeded) {
        completed = true;
        break;
      }
      continue;
    } else {
      stableTicks = 0;
    }

    float derivative = (dt > 0) ? (error - prevError) / dt : 0;
    float tentativeIntegral = integral + error * dt;
    float tentativeOutput = turnKp * error + turnKi * tentativeIntegral +
                            turnKd * derivative;
    if (tentativeOutput <= maxTurnSpeed && tentativeOutput >= -maxTurnSpeed) {
      integral = tentativeIntegral;
    }
    prevError = error;

    float output = turnKp * error + turnKi * integral + turnKd * derivative;
    output = constrain(output, -(float)maxTurnSpeed, (float)maxTurnSpeed);
    if (output > 0 && output < minOutput) output = minOutput;
    if (output < 0 && output > -minOutput) output = -minOutput;
    drive((int)output, -(int)output);
  }

  stopRotation();
  // Display right-positive angles, consistent with forward yaw telemetry.
  snprintf(turnLog, sizeof(turnLog),
           "TURN END | %s | target=%+.2fdeg actual=%+.2fdeg error=%+.2fdeg | time=%lums",
           completed ? "OK" : "FAILED", -relativeAngle,
           -(headingDeg - startHeading), -(targetHeading - headingDeg),
           millis() - startMs);
  logPrintln(turnLog);
  if (completed) {
    headingDeg = 0.0f;
    logPrintln("MPU RESET | rotation relative heading=0deg");
  }
  return completed;
}
