#include "rotation.h"

#include <Arduino.h>
#include <Wire.h>
#include <math.h>

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
constexpr float turnKd = 3.3f;
// Keep enough torque to overcome drivetrain stiction near the target.
constexpr int minOutput = 95;
constexpr int maxTurnSpeed = 150;
constexpr float angleToleranceDeg = 1.5f;
constexpr int stableTicksNeeded = 10;
constexpr unsigned long turnTimeoutMs = 3000;

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
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0 ||
      Wire.requestFrom((uint8_t)MPU_ADDR, count) != count) {
    return false;
  }
  for (uint8_t i = 0; i < count; ++i) data[i] = Wire.read();
  return true;
}

bool writeMPU6050(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.write(value);
  return Wire.endTransmission() == 0;
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
  if (!readMPU6050(0x75, &identity, 1) || identity != 0x68) return false;
  if (!writeMPU6050(0x6B, 0x01)) return false;
  delay(100);
  if (!writeMPU6050(0x1A, 0x06) ||
      !writeMPU6050(0x1B, 0x08)) return false;
  delay(100);

  logPrintln("Calibrating gyro bias - keep the robot completely still...");
  float sum = 0.0f;
  for (int i = 0; i < 200; ++i) {
    float rate;
    if (!readGyroZ(rate)) return false;
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

    unsigned long now = micros();
    float dt = (now - lastMicros) / 1000000.0f;
    if (dt < 0.005f) continue;
    lastMicros = now;

    float gyroZ;
    if (!readGyroZ(gyroZ)) {
      mpuReady = false;
      logPrintln("MPU read failed - stopping.");
      break;
    }
    float appliedGyroZ = -(gyroZ - gyroZBias);
    if (fabs(appliedGyroZ) < GYRO_DEADBAND_DPS) {
      appliedGyroZ = 0.0f;
    }
    headingDeg += appliedGyroZ * dt;

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
  return completed;
}
