#include "rotation.h"

#include <Arduino.h>
#include <Wire.h>
#include <math.h>
#include "DemoMotion.h"

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
constexpr float angleToleranceDeg = 1.5f;
constexpr int stableTicksNeeded = 10;
constexpr unsigned long turnTimeoutMs = 3000;

float gyroZBias = 0.0f;
float headingDeg = 0;
bool mpuReady = false;
char initializationFault[140] = "";

void logInitializationFault(const char *message) {
  snprintf(initializationFault, sizeof(initializationFault), "%s", message);
  logPrintln(message);
}

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

bool readMPU6050(uint8_t reg, uint8_t *data, uint8_t count,
                 bool duringInit = false) {
  const int attempts = duringInit ? 10 : 1;
  for (int attempt = 0; attempt < attempts; ++attempt) {
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(reg);
    if (Wire.endTransmission(false) == 0 &&
        Wire.requestFrom((uint8_t)MPU_ADDR, count) == count) {
      for (uint8_t i = 0; i < count; ++i) data[i] = Wire.read();
      return true;
    }
    if (duringInit) delay(5);
  }
  return false;
}

bool writeMPU6050(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.write(value);
  return Wire.endTransmission() == 0;
}

bool readGyroZ(float &rate, bool duringInit = false) {
  uint8_t data[2];
  if (!readMPU6050(0x47, data, 2, duringInit)) return false;
  int16_t raw = static_cast<int16_t>(
      (static_cast<uint16_t>(data[0]) << 8) | data[1]);
  rate = raw / 65.5f;
  return true;
}

bool initMPU() {
  uint8_t identity;
  if (!readMPU6050(0x75, &identity, 1)) {
    logInitializationFault("MPU INIT ERROR | rotation WHO_AM_I read failed");
    return false;
  }
  if (identity != 0x68) {
    char fault[80];
    snprintf(fault, sizeof(fault),
             "MPU INIT ERROR | rotation WHO_AM_I=0x%02X expected=0x68", identity);
    logInitializationFault(fault);
    return false;
  }
  uint8_t power1 = 0, gyroConfig = 0;
  bool powerReadOk = false, gyroReadOk = false;
  int configuredAttempt = 0;
  int attemptsMade = 0;
  const unsigned long configureStart = millis();
  for (int attempt = 1; attempt <= 10 && millis() - configureStart < 1800; ++attempt) {
    attemptsMade = attempt;
    if (!writeMPU6050(0x6B, 0x01)) {
      delay(10);
      continue;
    }
    delay(100);
    powerReadOk = readMPU6050(0x6B, &power1, 1);
    if (!powerReadOk || power1 != 0x01) continue;
    if (!writeMPU6050(0x1A, 0x06) ||
        !writeMPU6050(0x1B, 0x08)) continue;
    delay(50);
    gyroReadOk = readMPU6050(0x1B, &gyroConfig, 1);
    if (gyroReadOk && gyroConfig == 0x08) {
      configuredAttempt = attempt;
      break;
    }
  }
  if (!configuredAttempt) {
    char fault[140];
    snprintf(fault, sizeof(fault),
             "MPU INIT ERROR | rotation config failed %d tries/%lums pwr1=0x%02X ok=%d gyro_cfg=0x%02X ok=%d",
             attemptsMade, millis() - configureStart,
             power1, powerReadOk, gyroConfig, gyroReadOk);
    logInitializationFault(fault);
    return false;
  }
  if (configuredAttempt > 1) {
    char message[85];
    snprintf(message, sizeof(message),
             "MPU INIT | rotation configuration recovered on attempt %d/10",
             configuredAttempt);
    logPrintln(message);
  }
  logPrintln("Calibrating gyro bias - keep the robot completely still...");
  float sum = 0.0f;
  for (int i = 0; i < 200; ++i) {
    float rate;
    if (!readGyroZ(rate)) {
      char fault[90];
      snprintf(fault, sizeof(fault),
               "MPU INIT ERROR | rotation gyro calibration sample=%d/200", i + 1);
      logInitializationFault(fault);
      return false;
    }
    sum += rate;
    delay(10);
  }
  gyroZBias = sum / 200.0f;
  powerReadOk = readMPU6050(0x6B, &power1, 1);
  gyroReadOk = readMPU6050(0x1B, &gyroConfig, 1);
  if (!powerReadOk || !gyroReadOk ||
      power1 != 0x01 || gyroConfig != 0x08) {
    char fault[130];
    snprintf(fault, sizeof(fault),
             "MPU INIT ERROR | rotation after calibration pwr1=0x%02X ok=%d gyro_cfg=0x%02X ok=%d",
             power1, powerReadOk, gyroConfig, gyroReadOk);
    logInitializationFault(fault);
    return false;
  }
  return true;
}
} // namespace

void stopRotation() {
  drive(0, 0);
}

const char *rotationInitializationFault() {
  return initializationFault[0] ? initializationFault
                                : "MPU INIT ERROR | rotation failed without detail";
}

bool beginRotation() {
  Wire.begin(21, 22);
  headingDeg = 0;
  initializationFault[0] = '\0';
  mpuReady = initMPU();

  pinMode(MOTOR_EN, OUTPUT);
  digitalWrite(MOTOR_EN, LOW);
  pinMode(LEFT_IN1, OUTPUT);
  pinMode(LEFT_IN2, OUTPUT);
  pinMode(RIGHT_IN1, OUTPUT);
  pinMode(RIGHT_IN2, OUTPUT);
  stopRotation();

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
  uint8_t power1 = 0, gyroConfig = 0;
  const bool power1Ok = readMPU6050(0x6B, &power1, 1, true);
  const bool gyroConfigOk = readMPU6050(0x1B, &gyroConfig, 1, true);
  if (!power1Ok || !gyroConfigOk || power1 != 0x01 || gyroConfig != 0x08) {
    stopRotation();
    mpuReady = false;
    char fault[135];
    snprintf(fault, sizeof(fault),
             "TURN FAULT | MPU configuration lost | pwr1=0x%02X ok=%d | gyro_cfg=0x%02X ok=%d | reset ESP32",
             power1, power1Ok, gyroConfig, gyroConfigOk);
    logPrintln(fault);
    return false;
  }

  const float startHeading = headingDeg;
  char turnLog[220];
  snprintf(turnLog, sizeof(turnLog), "TURN | %s %.1fdeg",
           relativeAngle < 0 ? "RIGHT" : "LEFT", fabsf(relativeAngle));
  logPrintln(turnLog);

  const float targetHeading = headingDeg + relativeAngle;
  float integral = 0, prevError = 0;
  unsigned long lastMicros = micros();
  unsigned long startMs = millis();
  unsigned long lastProgressMs = startMs;
  unsigned long startLeftTicks, startRightTicks;
  demoReadEncoderTicks(startLeftTicks, startRightTicks);
  int stableTicks = 0;
  bool completed = false;
  bool mpuSnapshotLogged = false;

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

    if (millis() - lastProgressMs >= 250) {
      unsigned long leftTicks, rightTicks;
      demoReadEncoderTicks(leftTicks, rightTicks);
      char progress[240];
      snprintf(progress, sizeof(progress),
               "TURN PROGRESS | ms=%lu | target=%+.1f actual=%+.1f error=%+.1f deg | gyroZ=%+.2f bias=%+.2f rateRight=%+.2f dps | PWM 25/26=%d 14/27=%d | encoder ticks L/R=%lu/%lu",
               millis() - startMs, -relativeAngle, -(headingDeg - startHeading),
               -error, gyroZ, gyroZBias, -appliedGyroZ,
               (int)output, -(int)output,
               leftTicks - startLeftTicks, rightTicks - startRightTicks);
      logPrintln(progress);
      lastProgressMs = millis();

      if (!mpuSnapshotLogged &&
          leftTicks - startLeftTicks >= 50 &&
          rightTicks - startRightTicks >= 50 &&
          fabsf(headingDeg - startHeading) < 0.5f) {
        uint8_t identity = 0, power1 = 0, power2 = 0, gyroConfig = 0;
        uint8_t rawGyro[2] = {0, 0};
        const bool identityOk = readMPU6050(0x75, &identity, 1);
        const bool power1Ok = readMPU6050(0x6B, &power1, 1);
        const bool power2Ok = readMPU6050(0x6C, &power2, 1);
        const bool gyroConfigOk = readMPU6050(0x1B, &gyroConfig, 1);
        const bool rawGyroOk = readMPU6050(0x47, rawGyro, 2);
        char snapshot[240];
        snprintf(snapshot, sizeof(snapshot),
                 "TURN MPU SNAPSHOT | ms=%lu | id=0x%02X ok=%d | pwr1=0x%02X ok=%d | pwr2=0x%02X ok=%d | gyro_cfg=0x%02X ok=%d | gyro_z=0x%02X%02X ok=%d",
                 millis() - startMs, identity, identityOk,
                 power1, power1Ok, power2, power2Ok,
                 gyroConfig, gyroConfigOk,
                 rawGyro[0], rawGyro[1], rawGyroOk);
        logPrintln(snapshot);
        mpuSnapshotLogged = true;
        if ((power1Ok && power1 != 0x01) ||
            (gyroConfigOk && gyroConfig != 0x08)) {
          stopRotation();
          mpuReady = false;
          logPrintln("TURN FAULT | MPU configuration lost; motors stopped");
          break;
        }
      }
    }
  }

  stopRotation();
  unsigned long endLeftTicks, endRightTicks;
  demoReadEncoderTicks(endLeftTicks, endRightTicks);
  // Display right-positive angles, consistent with forward yaw telemetry.
  snprintf(turnLog, sizeof(turnLog),
           "TURN END | %s | target=%+.2fdeg actual=%+.2fdeg error=%+.2fdeg | time=%lums | encoder ticks L/R=%lu/%lu",
           completed ? "OK" : "FAILED", -relativeAngle,
           -(headingDeg - startHeading), -(targetHeading - headingDeg),
           millis() - startMs,
           endLeftTicks - startLeftTicks, endRightTicks - startRightTicks);
  logPrintln(turnLog);
  return completed;
}
