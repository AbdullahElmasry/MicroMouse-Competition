#pragma once

#include <math.h>

// Convert a recessed sensor's range to clearance from the chassis edge.
// Preserve invalid samples; a valid range inside the offset means zero clearance.
inline int sideClearanceMm(int rangeMm, int insetMm) {
  if (rangeMm < 0) return -1;
  return rangeMm > insetMm ? rangeMm - insetMm : 0;
}

struct ForwardWallSettings {
  int baseSpeed;
  int slowSpeed;
  int leftThresholdMm;
  int rightThresholdMm;
};

struct ForwardMotorCommands {
  int left;
  int right;
};

struct TwoWallPidSettings {
  float toleranceMm;
  int minimumSpeed;
  float kp;
  float ki;
  float kd;
  float maximumPwm;
};

class TwoWallPidController {
 public:
  void reset() {
    integral_ = 0.0f;
    previousError_ = 0.0f;
    havePrevious_ = false;
  }

  ForwardMotorCommands update(int leftMm, int rightMm, int baseSpeed,
                              const TwoWallPidSettings &settings, float dt,
                              float &errorMm, float &correctionPwm) {
    errorMm = 0.0f;
    correctionPwm = 0.0f;
    ForwardMotorCommands commands = {baseSpeed, baseSpeed};

    if (leftMm < 0 || rightMm < 0 || baseSpeed <= 0 ||
        !isfinite(dt) || dt <= 0.0f) {
      reset();
      return commands;
    }

    // Positive means the left wall is closer, so slow the physical right motor.
    errorMm = (float)rightMm - (float)leftMm;
    if (fabsf(errorMm) <= settings.toleranceMm) {
      errorMm = 0.0f;
      reset();
      return commands;
    }

    if (dt > 0.25f) reset();
    if (havePrevious_ && errorMm * previousError_ < 0.0f) integral_ = 0.0f;

    const float derivative = havePrevious_ ? (errorMm - previousError_) / dt : 0.0f;
    const float candidateIntegral = integral_ + errorMm * dt;
    const int minimum = baseSpeed < settings.minimumSpeed
                            ? baseSpeed : settings.minimumSpeed;
    const float available = (float)(baseSpeed - minimum);
    const float limit = available < settings.maximumPwm
                            ? available : settings.maximumPwm;
    const float low = errorMm > 0.0f ? 0.0f : -limit;
    const float high = errorMm > 0.0f ? limit : 0.0f;
    const float candidate = settings.kp * errorMm +
                            settings.ki * candidateIntegral +
                            settings.kd * derivative;

    if ((candidate >= low && candidate <= high) ||
        (candidate > high && errorMm < 0.0f) ||
        (candidate < low && errorMm > 0.0f)) {
      integral_ = candidateIntegral;
    }

    correctionPwm = settings.kp * errorMm + settings.ki * integral_ +
                    settings.kd * derivative;
    if (correctionPwm > high) correctionPwm = high;
    if (correctionPwm < low) correctionPwm = low;

    previousError_ = errorMm;
    havePrevious_ = true;

    if (correctionPwm > 0.0f) {
      commands.right = (int)lroundf(baseSpeed - correctionPwm);
    } else {
      commands.left = (int)lroundf(baseSpeed + correctionPwm);
    }
    return commands;
  }

 private:
  float integral_ = 0.0f;
  float previousError_ = 0.0f;
  bool havePrevious_ = false;
};

// Physical wheel convention: turn away from the close wall; left condition first.
// Negative distances indicate invalid sensor readings, not a nearby wall.
inline ForwardMotorCommands forwardWallCommands(
    int leftMm, int rightMm, const ForwardWallSettings &settings) {
  ForwardMotorCommands commands = {settings.baseSpeed, settings.baseSpeed};
  if (leftMm >= 0 && leftMm < settings.leftThresholdMm) {
    commands.right = settings.slowSpeed;
  } else if (rightMm >= 0 && rightMm < settings.rightThresholdMm) {
    commands.left = settings.slowSpeed;
  }
  return commands;
}

inline bool cellEncoderLimitReached(unsigned long left, unsigned long right,
                                    unsigned long leftTarget, unsigned long rightTarget) {
  // Brake both motors together, as main.ino does. Do not pivot to finish one wheel.
  return left >= leftTarget || right >= rightTarget;
}
