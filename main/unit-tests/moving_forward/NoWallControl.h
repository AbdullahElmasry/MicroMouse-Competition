#pragma once

#include <math.h>
#include "ForwardWallControl.h"

struct NoWallPidSettings {
  float kp;
  float ki;
  float kd;
  float maximumPwm;
};

struct NoWallSettings {
  float leftTicksPerCell;
  float rightTicksPerCell;
  int minimumSpeed;
  NoWallPidSettings encoderPid;
};

class NoWallController {
 public:
  void reset() {
    active_ = false;
    encoderPreviousValid_ = false;
    encoderIntegral_ = 0;
    encoderPreviousError_ = 0;
  }

  ForwardMotorCommands update(unsigned long leftTicks, unsigned long rightTicks,
      int baseSpeed, const NoWallSettings &settings, float dt,
      float &encoderError, float &encoderPwm) {
    encoderError = encoderPwm = 0;
    if (!valid(settings, dt) || baseSpeed <= 0) {
      reset();
      return {0, 0};
    }

    if (!active_) {
      active_ = true;
      startLeft_ = leftTicks;
      startRight_ = rightTicks;
      encoderPreviousValid_ = false;
      encoderIntegral_ = 0;
    }
    if (dt > 0.25f) {
      // Do not turn a delayed sensor/network cycle into a derivative spike.
      encoderPreviousValid_ = false;
    }

    const float leftCells = (leftTicks - startLeft_) / settings.leftTicksPerCell;
    const float rightCells = (rightTicks - startRight_) / settings.rightTicksPerCell;
    const float averageTicks = (settings.leftTicksPerCell + settings.rightTicksPerCell) * 0.5f;
    // Positive means the left encoder has travelled farther. Hardware logs show
    // that reducing the named RIGHT command corrects this physical drift.
    encoderError = (leftCells - rightCells) * averageTicks;

    const int minimum = baseSpeed < settings.minimumSpeed ? baseSpeed : settings.minimumSpeed;
    const float available = baseSpeed - minimum;
    const float pidLimit = settings.encoderPid.maximumPwm < available
        ? settings.encoderPid.maximumPwm : available;
    encoderPwm = pid(encoderError, dt, settings.encoderPid, pidLimit,
        encoderIntegral_, encoderPreviousError_, encoderPreviousValid_);
    ForwardMotorCommands commands = {baseSpeed, baseSpeed};
    if (encoderPwm > 0) commands.right = (int)lroundf(baseSpeed - encoderPwm);
    else if (encoderPwm < 0) commands.left = (int)lroundf(baseSpeed + encoderPwm);
    return commands;
  }

 private:
  static float clamp(float value, float limit) {
    return value > limit ? limit : (value < -limit ? -limit : value);
  }

  static bool valid(const NoWallSettings &s, float dt) {
    return isfinite(dt) && dt > 0 &&
           s.leftTicksPerCell > 0 && s.rightTicksPerCell > 0;
  }

  static float pid(float error, float dt, const NoWallPidSettings &s, float limit,
      float &integral, float &previousError, bool &previousValid) {
    const float derivative = previousValid ? (error - previousError) / dt : 0;
    const float candidateIntegral = integral + error * dt;
    const float candidate = s.kp * error + s.ki * candidateIntegral + s.kd * derivative;
    // Integrate inside the output range, or when the error would unwind saturation.
    if ((candidate >= -limit && candidate <= limit) ||
        (candidate > limit && error < 0) ||
        (candidate < -limit && error > 0)) {
      integral = candidateIntegral;
    }
    const float output = s.kp * error + s.ki * integral + s.kd * derivative;
    previousError = error;
    previousValid = true;
    return clamp(output, limit);
  }

  bool active_ = false;
  unsigned long startLeft_ = 0, startRight_ = 0;
  float encoderIntegral_ = 0;
  float encoderPreviousError_ = 0;
  bool encoderPreviousValid_ = false;
};
