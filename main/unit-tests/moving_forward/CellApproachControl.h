#pragma once
#include <math.h>

struct CellApproachSettings {
  float kp, ki, kd;
  float slowdownTicks;
  int cruisePwm, minimumPwm;
};

class CellApproachController {
 public:
  explicit CellApproachController(const CellApproachSettings &settings) : settings_(settings) { reset(); }
  void reset() { integral_ = 0; previousError_ = 0; approaching_ = false; }

  int update(float remainingTicks, float dtSeconds) {
    if (!isfinite(remainingTicks) || remainingTicks <= 0) { reset(); return 0; }
    if (!isfinite(dtSeconds) || dtSeconds <= 0) { reset(); return settings_.minimumPwm; }
    if (remainingTicks > settings_.slowdownTicks) { reset(); return settings_.cruisePwm; }
    const float derivative = approaching_ ? (remainingTicks - previousError_) / dtSeconds : 0;
    const float candidateIntegral = integral_ + remainingTicks * dtSeconds;
    const float candidate = settings_.kp * remainingTicks + settings_.ki * candidateIntegral + settings_.kd * derivative;
    // Do not accumulate integral while the command is limited at either end.
    if (candidate >= settings_.minimumPwm && candidate <= settings_.cruisePwm) integral_ = candidateIntegral;
    float output = settings_.kp * remainingTicks + settings_.ki * integral_ + settings_.kd * derivative;
    previousError_ = remainingTicks;
    approaching_ = true;
    if (output < settings_.minimumPwm) output = settings_.minimumPwm;
    if (output > settings_.cruisePwm) output = settings_.cruisePwm;
    return (int)lroundf(output);
  }

 private:
  CellApproachSettings settings_;
  float integral_, previousError_;
  bool approaching_;
};

inline unsigned long ticksBeforeBrake(unsigned long ticks, unsigned long target, unsigned long brakeLead) {
  const unsigned long brakeAt = brakeLead < target ? target - brakeLead : 0;
  return ticks < brakeAt ? brakeAt - ticks : 0;
}
