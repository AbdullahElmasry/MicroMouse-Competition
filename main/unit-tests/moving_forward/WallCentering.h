#pragma once
#include <math.h>

// Positive error means too close to the left wall: speed up left, slow right.
struct WallGeometry {
  float corridorWidthMm;
  float robotWidthMm;
  float leftSensorInsetMm;  // Positive when recessed inward from chassis side.
  float rightSensorInsetMm;
  float widthToleranceMm;
  float minimumClearanceMm;
  float centeringToleranceMm;
};

inline bool wallCenterError(float leftMm, float rightMm,
                            const WallGeometry &geometry, float &errorMm) {
  if (!isfinite(leftMm) || !isfinite(rightMm) || leftMm < 0 || rightMm < 0) return false;
  const float leftGap = leftMm - geometry.leftSensorInsetMm;
  const float rightGap = rightMm - geometry.rightSensorInsetMm;
  const float expectedGapSum = geometry.corridorWidthMm - geometry.robotWidthMm;
  if (expectedGapSum <= 0 || leftGap <= geometry.minimumClearanceMm ||
      rightGap <= geometry.minimumClearanceMm) return false;
  // This bench test requires two continuous walls; don't steer toward an opening.
  if (fabsf(leftGap + rightGap - expectedGapSum) > geometry.widthToleranceMm) return false;
  errorMm = (rightGap - leftGap) * 0.5f;
  if (fabsf(errorMm) <= geometry.centeringToleranceMm) errorMm = 0;
  return true;
}

class WallPid {
 public:
  WallPid(float kp, float ki, float kd, float limit)
      : kp_(kp), ki_(ki), kd_(kd), limit_(limit) { reset(); }

  void reset() { integral_ = 0; previousError_ = 0; havePrevious_ = false; }

  float update(float error, float dt) {
    if (!isfinite(error) || !isfinite(dt) || dt <= 0) { reset(); return 0; }
    const float derivative = havePrevious_ ? (error - previousError_) / dt : 0;
    const float proposedIntegral = integral_ + error * dt;
    const float proposed = kp_ * error + ki_ * proposedIntegral + kd_ * derivative;
    // Conditional integration prevents windup, but allows unwinding saturation.
    if ((proposed >= -limit_ && proposed <= limit_) ||
        (proposed > limit_ && error < 0) || (proposed < -limit_ && error > 0)) {
      integral_ = proposedIntegral;
    }
    previousError_ = error;
    havePrevious_ = true;
    const float output = kp_ * error + ki_ * integral_ + kd_ * derivative;
    return output > limit_ ? limit_ : (output < -limit_ ? -limit_ : output);
  }

 private:
  float kp_, ki_, kd_, limit_, integral_, previousError_;
  bool havePrevious_;
};
