#pragma once
#include <math.h>
#include "ForwardWallControl.h"

enum class WallMode { Two, LeftOnly, RightOnly, None };
class WallModeDetector {
 public:
  // Distances are chassis clearances (right inset already subtracted).
  static constexpr int ACQUIRE_MM = 80;
  static constexpr int RETAIN_MM = 100;
  static constexpr int CONFIRM_SAMPLES = 3;
  void reset() { left_=right_=false; leftCount_=rightCount_=0; }
  WallMode update(int left,int right) {
    left_=present(left,left_,leftCount_); right_=present(right,right_,rightCount_);
    if (left_ && right_) return WallMode::Two;
    if (left_) return WallMode::LeftOnly;
    if (right_) return WallMode::RightOnly;
    return WallMode::None;
  }
 private:
  bool present(int mm,bool previous,int &count) {
    if (mm<0 || mm>=RETAIN_MM) { count=0; return false; }
    if (previous) return true;
    if (mm>=ACQUIRE_MM) { count=0; return false; }
    if (count<CONFIRM_SAMPLES) ++count;
    return count>=CONFIRM_SAMPLES;
  }
  bool left_=false,right_=false;
  int leftCount_=0,rightCount_=0;
};

struct SingleWallSettings {
  float leftTargetMm, rightTargetMm, toleranceMm;
  int slowSpeed;
  float headingKp, yawRateKd, maximumMpuPwm;
  float wallKp, wallKi, wallKd, maximumWallPwm;
};
inline float clampSingleWall(float x,float limit) { return x>limit?limit:(x< -limit?-limit:x); }

class SingleWallController {
 public:
  void reset() { clearPid(); previousMode_=WallMode::None; }
  ForwardMotorCommands update(WallMode mode,int left,int right,
    float yawRightDeg,float rateRightDps,int base,const SingleWallSettings &s,
    float dt,float &wallError,float &wallPwm,float &mpuPwm) {
  wallError=wallPwm=mpuPwm=0;
  if ((mode!=WallMode::LeftOnly && mode!=WallMode::RightOnly) ||
      (mode==WallMode::LeftOnly?left:right)<0 ||
      !isfinite(yawRightDeg) || !isfinite(rateRightDps) || !isfinite(dt) || dt<=0) {
    reset(); return {0,0};
  }
  if (mode!=previousMode_ || dt>0.25f) clearPid();
  previousMode_=mode;
  // Positive error requests a rightward correction in the working motor convention.
  wallError=mode==WallMode::LeftOnly?s.leftTargetMm-left:right-s.rightTargetMm;
  if (fabsf(wallError)<=s.toleranceMm) wallError=0;
  ForwardMotorCommands commands={base,base};
  const int slow=base<s.slowSpeed?base:s.slowSpeed;
  const float available=base-slow;
  if (wallError!=0) {
    // Preserve the working direction: positive slows physical RIGHT, negative slows physical LEFT.
    const float limit=available<s.maximumWallPwm?available:s.maximumWallPwm;
    const float low=wallError>0?0:-limit, high=wallError>0?limit:0;
    if (havePrevious_ && wallError*previousError_<0) integral_=0;
    const float derivative=havePrevious_?(wallError-previousError_)/dt:0;
    const float candidateIntegral=integral_+wallError*dt;
    const float candidate=s.wallKp*wallError+s.wallKi*candidateIntegral+s.wallKd*derivative;
    // Anti-windup uses the actual speed-dependent motor limits.
    if ((candidate>=low && candidate<=high) ||
        (candidate>high && wallError<0) || (candidate<low && wallError>0)) integral_=candidateIntegral;
    float output=s.wallKp*wallError+s.wallKi*integral_+s.wallKd*derivative;
    wallPwm=output<low?low:(output>high?high:output);
    previousError_=wallError;
    havePrevious_=true;
    if (wallPwm>0) commands.right=(int)lroundf(base-wallPwm);
    else commands.left=(int)lroundf(base+wallPwm);
    return commands; // MPU cannot override a wall-distance correction.
  }
  clearPid(); // No stored integral/derivative inside the distance tolerance band.
  float headingError=-yawRightDeg;
  while (headingError>180) headingError-=360;
  while (headingError< -180) headingError+=360;
  float correction=clampSingleWall(s.headingKp*headingError-s.yawRateKd*rateRightDps,s.maximumMpuPwm);
  correction=clampSingleWall(correction,available);
  mpuPwm=correction;
  if (correction>0) commands.right=(int)lroundf(base-correction);
  else commands.left=(int)lroundf(base+correction);
  return commands;
  }
 private:
  void clearPid() { integral_=previousError_=0; havePrevious_=false; }
  float integral_=0,previousError_=0;
  bool havePrevious_=false;
  WallMode previousMode_=WallMode::None;
};
