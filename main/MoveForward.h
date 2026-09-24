#pragma once

#include <math.h>
#include <stdint.h>
#include <string.h>



// Convert a recessed sensor's range to clearance from the chassis edge.
// Preserve invalid samples; a valid range inside the offset means zero clearance.
inline int sideClearanceMm(int rangeMm, int insetMm) {
  if (rangeMm < 0) return -1;
  return rangeMm > insetMm ? rangeMm - insetMm : 0;
}

// Pololu VL53L1X range status values used by the front sensor.
// SignalFail (2) and OutOfBoundsFail (4) mean that no target was measured.
inline bool frontRangeStatusMeansOpen(uint8_t status) {
  return status == 2 || status == 4;
}

inline bool frontRangeStatusIsUsable(uint8_t status) {
  return status == 0 || status == 6 || frontRangeStatusMeansOpen(status);
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

inline int noWallApproachSpeed(int approachSpeed, int minimumBase, int cruise) {
  if (approachSpeed <= 0) return 0;
  const int floor = minimumBase < cruise ? minimumBase : cruise;
  return approachSpeed < floor ? floor : approachSpeed;
}

inline float noWallYawCorrection(float yawRightDeg, float rateRightDps,
    float headingKp, float yawRateKd, float maximumPwm) {
  float correction = -headingKp * yawRightDeg - yawRateKd * rateRightDps;
  if (correction > maximumPwm) return maximumPwm;
  if (correction < -maximumPwm) return -maximumPwm;
  return correction;
}

inline ForwardMotorCommands noWallCombinedCommands(int baseSpeed,
    int minimumSpeed, float encoderPwm, float mpuPwm, float mpuWeight) {
  ForwardMotorCommands commands = {baseSpeed, baseSpeed};
  const int minimum = minimumSpeed < baseSpeed ? minimumSpeed : baseSpeed;
  const float limit = baseSpeed - minimum;
  // Positive steering slows RIGHT; encoderPwm positive requests slowing LEFT.
  if (mpuWeight < 0) mpuWeight = 0;
  if (mpuWeight > 1) mpuWeight = 1;
  float steering = mpuWeight * mpuPwm -
      (1.0f - mpuWeight) * encoderPwm;
  if (steering > limit) steering = limit;
  if (steering < -limit) steering = -limit;
  if (steering > 0) commands.right = (int)lroundf(baseSpeed - steering);
  else if (steering < 0) commands.left = (int)lroundf(baseSpeed + steering);
  return commands;
}

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
    // User verified each encoder belongs to its same-named physical motor.
    // Positive means LEFT travelled farther: slow LEFT to reduce the error.
    encoderError = (leftCells - rightCells) * averageTicks;

    const int minimum = baseSpeed < settings.minimumSpeed ? baseSpeed : settings.minimumSpeed;
    const float available = baseSpeed - minimum;
    const float pidLimit = settings.encoderPid.maximumPwm < available
        ? settings.encoderPid.maximumPwm : available;
    encoderPwm = pid(encoderError, dt, settings.encoderPid, pidLimit,
        encoderIntegral_, encoderPreviousError_, encoderPreviousValid_);
    ForwardMotorCommands commands = {baseSpeed, baseSpeed};
    if (encoderPwm > 0) commands.left = (int)lroundf(baseSpeed - encoderPwm);
    else if (encoderPwm < 0) commands.right = (int)lroundf(baseSpeed + encoderPwm);
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

enum class MovementCommand { None, Start, Stop };

// One parser per transport: WiFi and USB bytes may arrive in fragments.
class MovementCommands {
 public:
  void reset() { length_ = 0; invalid_ = false; lastByteMs_ = 0; }

  MovementCommand feed(char value, uint32_t nowMs) {
    lastByteMs_ = nowMs;
    if (value >= 'A' && value <= 'Z') value += 'a' - 'A';
    // Stop does not require Enter, even during an unfinished command.
    if (value == 'd') { reset(); return MovementCommand::Stop; }
    if (value == '\r' || value == '\n' || value == ' ' || value == '\t') return finish();
    if (length_ < 5 && !invalid_) buffer_[length_++] = value;
    else invalid_ = true;
    return MovementCommand::None;
  }

  MovementCommand poll(uint32_t nowMs) {
    // Also accept a terminal configured to send without a line ending.
    if ((length_ == 1 || length_ == 5 || invalid_) && uint32_t(nowMs - lastByteMs_) >= 100) return finish();
    return MovementCommand::None;
  }

 private:
  MovementCommand finish() {
    bool start = !invalid_ && ((length_ == 1 && buffer_[0] == 's') ||
                              (length_ == 5 && memcmp(buffer_, "start", 5) == 0));
    reset();
    return start ? MovementCommand::Start : MovementCommand::None;
  }
  char buffer_[5] = {};
  uint8_t length_ = 0;
  bool invalid_ = false;
  uint32_t lastByteMs_ = 0;
};

inline MovementCommand mergeCommands(MovementCommand first, MovementCommand next) {
  if (first == MovementCommand::Stop || next == MovementCommand::Stop) return MovementCommand::Stop;
  if (first == MovementCommand::Start || next == MovementCommand::Start) return MovementCommand::Start;
  return MovementCommand::None;
}

// Average the last five usable readings. Invalid/no-target readings discard
// old wall history so an opening is recognized immediately.
class TofFilter {
 public:
  void reset() { count_ = next_ = sum_ = 0; }
  bool full() const { return count_ == 5; }
  int update(int mm, bool usable = true) {
    if (!usable || mm < 0) { reset(); return mm; }
    if (count_ == 5) sum_ -= samples_[next_];
    else ++count_;
    samples_[next_] = mm;
    sum_ += mm;
    next_ = (next_ + 1) % 5;
    return (sum_ + count_ / 2) / count_;
  }
 private:
  int samples_[5] = {};
  int count_ = 0, next_ = 0, sum_ = 0;
};

void moveForwardSetup();
void moveForwardLoop();
void moveForwardStop();

