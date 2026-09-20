#pragma once
#include <math.h>
#include <stdint.h>

// Same Kalman equation, tuning, and pre-filter deadband as the tested MPU sketch.
class YawEstimate {
 public:
  void reset(uint32_t now) { estimate_=0; covariance_=2; yaw_=0; lastMs_=now; healthy_=true; }
  bool update(float zDps, uint32_t now) {
    uint32_t elapsed=now-lastMs_;
    if (!healthy_ || !isfinite(zDps) || elapsed>100) { healthy_=false; return false; }
    if (!elapsed) return true;
    if (fabsf(zDps)<0.5f) zDps=0;
    float gain=covariance_/(covariance_+2.0f);
    float next=estimate_+gain*(zDps-estimate_);
    covariance_=(1-gain)*covariance_+fabsf(estimate_-next)*0.01f;
    estimate_=next;
    yaw_+=estimate_*(elapsed/1000.0f);
    lastMs_=now;
    return true;
  }
  void invalidate() { healthy_=false; }
  bool healthy(uint32_t now) const { return healthy_ && uint32_t(now-lastMs_)<=100; }
  float yaw() const { return yaw_; }
  float rate() const { return estimate_; }
 private:
  float estimate_=0, covariance_=2, yaw_=0;
  uint32_t lastMs_=0;
  bool healthy_=false;
};
