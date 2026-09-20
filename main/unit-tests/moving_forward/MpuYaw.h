#pragma once
#include <Wire.h>
#include <MPU6500_WE.h>
#include "YawEstimate.h"

class MpuYaw {
 public:
  bool begin() {
    // Wire is already initialized on the shared SDA 21/SCL 22 bus.
    if (!device_.init()) return false;
    device_.autoOffsets(); // Robot must remain stationary, as in tested calibration.
    device_.enableGyrDLPF();
    device_.setGyrDLPF(MPU6500_DLPF_6);
    device_.setGyrRange(MPU6500_GYRO_RANGE_500);
    offsetZ_=device_.getGyrOffsets().z;
    uint8_t config[2];
    if (!readBytes(0x1A, config, 2) || (config[0]&7)!=6 || (config[1]&0x1B)!=8) return false;
    ready_=isfinite(offsetZ_);
    reset();
    return ready_;
  }
  void reset() { estimate_.reset(millis()); lastPollMs_=millis(); }
  void service() {
    if (!ready_ || millis()-lastPollMs_<10) return;
    lastPollMs_=millis();
    uint8_t raw[2];
    if (!readBytes(0x47,raw,2)) { estimate_.invalidate(); return; }
    int16_t z=(int16_t)(((uint16_t)raw[0]<<8)|raw[1]);
    // Equivalent to MPU6500_WE getGyrValues() at range factor 2 (500 dps).
    // Checked reads avoid interpreting an I2C failure as zero angular velocity.
    float zDps=((float)z-offsetZ_/2.0f)*(500.0f/32768.0f);
    if (abs((int)z)>32700) { estimate_.invalidate(); return; }
    estimate_.update(zDps,millis());
  }
  bool healthy() const { return ready_ && estimate_.healthy(millis()); }
  float yaw() const { return estimate_.yaw(); }
  float rate() const { return estimate_.rate(); }
 private:
  bool readBytes(uint8_t reg,uint8_t *data,uint8_t count) {
    Wire.beginTransmission(0x68); Wire.write(reg);
    if (Wire.endTransmission(false)!=0) return false;
    if (Wire.requestFrom((uint8_t)0x68,count)!=count) return false;
    for (uint8_t i=0;i<count;++i) data[i]=Wire.read();
    return true;
  }
  MPU6500_WE device_{0x68};
  YawEstimate estimate_;
  bool ready_=false;
  float offsetZ_=0;
  unsigned long lastPollMs_=0;
};
