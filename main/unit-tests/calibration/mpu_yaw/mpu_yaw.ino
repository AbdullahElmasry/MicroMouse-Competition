#include <Arduino.h>
#include <Wire.h>

// MPU-6050 register map only. Refuse other identities instead of guessing.
constexpr uint8_t WHO_AM_I = 0x75, GYRO_ZOUT_H = 0x47;
constexpr float GYRO_LSB_PER_DPS = 131.0f; // +/-250 degrees/second
uint8_t mpuAddress = 0;
bool ready = false, haveSample = false, gapFault = false;
float yawDegrees = 0;
unsigned long previousSampleUs = 0, lastPrintMs = 0;

bool readRegisters(uint8_t address, uint8_t reg, uint8_t *data, uint8_t count) {
  Wire.beginTransmission(address);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(address, count) != count) return false;
  for (uint8_t i = 0; i < count; ++i) data[i] = Wire.read();
  return true;
}

bool writeRegister(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(mpuAddress);
  Wire.write(reg); Wire.write(value);
  return Wire.endTransmission() == 0;
}

void resetYaw() {
  yawDegrees = 0;
  haveSample = false;
  gapFault = false;
  Serial.println("Yaw zeroed; no gyro bias correction or filtering applied.");
}

void setup() {
  pinMode(23, OUTPUT); digitalWrite(23, LOW);
  const int motorPins[] = {25, 26, 27, 14};
  for (int pin : motorPins) { pinMode(pin, OUTPUT); digitalWrite(pin, LOW); }
  Serial.begin(115200);
  Wire.begin(); // Shared ToF/MPU bus: board-default SDA and SCL.
  delay(100);
  for (uint8_t address = 0x68; address <= 0x69; ++address) {
    uint8_t id;
    if (!readRegisters(address, WHO_AM_I, &id, 1)) continue;
    Serial.print("I2C address 0x"); Serial.print(address, HEX);
    Serial.print(" WHO_AM_I = 0x"); Serial.println(id, HEX);
    if (id == 0x68 && mpuAddress == 0) mpuAddress = address;
  }
  if (mpuAddress == 0) {
    Serial.println("No supported MPU-6050 found. Report your MPU model / WHO_AM_I value.");
    return;
  }
  if (!writeRegister(0x6B, 0x80)) return; // Device reset.
  delay(100);
  // PLL clock, all axes enabled, DLPF_CFG=0 (widest gyro bandwidth),
  // +/-250 dps, 200 Hz output (8 kHz / (39+1)), data-ready status enabled.
  ready = writeRegister(0x6B, 0x01) && writeRegister(0x6C, 0x00) &&
          writeRegister(0x1A, 0x00) && writeRegister(0x1B, 0x00) &&
          writeRegister(0x19, 39) && writeRegister(0x38, 0x01);
  if (!ready) { Serial.println("MPU configuration failed."); return; }
  delay(100);
  resetYaw();
  Serial.println("Keep board level with sensor Z pointing up. r: zero relative yaw.");
  Serial.println("No software filter, fusion, DMP, or bias subtraction. Sensor bandwidth remains finite.");
  Serial.println("ms,raw_gz,gz_deg_per_s,relative_yaw_deg");
}

void loop() {
  if (!ready) { delay(10); return; }
  if (Serial.available() && Serial.read() == 'r') resetYaw();
  if (gapFault) { delay(1); return; }
  uint8_t status;
  if (!readRegisters(mpuAddress, 0x3A, &status, 1)) {
    Serial.println("I2C ERROR: yaw invalid; send r to restart.");
    gapFault = true; return;
  }
  if (!(status & 0x01)) {
    if (haveSample && (unsigned long)(micros() - previousSampleUs) > 100000UL) {
      Serial.println("NO DATA: yaw invalid; send r to restart.");
      gapFault = true;
    }
    delay(1); return;
  }
  uint8_t data[2];
  if (!readRegisters(mpuAddress, GYRO_ZOUT_H, data, 2)) {
    Serial.println("I2C ERROR: yaw invalid; send r to restart.");
    gapFault = true; return;
  }
  unsigned long nowUs = micros();
  int16_t rawZ = (int16_t)(((uint16_t)data[0] << 8) | data[1]);
  float rate = rawZ / GYRO_LSB_PER_DPS;
  if (haveSample) {
    unsigned long elapsedUs = nowUs - previousSampleUs;
    if (elapsedUs > 20000UL) {
      Serial.println("SAMPLE GAP: yaw invalid; send r to restart.");
      gapFault = true; return;
    }
    yawDegrees += rate * (elapsedUs / 1000000.0f);
  }
  previousSampleUs = nowUs;
  haveSample = true;
  if (millis() - lastPrintMs >= 50) {
    lastPrintMs = millis();
    Serial.print(lastPrintMs); Serial.print(','); Serial.print(rawZ);
    Serial.print(','); Serial.print(rate, 4);
    Serial.print(','); Serial.println(yawDegrees, 4);
    if (rawZ >= 32700 || rawZ <= -32700) Serial.println("GYRO NEAR LIMIT: turn slower; yaw may be inaccurate.");
  }
}
