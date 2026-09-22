#include <Wire.h>
#include <math.h>

#define I2C_SDA 21
#define I2C_SCL 22
#define MPU6050_ADDR 0x68

float gyroZBias = 0.0f;

bool readMPU6050(uint8_t reg, uint8_t *data, uint8_t count) {
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0 ||
      Wire.requestFrom((uint8_t)MPU6050_ADDR, count) != count) {
    return false;
  }
  for (uint8_t i = 0; i < count; ++i) data[i] = Wire.read();
  return true;
}

bool writeMPU6050(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(reg);
  Wire.write(value);
  return Wire.endTransmission() == 0;
}

bool readGyroZ(float &rate) {
  uint8_t data[2];
  if (!readMPU6050(0x47, data, 2)) return false;
  int16_t raw = (int16_t)(((uint16_t)data[0] << 8) | data[1]);
  // MPU6050 sensitivity at +/-500 degrees/second.
  rate = raw / 65.5f;
  return true;
}

bool initMPU6050() {
  uint8_t identity;
  if (!readMPU6050(0x75, &identity, 1) || identity != 0x68) return false;
  if (!writeMPU6050(0x6B, 0x01)) return false; // Wake, X gyro PLL clock.
  delay(100);
  if (!writeMPU6050(0x1A, 0x06) || // DLPF: 5 Hz, matching previous setting.
      !writeMPU6050(0x1B, 0x08)) return false; // +/-500 degrees/second.
  delay(100);

  Serial.println("Calibrating... Please keep the robot completely still.");
  float sum = 0.0f;
  for (int i = 0; i < 200; ++i) {
    float rate;
    if (!readGyroZ(rate)) return false;
    sum += rate;
    delay(10);
  }
  gyroZBias = sum / 200.0f;
  Serial.println("Calibration Done!");
  return true;
}

unsigned long prevTime = 0;
unsigned long lastLogTime = 0;
float yawAngle = 0.0; 

void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);

  Wire.begin(I2C_SDA, I2C_SCL);

  if (!initMPU6050()) {
    Serial.println("MPU6050 Error! Check wiring and I2C address.");
    while (1) delay(10);
  }
  
  Serial.println("MPU6050 yaw ready.");
  Serial.println("Corrected rate | Applied rate | Yaw angle");
  prevTime = millis();
  lastLogTime = prevTime;
}

void loop() {
  // حساب فرق الزمن (dt)
  unsigned long currentTime = millis();
  float dt = (currentTime - prevTime) / 1000.0;
  prevTime = currentTime;

  float gyroZ;
  if (!readGyroZ(gyroZ)) {
    Serial.println("MPU6050 read failed. Reset to recalibrate yaw.");
    while (1) delay(10);
  }

  // إهمال القراءات الصغيرة جداً لمنع تراكم الأخطاء (Drift)
  const float correctedGyroZ = gyroZ - gyroZBias;
  float appliedGyroZ = correctedGyroZ;
  if (fabs(appliedGyroZ) < 0.5) {
    appliedGyroZ = 0.0;
  }

  // حساب زاوية الروبوت
  yawAngle += (appliedGyroZ * dt);

  // طباعة الزاوية الحالية للروبوت بشكل مقروء وواضح
  // Log at 20 Hz without slowing the gyro updates to that rate.
  // appliedGyroZ is the rate integrated into yaw after the deadband.
  if (currentTime - lastLogTime >= 50) {
    lastLogTime = currentTime;
    Serial.print("Rate: ");
    Serial.print(correctedGyroZ, 3);
    Serial.print(" deg/s | Applied: ");
    Serial.print(appliedGyroZ, 3);
    Serial.print(" deg/s | Yaw: ");
    Serial.print(yawAngle, 2);
    Serial.println(" degrees");
  }

  delay(10); 
}
