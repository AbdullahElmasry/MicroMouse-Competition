#include <Wire.h>
#include <MPU6500_WE.h>
#include <math.h>

#define I2C_SDA 21
#define I2C_SCL 22
#define MPU6500_ADDR 0x68

MPU6500_WE myMPU6500 = MPU6500_WE(MPU6500_ADDR);

// ==========================================
// فئة فلتر كالمان المبسط (Simple 1D Kalman Filter)
// ==========================================
class SimpleKalmanFilter {
  private:
    float _err_measure;
    float _err_estimate;
    float _q;
    float _current_estimate;
    float _last_estimate;
    float _kalman_gain;
    
  public:
    SimpleKalmanFilter(float mea_e, float est_e, float q_var) {
      _err_measure = mea_e;
      _err_estimate = est_e;
      _q = q_var;
      _current_estimate = 0.0;
      _last_estimate = 0.0;
    }
    
    float updateEstimate(float mea) {
      _kalman_gain = _err_estimate / (_err_estimate + _err_measure);
      _current_estimate = _last_estimate + _kalman_gain * (mea - _last_estimate);
      _err_estimate =  (1.0 - _kalman_gain) * _err_estimate + fabs(_last_estimate - _current_estimate) * _q;
      _last_estimate = _current_estimate;
      return _current_estimate;
    }
};

// إنشاء فلتر لمحور Z
SimpleKalmanFilter kalmanGyroZ(2.0, 2.0, 0.01);

unsigned long prevTime = 0;
float yawAngle = 0.0; 

void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);

  Wire.begin(I2C_SDA, I2C_SCL);

  if (!myMPU6500.init()) {
    Serial.println("MPU6500 Error! Check wiring.");
    while (1) delay(10);
  }
  
  Serial.println("Calibrating... Please keep the robot completely still.");
  myMPU6500.autoOffsets();
  Serial.println("Calibration Done!");
  
  myMPU6500.enableGyrDLPF(); 
  myMPU6500.setGyrDLPF(MPU6500_DLPF_6); 
  myMPU6500.setGyrRange(MPU6500_GYRO_RANGE_500); 
  
  prevTime = millis();
}

void loop() {
  // حساب فرق الزمن (dt)
  unsigned long currentTime = millis();
  float dt = (currentTime - prevTime) / 1000.0;
  prevTime = currentTime;

  xyzFloat gyr = myMPU6500.getGyrValues();

  // إهمال القراءات الصغيرة جداً لمنع تراكم الأخطاء (Drift)
  float rawGyroZ = gyr.z;
  if (fabs(rawGyroZ) < 0.5) { 
    rawGyroZ = 0.0; 
  }

  // تمرير القراءة عبر فلتر كالمان
  float filteredGyroZ = kalmanGyroZ.updateEstimate(rawGyroZ);

  // حساب زاوية الروبوت
  yawAngle += (filteredGyroZ * dt);

  // طباعة الزاوية الحالية للروبوت بشكل مقروء وواضح
  Serial.print("Current Angle (Yaw): ");
  Serial.print(yawAngle);
  Serial.println(" degrees");

  delay(10); 
}