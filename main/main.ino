#include <Wire.h>
#include <VL6180X.h>
#include <VL53L1X.h>

// ==========================================
// 1. التعريفات الأساسية
// ==========================================

#define MOTOR_LEFT_IN1 25
#define MOTOR_LEFT_IN2 26
#define MOTOR_RIGHT_IN1 27
#define MOTOR_RIGHT_IN2 14
#define MOTOR_EN_PIN 23 

#define ENCODER_LEFT_C1 33
#define ENCODER_RIGHT_C1 35

#define XSHUT_LEFT 4
#define XSHUT_RIGHT 5
#define XSHUT_FRONT 16 

volatile long leftTicks = 0;
volatile long rightTicks = 0;

// ==========================================
// بارامترات الحركة والمسافات (المعايرة الخاصة بك)
// ==========================================

int targetTicks = 600;          // نبضات الخلية الواحدة (18 سم)
int ticksPer90Degree = 620;     // النبضات الدقيقة للفة 90 درجة 

// معاملات توازن المواتير
float leftMotorFactor = 1.0;  
float rightMotorFactor = 0.78; 

// السرعات
int baseSpeed     = 70;  // سرعة الحركة المستقيمة
int slowSpeed     = 30;  // سرعة التعديل الجانبي
int maxTurnSpeed  = 90;  // سرعة الدوران القصوى
int minTurnSpeed  = 60;  // سرعة التهدئة قبل الوقوف (Soft Stop)

// حدود الاقتراب والحيطان بالملليمتر
int leftThreshold    = 40;   
int rightThreshold   = 50;   
int frontThreshold   = 100;  
int wallOpenDistance = 140; // لو المسافة أكبر من 14 سم، يعتبر المسار مفتوح

// كائنات حساسات الـ ToF
VL6180X tofLeft;
VL6180X tofRight;
VL53L1X tofFront; 

// ==========================================
// 2. الدوال الأساسية للمواتير واللف بالانكودر
// ==========================================

void IRAM_ATTR countLeft() { leftTicks++; }
void IRAM_ATTR countRight() { rightTicks++; }

void setMotorSpeed(int speedLeft, int speedRight) {
  int balancedLeftSpeed = speedLeft * leftMotorFactor;
  int balancedRightSpeed = speedRight * rightMotorFactor;

  if (balancedLeftSpeed > 0) {
    analogWrite(MOTOR_LEFT_IN1, balancedLeftSpeed);           
    analogWrite(MOTOR_LEFT_IN2, 0);   
  } else if (balancedLeftSpeed < 0) {
    analogWrite(MOTOR_LEFT_IN1, 0); 
    analogWrite(MOTOR_LEFT_IN2, abs(balancedLeftSpeed));              
  } else {
    analogWrite(MOTOR_LEFT_IN1, 255); 
    analogWrite(MOTOR_LEFT_IN2, 255);
  }

  if (balancedRightSpeed > 0) {
    analogWrite(MOTOR_RIGHT_IN1, 0);
    analogWrite(MOTOR_RIGHT_IN2, balancedRightSpeed);
  } else if (balancedRightSpeed < 0) {
    analogWrite(MOTOR_RIGHT_IN1, abs(balancedRightSpeed));
    analogWrite(MOTOR_RIGHT_IN2, 0);
  } else {
    analogWrite(MOTOR_RIGHT_IN1, 255); 
    analogWrite(MOTOR_RIGHT_IN2, 255);
  }
}

// دالة الدوران الدقيقة بالانكودر (بعد تعديل الاتجاهات)
void turnEncoder(float angleDeg) {
  leftTicks = 0;
  rightTicks = 0;
  
  long requiredTicks = (abs(angleDeg) / 90.0) * ticksPer90Degree;
  
  while (leftTicks < requiredTicks && rightTicks < requiredTicks) {
    int currentTurnSpeed = maxTurnSpeed;
    
    // تهدئة السرعة (Soft Stop) لامتصاص القصور الذاتي
    long avgTicks = (leftTicks + rightTicks) / 2;
    if (requiredTicks - avgTicks < 70) {
      currentTurnSpeed = minTurnSpeed;
    }

    // تم عكس الإشارات لتصحيح اتجاه الدوران
    if (angleDeg > 0) {
      setMotorSpeed(-currentTurnSpeed, currentTurnSpeed); // دوران يمين
    } else {
      setMotorSpeed(currentTurnSpeed, -currentTurnSpeed); // دوران شمال
    }
    delay(2);
  }

  setMotorSpeed(0, 0); // فرملة تامة
  delay(100); // راحة بسيطة جداً بعد اللف
}

// ==========================================
// 3. دالة التحرك خلية واحدة
// ==========================================

void moveOneCell() {
  leftTicks = 0;
  rightTicks = 0;
  
  while (true) {
    if (leftTicks >= targetTicks || rightTicks >= targetTicks) {
      setMotorSpeed(0, 0); 
      break;
    }
    
    int distFront = tofFront.read();
    if (distFront < frontThreshold && !tofFront.timeoutOccurred()) {
      setMotorSpeed(0, 0);
      break; 
    }

    int distLeft = tofLeft.readRangeSingleMillimeters();
    int distRight = tofRight.readRangeSingleMillimeters();
    
    int currentLeftSpeed = baseSpeed;
    int currentRightSpeed = baseSpeed;
    
    // تعديل المسار الجانبي بنعومة
    if (distLeft < leftThreshold && !tofLeft.timeoutOccurred()) {
      currentLeftSpeed = slowSpeed;  
      currentRightSpeed = baseSpeed; 
    } 
    else if (distRight < rightThreshold && !tofRight.timeoutOccurred()) {
      currentLeftSpeed = baseSpeed;  
      currentRightSpeed = slowSpeed; 
    } 
    
    setMotorSpeed(currentLeftSpeed, currentRightSpeed);
    delay(10); 
  }
}

// ==========================================
// 4. التصحيح المطلق (Front Wall Calibration)
// ==========================================
// لو الروبوت زحف شوية قدام الحيطة السد، الدالة دي هترجعه أو تقدمه لمكانه المثالي بالمللي
void calibrateWithFrontWall() {
  unsigned long startMs = millis();
  
  while (true) {
    int distFront = tofFront.read();
    if (tofFront.timeoutOccurred() || millis() - startMs > 600) break; 
    
    int error = distFront - frontThreshold;
    if (abs(error) <= 3) break; // نسبة سماحية 3 مم

    int alignSpeed = error * 2; 
    if (alignSpeed > 0) {
      alignSpeed = constrain(alignSpeed, 35, slowSpeed); 
    } else {
      alignSpeed = constrain(alignSpeed, -slowSpeed, -35); 
    }

    setMotorSpeed(alignSpeed, alignSpeed);
    delay(5);
  }
  
  setMotorSpeed(0, 0);
  delay(50); 
}

// ==========================================
// 5. Setup
// ==========================================

void setup() {
  Serial.begin(115200);
  Wire.begin(); 
  
  pinMode(MOTOR_EN_PIN, OUTPUT);
  digitalWrite(MOTOR_EN_PIN, HIGH); 
  
  pinMode(MOTOR_LEFT_IN1, OUTPUT);
  pinMode(MOTOR_LEFT_IN2, OUTPUT);
  pinMode(MOTOR_RIGHT_IN1, OUTPUT);
  pinMode(MOTOR_RIGHT_IN2, OUTPUT);

  pinMode(ENCODER_LEFT_C1, INPUT_PULLUP); 
  pinMode(ENCODER_RIGHT_C1, INPUT);       
  attachInterrupt(digitalPinToInterrupt(ENCODER_LEFT_C1), countLeft, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCODER_RIGHT_C1), countRight, RISING);

  pinMode(XSHUT_LEFT, OUTPUT);
  pinMode(XSHUT_RIGHT, OUTPUT);
  pinMode(XSHUT_FRONT, OUTPUT);
  
  digitalWrite(XSHUT_LEFT, LOW);
  digitalWrite(XSHUT_RIGHT, LOW);
  digitalWrite(XSHUT_FRONT, LOW);
  delay(10);
  
  digitalWrite(XSHUT_LEFT, HIGH);
  delay(50);
  tofLeft.init();
  tofLeft.configureDefault();
  tofLeft.setAddress(0x30); 
  tofLeft.setTimeout(200); 

  digitalWrite(XSHUT_RIGHT, HIGH);
  delay(50);
  tofRight.init();
  tofRight.configureDefault();
  tofRight.setAddress(0x31); 
  tofRight.setTimeout(200);

  digitalWrite(XSHUT_FRONT, HIGH);
  delay(50);
  tofFront.init();
  tofFront.setDistanceMode(VL53L1X::Medium); 
  tofFront.setAddress(0x32); 
  tofFront.setTimeout(200);
  tofFront.startContinuous(30); 

  Serial.println("الروبوت جاهز لاجتياز المتاهة...");
  delay(2000); 
}

// ==========================================
// 6. Loop (الملاحة واتخاذ القرار)
// ==========================================

void loop() {
  // 1. تحرك خلية واحدة للأمام
  moveOneCell();
  
  delay(200); // وقفة خفيفة في مركز الخلية قبل أخذ القراءات

  // 2. قراءة المسافات لتحديد الاتجاهات المتاحة
  int distLeft = tofLeft.readRangeSingleMillimeters();
  int distRight = tofRight.readRangeSingleMillimeters();
  int distFront = tofFront.read();

  bool leftOpen  = (distLeft > wallOpenDistance) || tofLeft.timeoutOccurred();
  bool rightOpen = (distRight > wallOpenDistance) || tofRight.timeoutOccurred();
  bool frontOpen = (distFront > frontThreshold) && !tofFront.timeoutOccurred();

  // 3. معايرة الأوفر شوت (لو قدامه حيطة سد هيستغلها لضبط المسافة بالمللي)
  if (!frontOpen) {
    calibrateWithFrontWall();
  }

  // 4. خوارزمية اتخاذ القرار بالانكودر
  if (leftOpen) {
    Serial.println("شمال فاضي -> لف شمال");
    turnEncoder(-90.0); // الدوران شمال
  } 
  else if (rightOpen) {
    Serial.println("يمين فاضي -> لف يمين");
    turnEncoder(90.0);  // الدوران يمين
  } 
  else if (!frontOpen) {
    Serial.println("طريق مسدود -> لف 180 للرجوع");
    turnEncoder(180.0); // الدوران 180
  }
  else {
    Serial.println("طريق سالك -> مكمل لقدام");
  }

  delay(200); // راحة بسيطة قبل ما يندفع للخلية اللي بعدها
}