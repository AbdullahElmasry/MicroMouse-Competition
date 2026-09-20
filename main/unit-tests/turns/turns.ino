#include <Wire.h>
#include <VL6180X.h>
#include <VL53L1X.h>

// Standalone hardware test. Pinout and starting calibration from main.ino.
constexpr int LEFT_IN1 = 25, LEFT_IN2 = 26;
constexpr int RIGHT_IN1 = 27, RIGHT_IN2 = 14, MOTOR_EN = 23;
constexpr int LEFT_ENCODER = 33, RIGHT_ENCODER = 35;
constexpr int LEFT_XSHUT = 4, RIGHT_XSHUT = 5, FRONT_XSHUT = 16;
constexpr long CELL_TICKS = 600, QUARTER_TURN_TICKS = 620;
constexpr int FORWARD_SPEED = 70, TURN_SPEED = 90, TURN_SLOW_SPEED = 60;
constexpr int FRONT_STOP_MM = 100;
constexpr unsigned long MOVE_TIMEOUT_MS = 15000, STALL_TIMEOUT_MS = 1500;
constexpr float LEFT_FACTOR = 1.0f, RIGHT_FACTOR = 0.78f;

volatile unsigned long leftTicks = 0, rightTicks = 0;
VL6180X leftTof, rightTof;
VL53L1X frontTof;
bool sensorsReady = false;

void IRAM_ATTR countLeft() { ++leftTicks; }
void IRAM_ATTR countRight() { ++rightTicks; }

void readTicks(unsigned long &left, unsigned long &right) {
  noInterrupts();
  left = leftTicks;
  right = rightTicks;
  interrupts();
}

void motor(int a, int b, int speed, float factor, bool reversePolarity) {
  int pwm = constrain((int)(abs(speed) * factor), 0, 255);
  if (speed == 0) {
    analogWrite(a, 255);
    analogWrite(b, 255);
    return;
  }
  bool positive = speed > 0;
  if (reversePolarity) positive = !positive;
  analogWrite(a, positive ? pwm : 0);
  analogWrite(b, positive ? 0 : pwm);
}

void drive(int left, int right) {
  motor(LEFT_IN1, LEFT_IN2, left, LEFT_FACTOR, false);
  motor(RIGHT_IN1, RIGHT_IN2, right, RIGHT_FACTOR, true);
}

bool sensorResponds(uint8_t address) {
  Wire.beginTransmission(address);
  return Wire.endTransmission() == 0;
}

bool initSensors() {
  Wire.begin(); // Uses the board's default SDA/SCL, as in main.ino.
  pinMode(LEFT_XSHUT, OUTPUT);
  pinMode(RIGHT_XSHUT, OUTPUT);
  pinMode(FRONT_XSHUT, OUTPUT);
  digitalWrite(LEFT_XSHUT, LOW);
  digitalWrite(RIGHT_XSHUT, LOW);
  digitalWrite(FRONT_XSHUT, LOW);
  delay(10);
  digitalWrite(LEFT_XSHUT, HIGH);
  delay(50);
  leftTof.setTimeout(200);
  if (!sensorResponds(0x29)) return false;
  leftTof.init(); // VL6180X::init() returns void.
  if (leftTof.last_status != 0) return false;
  leftTof.configureDefault();
  if (leftTof.last_status != 0) return false;
  leftTof.setAddress(0x30);
  if (leftTof.last_status != 0 || !sensorResponds(0x30)) return false;
  digitalWrite(RIGHT_XSHUT, HIGH);
  delay(50);
  rightTof.setTimeout(200);
  if (!sensorResponds(0x29)) return false;
  rightTof.init(); // VL6180X::init() returns void.
  if (rightTof.last_status != 0) return false;
  rightTof.configureDefault();
  if (rightTof.last_status != 0) return false;
  rightTof.setAddress(0x31);
  if (rightTof.last_status != 0 || !sensorResponds(0x31)) return false;
  digitalWrite(FRONT_XSHUT, HIGH);
  delay(50);
  frontTof.setTimeout(200);
  if (!frontTof.init()) return false;
  frontTof.setDistanceMode(VL53L1X::Medium);
  frontTof.setAddress(0x32);
  frontTof.setMeasurementTimingBudget(50000);
  frontTof.startContinuous(50);
  return true;
}

bool observe(int &front, int &left, int &right) {
  front = frontTof.read();
  bool frontValid = !frontTof.timeoutOccurred() &&
                    frontTof.ranging_data.range_status == VL53L1X::RangeValid;
  left = leftTof.readRangeSingleMillimeters();
  bool leftValid = !leftTof.timeoutOccurred() && leftTof.readRangeStatus() == 0;
  right = rightTof.readRangeSingleMillimeters();
  bool rightValid = !rightTof.timeoutOccurred() && rightTof.readRangeStatus() == 0;
  if (!leftValid) left = -1;
  if (!rightValid) right = -1;
  return frontValid; // Side sensors are diagnostic; no wall following in this test.
}

void runMove(long target, int leftSign, int rightSign, bool forward) {
  if (forward && !sensorsReady) {
    Serial.println("REFUSED: ToF initialization failed.");
    return;
  }
  unsigned long startLeft, startRight;
  readTicks(startLeft, startRight);
  unsigned long started = millis(), lastLeftChange = started, lastRightChange = started;
  unsigned long previousLeft = 0, previousRight = 0, lastLog = started;
  const char *result = "TIMEOUT";
  while (true) {
    bool abortRequested = false;
    while (Serial.available()) {
      char command = Serial.read();
      if (command == 'x' || command == 'X') abortRequested = true;
    }
    if (abortRequested) { result = "ABORTED"; break; }
    unsigned long rawLeft, rawRight;
    readTicks(rawLeft, rawRight);
    unsigned long left = rawLeft - startLeft, right = rawRight - startRight;
    unsigned long now = millis();
    if (left != previousLeft) { lastLeftChange = now; previousLeft = left; }
    if (right != previousRight) { lastRightChange = now; previousRight = right; }
    if (left >= (unsigned long)target && right >= (unsigned long)target) {
      result = "ENCODER TARGET REACHED"; break;
    }
    if (now - started >= MOVE_TIMEOUT_MS) break;
    if ((left < (unsigned long)target && now - lastLeftChange >= STALL_TIMEOUT_MS) ||
        (right < (unsigned long)target && now - lastRightChange >= STALL_TIMEOUT_MS)) {
      result = "ENCODER STALL"; break;
    }
    int front = -1, sideLeft = -1, sideRight = -1;
    if (forward) {
      if (!observe(front, sideLeft, sideRight)) { result = "INVALID TOF READING"; break; }
      if (front < FRONT_STOP_MM) { result = "FRONT OBSTACLE"; break; }
    }
    int speed = forward ? FORWARD_SPEED : TURN_SPEED;
    if (!forward && target - (long)min(left, right) < 70) speed = TURN_SLOW_SPEED;
    // Stop each wheel at its own target; a fast wheel cannot finish the other one.
    drive(left >= (unsigned long)target ? 0 : leftSign * speed,
          right >= (unsigned long)target ? 0 : rightSign * speed);
    if (now - lastLog >= 200) {
      Serial.print("ticks L/R: "); Serial.print(left); Serial.print('/'); Serial.print(right);
      if (forward) {
        Serial.print("  mm F/L/R: "); Serial.print(front); Serial.print('/');
        Serial.print(sideLeft); Serial.print('/'); Serial.print(sideRight);
      }
      Serial.println();
      lastLog = now;
    }
    delay(2);
  }
  drive(0, 0);
  delay(100);
  unsigned long endLeft, endRight;
  readTicks(endLeft, endRight);
  Serial.print(result);
  Serial.print(" | target per wheel: "); Serial.print(target);
  Serial.print(" | final L/R: "); Serial.print(endLeft - startLeft);
  Serial.print('/'); Serial.print(endRight - startRight);
  Serial.print(" | elapsed ms: "); Serial.println(millis() - started);
  Serial.println("Measure physical distance/angle; encoder completion alone is not a pass.");
}

void setup() {
  Serial.begin(115200);
  pinMode(MOTOR_EN, OUTPUT);
  digitalWrite(MOTOR_EN, LOW);
  pinMode(LEFT_IN1, OUTPUT); pinMode(LEFT_IN2, OUTPUT);
  pinMode(RIGHT_IN1, OUTPUT); pinMode(RIGHT_IN2, OUTPUT);
  drive(0, 0);
  pinMode(LEFT_ENCODER, INPUT_PULLUP);
  pinMode(RIGHT_ENCODER, INPUT);
  attachInterrupt(digitalPinToInterrupt(LEFT_ENCODER), countLeft, RISING);
  attachInterrupt(digitalPinToInterrupt(RIGHT_ENCODER), countRight, RISING);
  sensorsReady = initSensors();
  digitalWrite(MOTOR_EN, HIGH);
  Serial.println(sensorsReady ? "ToF initialization OK." : "ToF initialization FAILED.");
  Serial.println("l: left 90, r: right 90, u: right 180.");
  Serial.println("Send x to stop. No movement starts automatically.");
}

void loop() {
  if (!Serial.available()) return;
  char command = Serial.read();
  if (command == 'l') runMove(QUARTER_TURN_TICKS, 1, -1, false);
  if (command == 'r') runMove(QUARTER_TURN_TICKS, -1, 1, false);
  if (command == 'u') runMove(2 * QUARTER_TURN_TICKS, -1, 1, false);
}
