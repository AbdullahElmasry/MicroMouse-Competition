#include <Arduino.h>

// These are SOFTWARE labels. Observe which physical wheel actually rotates.
constexpr int LEFT_IN1 = 25, LEFT_IN2 = 26;
constexpr int RIGHT_IN1 = 27, RIGHT_IN2 = 14;
constexpr int MOTOR_EN = 23;
constexpr int LEFT_ENCODER = 33, RIGHT_ENCODER = 35;
constexpr int TEST_PWM = 140; // Raw PWM, no balance factors or PID.
constexpr unsigned long RUN_MS = 3000;
constexpr unsigned long PAUSE_MS = 1000;

volatile unsigned long leftTicks = 0, rightTicks = 0;
void IRAM_ATTR countLeft() { ++leftTicks; }
void IRAM_ATTR countRight() { ++rightTicks; }

enum class Phase { Idle, Left, Pause, Right };
Phase phase = Phase::Idle;
unsigned long phaseStarted = 0, baselineLeft = 0, baselineRight = 0;

void brakeBoth() {
  analogWrite(LEFT_IN1, 255);
  analogWrite(LEFT_IN2, 255);
  analogWrite(RIGHT_IN1, 255);
  analogWrite(RIGHT_IN2, 255);
}

void snapshot(unsigned long &left, unsigned long &right) {
  noInterrupts();
  left = leftTicks;
  right = rightTicks;
  interrupts();
}

void reportTicks(const char *label) {
  unsigned long left, right;
  snapshot(left, right);
  Serial.printf("%s | encoder delta L(GPIO33)/R(GPIO35): %lu/%lu\n",
                label, left - baselineLeft, right - baselineRight);
}

void startLeft() {
  brakeBoth();
  Serial.println("STEP 1: software LEFT output, pins 25/26, forward for 3 seconds.");
  Serial.println("Observe: which physical wheel rotates, and in which direction?");
  snapshot(baselineLeft, baselineRight);
  analogWrite(LEFT_IN1, TEST_PWM);
  analogWrite(LEFT_IN2, 0);
  phaseStarted = millis();
  phase = Phase::Left;
}

void startRight() {
  Serial.println("STEP 2: software RIGHT output, pins 27/14, forward for 3 seconds.");
  Serial.println("Observe: which physical wheel rotates, and in which direction?");
  snapshot(baselineLeft, baselineRight);
  // Same polarity as moving_forward.ino.
  analogWrite(RIGHT_IN1, 0);
  analogWrite(RIGHT_IN2, TEST_PWM);
  phaseStarted = millis();
  phase = Phase::Right;
}

void setup() {
  Serial.begin(115200);
  pinMode(MOTOR_EN, OUTPUT);
  digitalWrite(MOTOR_EN, LOW);
  pinMode(LEFT_IN1, OUTPUT);
  pinMode(LEFT_IN2, OUTPUT);
  pinMode(RIGHT_IN1, OUTPUT);
  pinMode(RIGHT_IN2, OUTPUT);
  brakeBoth();
  pinMode(LEFT_ENCODER, INPUT_PULLUP);
  pinMode(RIGHT_ENCODER, INPUT);
  attachInterrupt(digitalPinToInterrupt(LEFT_ENCODER), countLeft, RISING);
  attachInterrupt(digitalPinToInterrupt(RIGHT_ENCODER), countRight, RISING);
  digitalWrite(MOTOR_EN, HIGH);
  Serial.println("Motor mapping test. Lift both wheels off the ground.");
  Serial.println("USB Serial 115200: s = left 3s, pause 1s, right 3s; d = stop/cancel.");
  Serial.println("Left/right physical sides are viewed facing forward with the robot.");
}

void loop() {
  bool startRequested = false, stopRequested = false;
  const int pending = Serial.available();
  for (int i = 0; i < pending; ++i) {
    const char command = Serial.read();
    if (command == 'd' || command == 'D') stopRequested = true;
    if (command == 's' || command == 'S') startRequested = true;
  }
  // Stop wins if both commands arrived together. Starts while running are ignored.
  if (stopRequested) {
    brakeBoth();
    if (phase == Phase::Left || phase == Phase::Right) reportTicks("ABORTED");
    phase = Phase::Idle;
    Serial.println("Stopped; sequence cancelled. Send s to repeat.");
    return;
  }
  if (phase == Phase::Idle && startRequested) startLeft();

  const unsigned long elapsed = millis() - phaseStarted;
  if (phase == Phase::Left && elapsed >= RUN_MS) {
    brakeBoth();
    reportTicks("LEFT OUTPUT FINISHED");
    Serial.println("Both motors braked; 1 second pause.");
    phase = Phase::Pause;
    phaseStarted = millis();
  } else if (phase == Phase::Pause && elapsed >= PAUSE_MS) {
    startRight();
  } else if (phase == Phase::Right && elapsed >= RUN_MS) {
    brakeBoth();
    reportTicks("RIGHT OUTPUT FINISHED");
    phase = Phase::Idle;
    Serial.println("Test complete. Both motors braked. Send s to repeat.");
  }
  delay(1);
}
