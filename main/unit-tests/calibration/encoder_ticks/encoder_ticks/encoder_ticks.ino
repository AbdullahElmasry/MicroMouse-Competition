#include <Arduino.h>

constexpr int MOTOR_ENABLE = 23;
constexpr int MOTOR_PINS[] = {25, 26, 27, 14};
constexpr int LEFT_ENCODER = 33, RIGHT_ENCODER = 35;
constexpr int CELLS = 8;
constexpr float DISTANCE_MM = CELLS * 180.0f;
volatile unsigned long leftTicks = 0, rightTicks = 0;
volatile bool recording = false;
unsigned long lastPrint = 0;

void IRAM_ATTR onLeft() { if (recording) ++leftTicks; }
void IRAM_ATTR onRight() { if (recording) ++rightTicks; }

void report(bool finalResult) {
  unsigned long left, right;
  noInterrupts();
  left = leftTicks;
  right = rightTicks;
  interrupts();
  Serial.print("ticks L/R: "); Serial.print(left); Serial.print('/'); Serial.println(right);
  if (!finalResult) return;
  Serial.println("Assuming you manually moved exactly 1440 mm without slipping:");
  Serial.print("LEFT_TICKS_PER_CELL = "); Serial.println(left / (float)CELLS, 3);
  Serial.print("RIGHT_TICKS_PER_CELL = "); Serial.println(right / (float)CELLS, 3);
  Serial.print("Left ticks/mm = "); Serial.println(left / DISTANCE_MM, 6);
  Serial.print("Right ticks/mm = "); Serial.println(right / DISTANCE_MM, 6);
  if (left == 0 || right == 0) Serial.println("INVALID: one or both encoders recorded zero ticks.");
}

void setup() {
  // Keep the driver's enable LOW and all inputs LOW; never command movement.
  pinMode(MOTOR_ENABLE, OUTPUT);
  digitalWrite(MOTOR_ENABLE, LOW);
  for (int pin : MOTOR_PINS) { pinMode(pin, OUTPUT); digitalWrite(pin, LOW); }
  Serial.begin(115200);
  pinMode(LEFT_ENCODER, INPUT_PULLUP);
  pinMode(RIGHT_ENCODER, INPUT);
  attachInterrupt(digitalPinToInterrupt(LEFT_ENCODER), onLeft, RISING);
  attachInterrupt(digitalPinToInterrupt(RIGHT_ENCODER), onRight, RISING);
  Serial.println("Manual encoder calibration: 8 cells = 1440 mm. Motor driver disabled.");
  Serial.println("r: reset/start; s: stop and calculate; p: print counts.");
  Serial.println("Start at your mark, send r, push straight to 1440 mm, stop, then send s.");
}

void loop() {
  if (Serial.available()) {
    char command = Serial.read();
    if (command == 'r') {
      noInterrupts();
      recording = false;
      leftTicks = 0; rightTicks = 0;
      recording = true;
      interrupts();
      Serial.println("Recording. Move only forward; reverse movement also adds ticks.");
    } else if (command == 's') {
      noInterrupts(); recording = false; interrupts();
      report(true);
    } else if (command == 'p') report(false);
  }
  if (recording && millis() - lastPrint >= 250) {
    lastPrint = millis();
    report(false);
  }
}
