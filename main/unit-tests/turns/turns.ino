#include <Wire.h>
#include <WiFi.h>

#include "../moving_forward/MpuYaw.h"
#include "DifferentialTurn.h"
#include "TurnTarget.h"

// ============================================================
// WiFi
// ============================================================

const char *WIFI_SSID = "عبدالله";
const char *WIFI_PASSWORD = "1234567899";

constexpr uint16_t WIFI_SERIAL_PORT = 23;

WiFiServer wifiServer(WIFI_SERIAL_PORT);
WiFiClient wifiClient;

// ============================================================
// Hardware
// ============================================================

// Standalone 90-degree turn test. Keep the robot still during MPU calibration.
constexpr int LEFT_IN1 = 25, LEFT_IN2 = 26;
constexpr int RIGHT_IN1 = 27, RIGHT_IN2 = 14, MOTOR_EN = 23;
constexpr int LEFT_ENCODER = 33, RIGHT_ENCODER = 35;

// Turn test intentionally applies identical raw PWM to both motors.
constexpr float LEFT_FACTOR = 1.0f;
constexpr float RIGHT_FACTOR = 1.0f;
// Lower fixed speed reduces tire slip, MPU filter lag, and braking momentum.
// Both wheels still receive exactly the same raw PWM magnitude.
constexpr int TURN_PWM = 60;

// Verified MPU mounting: raw yaw decreases on a physical right turn.
constexpr float MPU_YAW_SIGN = -1.0f;

constexpr float TURN_TARGET_DEGREES = 90.0f;
constexpr float TURN_TOLERANCE_DEGREES = 1.5f;
// Hardware logs show 10-18 degrees of motion after braking at PWM 80.
// Predict that motion from yaw rate while retaining equal fixed motor PWM.
constexpr float TURN_BASE_BRAKE_LEAD_DEGREES = 4.0f;
constexpr float TURN_BRAKE_LOOKAHEAD_SECONDS = 0.050f;

// The old 620-tick estimate is diagnostic only. Yaw ends the turn;
// this larger limit catches a failed MPU or runaway turn without
// calibrating angle by ticks.
constexpr unsigned long TURN_ENCODER_SAFETY_TICKS = 900;
constexpr unsigned long TURN_TIMEOUT_MS = 5000;
constexpr unsigned long STALL_TIMEOUT_MS = 1000;


// ============================================================
// Globals
// ============================================================

volatile unsigned long leftTicks = 0;
volatile unsigned long rightTicks = 0;

MpuYaw mpuYaw;

bool mpuReady = false;

// ============================================================
// Logging
// ============================================================

void logPrint(const String &text) {
  Serial.print(text);

  if (wifiClient && wifiClient.connected()) {
    wifiClient.print(text);
  }
}

void logPrintln(const String &text = "") {
  Serial.println(text);

  if (wifiClient && wifiClient.connected()) {
    wifiClient.println(text);
  }
}

// ============================================================
// WiFi
// ============================================================

void setupWiFi() {
  WiFi.mode(WIFI_STA);

  logPrint("Connecting to WiFi: ");
  logPrintln(WIFI_SSID);

  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  unsigned long start = millis();

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");

    if (millis() - start > 15000) {
      Serial.println();
      Serial.println("WiFi connection timeout.");
      return;
    }
  }

  Serial.println();
  logPrintln("WiFi connected successfully.");

  logPrint("ESP32 IP address: ");
  logPrintln(WiFi.localIP().toString());

  wifiServer.begin();
  wifiServer.setNoDelay(true);

  logPrint("WiFi Serial port: ");
  logPrintln(String(WIFI_SERIAL_PORT));

  logPrintln("TCP serial server started.");
}

void serviceWiFiClient() {
  // Remove dead client.
  if (wifiClient && !wifiClient.connected()) {
    wifiClient.stop();
  }

  // Accept a new client.
  if (!wifiClient || !wifiClient.connected()) {
    WiFiClient newClient = wifiServer.available();

    if (newClient) {
      wifiClient = newClient;
      wifiClient.setNoDelay(true);

      logPrintln();
      logPrintln("WiFi client connected.");
      logPrintln("Commands:");
      logPrintln("  l = simultaneous-wheel left 90 degrees");
      logPrintln("  r = simultaneous-wheel right 90 degrees");
      logPrintln("  d/x = stop");
    }
  }
}

// ============================================================
// Encoders
// ============================================================

void IRAM_ATTR countLeft() {
  ++leftTicks;
}

void IRAM_ATTR countRight() {
  ++rightTicks;
}

void readTicks(unsigned long &left, unsigned long &right) {
  noInterrupts();

  left = leftTicks;
  right = rightTicks;

  interrupts();
}

// ============================================================
// Motors
// ============================================================

void motor(
    int a,
    int b,
    int speed,
    float factor,
    bool reversePolarity) {

  int pwm = constrain((int)(abs(speed) * factor), 0, 255);

  if (speed == 0) {
    analogWrite(a, 255);
    analogWrite(b, 255);
    return;
  }

  bool positive = speed > 0;

  if (reversePolarity) {
    positive = !positive;
  }

  analogWrite(a, positive ? pwm : 0);
  analogWrite(b, positive ? 0 : pwm);
}

void drive(int left, int right) {
  motor(
      LEFT_IN1,
      LEFT_IN2,
      left,
      LEFT_FACTOR,
      false);

  motor(
      RIGHT_IN1,
      RIGHT_IN2,
      right,
      RIGHT_FACTOR,
      true);
}

// ============================================================
// Command handling
// ============================================================

int readCommand() {
  // USB Serial has priority.
  if (Serial.available()) {
    return Serial.read();
  }

  serviceWiFiClient();

  if (wifiClient &&
      wifiClient.connected() &&
      wifiClient.available()) {

    return wifiClient.read();
  }

  return -1;
}

bool abortRequested() {
  bool stop = false;

  // Check USB serial.
  while (Serial.available()) {
    char command = Serial.read();

    if (command == 'd' ||
        command == 'D' ||
        command == 'x' ||
        command == 'X') {
      stop = true;
    }
  }

  // Check WiFi serial.
  serviceWiFiClient();

  while (wifiClient &&
         wifiClient.connected() &&
         wifiClient.available()) {

    char command = wifiClient.read();

    if (command == 'd' ||
        command == 'D' ||
        command == 'x' ||
        command == 'X') {
      stop = true;
    }
  }

  return stop;
}

// ============================================================
// MPU
// ============================================================

void serviceMpuFor(unsigned long durationMs) {
  unsigned long started = millis();

  do {
    mpuYaw.service();
    serviceWiFiClient();
    delay(1);
  }
  while (millis() - started < durationMs);
}

// ============================================================
// Turn
// ============================================================

bool runTurn(bool turnRight) {
  if (!mpuReady) {
    logPrintln("REFUSED: MPU initialization failed.");
    return false;
  }

  drive(0, 0);

  mpuYaw.reset();

  serviceMpuFor(50);

  if (!mpuYaw.healthy()) {
    logPrintln("REFUSED: MPU is unavailable or stale.");
    return false;
  }

  unsigned long startLeft;
  unsigned long startRight;

  readTicks(startLeft, startRight);

  unsigned long started = millis();
  unsigned long lastLeftChange = started;
  unsigned long lastRightChange = started;

  unsigned long previousLeft = 0;
  unsigned long previousRight = 0;

  unsigned long lastLog = started;

  const float target =
      turnRight
          ? TURN_TARGET_DEGREES
          : -TURN_TARGET_DEGREES;

  const char *result = "TIMEOUT";
  bool turnCompleted = false;
  float yawAtBrake = 0;

  while (true) {
    serviceWiFiClient();
    mpuYaw.service();

    if (abortRequested()) {
      result = "ABORTED";
      break;
    }

    if (!mpuYaw.healthy()) {
      result = "MPU UNAVAILABLE OR STALE";
      break;
    }

    unsigned long rawLeft;
    unsigned long rawRight;

    readTicks(rawLeft, rawRight);

    unsigned long left = rawLeft - startLeft;
    unsigned long right = rawRight - startRight;

    unsigned long now = millis();

    if (left != previousLeft) {
      previousLeft = left;
      lastLeftChange = now;
    }

    if (right != previousRight) {
      previousRight = right;
      lastRightChange = now;
    }

    if (left >= TURN_ENCODER_SAFETY_TICKS ||
        right >= TURN_ENCODER_SAFETY_TICKS) {

      result = "ENCODER SAFETY LIMIT";
      break;
    }

    if (now - started >= TURN_TIMEOUT_MS) {
      break;
    }

    const float yaw =
        mpuYaw.yaw() * MPU_YAW_SIGN;

    const float rate =
        mpuYaw.rate() * MPU_YAW_SIGN;

    if (now - lastLeftChange >= STALL_TIMEOUT_MS ||
        now - lastRightChange >= STALL_TIMEOUT_MS) {
      result = "ENCODER STALL";
      break;
    }

    const float error = turnTargetError(yaw,target);
    float projectedStop = yaw;
    const bool complete = turnTargetReached(
        turnRight,yaw,rate,target,TURN_TOLERANCE_DEGREES,
        TURN_BASE_BRAKE_LEAD_DEGREES,TURN_BRAKE_LOOKAHEAD_SECONDS,
        projectedStop);

    if (complete) {
      result = "PREDICTED 90-DEGREE STOP REACHED";
      turnCompleted = true;
      yawAtBrake = yaw;
      break;
    }

    // Both wheels receive the same fixed raw PWM magnitude until yaw completes.
    DifferentialTurnCommands commands =
        differentialTurnCommands(turnRight, TURN_PWM);
    drive(commands.left, commands.right);

    if (now - lastLog >= 100) {
      String line;

      line.reserve(160);

      line += "turn: ";
      line += turnRight ? "RIGHT" : "LEFT";

      line += " | yaw/target/error: ";
      line += String(yaw, 2);
      line += "/";
      line += String(target, 1);
      line += "/";
      line += String(error, 2);

      line += " | rate dps: ";
      line += String(rate, 2);

      line += " | projected stop: ";
      line += String(projectedStop, 2);

      line += " | PWM: ";
      line += String(commands.left);
      line += "/";
      line += String(commands.right);

      line += " | ticks L/R: ";
      line += String(left);
      line += "/";
      line += String(right);

      logPrintln(line);

      lastLog = now;
    }

    delay(2);
  }

  // ==========================================================
  // Brake / result
  // ==========================================================

  drive(0, 0);

  serviceMpuFor(150);

  unsigned long endLeft;
  unsigned long endRight;

  readTicks(endLeft, endRight);

  const float finalYaw =
      mpuYaw.yaw() * MPU_YAW_SIGN;

  String line;

  line.reserve(200);

  line += result;

  line += " | requested: ";
  line += String(target, 1);

  line += " deg | yaw at brake/final: ";
  line += String(yawAtBrake, 2);
  line += "/";
  line += String(finalYaw, 2);

  line += " | final error: ";
  line += String(target - finalYaw, 2);

  line += " | ticks L/R: ";
  line += String(endLeft - startLeft);
  line += "/";
  line += String(endRight - startRight);

  line += " | elapsed ms: ";
  line += String(millis() - started);

  logPrintln(line);

  logPrintln(
      "Measure the physical angle and center displacement; "
      "yaw alone is not a pass.");
  return turnCompleted;
}

// ============================================================
// Setup
// ============================================================

void setup() {
  Serial.begin(115200);

  pinMode(MOTOR_EN, OUTPUT);
  digitalWrite(MOTOR_EN, LOW);

  pinMode(LEFT_IN1, OUTPUT);
  pinMode(LEFT_IN2, OUTPUT);

  pinMode(RIGHT_IN1, OUTPUT);
  pinMode(RIGHT_IN2, OUTPUT);

  drive(0, 0);

  pinMode(LEFT_ENCODER, INPUT_PULLUP);
  pinMode(RIGHT_ENCODER, INPUT);

  attachInterrupt(
      digitalPinToInterrupt(LEFT_ENCODER),
      countLeft,
      RISING);

  attachInterrupt(
      digitalPinToInterrupt(RIGHT_ENCODER),
      countRight,
      RISING);

  // ----------------------------------------------------------
  // WiFi
  // ----------------------------------------------------------

  setupWiFi();

  // ----------------------------------------------------------
  // MPU
  // ----------------------------------------------------------

  Wire.begin();

  logPrintln(
      "Calibrating MPU. Keep the robot completely still.");

  mpuReady = mpuYaw.begin();

  digitalWrite(MOTOR_EN, HIGH);

  logPrintln(
      mpuReady
          ? "MPU initialization OK."
          : "MPU initialization FAILED.");

  logPrintln("l: simultaneous-wheel left 90, r: simultaneous-wheel right 90.");
  logPrintln("Both wheels counter-rotate until the MPU reaches 90 degrees.");
  logPrintln("d or x: stop. No movement starts automatically.");

  if (WiFi.status() == WL_CONNECTED) {
    logPrint("Connect using: ");
    logPrint(WiFi.localIP().toString());
    logPrint(":");
    logPrintln(String(WIFI_SERIAL_PORT));
  }
}

// ============================================================
// Main loop
// ============================================================

void loop() {
  serviceWiFiClient();
  mpuYaw.service();

  int input = readCommand();

  if (input < 0) {
    return;
  }

  char command = (char)input;

  if (command == 'l' || command == 'L') {
    runTurn(false);
  }
  else if (command == 'r' || command == 'R') {
    runTurn(true);
  }
  else if (command == 'd' ||
           command == 'D' ||
           command == 'x' ||
           command == 'X') {

    drive(0, 0);
    logPrintln("STOP.");
  }
}
