#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>
#include <VL6180X.h>
#include <VL53L1X.h>
#include "MoveForward.h"
#include "DemoMotion.h"
#include "config.h"
#include "WebDashboard.h"

template<typename T> void debugPrintln(const T& value);
void drive(int left, int right);

class MpuYaw {
 public:
  bool begin() {
    ready_ = false;
    failurePhase_ = "WHO_AM_I";
    failureOperation_ = "none";
    failureDetail_ = 0;
    failureAttempts_ = 0;
    failureElapsedMs_ = 0;
    uint8_t identity;
    if (!readBytes(0x75, &identity, 1)) return false;
    if (identity != 0x68) {
      failureOperation_ = "unexpected identity";
      failureDetail_ = identity;
      return false;
    }
    failurePhase_ = "wake";
    if (!writeByte(0x6B, 0x01)) return false;
    delay(100);
    failurePhase_ = "filter setup";
    if (!writeByte(0x1A, 0x06)) return false;
    failurePhase_ = "gyro range setup";
    if (!writeByte(0x1B, 0x08)) return false;
    delay(100);

    failurePhase_ = "gyro bias calibration";
    float sum = 0.0f;
    for (int i = 0; i < 200; ++i) {
      float rate;
      if (!readGyroZ(rate)) return false;
      sum += rate;
      delay(10);
    }
    gyroZBias_ = sum / 200.0f;
    ready_ = isfinite(gyroZBias_);
    if (!ready_) failureOperation_ = "invalid gyro bias";
    lastReadOk_ = ready_;
    reset();
    return ready_;
  }

  void reset() {
    yaw_ = 0.0f;
    rate_ = 0.0f;
    lastPollMs_ = millis();
    lastGoodMs_ = lastPollMs_;
  }

  void service() {
    if (!ready_ || !lastReadOk_) return;
    unsigned long now = millis();
    unsigned long elapsed = now - lastPollMs_;
    if (elapsed < 10) return;

    float gyroZ;
    failurePhase_ = "gyro sample";
    if (!readGyroZ(gyroZ)) {
      lastPollMs_ = millis();
      if (lastReadOk_) logFault("read lost");
      lastReadOk_ = false;
      return;
    }
    const unsigned long sampleMs = millis();
    const unsigned long sampleElapsed = sampleMs - lastPollMs_;
    lastPollMs_ = sampleMs;
    lastReadOk_ = true;

    // Keep the gyro convention used by the tested MPU yaw sketch.
    rate_ = gyroZ - gyroZBias_;
    if (fabsf(rate_) < 0.5f) rate_ = 0.0f;

    // Do not integrate one sample across a long blocking sensor interval.
    if (sampleElapsed <= 100) yaw_ += rate_ * (sampleElapsed / 1000.0f);
    lastGoodMs_ = sampleMs;
  }

  bool healthy() const {
    return ready_ && lastReadOk_ &&
           (unsigned long)(millis() - lastGoodMs_) <= 250;
  }

  float yaw() const { return yaw_; }
  float rate() const { return rate_; }
  void logFault(const char *context) const {
    char line[230];
    snprintf(line, sizeof(line),
             "MPU FAULT | %s | sensor=forward MPU6050 0x68 | phase=%s | operation=%s | reg=0x%02X | detail=%d | attempts=%d | elapsed=%lums",
             context, failurePhase_, failureOperation_, failureRegister_,
             failureDetail_, failureAttempts_, failureElapsedMs_);
    debugPrintln(line);
  }
  unsigned long lastGoodAgeMs() const { return millis() - lastGoodMs_; }
  bool initialized() const { return ready_; }
  bool lastReadOk() const { return lastReadOk_; }

 private:
  bool readBytes(uint8_t reg, uint8_t *data, uint8_t count) {
    failureRegister_ = reg;
    const unsigned long started = millis();
    for (int attempt = 1; attempt <= MPU_I2C_MAX_ATTEMPTS; ++attempt) {
      Wire.beginTransmission(0x68);
      Wire.write(reg);
      const uint8_t status = Wire.endTransmission(false);
      if (status == 0) {
        const uint8_t received = Wire.requestFrom((uint8_t)0x68, count);
        if (received == count) {
          for (uint8_t i = 0; i < count; ++i) data[i] = Wire.read();
          if (attempt > 1) {
            ++recoveredReads_;
            if (attempt > worstRecoveryAttempt_) worstRecoveryAttempt_ = attempt;
            const unsigned long now = millis();
            if (lastRecoveryLogMs_ == 0 || now - lastRecoveryLogMs_ >= 1000) {
              char line[220];
              snprintf(line, sizeof(line),
                       "MPU RECOVERED | forward reg=0x%02X | reads=%lu | worst=%d/%d | latest=%lums | select errors=%lu last I2C=%u | short reads=%lu last bytes=%u/%u",
                       reg, recoveredReads_, worstRecoveryAttempt_,
                       MPU_I2C_MAX_ATTEMPTS, now - started,
                       selectErrors_, lastSelectStatus_, shortReads_,
                       lastReceivedCount_, count);
              debugPrintln(line);
              recoveredReads_ = 0;
              worstRecoveryAttempt_ = 0;
              selectErrors_ = 0;
              shortReads_ = 0;
              lastRecoveryLogMs_ = now;
            }
          }
          return true;
        }
        failureOperation_ = "read byte count";
        failureDetail_ = received;
        ++shortReads_;
        lastReceivedCount_ = received;
      } else {
        failureOperation_ = "register select I2C";
        failureDetail_ = status;
        ++selectErrors_;
        lastSelectStatus_ = status;
      }
      // Brief retries are common on the shared I2C bus. Do not interrupt
      // the motors unless the gyro remains unavailable for too long.
      if (ready_ && millis() - started >= MPU_RETRY_MOTOR_HOLD_MS) drive(0, 0);
      failureAttempts_ = attempt;
      failureElapsedMs_ = millis() - started;
      if (attempt == MPU_I2C_MAX_ATTEMPTS ||
          millis() - started >= MPU_I2C_RETRY_BUDGET_MS) break;
      delay(5);
    }
    return false;
  }

  bool writeByte(uint8_t reg, uint8_t value) {
    failureRegister_ = reg;
    const unsigned long started = millis();
    for (int attempt = 1; attempt <= MPU_I2C_MAX_ATTEMPTS; ++attempt) {
      Wire.beginTransmission(0x68);
      Wire.write(reg);
      Wire.write(value);
      const uint8_t status = Wire.endTransmission();
      if (status == 0) return true;
      failureOperation_ = "register write I2C";
      failureDetail_ = status;
      failureAttempts_ = attempt;
      failureElapsedMs_ = millis() - started;
      if (attempt == MPU_I2C_MAX_ATTEMPTS ||
          millis() - started >= MPU_I2C_RETRY_BUDGET_MS) break;
      delay(5);
    }
    return false;
  }

  bool readGyroZ(float &rate) {
    uint8_t data[2];
    if (!readBytes(0x47, data, 2)) return false;
    int16_t raw = (int16_t)(((uint16_t)data[0] << 8) | data[1]);
    rate = raw / 65.5f;
    return true;
  }

  float gyroZBias_ = 0.0f;
  float yaw_ = 0.0f;
  float rate_ = 0.0f;
  bool ready_ = false;
  bool lastReadOk_ = false;
  const char *failurePhase_ = "not initialized";
  const char *failureOperation_ = "none";
  uint8_t failureRegister_ = 0;
  int failureDetail_ = 0;
  int failureAttempts_ = 0;
  unsigned long failureElapsedMs_ = 0;
  unsigned long recoveredReads_ = 0;
  unsigned long selectErrors_ = 0;
  unsigned long shortReads_ = 0;
  uint8_t lastSelectStatus_ = 0;
  uint8_t lastReceivedCount_ = 0;
  unsigned long lastRecoveryLogMs_ = 0;
  int worstRecoveryAttempt_ = 0;
  unsigned long lastPollMs_ = 0;
  unsigned long lastGoodMs_ = 0;
};

// ============================================================
// WiFi configuration
// ============================================================

const char* WIFI_SSID = "عبدالله";
const char* WIFI_PASSWORD = "1234567899";

constexpr uint16_t WIFI_SERIAL_PORT = 23;

WiFiServer wifiSerialServer(WIFI_SERIAL_PORT);
WiFiClient wifiSerialClient;

// ============================================================
// Hardware configuration
// ============================================================

// Confirmed by isolated motor test: 27/14 drives physical LEFT, 25/26 RIGHT.
constexpr int LEFT_IN1 = 27, LEFT_IN2 = 14;
constexpr int RIGHT_IN1 = 25, RIGHT_IN2 = 26, MOTOR_EN = 23;

constexpr int LEFT_ENCODER = 33, RIGHT_ENCODER = 35;

constexpr int LEFT_XSHUT = 4, RIGHT_XSHUT = 5, FRONT_XSHUT = 16;

// Five measured one-cell trials (L/R): 597/600, 528/594, 632/638,
// 614/597, 589/591. The median resists the anomalous 528 left count.
constexpr float LEFT_TICKS_PER_CELL = 623.0f;
constexpr float RIGHT_TICKS_PER_CELL = 623.0f;

constexpr int CELL_LENGTH_MM = 180;
constexpr int FORWARD_CELLS = 1;
constexpr int FRONT_EMERGENCY_STOP_MM = FRONT_WALL_STOP_TRIGGER_MM;
static_assert(FRONT_WALL_STOP_TRIGGER_MM >= FRONT_WALL_TARGET_MM,
              "Front stop trigger must approach the target from above");

constexpr unsigned long LEFT_TARGET_TICKS =
    (unsigned long)(LEFT_TICKS_PER_CELL * FORWARD_CELLS + 0.5f);

constexpr unsigned long RIGHT_TARGET_TICKS =
    (unsigned long)(RIGHT_TICKS_PER_CELL * FORWARD_CELLS + 0.5f);

static_assert(
    LEFT_TARGET_TICKS == 623 &&
    RIGHT_TARGET_TICKS == 623,
    "Check one-cell targets against the measured trials"
);

constexpr int FORWARD_TARGET_MM =
    CELL_LENGTH_MM * FORWARD_CELLS;

constexpr int FORWARD_SPEED = 140;         //////////// speeeed

constexpr unsigned long MOVE_TIMEOUT_MS =
    105000UL; // Whole seven-cell run; stall timeout remains 1500 ms.
    
constexpr unsigned long STALL_TIMEOUT_MS = 1500;

// Keep each factor on its original electrical output while fixing side labels.
constexpr float RIGHT_FACTOR = 1.0f;
constexpr float LEFT_FACTOR = 0.99f;

// Working main.ino behavior: asymmetric thresholds and slow the same named motor.
constexpr int WALL_SLOW_SPEED = 85; // Keep wall steering above the observed stall range.
constexpr int LEFT_WALL_THRESHOLD_MM = 40;
constexpr int RIGHT_TOF_INSET_MM = 10;
constexpr int RIGHT_WALL_THRESHOLD_MM = 40; // Chassis clearance: formerly 50 mm raw.
// A healthy VL6180X can report no valid target when the nearest wall is outside
// its useful range. Keep that distinct from an I2C/timeout fault and classify it
// as open space, safely above the 140 mm wall-retention threshold.
constexpr int SIDE_NO_TARGET_MM = 200;
constexpr int FRONT_NO_TARGET_MM = 200;
// Case 1: center between two side walls using right-clearance minus left-clearance.
constexpr float TWO_WALL_KP = 7.3f;
constexpr float TWO_WALL_KI = 0.0f;
constexpr float TWO_WALL_KD = 5.7f;
constexpr float TWO_WALL_MAX_PWM = 40.0f;
constexpr float TWO_WALL_TOLERANCE_MM = 3.6f;
constexpr TwoWallPidSettings TWO_WALL_PID_SETTINGS = {
    TWO_WALL_TOLERANCE_MM, WALL_SLOW_SPEED,
    TWO_WALL_KP, TWO_WALL_KI, TWO_WALL_KD, TWO_WALL_MAX_PWM
};
TwoWallPidController twoWallPid;
// Distance PID changes forward speed only; preserve the working steering direction.
constexpr float APPROACH_SLOWDOWN_TICKS = 300.0f;
constexpr int MIN_APPROACH_PWM = 100; // Maintain rolling torque until encoder braking.
constexpr CellApproachSettings APPROACH_SETTINGS = {
    0.24f, 0.01f, 0.01f, // Gentler approach response; tune against measured travel.
    APPROACH_SLOWDOWN_TICKS, // Begin approach about 87 mm before braking.
    FORWARD_SPEED, MIN_APPROACH_PWM
};
constexpr unsigned long BRAKE_LEAD_TICKS = 10; // Brake about 2.9 mm before the full-run endpoint.
static_assert(BRAKE_LEAD_TICKS < LEFT_TARGET_TICKS && BRAKE_LEAD_TICKS < RIGHT_TARGET_TICKS,
              "Brake lead must be smaller than the run target");
CellApproachController approachPid(APPROACH_SETTINGS);
// This installation reports negative raw yaw for the direction that must be
// corrected by slowing the left wheel. Reverse it so yaw and encoder steering
// request the same correction.
constexpr float MPU_YAW_SIGN = -1.0f;
constexpr float SINGLE_WALL_KP = 6.0f;
constexpr float SINGLE_WALL_KI = 0.0f;
constexpr float SINGLE_WALL_KD = 2.5f;
constexpr float SINGLE_WALL_MAX_PWM = 40.0f;
constexpr SingleWallSettings SINGLE_WALL_SETTINGS = {
    (float)LEFT_WALL_THRESHOLD_MM, (float)RIGHT_WALL_THRESHOLD_MM, 3.6f,
    WALL_SLOW_SPEED, // Minimum commanded PWM during correction.
    2.0f, 0.15f, 15.0f, // Stronger MPU trim inside the distance band.
    SINGLE_WALL_KP, SINGLE_WALL_KI, SINGLE_WALL_KD, SINGLE_WALL_MAX_PWM
};
SingleWallController singleWallPid;
// Case 3 uses the measured one-cell calibration to compare travelled distance.
constexpr float NO_WALL_ENCODER_KP = 1.7f;
constexpr float NO_WALL_ENCODER_KI = 0.10f;
constexpr float NO_WALL_ENCODER_KD = 0.0f;
constexpr float NO_WALL_ENCODER_MAX_PWM = 30.0f;
constexpr float NO_WALL_MPU_HEADING_KP = 10.0f;
constexpr float NO_WALL_MPU_RATE_KD = 0.5f;
constexpr float NO_WALL_MPU_MAX_PWM = 25.0f;
constexpr float NO_WALL_MPU_WEIGHT = 0.80f;
// Use the same rolling floors in every wall mode; 70/60 still stalled in the demo.
constexpr int NO_WALL_MIN_BASE_PWM = 120;
constexpr int NO_WALL_MIN_MOTOR_PWM = 100;
constexpr NoWallSettings NO_WALL_SETTINGS = {
    LEFT_TICKS_PER_CELL, RIGHT_TICKS_PER_CELL,
    NO_WALL_MIN_MOTOR_PWM,
    {NO_WALL_ENCODER_KP, NO_WALL_ENCODER_KI,
     NO_WALL_ENCODER_KD, NO_WALL_ENCODER_MAX_PWM}
};
NoWallController noWallPid;
MpuYaw mpuYaw;
WallModeDetector wallDetector;
bool motionActive=false;
bool motionStopRequested=false;
bool sideCommunicationFault=false;
const char *tofFailure = "none";
int tofFailureCode = 0;

// ============================================================
// Sensors / encoders
// ============================================================

volatile unsigned long leftTicks = 0;
volatile unsigned long rightTicks = 0;

VL6180X leftTof;
VL6180X rightTof;
VL53L1X frontTof;
constexpr int SIDE_CONVERGENCE_MS = 30;
constexpr unsigned long FRONT_TIMING_BUDGET_US = 20000;
constexpr int FRONT_PERIOD_MS = 25;
TofFilter frontFilter, leftFilter, rightFilter;
int tofFrontRaw = -1, tofLeftRaw = -1, tofFrontFiltered = -1;

bool sensorsReady = false;
MovementCommands usbCommands;
MovementCommands wifiCommands;
String dashboardLogLine;


// ============================================================
// WiFi
// ============================================================

void initWiFi() {
    WiFi.mode(WIFI_STA);
    WiFi.setTxPower(WIFI_POWER_8_5dBm);

    Serial.println();
    Serial.print("Connecting to WiFi: ");
    Serial.println(WIFI_SSID);

    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    unsigned long started = millis();

    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
        if (millis() - started > 15000) {
            Serial.println();
            Serial.println("WiFi connection FAILED.");
            Serial.println("Robot will continue with USB Serial only.");
            return;
        }
    }

    Serial.println();
    Serial.println("WiFi connected successfully.");
    Serial.print("ESP32 IP address: ");
    Serial.println(WiFi.localIP());
    Serial.print("WiFi Serial port: ");
    Serial.println(WIFI_SERIAL_PORT);

    wifiSerialServer.begin();
    wifiSerialServer.setNoDelay(true);
    Serial.println("TCP serial server started.");
}

void handleWiFiClient() {
    dashboardLoop();
    if (WiFi.status() != WL_CONNECTED) {
        wifiCommands.reset();
        return;
    }

    if (wifiSerialClient && wifiSerialClient.connected()) {
        return;
    }

    if (wifiSerialClient) {
        wifiSerialClient.stop();
    }
    wifiCommands.reset();

    WiFiClient newClient = wifiSerialServer.accept();
    if (newClient) {
        wifiSerialClient = newClient;
        wifiSerialClient.setNoDelay(true);
        Serial.println();
        Serial.println("WiFi serial client connected.");
        wifiSerialClient.println("=================================");
        wifiSerialClient.println("ESP32 Micromouse connected");
        wifiSerialClient.print("ESP32 IP: ");
        wifiSerialClient.println(WiFi.localIP());
        wifiSerialClient.println("Flood-fill explorer: s/start runs; d stops. Web dashboard is on port 80.");
        wifiSerialClient.println("=================================");
    }
}


// ============================================================
// Debug output
// ============================================================

// String / char / integer / etc.
template<typename T>
void debugPrint(const T& value) {

    Serial.print(value);

    if (wifiSerialClient && wifiSerialClient.connected()) {
        wifiSerialClient.print(value);
    }
    dashboardLogLine += String(value);
}


template<typename T>
void debugPrintln(const T& value) {

    Serial.println(value);

    if (wifiSerialClient && wifiSerialClient.connected()) {
        wifiSerialClient.println(value);
    }
    dashboardLogLine += String(value);
    dashboardAppendLog(dashboardLogLine.c_str());
    dashboardLogLine = "";
}


void debugPrintln() {

    Serial.println();

    if (wifiSerialClient && wifiSerialClient.connected()) {
        wifiSerialClient.println();
    }
    dashboardAppendLog(dashboardLogLine.c_str());
    dashboardLogLine = "";
}


// Float with decimal places
void debugPrint(float value, int digits) {

    Serial.print(value, digits);

    if (wifiSerialClient && wifiSerialClient.connected()) {
        wifiSerialClient.print(value, digits);
    }
    dashboardLogLine += String(value, digits);
}


// ============================================================
// Command input
// ============================================================

MovementCommand readCommand() {
    MovementCommand result = MovementCommand::None;
    // Bound each batch so neither input can monopolize the control loop.
    for (int i = 0; i < 64 && Serial.available(); ++i) {
        result = mergeCommands(result, usbCommands.feed(Serial.read(), millis()));
    }
    result = mergeCommands(result, usbCommands.poll(millis()));
    const bool webStop = dashboardTakeStop();
    const bool webStart = dashboardTakeStart();
    if (webStop) result = MovementCommand::Stop;
    else if (webStart) result = mergeCommands(result, MovementCommand::Start);
    if (wifiSerialClient && wifiSerialClient.connected()) {
        for (int i = 0; i < 64 && wifiSerialClient.available(); ++i) {
            result = mergeCommands(result, wifiCommands.feed(wifiSerialClient.read(), millis()));
        }
        result = mergeCommands(result, wifiCommands.poll(millis()));
    }
    return result; // A stop wins over a start received in the same batch.
}

void discardPendingCommands() {
    // Never replay a start received during a run or its final reporting.
    int usbPending = Serial.available();
    while (usbPending-- > 0) Serial.read();
    int wifiPending = wifiSerialClient ? wifiSerialClient.available() : 0;
    while (wifiPending-- > 0) wifiSerialClient.read();
    usbCommands.reset();
    wifiCommands.reset();
    dashboardTakeStop();
    dashboardTakeStart();
}


// ============================================================
// Encoder interrupts
// ============================================================

void IRAM_ATTR countLeft() {
    ++leftTicks;
}

void IRAM_ATTR countRight() {
    ++rightTicks;
}


void readTicks(
    unsigned long &left,
    unsigned long &right
) {

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
    bool reversePolarity
) {

    int pwm =
        constrain(
            (int)(abs(speed) * factor),
            0,
            255
        );

    if (speed == 0) {

        analogWrite(a, 255);
        analogWrite(b, 255);

        return;
    }

    bool positive = speed > 0;

    if (reversePolarity) {
        positive = !positive;
    }

    analogWrite(
        a,
        positive ? pwm : 0
    );

    analogWrite(
        b,
        positive ? 0 : pwm
    );
}


void drive(int left, int right) {

    motor(
        LEFT_IN1,
        LEFT_IN2,
        left,
        LEFT_FACTOR,
        true
    );

    motor(
        RIGHT_IN1,
        RIGHT_IN2,
        right,
        RIGHT_FACTOR,
        false
    );
}


// ============================================================
// Sensors
// ============================================================

uint8_t probeTof(uint8_t address) {
    Wire.beginTransmission(address);
    return Wire.endTransmission();
}

bool reportTofStep(const char *name, const char *step, uint8_t status) {
    if (status != 0) {
        debugPrint("TOF ERROR | "); debugPrint(name); debugPrint(" | ");
        debugPrint(step); debugPrint(" | I2C="); debugPrintln(status);
    }
    return status == 0;
}
bool initSensors() {
  Wire.begin();
  bool leftReady = false, rightReady = false, frontReady = false;
  // Match main.ino's sequence and settings; report diagnostics afterward.
  pinMode(LEFT_XSHUT, OUTPUT);
  pinMode(RIGHT_XSHUT, OUTPUT);
  pinMode(FRONT_XSHUT, OUTPUT);
  digitalWrite(LEFT_XSHUT, LOW);
  digitalWrite(RIGHT_XSHUT, LOW);
  digitalWrite(FRONT_XSHUT, LOW);
  delay(10);

  digitalWrite(LEFT_XSHUT, HIGH);
  delay(50);
  leftTof.init(); // VL6180X::init() returns void.
  uint8_t leftInitStatus = leftTof.last_status;
  leftTof.configureDefault();
  uint8_t leftConfigStatus = leftTof.last_status;
  leftTof.setAddress(0x30);
  uint8_t leftAddressStatus = leftTof.last_status;
  leftTof.setTimeout(200);
  leftTof.writeReg(VL6180X::SYSRANGE__MAX_CONVERGENCE_TIME, SIDE_CONVERGENCE_MS);
  bool leftTimingOk = leftTof.last_status == 0;

  digitalWrite(RIGHT_XSHUT, HIGH);
  delay(50);
  rightTof.init();
  uint8_t rightInitStatus = rightTof.last_status;
  rightTof.configureDefault();
  uint8_t rightConfigStatus = rightTof.last_status;
  rightTof.setAddress(0x31);
  uint8_t rightAddressStatus = rightTof.last_status;
  rightTof.setTimeout(200);
  rightTof.writeReg(VL6180X::SYSRANGE__MAX_CONVERGENCE_TIME, SIDE_CONVERGENCE_MS);
  bool rightTimingOk = rightTof.last_status == 0;

  digitalWrite(FRONT_XSHUT, HIGH);
  delay(50);
  bool frontInitOk = frontTof.init();
  bool frontModeOk = frontTof.setDistanceMode(VL53L1X::Short);
  bool frontTimingOk = frontTof.setMeasurementTimingBudget(FRONT_TIMING_BUDGET_US);
  frontTof.setAddress(0x32);
  uint8_t frontAddressStatus = frontTof.last_status;
  frontTof.setTimeout(200);
  frontTof.startContinuous(FRONT_PERIOD_MS);
  uint8_t frontStartStatus = frontTof.last_status;

  reportTofStep("LEFT", "init", leftInitStatus);
  reportTofStep("LEFT", "configureDefault", leftConfigStatus);
  reportTofStep("LEFT", "setAddress", leftAddressStatus);
  if (!leftTimingOk) debugPrintln("TOF ERROR | LEFT timing configuration failed");
  leftReady = reportTofStep("LEFT", "probe 0x30", probeTof(0x30));
  reportTofStep("RIGHT", "init", rightInitStatus);
  reportTofStep("RIGHT", "configureDefault", rightConfigStatus);
  reportTofStep("RIGHT", "setAddress", rightAddressStatus);
  if (!rightTimingOk) debugPrintln("TOF ERROR | RIGHT timing configuration failed");
  rightReady = reportTofStep("RIGHT", "probe 0x31", probeTof(0x31));
  if (!frontInitOk) debugPrintln("TOF ERROR | front initialization failed");
  if (!frontModeOk) debugPrintln("TOF ERROR | front distance mode failed");
  if (!frontTimingOk) debugPrintln("TOF ERROR | FRONT timing configuration failed");
  reportTofStep("FRONT", "setAddress", frontAddressStatus);
  reportTofStep("FRONT", "startContinuous", frontStartStatus);
  frontReady = reportTofStep("FRONT", "probe 0x32", probeTof(0x32)) && frontInitOk;

  debugPrintln(leftTimingOk && rightTimingOk && frontTimingOk ?
      "ToF fast timing configured." : "ToF timing configuration FAILED.");
  return leftReady && rightReady && frontReady && frontModeOk &&
      leftTimingOk && rightTimingOk && frontTimingOk;
}

void serviceMotionSensors() {
    mpuYaw.service();
    if (motionActive && readCommand()==MovementCommand::Stop) {
        motionStopRequested=true;
        drive(0,0);
    }
}

void waitWithMotionService(unsigned long durationMs) {
    unsigned long began=millis();
    do { serviceMotionSensors(); delay(1); } while (millis()-began<durationMs);
}

int readSideForMotion(VL6180X &sensor) {
    // Both shots were launched together by observe().
    unsigned long began=millis();
    while (true) {
        serviceMotionSensors();
        if (motionStopRequested) return -1;
        uint8_t status=sensor.readReg(VL6180X::RESULT__INTERRUPT_STATUS_GPIO);
        if (sensor.last_status!=0) {
            sideCommunicationFault=true;
            tofFailure = &sensor == &leftTof ? "LEFT I2C" : "RIGHT I2C";
            tofFailureCode = sensor.last_status;
            return -1;
        }
        if ((status&7)==4) break;
        if (millis()-began>=200) {
            sideCommunicationFault=true;
            tofFailure = &sensor == &leftTof ? "LEFT ready timeout" : "RIGHT ready timeout";
            return -1;
        }
        // A ToF conversion takes much longer than 1 ms. Leave bus time for
        // the MPU instead of polling this sensor every millisecond.
        delay(5);
    }
    int mm=sensor.readRangeContinuousMillimeters(); // Already ready; no long wait.
    bool valid=!sensor.timeoutOccurred() && sensor.last_status==0;
    uint8_t rangeStatus=sensor.readRangeStatus();
    if (!valid || sensor.last_status!=0) {
        sideCommunicationFault=true;
        tofFailure = &sensor == &leftTof ? "LEFT range read failed" : "RIGHT range read failed";
        tofFailureCode = sensor.last_status;
        return -1;
    }
    return rangeStatus==0 ? mm : SIDE_NO_TARGET_MM;
}

bool observe(int &front,int &left,int &right,int &rightRaw,
             bool stoppedScan = false, bool frontOnly = false) {
    sideCommunicationFault=false;
    tofFailure = "none";
    tofFailureCode = 0;
    front=left=right=rightRaw=-1;
    tofFrontRaw=tofLeftRaw=tofFrontFiltered=-1;
    if (!frontOnly) {
        // Side ranges are unnecessary during a short front-wall adjustment.
        leftTof.writeReg(VL6180X::SYSTEM__INTERRUPT_CLEAR,0x01);
        if (leftTof.last_status!=0) {
            sideCommunicationFault=true;
            tofFailure="LEFT clear I2C"; tofFailureCode=leftTof.last_status;
        }
        rightTof.writeReg(VL6180X::SYSTEM__INTERRUPT_CLEAR,0x01);
        if (rightTof.last_status!=0) {
            sideCommunicationFault=true;
            tofFailure="RIGHT clear I2C"; tofFailureCode=rightTof.last_status;
        }
        leftTof.writeReg(VL6180X::SYSRANGE__START,0x01);
        if (leftTof.last_status!=0) {
            sideCommunicationFault=true;
            tofFailure="LEFT trigger I2C"; tofFailureCode=leftTof.last_status;
        }
        rightTof.writeReg(VL6180X::SYSRANGE__START,0x01);
        if (rightTof.last_status!=0) {
            sideCommunicationFault=true;
            tofFailure="RIGHT trigger I2C"; tofFailureCode=rightTof.last_status;
        }
    }
    unsigned long began=millis();
    while (true) {
        serviceMotionSensors();
        if (motionStopRequested) return false;
        bool ready=frontTof.dataReady();
        if (frontTof.last_status!=0) {
            tofFailure="FRONT ready I2C"; tofFailureCode=frontTof.last_status;
            return false;
        }
        if (ready) break;
        if (millis()-began>=200) {
            tofFailure="FRONT ready timeout";
            return false;
        }
        // Avoid saturating the shared I2C bus while waiting for a new range.
        delay(5);
    }
    front=frontTof.read(false);
    bool frontTimedOut=frontTof.timeoutOccurred();
    const uint8_t frontStatus=(uint8_t)frontTof.ranging_data.range_status;
    const bool frontNoTarget=frontRangeStatusMeansOpen(frontStatus);
    const bool frontRangeUsable=frontRangeStatusIsUsable(frontStatus);
    bool frontValid=!frontTimedOut && frontTof.last_status==0 && frontRangeUsable;
    // Status 2/4 means no reliable target is in range. For maze navigation
    // that means open space, not a motion fault.
    if (frontValid && frontNoTarget) front=FRONT_NO_TARGET_MM;
    if (!frontValid) {
        if (frontTimedOut) tofFailure="FRONT read timeout";
        else if (frontTof.last_status!=0) {
            tofFailure="FRONT read I2C"; tofFailureCode=frontTof.last_status;
        } else {
            tofFailure="FRONT range rejected";
            tofFailureCode=frontStatus;
        }
    }
    tofFrontRaw=front;
    tofFrontFiltered=frontFilter.update(front,frontValid);
    // Brake before side reads or logging when the front sample requires a stop.
    if (frontValid && !frontOnly && front <= FRONT_PREMATURE_BRAKE_MM)
        drive(0, 0);
    if (!frontValid || front <= FRONT_EMERGENCY_STOP_MM) {
        drive(0, 0);
        // A stationary junction scan still needs side ranges at a dead end.
        // Forward motion retains the tested immediate-return brake behavior.
        if (!frontValid || !stoppedScan) {
            left=right=rightRaw=-1;
            return frontValid;
        }
    }
    if (frontOnly) return frontValid;
    tofLeftRaw=readSideForMotion(leftTof);
    // A newly distant reading must release the wall immediately, even if the
    // median still contains two old close readings. Distances are chassis-based.
    left=leftFilter.update(tofLeftRaw,tofLeftRaw>=0 &&
        tofLeftRaw<WallModeDetector::RETAIN_MM);
    rightRaw=readSideForMotion(rightTof);
    right=sideClearanceMm(rightFilter.update(rightRaw,
        rightRaw>=0 && sideClearanceMm(rightRaw,RIGHT_TOF_INSET_MM)<WallModeDetector::RETAIN_MM), RIGHT_TOF_INSET_MM);
    serviceMotionSensors();
    return frontValid;
}

// ============================================================
// Movement
// ============================================================

void reportMovementConfig() {
    debugPrintln("MOTION BUILD | encoder-median-v13 | " __DATE__ " " __TIME__);
    char line[180];
    snprintf(line, sizeof(line),
        "MOVE CONFIG | cell=%dmm | cruise=%d | base/motor floor=%d/%d | front target=%dmm +/-%.1f%% | trigger=%dmm | yaw: +right/-left",
        FORWARD_TARGET_MM, FORWARD_SPEED, MIN_APPROACH_PWM, WALL_SLOW_SPEED,
        FRONT_WALL_TARGET_MM, FRONT_WALL_TOLERANCE_PERCENT,
        FRONT_WALL_STOP_TRIGGER_MM);
    debugPrintln(line);
    snprintf(line, sizeof(line),
        "ENCODER CAL | ticks/cell L/R=%.0f/%.0f | five one-cell trials, median",
        LEFT_TICKS_PER_CELL, RIGHT_TICKS_PER_CELL);
    debugPrintln(line);
    snprintf(line, sizeof(line),
        "NO WALL | MPU Kp/Kd=%.2f/%.2f max=%.0f weight=%.0f%% | encoder Kp/Ki=%.2f/%.2f max=%.0f",
        NO_WALL_MPU_HEADING_KP, NO_WALL_MPU_RATE_KD, NO_WALL_MPU_MAX_PWM,
        NO_WALL_MPU_WEIGHT*100.0f, NO_WALL_ENCODER_KP, NO_WALL_ENCODER_KI,
        NO_WALL_ENCODER_MAX_PWM);
    debugPrintln(line);
    snprintf(line, sizeof(line),
        "NO WALL SPEED | front>%dmm base/motor floor=%d/%d | near front base=%d",
        FRONT_MAP_OPEN_MM, NO_WALL_MIN_BASE_PWM, NO_WALL_MIN_MOTOR_PWM,
        MIN_APPROACH_PWM);
    debugPrintln(line);
    snprintf(line, sizeof(line),
        "WALL PID | single Kp/Ki/Kd=%.2f/%.2f/%.2f | two Kp/Ki/Kd=%.2f/%.2f/%.2f",
        SINGLE_WALL_KP, SINGLE_WALL_KI, SINGLE_WALL_KD,
        TWO_WALL_KP, TWO_WALL_KI, TWO_WALL_KD);
    debugPrintln(line);
}

DemoMoveResult runForwardDistance(int cells) {
    if (!sensorsReady) {
        debugPrintln("REFUSED: ToF initialization failed.");
        return DemoMoveResult::Failed;
    }
    // Zero cells is a short wall-reference adjustment after an encoder move.
    if (cells < 0 || cells > 3) return DemoMoveResult::Failed;

    const unsigned long leftTarget = LEFT_TARGET_TICKS * cells;
    const unsigned long rightTarget = RIGHT_TARGET_TICKS * cells;
    // The front ToF decides when the final cell ends. Encoders only confirm
    // that earlier cells in a continuous run have been passed.
    const float minimumFrontWallTravelMm = cells <= 1 ? 0.0f :
        (cells - 1) * CELL_LENGTH_MM;
    char startLine[100];
    if (cells == 0)
        snprintf(startLine, sizeof(startLine),
                 "WALL REFERENCE START | front target=%dmm | max extra=%.0fmm",
                 FRONT_WALL_TARGET_MM, FRONT_APPROACH_MAX_EXTRA_MM);
    else
        snprintf(startLine, sizeof(startLine), "MOVE START | cells=%d | target=%dmm%s",
                 cells, cells * CELL_LENGTH_MM, cells > 1 ? " | continuous" : "");
    debugPrintln(startLine);

    unsigned long startLeft, startRight;
    readTicks(startLeft, startRight);
    unsigned long started = millis();
    unsigned long lastLeftChange = started, lastRightChange = started;
    unsigned long previousLeft = 0, previousRight = 0, lastLog = started;
    const char *result = "TIMEOUT";
    DemoMoveResult outcome = DemoMoveResult::Failed;
    bool approachingFrontWall = cells == 0;
    unsigned long frontApproachStarted = started, approachLeft = 0, approachRight = 0;
    int lastFrontMm = -1;
    unsigned long previousControlMs = started;
    approachPid.reset();
    twoWallPid.reset();
    singleWallPid.reset();
    noWallPid.reset();

    while (true) {
        handleWiFiClient();
        if (motionStopRequested || readCommand() == MovementCommand::Stop) {
            result = "ABORTED";
            outcome = DemoMoveResult::Stopped;
            break;
        }

        unsigned long rawLeft, rawRight;
        readTicks(rawLeft, rawRight);
        unsigned long left = rawLeft - startLeft, right = rawRight - startRight;
        unsigned long now = millis();
        if (left != previousLeft) { lastLeftChange = now; previousLeft = left; }
        if (right != previousRight) { lastRightChange = now; previousRight = right; }

        // Brake at the nominal endpoint while getting a fresh front reading.
        // An open path finishes here; a nearby wall may need extra approach.
        if (!approachingFrontWall &&
            cellEncoderLimitReached(left, right, leftTarget - BRAKE_LEAD_TICKS,
                                    rightTarget - BRAKE_LEAD_TICKS)) {
            drive(0, 0);
        }
        if (now - started >= MOVE_TIMEOUT_MS) break;
        if (now - lastLeftChange >= STALL_TIMEOUT_MS ||
            now - lastRightChange >= STALL_TIMEOUT_MS) {
            result = "ENCODER STALL";
            break;
        }

        int front = -1, sideLeft = -1, sideRight = -1, sideRightRaw = -1;
        bool frontValid = observe(front, sideLeft, sideRight, sideRightRaw,
                                  false, cells == 0);
        // Stop wins over a sensor fault received during a blocking read.
        handleWiFiClient();
        if (motionStopRequested || readCommand() == MovementCommand::Stop) {
            result = "ABORTED";
            outcome = DemoMoveResult::Stopped;
            break;
        }
        if (!frontValid) {
            result = "INVALID FRONT TOF READING";
            char line[125];
            snprintf(line, sizeof(line), "MOVE SENSOR FAULT | %s | code=%d",
                     tofFailure, tofFailureCode);
            debugPrintln(line);
            break;
        }
        lastFrontMm = front;
        // ToF reads block; use encoder counts from the same instant as the
        // front-wall decision rather than counts taken before the read.
        readTicks(rawLeft, rawRight);
        left = rawLeft - startLeft;
        right = rawRight - startRight;
        const float leftTravelMm = left * CELL_LENGTH_MM / LEFT_TICKS_PER_CELL;
        const float rightTravelMm = right * CELL_LENGTH_MM / RIGHT_TICKS_PER_CELL;
        const bool cellTravelled = cellTravelMeetsMinimum(
            leftTravelMm, rightTravelMm, minimumFrontWallTravelMm);
        if (cells != 0 && front <= FRONT_PREMATURE_BRAKE_MM && !cellTravelled) {
            drive(0, 0);
            result = "EARLY FRONT WALL: CELL NOT REACHED";
            break;
        }
        if (front <= FRONT_EMERGENCY_STOP_MM) {
            if (cellTravelled) {
                result = "FRONT WALL BRAKE TRIGGERED";
                outcome = DemoMoveResult::FrontWallReached;
            } else {
                result = "FRONT WALL TOO EARLY: PLANNED CELLS NOT REACHED";
            }
            break;
        }
        if (cells != 0 && front <= FRONT_MAP_OPEN_MM &&
            fabsf(mpuYaw.yaw() * MPU_YAW_SIGN) >= FRONT_SKEW_BRAKE_DEG) {
            drive(0, 0);
            result = "FRONT WALL NEAR WITH LARGE YAW; ALIGN BEFORE APPROACH";
            if (cellTravelled) outcome = DemoMoveResult::FrontWallReached;
            break;
        }
        if (cells == 0 && front > FRONT_MAP_OPEN_MM) {
            result = "FRONT WALL LOST DURING REFERENCE";
            break;
        }
        if (sideCommunicationFault && cells != 0) {
            result="SIDE TOF COMMUNICATION FAILURE";
            drive(0, 0);
            char line[125];
            snprintf(line, sizeof(line), "MOVE SENSOR FAULT | %s | code=%d",
                     tofFailure, tofFailureCode);
            debugPrintln(line);
            break;
        }
        if (cells != 0 && !mpuYaw.healthy()) {
            result = "MPU UNAVAILABLE OR STALE";
            drive(0, 0);
            if (!mpuYaw.lastReadOk()) mpuYaw.logFault("forward movement stopped");
            else {
                char line[100];
                snprintf(line, sizeof(line),
                         "MPU FAULT | forward gyro sample stale | last good=%lums ago",
                         mpuYaw.lastGoodAgeMs());
                debugPrintln(line);
            }
            break;
        }
        WallMode mode=wallDetector.update(sideLeft,sideRight);
        if (cells != 0 && mode==WallMode::None && (sideLeft<0 || sideRight<0)) {
            result="CASE 3 REQUIRES TWO VALID SIDE READINGS"; break;
        }
        const bool oneWallMode=mode==WallMode::LeftOnly || mode==WallMode::RightOnly;
        if (cells != 0 && oneWallMode && MPU_YAW_SIGN==0) {
            result="SET MPU_YAW_SIGN BEFORE CASE 2"; break;
        }

        if (!approachingFrontWall &&
            cellEncoderLimitReached(left, right, leftTarget - BRAKE_LEAD_TICKS,
                                    rightTarget - BRAKE_LEAD_TICKS)) {
            if (front > FRONT_OPEN_MM) {
                result = "RUN ENCODER LIMIT REACHED";
                outcome = DemoMoveResult::EncoderReached;
                break;
            }
            approachingFrontWall = true;
            frontApproachStarted = millis();
            approachLeft = left; approachRight = right;
            char line[120];
            snprintf(line,sizeof(line),
                "WALL APPROACH | encoder target reached | front=%dmm | continue to %dmm",
                front,FRONT_EMERGENCY_STOP_MM);
            debugPrintln(line);
        }
        if (approachingFrontWall) {
            const float extraLeft = (left-approachLeft)*CELL_LENGTH_MM/LEFT_TICKS_PER_CELL;
            const float extraRight = (right-approachRight)*CELL_LENGTH_MM/RIGHT_TICKS_PER_CELL;
            if (millis()-frontApproachStarted >= FRONT_APPROACH_TIMEOUT_MS ||
                extraLeft >= FRONT_APPROACH_MAX_EXTRA_MM || extraRight >= FRONT_APPROACH_MAX_EXTRA_MM) {
                result = "FRONT WALL APPROACH LIMIT: TARGET NOT REACHED";
                break;
            }
        }
        unsigned long leftRemaining = ticksBeforeBrake(left, leftTarget, BRAKE_LEAD_TICKS);
        unsigned long rightRemaining = ticksBeforeBrake(right, rightTarget, BRAKE_LEAD_TICKS);
        unsigned long remaining = leftRemaining < rightRemaining ? leftRemaining : rightRemaining;
        unsigned long controlMs = millis();
        float dt = (controlMs - previousControlMs) / 1000.0f;
        previousControlMs = controlMs;
        // Keep enough torque even after encoder remaining distance becomes zero.
        int approachSpeed = approachingFrontWall ? MIN_APPROACH_PWM :
            approachPid.update((float)remaining, dt);
        ForwardMotorCommands commands;
        float wallError=0, wallPwm=0, mpuPwm=0;
        float encoderError=0, encoderPwm=0;
        const float yawRight=mpuYaw.yaw()*MPU_YAW_SIGN;
        const float rateRight=mpuYaw.rate()*MPU_YAW_SIGN;
        if (cells == 0) {
            commands = {MIN_APPROACH_PWM, MIN_APPROACH_PWM};
        } else if (mode==WallMode::Two) {
            singleWallPid.reset();
            noWallPid.reset();
            commands=twoWallPid.update(sideLeft,sideRight,approachSpeed,
                TWO_WALL_PID_SETTINGS,dt,wallError,wallPwm);
        } else if (mode==WallMode::LeftOnly || mode==WallMode::RightOnly) {
            twoWallPid.reset();
            noWallPid.reset();
            commands=singleWallPid.update(mode,sideLeft,sideRight,yawRight,rateRight,
                approachSpeed,SINGLE_WALL_SETTINGS,dt,wallError,wallPwm,mpuPwm);
        } else {
            twoWallPid.reset();
            singleWallPid.reset();
            const int baseFloor = front > FRONT_MAP_OPEN_MM
                ? NO_WALL_MIN_BASE_PWM : MIN_APPROACH_PWM;
            approachSpeed=noWallApproachSpeed(approachSpeed,baseFloor,FORWARD_SPEED);
            commands=noWallPid.update(left,right,approachSpeed,
                NO_WALL_SETTINGS,dt,encoderError,encoderPwm);
            const bool noWallMpuReady=mpuYaw.healthy();
            mpuPwm=noWallMpuReady
                ? noWallYawCorrection(yawRight,rateRight,
                    NO_WALL_MPU_HEADING_KP,NO_WALL_MPU_RATE_KD,
                    NO_WALL_MPU_MAX_PWM)
                : 0.0f;
            commands=noWallCombinedCommands(approachSpeed,
                NO_WALL_MIN_MOTOR_PWM,encoderPwm,mpuPwm,
                noWallMpuReady ? NO_WALL_MPU_WEIGHT : 0.0f);
        }
        drive(commands.left, commands.right);

        if (millis() - lastLog >= 200) {
            // One complete line reduces the number of WiFi write calls.
            char telemetry[280];
            snprintf(telemetry, sizeof(telemetry),
                "MOVE | %s | mm F/L/R=%d/%d/%d | travel L/R=%.1f/%.1fmm | yaw=%+.2fdeg | PWM L/R=%d/%d | correction wall/mpu/enc=%.1f/%.1f/%.1f",
                cells == 0 ? "front reference" :
                mode==WallMode::Two ? "2 walls" :
                    (mode==WallMode::LeftOnly ? "left wall" :
                    (mode==WallMode::RightOnly ? "right wall" : "no walls")),
                front, sideLeft, sideRight,
                left * CELL_LENGTH_MM / LEFT_TICKS_PER_CELL,
                right * CELL_LENGTH_MM / RIGHT_TICKS_PER_CELL,
                yawRight, commands.left, commands.right,
                wallPwm, mpuPwm, encoderPwm);
            debugPrintln(telemetry);
            lastLog = millis();
        }
        waitWithMotionService(5); // MPU is also serviced during sensor waits.
    }

    drive(0, 0); // Always brake both wheels together.
    approachPid.reset();
    twoWallPid.reset();
    singleWallPid.reset();
    noWallPid.reset();
    waitWithMotionService(100); // Sample yaw and residual movement after braking.
    int stoppedFront = -1, unusedLeft, unusedRight, unusedRightRaw;
    if (!observe(stoppedFront, unusedLeft, unusedRight, unusedRightRaw,
                 true, true)) stoppedFront = -1;
    unsigned long endLeft, endRight;
    readTicks(endLeft, endRight);
    char summary[240];
    snprintf(summary, sizeof(summary),
        "MOVE END | %s | encoder travel L/R=%.1f/%.1fmm | yaw=%+.2fdeg | time=%lums",
        result,
        (endLeft - startLeft) * CELL_LENGTH_MM / LEFT_TICKS_PER_CELL,
        (endRight - startRight) * CELL_LENGTH_MM / RIGHT_TICKS_PER_CELL,
        mpuYaw.yaw() * MPU_YAW_SIGN, millis() - started);
    debugPrintln(summary);
    if (outcome == DemoMoveResult::FrontWallReached) {
        const bool readingInBand = stoppedFront >= 0 &&
            fabsf(stoppedFront - FRONT_WALL_TARGET_MM) <= FRONT_WALL_TOLERANCE_MM;
        snprintf(summary,sizeof(summary),
                 "WALL REFERENCE | target=%dmm +/-%.1f%% | trigger=%dmm | front at brake/stopped=%d/%dmm | %s",
                 FRONT_WALL_TARGET_MM,FRONT_WALL_TOLERANCE_PERCENT,
                 FRONT_WALL_STOP_TRIGGER_MM,lastFrontMm,stoppedFront,
                 readingInBand ? "IN BAND" :
                     (stoppedFront < FRONT_WALL_TARGET_MM ? "BELOW TARGET" : "ABOVE TARGET"));
        debugPrintln(summary);
    }
    return motionStopRequested ? DemoMoveResult::Stopped : outcome;
}

void runMove() {
    reportMovementConfig();
    frontFilter.reset(); leftFilter.reset(); rightFilter.reset();
    motionStopRequested=false;
    mpuYaw.reset();
    wallDetector.reset();
    debugPrintln("Moving one cell (180 mm); front wall target 60 mm +/-1%.");
    DemoMoveResult outcome = runForwardDistance(1);
    if (outcome == DemoMoveResult::Stopped || outcome == DemoMoveResult::Failed) {
        debugPrintln("Run stopped; send s when ready for a new run.");
        return;
    }
    debugPrintln("One cell completed.");
}


// ============================================================
// Setup
// ============================================================

void moveForwardSetup() {

    Serial.begin(115200);

    delay(500);


    // --------------------------------------------------------
    // Motors
    // --------------------------------------------------------

    pinMode(MOTOR_EN, OUTPUT);

    digitalWrite(
        MOTOR_EN,
        LOW
    );

    pinMode(
        LEFT_IN1,
        OUTPUT
    );

    pinMode(
        LEFT_IN2,
        OUTPUT
    );

    pinMode(
        RIGHT_IN1,
        OUTPUT
    );

    pinMode(
        RIGHT_IN2,
        OUTPUT
    );

    drive(0, 0);

    // Establish stopped motor outputs before enabling WiFi commands.
    initWiFi();

    // --------------------------------------------------------
    // Encoders
    // --------------------------------------------------------

    pinMode(
        LEFT_ENCODER,
        INPUT_PULLUP
    );

    pinMode(
        RIGHT_ENCODER,
        INPUT
    );


    attachInterrupt(
        digitalPinToInterrupt(
            LEFT_ENCODER
        ),
        countLeft,
        RISING
    );


    attachInterrupt(
        digitalPinToInterrupt(
            RIGHT_ENCODER
        ),
        countRight,
        RISING
    );


    // --------------------------------------------------------
    // Sensors
    // --------------------------------------------------------

    sensorsReady =
        initSensors();
    debugPrintln("MPU6050 calibration: keep the robot completely still.");
    const bool mpuReady = mpuYaw.begin();
    if (!mpuReady) mpuYaw.logFault("initialization after I2C retries");
    debugPrintln(mpuReady?"MPU6050 ready (direct yaw, no software Kalman filter).":"MPU initialization FAILED; forward movement unavailable.");
    if (MPU_YAW_SIGN==0) debugPrintln("Case 2 needs MPU_YAW_SIGN: +1 for right-positive yaw, -1 for right-negative yaw.");

    digitalWrite(
        MOTOR_EN,
        HIGH
    );


    debugPrintln(
        sensorsReady
            ? "ToF initialization OK."
            : "ToF initialization FAILED."
    );


    if (!sensorsReady) {

        debugPrintln(
            "Movement unavailable: ToF initialization failed. Reset to retry."
        );

        return;
    }


    debugPrintln(
        "Forward module ready for one-cell moves."
    );


    discardPendingCommands();
}


// ============================================================
// Loop
// ============================================================

void moveForwardLoop() {

    // Standalone one-cell entry point; the demo uses demoMoveOneCell instead.
    handleWiFiClient();
    mpuYaw.service();
    MovementCommand command = readCommand();
    if (command == MovementCommand::Stop) {
        drive(0, 0);

        discardPendingCommands();
        debugPrintln("Stopped. Send s for a fresh one-cell run.");
    } else if (command == MovementCommand::Start) {
        debugPrintln("Start received.");
        motionActive=true;
        runMove();
        motionActive=false;
        discardPendingCommands();
        debugPrintln("Run ended. Staying stopped; send s for a new one-cell run.");
    }

    delay(10);
}

void moveForwardStop() {
    motionStopRequested = true;
    drive(0, 0);
}

// Demo adapter: both controllers share this transport and run sequentially.
bool demoMotionReady() {
    if (!sensorsReady) {
        debugPrintln("READY FAULT | ToF initialization failed; see TOF ERROR lines");
        return false;
    }
    mpuYaw.service();
    if (mpuYaw.healthy()) return true;
    if (!mpuYaw.initialized()) {
        mpuYaw.logFault("forward MPU not initialized");
    } else {
        char line[115];
        snprintf(line, sizeof(line),
                 "READY FAULT | sensor=forward MPU6050 0x68 | last read ok=%d | last good=%lums ago",
                 mpuYaw.lastReadOk(), mpuYaw.lastGoodAgeMs());
        debugPrintln(line);
        if (!mpuYaw.lastReadOk()) mpuYaw.logFault("latest gyro read failed");
    }
    return false;
}

MovementCommand demoReadCommand() {
    handleWiFiClient();
    mpuYaw.service();
    return readCommand();
}

void demoBeginRun() {
    motionStopRequested = false;
    motionActive = true;
    reportMovementConfig();
}

void demoEndRun() {
    moveForwardStop();
    motionActive = false;
    discardPendingCommands();
}

bool demoStopped() { return motionStopRequested; }

bool demoReadPaths(int &front, int &left, int &right) {
    if (!sensorsReady || motionStopRequested) return false;
    drive(0, 0);
    // Retry only while stopped. Forward motion still brakes on its first fault.
    for (int attempt=1; attempt<=3; ++attempt) {
        handleWiFiClient();
        serviceMotionSensors();
        if (motionStopRequested) return false;
        frontFilter.reset(); leftFilter.reset(); rightFilter.reset();
        int rightRaw = -1;
        bool valid = observe(front, left, right, rightRaw, true);
        if (motionStopRequested) return false;
        if (valid && !sideCommunicationFault && front>=0 && left>=0 && right>=0) {
            if (attempt>1) debugPrintln("SCAN | recovered; valid sample received");
            return true;
        }
        char line[200];
        snprintf(line,sizeof(line),
            "SCAN ERROR | attempt=%d/3 | %s | code=%d | mm F/L/R=%d/%d/%d",
            attempt,tofFailure,tofFailureCode,front,left,right);
        debugPrintln(line);
        if (attempt<3) waitWithMotionService(50);
    }
    return false;
}

DemoMoveResult demoMoveOneCell() {
    return demoMoveStraightCells(1);
}

DemoMoveResult demoMoveStraightCells(int cells) {
    if (motionStopRequested) return DemoMoveResult::Stopped;
    if (!demoMotionReady()) return DemoMoveResult::Failed;
    frontFilter.reset(); leftFilter.reset(); rightFilter.reset();
    wallDetector.reset();
    // Capture a new heading after every turn, before the next forward move.
    mpuYaw.reset();
    return runForwardDistance(cells);
}

bool readFrontForReference(int &front) {
    drive(0, 0);
    for (int attempt = 0; attempt < 3; ++attempt) {
        handleWiFiClient();
        int left, right, rightRaw;
        if (observe(front, left, right, rightRaw, true, true)) return true;
        if (motionStopRequested) return false;
        if (attempt < 2) waitWithMotionService(50);
    }
    char line[110];
    snprintf(line, sizeof(line), "WALL REFERENCE SCAN ERROR | %s | code=%d",
             tofFailure, tofFailureCode);
    debugPrintln(line);
    return false;
}

DemoMoveResult demoReachFrontWallReference() {
    if (motionStopRequested) return DemoMoveResult::Stopped;
    int front;
    if (!readFrontForReference(front))
        return motionStopRequested ? DemoMoveResult::Stopped : DemoMoveResult::Failed;
    if (front > FRONT_MAP_OPEN_MM) return DemoMoveResult::EncoderReached;
    if (front <= FRONT_WALL_STOP_TRIGGER_MM) {
        char line[90];
        snprintf(line, sizeof(line), "WALL REFERENCE | already at/below trigger | front=%dmm", front);
        debugPrintln(line);
        return DemoMoveResult::FrontWallReached;
    }
    // Confirm a nearby wall after the chassis and heading have settled.
    if (!readFrontForReference(front))
        return motionStopRequested ? DemoMoveResult::Stopped : DemoMoveResult::Failed;
    if (front > FRONT_MAP_OPEN_MM) return DemoMoveResult::EncoderReached;
    if (front <= FRONT_WALL_STOP_TRIGGER_MM) return DemoMoveResult::FrontWallReached;
    mpuYaw.reset();
    return runForwardDistance(0);
}

bool demoHeadingError(float &yawRightDegrees) {
    mpuYaw.service();
    if (mpuYaw.healthy()) {
        yawRightDegrees = mpuYaw.yaw() * MPU_YAW_SIGN;
        return isfinite(yawRightDegrees);
    }
    debugPrintln("ALIGN | MPU unavailable after I2C retries");
    if (!mpuYaw.lastReadOk()) mpuYaw.logFault("arrival alignment");
    else {
        char line[90];
        snprintf(line, sizeof(line), "MPU FAULT | alignment sample stale | last good=%lums ago",
                 mpuYaw.lastGoodAgeMs());
        debugPrintln(line);
    }
    return false;
}

bool demoResetYaw() {
    if (!demoMotionReady()) return false;
    mpuYaw.reset();
    debugPrintln("MPU RESET | forward relative yaw=0deg");
    return true;
}

void demoLog(const char *text) { debugPrintln(text); }

// Required by the unchanged rotation.cpp implementation.
void logPrint(const String &text) { debugPrint(text); }
void logPrintln(const String &text) { debugPrintln(text); }
bool abortRequested() {
    handleWiFiClient();
    if (readCommand() == MovementCommand::Stop) motionStopRequested = true;
    return motionStopRequested;
}
