#include <Wire.h>
#include <WiFi.h>
#include <VL6180X.h>
#include <VL53L1X.h>
#include "ForwardWallControl.h"
#include "CellApproachControl.h"
#include "MovementCommands.h"

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

constexpr int LEFT_IN1 = 25, LEFT_IN2 = 26;
constexpr int RIGHT_IN1 = 27, RIGHT_IN2 = 14, MOTOR_EN = 23;

constexpr int LEFT_ENCODER = 33, RIGHT_ENCODER = 35;

constexpr int LEFT_XSHUT = 4, RIGHT_XSHUT = 5, FRONT_XSHUT = 16;

// Two hand-pushed trials over 7 cells (1260 mm): L/R
// 4373/4344, 4345/4325.
constexpr float LEFT_TICKS_PER_CELL =
    ((4373.0f + 4345.0f) / 2.0f) / 7.0f;

constexpr float RIGHT_TICKS_PER_CELL =
    ((4344.0f + 4325.0f) / 2.0f) / 7.0f;

constexpr int CELL_LENGTH_MM = 180;
constexpr int FORWARD_CELLS = 7;
constexpr unsigned long CELL_PAUSE_MS = 500;

constexpr unsigned long LEFT_TARGET_TICKS =
    (unsigned long)(LEFT_TICKS_PER_CELL + 0.5f);

constexpr unsigned long RIGHT_TARGET_TICKS =
    (unsigned long)(RIGHT_TICKS_PER_CELL + 0.5f);

static_assert(
    LEFT_TARGET_TICKS == 623 &&
    RIGHT_TARGET_TICKS == 619,
    "Check one-cell targets from seven-cell calibration"
);

constexpr int FORWARD_TARGET_MM =
    CELL_LENGTH_MM * FORWARD_CELLS;

constexpr int FORWARD_SPEED = 70;

constexpr unsigned long MOVE_TIMEOUT_MS =
    15000UL; // Per-cell movement timeout; pauses are separate.

constexpr unsigned long STALL_TIMEOUT_MS = 1500;

constexpr float LEFT_FACTOR = 1.0f;
constexpr float RIGHT_FACTOR = 0.78f;

// Working main.ino behavior: asymmetric thresholds and slow the same named motor.
constexpr int WALL_SLOW_SPEED = 30;
constexpr int LEFT_WALL_THRESHOLD_MM = 40;
constexpr int RIGHT_WALL_THRESHOLD_MM = 50;
constexpr ForwardWallSettings WALL_SETTINGS = {
    FORWARD_SPEED, WALL_SLOW_SPEED, LEFT_WALL_THRESHOLD_MM, RIGHT_WALL_THRESHOLD_MM
};
// Distance PID changes forward speed only; preserve the working steering direction.
constexpr float APPROACH_SLOWDOWN_TICKS = 300.0f;
constexpr int MIN_APPROACH_PWM = 35;
constexpr CellApproachSettings APPROACH_SETTINGS = {
    0.24f, 0.01f, 0.01f, // Gentler approach response; tune against measured travel.
    APPROACH_SLOWDOWN_TICKS, // Begin approach about 87 mm before braking.
    FORWARD_SPEED, MIN_APPROACH_PWM
};
constexpr unsigned long BRAKE_LEAD_TICKS = 10; // Brake about 2.9 mm earlier per cell.
static_assert(BRAKE_LEAD_TICKS < LEFT_TARGET_TICKS && BRAKE_LEAD_TICKS < RIGHT_TARGET_TICKS,
              "Brake lead must be smaller than a cell");
CellApproachController approachPid(APPROACH_SETTINGS);

// ============================================================
// Sensors / encoders
// ============================================================

volatile unsigned long leftTicks = 0;
volatile unsigned long rightTicks = 0;

VL6180X leftTof;
VL6180X rightTof;
VL53L1X frontTof;

bool sensorsReady = false;
MovementCommands usbCommands;
MovementCommands wifiCommands;


// ============================================================
// WiFi
// ============================================================

void initWiFi() {

    WiFi.mode(WIFI_STA);

    Serial.println();
    Serial.print("Connecting to WiFi: ");
    Serial.println(WIFI_SSID);

    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

    unsigned long started = millis();

    while (WiFi.status() != WL_CONNECTED) {

        delay(500);
        Serial.print(".");

        // Don't block forever if the WiFi is unavailable.
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

    // WiFi isn't connected.
    if (WiFi.status() != WL_CONNECTED) {
        wifiCommands.reset();
        return;
    }

    // Keep current client if connected.
    if (wifiSerialClient &&
        wifiSerialClient.connected()) {
        return;
    }

    // Remove disconnected client.
    if (wifiSerialClient) {
        wifiSerialClient.stop();
    }
    wifiCommands.reset();

    // Check for new connection.
    WiFiClient newClient =
        wifiSerialServer.accept();

    if (newClient) {

        wifiSerialClient = newClient;
        wifiSerialClient.setNoDelay(true);

        Serial.println();
        Serial.println(
            "WiFi serial client connected."
        );

        wifiSerialClient.println(
            "================================="
        );

        wifiSerialClient.println(
            "ESP32 Micromouse connected"
        );

        wifiSerialClient.print(
            "ESP32 IP: "
        );

        wifiSerialClient.println(
            WiFi.localIP()
        );

        wifiSerialClient.println(
            "Send start for 7 cells with 500 ms pauses at each cell; d stops the run."
        );

        wifiSerialClient.println(
            "================================="
        );
    }
}


// ============================================================
// Debug output
// ============================================================

// String / char / integer / etc.
template<typename T>
void debugPrint(const T& value) {

    Serial.print(value);

    if (wifiSerialClient &&
        wifiSerialClient.connected()) {

        wifiSerialClient.print(value);
    }
}


template<typename T>
void debugPrintln(const T& value) {

    Serial.println(value);

    if (wifiSerialClient &&
        wifiSerialClient.connected()) {

        wifiSerialClient.println(value);
    }
}


void debugPrintln() {

    Serial.println();

    if (wifiSerialClient &&
        wifiSerialClient.connected()) {

        wifiSerialClient.println();
    }
}


// Float with decimal places
void debugPrint(float value, int digits) {

    Serial.print(value, digits);

    if (wifiSerialClient &&
        wifiSerialClient.connected()) {

        wifiSerialClient.print(value, digits);
    }
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
        false
    );

    motor(
        RIGHT_IN1,
        RIGHT_IN2,
        right,
        RIGHT_FACTOR,
        true
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
    debugPrint(name);
    debugPrint(" | ");
    debugPrint(step);
    debugPrint(" | I2C code=");
    debugPrint(status);
    debugPrintln(status == 0 ? " OK" : " FAILED");
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

  digitalWrite(RIGHT_XSHUT, HIGH);
  delay(50);
  rightTof.init();
  uint8_t rightInitStatus = rightTof.last_status;
  rightTof.configureDefault();
  uint8_t rightConfigStatus = rightTof.last_status;
  rightTof.setAddress(0x31);
  uint8_t rightAddressStatus = rightTof.last_status;
  rightTof.setTimeout(200);

  digitalWrite(FRONT_XSHUT, HIGH);
  delay(50);
  bool frontInitOk = frontTof.init();
  bool frontModeOk = frontTof.setDistanceMode(VL53L1X::Medium);
  frontTof.setAddress(0x32);
  uint8_t frontAddressStatus = frontTof.last_status;
  frontTof.setTimeout(200);
  frontTof.startContinuous(30);
  uint8_t frontStartStatus = frontTof.last_status;

  reportTofStep("LEFT", "init", leftInitStatus);
  reportTofStep("LEFT", "configureDefault", leftConfigStatus);
  reportTofStep("LEFT", "setAddress", leftAddressStatus);
  leftReady = reportTofStep("LEFT", "probe 0x30", probeTof(0x30));
  reportTofStep("RIGHT", "init", rightInitStatus);
  reportTofStep("RIGHT", "configureDefault", rightConfigStatus);
  reportTofStep("RIGHT", "setAddress", rightAddressStatus);
  rightReady = reportTofStep("RIGHT", "probe 0x31", probeTof(0x31));
  debugPrintln(frontInitOk ? "FRONT | init OK" : "FRONT | init FAILED");
  debugPrintln(frontModeOk ? "FRONT | distance mode OK" : "FRONT | distance mode FAILED");
  reportTofStep("FRONT", "setAddress", frontAddressStatus);
  reportTofStep("FRONT", "startContinuous", frontStartStatus);
  frontReady = reportTofStep("FRONT", "probe 0x32", probeTof(0x32)) && frontInitOk;
  debugPrintln("Address ACK allows diagnostic reads; check range validity below.");
  return leftReady && rightReady && frontReady;
}

bool observe(
    int &front,
    int &left,
    int &right
) {

    front = frontTof.read();

    bool frontValid =
        !frontTof.timeoutOccurred() &&
        frontTof.last_status == 0 &&
        frontTof.ranging_data.range_status ==
        VL53L1X::RangeValid;


    left =
        leftTof.readRangeSingleMillimeters();

    bool leftValid =
        !leftTof.timeoutOccurred() &&
        leftTof.last_status == 0;

    uint8_t leftStatus =
        leftTof.readRangeStatus();

    leftValid =
        leftValid &&
        leftStatus == 0 &&
        leftTof.last_status == 0;


    right =
        rightTof.readRangeSingleMillimeters();

    bool rightValid =
        !rightTof.timeoutOccurred() &&
        rightTof.last_status == 0;

    uint8_t rightStatus =
        rightTof.readRangeStatus();

    rightValid =
        rightValid &&
        rightStatus == 0 &&
        rightTof.last_status == 0;


    if (!leftValid)
        left = -1;

    if (!rightValid)
        right = -1;

    return frontValid;
}


// ============================================================
// Movement
// ============================================================

bool runOneCell(unsigned long &stoppedAtMs) {
    if (!sensorsReady) {
        debugPrintln("REFUSED: ToF initialization failed.");
        return false;
    }

    unsigned long startLeft, startRight;
    readTicks(startLeft, startRight);
    unsigned long started = millis();
    unsigned long lastLeftChange = started, lastRightChange = started;
    unsigned long previousLeft = 0, previousRight = 0, lastLog = started;
    const char *result = "TIMEOUT";
    bool cellCompleted = false;
    unsigned long previousControlMs = started;
    approachPid.reset();

    while (true) {
        handleWiFiClient();
        if (readCommand() == MovementCommand::Stop) {
            result = "ABORTED";
            break;
        }

        unsigned long rawLeft, rawRight;
        readTicks(rawLeft, rawRight);
        unsigned long left = rawLeft - startLeft, right = rawRight - startRight;
        unsigned long now = millis();
        if (left != previousLeft) { lastLeftChange = now; previousLeft = left; }
        if (right != previousRight) { lastRightChange = now; previousRight = right; }

        // Exactly the base sketch's OR stopping rule, with calibrated targets.
        if (cellEncoderLimitReached(left, right, LEFT_TARGET_TICKS - BRAKE_LEAD_TICKS,
                                    RIGHT_TARGET_TICKS - BRAKE_LEAD_TICKS)) {
            result = "CELL ENCODER LIMIT REACHED";
            cellCompleted = true;
            break;
        }
        if (now - started >= MOVE_TIMEOUT_MS) break;
        if (now - lastLeftChange >= STALL_TIMEOUT_MS ||
            now - lastRightChange >= STALL_TIMEOUT_MS) {
            result = "ENCODER STALL";
            break;
        }

        int front = -1, sideLeft = -1, sideRight = -1;
        bool frontValid = observe(front, sideLeft, sideRight);
        // Stop wins over a sensor fault received during a blocking read.
        handleWiFiClient();
        if (readCommand() == MovementCommand::Stop) {
            result = "ABORTED";
            break;
        }
        if (!frontValid) {
            result = "INVALID FRONT TOF READING";
            break;
        }
        // Front obstacle proximity stopping remains disabled as requested.
        if (sideLeft < 0 && sideRight < 0) {
            result = "BOTH SIDE TOF READINGS INVALID";
            break;
        }

        // Sensor calls can take time: check distance again before applying PWM.
        readTicks(rawLeft, rawRight);
        left = rawLeft - startLeft;
        right = rawRight - startRight;
        if (cellEncoderLimitReached(left, right, LEFT_TARGET_TICKS - BRAKE_LEAD_TICKS,
                                    RIGHT_TARGET_TICKS - BRAKE_LEAD_TICKS)) {
            result = "CELL ENCODER LIMIT REACHED";
            cellCompleted = true;
            break;
        }
        unsigned long leftRemaining = ticksBeforeBrake(left, LEFT_TARGET_TICKS, BRAKE_LEAD_TICKS);
        unsigned long rightRemaining = ticksBeforeBrake(right, RIGHT_TARGET_TICKS, BRAKE_LEAD_TICKS);
        unsigned long remaining = leftRemaining < rightRemaining ? leftRemaining : rightRemaining;
        unsigned long controlMs = millis();
        float dt = (controlMs - previousControlMs) / 1000.0f;
        previousControlMs = controlMs;
        int approachSpeed = approachPid.update((float)remaining, dt);
        ForwardWallSettings steering = WALL_SETTINGS;
        steering.baseSpeed = approachSpeed;
        if (steering.slowSpeed > approachSpeed) steering.slowSpeed = approachSpeed;
        const ForwardMotorCommands commands = forwardWallCommands(sideLeft, sideRight, steering);
        drive(commands.left, commands.right);

        if (millis() - lastLog >= 200) {
            // One complete line reduces the number of WiFi write calls.
            char telemetry[220];
            snprintf(telemetry, sizeof(telemetry),
                "ticks L/R: %lu/%lu | mm F/L/R: %d/%d/%d | command L/R: %d/%d | remaining ticks: %lu | distance PID PWM: %d",
                left, right, front, sideLeft, sideRight, commands.left, commands.right, remaining, approachSpeed);
            debugPrintln(telemetry);
            lastLog = millis();
        }
        delay(10); // Same movement-loop delay as main.ino.
    }

    drive(0, 0); // Always brake both wheels together.
    stoppedAtMs = millis();
    unsigned long brakeLeft, brakeRight;
    readTicks(brakeLeft, brakeRight);
    approachPid.reset();
    delay(100); // Settling time counts toward the 500 ms cell pause.
    unsigned long endLeft, endRight;
    readTicks(endLeft, endRight);
    debugPrint(result);
    debugPrint(" | per-cell limits L/R: "); debugPrint(LEFT_TARGET_TICKS);
    debugPrint('/'); debugPrint(RIGHT_TARGET_TICKS);
    debugPrint(" | final L/R: "); debugPrint(endLeft - startLeft);
    debugPrint('/'); debugPrint(endRight - startRight);
    debugPrint(" | ticks at brake L/R: "); debugPrint(brakeLeft - startLeft);
    debugPrint('/'); debugPrint(brakeRight - startRight);
    debugPrint(" | ticks after brake L/R: "); debugPrint(endLeft - brakeLeft);
    debugPrint('/'); debugPrint(endRight - brakeRight);
    debugPrint(" | elapsed ms: "); debugPrintln(millis() - started);
    debugPrintln("Both motors stop at the first encoder limit. Measure actual cell travel against 180 mm.");
    return cellCompleted;
}

void runMove() {
    for (int cell = 1; cell <= FORWARD_CELLS; ++cell) {
        // Stop wins even at the transition between cells; extra starts are ignored.
        handleWiFiClient();
        if (readCommand() == MovementCommand::Stop) {
            drive(0, 0);
            debugPrintln("ABORTED: remaining cells cancelled.");
            return;
        }
        debugPrint("Moving cell "); debugPrint(cell);
        debugPrint('/'); debugPrintln(FORWARD_CELLS);
        unsigned long stoppedAtMs = 0;
        if (!runOneCell(stoppedAtMs)) {
            debugPrintln("Run stopped; remaining cells cancelled.");
            return;
        }
        if (cell == FORWARD_CELLS) break;
        debugPrintln("Cell complete. Pausing 500 ms, then continuing automatically.");
        // Count the existing settling/reporting time toward the half-second stop.
        do {
            handleWiFiClient();
            if (readCommand() == MovementCommand::Stop) {
                drive(0, 0);
                debugPrintln("ABORTED during pause: remaining cells cancelled.");
                return;
            }
            delay(1);
        } while (millis() - stoppedAtMs < CELL_PAUSE_MS);
    }
    debugPrintln("All 7 cells completed. Robot remains stopped.");
}


// ============================================================
// Setup
// ============================================================

void setup() {

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

    // Establish stopped motor outputs before waiting for WiFi connection.
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

    digitalWrite(
        MOTOR_EN,
        HIGH
    );


    debugPrintln(
        sensorsReady
            ? "ToF initialization OK."
            : "ToF initialization FAILED."
    );


    debugPrint(
        "Forward run on start command: "
    );

    debugPrint(
        FORWARD_CELLS
    );

    debugPrint(
        " cells, target mm: "
    );

    debugPrint(
        FORWARD_TARGET_MM
    );

    debugPrint(
        ", per-cell target ticks L/R: "
    );

    debugPrint(
        LEFT_TARGET_TICKS
    );

    debugPrint('/');

    debugPrintln(
        RIGHT_TARGET_TICKS
    );


    debugPrintln(
        "main.ino wall steering + distance PID: cruise 70, approach floor 35; slow steering 30."
    );

    debugPrintln(
        "Front obstacle proximity stop disabled; invalid readings still stop the run."
    );

    debugPrintln(
        "Distance PID approaches over 300 ticks; both wheels brake 10 ticks before their nominal cell limits."
    );


    if (!sensorsReady) {

        debugPrintln(
            "Movement unavailable: ToF initialization failed. Reset to retry."
        );

        return;
    }


    debugPrintln(
        "Ready. Send start for 7 cells, pausing 500 ms after each cell; d cancels the run."
    );


    discardPendingCommands();
}


// ============================================================
// Loop
// ============================================================

void loop() {

    // One start runs seven cells with automatic half-second stops between cells.
    handleWiFiClient();
    MovementCommand command = readCommand();
    if (command == MovementCommand::Stop) {
        drive(0, 0);

        discardPendingCommands();
        debugPrintln("Stopped. Send start for a fresh seven-cell run.");
    } else if (command == MovementCommand::Start) {
        debugPrintln("Start received.");
        runMove();
        discardPendingCommands();
        debugPrintln("Run ended. Staying stopped; send start for a new seven-cell run.");
    }

    delay(10);
}
