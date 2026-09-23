#include <Wire.h>
#include <VL6180X.h>
#include <VL53L1X.h>

constexpr int LEFT_XSHUT = 4, RIGHT_XSHUT = 5, FRONT_XSHUT = 16;
constexpr int MOTOR_ENABLE = 23;
VL6180X leftTof, rightTof;
VL53L1X frontTof;
bool leftReady = false, rightReady = false, frontReady = false;
unsigned long lastReadMs = 0;

uint8_t probe(uint8_t address) {
  Wire.beginTransmission(address);
  return Wire.endTransmission();
}

void scanBus() {
  Serial.println("I2C scan (ACK addresses):");
  int found = 0;
  for (uint8_t address = 1; address < 127; ++address) {
    uint8_t status = probe(address);
    if (status == 0) {
      Serial.print("  0x"); Serial.println(address, HEX);
      ++found;
    } else if (status != 2) {
      Serial.print("  Bus error at 0x"); Serial.print(address, HEX);
      Serial.print(": code "); Serial.println(status);
    }
  }
  if (!found) Serial.println("  No devices acknowledged.");
}

bool checkStep(const char *name, const char *step, uint8_t status) {
  Serial.print(name); Serial.print(" | "); Serial.print(step);
  Serial.print(" | I2C code="); Serial.print(status);
  Serial.println(status == 0 ? " OK" : " FAILED");
  return status == 0;
}

void initSensorsLikeMain() {
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

  checkStep("LEFT", "init", leftInitStatus);
  checkStep("LEFT", "configureDefault", leftConfigStatus);
  checkStep("LEFT", "setAddress", leftAddressStatus);
  leftReady = checkStep("LEFT", "probe 0x30", probe(0x30));
  checkStep("RIGHT", "init", rightInitStatus);
  checkStep("RIGHT", "configureDefault", rightConfigStatus);
  checkStep("RIGHT", "setAddress", rightAddressStatus);
  rightReady = checkStep("RIGHT", "probe 0x31", probe(0x31));
  Serial.println(frontInitOk ? "FRONT | init OK" : "FRONT | init FAILED");
  Serial.println(frontModeOk ? "FRONT | distance mode OK" : "FRONT | distance mode FAILED");
  checkStep("FRONT", "setAddress", frontAddressStatus);
  checkStep("FRONT", "startContinuous", frontStartStatus);
  frontReady = checkStep("FRONT", "probe 0x32", probe(0x32)) && frontInitOk;
  Serial.println("Address ACK allows diagnostic reads; check range validity below.");
}

void printSide(VL6180X &sensor, const char *name, bool ready) {
  Serial.print(name); Serial.print(": ");
  if (!ready) { Serial.println("OFFLINE (assigned address did not acknowledge)"); return; }
  uint16_t mm = sensor.readRangeSingleMillimeters();
  bool timedOut = sensor.timeoutOccurred();
  uint8_t readI2c = sensor.last_status;
  uint8_t rangeStatus = sensor.readRangeStatus();
  uint8_t statusI2c = sensor.last_status;
  bool valid = !timedOut && readI2c == 0 && statusI2c == 0 && rangeStatus == 0;
  Serial.print(valid ? "VALID" : "INVALID");
  Serial.print(" | mm="); Serial.print(mm);
  Serial.print(" | range_status="); Serial.print(rangeStatus);
  Serial.print(" | timeout="); Serial.print(timedOut);
  Serial.print(" | read_i2c="); Serial.print(readI2c);
  Serial.print(" | status_i2c="); Serial.println(statusI2c);
}

void printFront() {
  Serial.print("FRONT: ");
  if (!frontReady) { Serial.println("OFFLINE (initialization failed)"); return; }
  uint16_t mm = frontTof.read();
  bool timedOut = frontTof.timeoutOccurred();
  uint8_t i2cStatus = frontTof.last_status;
  bool valid = !timedOut && i2cStatus == 0 &&
               frontTof.ranging_data.range_status == VL53L1X::RangeValid;
  Serial.print(valid ? "VALID" : "INVALID");
  Serial.print(" | mm="); Serial.print(mm);
  Serial.print(" | range_status="); Serial.print((int)frontTof.ranging_data.range_status);
  Serial.print(" ("); Serial.print(VL53L1X::rangeStatusToString(frontTof.ranging_data.range_status));
  Serial.print(") | timeout="); Serial.print(timedOut);
  Serial.print(" | read_i2c="); Serial.println(i2cStatus);
}

void setup() {
  pinMode(MOTOR_ENABLE, OUTPUT); digitalWrite(MOTOR_ENABLE, LOW);
  const int motorPins[] = {25, 26, 27, 14};
  for (int pin : motorPins) { pinMode(pin, OUTPUT); digitalWrite(pin, LOW); }
  Serial.begin(115200);
  delay(1000); // Allow the serial monitor to attach before diagnostics.
  Serial.println("Three-ToF bench test. Motors disabled. Board-default I2C pins, as in main.ino.");
  Serial.println("Left VL6180X: XSHUT=4, target=0x30");
  Serial.println("Right VL6180X: XSHUT=5, target=0x31");
  Serial.println("Front VL53L1X: XSHUT=16, target=0x32");
  Wire.begin();
  initSensorsLikeMain();
  Serial.println("Final bus scan: expect 0x30, 0x31, 0x32 for successful sensors.");
  scanBus();
  Serial.println("Readings start automatically. s: scan bus. Press board RESET to retry initialization.");
  Serial.println("INVALID distances must not be interpreted as free space.");
}

void loop() {
  if (Serial.available() && Serial.read() == 's') scanBus();
  if (millis() - lastReadMs < 500) return;
  lastReadMs = millis();
  Serial.print("--- ms="); Serial.println(lastReadMs);
  printSide(leftTof, "LEFT", leftReady);
  printSide(rightTof, "RIGHT", rightReady);
  printFront();
}
