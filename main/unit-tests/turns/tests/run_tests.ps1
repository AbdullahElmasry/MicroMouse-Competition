$ErrorActionPreference = 'Stop'
$turnsDirectory = Split-Path $PSScriptRoot -Parent
$testDirectory = Join-Path ([IO.Path]::GetTempPath()) ('rotation-tests-' + [guid]::NewGuid())
New-Item -ItemType Directory -Path $testDirectory | Out-Null

# Keep fake hardware outside the sketch so Arduino only builds production files.
@'
#pragma once
#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstdint>
#include <string>
#include <deque>
class String : public std::string {
public:
  using std::string::string;
  template<typename T> String(T value) : std::string(std::to_string(value)) {}
};
constexpr int OUTPUT = 1, HIGH = 1;
inline unsigned long clockUs = 0;
inline int pwm[40] = {}, firstLeft = 0, firstRight = 0;
inline int maxPwm = 0, minPwm = 255, samples = 0;
inline float physicalAngle = 0;
inline float simulatedRate = 0;
inline bool frozen = false, readFailure = false, initFailure = false;
inline unsigned long injectStopAt = 0;
inline bool stopViaWifi = false;
inline std::deque<int> usbInput, wifiInput;
inline std::string logs;
template<typename T> T constrain(T v, T lo, T hi) {
  return std::max(lo, std::min(v, hi));
}
inline unsigned long micros() {
  clockUs += 1000;
  if (injectStopAt && clockUs >= injectStopAt) {
    (stopViaWifi ? wifiInput : usbInput).push_back('x');
    injectStopAt = 0;
  }
  return clockUs;
}
inline unsigned long millis() { return clockUs / 1000; }
inline void delay(unsigned long ms) { clockUs += ms * 1000; }
inline void pinMode(int, int) {}
inline void digitalWrite(int, int) {}
inline void analogWrite(int pin, int value) {
  assert(value >= 0 && value <= 150);
  pwm[pin] = value;
  if (value) {
    maxPwm = std::max(maxPwm, value);
    minPwm = std::min(minPwm, value);
    if (!firstLeft && (pin == 25 || pin == 26)) firstLeft = pin;
    if (!firstRight && (pin == 14 || pin == 27)) firstRight = pin;
  }
}
struct FakeSerial {
  void begin(int) {}
  int available() { return usbInput.size(); }
  int read() { int c = usbInput.front(); usbInput.pop_front(); return c; }
  void print(const String &s) { logs += s; }
  void println(const String &s = "") { logs += s; logs += '\n'; }
};
inline FakeSerial Serial;
'@ | Set-Content -LiteralPath (Join-Path $testDirectory 'Arduino.h')

@'
#pragma once
#include "Arduino.h"
struct FakeWire {
  uint16_t raw = 0;
  uint8_t reg = 0;
  int writes = 0;
  bool highByte = true;
  void begin(int sda, int scl) { assert(sda == 21 && scl == 22); }
  void beginTransmission(int address) { assert(address == 0x68); writes = 0; }
  void write(int value) { if (writes++ == 0) reg = value; }
  int endTransmission(bool = true) { return initFailure ? 1 : 0; }
  int requestFrom(uint8_t, uint8_t count) {
    if (readFailure) return 0;
    if (reg == 0x75) return count;
    ++samples;
    int direction = frozen ? 0 : (pwm[25] ? 1 : (pwm[26] ? -1 : 0));
    int motorPwm = std::max(pwm[25], pwm[26]);
    float targetRate = direction * motorPwm;
    simulatedRate += (targetRate - simulatedRate) * 0.2f;
    physicalAngle += simulatedRate * 0.005f;
    raw = static_cast<uint16_t>(42 - simulatedRate * 65.5f);
    highByte = true;
    return count;
  }
  int read() {
    if (reg == 0x75) return 0x68;
    int value = highByte ? raw >> 8 : raw & 255;
    highByte = false;
    return value;
  }
};
inline FakeWire Wire;
'@ | Set-Content -LiteralPath (Join-Path $testDirectory 'Wire.h')

@'
#pragma once
#include "Arduino.h"
constexpr int WIFI_STA = 1, WL_CONNECTED = 3;
struct WiFiClient {
  bool active = false;
  explicit operator bool() const { return active; }
  bool connected() { return active; }
  void stop() { active = false; }
  void setNoDelay(bool) {}
  int available() { return wifiInput.size(); }
  int read() { int c = wifiInput.front(); wifiInput.pop_front(); return c; }
  void print(const String &) {}
  void println(const String &) {}
};
struct WiFiServer {
  WiFiServer(int) {}
  void begin() {}
  void setNoDelay(bool) {}
  WiFiClient available() { return {}; }
};
struct FakeIP { String toString() { return "127.0.0.1"; } };
struct FakeWiFi {
  void mode(int) {}
  void begin(const char *, const char *) {}
  int status() { return WL_CONNECTED; }
  FakeIP localIP() { return {}; }
};
inline FakeWiFi WiFi;
'@ | Set-Content -LiteralPath (Join-Path $testDirectory 'WiFi.h')

@'
#include "Arduino.h"
#include "turns.ino"
#include <iostream>
void assertStopped() {
  assert(pwm[25] == 0 && pwm[26] == 0 && pwm[14] == 0 && pwm[27] == 0);
}
void reset() {
  stopRotation();
  frozen = readFailure = initFailure = false;
  injectStopAt = 0;
  firstLeft = firstRight = maxPwm = 0;
  minPwm = 255;
  physicalAngle = 0;
  simulatedRate = 0;
  usbInput.clear(); wifiInput.clear(); logs.clear();
  assert(beginRotation());
}
int main() {
  setup();
  assertStopped();
  assert(samples == 200);
  for (char command : {'r', 'R', 'l', 'L'}) {
    reset();
    usbInput.push_back(command);
    loop();
    bool right = command == 'r' || command == 'R';
    assert(firstLeft == (right ? 25 : 26));
    assert(firstRight == (right ? 27 : 14));
    assert(std::fabs(physicalAngle - (right ? 90 : -90)) <= 1.5f);
    assert(logs.find("Turn done.") != std::string::npos);
    assert(maxPwm == 150 && minPwm == 85);
    assertStopped();
  }
  reset();
  assert(turnDegrees(90)); assert(turnDegrees(90));
  assert(turnDegrees(-90)); assert(turnDegrees(-90));
  assert(std::fabs(physicalAngle) <= 6);
  reset();
  wifiClient.active = true;
  wifiInput.push_back('r'); loop();
  assert(logs.find("Turn done.") != std::string::npos);
  for (bool wifi : {false, true}) {
    reset(); stopViaWifi = wifi;
    injectStopAt = clockUs + 30000;
    assert(!turnDegrees(90));
    assert(logs.find("aborted") != std::string::npos);
    assertStopped();
  }
  for (char command : {'d', 'D', 'x', 'X'}) {
    reset(); wifiInput.push_back(command);
    assert(!turnDegrees(90)); assertStopped();
    reset(); usbInput.push_back(command);
    assert(!turnDegrees(90)); assertStopped();
  }
  reset(); frozen = true;
  unsigned long start = millis();
  assert(!turnDegrees(90)); assertStopped();
  assert(millis() - start > 2000);
  assert(logs.find("timeout") != std::string::npos);
  reset(); readFailure = true;
  assert(!turnDegrees(90)); assertStopped();
  readFailure = false;
  assert(!turnDegrees(90));
  reset(); initFailure = true;
  assert(!beginRotation()); assert(!turnDegrees(90)); assertStopped();
  std::cout << "PASS: rotation, relative targets, USB/Wi-Fi commands, stops, timeout, MPU failures\n";
}
'@ | Set-Content -LiteralPath (Join-Path $testDirectory 'rotation_test.cpp')

$executable = Join-Path $testDirectory 'rotation_test.exe'
& g++ -static -std=c++17 -Wall -Wextra -Werror "-I$testDirectory" "-I$turnsDirectory" (Join-Path $testDirectory 'rotation_test.cpp') (Join-Path $turnsDirectory 'rotation.cpp') -o $executable
if ($LASTEXITCODE -ne 0) { throw 'Rotation test compilation failed.' }
& $executable
if ($LASTEXITCODE -ne 0) { throw "Rotation tests failed (exit $LASTEXITCODE)." }
