#pragma once
#include <cstdio>
#include <cstdint>

extern unsigned long testClock;
inline unsigned long millis() { return testClock; }
inline void delay(unsigned long ms) { testClock += ms; }
struct FakeSerial { void begin(int) {} };
extern FakeSerial Serial;
