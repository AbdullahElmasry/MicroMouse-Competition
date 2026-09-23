#include "../MoveForward.h"
#include <assert.h>
#include <stdio.h>

int main() {
  const CellApproachSettings settings = {0.24f, 0.01f, 0.01f, 300, 70, 35};
  CellApproachController pid(settings);
  assert(pid.update(600, 0.05f) == 70);
  assert(pid.update(300, 0.05f) == 70);
  int previous = 70;
  for (int ticks = 275; ticks > 0; ticks -= 25) {
    int speed = pid.update((float)ticks, 0.05f);
    assert(speed >= 35 && speed <= previous);
    previous = speed;
  }
  assert(pid.update(0, 0.05f) == 0);
  assert(pid.update(600, 0.05f) == 70); // A fresh cell returns to cruise.
  for (int i = 0; i < 1000; ++i) assert(pid.update(1, 0.05f) == 35);
  pid.reset();
  assert(pid.update(200, 0.05f) == 48);
  assert(pid.update(100, 0) == 35);
  assert(pid.update(NAN, 0.05f) == 0);
  assert(ticksBeforeBrake(500, 623, 0) == 123);
  assert(ticksBeforeBrake(500, 623, 20) == 103);
  assert(ticksBeforeBrake(612, 623, 10) == 1);
  assert(ticksBeforeBrake(613, 623, 10) == 0);
  assert(ticksBeforeBrake(609, 619, 10) == 0);
  assert(ticksBeforeBrake(624, 623, 0) == 0); // No unsigned wrap after overshoot.
  puts("PASS: distance PID slowdown, limits, reset, anti-windup, brake lead, overshoot arithmetic");
}
