#include "../WallCentering.h"
#include <assert.h>
#include <stdio.h>

int main() {
  WallGeometry geometry = {180, 100, 0, 0, 30, 8, 3.6f};
  float error = 0;
  assert(wallCenterError(40, 40, geometry, error) && error == 0);
  assert(wallCenterError(37, 43, geometry, error) && error == 0);
  assert(wallCenterError(36.5f, 43.5f, geometry, error) && error == 0);
  assert(wallCenterError(43.5f, 36.5f, geometry, error) && error == 0);
  assert(wallCenterError(36, 44, geometry, error) && error == 4);
  assert(wallCenterError(25, 55, geometry, error) && error == 15);
  assert(wallCenterError(55, 25, geometry, error) && error == -15);
  assert(wallCenterError(45, 56, geometry, error) && error == 5.5f);
  assert(wallCenterError(46, 56, geometry, error) && error == 5.0f);
  assert(wallCenterError(45, 58, geometry, error) && error == 6.5f);
  assert(wallCenterError(40, 70, geometry, error)); // Upper sum boundary: 110 mm.
  assert(!wallCenterError(40, 71, geometry, error));
  geometry.leftSensorInsetMm = 5;
  assert(wallCenterError(45, 40, geometry, error) && error == 0);
  assert(!wallCenterError(-1, 40, geometry, error));
  assert(!wallCenterError(45, 180, geometry, error)); // Side opening.
  assert(!wallCenterError(10, 75, geometry, error)); // Too close to body.
  assert(!wallCenterError(NAN, 40, geometry, error));

  WallPid pid(0.8f, 0.05f, 0.03f, 20);
  assert(pid.update(0, 0.05f) == 0);
  pid.reset();
  assert(pid.update(15, 0.05f) > 0); // Left command rises, right falls.
  pid.reset();
  assert(pid.update(-15, 0.05f) < 0);
  for (int i = 0; i < 1000; ++i) assert(pid.update(100, 0.05f) <= 20);
  assert(pid.update(0, 0.05f) >= -20);
  assert(fabsf(pid.update(0, 0.05f)) < 0.1f); // No accumulated saturated integral.
  pid.reset();
  assert(pid.update(0, 0.05f) == 0);
  assert(pid.update(10, 0) == 0);
  assert(pid.update(NAN, 0.05f) == 0);

  // Integral uses seconds, rather than number of calls.
  WallPid slow(0, 1, 0, 100), fast(0, 1, 0, 100);
  float slowOut = 0, fastOut = 0;
  for (int i = 0; i < 10; ++i) slowOut = slow.update(2, 0.1f);
  for (int i = 0; i < 20; ++i) fastOut = fast.update(2, 0.05f);
  assert(fabsf(slowOut - fastOut) < 0.0001f);
  puts("PASS: wall geometry, steering direction, PID limits, anti-windup, reset, timing");
}
