#include "../ForwardWallControl.h"
#include <assert.h>
#include <stdio.h>

int main() {
  TwoWallPidController pid;
  const TwoWallPidSettings settings = {3.6f, 30, 1.0f, 0.0f, 0.0f, 40.0f};
  float error = 0.0f;
  float correction = 0.0f;

  auto centered = pid.update(40, 42, 112, settings, 0.05f, error, correction);
  assert(centered.left == 112 && centered.right == 112);
  assert(error == 0.0f && correction == 0.0f);

  auto nearLeft = pid.update(30, 50, 112, settings, 0.05f, error, correction);
  assert(error == 20.0f && correction == 20.0f);
  assert(nearLeft.left == 92 && nearLeft.right == 112);

  pid.reset();
  auto nearRight = pid.update(50, 30, 112, settings, 0.05f, error, correction);
  assert(error == -20.0f && correction == -20.0f);
  assert(nearRight.left == 112 && nearRight.right == 92);

  pid.reset();
  auto limited = pid.update(20, 100, 112, settings, 0.05f, error, correction);
  assert(correction == 40.0f);
  assert(limited.left == 72 && limited.right == 112);

  auto approaching = pid.update(20, 100, 35, settings, 0.05f, error, correction);
  assert(correction == 5.0f);
  assert(approaching.left == 30 && approaching.right == 35);

  const TwoWallPidSettings derivativeSettings = {0.0f, 30, 1.0f, 0.0f, 5.0f, 40.0f};
  pid.reset();
  pid.update(40, 60, 112, derivativeSettings, 0.05f, error, correction);
  auto improvingLeft = pid.update(40, 41, 112, derivativeSettings, 0.05f,
                                  error, correction);
  assert(correction >= 0.0f);
  assert(improvingLeft.right == 112);

  puts("PASS: two-wall PID direction, tolerance, output limit, and speed floor");
}
