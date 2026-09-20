#include "../ForwardWallControl.h"
#include <assert.h>
#include <stdio.h>

int main() {
  assert(sideClearanceMm(50, 10) == 40);
  assert(sideClearanceMm(49, 10) == 39);
  assert(sideClearanceMm(10, 10) == 0);
  assert(sideClearanceMm(5, 10) == 0);
  assert(sideClearanceMm(-1, 10) == -1);
  const ForwardWallSettings correctedSettings = {70, 30, 40, 40};
  assert(forwardWallCommands(40, sideClearanceMm(50, 10), correctedSettings).right == 70);
  assert(forwardWallCommands(40, sideClearanceMm(49, 10), correctedSettings).right == 30);
  const ForwardWallSettings settings = {70, 30, 40, 50};
  struct Example { int leftMm, rightMm, leftCommand, rightCommand; };
  const Example examples[] = {
    {47, 49, 70, 30}, // User log: right below original 50 mm threshold.
    {44, 55, 70, 70},
    {32, 68, 30, 70}, // User log: original left correction, opposite old PID.
    {40, 50, 70, 70}, // Strict threshold boundaries from main.ino.
    {39, 50, 30, 70},
    {40, 49, 70, 30},
    {20, 20, 30, 70}, // Left wins when both thresholds trigger.
    {-1, 35, 70, 30}, // Invalid left must not trigger left correction.
    {35, -1, 30, 70},
    {80, -1, 70, 70},
    {45, 58, 70, 70} // Observed 103 mm sum no longer rejects a valid sample.
  };
  for (const Example &example : examples) {
    const auto commands = forwardWallCommands(example.leftMm, example.rightMm, settings);
    assert(commands.left == example.leftCommand);
    assert(commands.right == example.rightCommand);
  }
  assert(!cellEncoderLimitReached(622, 618, 623, 619));
  assert(cellEncoderLimitReached(623, 580, 623, 619));
  assert(cellEncoderLimitReached(580, 619, 623, 619));
  assert(cellEncoderLimitReached(640, 630, 623, 619));
  puts("PASS: recorded readings, original steering direction, thresholds, invalid sides, joint braking");
}
