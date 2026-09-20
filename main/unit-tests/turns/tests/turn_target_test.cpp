#include "../TurnTarget.h"
#include <assert.h>
#include <stdio.h>

int main() {
  float projected;
  assert(!turnTargetReached(true,76,160,90,1.5f,4,0.05f,projected));
  assert(projected==88);
  assert(turnTargetReached(true,77,160,90,1.5f,4,0.05f,projected));
  assert(projected==89);
  assert(turnTargetReached(true,92,0,90,1.5f,4,0.05f,projected));
  assert(!turnTargetReached(false,-76,-160,-90,1.5f,4,0.05f,projected));
  assert(projected== -88);
  assert(turnTargetReached(false,-77,-160,-90,1.5f,4,0.05f,projected));
  assert(projected== -89);
  assert(turnTargetReached(false,-92,0,-90,1.5f,4,0.05f,projected));
  assert(turnTargetError(30,90)==60);
  assert(turnTargetError(-30,-90)== -60);
  puts("PASS: left/right yaw thresholds, tolerance, and overshoot capture");
}
