#include "../MoveForward.h"
#include <assert.h>
#include <stdio.h>

int main() {
  WallModeDetector detector;
  assert(detector.update(45,55)==WallMode::None);
  assert(detector.update(45,55)==WallMode::None);
  assert(detector.update(45,55)==WallMode::Two);
  assert(detector.update(45,90)==WallMode::Two); // Retention hysteresis.
  assert(detector.update(45,100)==WallMode::LeftOnly);
  assert(detector.update(45,130)==WallMode::LeftOnly);
  assert(detector.update(45,55)==WallMode::LeftOnly);
  assert(detector.update(45,55)==WallMode::LeftOnly);
  assert(detector.update(45,55)==WallMode::Two);
  assert(detector.update(-1,45)==WallMode::RightOnly);
  assert(detector.update(-1,200)==WallMode::None);
  detector.reset();
  assert(detector.update(125,125)==WallMode::None);
  // Replay the open-space run: these distant right readings must not steer.
  const int openReadings[] = {190,87,134,190,190};
  for (int right : openReadings)
    assert(detector.update(200,right)==WallMode::None);
  assert(detector.update(200,50)==WallMode::None); // Single close spike.
  assert(detector.update(200,190)==WallMode::None);
  // New uploaded log reported right+MPU at 134 and two walls at 129/38.
  // Neither is possible with the current retention threshold, even when acquired.
  detector.reset();
  for (int i=0;i<3;++i) detector.update(40,40);
  assert(detector.update(200,134)==WallMode::None);
  for (int i=0;i<3;++i) detector.update(40,40);
  assert(detector.update(129,38)==WallMode::RightOnly);
  const SingleWallSettings settings={40,50,3.6f,30,1,0.10f,5,1.5f,0.05f,0.08f,40};
  SingleWallController pid;
  float error,wall,mpu;
  auto evaluate=[&](WallMode mode,int left,int right,float yaw,float rate,int base) {
    pid.reset();
    return pid.update(mode,left,right,yaw,rate,base,settings,0.05f,error,wall,mpu);
  };
  auto commands=evaluate(WallMode::LeftOnly,40,200,0,0,70);
  assert(commands.right==70 && commands.left==70 && wall==0);
  commands=evaluate(WallMode::LeftOnly,25,200,0,0,70);
  assert(commands.right>30 && commands.right<70 && commands.left==70 && error==15 && wall>0);
  commands=evaluate(WallMode::RightOnly,200,25,0,0,70);
  assert(commands.left>=30 && commands.left<70 && commands.right==70 && error== -25);
  commands=evaluate(WallMode::LeftOnly,55,200,0,0,70);
  assert(commands.left<70 && commands.right==70);
  commands=evaluate(WallMode::RightOnly,200,55,0,0,70);
  assert(commands.right<70 && commands.left==70);
  commands=evaluate(WallMode::RightOnly,200,65,45,100,70);
  assert(commands.right<70 && commands.left==70 && mpu==0); // MPU cannot reverse wall PID.
  commands=evaluate(WallMode::RightOnly,200,53,0,0,70);
  assert(commands.right==70 && commands.left==70 && wall==0);
  commands=evaluate(WallMode::LeftOnly,40,200,5,0,70);
  assert(commands.left==65 && commands.right==70 && mpu== -5);
  commands=evaluate(WallMode::LeftOnly,40,200,0,10,70);
  assert(commands.left<commands.right);
  commands=evaluate(WallMode::LeftOnly,1,200,-20,0,35);
  assert(commands.right==30 && commands.left==35 && wall==5);
  // Sustained saturation must not accumulate a large integral.
  for(int i=0;i<1000;++i) pid.update(WallMode::LeftOnly,1,200,0,0,35,settings,0.05f,error,wall,mpu);
  pid.update(WallMode::LeftOnly,35,200,0,0,70,settings,0.05f,error,wall,mpu);
  pid.update(WallMode::LeftOnly,35,200,0,0,70,settings,0.05f,error,wall,mpu);
  assert(wall>7 && wall<8);
  // Integral increases correction for constant error; derivative damps an improving error.
  pid.reset();
  pid.update(WallMode::LeftOnly,30,200,0,0,70,settings,0.05f,error,wall,mpu);
  float initial=wall;
  for(int i=0;i<20;++i) pid.update(WallMode::LeftOnly,30,200,0,0,70,settings,0.05f,error,wall,mpu);
  assert(wall>initial);
  pid.update(WallMode::LeftOnly,32,200,0,0,70,settings,0.05f,error,wall,mpu);
  assert(wall<settings.wallKp*8);
  // Switching walls clears old integral and derivative history.
  pid.update(WallMode::RightOnly,200,60,0,0,70,settings,0.05f,error,wall,mpu);
  assert(fabsf(wall-initial)<0.0001f);
  pid.update(WallMode::RightOnly,200,50,0,0,70,settings,0.05f,error,wall,mpu);
  pid.update(WallMode::RightOnly,200,60,0,0,70,settings,0.05f,error,wall,mpu);
  assert(fabsf(wall-initial)<0.0001f); // Deadband resets PID too.
  commands=pid.update(WallMode::RightOnly,200,60,0,0,70,settings,0,error,wall,mpu);
  assert(commands.right==0 && commands.left==0);

  // Recorded left-wall sequence 50 -> 49 -> 45 mm must keep correcting
  // toward the wall instead of alternating between full correction and zero.
  const SingleWallSettings tuned={40,40,3.6f,30,2,0.15f,15,3,0,0.05f,40};
  pid.reset();
  commands=pid.update(WallMode::LeftOnly,50,200,0,0,140,tuned,0.022f,error,wall,mpu);
  assert(wall<0 && commands.left<140 && commands.right==140);
  commands=pid.update(WallMode::LeftOnly,49,200,0,0,140,tuned,0.022f,error,wall,mpu);
  assert(wall<0 && commands.left<140 && commands.right==140);
  commands=pid.update(WallMode::LeftOnly,45,200,0,0,140,tuned,0.022f,error,wall,mpu);
  assert(wall<0 && commands.left<140 && commands.right==140);

  auto old=forwardWallCommands(32,68,{70,30,40,50});
  assert(old.right==30 && old.left==70); // Case 1 unchanged.

  puts("PASS: wall modes, hysteresis, mirrored wall correction, heading/rate feedback and speed bounds");
}
