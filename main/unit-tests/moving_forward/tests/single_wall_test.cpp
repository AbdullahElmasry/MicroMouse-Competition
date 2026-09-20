#include "../SingleWallControl.h"
#include "../YawEstimate.h"
#include <assert.h>
#include <stdio.h>

int main() {
  WallModeDetector detector;
  assert(detector.update(45,55)==WallMode::Two);
  assert(detector.update(45,130)==WallMode::Two); // Hysteresis retains detected wall.
  assert(detector.update(45,140)==WallMode::LeftOnly);
  assert(detector.update(45,130)==WallMode::LeftOnly);
  assert(detector.update(45,110)==WallMode::Two);
  assert(detector.update(-1,45)==WallMode::RightOnly);
  assert(detector.update(-1,200)==WallMode::None);
  detector.reset();
  assert(detector.update(125,125)==WallMode::None);
  const SingleWallSettings settings={40,50,3.6f,30,1,0.10f,5,1.5f,0.05f,0.08f,40};
  SingleWallController pid;
  float error,wall,mpu;
  auto evaluate=[&](WallMode mode,int left,int right,float yaw,float rate,int base) {
    pid.reset();
    return pid.update(mode,left,right,yaw,rate,base,settings,0.05f,error,wall,mpu);
  };
  auto commands=evaluate(WallMode::LeftOnly,40,200,0,0,70);
  assert(commands.left==70 && commands.right==70 && wall==0);
  commands=evaluate(WallMode::LeftOnly,25,200,0,0,70);
  assert(commands.left>30 && commands.left<70 && commands.right==70 && error==15 && wall>0);
  commands=evaluate(WallMode::RightOnly,200,25,0,0,70);
  assert(commands.right>=30 && commands.right<70 && commands.left==70 && error== -25);
  commands=evaluate(WallMode::LeftOnly,55,200,0,0,70);
  assert(commands.right<70 && commands.left==70);
  commands=evaluate(WallMode::RightOnly,200,55,0,0,70);
  assert(commands.left<70 && commands.right==70);
  commands=evaluate(WallMode::RightOnly,200,65,45,100,70);
  assert(commands.left<70 && commands.right==70 && mpu==0); // MPU cannot reverse wall PID.
  commands=evaluate(WallMode::RightOnly,200,53,0,0,70);
  assert(commands.left==70 && commands.right==70 && wall==0);
  commands=evaluate(WallMode::LeftOnly,40,200,5,0,70);
  assert(commands.right==65 && commands.left==70 && mpu== -5);
  commands=evaluate(WallMode::LeftOnly,40,200,0,10,70);
  assert(commands.right<commands.left);
  commands=evaluate(WallMode::LeftOnly,1,200,-20,0,35);
  assert(commands.left==30 && commands.right==35 && wall==5);
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
  assert(commands.left==0 && commands.right==0);
  auto old=forwardWallCommands(32,68,{70,30,40,50});
  assert(old.left==30 && old.right==70); // Case 1 unchanged.

  YawEstimate yaw;
  yaw.reset(0);
  assert(yaw.update(0.4f,10) && yaw.yaw()==0); // Tested deadband.
  yaw.reset(0);
  assert(yaw.update(10,10));
  assert(fabsf(yaw.rate()-5)<0.00001f && fabsf(yaw.yaw()-0.05f)<0.00001f);
  for (int t=20;t<=1000;t+=10) assert(yaw.update(10,t));
  assert(yaw.yaw()>8 && yaw.yaw()<10);
  assert(!yaw.healthy(1101));
  assert(!yaw.update(10,1200)); // A stale integration is never silently resumed.
  assert(!yaw.update(10,1210));
  yaw.reset(1220);
  assert(yaw.healthy(1220) && yaw.yaw()==0);
  assert(!yaw.update(NAN,1230));
  puts("PASS: wall modes, hysteresis, mirrored wall correction, heading/rate feedback, speed bounds, yaw baseline and faults");
}
