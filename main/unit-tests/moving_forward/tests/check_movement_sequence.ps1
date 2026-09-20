$ErrorActionPreference = 'Stop'
$sketchDirectory = Split-Path $PSScriptRoot -Parent
$source = Get-Content -LiteralPath (Join-Path $sketchDirectory 'moving_forward.ino') -Raw
$begin = $source.IndexOf('bool runForwardDistance(')
$end = $source.IndexOf('void setup()', $begin)
if ($begin -lt 0 -or $end -lt 0) { throw 'Movement functions not found' }
$functions = $source.Substring($begin, $end - $begin)
$harness = @'
#include "ForwardWallControl.h"
#include "CellApproachControl.h"
#include "MovementCommands.h"
#include "SingleWallControl.h"
#include "NoWallControl.h"
#include <assert.h>
#include <stdio.h>
constexpr unsigned long LEFT_TARGET_TICKS=4359, RIGHT_TARGET_TICKS=4335;
constexpr unsigned long MOVE_TIMEOUT_MS=105000, STALL_TIMEOUT_MS=1500;
constexpr int FRONT_EMERGENCY_STOP_MM=40;
constexpr ForwardWallSettings WALL_SETTINGS={70,30,40,40};
constexpr CellApproachSettings APPROACH_SETTINGS={0.24f,0.01f,0.01f,300,70,35};
constexpr unsigned long BRAKE_LEAD_TICKS=10;
CellApproachController approachPid(APPROACH_SETTINGS);
const SingleWallSettings SINGLE_WALL_SETTINGS={40,40,3.6f,30,1,0.10f,5,1.5f,0.05f,0.08f,40};
SingleWallController singleWallPid;
const NoWallSettings NO_WALL_SETTINGS={4359.0f/7.0f,4334.5f/7.0f,30,{1,0.10f,0,40}};
NoWallController noWallPid;
float MPU_YAW_SIGN=1;
WallModeDetector wallDetector;
bool motionStopRequested=false, sideCommunicationFault=false;
bool sensorsReady=true;
unsigned long clockMs=0, leftTicks=0, rightTicks=0, lastStopMs=0, moveStartMs=0;
int leftCommand=0, rightCommand=0, starts=0, stops=0, scenario=0;
struct FakeMpu {
  void reset() {}
  bool healthy() const { return scenario!=8 && scenario!=17; }
  float yaw() const { return 0; }
  float rate() const { return 0; }
} mpuYaw;
unsigned long millis() { return clockMs; }
void delay(unsigned long ms) {
  clockMs += ms;
  if (leftCommand) leftTicks += ms * leftCommand / 20;
  if (rightCommand) rightTicks += ms * rightCommand / 20;
}
template<class T> void debugPrint(const T&) {}
template<class T> void debugPrintln(const T&) {}
void debugPrintln() {}
void handleWiFiClient() {}
void serviceMotionSensors() {}
void waitWithMotionService(unsigned long ms) { delay(ms); }
void readTicks(unsigned long &left, unsigned long &right) { left=leftTicks; right=rightTicks; }
void drive(int left, int right) {
  assert((left==0)==(right==0)); // Never finish a cell by pivoting one wheel.
  if (!leftCommand && left) {
    assert(starts==0); // No intermediate stops or restart.
    ++starts; moveStartMs=clockMs;
  }
  if (leftCommand && !left) { ++stops; lastStopMs=clockMs; }
  leftCommand=left; rightCommand=right;
}
MovementCommand readCommand() {
  if (scenario==1 && leftCommand && clockMs-moveStartMs >= 30) return MovementCommand::Stop;
  if (scenario==2 && leftCommand && leftTicks>=100) return MovementCommand::Stop;
  return MovementCommand::None;
}
bool observe(int &front, int &left, int &right) {
  delay(10);
  front=130; left=44; right=55;
  if (scenario==11) front=40;
  if (scenario==12) front=39;
  if (scenario==13) front=41;
  if (scenario==14 && leftTicks>=100) front=40;
  if (scenario==15) { left=right=leftTicks<1000?200:45; } // Case 3 to case 1.
  if (scenario==16) { left=right=leftTicks<1000?45:200; } // Case 1 to case 3.
  if (scenario==17) { left=right=200; } // Case 3 does not depend on MPU health.
  if (scenario==3 && leftTicks>=100) { left=-1; right=-1; }
  if (scenario==4 && starts>=1) return false;
  if (scenario==5 || scenario==8 || scenario==9) { left=40; right=200; }
  if (scenario==6) { left=200; right=40; }
  if (scenario==7) { left=200; right=200; }
  if (scenario==10 && (leftTicks/600)%2==1) { left=40; right=-1; }
  return true;
}
'@
$checks = @'
int main() {
  for (scenario=0; scenario<=17; ++scenario) {
    clockMs=leftTicks=rightTicks=lastStopMs=moveStartMs=0;
    leftCommand=rightCommand=starts=stops=0;
    MPU_YAW_SIGN=scenario==9?0:1;
    runMove();
    assert(leftCommand==0 && rightCommand==0);
    if (scenario==0 || scenario==5 || scenario==6 || scenario==10 || scenario==13 || scenario==15 || scenario==16 || scenario==17) { assert(starts==1 && stops==1); assert(leftTicks>=LEFT_TARGET_TICKS-BRAKE_LEAD_TICKS || rightTicks>=RIGHT_TARGET_TICKS-BRAKE_LEAD_TICKS); }
    else if (scenario==8 || scenario==9 || scenario==11 || scenario==12) assert(starts==0 && stops==0);
    else if (scenario==7) assert(starts==1 && stops==1);
    else assert(starts==1 && stops==1);
  }
  puts("PASS: cases 1/2/3, transitions, encoder-only case 3, continuous seven cells, front brake, stop/fault cancellation");
}
'@
$testDirectory = Join-Path ([System.IO.Path]::GetTempPath()) ('micromouse-sequence-' + [guid]::NewGuid().ToString('N'))
[System.IO.Directory]::CreateDirectory($testDirectory) | Out-Null
$testSource = Join-Path $testDirectory 'sequence_test.cpp'
$testExecutable = Join-Path $testDirectory 'sequence_test.exe'
[System.IO.File]::WriteAllText($testSource, $harness + "`n" + $functions + "`n" + $checks)
& g++ -std=c++11 -Wall -Wextra -Werror "-I$sketchDirectory" $testSource -o $testExecutable
if ($LASTEXITCODE -ne 0) { throw 'Sequence test compilation failed' }
& $testExecutable
if ($LASTEXITCODE -ne 0) { throw 'Sequence tests failed' }
