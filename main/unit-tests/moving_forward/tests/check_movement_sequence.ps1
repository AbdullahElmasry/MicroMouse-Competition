$ErrorActionPreference = 'Stop'
$sketchDirectory = Split-Path $PSScriptRoot -Parent
$source = Get-Content -LiteralPath (Join-Path $sketchDirectory 'moving_forward.ino') -Raw
$begin = $source.IndexOf('bool runOneCell(')
$end = $source.IndexOf('void setup()', $begin)
if ($begin -lt 0 -or $end -lt 0) { throw 'Movement functions not found' }
$functions = $source.Substring($begin, $end - $begin)
$harness = @'
#include "ForwardWallControl.h"
#include "CellApproachControl.h"
#include "MovementCommands.h"
#include <assert.h>
#include <stdio.h>
constexpr unsigned long LEFT_TARGET_TICKS=623, RIGHT_TARGET_TICKS=619;
constexpr unsigned long MOVE_TIMEOUT_MS=15000, STALL_TIMEOUT_MS=1500, CELL_PAUSE_MS=500;
constexpr int FORWARD_CELLS=7;
constexpr ForwardWallSettings WALL_SETTINGS={70,30,40,50};
constexpr CellApproachSettings APPROACH_SETTINGS={0.24f,0.01f,0.01f,300,70,35};
constexpr unsigned long BRAKE_LEAD_TICKS=10;
CellApproachController approachPid(APPROACH_SETTINGS);
bool sensorsReady=true;
unsigned long clockMs=0, leftTicks=0, rightTicks=0, lastStopMs=0, moveStartMs=0;
int leftCommand=0, rightCommand=0, starts=0, stops=0, scenario=0;
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
void readTicks(unsigned long &left, unsigned long &right) { left=leftTicks; right=rightTicks; }
void drive(int left, int right) {
  assert((left==0)==(right==0)); // Never finish a cell by pivoting one wheel.
  if (!leftCommand && left) {
    if (starts) assert(clockMs-lastStopMs >= 500);
    ++starts; moveStartMs=clockMs;
  }
  if (leftCommand && !left) { ++stops; lastStopMs=clockMs; }
  leftCommand=left; rightCommand=right;
}
MovementCommand readCommand() {
  if (scenario==1 && leftCommand && clockMs-moveStartMs >= 30) return MovementCommand::Stop;
  if (scenario==2 && stops==1 && !leftCommand && clockMs-lastStopMs >= 150) return MovementCommand::Stop;
  return MovementCommand::None;
}
bool observe(int &front, int &left, int &right) {
  delay(10);
  front=130; left=44; right=55;
  if (scenario==3 && stops>=1) { left=-1; right=-1; }
  if (scenario==4 && starts>=1) return false;
  return true;
}
'@
$checks = @'
int main() {
  for (scenario=0; scenario<=4; ++scenario) {
    clockMs=leftTicks=rightTicks=lastStopMs=moveStartMs=0;
    leftCommand=rightCommand=starts=stops=0;
    runMove();
    assert(leftCommand==0 && rightCommand==0);
    if (scenario==0) assert(starts==7 && stops==7);
    else assert(starts==1 && stops==1);
  }
  puts("PASS: seven cells, >=500 ms pauses, joint braking, stop during movement/pause, sensor fault cancellation");
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
