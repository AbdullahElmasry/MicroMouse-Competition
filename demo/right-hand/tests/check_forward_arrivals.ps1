$ErrorActionPreference = 'Stop'
$demoPath = Split-Path $PSScriptRoot -Parent
$source = Get-Content -LiteralPath (Join-Path $demoPath 'MoveForward.cpp') -Raw
$begin = $source.IndexOf('DemoMoveResult runForwardDistance()')
$end = $source.IndexOf('void runMove()', $begin)
if ($begin -lt 0 -or $end -lt 0) { throw 'Forward function not found' }
$harness = @'
#include "DemoMotion.h"
#include "config.h"
#include <cassert>
#include <cstdio>
#include <string>
std::string logs;
constexpr int CELL_LENGTH_MM=180, FORWARD_SPEED=140;
constexpr int FRONT_EMERGENCY_STOP_MM=FRONT_WALL_STOP_TRIGGER_MM;
constexpr float LEFT_TICKS_PER_CELL=616.3f, RIGHT_TICKS_PER_CELL=617.36f;
constexpr unsigned long LEFT_TARGET_TICKS=616, RIGHT_TARGET_TICKS=617, BRAKE_LEAD_TICKS=10;
constexpr unsigned long MOVE_TIMEOUT_MS=105000, STALL_TIMEOUT_MS=1500;
constexpr int NO_WALL_MIN_BASE_PWM=100, NO_WALL_MIN_MOTOR_PWM=85;
constexpr int MIN_APPROACH_PWM=100;
constexpr float MPU_YAW_SIGN=-1, NO_WALL_MPU_HEADING_KP=10, NO_WALL_MPU_RATE_KD=0.5f;
constexpr float NO_WALL_MPU_MAX_PWM=25, NO_WALL_MPU_WEIGHT=0.8f;
const TwoWallPidSettings TWO_WALL_PID_SETTINGS={3.6f,85,7.3f,0,5.7f,40};
const SingleWallSettings SINGLE_WALL_SETTINGS={40,40,3.6f,85,2,0.15f,15,6,0,2.5f,40};
const NoWallSettings NO_WALL_SETTINGS={LEFT_TICKS_PER_CELL,RIGHT_TICKS_PER_CELL,85,{1.7f,0.1f,0,30}};
CellApproachController approachPid({0.24f,0.01f,0.01f,300,140,100});
TwoWallPidController twoWallPid;
SingleWallController singleWallPid;
NoWallController noWallPid;
WallModeDetector wallDetector;
bool sensorsReady=true, motionStopRequested=false, sideCommunicationFault=false;
int scenario=0, leftCommand=0, rightCommand=0;
unsigned long clockMs=0, leftTicks=0, rightTicks=0;
struct Mpu {
    bool healthy() const { return true; }
    float yaw() const { return -3.64f; }
    float rate() const { return 0; }
} mpuYaw;
unsigned long millis() { return clockMs; }
void handleWiFiClient() {}
void waitWithMotionService(unsigned long ms) {
    clockMs+=ms;
    if (scenario==18 && leftTicks>=606) {
        if (leftCommand) ++leftTicks;
        if (rightCommand) ++rightTicks;
    } else if (scenario!=6 && !(scenario==17 && leftTicks>=650)) {
        leftTicks+=leftCommand*ms/20; rightTicks+=rightCommand*ms/20;
    }
    if (scenario==7 && ms==100) motionStopRequested=true;
}
void readTicks(unsigned long &l,unsigned long &r) { l=leftTicks; r=rightTicks; }
void drive(int l,int r) { assert((l==0 && r==0) || (l>=85 && r>=85)); leftCommand=l; rightCommand=r; }
template<class T> void debugPrintln(const T &text) { logs+=text; logs+='\n'; }
MovementCommand readCommand() {
    return scenario==4 || (scenario==16 && leftTicks>=650) ? MovementCommand::Stop : MovementCommand::None;
}
bool observe(int &front,int &left,int &right,int &raw) {
    waitWithMotionService(10);
    front=200; left=40; right=40; raw=50;
    if (scenario==8) right=200;
    if (scenario==9) left=200;
    if (scenario==10) left=right=200;
    if (scenario==1 && leftTicks>100) front=61;
    if (scenario==2 || scenario==3 || scenario==5 || scenario==7) front=60;
    if (scenario==5) sideCommunicationFault=true;
    if (scenario>=11) front=90;
    if (scenario==12) front=100;
    if ((scenario==11 || scenario==12) && leftTicks>=700) front=61;
    if (scenario==13) front=101;
    if (scenario==15 && leftTicks>=650) return false;
    return scenario!=3; // An invalid near reading is a fault, never an arrival.
}
'@
$checks = @'
int main() {
    static_assert(FRONT_WALL_TARGET_MM==60,"target");
    static_assert(FRONT_WALL_STOP_TRIGGER_MM==61,"integer upper band");
    static_assert(FRONT_WALL_MIN_IN_BAND_MM==59,"integer lower band");
    for (scenario=0; scenario<=18; ++scenario) {
        logs.clear();
        clockMs=leftTicks=rightTicks=0; leftCommand=rightCommand=0;
        motionStopRequested=sideCommunicationFault=false;
        wallDetector.reset();
        DemoMoveResult result=runForwardDistance();
        assert(leftCommand==0 && rightCommand==0);
        if (scenario==0 || (scenario>=8 && scenario<=10) || scenario==13)
            assert(result==DemoMoveResult::EncoderReached);
        if (scenario==1 || scenario==2) assert(result==DemoMoveResult::FrontWallReached);
        if (scenario==3 || scenario==5 || scenario==6) assert(result==DemoMoveResult::Failed);
        if (scenario==4 || scenario==7) assert(result==DemoMoveResult::Stopped);
        if (scenario==1) assert(leftTicks<LEFT_TARGET_TICKS-BRAKE_LEAD_TICKS);
        if (scenario==11 || scenario==12) {
            assert(result==DemoMoveResult::FrontWallReached && leftTicks>=700);
            assert(logs.find("WALL APPROACH")!=std::string::npos);
            assert(logs.find("target=60mm +/-3.0%")!=std::string::npos);
            assert(logs.find("front at brake=61mm | IN BAND")!=std::string::npos);
        }
        if (scenario==13) assert(logs.find("WALL APPROACH")==std::string::npos);
        if (scenario==14 || scenario==15 || scenario==17 || scenario==18)
            assert(result==DemoMoveResult::Failed);
        if (scenario==14 || scenario==18)
            assert(logs.find("FRONT WALL APPROACH LIMIT")!=std::string::npos);
        if (scenario==16) assert(result==DemoMoveResult::Stopped);
        if (scenario==17) assert(logs.find("ENCODER STALL")!=std::string::npos);
    }
    puts("PASS: front-wall reference beyond encoder endpoint, open-path finish, approach bounds, faults, stalls and manual stops");
}
'@
$build = Join-Path ([IO.Path]::GetTempPath()) ('forward-arrival-tests-' + [guid]::NewGuid().ToString('N'))
[IO.Directory]::CreateDirectory($build) | Out-Null
$cpp = Join-Path $build 'arrival.cpp'
$exe = Join-Path $build 'arrival.exe'
[IO.File]::WriteAllText($cpp,$harness + "`n" + $source.Substring($begin,$end-$begin) + "`n" + $checks)
& g++ -static -std=c++11 -Wall -Wextra -Werror "-I$demoPath" $cpp -o $exe
if ($LASTEXITCODE -ne 0) { throw 'Forward arrival test compilation failed' }
& $exe
if ($LASTEXITCODE -ne 0) { throw 'Forward arrival tests failed' }
