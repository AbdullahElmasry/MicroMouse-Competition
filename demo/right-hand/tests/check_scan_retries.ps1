$ErrorActionPreference = 'Stop'
$source = Get-Content -LiteralPath (Join-Path $PSScriptRoot '../MoveForward.cpp') -Raw
$begin = $source.IndexOf('bool demoReadPaths(')
$end = $source.IndexOf('DemoMoveResult demoMoveOneCell()', $begin)
if ($begin -lt 0 -or $end -lt 0) { throw 'Scan adapter not found' }
$harness = @'
#include <cassert>
#include <cstdio>
#include <string>
bool sensorsReady=true, motionStopRequested=false, sideCommunicationFault=false;
const char *tofFailure="FRONT range rejected";
int tofFailureCode=2, calls=0, waits=0, scenario=0;
std::string logs;
struct Filter { void reset() {} } frontFilter, leftFilter, rightFilter;
void drive(int l, int r) { assert(l==0 && r==0); }
void handleWiFiClient() {}
void serviceMotionSensors() {}
void debugPrintln(const char *line) { logs+=line; logs+='\n'; }
void waitWithMotionService(unsigned long ms) {
    assert(ms==50); ++waits;
    if (scenario==5) motionStopRequested=true;
}
bool observe(int &f,int &l,int &r,int &raw,bool stoppedScan) {
    assert(stoppedScan); ++calls; f=200; l=40; r=raw=190;
    sideCommunicationFault=false;
    if (scenario==4) motionStopRequested=true;
    if (scenario==3 && calls==1) sideCommunicationFault=true;
    if (scenario==7) l=-1;
    return scenario!=2 && scenario!=5 && !(scenario==1 && calls==1);
}
'@
$checks = @'
int main() {
    int f,l,r;
    for (scenario=0;scenario<=7;++scenario) {
        calls=waits=0; logs.clear(); motionStopRequested=scenario==6;
        bool ok=demoReadPaths(f,l,r);
        assert(ok==(scenario==0 || scenario==1 || scenario==3));
        if (scenario==0) assert(calls==1 && waits==0);
        if (scenario==1 || scenario==3) assert(calls==2 && waits==1);
        if (scenario==2 || scenario==7) assert(calls==3 && waits==2);
        if (scenario==4) assert(calls==1 && waits==0);
        if (scenario==5) assert(calls==1 && waits==1);
        if (scenario==6) assert(calls==0);
        if (scenario==2) {
            assert(logs.find("SCAN ERROR | attempt=3/3")!=std::string::npos);
            assert(logs.find("FRONT range rejected | code=2")!=std::string::npos);
        }
    }
    sensorsReady=false; calls=0;
    assert(!demoReadPaths(f,l,r) && calls==0);
    puts("PASS: actual stopped-scan retry adapter, recovery, bounded failure, invalid samples and stop cancellation");
}
'@
$build = Join-Path ([IO.Path]::GetTempPath()) ('scan-retry-test-' + [guid]::NewGuid().ToString('N'))
[IO.Directory]::CreateDirectory($build) | Out-Null
$cpp = Join-Path $build 'scan.cpp'
$exe = Join-Path $build 'scan.exe'
[IO.File]::WriteAllText($cpp,$harness + "`n" + $source.Substring($begin,$end-$begin) + "`n" + $checks)
& g++ -static -std=c++11 -Wall -Wextra -Werror $cpp -o $exe
if ($LASTEXITCODE -ne 0) { throw 'Scan test compilation failed' }
& $exe
if ($LASTEXITCODE -ne 0) { throw 'Scan test failed' }
