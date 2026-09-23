#include <cassert>
#include <deque>
#include <vector>
#include <string>
#include "../right-hand.ino"

unsigned long testClock = 0;
FakeSerial Serial;
struct Ranges { int front, left, right; };
std::deque<Ranges> readings;
std::vector<float> turns;
std::vector<std::string> messages;
MovementCommand nextCommand = MovementCommand::None;
bool stopped = false, ready = true, moveOk = true, turnOk = true;
bool stopInScan = false, stopInTurn = false;
bool wallArrival = false, stopAfterMove = false, headingReady = true;
float arrivalYaw = 0;
int moves = 0, scans = 0, stopAfterScan = 0, rotationStops = 0;

void moveForwardSetup() {}
bool beginRotation() { return true; }
void moveForwardStop() { stopped = true; }
void stopRotation() { ++rotationStops; }
bool demoMotionReady() { return ready; }
MovementCommand demoReadCommand() {
    MovementCommand command = nextCommand;
    nextCommand = MovementCommand::None;
    return command;
}
void demoBeginRun() { stopped = false; }
void demoEndRun() { stopped = true; }
bool demoStopped() { return stopped; }
void demoLog(const char *text) { messages.push_back(text); }
bool demoReadPaths(int &front, int &left, int &right) {
    if (stopInScan) { stopped = true; return false; }
    if (readings.empty()) return false;
    Ranges sample = readings.front(); readings.pop_front();
    front = sample.front; left = sample.left; right = sample.right;
    ++scans;
    if (scans == stopAfterScan) nextCommand = MovementCommand::Stop;
    return front >= 0 && left >= 0 && right >= 0;
}
DemoMoveResult demoMoveOneCell() {
    assert(!stopped); ++moves;
    if (stopAfterMove) nextCommand = MovementCommand::Stop;
    if (!moveOk) return DemoMoveResult::Failed;
    return wallArrival ? DemoMoveResult::FrontWallReached : DemoMoveResult::EncoderReached;
}
bool demoHeadingError(float &yaw) { yaw=arrivalYaw; return headingReady; }
bool turnDegrees(float angle) {
    assert(!stopped);
    turns.push_back(angle);
    if (stopInTurn) { stopped = true; return false; }
    return turnOk;
}

void reset() {
    testClock = 0;
    readings.clear(); turns.clear(); messages.clear();
    nextCommand = MovementCommand::None;
    stopped = stopInScan = stopInTurn = false;
    ready = moveOk = turnOk = rotationReady = demoRunning = true;
    moves = scans = stopAfterScan = rotationStops = 0;
    completedCells = 0;
    wallArrival = stopAfterMove = false;
    headingReady = true;
    arrivalYaw = 0;
}
void addScan(int front, int left, int right) {
    for (int i = 0; i < OPEN_CONFIRM_SAMPLES; ++i)
        readings.push_back({front, left, right});
}

int main() {
    assert(frontRangeStatusMeansOpen(2));
    assert(frontRangeStatusMeansOpen(4));
    assert(!frontRangeStatusMeansOpen(0));
    for (uint8_t status : {0, 2, 4, 6})
        assert(frontRangeStatusIsUsable(status));
    for (uint8_t status : {1, 3, 5, 7, 8, 9})
        assert(!frontRangeStatusIsUsable(status));

    // Either arrival cause keeps following after correcting heading in place.
    for (bool frontWall : {false,true}) {
        for (float yaw : {-5.0f,5.0f}) {
            reset(); wallArrival=frontWall; arrivalYaw=yaw;
            addScan(200,40,40); runOneCell();
            assert(turns==std::vector<float>{yaw});
            assert(moves==1 && completedCells==1 && demoRunning);
        }
    }
    reset(); wallArrival=true; arrivalYaw=-3.64f;
    addScan(200,40,40); runOneCell();
    assert(turns==std::vector<float>{-3.64f} && completedCells==1);
    // The next scan confirms three walls and performs the requested U-turn.
    arrivalYaw=0; wallArrival=false;
    addScan(30,67,29); addScan(200,40,40); runOneCell();
    assert((turns==std::vector<float>{-3.64f,-90,-90}));
    assert(moves==2 && completedCells==2 && demoRunning);

    reset(); wallArrival=true; arrivalYaw=0.5f;
    addScan(200,40,40); runOneCell();
    assert(turns.empty() && completedCells==1); // Already within tolerance.
    reset(); wallArrival=true; arrivalYaw=5; stopAfterMove=true;
    addScan(200,40,40); runOneCell();
    assert(turns.empty() && stopped && !demoRunning && completedCells==0);
    reset(); wallArrival=true; arrivalYaw=5; stopInTurn=true;
    addScan(200,40,40); runOneCell();
    assert(turns.size()==1 && stopped && !demoRunning && completedCells==0);
    reset(); wallArrival=true; arrivalYaw=5; turnOk=false;
    addScan(200,40,40); runOneCell();
    assert(stopped && !demoRunning && completedCells==0);
    reset(); wallArrival=true; headingReady=false;
    addScan(200,40,40); runOneCell();
    assert(turns.empty() && stopped && !demoRunning && completedCells==0);

    assert(FRONT_OPEN_MM == 100 && SIDE_OPEN_MM == 100);
    reset(); addScan(101, 100, 100); runOneCell();
    assert(turns.empty() && moves == 1); // Front >10 cm allows straight.
    reset(); addScan(100, 100, 101); addScan(101, 100, 100);
    runOneCell(); assert(turns == std::vector<float>{-90} && moves == 1);
    reset(); addScan(100, 101, 100); addScan(101, 100, 100);
    runOneCell(); assert(turns == std::vector<float>{90} && moves == 1);

    // Right wins even when all directions are open. The corrected physical
    // right is negative in the tested rotation API.
    reset(); addScan(250, 200, 190); addScan(250, 40, 40);
    runOneCell();
    assert(turns == std::vector<float>{-90});
    assert(moves == 1 && completedCells == 1 && demoRunning);

    reset(); addScan(250, 200, 40); runOneCell();
    assert(turns.empty() && moves == 1);

    reset(); addScan(70, 200, 40); addScan(250, 40, 40);
    runOneCell();
    assert(turns == std::vector<float>{90} && moves == 1);

    reset(); addScan(30, 40, 40); addScan(250, 40, 40);
    runOneCell();
    assert((turns == std::vector<float>{-90, -90}) && moves == 1);

    // Unsensed rear / selected opening must actually be clear after turning.
    reset(); addScan(70, 40, 40); addScan(70, 40, 40);
    runOneCell();
    assert(moves == 0 && stopped && !demoRunning);

    reset(); addScan(250, 40, 190); addScan(70, 40, 40);
    runOneCell();
    assert(moves == 0 && stopped);

    // Exact threshold is blocked; one transient side opening is insufficient.
    reset(); addScan(250, SIDE_OPEN_MM, SIDE_OPEN_MM); runOneCell();
    assert(turns.empty() && moves == 1);
    reset();
    readings.push_back({250, 40, 190});
    readings.push_back({250, 40, 40});
    readings.push_back({250, 40, 190});
    runOneCell();
    assert(turns.empty() && moves == 1);
    reset(); addScan(FRONT_OPEN_MM, 200, 40); addScan(250, 40, 40);
    runOneCell(); assert(turns == std::vector<float>{90});

    for (int invalidSide = 0; invalidSide < 3; ++invalidSide) {
        reset(); addScan(invalidSide == 0 ? -1 : 250,
                         invalidSide == 1 ? -1 : 40,
                         invalidSide == 2 ? -1 : 40);
        runOneCell();
        assert(stopped && !demoRunning && moves == 0 && turns.empty());
    }
    reset(); stopInScan = true; runOneCell();
    assert(stopped && moves == 0 && turns.empty());
    reset(); nextCommand = MovementCommand::Stop; runOneCell();
    assert(stopped && moves == 0 && turns.empty());
    reset(); addScan(250, 40, 40); stopAfterScan = OPEN_CONFIRM_SAMPLES;
    runOneCell(); assert(stopped && moves == 0);
    reset(); addScan(70, 40, 40); stopInTurn = true;
    runOneCell(); assert(turns.size() == 1 && moves == 0 && stopped);
    reset(); addScan(250, 40, 190); turnOk = false;
    runOneCell(); assert(moves == 0 && stopped && !demoRunning);
    reset(); addScan(250, 40, 40); moveOk = false;
    runOneCell(); assert(completedCells == 0 && stopped && !demoRunning);

    // Idle startup, explicit start, repeated cells, then stop without another move.
    reset(); demoRunning = false; setup();
    assert(!demoRunning && moves == 0 && stopped);
    addScan(250, 40, 40); nextCommand = MovementCommand::Start; loop();
    assert(demoRunning && moves == 1 && completedCells == 1);
    addScan(250, 40, 40); loop(); assert(moves == 2 && completedCells == 2);
    nextCommand = MovementCommand::Stop; loop();
    assert(!demoRunning && stopped && moves == 2 && rotationStops > 0);
    reset(); demoRunning = false; ready = false;
    nextCommand = MovementCommand::Start; loop();
    assert(!demoRunning && moves == 0);
    reset(); demoRunning = false; rotationReady = false;
    nextCommand = MovementCommand::Start; loop();
    assert(!demoRunning && moves == 0);
    puts("PASS: arrivals, yaw correction signs, front-wall recovery to U-turn, cancellation, faults and right-hand navigation");
}
