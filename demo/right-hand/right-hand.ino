#include <Arduino.h>
#include "DemoMotion.h"
#include "rotation.h"
#include "config.h"

bool rotationReady = false;
bool demoRunning = false;
unsigned long completedCells = 0;

bool stopRequested() {
    if (demoReadCommand() == MovementCommand::Stop) moveForwardStop();
    return demoStopped();
}

bool settle() {
    unsigned long started = millis();
    do {
        if (stopRequested()) return false;
        delay(1);
    } while (millis() - started < SETTLE_MS);
    return true;
}

void stopDemo(const char *reason) {
    stopRotation();
    demoEndRun(); // Finish with the forward controller's both-wheel brake.
    demoRunning = false;
    demoLog(reason);
    demoLog("Stopped. Send s/start for a new run.");
}

bool readOpenPaths(bool &frontOpen, bool &leftOpen, bool &rightOpen) {
    frontOpen = leftOpen = rightOpen = true;
    int minFront = 32767, minLeft = 32767, minRight = 32767;
    for (int i = 0; i < OPEN_CONFIRM_SAMPLES; ++i) {
        if (stopRequested()) return false;
        int front, left, right;
        if (!demoReadPaths(front, left, right)) return false;
        // An opening must be present in all three fresh stationary samples.
        frontOpen = frontOpen && front > FRONT_OPEN_MM;
        leftOpen = leftOpen && left > SIDE_OPEN_MM;
        rightOpen = rightOpen && right > SIDE_OPEN_MM;
        if (front < minFront) minFront = front;
        if (left < minLeft) minLeft = left;
        if (right < minRight) minRight = right;
    }
    char line[160];
    snprintf(line, sizeof(line),
        "PATHS | front=%s (%dmm) | left=%s (%dmm) | right=%s (%dmm)",
        frontOpen ? "OPEN" : "WALL", minFront,
        leftOpen ? "OPEN" : "WALL", minLeft,
        rightOpen ? "OPEN" : "WALL", minRight);
    demoLog(line);
    return !stopRequested();
}

bool turnAndSettle(float degrees) {
    if (stopRequested() || !turnDegrees(degrees)) return false;
    return settle();
}

bool alignArrivalHeading() {
    if (!settle()) return false;
    float yawRight;
    if (!demoHeadingError(yawRight)) {
        demoLog("ALIGN | MPU unavailable; cannot correct heading");
        return false;
    }
    char line[100];
    if (fabsf(yawRight) <= ARRIVAL_YAW_TOLERANCE_DEG) {
        snprintf(line,sizeof(line),"ALIGN | yaw=%+.2fdeg | within tolerance",yawRight);
        demoLog(line);
        return !stopRequested();
    }
    snprintf(line,sizeof(line),"ALIGN | yaw=%+.2fdeg | rotate in place to cell heading",yawRight);
    demoLog(line);
    // Forward yaw is right-positive; rotation accepts left-positive angles.
    // A +5 degree right drift therefore needs turnDegrees(+5), a left turn.
    return turnAndSettle(yawRight);
}

// Called once from the starting cell, then only after a completed cell.
// Forward wall sensing adjusts steering; it never chooses the next path.
void runOneCell() {
    if (!settle()) { stopDemo("Stop requested."); return; }
    char cellLine[64];
    snprintf(cellLine, sizeof(cellLine), "CELL %lu | scanning", completedCells + 1);
    demoLog(cellLine);
    bool frontOpen, leftOpen, rightOpen;
    if (!readOpenPaths(frontOpen, leftOpen, rightOpen)) {
        stopDemo(demoStopped() ? "Scan cancelled by stop command." :
                 "Scan failed after retries; see SCAN ERROR above.");
        return;
    }

    bool turned = rightOpen || !frontOpen;
    if (rightOpen) {
        demoLog("DECISION | RIGHT");
        if (!turnAndSettle(-90)) { stopDemo("Right turn stopped or failed."); return; }
    } else if (frontOpen) {
        demoLog("DECISION | STRAIGHT");
    } else if (leftOpen) {
        demoLog("DECISION | LEFT");
        if (!turnAndSettle(90)) { stopDemo("Left turn stopped or failed."); return; }
    } else {
        demoLog("DECISION | U-TURN (dead end)");
        if (!turnAndSettle(-90) || !turnAndSettle(-90)) {
            stopDemo("U-turn stopped or failed.");
            return;
        }
    }

    if (turned) {
        demoLog("CHECK | path after turn");
        // Verify the newly facing cell, including the unsensed rear at a dead end.
        if (!readOpenPaths(frontOpen, leftOpen, rightOpen) || !frontOpen) {
            stopDemo("Chosen path blocked or scan failed after turn.");
            return;
        }
    }
    if (stopRequested()) { stopDemo("Stop requested."); return; }
    DemoMoveResult outcome = demoMoveOneCell();
    if (outcome == DemoMoveResult::Stopped || outcome == DemoMoveResult::Failed) {
        stopDemo(outcome == DemoMoveResult::Stopped ? "Move cancelled by stop command." :
                 "Movement fault; see MOVE END above.");
        return;
    }
    demoLog(outcome == DemoMoveResult::FrontWallReached ?
        "ARRIVAL | front-wall brake; counted as cell reached" :
        "ARRIVAL | encoder target reached");
    if (!alignArrivalHeading()) {
        stopDemo("Heading correction stopped or failed.");
        return;
    }
    ++completedCells;
    char line[80];
    snprintf(line, sizeof(line), "CELL %lu | reached and aligned; ready for next decision", completedCells);
    demoLog(line);
}

void setup() {
    // Initialize rotation before ToF setup so it cannot reinitialize the bus
    // after the ToF addresses are assigned. Both calibrations happen at rest.
    Serial.begin(115200);
    rotationReady = beginRotation();
    moveForwardSetup();
    demoEndRun();
    demoLog("Right-hand demo: right > straight > left > U-turn; one cell per move.");
    demoLog(rotationReady && demoMotionReady()
        ? "Ready. Send s/start to run; d stops."
        : "Initialization failed. Keep the robot still and reset to retry.");
}

void loop() {
    MovementCommand command = demoReadCommand();
    if (command == MovementCommand::Stop) {
        stopDemo("Stop requested.");
    } else if (command == MovementCommand::Start && !demoRunning) {
        if (!rotationReady || !demoMotionReady()) {
            demoLog("REFUSED: rotation, MPU or ToF is unavailable. Reset to retry.");
        } else {
            completedCells = 0;
            char settings[120];
            snprintf(settings, sizeof(settings),
                "START | openings: front>%dmm, side>%dmm | samples=%d | settle=%lums",
                FRONT_OPEN_MM, SIDE_OPEN_MM, OPEN_CONFIRM_SAMPLES, SETTLE_MS);
            demoLog(settings);
            demoBeginRun();
            demoRunning = true;
        }
    }
    if (demoRunning) runOneCell();
    delay(10);
}
