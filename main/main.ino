#include <Arduino.h>
#include <WiFi.h>

#include "DemoMotion.h"
#include "MazeMap.h"
#include "WebDashboard.h"
#include "config.h"
#include "rotation.h"

#ifndef MAIN_UNIT_TEST_MODE
#define MAIN_UNIT_TEST_MODE 0  // Set to 1 to run UNIT_TEST_PATTERN.
#endif

MazeMap maze;
bool rotationReady = false;
bool explorerRunning = false;
#if MAIN_UNIT_TEST_MODE
enum class UnitTestAction { Forward, Right, Left };  // Available actions; do not repeat names here.

// Edit this list to choose the test path. Forward moves one cell; turns are 90 degrees.
constexpr UnitTestAction UNIT_TEST_PATTERN[] = {
    UnitTestAction::Forward,
    UnitTestAction::Forward,
    UnitTestAction::Forward,
    UnitTestAction::Forward,
};
constexpr int UNIT_TEST_STEP_COUNT =
    sizeof(UNIT_TEST_PATTERN) / sizeof(UNIT_TEST_PATTERN[0]);

bool unitTestRunning = false;
int unitTestStep = 0;
#endif
unsigned long completedCells = 0;
int robotX = 0;
int robotY = 0;
Direction robotHeading = Direction::North;

void publishState(const char *status) {
  dashboardUpdateMap(
      maze, robotX, robotY, robotHeading, explorerRunning, status);
}

void logPose(const char *event) {
  const MazeCell &cell = maze.cell(robotX, robotY);
  char line[180];
  snprintf(line, sizeof(line),
           "POSE | %s | cell=(%d,%d) | heading=%s | flood=%u | moves=%lu",
           event, robotX, robotY, MazeMap::directionName(robotHeading),
           cell.distance, completedCells);
  demoLog(line);
}

bool stopRequested() {
  if (demoReadCommand() == MovementCommand::Stop) moveForwardStop();
  return demoStopped();
}

bool settle() {
  const unsigned long started = millis();
  do {
    if (stopRequested()) return false;
    delay(1);
  } while (millis() - started < SETTLE_MS);
  return true;
}

void stopExplorer(const char *reason) {
  stopRotation();
  demoEndRun();
  explorerRunning = false;
  demoLog(reason);
  logPose("STOPPED");
  publishState(reason);
}

void resetMap() {
  maze.reset();
  robotX = 0;
  robotY = 0;
  robotHeading = Direction::North;
  completedCells = 0;
  demoLog("MAP RESET | start=(0,0) | heading=NORTH | goal=(7,7)-(8,8)");
  logPose("RESET");
  publishState("Ready");
}

bool readOpenPaths(bool &frontOpen, bool &leftOpen, bool &rightOpen,
                   int &frontMm, int &leftMm, int &rightMm) {
  frontOpen = leftOpen = rightOpen = true;
  frontMm = leftMm = rightMm = 32767;
  for (int sample = 0; sample < OPEN_CONFIRM_SAMPLES; ++sample) {
    if (stopRequested()) return false;
    int front, left, right;
    if (!demoReadPaths(front, left, right)) return false;
    frontOpen = frontOpen && front > FRONT_OPEN_MM;
    leftOpen = leftOpen && left > SIDE_OPEN_MM;
    rightOpen = rightOpen && right > SIDE_OPEN_MM;
    if (front < frontMm) frontMm = front;
    if (left < leftMm) leftMm = left;
    if (right < rightMm) rightMm = right;
  }
  char line[180];
  snprintf(line, sizeof(line),
           "SENSORS | F=%s %dmm | L=%s %dmm | R=%s %dmm",
           frontOpen ? "OPEN" : "WALL", frontMm,
           leftOpen ? "OPEN" : "WALL", leftMm,
           rightOpen ? "OPEN" : "WALL", rightMm);
  demoLog(line);
  return !stopRequested();
}

void updateMapFromScan(bool frontOpen, bool leftOpen, bool rightOpen) {
  const int conflicts = maze.observe(
      robotX, robotY, robotHeading, !frontOpen, !leftOpen, !rightOpen);
  maze.floodFill();
  char line[180];
  snprintf(line, sizeof(line),
           "MAP | cell=(%d,%d) heading=%s | walls F/L/R=%d/%d/%d | conflicts=%d",
           robotX, robotY, MazeMap::directionName(robotHeading),
           !frontOpen, !leftOpen, !rightOpen, conflicts);
  demoLog(line);
  if (conflicts) {
    demoLog("MAP WARNING | sensor disagrees with known edge; traversed openings stay open");
  }
  publishState("Mapping");
}

bool turnAndSettle(float degrees) {
  if (stopRequested() || !turnDegrees(degrees)) return false;
  return settle();
}

bool turnTo(Direction target) {
  const int change = ((int)target - (int)robotHeading + 4) % 4;
  char line[120];
  snprintf(line, sizeof(line), "TURN PLAN | %s -> %s",
           MazeMap::directionName(robotHeading),
           MazeMap::directionName(target));
  demoLog(line);

  bool ok = true;
  if (change == 1) ok = turnAndSettle(-90.0f);
  else if (change == 3) ok = turnAndSettle(90.0f);
  else if (change == 2)
    ok = turnAndSettle(-90.0f) && turnAndSettle(-90.0f);
  if (ok) robotHeading = target;
  return ok;
}

bool alignArrivalHeading() {
  if (!settle()) return false;
  float yawRight;
  if (!demoHeadingError(yawRight)) {
    demoLog("ALIGN | MPU unavailable; cannot correct heading");
    return false;
  }
  char line[120];
  if (fabsf(yawRight) <= ARRIVAL_YAW_TOLERANCE_DEG) {
    snprintf(line, sizeof(line), "ALIGN | yaw=%+.2fdeg | within tolerance",
             yawRight);
    demoLog(line);
    return !stopRequested();
  }
  snprintf(line, sizeof(line),
           "ALIGN | yaw=%+.2fdeg | rotate in place to cardinal heading",
           yawRight);
  demoLog(line);
  return turnAndSettle(yawRight);
}

void runFloodStep() {
  if (!settle()) {
    stopExplorer("STOP | command received before scan");
    return;
  }

  char line[180];
  snprintf(line, sizeof(line), "========== STEP %lu | CELL (%d,%d) | %s ==========",
          completedCells + 1, robotX, robotY,
          MazeMap::directionName(robotHeading));
  demoLog("");
  demoLog(line);

  bool frontOpen, leftOpen, rightOpen;
  int frontMm, leftMm, rightMm;
  if (!readOpenPaths(frontOpen, leftOpen, rightOpen,
                     frontMm, leftMm, rightMm)) {
    stopExplorer(demoStopped()
        ? "STOP | scan cancelled"
        : "FAULT | scan failed after retries; see SCAN ERROR");
    return;
  }
  updateMapFromScan(frontOpen, leftOpen, rightOpen);
  logPose("SCANNED");

  if (maze.isGoal(robotX, robotY)) {
    stopExplorer("GOAL | center reached; exploration complete");
    return;
  }

  Direction selected;
  if (!maze.chooseNext(robotX, robotY, robotHeading, selected)) {
    stopExplorer("FAULT | flood fill found no reachable neighbor");
    return;
  }
  const int nextX = robotX + MazeMap::dx(selected);
  const int nextY = robotY + MazeMap::dy(selected);
  snprintf(line, sizeof(line),
           "FLOOD | current=%u | choose=%s | next=(%d,%d) distance=%u visited=%d",
           maze.cell(robotX, robotY).distance,
           MazeMap::directionName(selected), nextX, nextY,
           maze.cell(nextX, nextY).distance,
           maze.cell(nextX, nextY).visited);
  demoLog(line);

  if (!turnTo(selected)) {
    stopExplorer("FAULT | navigation turn stopped or failed");
    return;
  }

  demoLog("CHECK | verify selected cell after turn");
  if (!readOpenPaths(frontOpen, leftOpen, rightOpen,
                     frontMm, leftMm, rightMm)) {
    stopExplorer(demoStopped()
        ? "STOP | post-turn scan cancelled"
        : "FAULT | post-turn scan failed");
    return;
  }
  updateMapFromScan(frontOpen, leftOpen, rightOpen);
  if (!frontOpen) {
    demoLog("REPLAN | selected edge is now blocked; remain in current cell");
    publishState("Replanning");
    return;
  }

  if (stopRequested()) {
    stopExplorer("STOP | command received before movement");
    return;
  }
  snprintf(line, sizeof(line), "MOVE PLAN | (%d,%d) -> (%d,%d) heading=%s",
           robotX, robotY, nextX, nextY,
           MazeMap::directionName(robotHeading));
  demoLog(line);
  const DemoMoveResult outcome = demoMoveOneCell();
  if (outcome == DemoMoveResult::Stopped ||
      outcome == DemoMoveResult::Failed) {
    stopExplorer(outcome == DemoMoveResult::Stopped
        ? "STOP | cell movement cancelled"
        : "FAULT | cell movement failed; coordinates unchanged");
    return;
  }
  demoLog(outcome == DemoMoveResult::FrontWallReached
      ? "ARRIVAL | front-wall reference reached"
      : "ARRIVAL | encoder target reached");
  if (!maze.isGoal(nextX, nextY) && !alignArrivalHeading()) {
    stopExplorer("FAULT | arrival heading correction failed");
    return;
  }

  const int oldX = robotX;
  const int oldY = robotY;
  robotX += MazeMap::dx(robotHeading);
  robotY += MazeMap::dy(robotHeading);
  if (!maze.inBounds(robotX, robotY)) {
    robotX = oldX;
    robotY = oldY;
    stopExplorer("FAULT | move would place pose outside maze");
    return;
  }
  maze.markTraversed(oldX, oldY, robotHeading);
  maze.floodFill();
  ++completedCells;
  logPose("CELL REACHED");
  if (maze.isGoal(robotX, robotY)) {
    stopExplorer("GOAL | center reached; exploration complete");
    return;
  }
  publishState("Exploring");
}

#if MAIN_UNIT_TEST_MODE
void stopUnitTest(const char *reason) {
  stopRotation();
  demoEndRun();
  unitTestRunning = false;
  demoLog(reason);
  publishState(reason);
}

void runUnitTestStep() {
  if (!settle()) {
    stopUnitTest("UNIT TEST | STOP | command received");
    return;
  }

  const UnitTestAction action = UNIT_TEST_PATTERN[unitTestStep];
  const char *actionName = "FORWARD 1 CELL";
  if (action == UnitTestAction::Right) actionName = "RIGHT 90deg";
  else if (action == UnitTestAction::Left) actionName = "LEFT 90deg";
  char line[110];
  snprintf(line, sizeof(line), "UNIT TEST | STEP %d/%d | %s",
           unitTestStep + 1, UNIT_TEST_STEP_COUNT, actionName);
  demoLog("");
  demoLog(line);

  if (action != UnitTestAction::Forward) {
    const float turnAngle = action == UnitTestAction::Right ? -90.0f : 90.0f;
    if (!turnDegrees(turnAngle)) {
      stopUnitTest(demoStopped()
          ? "UNIT TEST | STOP | turn cancelled"
          : "UNIT TEST | FAULT | turn failed");
      return;
    }
  } else {
    const DemoMoveResult result = demoMoveOneCell();
    if (result != DemoMoveResult::EncoderReached) {
      if (result == DemoMoveResult::Stopped)
        stopUnitTest("UNIT TEST | STOP | forward move cancelled");
      else if (result == DemoMoveResult::FrontWallReached)
        stopUnitTest("UNIT TEST | STOP | front wall interrupted scripted move");
      else
        stopUnitTest("UNIT TEST | FAULT | forward move failed");
      return;
    }
    if (!alignArrivalHeading()) {
      stopUnitTest(demoStopped()
          ? "UNIT TEST | STOP | arrival alignment cancelled"
          : "UNIT TEST | FAULT | arrival heading correction failed");
      return;
    }
  }

  snprintf(line, sizeof(line), "UNIT TEST | RESULT %d/%d | OK",
           unitTestStep + 1, UNIT_TEST_STEP_COUNT);
  demoLog(line);
  ++unitTestStep;
  if (unitTestStep == UNIT_TEST_STEP_COUNT)
    stopUnitTest("UNIT TEST | COMPLETE | pattern finished");
}
#endif

void setup() {
  Serial.begin(115200);
  rotationReady = beginRotation();
  moveForwardSetup();
  demoEndRun();

  maze.reset();
  dashboardBegin();
#if MAIN_UNIT_TEST_MODE
  char patternInfo[60];
  snprintf(patternInfo, sizeof(patternInfo), "UNIT TEST | MODE | %d scripted steps",
           UNIT_TEST_STEP_COUNT);
  demoLog(patternInfo);
  demoLog("UNIT TEST | COMMANDS | s/start runs once | d/stop cancels");
#else
  resetMap();

  demoLog("BUILD | flood-fill exploration-v1 | speed run disabled");
  demoLog("COMMANDS | s/start begins or resumes | d/stop stops | web reset clears map");
  demoLog("COORDINATES | start=(0,0), north=+Y, east=+X");
#endif
  if (WiFi.status() == WL_CONNECTED) {
    char url[80];
    snprintf(url, sizeof(url), "WEB | open http://%s/",
             WiFi.localIP().toString().c_str());
    demoLog(url);
  }
  const bool motionReady = demoMotionReady();
  if (!rotationReady) {
    demoLog(rotationInitializationFault());
    demoLog("FAULT | rotation MPU initialization failed");
  }
  if (!motionReady) {
    char fault[120];
    snprintf(fault, sizeof(fault), "FAULT | %s", demoMotionFault());
    demoLog(fault);
  }
  if (rotationReady && motionReady) demoLog("READY | open the web page or send s/start");
#if MAIN_UNIT_TEST_MODE
  publishState(rotationReady && motionReady
      ? "Unit test ready" : "Initialization failed");
#else
  publishState(rotationReady && motionReady
      ? "Ready" : "Initialization failed");
#endif
}

void loop() {
  dashboardLoop();
#if MAIN_UNIT_TEST_MODE
  if (dashboardTakeReset()) {
    if (unitTestRunning) stopUnitTest("UNIT TEST | STOP | web reset requested");
    maze.reset();
    demoLog("UNIT TEST | RESET | ready for a new run");
    publishState("Unit test ready");
  }

  const MovementCommand command = demoReadCommand();
  if (command == MovementCommand::Stop) {
    if (unitTestRunning) stopUnitTest("UNIT TEST | STOP | command received");
  } else if (command == MovementCommand::Start && !unitTestRunning) {
    const bool motionReady = demoMotionReady();
    if (!rotationReady || !motionReady) {
      if (!rotationReady) demoLog("UNIT TEST | REFUSED | rotation MPU unavailable");
      if (!motionReady) {
        char fault[120];
        snprintf(fault, sizeof(fault), "UNIT TEST | REFUSED | %s", demoMotionFault());
        demoLog(fault);
      }
      publishState("Unit test initialization fault");
    } else {
      demoBeginRun();
      unitTestStep = 0;
      unitTestRunning = true;
      demoLog("UNIT TEST | START | running scripted pattern");
      publishState("Unit test running");
    }
  }

  if (unitTestRunning) runUnitTestStep();
#else
  if (dashboardTakeReset()) {
    if (explorerRunning) stopExplorer("STOP | web map reset requested");
    resetMap();
  }

  const MovementCommand command = demoReadCommand();
  if (command == MovementCommand::Stop) {
    if (explorerRunning) stopExplorer("STOP | command received");
  } else if (command == MovementCommand::Start && !explorerRunning) {
    const bool motionReady = demoMotionReady();
    if (!rotationReady || !motionReady) {
      if (!rotationReady) demoLog("REFUSED | rotation MPU initialization failed; reset ESP32");
      if (!motionReady) {
        char fault[120];
        snprintf(fault, sizeof(fault), "REFUSED | %s", demoMotionFault());
        demoLog(fault);
      }
      publishState("Initialization fault");
    } else if (maze.isGoal(robotX, robotY)) {
      demoLog("REFUSED | goal already reached; reset the map for another run");
    } else {
      demoBeginRun();
      explorerRunning = true;
      demoLog("START | flood-fill exploration; speed run disabled");
      logPose("START");
      publishState("Exploring");
    }
  }

  if (explorerRunning) runFloodStep();
#endif
  dashboardLoop();
  delay(10);
}
