#include <Arduino.h>
#include <WiFi.h>

#include "DemoMotion.h"
#include "MazeMap.h"
#include "WebDashboard.h"
#include "config.h"
#include "rotation.h"

MazeMap maze;
bool rotationReady = false;
bool explorerRunning = false;
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
    demoLog("MAP WARNING | sensor result changed a previously known wall");
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
  snprintf(line, sizeof(line), "STEP %lu | scan cell=(%d,%d) heading=%s",
           completedCells + 1, robotX, robotY,
           MazeMap::directionName(robotHeading));
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
  if (!alignArrivalHeading()) {
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
  publishState(maze.isGoal(robotX, robotY)
      ? "Center reached; final scan pending"
      : "Exploring");
}

void setup() {
  Serial.begin(115200);
  rotationReady = beginRotation();
  moveForwardSetup();
  demoEndRun();

  maze.reset();
  dashboardBegin();
  resetMap();

  demoLog("BUILD | flood-fill exploration-v1 | speed run disabled");
  demoLog("COMMANDS | s/start begins or resumes | d/stop stops | web reset clears map");
  demoLog("COORDINATES | start=(0,0), north=+Y, east=+X");
  if (WiFi.status() == WL_CONNECTED) {
    char url[80];
    snprintf(url, sizeof(url), "WEB | open http://%s/",
             WiFi.localIP().toString().c_str());
    demoLog(url);
  }
  demoLog(rotationReady && demoMotionReady()
      ? "READY | open the web page or send s/start"
      : "FAULT | rotation, MPU or ToF initialization failed");
  publishState(rotationReady && demoMotionReady()
      ? "Ready" : "Initialization failed");
}

void loop() {
  dashboardLoop();
  if (dashboardTakeReset()) {
    if (explorerRunning) stopExplorer("STOP | web map reset requested");
    resetMap();
  }

  const MovementCommand command = demoReadCommand();
  if (command == MovementCommand::Stop) {
    if (explorerRunning) stopExplorer("STOP | command received");
  } else if (command == MovementCommand::Start && !explorerRunning) {
    if (!rotationReady || !demoMotionReady()) {
      demoLog("REFUSED | rotation, MPU or ToF unavailable; reset ESP32");
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
  dashboardLoop();
  delay(10);
}
