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

#ifdef LED_BUILTIN
constexpr int STATUS_LED_PIN = LED_BUILTIN;
#else
constexpr int STATUS_LED_PIN = 2;
#endif

#if !MAIN_UNIT_TEST_MODE
enum class LedMode { Solid, ReadyBlink, GoalBlink };
enum class HandState { WaitClear, WaitHold, Armed };
LedMode ledMode = LedMode::Solid;
HandState handState = HandState::WaitClear;
bool handHolding = false;
bool handStartEnabled = true;
bool savedMapReady = false;
bool speedRunning = false;
Direction speedRoute[MAZE_SIZE * MAZE_SIZE];
int speedRouteLength = 0;
int speedRouteIndex = 0;
unsigned long handHoldStarted = 0;
unsigned long ledBlinkStarted = 0;
unsigned long lastValidHandReading = 0;
unsigned long lastHandSample = 0;
unsigned long lastHandLog = 0;
int handBaselineMm = 0;
int handBaselineSamples = 0;
int handInitialMm[3] = {};
int lastHoldSecond = 0;
constexpr int HAND_MAX_NEAR_MM = 100;
constexpr int HAND_DROP_MM = 25;
constexpr int HAND_RELEASE_MARGIN_MM = 5;
constexpr unsigned long HAND_HOLD_MS = 3000;

int handNearThreshold() {
  const int drop = handBaselineMm < 70 ? 10 : HAND_DROP_MM;
  return min(HAND_MAX_NEAR_MM, handBaselineMm - drop);
}

int handReleaseThreshold() {
  return max(handNearThreshold() + 10,
             handBaselineMm - HAND_RELEASE_MARGIN_MM);
}

void updateStatusLed() {
  if (ledMode == LedMode::Solid) {
    digitalWrite(STATUS_LED_PIN, HIGH);
    return;
  }
  const unsigned long period = ledMode == LedMode::ReadyBlink ? 250 : 100;
  digitalWrite(STATUS_LED_PIN,
               ((millis() - ledBlinkStarted) / period) % 2 == 0 ? LOW : HIGH);
}

void blinkCellLed() {
  digitalWrite(STATUS_LED_PIN, LOW);
  delay(80);
  updateStatusLed();
}

bool handReleasedAfterHold() {
  if (millis() - lastHandSample < 50) return false;
  lastHandSample = millis();
  int front;
  bool covered = false;
  if (!demoReadFrontDistance(front, &covered)) {
    if (handHolding && millis() - lastValidHandReading > 150) {
      handHolding = false;
      demoLog("HAND | hold interrupted; timer reset");
    }
    if (millis() - lastValidHandReading > 2000 &&
        millis() - lastHandLog > 2000) {
      demoLog("HAND SENSOR | no valid front ToF samples");
      lastHandLog = millis();
    }
    return false;
  }
  lastValidHandReading = millis();
  if (handState == HandState::WaitClear) {
    if (covered) {
      if (millis() - lastHandLog >= 2000) {
        demoLog("HAND | uncover front sensor to establish idle range");
        lastHandLog = millis();
      }
      return false;
    }
    handInitialMm[handBaselineSamples] = front;
    if (++handBaselineSamples >= 3) {
      const int a = handInitialMm[0], b = handInitialMm[1], c = handInitialMm[2];
      handBaselineMm = a + b + c - min(a, min(b, c)) - max(a, max(b, c));
      handState = HandState::WaitHold;
      char line[100];
      snprintf(line, sizeof(line), "HAND | idle front=%dmm | near trigger <=%dmm",
               handBaselineMm, handNearThreshold());
      demoLog(line);
    }
  } else if (handState == HandState::WaitHold) {
    if (!covered && !handHolding && front > handBaselineMm &&
        front - handBaselineMm <= 40)
      handBaselineMm = front;
    const int nearMm = handNearThreshold();
    if (covered || front <= nearMm + (handHolding ? 5 : 0)) {
      if (!handHolding) {
        handHolding = true;
        handHoldStarted = millis();
        lastHoldSecond = 0;
        char line[100];
        snprintf(line, sizeof(line),
                 "HAND | detected %s | hold for 3 full seconds",
                 covered ? "sensor covered" : "near object");
        demoLog(line);
      } else {
        const unsigned long heldMs = millis() - handHoldStarted;
        if (heldMs >= HAND_HOLD_MS) {
          handState = HandState::Armed;
          ledMode = LedMode::ReadyBlink;
          ledBlinkStarted = millis();
          updateStatusLed();
          char line[100];
          snprintf(line, sizeof(line),
                   "HAND | armed after %lums; LED blinking; remove hand to start",
                   heldMs);
          demoLog(line);
        } else if ((int)(heldMs / 1000) > lastHoldSecond) {
          lastHoldSecond = heldMs / 1000;
          char line[80];
          snprintf(line, sizeof(line), "HAND | holding %d/3 seconds",
                   lastHoldSecond);
          demoLog(line);
        }
      }
    } else {
      if (handHolding) demoLog("HAND | released too early; timer reset");
      handHolding = false;
    }
  } else if (!covered && front >= handReleaseThreshold()) {
    handState = HandState::WaitClear;
    handHolding = false;
    ledMode = LedMode::Solid;
    updateStatusLed();
    demoLog("HAND | released; start requested");
    return true;
  }
  if (millis() - lastHandLog >= 2000) {
    char line[160];
    snprintf(line, sizeof(line),
             "HAND SENSOR | front=%dmm covered=%d | idle=%dmm | near<=%dmm | release>=%dmm | state=%s",
             front, covered, handBaselineMm,
             handNearThreshold(), handReleaseThreshold(),
             handState == HandState::Armed ? "ARMED" : "WAIT");
    demoLog(line);
    lastHandLog = millis();
  }
  return false;
}
#endif

void publishState(const char *status) {
#if !MAIN_UNIT_TEST_MODE
  const bool running = explorerRunning || speedRunning;
#else
  const bool running = explorerRunning;
#endif
  dashboardUpdateMap(
      maze, robotX, robotY, robotHeading, running, status);
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
#if !MAIN_UNIT_TEST_MODE
  MazeMap::clearSaved();
  savedMapReady = false;
  speedRunning = false;
  speedRouteLength = speedRouteIndex = 0;
  handStartEnabled = true;
  handState = HandState::WaitClear;
  handHolding = false;
  handBaselineMm = 0;
  handBaselineSamples = 0;
  ledMode = LedMode::Solid;
#endif
  robotX = 0;
  robotY = 0;
  robotHeading = Direction::North;
  completedCells = 0;
  demoLog("MAP RESET | start=(0,0) | heading=NORTH | goal=(7,7)-(8,8)");
  logPose("RESET");
  publishState("Ready");
}

#if !MAIN_UNIT_TEST_MODE
void completeExploration() {
  stopExplorer("GOAL | center reached; exploration complete");
  savedMapReady = maze.save();
  handStartEnabled = true;
  demoLog(savedMapReady ? "MAP SAVE | OK | speed run armed for next gesture"
                        : "MAP SAVE | FAILED | speed run unavailable");
  handState = HandState::WaitClear;
  handHolding = false;
  handBaselineMm = 0;
  handBaselineSamples = 0;
  ledMode = LedMode::GoalBlink;
  ledBlinkStarted = millis();
  publishState(savedMapReady ? "Goal reached; maze saved"
                             : "Goal reached; maze save failed");
}

bool planSpeedRoute() {
  constexpr int cellCount = MAZE_SIZE * MAZE_SIZE;
  int16_t previous[cellCount];
  Direction entered[cellCount];
  uint16_t queue[cellCount];
  for (int i = 0; i < cellCount; ++i) previous[i] = -1;
  previous[0] = 0;
  queue[0] = 0;
  int head = 0, tail = 1, goal = -1;
  while (head < tail) {
    const int current = queue[head++];
    const int x = current % MAZE_SIZE;
    const int y = current / MAZE_SIZE;
    if (maze.isGoal(x, y)) { goal = current; break; }
    for (int raw = 0; raw < 4; ++raw) {
      const Direction direction = (Direction)raw;
      if (!(maze.cell(x, y).traversed & (1U << raw))) continue;
      const int nx = x + MazeMap::dx(direction);
      const int ny = y + MazeMap::dy(direction);
      if (!maze.inBounds(nx, ny)) continue;
      const int next = ny * MAZE_SIZE + nx;
      if (previous[next] != -1) continue;
      previous[next] = current;
      entered[next] = direction;
      queue[tail++] = next;
    }
  }
  if (goal < 0) return false;
  speedRouteLength = 0;
  for (int current = goal; current != 0; current = previous[current])
    speedRoute[speedRouteLength++] = entered[current];
  for (int i = 0; i < speedRouteLength / 2; ++i) {
    const Direction first = speedRoute[i];
    speedRoute[i] = speedRoute[speedRouteLength - 1 - i];
    speedRoute[speedRouteLength - 1 - i] = first;
  }
  speedRouteIndex = 0;
  return speedRouteLength > 0;
}

void stopSpeedRun(const char *reason) {
  stopRotation();
  demoEndRun();
  speedRunning = false;
  demoLog(reason);
  logPose("SPEED STOPPED");
  publishState(reason);
}
#endif

bool readOpenPaths(bool &frontOpen, bool &leftOpen, bool &rightOpen,
                   int &frontMm, int &leftMm, int &rightMm) {
  static_assert(OPEN_CONFIRM_SAMPLES >= 5, "ToF scan needs five readings");
  int frontSamples[5], leftSamples[5], rightSamples[5];
  for (int sample = 0; sample < OPEN_CONFIRM_SAMPLES; ++sample) {
    if (stopRequested()) return false;
    int front, left, right;
    if (!demoReadPaths(front, left, right)) return false;
    frontSamples[sample % 5] = front;
    leftSamples[sample % 5] = left;
    rightSamples[sample % 5] = right;
  }
  int frontSum = 0, leftSum = 0, rightSum = 0;
  for (int i = 0; i < 5; ++i) {
    frontSum += frontSamples[i];
    leftSum += leftSamples[i];
    rightSum += rightSamples[i];
  }
  frontMm = (frontSum + 2) / 5;
  leftMm = (leftSum + 2) / 5;
  rightMm = (rightSum + 2) / 5;
  frontOpen = frontMm > FRONT_OPEN_MM;
  leftOpen = leftMm > SIDE_OPEN_MM;
  rightOpen = rightMm > SIDE_OPEN_MM;
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

#if !MAIN_UNIT_TEST_MODE
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
    completeExploration();
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
  blinkCellLed();
  if (maze.isGoal(robotX, robotY)) {
    completeExploration();
    return;
  }
  publishState("Exploring");
}

void runSpeedStep() {
  if (speedRouteIndex >= speedRouteLength) return;
  const Direction direction = speedRoute[speedRouteIndex];
  int cells = 1;
  while (speedRouteIndex + cells < speedRouteLength &&
         speedRoute[speedRouteIndex + cells] == direction) ++cells;
  const int endX = robotX + cells * MazeMap::dx(direction);
  const int endY = robotY + cells * MazeMap::dy(direction);
  char line[150];
  snprintf(line, sizeof(line),
           "SPEED | segment %d/%d | %s %d cells | (%d,%d) -> (%d,%d)",
           speedRouteIndex + 1, speedRouteLength,
           MazeMap::directionName(direction), cells,
           robotX, robotY, endX, endY);
  demoLog(line);
  if (!turnTo(direction)) {
    stopSpeedRun(demoStopped() ? "SPEED | STOP | turn cancelled"
                               : "SPEED | FAULT | turn failed");
    return;
  }
  const DemoMoveResult outcome = demoMoveStraightCells(cells);
  if (outcome == DemoMoveResult::Stopped || outcome == DemoMoveResult::Failed) {
    stopSpeedRun(outcome == DemoMoveResult::Stopped
        ? "SPEED | STOP | move cancelled"
        : "SPEED | FAULT | straight move failed; pose uncertain");
    return;
  }
  if (outcome == DemoMoveResult::FrontWallReached &&
      !maze.isGoal(endX, endY)) {
    const uint8_t edge = 1U << (uint8_t)direction;
    const MazeCell &endCell = maze.cell(endX, endY);
    if (!(endCell.known & edge) || !(endCell.walls & edge)) {
      stopSpeedRun("SPEED | FAULT | unexpected front wall; pose uncertain");
      return;
    }
  }
  if (!maze.isGoal(endX, endY) && !alignArrivalHeading()) {
    stopSpeedRun("SPEED | FAULT | heading correction failed");
    return;
  }
  robotX = endX;
  robotY = endY;
  speedRouteIndex += cells;
  completedCells += cells;
  logPose("SPEED SEGMENT REACHED");
  publishState("Speed run");
  if (maze.isGoal(robotX, robotY)) {
    stopSpeedRun("SPEED | GOAL REACHED");
    ledMode = LedMode::GoalBlink;
    ledBlinkStarted = millis();
  }
}
#endif

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
  pinMode(STATUS_LED_PIN, OUTPUT);
  digitalWrite(STATUS_LED_PIN, HIGH);
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
  if (maze.load()) {
    savedMapReady = true;
    demoLog("MAP LOAD | saved goal maze ready; place robot at start for speed run");
    logPose("SAVED MAP LOADED");
    publishState("Saved maze loaded; speed run pending");
  } else {
    resetMap();
  }

  demoLog("BUILD | flood-fill exploration and saved-route speed run");
  demoLog("COMMANDS | hold hand 3s then release to start | s/start also starts | d/stop stops | web reset clears map");
  demoLog("HAND | hold object nearer than logged threshold or cover sensor for 3s, then remove");
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
  if (rotationReady && motionReady) {
#if MAIN_UNIT_TEST_MODE
    demoLog("READY | send s/start for scripted test");
#else
    demoLog("READY | hold and release hand, or send s/start");
#endif
  }
#if MAIN_UNIT_TEST_MODE
  publishState(rotationReady && motionReady
      ? "Unit test ready" : "Initialization failed");
#else
  publishState(rotationReady && motionReady
      ? (savedMapReady ? "Saved maze loaded; speed run pending" : "Ready")
      : "Initialization failed");
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
  updateStatusLed();
  if (dashboardTakeReset()) {
    if (explorerRunning) stopExplorer("STOP | web map reset requested");
    if (speedRunning) stopSpeedRun("SPEED | STOP | web map reset requested");
    resetMap();
  }

  const MovementCommand command = demoReadCommand();
  const bool handStart = handStartEnabled && !explorerRunning && !speedRunning &&
                         handReleasedAfterHold();
  if (command == MovementCommand::Stop) {
    if (explorerRunning) stopExplorer("STOP | command received");
    if (speedRunning) stopSpeedRun("SPEED | STOP | command received");
  } else if ((command == MovementCommand::Start || handStart) &&
             !explorerRunning && !speedRunning) {
    const bool motionReady = demoMotionReady();
    if (!rotationReady || !motionReady) {
      if (!rotationReady) demoLog("REFUSED | rotation MPU initialization failed; reset ESP32");
      if (!motionReady) {
        char fault[120];
        snprintf(fault, sizeof(fault), "REFUSED | %s", demoMotionFault());
        demoLog(fault);
      }
      publishState("Initialization fault");
    } else if (savedMapReady) {
      if (!planSpeedRoute()) {
        demoLog("SPEED | REFUSED | no traversed path from start to goal");
        publishState("No confirmed speed route");
      } else {
        handStartEnabled = false;
        handState = HandState::WaitClear;
        handHolding = false;
        handBaselineMm = 0;
        handBaselineSamples = 0;
        ledMode = LedMode::Solid;
        updateStatusLed();
        robotX = 0;
        robotY = 0;
        robotHeading = Direction::North;
        completedCells = 0;
        demoBeginRun();
        speedRunning = true;
        char speedInfo[80];
        snprintf(speedInfo, sizeof(speedInfo),
                 "SPEED | START | %d confirmed cells to goal", speedRouteLength);
        demoLog(speedInfo);
        logPose("SPEED START");
        publishState("Speed run");
      }
    } else if (maze.isGoal(robotX, robotY)) {
      demoLog("REFUSED | goal already reached; map was not saved");
    } else {
      handStartEnabled = false;
      handState = HandState::WaitClear;
      handHolding = false;
      handBaselineMm = 0;
      handBaselineSamples = 0;
      ledMode = LedMode::Solid;
      updateStatusLed();
      demoBeginRun();
      explorerRunning = true;
      demoLog("START | flood-fill exploration; speed run disabled");
      logPose("START");
      publishState("Exploring");
    }
  }

  if (explorerRunning) runFloodStep();
  if (speedRunning) runSpeedStep();
#endif
  dashboardLoop();
  delay(10);
}
