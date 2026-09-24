#pragma once

#include "MoveForward.h"

bool demoMotionReady();
const char *demoMotionFault();
MovementCommand demoReadCommand();
void demoBeginRun();
void demoEndRun();
bool demoStopped();
bool demoReadPaths(int &front, int &left, int &right);
bool demoReadFrontDistance(int &front, bool *covered = nullptr);
enum class DemoMoveResult { EncoderReached, FrontWallReached, Stopped, Failed };
DemoMoveResult demoMoveOneCell();
DemoMoveResult demoMoveStraightCells(int cells);
bool demoHeadingError(float &yawRightDegrees);
void demoLog(const char *text);
void demoReadEncoderTicks(unsigned long &left, unsigned long &right);
