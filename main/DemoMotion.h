#pragma once

#include "MoveForward.h"

bool demoMotionReady();
MovementCommand demoReadCommand();
void demoBeginRun();
void demoEndRun();
bool demoStopped();
bool demoReadPaths(int &front, int &left, int &right);
enum class DemoMoveResult { EncoderReached, FrontWallReached, Stopped, Failed };
DemoMoveResult demoMoveOneCell();
DemoMoveResult demoMoveStraightCells(int cells);
DemoMoveResult demoReachFrontWallReference();
bool demoHeadingError(float &yawRightDegrees);
bool demoResetYaw();
void demoLog(const char *text);
