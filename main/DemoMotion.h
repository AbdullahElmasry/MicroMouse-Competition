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
bool demoHeadingError(float &yawRightDegrees);
void demoLog(const char *text);
