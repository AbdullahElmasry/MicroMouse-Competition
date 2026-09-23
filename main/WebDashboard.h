#pragma once

#include <Arduino.h>

#include "MazeMap.h"

void dashboardBegin();
void dashboardLoop();
bool dashboardTakeStart();
bool dashboardTakeStop();
bool dashboardTakeReset();
void dashboardAppendLog(const char *line);
void dashboardUpdateMap(const MazeMap &maze, int robotX, int robotY,
                        Direction heading, bool running, const char *status);
