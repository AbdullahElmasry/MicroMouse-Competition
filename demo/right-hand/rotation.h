#pragma once

bool beginRotation();
// Tested robot direction: positive angles turn left; negative angles turn right.
bool turnDegrees(float relativeAngle);
void stopRotation();
