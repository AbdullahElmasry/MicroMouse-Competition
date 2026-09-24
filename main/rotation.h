#pragma once

bool beginRotation();
const char *rotationInitializationFault();
// Tested robot direction: positive angles turn left; negative angles turn right.
bool turnDegrees(float relativeAngle);
void stopRotation();
