#pragma once

// Convert a recessed sensor's range to clearance from the chassis edge.
// Preserve invalid samples; a valid range inside the offset means zero clearance.
inline int sideClearanceMm(int rangeMm, int insetMm) {
  if (rangeMm < 0) return -1;
  return rangeMm > insetMm ? rangeMm - insetMm : 0;
}

struct ForwardWallSettings {
  int baseSpeed;
  int slowSpeed;
  int leftThresholdMm;
  int rightThresholdMm;
};

struct ForwardMotorCommands {
  int left;
  int right;
};

// Preserve main.ino's physical motor convention and left-first priority.
// Negative distances indicate invalid sensor readings, not a nearby wall.
inline ForwardMotorCommands forwardWallCommands(
    int leftMm, int rightMm, const ForwardWallSettings &settings) {
  ForwardMotorCommands commands = {settings.baseSpeed, settings.baseSpeed};
  if (leftMm >= 0 && leftMm < settings.leftThresholdMm) {
    commands.left = settings.slowSpeed;
  } else if (rightMm >= 0 && rightMm < settings.rightThresholdMm) {
    commands.right = settings.slowSpeed;
  }
  return commands;
}

inline bool cellEncoderLimitReached(unsigned long left, unsigned long right,
                                    unsigned long leftTarget, unsigned long rightTarget) {
  // Brake both motors together, as main.ino does. Do not pivot to finish one wheel.
  return left >= leftTarget || right >= rightTarget;
}
