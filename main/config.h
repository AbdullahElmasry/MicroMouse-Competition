#pragma once

// Opening distances measured at a cell center. Right distance includes the
// same 10 mm sensor-inset correction as the tested forward module.
constexpr int SIDE_OPEN_MM = 100;
constexpr int FRONT_OPEN_MM = 100;
// Mapping needs more clearance than the forward controller's wall approach.
// In the run log, 103-110 mm was only 40-67 mm from the stopping wall.
constexpr int FRONT_MAP_OPEN_MM = 120;
inline bool cellTravelMeetsMinimum(float leftMm, float rightMm,
                                   float minimumMm) {
  return leftMm >= minimumMm && rightMm >= minimumMm;
}
constexpr int OPEN_CONFIRM_SAMPLES = 6;
constexpr unsigned long SETTLE_MS = 150;
constexpr float ARRIVAL_YAW_TOLERANCE_DEG = 1.0f;
constexpr int MPU_I2C_MAX_ATTEMPTS = 10;
constexpr unsigned long MPU_I2C_RETRY_BUDGET_MS = 1000;
constexpr unsigned long MPU_RETRY_MOTOR_HOLD_MS = 40;
constexpr int FRONT_WALL_TARGET_MM = 40;
constexpr float FRONT_WALL_TOLERANCE_PERCENT = 1.0f;
constexpr float FRONT_WALL_TOLERANCE_MM =
    FRONT_WALL_TARGET_MM * FRONT_WALL_TOLERANCE_PERCENT / 100.0f;
// Brake before the 60 mm target: recent runs continued another 10-19 mm
// after the trigger and reached 41-57 mm at the wall.
constexpr int FRONT_WALL_STOP_TRIGGER_MM = 90;
constexpr int FRONT_PREMATURE_BRAKE_MM = 100;
constexpr float FRONT_SKEW_BRAKE_DEG = 8.0f;
// Bounds on extra travel after the encoder endpoint when a front wall is seen.
constexpr unsigned long FRONT_APPROACH_TIMEOUT_MS = 3000;
constexpr float FRONT_APPROACH_MAX_EXTRA_MM = 160.0f;
