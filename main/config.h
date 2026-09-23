#pragma once

// Opening distances measured at a cell center. Right distance includes the
// same 10 mm sensor-inset correction as the tested forward module.
constexpr int SIDE_OPEN_MM = 100;
constexpr int FRONT_OPEN_MM = 100;
constexpr int OPEN_CONFIRM_SAMPLES = 3;
constexpr unsigned long SETTLE_MS = 150;
constexpr float ARRIVAL_YAW_TOLERANCE_DEG = 1.0f;
constexpr int FRONT_WALL_TARGET_MM = 60;
constexpr float FRONT_WALL_TOLERANCE_PERCENT = 3.0f;
constexpr float FRONT_WALL_TOLERANCE_MM =
    FRONT_WALL_TARGET_MM * FRONT_WALL_TOLERANCE_PERCENT / 100.0f;
// VL53L1X readings are whole millimetres. 61 mm is the largest integer
// inside the 58.2-61.8 mm target band, so braking begins there.
constexpr int FRONT_WALL_STOP_TRIGGER_MM = 61;
constexpr int FRONT_WALL_MIN_IN_BAND_MM = 59;
// Bounds on extra travel after the encoder endpoint when a front wall is seen.
constexpr unsigned long FRONT_APPROACH_TIMEOUT_MS = 3000;
constexpr float FRONT_APPROACH_MAX_EXTRA_MM = 120.0f;
