## Right ToF mounting correction

The right sensor is recessed 10 mm. The forward sketch subtracts RIGHT_TOF_INSET_MM = 10 from valid right readings before wall detection and both steering cases; telemetry labels R as corrected. Invalid readings remain invalid, and corrected negative clearances clamp to zero. Both side targets are now 40 mm from the chassis edge. The previous right target of 50 mm raw becomes 40 mm corrected, preserving its physical setpoint without compensating twice. Calibration sketches continue to report raw sensor ranges.

# Seven-cell forward test: two-wall and one-wall guidance

Send `start` over WiFi TCP port 23 or USB Serial at 115200 baud. The robot moves seven 180 mm cells, continuously, with no intermediate cell stops. After cell seven it stays stopped. Send `d` during movement to cancel the remaining run. A new start begins a new seven-cell sequence, not the remainder of an interrupted run. No automatic startup movement occurs. Blocking sensor reads and WiFi writes can delay command handling.

## Why the controller changed

The earlier PID applied the opposite motor command change to the hardware-tested base sketch. For a small left distance, main.ino reduces the named left motor command to 30 and keeps the right at 70; the PID increased the left and reduced the right. The old test also stopped wheels independently, potentially rotating the robot while one wheel finished its target. This version follows the working base sketch's command convention and brakes both together when either wheel reaches its calibrated limit.

## Active movement logic

| Side readings | Left command | Right command |
|---|---:|---:|
| Valid left below 40 mm | 30 | 70 |
| Otherwise, valid right below 40 mm | 70 | 30 |
| Otherwise | 70 | 70 |

The left condition takes priority if both thresholds trigger. These are pre-balance commands: factors remain left 1.0 and right 0.78. The table describes cruise speed. During the approach, the distance PID reduces the base command from 70 toward 35; a wall correction still slows the same motor to 30. The movement loop delay is 10 ms, as in main.ino. Motor pins, polarity, and braking outputs are unchanged. Both thresholds now use 40 mm chassis clearance; lateral correction remains threshold-based. A separate encoder-distance PID controls forward approach speed.

Tune `FORWARD_SPEED`, `WALL_SLOW_SPEED`, `LEFT_WALL_THRESHOLD_MM`, `RIGHT_WALL_THRESHOLD_MM`, `LEFT_FACTOR`, and `RIGHT_FACTOR` in the sketch. `ForwardWallControl.h` contains the steering decision and encoder stopping rule. The old `WallCentering.h` and its tests remain available for reference but are not included by this sketch. The last manually tuned PID values observed before this change were Kp=0.2, Ki=0, Kd=0, correction limit=40; those old lateral PID gains are not active.

Each run uses one encoder baseline and total limits of 4359 left / 4335 right ticks, derived from the two seven-cell hand-pushed measurements. Both wheels brake when either limit is reached, just like main.ino's OR condition. This intentionally differs from requiring both wheel targets. It can shorten travel if wheel progress differs, so measure total distance instead of assuming 180 mm or 1260 mm is exact.

## Reducing endpoint overshoot

`CellApproachControl.h` adds encoder-distance PID without reversing the working wall steering. It uses the smaller remaining count to either wheel's braking threshold, matching the rule that both motors stop at the first encoder limit. Outside the final 300 ticks (about 87 mm), the base PWM is 70. Inside that zone, PID reduces base PWM toward a floor of 35. The wall controller can still reduce the corresponding side to 30. All commands are before motor balance factors.

Edit `APPROACH_SETTINGS` in the sketch:

- Kp = 0.24 PWM/tick; Ki = 0.01 PWM/(tick*second); Kd = 0.01 PWM*second/tick. These are initial, unverified physical tuning values.
- `APPROACH_SLOWDOWN_TICKS = 300`. A larger zone permits earlier deceleration, but speed is also determined by the PID output.
- `MIN_APPROACH_PWM = 35`. Lower cautiously if the robot still overshoots; both motors must remain able to roll under load.
- `BRAKE_LEAD_TICKS = 10`, requesting braking approximately 2.9 mm before the nominal seven-cell endpoint. Effective thresholds are 4349 left / 4325 right ticks. Increase gradually; this changes the braking point, not the distance calibration.

Keep integral small: this is an endpoint approach, not a speed regulator. PID state resets at each run, output is bounded, and integral accumulation is suppressed at the limits. Encoder crossing triggers joint braking regardless of PID state.

Each run reports counts at braking and counts added during the following 100 ms. This separates target-crossing delay from rolling after braking. Measure actual travel too: tire slip and motion after the 100 ms reporting window are not captured by that difference. Blocking sensor/network calls can delay endpoint checks. The change should reduce approach speed; exact 1260 mm stopping still needs hardware tuning.

## Sensors and stop conditions

ToF startup matches the working calibration/tof_readings sketch: Wire.begin(), 10 ms shutdown, left/right/front wake order with 50 ms delays, addresses 0x30/0x31/0x32, timeouts of 200 ms after address assignment, and front Medium mode with startContinuous(30). Status is reported after initialization rather than aborting at intermediate checks.

A valid front ToF reading at or below `FRONT_EMERGENCY_STOP_MM = 40` brakes both motors immediately before side sensor reads. The run ends and requires a new `start`; clearing the obstacle does not resume it. This threshold is measured from the sensor face; actual stopping clearance depends on sampling and braking travel. Invalid front readings, both side readings invalid, encoder stall, whole-run timeout (105 seconds), or d cancel the run. A single invalid side is excluded from steering; the valid side can still request correction. If neither valid side is below its threshold, the robot uses 70/70. Invalid readings are shown as -1. The old side-distance-sum guard and 8 mm side stop are not used; case 2 has its own distance deadband and MPU freshness check. MPU feedback is used in one-wall mode only. Front-wall alignment is not implemented.

## Case 2: one usable side wall plus MPU yaw

Case 1 keeps its tested main.ino threshold steering. Case 2 uses a dedicated wall-distance PID. Targets are 40 mm chassis clearance on both sides, with +/-3.6 mm tolerance; the right reading is corrected for its 10 mm inset first. Outside that band, positive signed error (left wall too close or right wall too far) slows the left motor; negative error slows the right. The amount is proportional/integral/derivative feedback rather than a fixed drop straight to 30 PWM. MPU trim remains secondary: it operates only inside the distance band and cannot override a distance correction.

Tune these named constants in moving_forward.ino:

- `SINGLE_WALL_KP = 1.5`: PWM per millimeter of distance error.
- `SINGLE_WALL_KI = 0.05`: PWM per accumulated millimeter-second.
- `SINGLE_WALL_KD = 0.08`: PWM per millimeter/second of error change.
- `SINGLE_WALL_MAX_PWM = 40`: maximum wall correction, further limited by current approach speed and minimum motor PWM 30.

The PID uses actual sample time. State resets at each run, on a wall-side change, on return to two-wall mode, and inside the distance tolerance band. Conditional integration limits windup using the available motor-speed range. Derivative action can reduce a correction as error improves; outside the band, the output is restricted to the established correction direction. These are initial hardware tuning gains.

`SingleWallControl.h` owns wall-mode detection and the one-wall control law. `YawEstimate.h` owns the tested Kalman calculation and yaw integration. `MpuYaw.h` initializes the MPU6500 at 0x68 on the existing SDA 21/SCL 22 bus, calibrates offsets while stationary, and uses DLPF_6 / +/-500 dps. The filter retains measurement error 2, initial estimate error 2, Q=0.01, and the 0.5 dps deadband before filtering. Checked gyro reads use the same offset/range conversion as the [MPU6500_WE source](https://github.com/wollewald/MPU9250_WE/blob/main/src/MPU6500_WE.cpp). Install the MPU9250_WE library providing MPU6500_WE.h, as for the tested calibration sketch.

**Before enabling case 2:** set `MPU_YAW_SIGN` to +1 if the tested MPU yaw increases during a physical clockwise/right turn, or -1 if it decreases. The current sketch has -1, preserving your selected setting. A value of 0 disables case 2. Never guess this sign. All controller yaw values are normalized to right-positive after applying the sign.

At each start, relative yaw is zeroed once. Keep the robot level and parallel to the intended corridor direction at that moment. Yaw is not reset at individual cell boundaries or when switching wall modes, so accumulated heading error is not accepted as a new straight direction. Gyro yaw is relative and can drift; this is for the short seven-cell test, not long-term localization.

MPU updates are serviced approximately every 10 ms, including while the ToFs complete their measurements and while stopped between cells. ToF acquisition preserves front/left/right order and side single-shot measurements but polls readiness cooperatively instead of blocking for whole measurements. No separate task accesses the I2C bus. Network stalls can still delay sampling. A gap over 100 ms or a failed gyro read invalidates yaw until a new start resets it; case 2 stops while yaw is invalid. Case 1 does not depend on MPU health.

Wall detection uses hysteresis: acquire a usable wall below 120 mm and retain it below 140 mm. A healthy side sensor that completes a measurement but reports no target/out of range is represented as `SIDE_NO_TARGET_MM = 200`, above both detection thresholds, so a wall one or more cells away does not become a guidance wall. I2C failures and measurement timeouts still set `sideCommunicationFault` and cancel movement. One detected wall selects case 2, two select case 1, and no nearby walls select case 3.

Initial `SINGLE_WALL_SETTINGS` values:

| Setting | Value |
|---|---:|
| Left / right target clearance | 40 / 40 mm from the chassis edge |
| Distance deadband | +/-3.6 mm |
| Wall correction | PID reduction up to 40 PWM; motor command floor 30 |
| MPU use | Only within the wall-distance tolerance band |
| Heading proportional gain | 1 PWM/degree |
| Yaw-rate damping | 0.10 PWM/(degree/second) |
| Maximum MPU trim | 5 PWM |
| Minimum commanded motor PWM | 30, before balance factors |

The one-wall controller only slows a wheel; it never raises either above the existing distance-PID approach speed. During the final approach, available steering reduction shrinks to preserve the minimum command. Single-wall targets reuse the case-1 thresholds: 40 mm left and 40 mm corrected right. Verify actual chassis clearance during hardware tuning. Case-1 behavior is unchanged apart from applying the known right-sensor inset correction.

Test a left-only corridor and then a right-only corridor. Telemetry labels case two / left+MPU / right+MPU, and reports normalized yaw, wall error, wall PID PWM, and MPU trim separately. A nonzero wall error selects wall PID; zero wall error enables the limited MPU trim. Check corrections from slightly closer and farther positions, then check two-to-one and one-to-two wall transitions. Seven continuous cells, final-approach distance PID, brake lead applied once at the end, and start/d behavior remain active.

## Case 3: no side walls

When neither side wall is detected, `NoWallControl.h` holds the direction using calibrated encoder progress only. Entering case 3 captures the current left/right counts, so corrections made before the open area do not create an immediate encoder error. If either wall returns, the controller resets and the movement loop switches to case 1 or case 2 on that sample. Case 3 does not require a healthy MPU or use yaw in its steering command.

Positive encoder correction slows the named right motor; negative correction slows the named left motor. This direction comes from hardware case-3 logs: reducing the left command while the encoder error was positive increased the right drift and drove the PID into saturation. The active case-3 PID is:

- Encoder PID: Kp = 1.0, Ki = 0.10, Kd = 0.0.
- Encoder PID output limit = 40 PWM.

Encoder error compares distance progress using the two manual seven-cell trials: 4373/4344 and 4345/4325 ticks. Their averages are 4359 left and 4334.5 right over 1260 mm, or about 622.7/619.2 ticks per cell. Comparing normalized progress means this expected count difference represents equal travel rather than steering error. Telemetry labels this mode `none:encoder` and prints encoder error/PWM. On this robot, extra left-encoder progress reduces the named right command, and extra right-encoder progress reduces the named left command. Ki corrects persistent residual imbalance; conditional integration prevents windup while the output is limited.

## Verification

Start parallel to a straight corridor and keep d available. At readings 32/68 mm, telemetry must show commands 30/70. At corrected readings 47/39 mm it must show 70/30. At 44/55 mm it must show 70/70. Verify the physical corrections match the behavior of your base sketch. Record final heading, total travel, and left/right counts at the final stop. Verify the front emergency brake and d command before distance trials.

Host checks:

- `tests/single_wall_test.cpp`: wall classification/hysteresis, both one-wall sides, heading/rate corrections, command bounds, Kalman/deadband integration, and stale-data rejection.
- `tests/cell_approach_test.cpp`: distance-PID deceleration, output limits, reset, anti-windup, and brake-lead arithmetic.
- `tests/forward_wall_control_test.cpp`: recorded sensor examples, threshold boundaries, left-first priority, invalid-side handling, and joint encoder braking.
- `tests/check_movement_sequence.ps1`: extracts the actual movement functions and runs them with fake time, sensors, and motors; checks continuous seven-cell travel without intermediate stops, joint braking, front readings of 39/40/41 mm, obstacle detection during motion, d during motion, and cancellation after sensor faults.
- `tests/no_wall_test.cpp`: checks case 3 reference capture, the manual left/right calibration, both steering directions, integral correction, and anti-windup.
- `tests/movement_commands_test.cpp`: USB/WiFi command parsing.

These checks passed with g++ using C++11 and warnings as errors. They do not simulate mechanical dynamics. ESP32 compilation and physical verification remain pending.
