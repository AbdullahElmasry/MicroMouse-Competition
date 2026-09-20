# Seven-cell forward test with wall control and distance PID

Send `start` over WiFi TCP port 23 or USB Serial at 115200 baud. The robot moves seven 180 mm cells, braking both motors and pausing 500 ms between cells. After cell seven it stays stopped. Send `d` during movement or a pause to cancel the remaining run. A new start begins a new seven-cell sequence, not the remainder of an interrupted run. No automatic startup movement occurs. Blocking sensor reads and WiFi writes can delay command handling or extend pauses.

## Why the controller changed

The earlier PID applied the opposite motor command change to the hardware-tested base sketch. For a small left distance, main.ino reduces the named left motor command to 30 and keeps the right at 70; the PID increased the left and reduced the right. The old test also stopped wheels independently, potentially rotating the robot while one wheel finished its target. This version follows the working base sketch's command convention and brakes both together when either wheel reaches its calibrated limit.

## Active movement logic

| Side readings | Left command | Right command |
|---|---:|---:|
| Valid left below 40 mm | 30 | 70 |
| Otherwise, valid right below 50 mm | 70 | 30 |
| Otherwise | 70 | 70 |

The left condition takes priority if both thresholds trigger. These are pre-balance commands: factors remain left 1.0 and right 0.78. The table describes cruise speed. During the approach, the distance PID reduces the base command from 70 toward 35; a wall correction still slows the same motor to 30. The movement loop delay is 10 ms, as in main.ino. Motor pins, polarity, and braking outputs are unchanged. The thresholds intentionally remain asymmetric because these are the values from the working base sketch; lateral correction remains threshold-based. A separate encoder-distance PID controls forward approach speed.

Tune `FORWARD_SPEED`, `WALL_SLOW_SPEED`, `LEFT_WALL_THRESHOLD_MM`, `RIGHT_WALL_THRESHOLD_MM`, `LEFT_FACTOR`, and `RIGHT_FACTOR` in the sketch. `ForwardWallControl.h` contains the steering decision and encoder stopping rule. The old `WallCentering.h` and its tests remain available for reference but are not included by this sketch. The last manually tuned PID values observed before this change were Kp=0.2, Ki=0, Kd=0, correction limit=40; those old lateral PID gains are not active.

Each cell uses a new encoder baseline and limits of 623 left / 619 right ticks, derived from the two seven-cell hand-pushed measurements. Both wheels brake when either limit is reached, just like main.ino's OR condition. This intentionally differs from requiring both wheel targets. It can shorten travel if wheel progress differs, so measure each cell and total distance instead of assuming 180 mm or 1260 mm is exact.

## Reducing cell overshoot

`CellApproachControl.h` adds encoder-distance PID without reversing the working wall steering. It uses the smaller remaining count to either wheel's braking threshold, matching the rule that both motors stop at the first encoder limit. Outside the final 300 ticks (about 87 mm), the base PWM is 70. Inside that zone, PID reduces base PWM toward a floor of 35. The wall controller can still reduce the corresponding side to 30. All commands are before motor balance factors.

Edit `APPROACH_SETTINGS` in the sketch:

- Kp = 0.24 PWM/tick; Ki = 0.01 PWM/(tick*second); Kd = 0.01 PWM*second/tick. These are initial, unverified physical tuning values.
- `APPROACH_SLOWDOWN_TICKS = 300`. A larger zone permits earlier deceleration, but speed is also determined by the PID output.
- `MIN_APPROACH_PWM = 35`. Lower cautiously if the robot still overshoots; both motors must remain able to roll under load.
- `BRAKE_LEAD_TICKS = 10`, requesting braking approximately 2.9 mm before the nominal cell endpoint. Effective thresholds are 613 left / 609 right ticks. Increase gradually; this changes the braking point, not the distance calibration.

Keep integral small: this is an endpoint approach, not a speed regulator. PID state resets at each cell, output is bounded, and integral accumulation is suppressed at the limits. Encoder crossing triggers joint braking regardless of PID state.

Each cell reports counts at braking and counts added during the following 100 ms. This separates target-crossing delay from rolling after braking. Measure actual travel too: tire slip and motion after the 100 ms reporting window are not captured by that difference. Blocking sensor/network calls can delay endpoint checks. The change should reduce approach speed; exact 180 mm stopping still needs hardware tuning.

## Sensors and stop conditions

ToF startup matches the working calibration/tof_readings sketch: Wire.begin(), 10 ms shutdown, left/right/front wake order with 50 ms delays, addresses 0x30/0x31/0x32, timeouts of 200 ms after address assignment, and front Medium mode with startContinuous(30). Status is reported after initialization rather than aborting at intermediate checks.

Front obstacle-distance stopping remains disabled as requested. Invalid front readings, both side readings invalid, encoder stall, cell timeout, or d cancel the run. A single invalid side is excluded from steering; the valid side can still request correction. If neither valid side is below its threshold, the robot uses 70/70. Invalid readings are shown as -1. The old side-distance-sum guard, 8 mm side stop, PID deadband, and PID sample-gap check are not used. There is no MPU feedback or front-wall alignment in this forward-only test.

## Verification

Start parallel to a straight corridor and keep d available. At readings 32/68 mm, telemetry must show commands 30/70. At 47/49 mm it must show 70/30. At 44/55 mm it must show 70/70. Verify the physical corrections match the behavior of your base sketch. Record final heading, travel per cell, and left/right counts at each stop. Run seven cells only once the first cell behaves correctly.

Host checks:

- `tests/cell_approach_test.cpp`: distance-PID deceleration, output limits, reset, anti-windup, and brake-lead arithmetic.
- `tests/forward_wall_control_test.cpp`: recorded sensor examples, threshold boundaries, left-first priority, invalid-side handling, and joint encoder braking.
- `tests/check_movement_sequence.ps1`: extracts the actual movement functions and runs them with fake time, sensors, and motors; checks seven cells, pauses of at least 500 ms, joint braking, d during motion/pause, and cancellation after sensor faults.
- `tests/movement_commands_test.cpp`: USB/WiFi command parsing.

These checks passed with g++ using C++11 and warnings as errors. They do not simulate mechanical dynamics. ESP32 compilation and physical verification remain pending.
