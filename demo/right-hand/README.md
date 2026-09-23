# Right-hand follower demo

Open `right-hand.ino` in Arduino IDE and upload to the same ESP32 used for the
forward and rotation tests. Keep all the `.h` and `.cpp` files in this folder.
Dependencies: ESP32 Arduino core, Pololu VL6180X and VL53L1X libraries.

Place the robot at a cell center, facing along the maze, and keep it still
during both MPU calibrations. Connect to its printed IP on TCP port **23**,
or use USB Serial at **115200 baud**. The Wi-Fi credentials are copied from the
tested forward module in `MoveForward.cpp`; transmit power remains 8.5 dBm.

- `s` or `start`: start following. It continues until stopped or a motion fails.
- `d`: stop during a scan, movement, turn, or settling pause.
- At each center: choose **right, straight, left, then U-turn**, in that order.
- Each decision advances one nominal **180 mm cell**. A front wall can stop it
  earlier, or extend the encoder distance slightly to reach the 60 mm reference.
- A valid front-wall brake counts as cell arrival. The robot corrects its heading
  in place, then scans and continues, including a U-turn at a confirmed dead end.
- A U-turn uses two tested right 90-degree turns, each with its own timeout.
- There is no map, goal detection, or maze-solving completion condition.

## Tested controllers retained

This is a local snapshot of the working files in `main/unit-tests/moving_forward`
and `main/unit-tests/turns`, including the latest tuning present when this demo was
created. Later edits to those tests do not automatically update this demo.

`rotation.h` is copied unchanged; `rotation.cpp` only has revised logging: physical right uses
`turnDegrees(-90)` and left uses `turnDegrees(90)`. The rotation PID remains
15.9 / 0 / 1.3, minimum PWM 95, maximum 150, tolerance 1 degree, timeout 2 seconds.

`MoveForward.h` is copied unchanged. The forward steering, yaw estimator,
encoder calibration, motor mapping and braking are retained. A valid front-wall
brake is now a normal arrival result rather than a failed move.
The demo's forward distance is one cell instead of seven: nominal encoder
targets are **616 left / 617 right**, with the existing 10-tick brake lead.
At this endpoint a nearby front wall takes priority over encoder distance:
the robot finishes approaching the wall before declaring arrival.
No-wall tuning is MPU Kp **10**, Kd **0.5**, max **25**, weight **80%**, encoder
max **30**, and yaw sign **-1**. One-wall and two-wall tuning is also copied.

The added adapter shares the forward module's USB/Wi-Fi commands and logging
with rotation. Controllers run sequentially. Forward yaw and controller state
reset before each cell, after the turn. Stationary scans can read side sensors
even when the front wall is within the front stopping distance.

## Opening decisions

`config.h` contains the demo's opening thresholds: sides greater than **100 mm**,
front greater than **100 mm**, confirmed in all **three** fresh stationary
samples. Right clearance retains the tested 10 mm sensor-inset correction.
These thresholds assume the robot starts and stops at cell centers; check them
against the printed `PATHS` distances for your maze and chassis.
Healthy side-sensor no-target readings retain the tested open-space handling.

After a navigation turn the demo verifies the newly facing path before moving.
A failed sensor read, turn timeout, encoder stall or manual stop still ends the
run. A valid front distance at or below the 61 mm trigger brakes forward travel and counts
as arrival, even before the nominal encoder distance. Logs retain the actual
encoder travel; this count does not claim an exact geometric cell center.

Front range status 2 (insufficient signal) and status 4 (no target in range)
are treated as 200 mm of open space during scans and movement. Status 6 remains
usable at its reported distance. I2C errors, read timeouts and range statuses
that indicate a hardware or wrapped-target fault remain sensor faults.

The reported 1.6-degree result was for your standalone forward test. Combined
turns and one-cell stopping still require a physical trial; encoder distance
and headings can accumulate error over multiple cells.

## Checks

The complete sketch compiled successfully for `esp32:esp32:esp32` with the
installed ESP32 core 3.3.11. The new arrival and heading-alignment behavior still
needs a physical trial on the robot.

Run `powershell -NoProfile -ExecutionPolicy Bypass -File tests/run_tests.ps1`
from this folder for navigation tests (requires g++). They exercise the actual
sketch with simulated motion/sensors: priorities, both turn signs, U-turns,
three-sample confirmation, post-turn clearance, failed moves, stop requests,
startup and repeated cells. Additional checks exercise the actual forward loop
for front-wall arrivals versus faults and manual stops, and verify heading
correction in both directions followed by dead-end recovery.

## Compact tuning logs

USB and Wi-Fi show the same lines. Send `s` to start automatic following;
no direction commands are needed. Configuration prints once per run.

- `CELL`: next cell number and completion.
- `PATHS`: OPEN/WALL with minimum F/L/R distances across the three samples.
  Any blocked sample makes a path WALL; thresholds print at START.
- `DECISION`: RIGHT, STRAIGHT, LEFT or U-TURN. CHECK marks the post-turn scan.
- `TURN END`: relative target/measured angle, remaining error and duration.
- `MOVE`: every 200 ms, wall case, corrected F/L/R distances, yaw, motor PWM
  and wall/MPU/encoder corrections before the 80/20 no-wall blend.
- `MOVE END`: stop reason, encoder-estimated travel per wheel, final yaw and
  duration. Encoder travel is not an independent ground-distance measurement.
- `WALL APPROACH`: encoder endpoint reached, but continuing toward a nearby wall.
- `WALL REFERENCE`: desired gap and the front reading that triggered braking.
- `ARRIVAL`: encoder target or front-wall brake.
- `ALIGN`: remaining yaw correction, followed by the rotation controller's
  `TURN END` report. `CELL ... reached and aligned` precedes the next decision.

Angle logs use positive = right, negative = left. Forward yaw is relative to
that cell's starting heading; turn angles are relative to that turn's start.
Neither is an absolute maze heading. The rotation API remains positive-left;
only its displayed signs are converted. Raw/filtered duplicates, per-sample
scan spam, pin listings, repeated encoder counts and successful I2C setup
messages are removed. Errors and stop reasons remain visible. The movement log interval remains 200 ms.
## Minimum moving speed

Build `right-hand rolling-floor-v2` raises the demo's approach base floor from
35 to 70 PWM and the one-/two-wall steering motor floor from 30 to 60 PWM (historical v2 settings).
This addresses the recorded one-wall stall at roughly 128/133 mm, where the
commands had dropped to 30-39 PWM. Version 6 now uses 100/85 in all wall modes.
These are commanded PWM values before the existing motor balance factors.
The startup MOVE CONFIG line prints the floors for checking uploaded firmware.

The controller still brakes both wheels at the encoder endpoint or on stop,
front obstacle, sensor failure, or stall. The 1.5-second stall timeout and
front-wall stop remains active. Gains, calibration and rotation are unchanged.
Check the next cell's encoder travel and final yaw: the higher approach speed
may change stopping distance and needs a physical trial.
Build `right-hand openings-100mm-v3` uses >100 mm for every opening decision;
exactly 100 mm is blocked. Decisions occur at the starting cell and after each
completed cell. During travel the sensors only guide steering or stop motion;
they do not trigger a new turn. Post-turn checks verify the already chosen path.
A valid front-wall brake ends forward travel and counts as arrival. Opening
detection does not guarantee 180 mm of clear travel ahead.

## Scan failures

Build `right-hand scan-diagnostics-v4` retries a failed stationary sample up to
three times, waiting 50 ms between attempts while servicing stop commands.
Three valid confirmation samples are still required for the path decision.
Moving sensor faults still stop immediately; invalid front readings never
become an open path. Repeated sensor faults still prevent movement.

`SCAN ERROR` identifies the failed sensor/operation, code and available ranges
(-1 means unavailable). For `FRONT range rejected`, the code is the VL53L1X
range status: 1 = sigma failure, 2 = insufficient signal, 4 = out of bounds,
5 = hardware failure, 6 = wrap check not yet done, 7 = wrapped target.
For messages mentioning I2C, the code is the I2C transaction result instead.
`SCAN | recovered` means the next attempt obtained a valid sample. A stop
command is reported separately from a sensor fault.

The old generic failure message cannot establish which sensor caused a failure.
If v4 still refuses to move, use its `SCAN ERROR` lines to diagnose that sensor;
retries cannot repair a persistent hardware, signal or communication problem.

## Arrival and independent heading correction

Build `right-hand arrival-yaw-v5` separates distance completion from orientation.
After either encoder arrival or a valid front-wall brake, the wheels stop,
settle, and the forward MPU supplies the remaining error relative to the heading
at the start of that cell. If it exceeds `ARRIVAL_YAW_TOLERANCE_DEG` (1 degree),
the tested rotation controller commands an in-place correction. There is no
additional forward-distance command during alignment. Physical rotation can
still involve wheel slip; the MPU does not measure translation.

The forward yaw is right-positive, while the rotation API is left-positive:
a +5 degree forward yaw error requests `turnDegrees(+5)` to correct left.
After alignment, a fresh stopped scan chooses right, straight, left or a U-turn.
The forward yaw reference resets only when the next cell move begins.

A front-wall brake does not cancel heading correction. Manual `d`, invalid MPU
data, a failed turn, or a sensor/encoder fault still stops the demo. Front-wall
arrival is accepted only from a valid front reading, with no side communication
fault. Invalid readings are never converted to arrivals.

## Current rolling torque settings

Build `right-hand rolling-torque-v6` uses a minimum approach base of **100 PWM**
and a minimum moving motor command of **85 PWM**, before balance factors, in
all wall modes. The previous 70/60 floors produced two stalls and a 9.05-second
cell in the robot log. Cruise remains 140; steering gains, rotation, arrival
alignment and front-wall arrival handling are unchanged. The no-wall floors
reference the same constants so modes cannot fall back to the old values.

MOVE lines now include encoder-estimated `travel L/R` in mm. If PWM stays high
but encoder travel barely advances, inspect drivetrain load, wheel contact and
supply voltage rather than assuming another gain increase will solve it.
The existing 1.5-second no-tick stall check remains active; individual ticks
reset that timer, so it does not detect every creeping/slipping condition.
The 100/85 setting is a hardware trial and may increase stopping overshoot.

## Front-wall distance reference

Build `right-hand wall-reference-60mm-v9` checks a fresh front reading at the encoder
endpoint. If it is greater than 100 mm, the path is open and encoder arrival
finishes normally. If it is between 61 and 100 mm, forward motion continues
at the 100 PWM approach base with the existing steering and 85 PWM motor floor,
until a valid front reading reaches 61 mm, the largest whole-millimetre reading
inside the requested 60 mm +/-3% band (58.2-61.8 mm). A front wall at or below
61 mm also brakes immediately before the encoder endpoint.

This replaces an encoder-only stopping position with a front-wall reference
where a wall is present. It does not skip cells to chase a distant wall. Each
next move already starts from new encoder baselines; calibration and cell size
are not rewritten. The physical reference corrects longitudinal stopping drift
against that wall, not lateral position or a global maze coordinate.

Extra approach is limited by `FRONT_APPROACH_MAX_EXTRA_MM` (120 mm per wheel)
and `FRONT_APPROACH_TIMEOUT_MS` (3 seconds). Exceeding either limit is a fault,
not successful arrival. Manual stop, remaining sensor faults and encoder stall
remain active. After front-wall arrival the existing in-place yaw alignment
and right-hand decision sequence continue normally.

The 60 mm value is a sensor trigger target, not a guarantee that the final
physical clearance is within 3%: sampling, mounting, inertia and yaw alignment affect
the measured gap. `WALL REFERENCE` reports the reading at braking, not a
separately measured final resting gap. Check actual stopping clearance on the
robot before relying on this as a precise reference.
