# Flood-fill exploration firmware

Open main.ino in Arduino IDE and upload it to the ESP32. This firmware uses
the forward and MPU6050 rotation controllers from the right-hand demo. It
implements the exploration run only; there is no speed-run code.

Keep the robot still during startup calibration. When Wi-Fi connects, Serial
prints a URL such as:

    WEB | open http://10.234.131.104/

Open that address from a device on the same Wi-Fi network. The page shows:

- the robot's current cell and cardinal heading;
- visited cells and every known wall;
- the current flood-fill distance in each cell;
- the same diagnostic log written to USB Serial and TCP port 23;
- Start, Stop and Reset-map controls plus a command input.

The command input accepts s/start, d/stop, and r/reset. Start resumes the
stored pose and map after a clean stop. Reset stops an active run and sets the
stored pose to (0,0) facing north. Place the robot there physically before
starting again.

## Map convention

The maze is 16 by 16 cells. The starting cell is (0,0), north increases Y,
and east increases X. The four goal cells are (7,7), (7,8), (8,7) and (8,8).
If the physical starting direction is not north in this coordinate system,
rotate the maze convention or change robotHeading before running.

At each cell the firmware confirms six front/left/right samples, stores the
walls in absolute directions, recalculates flood distances, turns toward the
best reachable neighbor, verifies that edge again, and moves forward. Unknown
internal edges are considered traversable until measured. Outer boundary walls
cannot be cleared by a sensor reading.

On a first visit, each cell is scanned at rest. On a previously mapped straight
route, the robot can drive through up to three known cells without braking at
their internal boundaries. It brakes at the end of the run and scans there.
The front emergency stop, encoder stall check and manual Stop remain active
throughout the rolling run. A premature brake or insufficient encoder travel
leaves the pose uncertain and stops exploration.

After an encoder arrival and heading correction, the robot checks the front
distance again. If a front wall is 91-120 mm away, it advances at 100 PWM to
the 90 mm brake trigger before recording the new cell. The brake now starts
ahead of the desired 60 mm stopped distance because the recent log showed
10-19 mm of additional approach after triggering. A wall at 100 mm or less
before minimum cell travel, or a nearby wall with large yaw, also brakes early.
This short adjustment is limited to 120 mm and three seconds; a lost front
reading or stall stops the run with an uncertain pose.

The no-wall mode uses a 120 PWM base floor and a 100 PWM motor floor while the
front reading is above 120 mm. It drops back to a 100 PWM base near a front
wall. MPU yaw correction is capped at 25 PWM. These changes need a physical
run to check steering and stopping distance.

The encoder distance conversion uses 597 ticks per 180 mm cell on each wheel,
the median of five one-cell measurements. The 528-tick left trial is an outlier.

The front scan needs more than 120 mm to call a path open. If the first start
scan finds no exit, the robot retries twice before committing those walls.
The forward motion
controller still uses its tested 100 mm wall-approach threshold. A front-wall
brake counts as a full cell only after at least 120 mm of encoder travel on
each wheel; this rejects the 40-67 mm false arrivals seen in the run log.
Once the robot successfully crosses an edge, later contradictory ToF scans
cannot draw a wall across that edge.

Coordinates change only after a complete cell move and heading correction.
A failed or interrupted move or turn stops exploration and marks the pose
uncertain. The robot will refuse Start until it is physically returned to the
start pose and the map is reset. A scan fault while stationary leaves the pose
usable, so Start can retry it.

## Useful debug lines

- POSE: stored coordinates, heading, flood value and completed moves.
- SENSORS: confirmed front, left and right classifications and distances.
- MAP: absolute cell update and any contradiction with a previous reading.
- FLOOD: chosen direction, next coordinates, distance and visited state.
- TURN PLAN / MOVE PLAN: intended physical action and number of cells.
- MOVE START / MOVE END: continuous segment length, encoder travel and yaw.
- REPLAN: a post-turn scan found the selected edge blocked.
- FAULT: movement stopped and coordinates were not advanced.

Run the host map test with:

    powershell -NoProfile -ExecutionPolicy Bypass -File tests/run_tests.ps1
