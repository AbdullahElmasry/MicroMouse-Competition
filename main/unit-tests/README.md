# Basic movement hardware tests

For hand-pushed encoder measurement over 144 cm and unfiltered MPU gyro/yaw checks, see [calibration](calibration/README.md).

These are standalone Arduino hardware tests, not automated unit tests. Run them in order before adding navigation. Each sketch is self-contained so it can be opened and uploaded directly; calibration changes are local to that sketch. The original main.ino is unchanged.

## Setup

Use the same ESP32 board configuration as the base sketch. Install the Pololu [VL6180X](https://github.com/pololu/vl6180x-arduino) and [VL53L1X](https://github.com/pololu/vl53l1x-arduino) libraries. Open one sketch, upload it, and open Serial Monitor at 115200 baud. Nothing moves until you send a command. Send one command at a time; send `x` to stop an active movement. Serial abort is checked between sensor reads, so it is not instantaneous during a blocking sensor read.

The pinout, motor polarity, brake behavior, balance factors, and encoder calibration come from main.ino. Initially check wheel directions with the wheels raised, then measure on the actual maze surface. Commands use the base sketch's left/right sign convention; confirm the physical direction before recording a pass. Single-channel encoders count pulses but cannot verify direction or detect wheel slip.

The MPU will share the ToF sensors' I2C bus (the board-default SDA/SCL used by Wire.begin()). Its model and library still need confirmation before adding its driver. These sketches do not yet measure heading with an IMU. An MPU can help measure heading drift; it does not directly verify traveled distance.

## 1. Moving forward

Open `moving_forward/moving_forward.ino`. Each cell is 180 mm. Mark a straight baseline with the start, every 180 mm interval, and the eight-cell target at 1440 mm (144 cm). Use the same fixed reference point on the chassis for the start and finish measurements. Send `f` for one continuous eight-cell move: 4800 ticks per wheel with the initial 600-tick calibration. There are no intermediate cell stops. Measure travel along the baseline, lateral drift, and final heading. Repeat five times from the same starting pose.

Encoder counts estimate distance; they are not an independent measurement. Verify the actual travel with a tape measure or ruler, record error as measured_mm - 1440, and record the measuring tool's resolution. Exact distance cannot be guaranteed: define an acceptable tolerance and check repeatability. A successful eight-cell test checks long-distance calibration and drift, but does not prove that individual 180 mm stop/start moves are accurate; those need a separate one-cell test afterward.

This isolates basic straight movement using the original speed 70 and right-motor factor 0.78. It does not apply the base sketch's side-wall steering or front-wall alignment, which would obscure motor imbalance. Side ToF readings are logged; -1 means invalid. Invalid front readings or a front distance below 100 mm stop the test. Use a setup with a valid front return and enough clearance for the full move; stopping for an obstacle is not successful cell completion.

Initial acceptance proposal: all five trials travel 1440 mm +/- 10 mm, lateral drift stays within 10 mm, and heading error stays within 3 degrees. These are bench targets to confirm, not competition requirements. Separately verify that placing a front obstacle stops movement and reports FRONT OBSTACLE. Sensor initialization failure must refuse forward movement.

## 2. Turns

Open `turns/turns.ino`. Mark the robot center and initial heading on paper. Send `l` for left 90 degrees, `r` for right 90 degrees, or `u` for right 180 degrees. The starting calibration is 620 ticks per 90 degrees per wheel. Repeat each command five times and measure angle and center displacement.

Initial acceptance proposal: angle error within 3 degrees for 90-degree turns and 5 degrees for 180-degree turns, with center displacement within 5 mm. Verify the commanded direction as well as the angle. Turns use encoders only; ToF readings do not protect the swept area, so provide clearance.

## 3. Full 360-degree turn

Open `turn_360/turn_360.ino`. Send `c` for clockwise or `a` for anticlockwise. Each commands one continuous rotation with 2480 ticks per wheel. Observe the entire rotation: matching the final heading alone cannot prove a full revolution. Repeat five times in each direction.

Initial acceptance proposal: one full revolution, final heading error within 5 degrees, and center displacement within 5 mm. A continuous 360-degree test also differs from four separate 90-degree turns; run four quarter turns afterward to check accumulated stop/start error.

## Results and calibration

Each run reports its stop reason, target, final left/right ticks, and elapsed time. ENCODER TARGET REACHED means both wheels reached their count targets; physical measurements decide whether the behavior passed. Each wheel brakes at its own target. A wheel without progress for 1500 ms ends the run. The total timeout is 120 seconds for eight-cell forward movement and 15 seconds for turns. Partial movement is a failed trial; reposition manually before retrying.

Record results in `results.csv` with target_mm = 1440 for forward trials. Change one parameter at a time. For consistent distance error across straight completed runs, an initial correction is new CELL_TICKS = round(old CELL_TICKS * 1440 / mean_measured_mm). For example, an average travel of 1400 mm gives round(600 * 1440 / 1400) = 617 ticks per cell. Rerun the test after updating the value; do not calibrate from obstructed, aborted, stalled, or curved runs. For consistent angular error, new QUARTER_TURN_TICKS = old QUARTER_TURN_TICKS * requested_angle / measured_angle. Correct motor direction and excessive drift before calibrating distance. Recheck both turn directions after any change. Copy confirmed values between sketches explicitly.

No board build or physical run has been performed in this workspace. Arduino CLI and PlatformIO were not available on PATH during setup.
