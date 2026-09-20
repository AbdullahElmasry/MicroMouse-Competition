# Basic movement hardware tests

For sensor initialization failures or checking all three distances independently, run the [three-ToF diagnostic](calibration/tof_readings/README.md).

For hand-pushed encoder measurement over 144 cm and unfiltered MPU gyro/yaw checks, see [calibration](calibration/README.md).

These are standalone Arduino hardware tests, not automated unit tests. Run them in order before adding navigation. Each sketch is self-contained so it can be opened and uploaded directly; calibration changes are local to that sketch. The original main.ino is unchanged.

## Setup

Use the same ESP32 board configuration as the base sketch. Install the Pololu [VL6180X](https://github.com/pololu/vl6180x-arduino) and [VL53L1X](https://github.com/pololu/vl53l1x-arduino) libraries. Open one sketch, upload it, and open Serial Monitor at 115200 baud. The forward sketch waits for `start` over WiFi TCP (port 23) or USB Serial, and stops on `d`. Turn sketches retain their existing commands and `x` stop. Serial abort is checked between sensor reads, so it is not instantaneous during a blocking sensor read.

The pinout, motor polarity, brake behavior, balance factors, and encoder calibration come from main.ino. Initially check wheel directions with the wheels raised, then measure on the actual maze surface. Commands use the base sketch's left/right sign convention; confirm the physical direction before recording a pass. Single-channel encoders count pulses but cannot verify direction or detect wheel slip.

The MPU will share the ToF sensors' I2C bus (the board-default SDA/SCL used by Wire.begin()). The tested calibration baseline uses MPU6500_WE, address 0x68, SDA 21, SCL 22, and Kalman-filtered gyro Z. These sketches do not yet measure heading with an IMU. An MPU can help measure heading drift; it does not directly verify traveled distance.

## 1. Moving forward

Open `moving_forward/moving_forward.ino`. Send `start` to move seven 180 mm cells, automatically pausing 500 ms between cells. It remains stopped after the seventh cell. Send `d` during movement or a pause to cancel the remaining sequence. Each move targets 623 left ticks and 619 right ticks, using the original seven-cell calibration (4373/4344 and 4345/4325). If interrupted, the next start requests a new seven-cell sequence from the stopped position; reposition before retrying if necessary.

Mark 180 mm intervals and measure each stop using the same chassis reference point. Repeat individual moves and check accumulated error over the seven-cell sequence (1260 mm total). Encoder completion is not an independent distance measurement: record measured distance minus 180 mm for each cell.

Forward movement now follows the working main.ino threshold steering: left below 40 mm commands 30/70, otherwise right below 50 mm commands 70/30, otherwise 70/70. It brakes both wheels when either calibrated per-cell encoder limit is reached. The old wall-centering PID and corridor-sum rejection remain inactive. A separate encoder-distance PID now slows the approach over the final 300 ticks, from cruise PWM 70 toward 35, to reduce overshoot. The front obstacle-distance stop remains disabled. See the [forward movement guide](moving_forward/README.md) for the comparison, active settings, fault handling, and tests.

Initial acceptance proposal: all five one-cell trials travel 180 mm +/- 5 mm, lateral drift stays within 10 mm, and heading error stays within 3 degrees. These are bench targets to confirm, not competition requirements. Sensor initialization failure must refuse forward movement.

## 2. Turns

Open `turns/turns.ino`. Mark the robot center and initial heading on paper. Send `l` for left 90 degrees, `r` for right 90 degrees, or `u` for right 180 degrees. The starting calibration is 620 ticks per 90 degrees per wheel. Repeat each command five times and measure angle and center displacement.

Initial acceptance proposal: angle error within 3 degrees for 90-degree turns and 5 degrees for 180-degree turns, with center displacement within 5 mm. Verify the commanded direction as well as the angle. Turns use encoders only; ToF readings do not protect the swept area, so provide clearance.

## 3. Full 360-degree turn

Open `turn_360/turn_360.ino`. Send `c` for clockwise or `a` for anticlockwise. Each commands one continuous rotation with 2480 ticks per wheel. Observe the entire rotation: matching the final heading alone cannot prove a full revolution. Repeat five times in each direction.

Initial acceptance proposal: one full revolution, final heading error within 5 degrees, and center displacement within 5 mm. A continuous 360-degree test also differs from four separate 90-degree turns; run four quarter turns afterward to check accumulated stop/start error.

## Results and calibration

Each run reports its stop reason, target, final left/right ticks, and elapsed time. The forward message CELL ENCODER LIMIT REACHED means at least one wheel reached its per-cell limit; physical measurements decide whether the behavior passed. The forward test brakes both wheels when either per-cell target is reached; the turn tests retain their own stopping behavior. A wheel without progress for 1500 ms ends the run. The total timeout is 15 seconds for one-cell forward movement and 15 seconds for turns. Partial movement is a failed trial; reposition manually before retrying.

Record results in `results.csv` with target_mm = 180 for one-cell forward trials. Change one parameter at a time. For consistent distance error across straight completed runs, an initial correction is new ticks_per_cell = old ticks_per_cell * 180 / mean_measured_mm, separately for each wheel. Current values are approximately 622.7143 left and 619.2143 right ticks per cell; round only the final movement target. Rerun the test after updating the value; do not calibrate from obstructed, aborted, stalled, or curved runs. For consistent angular error, new QUARTER_TURN_TICKS = old QUARTER_TURN_TICKS * requested_angle / measured_angle. Correct motor direction and excessive drift before calibrating distance. Recheck both turn directions after any change. Copy confirmed values between sketches explicitly.

No board build or physical run has been performed in this workspace. Arduino CLI and PlatformIO were not available on PATH during setup.
