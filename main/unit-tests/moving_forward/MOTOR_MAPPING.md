# Verified physical motor mapping

The isolated output test confirmed GPIO 25/26 drives the physical right wheel.
The forward sketch now uses these physical labels:

| Side | Motor inputs | Forward outputs | Encoder | Factor |
| --- | --- | --- | --- | --- |
| Left | 27/14 | 0/PWM | 33 | 0.99 |
| Right | 25/26 | PWM/0 | 35 | 1.00 |

Factors were relabelled with their original pin pairs, not retuned. The build
banner is `physical-motor-map-v3`. This correction applies to moving_forward;
the diagnostic motor_mapping sketch intentionally retains its original output
labels so its observed sequence remains reproducible. Other sketches have not
been migrated to this mapping.

Case 3 slows the physical wheel ahead in calibrated encoder travel. Wall
controllers slow the right wheel to turn right (away from a close left wall),
and slow the left wheel to turn left. MPU positive-right yaw requests a leftward
correction by slowing the left wheel. The PID gains and yaw sign are unchanged.

Run `tests/check_motor_mapping.ps1` to check the actual sketch's pin writes,
forward polarity, factor assignment and braking using a host stub.
