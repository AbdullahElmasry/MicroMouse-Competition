# 90-degree left/right turn test

Open `turns.ino`, upload it, and keep the robot completely still while the MPU calibrates. Use USB Serial at 115200 baud:

- `l`: simultaneous-wheel left 90-degree turn.
- `r`: simultaneous-wheel right 90-degree turn.
- `d` or `x`: brake and abort the active turn.

No turn starts automatically. Both wheels move throughout the turn in opposite directions. For a right turn, the left wheel moves forward while the right wheel reverses. A left turn mirrors those commands. MPU yaw ends the movement at the 90-degree target.

Both motors receive the same fixed raw PWM magnitude of 60 until the predicted braking point. Turn-specific motor factors are both 1.0, so neither side is reduced by the forward movement multipliers. The commands always have equal magnitude and opposite direction; encoder readings are diagnostic and never alter either motor command. PWM was reduced from 80 after hardware logs showed high yaw rates, 10-18 degrees of momentum, and large inconsistent wheel-count differences caused by slip.

This rotates around the wheel-axle center. Because that point is 43 mm behind the chassis center, the chassis center still follows an arc during rotation. Test with the wheels raised first to verify both directions, then test on the maze surface with clearance around the swept body.

## Control and safety

The tested Kalman-filtered MPU yaw supplies the stop threshold. `MPU_YAW_SIGN = -1` matches the verified mounting: physical right turns normalize positive and physical left turns normalize negative. The right target is +90 degrees and the left target is -90 degrees.

- Completion tolerance = +/-1.5 degrees.
- Fixed raw PWM magnitude = 60.
- Turn motor factors = 1.0 left and 1.0 right.
- Predictive brake lead = 4 degrees + 0.050 seconds of current directional yaw rate.

The predictive lead comes from hardware logs where braking at 89-90 degrees still produced another 10-18 degrees of rotation. For example, at 160 degrees/second the projected lead is 12 degrees, so braking begins around 78 degrees and the active brake absorbs the remaining motion. Telemetry prints `projected stop` for direct tuning against the final yaw.

Both encoders remain active for stall detection and diagnostics. They do not decide the final angle. Either wheel reaching 900 ticks stops the turn as a safety fault. Either wheel having no encoder progress for 1000 ms, an unhealthy MPU, or a five-second timeout also brakes both motors.

Telemetry reports both encoder counts, normalized yaw, yaw rate, projected stopping angle, and the equal-magnitude signed motor commands. At completion it reports yaw when braking started and yaw after 150 ms of braking, which shows whether the prediction matched the actual momentum.

## Hardware tuning

Run at least five left and five right turns and measure the physical result with the same chassis reference line. Record final yaw, physical angle, left/right counts, and center displacement. Adjust one setting at a time:

- Consistent overshoot in both directions: increase `TURN_BASE_BRAKE_LEAD_DEGREES` or `TURN_BRAKE_LOOKAHEAD_SECONDS`.
- Consistent undershoot: reduce the base lead or lookahead.
- One direction differs from the other: compare encoder counts and inspect reverse-direction motor behavior before changing the shared PWM.

The first acceptance target is physical angle error within 3 degrees and center displacement within 5 mm in both directions. MPU yaw is feedback, not an independent physical measurement.
