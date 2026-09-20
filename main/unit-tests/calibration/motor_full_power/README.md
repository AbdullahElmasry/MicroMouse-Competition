# Full-power motor calibration

Upload `motor_full_power.ino` and open USB Serial at 115200 baud. The robot remains braked after upload or reset.

- Send `f` to drive both motors forward at raw PWM 255 with no balance multipliers.
- Send `d` or `x` to brake both motors.

Raise the wheels for the first direction check. On the floor, provide a long clear path and keep the stop command ready. This sketch does not initialize the ToFs, MPU, encoders, or WiFi and has no automatic distance or obstacle stop.
