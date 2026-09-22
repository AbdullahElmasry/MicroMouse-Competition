# Motor output to encoder mapping

Upload `motor_mapping.ino`. Lift the wheels and open USB Serial at 115200 baud.
Send `s` (no Enter required):

1. Output labelled LEFT (GPIO 25/26) runs forward at raw PWM 140 for 3 seconds.
2. Both outputs brake for 1 second.
3. Output labelled RIGHT (GPIO 27/14) runs forward at raw PWM 140 for 3 seconds.
4. Both outputs remain braked until another `s`.

Send `d` to cancel immediately, including during the pause. Each powered step
prints encoder deltas for GPIO 33 (L) and GPIO 35 (R). Observe which physical
wheel rotates and whether it moves forward. Identify left/right facing in the
robot's forward direction. Send both observations and the printed deltas.

The sketch uses the forward sketch's pin/polarity mapping without PID or motor
factors. It does not initialize WiFi, ToFs or the MPU. It waits for `s` after reset.
