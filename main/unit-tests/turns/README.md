# 90-degree left/right rotation test

Open `turns.ino`, upload it, and keep the robot completely still during the 200-sample gyro calibration. No movement starts automatically.

Use USB Serial at 115200 baud, or connect to the printed ESP32 IP address on TCP port 23 using the existing Wi-Fi credentials:

- `l` / `L`: left 90 degrees.
- `r` / `R`: right 90 degrees.
- `d` / `D` / `x` / `X`: stop, including during a turn.

Other commands received during a turn are discarded, as before.

`turns.ino` handles Wi-Fi, USB commands, and logging. `rotation.h` and `rotation.cpp` contain the motor driving, raw MPU reads, calibration, and relative-angle PID rotation. They replace the old fixed-PWM/predictive-braking helpers and the dependency on `moving_forward/MpuYaw.h`.

The rotation follows the supplied working code:

- Left motor IN1/IN2: GPIO 25/26; right motor IN1/IN2: GPIO 14/27; enable: GPIO 23.
- I2C SDA/SCL: GPIO 21/22; MPU address: `0x68`.
- MPU6050 gyro range: +/-500 degrees/second, scale 65.5, with negated Z rate so right turns are positive.
- Startup calibration averages 200 stationary Z-rate samples. A 0.5-degree/second deadband prevents stationary noise from accumulating, with no software Kalman filter.
- The MPU6050 hardware gyro filter uses `DLPF_CFG=6` (5 Hz).
- PID gains: Kp = 5.62, Ki = 0, Kd = 0, with integral anti-windup.
- PWM floor: 85; cap: 150; equal and opposite wheel commands. The floor keeps enough torque to overcome drivetrain stiction during the final correction.
- Update interval: at least 5 ms; tolerance: 1.5 degrees for 10 consecutive stopped samples; timeout: 2000 ms.
- Stops set both input pins to zero, matching the supplied code.

Tune the constants near the top of `rotation.cpp`. Heading is integrated during each turn, and each requested angle is relative to its starting heading. The supplied loop's motor output within the tolerance band is preserved. Encoder stall checks and predictive braking are no longer used.

Wi-Fi/USB stop commands are checked throughout the loop. Failed MPU initialization refuses turns; a failed gyro read stops the motors and requires reinitialization/reset before further turns.

Run the host checks with `powershell -ExecutionPolicy Bypass -File unit-tests/turns/tests/run_tests.ps1` (requires `g++`). These exercise the actual rotation and command code with simulated Arduino/Wi-Fi/I2C hardware. An ESP32 build and physical left/right turn checks are still needed to verify hardware behavior.
