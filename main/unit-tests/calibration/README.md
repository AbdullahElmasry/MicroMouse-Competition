# Manual calibration

For a raw full-power direction and motor-strength check, use [`motor_full_power/motor_full_power.ino`](motor_full_power/motor_full_power.ino).

Open and upload one sketch at a time from its own folder. Separate sketch folders prevent Arduino from combining two setup()/loop() definitions. Serial Monitor: 115200 baud. Both sketches hold motor-enable pin 23 LOW and the motor inputs LOW. Confirm the actual driver lets the wheels roll freely; do not force a braked drivetrain.

## Encoder ticks over eight cells

Open `encoder_ticks/encoder_ticks.ino`. Mark a straight 1440 mm track (8 x 180 mm), measured using the same reference point on the chassis at both ends.

1. Position the robot at the start, wheels on the normal maze surface.
2. Send `r` to zero both counts and begin recording.
3. Push slowly and straight forward for exactly 1440 mm. Avoid wheel slip, lifting, and reversing.
4. Stop at the end mark and send `s` to freeze the counters and print separate left/right ticks per cell and ticks per millimeter.
5. Repeat at least five times. Average each wheel's completed-trial counts separately, then divide by 8 for that wheel's ticks per cell. Large variation means the measurement or encoder signals need checking before adopting calibration.

`p` prints counts without stopping. A new `r` discards the previous trial. Single-channel encoders cannot determine direction: reversing adds counts and invalidates the trial. These measurements calibrate rolling distance, not motor speed balance. Existing motion sketches currently use one shared target; separate left/right results are reported here and are not automatically applied. Powered movement still needs verification for slip and braking distance.

## Raw MPU yaw

Open `mpu_yaw/mpu_yaw.ino`. Uses Wire only, on the same board-default SDA/SCL bus as the ToFs. The model is not yet confirmed: this first driver supports the MPU-6050 register map and requires WHO_AM_I = 0x68 at address 0x68 or 0x69. Other detected identities are printed but not configured; report the model or printed identity to add the matching driver.

An MPU-6050 has no raw yaw-angle register. This sketch prints raw gyro Z, angular rate, and unwrapped relative yaw calculated as yaw += rate * elapsed_seconds. It applies no software smoothing, sensor fusion, DMP, or bias subtraction. DLPF_CFG=0 selects the widest gyro bandwidth; the physical sensor still has finite bandwidth. Register settings follow the manufacturer's [MPU-6000/MPU-6050 register map](https://www.invensense.com/wp-content/uploads/2015/02/MPU-6000-Register-Map1.pdf).

Keep the sensor level with its Z axis vertical. Send `r` at a marked starting heading. First leave it still for 30 seconds and record yaw drift. Then zero and manually rotate through measured +90, -90, 180, and 360 degree angles, resetting between trials. Compare the printed yaw with physical angle marks and repeat each direction. Turn below 250 degrees/second. Positive/negative rotation follows the sensor's axis orientation, not an assumed robot turn direction.

Gyro bias causes yaw drift even when stationary; this deliberately uncorrected test exposes it. The reported angle is relative, not compass heading, and Z integration is only appropriate for level rotation. I2C failures or large sampling gaps invalidate the run and require `r`. The serial output is reduced to 20 Hz for readability; valid samples are integrated at approximately 200 Hz without averaging. No hardware or board compilation has been performed here.
