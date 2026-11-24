# MPU6050

Gyro-Z driver over [i2c_bus](../i2c_bus/README.md), returning offset-corrected yaw rate in rad/s.

Flow: `init()` → `calibrateGyroZ()` while stationary → `readGyroZ()` → odometry.

`init()` preserves an established gyro bias during recovery. Firmware retries a
failed initial calibration only while stopped; it does not recalibrate after a
runtime outage. Encoder-only odometry bridges missing gyro samples.

### Documents

- [Datasheet (PDF, rev. 3.1)](https://cdn.sparkfun.com/datasheets/Components/General%20IC/PS-MPU-6000A.pdf)
- [Register map (PDF, rev. 4.0)](https://cdn.sparkfun.com/datasheets/Sensors/Accelerometers/RM-MPU-6000A.pdf)
