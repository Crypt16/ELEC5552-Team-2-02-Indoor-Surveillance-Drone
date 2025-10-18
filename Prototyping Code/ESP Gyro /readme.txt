ESP32 T-Display S3 + MPU-6050 (Reduced Yaw Drift)
=================================================

Overview
--------
This program reads data from an MPU-6050 IMU module and displays roll, pitch, and yaw
on the T-Display S3’s TFT screen. It applies a complementary filter to fuse accelerometer
and gyroscope data for stable roll and pitch readings, while using kinematic compensation
to reduce yaw drift. A button (IO14) allows the user to reset yaw to zero in real time.

Key Features
-------------
- Reads raw acceleration and gyroscope data directly from the MPU-6050 over I2C.
- Applies a complementary filter (98% gyro / 2% accel) for smooth tilt estimation.
- Computes yaw rate using full rotational kinematics to reduce cross-axis drift.
- Includes gyro bias calibration on startup and slow bias trimming during operation.
- Displays roll, pitch, and yaw on the T-Display S3 using the TFT_eSPI library.
- Button on IO14 (active LOW) zeros yaw during operation.

Hardware Setup
--------------
Board:        ESP32-S3 T-Display  
IMU Module:   GY-521 (MPU-6050)  
Connections:
  SDA  -> GPIO 21
  SCL  -> GPIO 16
  BTN  -> GPIO 14 (active LOW, internal pull-up)
Power: 3.3V supply to MPU-6050  
I2C Address: Auto-detects 0x68 or 0x69

Required Libraries
------------------
- Wire (standard)
- SPI (standard)
- math.h (standard)
- TFT_eSPI (configured for T-Display S3)

Main Code Flow
--------------
1. **Setup**
   - Initializes the TFT display.
   - Starts I2C and probes for MPU-6050.
   - Configures sample rate (100 Hz), DLPF (~42 Hz), and measurement ranges.
   - Calibrates gyro over 1500 samples (sensor must remain still).
   - Seeds roll and pitch angles from accelerometer data.

2. **Loop**
   - Reads accelerometer and gyroscope data.
   - Converts to physical units (g, °/s).
   - Filters roll/pitch using complementary filter.
   - Estimates yaw rate from body rates using roll/pitch compensation.
   - Integrates yaw over time, subtracting stored bias.
   - Detects stationary periods and trims biases slowly.
   - Checks button input to zero yaw.
   - Refreshes TFT screen every 100 ms with updated orientation.

Controls
--------
- Press IO14 to set the current yaw as 0°.
- Yaw display shows "Yaw*" with a star to indicate it’s gyro-based (will drift slowly).

Tuning Parameters
-----------------
- Complementary filter: alpha = 0.98
- Gyro deadband: ±0.03 °/s
- In-run bias trim gain: k = 0.0015
- Gyro sensitivity: 131 LSB/°/s (±250 dps)
- Accelerometer sensitivity: 16384 LSB/g (±2 g)

Display Output
--------------
MPU-6050 Orientation

Roll :  X.X deg  
Pitch:  X.X deg  
Yaw  :  X.X deg*  

*gyro-only (drifts)  
IO14: zero yaw

Usage Notes
-----------
- Keep the sensor flat and still during startup calibration.
- Press IO14 to reset yaw when needed.
- Roll and pitch stabilize using accelerometer; yaw remains drift-prone but reduced.
- Avoid operation at ±90° pitch (singularity in kinematic yaw calculation).
- Ensure good I2C wiring; check addresses 0x68/0x69 if MPU not found.

Limitations
-----------
- No magnetometer correction → yaw will drift slowly.
- Not suitable for full 3D orientation near vertical pitch angles.
- For absolute yaw, add a magnetometer or sensor fusion library (e.g. Madgwick/Mahony).

Author - NC 
------
Adapted for UWA Indoor Drone Navigation Project.
Implements improved yaw stability for IMU-based orientation visualization on ESP32.

