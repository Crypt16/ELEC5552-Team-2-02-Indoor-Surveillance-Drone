README — Combined Odometry (PMW3901) + Obstacle & Height (VL53L1X)
==================================================================

Overview
--------
ESP32 sketch that fuses:
- PMW3901 optical flow (SPI) for XY ground motion
- Multiple VL53L1X ToF sensors (I2C) for obstacles (L/R/F/B)
- One VL53L1X ToF dedicated to height-above-ground

Height is filtered (median + EMA + slew limit) and used to scale optical-flow
pixels into real-world displacement. Status prints at 10 Hz.

Hardware / Pins (edit in CONFIG)
--------------------------------
I2C:
  SDA = 21, SCL = 22   (change to your board)
SPI for PMW3901:
  MISO = 19, MOSI = 23, SCK = 18, CS = 5

VL53L1X XSHUT wiring (each sensor needs its own XSHUT pin):
  cfgs[] = {
    { "Left",   17, 0x30, false },
    { "Right",   1, 0x31, false },
    { "Front",  37, 0x32, false },
    { "Back",   36, 0x33, false },
    { "Height", 18, 0x34, true  },  // dedicated altitude sensor
  }
- Use unique I2C addrs (0x30–0x37). Update pins/addresses to match your wiring.

Required Libraries
------------------
- PMW3901 optical flow:
  * Bitcraze_PMW3901 (recommended) or another PMW3901 lib providing:
      bool PMW3901::begin(csPin), bool readMotionCount(int16_t*, int16_t*)
- VL53L1X by Pololu (supports multi-sensor XSHUT readdressing)

Build Settings
--------------
Board:    Any ESP32 (e.g., ESP32 Dev Module / ESP32-S3)
Core:     ESP32 Arduino core 2.0.x+
Baud:     115200

What the code does (high level)
-------------------------------
1) Starts I2C @ 400 kHz and SPI for PMW3901.
2) Powers all VL53L1X off (XSHUT LOW), then brings them up one-by-one:
   - init at default 0x29, assign unique address from cfgs[], set mode, start continuous.
3) Reads all ToF sensors every loop:
   - One is the “Height” sensor → filtered (median + EMA + slew).
   - Others are obstacle sensors → each compared to OBSTACLE_LIMIT_MM to flag proximity.
4) Every 10 ms (~100 Hz) reads PMW3901 dx/dy pixels:
   - Converts pixels → angle with K_pix_to_rad_x/y
   - Scales by current height to mm displacement
   - Integrates into global pose (x_mm, y_mm); estimates velocity (mm/s)
5) Prints height, pose, velocity, and obstacle distances at 10 Hz.

Important constants to tune
---------------------------
- FLOW_SIGN_X / FLOW_SIGN_Y: set to -1 if your axes are flipped (default X=+1, Y=-1).
- K_pix_to_rad_x / K_pix_to_rad_y: pixel→radian scale (start ~0.00153f and calibrate).
- OBSTACLE_LIMIT_MM: proximity threshold (default 800 mm).

Height filtering (stabilizes scale)
-----------------------------------
Median-of-5  →  EMA (H_ALPHA=0.55)  →  slew limit (H_SLEW_MM per update).
If height is noisy: increase H_BUF_N (to 7/9), lower H_ALPHA (e.g., 0.35), or reduce H_SLEW_MM.

Ranging timing (VL53L1X)
------------------------
- DistanceMode: Short
- TIMING_BUDGET_MS = 20, MEAS_PERIOD_MS = 20  (≈50 Hz per sensor)
Ensure period ≥ budget. More sensors → consider longer period or slower prints.

Serial output (example)
-----------------------
H=245 mm  Pose=(123, -78) mm  v=(60, -15) mm/s  Obstacle Left 120mm  Right 200mm  Front 850mm  Back -1mm
- H: filtered height (mm)
- Pose: integrated XY (mm)
- v: velocity estimate (mm/s)
- Obstacle distances: last measured per side (-1 means invalid/stale)

Calibration steps (optical flow scale)
--------------------------------------
1) Fix height (e.g., 200 mm).
2) Move a measured distance (e.g., 1.0 m) along X; compare ΔPose to real distance.
3) Scale K_pix_to_rad_x accordingly. Repeat for Y (K_pix_to_rad_y).
4) If drift sign is wrong, flip FLOW_SIGN_X/Y.

Utilities in the code
---------------------
- driveAllXShutLow(): pulls every VL53L1X into reset (XSHUT LOW).
- bringUpOne(i): boots one VL53L1X at 0x29, assigns cfgs[i].addr, starts ranging.
- i2cScanOnce(): optional bus scan for debugging.

Common pitfalls & fixes
-----------------------
- All sensors show 0x29: your XSHUT lines aren’t isolating; bring-up one at a time.
- PMW3901 read fail: check SPI pins/CS, power, and orientation; needs textured surface & light.
- Pose scale wrong: re-tune K_pix_to_rad_* with a measured run at known height.
- Height jitter: tweak H_BUF_N / H_ALPHA / H_SLEW_MM.

Limits
------
- No yaw/heading fusion here; pose is in the sensor frame (no rotation compensation).
- VL53L1X “last value” can be stale if no new data; code prefers dataReady() when available.

License
-------

- DP 
