README.txt
===========

ESP32 WROOM-32 Flight Controller (Takeoff/Hover Demo)
-----------------------------------------------------

This sketch turns an ESP32 WROOM-32 dev board into a simple flight-controller core with web control.
It can arm/disarm motors, read an MPU-6050 IMU, run a lightweight roll/pitch PID stabilizer, mix
outputs to 4 motors, and expose a tiny web API + WebSocket for status and tuning.

1) Features (what it does)
--------------------------
- Wi-Fi AP or STA mode with a tiny web server.
- HTTP endpoints to arm/disarm, fetch status, and tune parameters.
- WebSocket (/ws) streaming of live status lines.
- MPU-6050 IMU readout @ ~250 Hz with complementary filter for roll/pitch and integrated yaw.
- Roll/Pitch PID stabilizer with integral clamping and rate damping.
- Mixer for a standard “X” quad (M1=FR, M2=BR, M3=BL, M4=FL) with thrust-sum hold.
- Motor output via LEDC PWM (8-bit, 18 kHz) with slew-rate limiting for smooth changes.
- Battery voltage read on ADC and reported in status.
- Config persistence (NVS/flash): PID gains, trims, base throttle, stab toggle.
- Safety: auto-disarm if last WebSocket client disconnects; gentle throttle ramp on arm.

2) Hardware
-----------
- MCU: ESP32 WROOM-32 dev board
- IMU: MPU-6050 (I²C @ 0x68)
- Motors/ESCs: 4 PWM-driven ESC inputs
- Battery sense: ADC with ~2:1 divider (code assumes x2; adjust if different)

Pin map (change in code if needed):
- VBAT_ADC_PIN = GPIO34 (ADC1_CH6)
- I2C_SDA = GPIO21
- I2C_SCL = GPIO22
- MOTOR1 (Front-Right) = GPIO32
- MOTOR2 (Back-Right)  = GPIO26
- MOTOR3 (Back-Left)   = GPIO25
- MOTOR4 (Front-Left)  = GPIO33

3) Libraries you need
---------------------
- ESPAsyncWebServer
- AsyncTCP
- Arduino core libs: WiFi.h, Wire.h, Preferences.h

4) Build/Flash
--------------
- Board: ESP32 Dev Module (WROOM-32)
- Serial: 115200 (for logs)
- Optional compile flags:
  - DEBUG_SERIAL (on by default) — verbose logs
  - DEBUG_MOTOR_SINGLE — only Motor 1 runs (bench testing)

5) Wi-Fi modes
--------------
- STA mode (joins router) if WIFI_STA_SSID/PASS macros are non-empty.
- Else AP mode:
  - SSID: ProtoDrone
  - PASS: 12345678
- In AP mode, open: http://192.168.4.1/

6) Web API (HTTP)
-----------------
- GET /          → minimal UI (from web_ui.h)
- GET /arm       → arm motors (starts soft ramp & loop)
- GET /disarm    → stop all motors
- GET /state     → "STATE armed=0|1"
- GET /status    → one-line status (motors %, VBAT, R/P/Y if IMU OK)
- GET /cfg       → current settings (JSON)
- GET /setstab?on=0|1
  - Enables/disables stabilization. When enabling, captures level refs from current averaged
    roll/pitch (level the frame first).
- GET /setall?...  (all values persisted to flash)
  - base=<0..100>       base throttle (%)
  - kp, ki, kd          PID gains (floats)
  - trimR, trimP        roll/pitch trims in ±5% range
- GET /zeroref   → capture new level references (same as enabling stab while level)

WebSocket: /ws — streams periodic status lines and state; if last client disconnects while
armed, the FC auto-disarms.

7) Control loop & math (how it works)
-------------------------------------
Timings
- IMU read & filter: ~every 4 ms (~250 Hz)
- Main control loop: 4 ms (250 Hz)
- Status push: 20 Hz
- Battery update: 2 Hz (smoothed)

IMU
- Reads 14-byte burst from MPU-6050; converts to g/dps; subtracts gyro biases (short boot calibration).
- Complementary filter for roll/pitch (98% gyro / 2% accel when |acc|≈1g).
- Yaw integrates a tilt-compensated rate; wrapped to ±180°. (Yaw is not stabilized.)

Stabilizer (roll/pitch only)
- Uses ~100 ms sliding average of roll/pitch to reduce noise.
- /setstab?on=1 or /zeroref captures current averages as references.
- PID runs only when baseThrottlePct ≥ 65% (PID_ON_PCT) to avoid fighting idling motors.
- Integral clamp ±50 with decay when disabled; extra pitch scaling ×1.6.

Mixer
- Standard X quad:
  M1 FR = base + +Pitch − Roll
  M2 BR = base − Pitch − Roll
  M3 BL = base − Pitch + Roll
  M4 FL = base + +Pitch + Roll
- Thrust-sum hold: total output forced to 4×base; individual outputs clamped 0–100%.
- Slew limit: 300%/s per motor for smoothness.

Arming
- setArmed(true) zeros yaw and ramps throttle from ~4% toward base over ~600 ms (if base higher).
- Disarm sets all PWM = 0 immediately.

8) Key tunables (defaults)
--------------------------
- baseThrottlePct = 20% (bench value; increase cautiously with props & tether)
- PID: Kp=0.06, Ki=0.003, Kd=0.020
- Trims: trimRollPct / trimPitchPct ±5% max
- PID_ON_PCT = 65% (min throttle for PID)
- PWM = 18 kHz, 8-bit (change if your ESCs need different)

9) Battery sensing
------------------
VBAT_ADC_PIN sampled 8×; average scaled for 2:1 divider:
  Vbat = ADC/4095 * 3.3V * 2.0
Adjust multiplier in readVBAT() if your divider ratio differs.

10) Safety
----------
- PROPS OFF for first power-on. Use DEBUG_MOTOR_SINGLE to test one ESC channel.
- Tether/rig when enabling stabilization the first time.
- Keep a browser tab open; auto-disarm triggers if last WebSocket client disconnects.
- Verify motor order & spin direction match mixer mapping before flight.

11) Typical bring-up
--------------------
1. Flash firmware; open Serial @ 115200.
2. Power with props OFF; confirm IMU init + gyro bias print.
3. Connect to AP “ProtoDrone” (or your STA IP) and open root page.
4. Enable stabilization while holding frame level (/setstab?on=1) to capture refs.
5. Arm (/arm); confirm smooth ramp. Watch /status.
6. Increase base and tune Kp/Ki/Kd in small steps. Disarm via /disarm or power.

12) Troubleshooting
-------------------
- IMU WHO_AM_I wrong → check wiring/address/pull-ups/3V3.
- Motors idle → check LEDC pins/channels, ESC arming needs, power rails.
- Oscillation → reduce Kp/Kd; ensure PID only active above PID_ON_PCT.
- Tilts when stab on → /zeroref while level; trims near 0.
- Wrong VBAT → fix divider multiplier in readVBAT().

13) Files of interest
---------------------
- main.cpp  — all logic (Wi-Fi/AP, web API, IMU, PID, mixer, motors)
- web_ui.h  — minimal embedded HTML/JS page served at “/”

14) License & Use
-----------------
Educational prototype for bench testing.
Flying multirotors is dangerous—use at your own risk and obey local regulations.

- JW
