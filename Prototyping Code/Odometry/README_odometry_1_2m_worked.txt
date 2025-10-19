
PMW3901 Flow Odometry — 1.2 m Validation Run (ESP32‑S3)
=======================================================

This README documents `ODOMETRY_1.2M_WORKED.ino`. The sketch performs planar
odometry with a **PMW3901** optical‑flow sensor (and optional **VL53L1X/L0X**
height) on an **ESP32‑S3** (e.g., TTGO T‑Display S3). It’s tuned for a test in
which the rig successfully travelled **≈1.2 m**, validating scale and drift.

Contents
--------
1) Purpose & What “Worked” Means
2) Hardware & Wiring
3) Required Libraries & IDE
4) Flow → Velocity → Position Pipeline
5) Parameters You’ll Want to Tune
6) Suggested Test Procedure (1.2 m)
7) Example Serial Output
8) Troubleshooting & Tips
9) Extensions
10) License

1) Purpose & What “Worked” Means
--------------------------------
- Confirms PMW3901 counts are converted into **m/s** with correct **FoV scale**
  and **height** so that integrated position matches a **1.2 m** ground‑truth.
- Demonstrates stable **deadband/outlier** handling so the position doesn’t run
  away while stationary.
- Provides a baseline config you can re‑use for longer paths and waypoint tests.

2) Hardware & Wiring
--------------------
Target board: **ESP32‑S3** (TTGO T‑Display S3). Example pins (adapt as needed):

- **PMW3901 (SPI)**: `MISO=13`, `MOSI=11`, `SCK=12`, `CS=10`
- **VL53L1X/L0X (I²C, optional height)**: `SDA=1`, `SCL=2`
- Power: 3V3 and GND common to all devices.
- Mount PMW3901 **down‑facing**; keep 20–200 mm above a **matte, textured** surface.

3) Required Libraries & IDE
---------------------------
- Arduino IDE with ESP32 boards (select your S3 variant).
- Libraries:
  - `Bitcraze_PMW3901` (or equivalent PMW3901 driver)
  - `Adafruit_VL53L1X` / `Adafruit_VL53L0X` (if using ToF height)
  - `SPI`, `Wire` (built‑in)
- Serial Monitor at **115200 baud**.

4) Flow → Velocity → Position Pipeline
--------------------------------------
Small‑angle model:
- counts → radians: `ωx = Δx / Kx`, `ωy = Δy / Ky` (Kx,Ky depend on lens FoV)
- velocities: `vx = h * ωx`, `vy = h * ωy` (h in metres from ToF or fixed)
- integrate: `x += vx * dt`, `y += vy * dt` (dt from micros/millis)

Practical extras in this build:
- **Deadband** on Δx/Δy to suppress drift at rest.
- **Outlier rejection** for occasional large spikes.
- Optional **exponential smoothing** on height: `H_alpha` (0–1).

5) Parameters You’ll Want to Tune
---------------------------------
(Names may differ slightly in your copy.)

- **Scale/FoV**: `FOV_X_DEG`, `FOV_Y_DEG` or `Kx,Ky` (counts→rad). This is key.
- **Height**: ToF on/off, `H_alpha`, and clamps (e.g., 0.02–2.0 m).
- **Deadband**: `DEADBAND_COUNTS` for Δx/Δy near zero.
- **Outliers**: thresholds on |Δ| or |vx,vy| to drop bad frames.
- **Rates**: `FLOW_DT_MS` (~10 ms), `PRINT_DT_MS` (100–250 ms).
- **Guards**: cap |vx|,|vy| and bound |x|,|y| to catch runaway integration.

6) Suggested Test Procedure (1.2 m)
-----------------------------------
1) **Stationary** 10–20 s: verify drift < ~2–4 cm.
2) **Axis check**: push forward 0.5 m; ensure **x** increases (signs OK).
3) **Scale**: move exactly **1.20 m** at constant height; compare reported
   distance. Adjust FoV/Kx,Ky until reported distance ≈ **1.20 m ± 0.03 m**.
4) **Lateral sanity**: 0.5 m left; ensure **y** magnitude/sign looks right.
5) **Square path**: 0.3 m per side; end‑point error should be small (<7–10 cm).

7) Example Serial Output
------------------------
```
[FLOW] dx=5 dy=-3 dt=0.010s h=0.12m  vx=0.02 vy=-0.01 | x=0.84 y=-0.02
...
--- 1.2 m pass complete ---
[ODOM] dist=1.19 m  (target 1.20 m)  err=-0.01 m
```
If your numbers are far off, re‑check FoV/Kx,Ky and height smoothing.

8) Troubleshooting & Tips
-------------------------
- **Large drift at rest**: increase deadband; improve surface texture/lighting.
- **Under/over‑estimate of distance**: tweak FoV or Kx,Ky scale.
- **Spiky Δ**: add median‑of‑3 or raise outlier thresholds; shorten SPI wires.
- **Height noisy**: increase ToF timing budget; lower `H_alpha` (more smoothing).
- **Axes swapped/rotated**: correct signs or add a small mount‑angle rotation.
