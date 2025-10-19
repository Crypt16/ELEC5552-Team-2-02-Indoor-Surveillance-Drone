
ESP32‑S3 Odometry (PMW3901 + optional VL53L1X) — 16 Oct 2025
============================================================

This README documents `16_10_2025_Odometry_with_s3.ino`. The sketch computes
planar odometry on an **ESP32‑S3** (e.g., TTGO T‑Display S3) using a **PMW3901**
optical‑flow sensor, with optional height from a **VL53L1X** ToF sensor. It
converts flow counts → velocities using FoV & height, then integrates to (x,y).

Contents
--------
1) Overview
2) Hardware & Wiring
3) Libraries & IDE Setup
4) Frames, Signs & Units
5) Flow → Velocity → Position Pipeline
6) Height Handling
7) Configurable Parameters
8) Serial Output Reference
9) Quick‑Start Calibration
10) Tips, Limits & Troubleshooting
11) Extension Ideas
12) License

1) Overview
-----------
- Reads **Δx, Δy** counts from **PMW3901** at ~100 Hz (SPI).
- Uses **FoV scale** and **height** to compute ground velocities (m/s).
- Integrates velocities to **position (x,y)** in meters at fixed dt.
- Optional smoothing / deadband / outlier rejection to reduce drift.
- Prints human‑readable telemetry at 115200 baud.

2) Hardware & Wiring
--------------------
Target board: **ESP32‑S3** (TTGO T‑Display S3)

Example pins (adjust to match your build):
- **PMW3901 (SPI)**: `MISO=13`, `MOSI=11`, `SCK=12`, `CS=10`
- **VL53L1X (I²C)**: `SDA=1`, `SCL=2`  (optional)
- Power: 3V3 and GND common. Mount PMW3901 **down‑facing**.

Surface & height:
- Operates best at **~20–200 mm** above a **matte, textured** surface.
- Avoid shiny floors, repeating patterns, or very low light.

3) Libraries & IDE Setup
------------------------
- Arduino IDE + ESP32 boards (select your S3 variant).
- Libraries:
  - `Bitcraze_PMW3901` (or equivalent PMW3901 driver)
  - `Adafruit_VL53L1X` (if using ToF height)
  - `SPI`, `Wire` (built‑in)

4) Frames, Signs & Units
------------------------
- **Body frame**: +X forward, +Y left (choose and stick to one).
- PMW3901 returns Δ in **sensor axes**; map to body axes with sign flips and
  optional small **mount‑angle** rotation if the module isn’t square.
- Positions, velocities, and waypoints (if any) are in **meters**.

5) Flow → Velocity → Position Pipeline
--------------------------------------
Core math (small‑angle approx):
- counts → radians: `ωx = Δx / Kx`, `ωy = Δy / Ky`
- ground velocities: `vx = h * ωx`, `vy = h * ωy`
- integration: `x += vx * dt`, `y += vy * dt`

Implementation details:
- **dt** from micros()/millis() timestamps.
- **Deadband** around zero counts to suppress stationary drift.
- Optional **median‑of‑3** / spike filter before scaling.

6) Height Handling
------------------
- If ToF is present: read **height h (m)** and optionally smooth:
  `h_filt = H_alpha*h + (1-H_alpha)*h_prev`.
- If no ToF: set a **fixed height** (for bench tests) or compile‑time constant.
- Clamp `h` to sensible bounds (e.g., 0.02–2.0 m).

7) Configurable Parameters
--------------------------
(Naming may differ slightly in your sketch; tune as needed.)

- **Pins**: SPI CS/SCK/MOSI/MISO for PMW3901; I²C SDA/SCL for ToF.
- **Rates**: `FLOW_DT_MS` (~10 ms), `PRINT_DT_MS` (e.g., 100–250 ms).
- **FoV / Scale**: `FOV_X_DEG`, `FOV_Y_DEG` or `Kx,Ky` (counts→rad).
- **Mount angle**: `THETA_MOUNT_DEG` to rotate sensor frame to body frame.
- **Deadband**: `DEADBAND_COUNTS` for Δx/Δy.
- **Outlier rejection**: thresholds on |Δ| or |vx,vy|.
- **Height filter**: `H_alpha` (0–1).
- **Velocity / position guards**: cap |vx|, |vy| and bound |x|, |y|.

8) Serial Output Reference
--------------------------
Typical lines:
```
[FLOW] dx=-12 dy=8 dt=0.010s  h=0.12m  vx=-0.04 vy=0.03  |  x=0.02 y=0.01
(stationary) deadband active
```
If values jump while stationary, increase the deadband and verify FoV scale.

9) Quick‑Start Calibration
--------------------------
1) **Stationary** (10–20 s): ensure drift is minimal. Raise deadband if needed.
2) **Axis check**: push forward 0.5 m; does **x** increase? Fix signs/rotation.
3) **Scale**: move exactly **1.00 m** at fixed height; adjust FoV/Kx,Ky until
   integrated distance ≈ 1.00 m.
4) **Height**: vary height; velocities should scale linearly with `h`.
5) **Surface**: switch to a matte, random‑textured pattern if tracking drops.

10) Tips, Limits & Troubleshooting
----------------------------------
- Integration drift is unavoidable without absolute references.
- Low texture/light or excessive altitude degrades quality.
- Fast yaw/roll/pitch corrupts planar odometry unless compensated.
- **Fixes**: better texture, increase timing budget for ToF, add IMU yaw,
  add winsorized/median filters, clamp velocities, implement resets.
