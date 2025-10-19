Waypoints Odometry with PMW3901 + VL53L1X (ESP32‑C3, Arduino)
==============================================================

This sketch fuses **PMW3901 optical flow** with a **VL53L1X ToF** height sensor to
estimate 2D pose (x, y) and execute simple **queued waypoint moves** entered
over Serial (e.g., `move x 3m, move y -0.5m`). It runs on an **ESP32‑C3 Mini**.

---
Hardware & Wiring
-----------------
- **MCU:** ESP32‑C3 Mini
- **Optical Flow:** Bitcraze PMW3901 (SPI)
  * CS → **GPIO 7**
  * SCK → **GPIO 4**
  * MOSI → **GPIO 6**
  * MISO → **GPIO 5**
- **Height Sensor:** VL53L1X (I²C)
  * SDA → **GPIO 8**
  * SCL → **GPIO 9**
  * Power: **3.3 V** and **GND**
- Bus speed: I²C at **400 kHz** (lower to 100 kHz if needed).

Libraries
---------
- `Adafruit_VL53L1X`
- `Bitcraze_PMW3901`
- (built‑in) `Wire`

Features at a Glance
--------------------
- **Continuous height** from VL53L1X with robust filter:
  * median‑of‑4 → EMA (α = **0.55**) → **slew limit** (±**500 mm**/update).
- **Optical‑flow odometry**:
  * Counts → radians via scale `K_pix_to_rad_x/y` (default **0.00193**).
  * Height‑scaled velocities (mm/s) integrated → **(x_mm, y_mm)**.
  * 1st‑order **velocity LPF** `alpha_v = 0.30`.
  * Assumes yaw ≈ 0 rad (constant `yaw_rad`, editable).
- **Waypoint queue** (FIFO, up to **16** items):
  * Commands like `move x 3m`, `move y 200mm` are parsed and queued.
  * Controller monitors remaining distance and auto‑advances on **tolerance**.
  * Default tolerance **50 mm** (configurable at runtime).
- **Resilient time‑base**:
  * Independent cadences for Flow, ToF and printing: 10 ms / 10 ms / 100 ms.
- **Informative Serial UI** (115200 baud):
  * Shows pose and height at 10 Hz, plus status/help text.

Tunable Parameters (in code)
---------------------------
- `K_pix_to_rad_x`, `K_pix_to_rad_y` (default **0.00193** each)
  * Flow **scale factors**. Calibrate using a known travel:  
    `K_new = K_old * (true_distance / measured_distance)`
- `alpha_v = 0.30` – velocity LPF (0 = off; higher = snappier, noisier).
- Height filter:
  * `H_ALPHA = 0.55` – EMA weight (increase to react faster, reduce noise rejection).
  * `H_SLEW_MM = 500` – max mm step per update (limits spikes).
  * `H_BUF_N = 4` – median window length.
- Valid height gate: `H_MIN = 60 mm`, `H_MAX = 4000 mm`.
- Move tolerance: `MOVE_TOL_MM = 50` (also settable at runtime with `t N`).

Serial Commands
---------------
- **Queue moves** (commas/newlines allowed; units: `mm`, `cm`, `m`; default `mm`):
  * `move x 3m`
  * `move y -0.5m`
  * `move x 200` (200 mm)
- **Control & status**
  * `r` or `reset` — zero odometry to (0,0).
  * `c` or `clear` — clear the move queue and stop the active move.
  * `t 80` — set tolerance to **80 mm** for move completion.
  * `status` or `s` — show current pose, height and queue size.
  * `help` or `?` — list commands.

Expected Serial Output (examples)
---------------------------------
- Startup:
  ```
  VL53L1X ready (timing budget 50 ms, filtered height).
  PMW3901 ready
  Origin (0,0) mm.
  Type commands like: move x 3m, move y 2m, move x 2m
  Other: r (reset), c (clear queue), t 80 (tolerance), status, help
  ```
- Streaming status (10 Hz):
  ```
  H=842 mm  Pose: (120, -35) mm
  ```
- Queueing and execution:
  ```
  Queued: Move X 3000 mm
  GO: X 3000 mm
  ...
  DONE ✓
  ```

How It Works
------------
1. **Sensors**  
   - VL53L1X starts continuous ranging with **50 ms timing budget** (~20 Hz).  
   - PMW3901 is polled every **10 ms** for Δx/Δy counts which are accumulated.

2. **Height pipeline**  
   Raw mm → median(4) → EMA(α=0.55) → slew‑limit to produce `last_height_mm`.

3. **Odometry**  
   Δcounts × `K_pix_to_rad_*` → angular rate (rad) / `dt` → **camera‑frame v**.  
   Rotate by `yaw_rad` to **world‑frame v**, low‑pass (α=0.30), integrate to `x_mm,y_mm`.

4. **Move controller**  
   - Maintains an active target (dx,dy) from the queue.
   - Computes remaining distance; if both |rem_x| and |rem_y| ≤ tolerance → **DONE**.
   - Automatically starts the next queued move, if any.

Getting Started
---------------
1. Install libraries (`Adafruit_VL53L1X`, `Bitcraze_PMW3901`) in Arduino IDE.
2. Select the **ESP32‑C3** board; set **Upload speed/Partition** as usual.
3. Wire the sensors as above; power from **3.3 V**.
4. Open Serial Monitor at **115200** baud.
5. Send a command: `move x 2m, move y 1m`. Watch pose/height printouts.

Calibration Tips
----------------
- Perform a straight‑line test of **known distance** on a consistent surface.  
  Adjust `K_pix_to_rad_x`/`y` using the formula above until measured pose matches ground truth.
- If pose is **laggy**, raise `alpha_v` slightly (e.g., 0.4).  
  If **jittery**, lower it (e.g., 0.2) or increase `H_ALPHA` a bit.
- If height spikes, consider reducing `H_SLEW_MM` or increasing `H_BUF_N`.

Troubleshooting
---------------
- **“VL53L1X init failed, status=…”**  
  Check I²C pins (SDA=8, SCL=9), 3.3 V supply, and that only one VL device is present at 0x29.
- **“PMW3901 init failed”**  
  Recheck SPI wiring (CS=7, SCK=4, MOSI=6, MISO=5) and sensor orientation.
- **Pose not changing / nonsense values**  
  Ensure `last_height_mm` is valid (60–4000 mm). Verify `K_pix_to_rad_*` scales.
- **Queue appears stuck**  
  Use `status` to confirm an active move; adjust tolerance with `t N` or `reset` odom.
