# Navigation_Combined_No_Webserver_No_Motor
**ESP32‑S3 navigation + obstacle detection demo (no motor outputs)**  
*File:* `Navigation_Combined_No_Webserver_No_Motor.ino` • *Updated:* 2025-10-18

This sketch combines **PMW3901 optical‑flow odometry** with **multi‑VL53L1X ToF ranging** (via XSHUT pin control) to simulate waypoint navigation and basic obstacle detection. It **does not drive motors**—it prints state, pose, and events over Serial so you can validate sensing and logic before wiring actuators.

---

## Features
- **Optical‑flow odometry (Bitcraze PMW3901):** converts pixel counts → counts/s → rad/s with calibration constants, then to planar velocity using current height:  
  \(\dot\theta_x = \text{dps}_x \cdot K_{\text{pix→rad},x}\), \(\dot\theta_y = \text{dps}_y \cdot K_{\text{pix→rad},y}\)  
  \(v_x = h\,\dot\theta_y\), \(v_y = -h\,\dot\theta_x\) (axis mapping accounted for in code)
- **Height filter pipeline:** *Median‑of‑N → slew‑rate limiter → EMA* to smooth noisy ToF height.
- **Four obstacle ToFs + one height ToF** on a shared I²C bus using **Pololu VL53L1X** lib and **XSHUT** lines.
- **Waypoint queue via Serial**: `move <dx> <dy>, move <dx> <dy>, ...` processed in order.
- **State machine:** `NAVIGATE` ⇄ `HOVER` — entering `HOVER` when an obstacle is within a set threshold; the queue is **preserved**.
- **Readable telemetry** every 100 ms (pose, velocity, mode, step status, obstacles).

---

## Hardware & Libraries
- **MCU:** ESP32‑S3 (pins configured in sketch; adjust if your board differs).
- **Sensors:**
  - **PMW3901** optical‑flow (SPI).
  - **5× VL53L1X** ToF sensors (I²C) — *Left, Right, Forward, Back, Height*.
- **Libraries (Arduino IDE → Library Manager):**
  - `Bitcraze_PMW3901` (optical flow)
  - `VL53L1X` by **Pololu** (ToF)


### Default Pin Map (from the sketch)
SPI (PMW3901):
- `PIN_CS = 29`
- `PIN_SCK = 30`, `PIN_MISO = 31`, `PIN_MOSI = 37`

I²C (VL53L1X bus):
- `PIN_SDA = 33`, `PIN_SCL = 36`

XSHUT pins & I²C addresses (assigned at boot):
- **Left**:    XSHUT `26` → `0x30`
- **Right**:   XSHUT `16` → `0x31`
- **Forward**: XSHUT `27` → `0x32`
- **Back**:    XSHUT `28` → `0x33`
- **Height**:  XSHUT `9`  → `0x34`

> If your wiring differs, update the constants in the `g_cfgs[]` array and the I/O pin defines at the top of the sketch.

---

## Build & Upload
1. **Board:** select an **ESP32‑S3** board in *Tools → Board* (install Espressif package if needed).
2. **Libraries:** install the libraries listed above.
3. **Port:** choose your COM/tty port.
4. **Upload.**
5. Open **Serial Monitor** at **115200 baud** (newline OK). On reset you’ll see a banner like:
   ```
   [ESP32] Nav+Obstacle v3 (1.2M odom + Serial waypoints)
   ```

---

## How It Works
### Timing
- Optical‑flow loop: **every 10 ms** (`FLOW_DT_MS = 10`)
- Print cadence: **every 100 ms** (`PRINT_PERIOD_MS = 100`)
- VL53L1X timing budget: **50 ms** (`VL_TIMING_BUDGET_MS = 50`), inter‑measurement **0 ms** (continuous).

### Optical‑Flow → Velocity → Pose
- Deadband on small counts (set `DEAD` if needed).
- Velocity low‑pass: `alpha_v = 0.30` (higher = snappier).
- Height‑scaled velocity integration (1.2M logic) to estimate pose.

**Calibration constants:**
```cpp
float K_pix_to_rad_x = 0.00193f;  // rad per count (sensor X → world +Y)
float K_pix_to_rad_y = 0.00193f;  // rad per count (sensor Y → world −X)
```
Tune these to match your lens/sensor and mounting geometry.

### Height Filtering
```cpp
// Height pipeline
int   N = 4;          // median window
float alpha = 0.20f;  // EMA
int   H_MIN = 60;     // mm clamp
int   H_MAX = 4000;   // mm clamp
```
A **slew‑rate limiter** guards against sudden jumps. The result feeds the odometry scale.

### Obstacle Detection
Per‑side thresholds (mm):
```cpp
THRESH_LEFT = 500; THRESH_RIGHT = 500;
THRESH_FWD  = 500; THRESH_BACK  = 500;
```
When any side is ≤ its threshold, the mode switches to **HOVER** and an alert is printed. The waypoint queue remains intact.

### Modes
- **NAVIGATE:** integrate odometry and walk the active `Step` until within tolerance (`STEP_TOLERANCE_M = 0.05`).
- **HOVER:** hold position (no motor commands in this sketch); continue sensing and keep the waypoint queue.

---

## Serial Commands
Queue waypoints with comma‑separated `move` tokens:
```
move 0.50  0.00,  move -0.30  0.20,  move 0.00 -0.60
```
- Units: **metres** (dx, dy in the world frame used by the sketch).
- On completion of each step you’ll see `"Step Complete"`; when all are done: `"All Steps Complete"`.
- If you mistype, you’ll get `Parse error: use 'move <dx> <dy>'`.

> Tip: You can paste a long sequence at once. The input buffer is trimmed if it grows too large.

---

## Key Constants (tune to your build)
```cpp
// Rates
constexpr uint32_t FLOW_DT_MS       = 10;
constexpr uint32_t PRINT_PERIOD_MS  = 100;

// Ranging
constexpr uint16_t VL_TIMING_BUDGET_MS = 50;
constexpr uint16_t VL_INTER_MEAS_MS    = 0;

// Height clamps
constexpr int H_MIN = 60, H_MAX = 4000;

// Obstacle thresholds (mm)
constexpr uint16_t THRESH_LEFT=500, THRESH_RIGHT=500, THRESH_FWD=500, THRESH_BACK=500;

// Waypoint tolerance
constexpr float STEP_TOLERANCE_M = 0.05f;

// Flow calibration & filtering
float K_pix_to_rad_x = 0.00193f, K_pix_to_rad_y = 0.00193f;
float alpha_v = 0.30f;  // velocity LPF
```
If your flow scale or mounting differs, adjust `K_pix_to_rad_*`. For a choppier height signal, increase median `N` or reduce `alpha` slightly.

---

## Typical Serial Output
```
Mode=NAVIGATE  pose=(x=0.12, y=0.03)  v=(0.08, -0.02)  h=236mm
Step: (dx=0.50, dy=0.00)  remaining=(0.38, -0.03)
Obstacle Within 500mm Forward
Entering Hover State
Mode=HOVER  pose=(x=0.13, y=0.03)  v=(0.00, 0.00)  h=238mm
```
Use this to verify sensing and logic before adding motor control.

---

- DP
