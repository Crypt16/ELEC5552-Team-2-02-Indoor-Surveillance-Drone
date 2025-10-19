
Five VL53L1X Sensors — Robust Bring‑Up & Readout (ESP32‑S3)
===========================================================

This README documents `5_working_VL53L1X_14_10_25.ino`, a proven setup that
initialises and reads **five** VL53L1X ToF sensors on a single I²C bus using
**XSHUT sequencing** and per‑device addressing. Target board is an ESP32‑S3
(e.g., TTGO T‑Display S3).

Contents
--------
1) What this sketch does
2) Hardware & Wiring
3) Libraries & IDE Setup
4) Boot Sequence (XSHUT → Unique I²C Addresses)
5) Ranging & Loop Behaviour
6) Configuration (Pins, Addresses, Timing)
7) Serial Output Reference
8) Common Issues & Fixes
9) Extensions
10) License

1) What this sketch does
------------------------
- Brings up **five** VL53L1X sensors on the same I²C bus.
- Uses **XSHUT** lines to enable one sensor at a time, assign a **unique I²C
  address** (e.g., 0x30…0x34), then starts ranging on all.
- Non‑blocking main loop: polls for **data‑ready** per sensor and prints raw
  distances (and optionally smoothed values).
- Includes basic **retries** and helpful debug prints for bring‑up.

2) Hardware & Wiring
--------------------
Target: **ESP32‑S3** (TTGO T‑Display S3 recommended)

I²C bus (project defaults):
- **SDA → GPIO 1**
- **SCL → GPIO 2**

Per‑sensor **XSHUT** (example mapping used in this project; adjust if needed):
- LEFT    → **GPIO 17**
- RIGHT   → **GPIO 1**  *(if GPIO 1 is used for SDA in your build, pick a different pin)*
- FORWARD → **GPIO 37**
- BACK    → **GPIO 36**
- HEIGHT  → **GPIO 18**

> Important: choose valid, unique GPIOs for XSHUT. Avoid USB/JTAG strap pins.

Power:
- VIN → 3V3 (some boards accept 5V; check your module)
- GND → GND

Pull‑ups:
- Most breakout boards include SDA/SCL pull‑ups. For longer wiring or many
  devices, ensure effective pull‑up (~4.7 kΩ) and clean routing.

3) Libraries & IDE Setup
------------------------
- **Arduino IDE** with ESP32 boards installed (select your ESP32‑S3 variant).
- Libraries:
  - `Adafruit_VL53L1X`  (or Pololu `VL53L1X` if that’s what your code uses)
  - `Wire` (built‑in)

Serial Monitor: **115200 baud**.

4) Boot Sequence (XSHUT → Unique I²C Addresses)
-----------------------------------------------
Typical sequence implemented by the sketch:
1. Drive **all XSHUT LOW** (all sensors in reset).
2. Bring **one** sensor’s XSHUT **HIGH** → it appears at default **0x29**.
3. `begin(0x29, &Wire)` and then **assign new address** (e.g., 0x30).
4. Repeat for each sensor with the next address (0x31, 0x32, 0x33, 0x34).
5. Optionally run an **I²C scan** to confirm all addresses are present.
6. Start **ranging** on each sensor.

This prevents address collisions and guarantees a deterministic mapping from
physical sensor → logical role (LEFT/RIGHT/FORWARD/BACK/HEIGHT).

5) Ranging & Loop Behaviour
---------------------------
- The loop polls each sensor for **data ready** (no long `delay()`).
- When ready, it reads **distance (mm)**, clears the sensor interrupt, and
  prints values (optionally updates a moving average).
- If `begin()` fails or a sensor stops responding, the code prints a clear
  error message and may **retry** initialisation.

6) Configuration (Pins, Addresses, Timing)
------------------------------------------
Look near the top of the sketch (names may vary slightly):

- **Pins**
  - `PIN_SDA`, `PIN_SCL` — default **1, 2**.
  - `XSHUT_LEFT`, `XSHUT_RIGHT`, `XSHUT_FORWARD`, `XSHUT_BACK`, `XSHUT_HEIGHT`.

- **Addresses**
  - `ADDR_LEFT=0x30`, `ADDR_RIGHT=0x31`, `ADDR_FORWARD=0x32`,
    `ADDR_BACK=0x33`, `ADDR_HEIGHT=0x34` (example).

- **Timing / Modes**
  - `TIMING_BUDGET_MS` (e.g., 50–200 ms): longer = smoother, lower rate.
  - `POLL_INTERVAL_MS` (e.g., 20 ms): loop cadence for readiness checks.
  - Distance mode: Short/Medium/Long (if your driver exposes it).
  - Optional ROI (Region of Interest) to tighten FoV.

- **Averaging (optional)**
  - Moving average window size (e.g., N=5). Increase to reduce jitter.

7) Serial Output Reference
--------------------------
Example console on success:
```
I2C scan:
0x30 0x31 0x32 0x33 0x34
Init OK — TB=200 ms

LEFT:   412 mm
RIGHT:  409 mm
FRONT:  655 mm
BACK:   275 mm
HEIGHT: 812 mm
```

If any sensor fails to init, you’ll see which step failed (e.g., not found at
0x29, begin() failed, etc.) to speed up diagnosis.

8) Common Issues & Fixes
------------------------
- **Only one sensor responds**
  - Ensure XSHUT **sequencing** (one‑at‑a‑time) and **unique addresses**.
  - Verify each XSHUT pin is configured `OUTPUT` and default **LOW** at boot.

- **`begin()` fails repeatedly**
  - Power cycle the system; try `Wire.begin(PIN_SDA, PIN_SCL, 100000)` (100 kHz).
  - Check 3V3/GND, SDA/SCL orientation, and that no other device holds the bus.

- **Noisy / jumping readings**
  - Raise `TIMING_BUDGET_MS` or increase averaging window.
  - Ensure clear line‑of‑sight; avoid shiny/absorptive targets.

- **Wrong pins on TTGO S3**
  - Double‑check the board pinout; avoid strapping pins / USB/JTAG pins.

- DP
