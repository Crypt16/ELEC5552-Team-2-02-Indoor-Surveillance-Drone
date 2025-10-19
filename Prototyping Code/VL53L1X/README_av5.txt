
VL53L1X Non‑Blocking Ranging — av_5 Variant (ESP32‑S3, TTGO T‑Display S3)
========================================================================

This sketch (`VL_working_code_non_blocking_av_5.ino`) is a variant of the
non‑blocking VL53L1X reader tailored around a **fixed 5‑sample moving average**
for steady, low‑noise distance reporting. It uses a compact state machine so
the loop never blocks while waiting for new measurements.

Contents
--------
1) What’s in this variant
2) Hardware & Wiring
3) Libraries & IDE Setup
4) Run‑Loop Design
5) Configuration Knobs
6) Serial Output
7) Troubleshooting
8) Ideas for Extensions
9) License

1) What’s in this variant
-------------------------
- Same non‑blocking state machine as the base sketch:
  STARTUP → TRY_INIT → RUNNING → ERROR_RETRY (with capped back‑off).
- **Moving average window is fixed at 5** samples (hence _av_5_).
- I²C bus scan on boot to verify the device at **0x29** is present.
- Safe periodic polling (no `delay()` blocking for readiness checks).
- Clean serial prints showing **Raw** and **Avg(5)** distances in mm.
- Auto re‑init on failures (power hiccups, transient I²C errors, etc.).

2) Hardware & Wiring
--------------------
Target board: **TTGO ESP32‑S3 (T‑Display S3)**

I²C lines (as used in this project):
- SDA → **GPIO 1**
- SCL → **GPIO 2**

Optional lines (disabled by default in this variant):
- XSHUT → not used (PIN_XSHUT = -1)
- INT (VL53L1X GPIO1) → not used (PIN_INT = -1)

Sensor power:
- VIN → 3V3 (or 5V if your breakout supports it; consult your module docs)
- GND → GND

3) Libraries & IDE Setup
------------------------
- **Arduino IDE** with **ESP32 boards** installed (select an ESP32‑S3 profile).
- Libraries:
  - `Adafruit_VL53L1X`
  - `Wire` (built‑in)

Recommended IDE settings:
- Board: *ESP32S3 Dev Module* (or your TTGO S3 flavour)
- Upload Speed: default or lower if you see flashing issues
- **Serial Monitor: 115200 baud**

4) Run‑Loop Design
------------------
State machine overview:

- **STARTUP**: initial state → transitions immediately to TRY_INIT.
- **TRY_INIT**: calls `vl53.begin(0x29, &Wire)`, configures timing budget,
  starts ranging, and resets the 5‑sample averaging buffer. On success → RUNNING;
  on failure → ERROR_RETRY.
- **RUNNING**: at a fixed polling cadence, checks for new data; when ready,
  reads the distance (mm), clears the sensor interrupt, updates the Avg(5),
  and prints both values.
- **ERROR_RETRY**: waits for a back‑off interval, then attempts init again.

An **I²C scan** occurs on boot to confirm the device responds at **0x29**.

5) Configuration Knobs
----------------------
Near the top of the sketch you can tune:

- `PIN_SDA` / `PIN_SCL` : I²C pins (defaults: 1 and 2 on TTGO S3 in this project).
- `TIMING_BUDGET_MS`    : longer budgets improve stability (e.g., 200 ms ≈ ~5 Hz).
- `POLL_INTERVAL_MS`    : cadence for checking data‑ready (e.g., 20 ms).
- **Averaging**         : fixed **5** samples in this variant.
- Optional interrupt:
  - If you wire VL53L1X GPIO1 to a free ESP32‑S3 pin, set `PIN_INT` accordingly
    and enable the `attachInterrupt` block to go IRQ‑driven instead of polling.

6) Serial Output
----------------
Typical output at 115200 baud:

```
I2C scan:
I2C device at 0x29
VL53L1X ready
Raw: 420 mm   |   Avg(5): 417.8 mm
Raw: 422 mm   |   Avg(5): 419.6 mm
...
```

Driver status codes (if any) are printed on OOR/failed reads to help debug.

7) Troubleshooting
------------------
- **No 0x29 in I²C scan**:
  - Recheck SDA=1 / SCL=2 wiring and module power (3V3/GND).
  - Ensure pull‑ups exist (many breakout boards include them).
  - Make sure XSHUT is not held LOW if your breakout exposes it.

- **Only boot ROM text in Serial**:
  - Pick the correct port; try a different known‑good USB‑C cable.
  - Ensure normal boot (not BOOT/flash mode).

- **Init keeps failing**:
  - Power cycle the sensor or the board.
  - Lower I²C speed via `Wire.begin(PIN_SDA, PIN_SCL, 100000)`.
  - Verify address (0x29) and voltage levels.

- **Noisy/erratic readings**:
  - Increase `TIMING_BUDGET_MS` for better measurement stability.
  - Confirm the sensor’s field‑of‑view is unobstructed.
  - Highly shiny/absorbing targets may reduce reliability.