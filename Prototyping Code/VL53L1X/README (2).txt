
VL53L1X Non‑Blocking Ranging (ESP32‑S3, TTGO T‑Display S3)
=========================================================

This sketch reads distance from a VL53L1X time‑of‑flight sensor on an ESP32‑S3 (TTGO T‑Display S3)
using a fully **non‑blocking** state machine. It repeatedly prints raw and moving‑average distances
to the Serial Monitor and automatically retries sensor bring‑up if anything fails.

Contents
--------
1) Features
2) Hardware & Wiring
3) Libraries & IDE Setup
4) How it Works
5) Configuration
6) Serial Output
7) Common Issues & Troubleshooting
8) Extending the Sketch
9) License

1) Features
-----------
- Non‑blocking bring‑up and run loop via a small state machine:
  STARTUP → TRY_INIT → RUNNING → ERROR_RETRY (with back‑off)
- Optional interrupt support (data‑ready pin) — disabled by default.
- I²C bus scan on boot to verify the sensor responds at 0x29.
- 5‑sample moving average smoothing for cleaner distance prints.
- Safe, periodic polling (no delay()-based blocking for measurement readiness).
- Automatic re‑initialisation with exponential back‑off after failures.

2) Hardware & Wiring
--------------------
Target board: **TTGO ESP32‑S3 (T‑Display S3)**

I²C (as hard‑wired on this board in this project):
- SDA → **GPIO 1**
- SCL → **GPIO 2**

Other sensor lines (not used by default):
- XSHUT → not used (PIN_XSHUT = -1)
- INT (GPIO from VL53L1X) → not used (PIN_INT = -1)

VL53L1X power:
- VIN → 3V3 (or 5V on some breakout boards; check your module)
- GND → GND

3) Libraries & IDE Setup
------------------------
- **Arduino IDE** with **ESP32 boards** installed (ESP32‑S3 selected).
- Libraries:
  - `Adafruit_VL53L1X` (Library Manager)
  - `Wire` (built‑in)

Board selection tips:
- Tools → Board: “ESP32S3 Dev Module” (or your TTGO S3 variant)
- Use a **data‑capable USB‑C cable** and set a suitable upload port.
- Serial Monitor at **115200 baud**.

4) How it Works
---------------
The sketch organises the sensor lifecycle into a state machine:

- **STARTUP**: initial state; immediately transitions to TRY_INIT.
- **TRY_INIT**: attempts `vl53.begin(0x29, &Wire)`, configures timing budget,
  starts ranging, and clears the moving‑average buffer. On success → RUNNING.
  On failure → ERROR_RETRY.
- **RUNNING**: polls the sensor for data readiness at a fixed cadence. When
  new data is ready, reads the distance (mm), clears the sensor interrupt,
  updates the 5‑sample moving average, and prints both raw and averaged values.
- **ERROR_RETRY**: waits for a back‑off interval, then returns to TRY_INIT.
  The back‑off time grows up to a cap to avoid busy looping during faults.

An I²C scan runs at boot so you can confirm the device is visible at **0x29**.

5) Configuration
----------------
Key constants near the top of the sketch:

- `PIN_SDA` / `PIN_SCL` : I²C pins (defaults: 1, 2 on TTGO S3 build here).
- `TIMING_BUDGET_MS`    : VL53L1X timing budget (e.g. 200 ms ≈ ~5 Hz).
- `POLL_INTERVAL_MS`    : loop polling cadence for data readiness (e.g. 20 ms).
- Moving average window: `AVG_N = 5` (tune for more/less smoothing).
- Optional interrupt:
  - Set `PIN_INT` to the connected VL53L1X GPIO; enable the `attachInterrupt`
    block in `setup()` to use an IRQ‑driven data‑ready pathway.

6) Serial Output
----------------
At 115200 baud you’ll see lines like:

```
I2C scan:
I2C device at 0x29
VL53L1X ready
Raw: 411 mm   |   Avg(5): 408.2 mm
Raw: 409 mm   |   Avg(5): 409.4 mm
...
```

If a read fails or is out of range (OOR), you’ll see the driver status code
printed to help with diagnosis.

7) Common Issues & Troubleshooting
----------------------------------
- **No device at 0x29 in the I²C scan**:
  - Check 3V3/5V and GND, and that SDA=1 / SCL=2 match your wiring.
  - Make sure pull‑ups are present (most breakout boards include them).
  - Ensure no other devices are holding the bus or XSHUT is not held low.

- **Serial shows only boot ROM text**:
  - Pick the correct COM/tty port. Try a known‑good data USB‑C cable.
  - Confirm the board is in normal boot mode (not download/BOOT mode).

- **`vl53.begin(...)` fails repeatedly**:
  - Power cycle the sensor; some modules need a clean power‑on.
  - Try a slower I²C clock: change `Wire.begin(PIN_SDA, PIN_SCL, 100000)`.
  - Double‑check sensor address (0x29) and voltage level compatibility.

- **Noisy or jumping readings**:
  - Increase `AVG_N` or `TIMING_BUDGET_MS` for more stable results.
  - Ensure the sensor has a clear field of view; avoid finger/edge occlusion.
  - Highly reflective/absorptive targets can cause spurious ranges.

8) Extending the Sketch
-----------------------
- **Interrupt‑driven reads**: set `PIN_INT` to a valid GPIO, wire VL53L1X’s
  GPIO1 (data‑ready) to that pin, and enable the `attachInterrupt` section.
- **Distance mode / ROI**: the Adafruit driver supports additional controls
  (e.g., distance modes, ROI). After `begin()`, call the appropriate setters.
- **Multiple sensors**: use dedicated XSHUT lines to hold others in reset, then
  re‑address each device before starting ranging.

9) License
----------
This README is provided for your project use. The original Adafruit VL53L1X
library is licensed under its own terms; please consult that library’s README.

—
Generated for: VL_working_code_non_blocking.ino
- DP
