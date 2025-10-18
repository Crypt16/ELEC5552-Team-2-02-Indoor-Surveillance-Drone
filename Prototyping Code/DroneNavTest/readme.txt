DroneNavTest — Step Navigation + Fake ToF Avoidance (ESP32/Arduino)
==================================================================

Overview
--------
Single-file demo that:
- Executes a step-based navigation plan (e.g., “FORWARD 1.5 m”, “ROTATE 1.57 rad”).
- Simulates Time-of-Flight (ToF) obstacle distances (front/left/right/diagonal).
- Performs simple avoidance: sidestep 0.10 m until clear, then resumes the plan.
- Includes a latency watchdog (REQ-007). If no heartbeat for > 2000 ms, it LANDs then CUTOFFs.
- Stops completely (halts loop) once the navigation plan is finished.

What You’ll See
---------------
All behavior is logged to the Serial Monitor (115200 baud), e.g.:
- Planned motions:  [NAV] Step 2/5 → [MOTION] RIGHT 1.00m
- Avoidance kicks:  [AVOID] Front <0.5m → RIGHT 0.1m
- Failsafe:         [FAILSAFE] Latency/comms > 200 ms → LAND then CUTOFF (REQ-007)
- Completion:       [NAV] Plan complete — stopping simulation.

Key Files
---------
- DroneNavTest.ino  (this sketch only)

Configuration (top of file)
---------------------------
LOG_BAUD               Serial speed (default 115200).
FRONT_THRESH_M         Front obstacle threshold (m).
SIDE_THRESH_M          Side obstacle threshold (m).
SIDESTEP_M             Sidestep distance (m) when avoiding.
FAKE_SENSOR_SCENARIO   0=clear, 1=front, 2=left, 3=right, 4=front-left, 5=front-right.
LOOP_PERIOD_MS         Update tick (ms).
WATCHDOG_TIMEOUT_MS    Failsafe timeout (ms); triggers LAND + CUTOFF.
SIM_HEARTBEAT_LOSS_AFTER  Not used here (set to 0). Heartbeat is refreshed each loop.

How the Fake Sensors Work
-------------------------
readToF() produces a “pulsing” near obstacle (0.30–0.55 m) to repeatedly trigger avoidance.
- Scenario 1: front only -> s1 decreases.
- Scenario 2: left only  -> s2 decreases.
- Scenario 3: right only -> s3 decreases.
- Scenario 4: front-left (~45°)  -> s1 low + s2 slightly lower.
- Scenario 5: front-right (~45°) -> s1 low + s3 slightly lower.

Navigation Plan (edit here)
---------------------------
In setup():
  planReset(&g_plan);
  planAdd(&g_plan, NAV_CMD_FORWARD, 1.5f);
  planAdd(&g_plan, NAV_CMD_RIGHT,   1.0f);
  planAdd(&g_plan, NAV_CMD_ROTATE,  1.57f);
  planAdd(&g_plan, NAV_CMD_FORWARD, 1.0f);
  planAdd(&g_plan, NAV_CMD_LEFT,    0.5f);

- Commands:
  NAV_CMD_FORWARD/BACKWARD/LEFT/RIGHT → distance in meters.
  NAV_CMD_ROTATE → angle in radians.
  (NAV_CMD_LAND / NAV_CMD_CUTOFF are used by the failsafe, not in your plan.)

Avoidance Logic (simple rule set)
---------------------------------
- If FRONT (s1) < 0.5 m while steps remain:
    - If RIGHT side (s3) is clearer than LEFT (s2) → move RIGHT 0.10 m.
    - Else → move LEFT 0.10 m.
- If LEFT or RIGHT (s2 or s3) < 0.5 m while steps remain:
    - If s2 > s3 → move LEFT 0.10 m (more space on left).
    - Else → move RIGHT 0.10 m.

Failsafe / Watchdog (REQ-007)
-----------------------------
- navHeartbeat() refreshes a “last seen” timestamp each loop.
- If (now - lastHeartbeat) > WATCHDOG_TIMEOUT_MS → triggers:
    1) NAV_CMD_LAND (print only in this demo), then
    2) NAV_CMD_CUTOFF (print only), and marks plan complete.

Program Flow
------------
setup():
  - Starts Serial, prints banner.
  - Builds the step plan.
  - Initializes watchdog timer.
loop():
  - Maintains heartbeat and checks watchdog.
  - Generates fake ToF distances (readToF).
  - Runs navUpdate():
      • First tries avoidanceStep() → issue 0.1 m sidestep if needed.
      • Otherwise executes the current planned step once and advances.
  - When plan finishes → prints “Plan complete” and halts forever.

Porting to Real Hardware
------------------------
- Replace readToF() with real sensor readings:
    tof_sample_t t = {front_m, left_m, right_m, back_m};
- Replace emitMove() stubs with your motor/FC API:
    moveForward(m), strafeLeft(m), rotateRad(r) → calls into your controller.
- Keep thresholds (FRONT_THRESH_M, SIDE_THRESH_M) and SIDESTEP_M as tunables.
- Keep navHeartbeat() tied to your comms/FC loop to enforce REQ-007.

Build & Run
-----------
1) Open in Arduino IDE / PlatformIO on an ESP32 board.
2) Select the correct board and COM port.
3) Upload and open Serial Monitor @ 115200 baud.
4) Adjust FAKE_SENSOR_SCENARIO to watch different avoidance behaviors.

Limitations (Demo)
------------------
- Movement and landing/cutoff are print-only; no actual motor control.
- No position/odometry; steps are treated as “atomic commands” for logging.
- Avoidance is simplistic and does not accumulate a new path; it just sidesteps.

License / Attribution
---------------------
Created for UWA Indoor Drone project demoing navigation + avoidance with simulated ToF data.
Feel free to adapt for your team’s controller and sensors.

Author - BV
