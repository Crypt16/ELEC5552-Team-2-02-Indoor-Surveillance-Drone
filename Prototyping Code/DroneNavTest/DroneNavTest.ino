/* -----------------------------------------------------------
 * DroneNavTest.ino (single-file demo)
 * - Step-based navigation with obstacle avoidance.
 * - FAKE SENSOR MODE for demo (front/left/right/diagonal).
 * - Watchdog (REQ-007) stops system if latency > 200 ms.
 * - Stops completely once navigation plan finishes.
 * ----------------------------------------------------------- */

#include <Arduino.h>
#include <string.h>

/* ===================== CONFIG ===================== */
#define LOG_BAUD                 115200

#define FRONT_THRESH_M           0.50f
#define SIDE_THRESH_M            0.50f
#define SIDESTEP_M               0.10f

// 0=clear, 1=front, 2=left, 3=right, 4=front-left (~45°), 5=front-right (~45°)
#define FAKE_SENSOR_SCENARIO     4

#define LOOP_PERIOD_MS           150

// Failsafe / watchdog (REQ-007)
#define WATCHDOG_TIMEOUT_MS      2000
#define SIM_HEARTBEAT_LOSS_AFTER 0UL  // set >0 (ms) to simulate comms loss

/* ====================== TYPES ====================== */
typedef enum {
  NAV_CMD_NONE = 0,
  NAV_CMD_FORWARD,
  NAV_CMD_BACKWARD,
  NAV_CMD_LEFT,
  NAV_CMD_RIGHT,
  NAV_CMD_ROTATE,
  NAV_CMD_LAND,
  NAV_CMD_CUTOFF
} nav_cmd_t;

typedef struct {
  nav_cmd_t cmd;
  float     value;   // meters for moves; radians for rotate
} nav_step_t;

#define NAV_PLAN_MAX_STEPS   32
typedef struct {
  nav_step_t steps[NAV_PLAN_MAX_STEPS];
  uint8_t    count;
  uint8_t    index;
  bool       complete;
} nav_plan_t;

typedef struct {
  float s1_front_m;
  float s2_left_m;
  float s3_right_m;
  float s4_back_m;
} tof_sample_t;

/* ================== GLOBAL STATE =================== */
nav_plan_t g_plan;
static uint32_t g_lastHeartbeatMs = 0;
static bool     g_failsafeActive  = false;

/* ================== PLAN HELPERS =================== */
static void planReset(nav_plan_t *p) { memset(p, 0, sizeof(*p)); }
static bool planAdd(nav_plan_t *p, nav_cmd_t c, float v) {
  if (p->count >= NAV_PLAN_MAX_STEPS) return false;
  p->steps[p->count].cmd = c;
  p->steps[p->count].value = v;
  p->count++;
  return true;
}
static inline bool planDone(const nav_plan_t *p) {
  return p->complete || (p->index >= p->count);
}

/* =================== ACTUATORS ===================== */
static void moveForward(float m)  { Serial.print("[MOTION] FORWARD  ");  Serial.print(m, 2); Serial.println("m"); }
static void moveBackward(float m) { Serial.print("[MOTION] BACKWARD ");  Serial.print(m, 2); Serial.println("m"); }
static void strafeLeft(float m)   { Serial.print("[MOTION] LEFT     ");  Serial.print(m, 2); Serial.println("m"); }
static void strafeRight(float m)  { Serial.print("[MOTION] RIGHT    ");  Serial.print(m, 2); Serial.println("m"); }
static void rotateRad(float r)    { Serial.print("[MOTION] ROTATE   ");  Serial.print(r, 2); Serial.println("rad"); }

static void actionSafeLand()      { Serial.println("[ACTION] SAFE LAND: reduce thrust, descend, disarm FC"); }
static void actionEmergencyCutoff(){ Serial.println("[ACTION] EMERGENCY CUTOFF: stop all motors"); }

static void emitMove(nav_cmd_t cmd, float val) {
  switch (cmd) {
    case NAV_CMD_FORWARD:  moveForward(val);  break;
    case NAV_CMD_BACKWARD: moveBackward(val); break;
    case NAV_CMD_LEFT:     strafeLeft(val);   break;
    case NAV_CMD_RIGHT:    strafeRight(val);  break;
    case NAV_CMD_ROTATE:   rotateRad(val);    break;
    case NAV_CMD_LAND:     actionSafeLand();  break;
    case NAV_CMD_CUTOFF:   actionEmergencyCutoff(); break;
    default: break;
  }
}

/* ================ FAKE SENSORS ===================== */
static void readToF(tof_sample_t *t) {
  static uint16_t k = 0; k++;

  // Default = clear
  t->s1_front_m = 2.0f; t->s2_left_m  = 2.0f;
  t->s3_right_m = 2.0f; t->s4_back_m  = 2.0f;

  // Pulsing distance (0.30–0.55 m)
  float pulse = 0.30f + 0.25f * ((k % 6) / 5.0f);

  switch (FAKE_SENSOR_SCENARIO) {
    case 1: t->s1_front_m = pulse; break;  // front
    case 2: t->s2_left_m  = pulse; break;  // left
    case 3: t->s3_right_m = pulse; break;  // right
    case 4: t->s1_front_m = pulse; t->s2_left_m  = pulse * 0.95f; break;  // front-left
    case 5: t->s1_front_m = pulse; t->s3_right_m = pulse * 0.95f; break;  // front-right
    default: break; // clear
  }
}

/* ============== WATCHDOG / FAILSAFE ================ */
static inline void navHeartbeat() { g_lastHeartbeatMs = millis(); }

static void navCheckWatchdog() {
  if (g_failsafeActive) return;
  if (millis() - g_lastHeartbeatMs > WATCHDOG_TIMEOUT_MS) {
    g_failsafeActive = true;
    Serial.println("[FAILSAFE] Latency/comms > 200 ms → LAND then CUTOFF (REQ-007)");
    emitMove(NAV_CMD_LAND,   0.0f);
    emitMove(NAV_CMD_CUTOFF, 0.0f);
    g_plan.complete = true;
  }
}

/* ================== AVOIDANCE ====================== */
static bool avoidanceStep(const nav_plan_t *p, const tof_sample_t *t) {
  const bool haveSteps = (p && (p->index < p->count));

  // Front obstacle
  if (t->s1_front_m < FRONT_THRESH_M && haveSteps) {
    if (t->s3_right_m > t->s2_left_m) {
      Serial.println("[AVOID] Front <0.5m → RIGHT 0.1m");
      emitMove(NAV_CMD_RIGHT, SIDESTEP_M);
    } else {
      Serial.println("[AVOID] Front <0.5m → LEFT 0.1m");
      emitMove(NAV_CMD_LEFT, SIDESTEP_M);
    }
    return true;
  }

  // Side obstacle(s)
  bool leftClose  = (t->s2_left_m  < SIDE_THRESH_M);
  bool rightClose = (t->s3_right_m < SIDE_THRESH_M);

  if ((leftClose || rightClose) && haveSteps) {
    if (t->s2_left_m > t->s3_right_m) {
      Serial.println("[AVOID] Side <0.5m → LEFT 0.1m");
      emitMove(NAV_CMD_LEFT, SIDESTEP_M);
    } else {
      Serial.println("[AVOID] Side <0.5m → RIGHT 0.1m");
      emitMove(NAV_CMD_RIGHT, SIDESTEP_M);
    }
    return true;
  }

  return false;
}

/* ================== NAV UPDATE ===================== */
static void navUpdate(nav_plan_t *p, const tof_sample_t *tof) {
  if (!p || planDone(p)) return;

  if (avoidanceStep(p, tof)) return;

  const nav_step_t *s = &p->steps[p->index];
  Serial.print("[NAV] Step "); Serial.print(p->index + 1);
  Serial.print("/"); Serial.print(p->count);
  Serial.print(" → ");
  emitMove(s->cmd, s->value);

  p->index++;
  if (p->index >= p->count) {
    p->complete = true;
    Serial.println("[NAV] Plan complete — stopping simulation.");
  }
}

/* =================== ARDUINO ======================= */
void setup() {
  Serial.begin(LOG_BAUD);
  uint32_t t0 = millis(); while (!Serial && (millis() - t0) < 2000) {}
  delay(200);

  Serial.println("\n=== DroneNavTest (simple auto obstacles) ===");

  planReset(&g_plan);
  planAdd(&g_plan, NAV_CMD_FORWARD, 1.5f);
  planAdd(&g_plan, NAV_CMD_RIGHT,   1.0f);
  planAdd(&g_plan, NAV_CMD_ROTATE,  1.57f);
  planAdd(&g_plan, NAV_CMD_FORWARD, 1.0f);
  planAdd(&g_plan, NAV_CMD_LEFT,    0.5f);

  Serial.print("[INIT] Plan steps: "); Serial.println(g_plan.count);
  Serial.println("[INIT] Fake obstacle = Front-Left (~45°)");
  Serial.println("[INIT] Watchdog = 2000 ms (REQ-007)");

  g_lastHeartbeatMs = millis();
}

void loop() {
  static uint32_t last = 0;
  const uint32_t now = millis();

  if (g_failsafeActive) return;
  if (now - last < LOOP_PERIOD_MS) return;
  last = now;

  navHeartbeat();
  navCheckWatchdog();

  tof_sample_t tof;
  readToF(&tof);

  navUpdate(&g_plan, &tof);

  // Stop simulation completely once done
  if (planDone(&g_plan)) {
    while (1);  // freeze forever
  }
}

