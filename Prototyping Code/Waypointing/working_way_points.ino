#include <Wire.h>
#include "Adafruit_VL53L1X.h"
#include "Bitcraze_PMW3901.h"

// ===================== Pins (ESP32-C3 Mini) =====================
constexpr int PIN_SDA  = 8;  // I2C SDA (VL53L1X)
constexpr int PIN_SCL  = 9;  // I2C SCL (VL53L1X)
constexpr int PIN_MISO = 5;  // PMW3901 MISO
constexpr int PIN_MOSI = 6;  // PMW3901 MOSI
constexpr int PIN_SCK  = 4;  // PMW3901 SCK
constexpr int PIN_CS   = 7;  // PMW3901 CS/SS

// ===================== Rates =====================
constexpr uint32_t FLOW_DT_MS  = 10;   // ~100 Hz (flow)
constexpr uint32_t TOF_DT_MS   = 10;   // poll ToF frequently
constexpr uint32_t PRINT_DT_MS = 100;  // 10 Hz serial

// ===================== Flow scale =====================
// Calibrate by: K_new = K_old * (true_distance / measured_distance)
float K_pix_to_rad_x = 0.00193f;  // tweak by scalar
float K_pix_to_rad_y = 0.00193f;

constexpr int DEAD = 0;            // keep 0..1 at ~2 m altitude
constexpr int H_MIN = 60;          // mm
constexpr int H_MAX = 4000;        // mm

float alpha_v = 0.3f;              // velocity LPF (0 = off)

// ===================== Devices =====================
Adafruit_VL53L1X vl53(-1, -1);   // (XSHUT, INT) unused
Bitcraze_PMW3901 flow(PIN_CS);

// ===================== State =====================
bool tof_ok  = false;
bool flow_ok = false;

int16_t last_height_mm = -1;

volatile int32_t acc_dx = 0, acc_dy = 0;

double x_mm = 0.0, y_mm = 0.0;   // world pose at boot frame
float  yaw_rad = 0.0f;           // assume ~0 while walking

uint32_t t_next_flow  = 0;
uint32_t t_next_tof   = 0;
uint32_t t_next_print = 0;

float vx_dbg = 0.0f, vy_dbg = 0.0f;   // for display

// ===================== Height filtering =====================
// Median-of-4 + EMA + slew limit (fast-ish per your last settings)
constexpr float H_ALPHA   = 0.55f;     // EMA weight (higher = faster)
constexpr int   H_SLEW_MM = 500;       // max change per update (mm)
constexpr int   H_BUF_N   = 4;

int16_t h_buf[H_BUF_N] = { -1,-1,-1,-1 };
uint8_t h_idx = 0, h_count = 0;
float   h_ema = -1.0f;

inline int16_t deband(int16_t v) { return (v >= -DEAD && v <= DEAD) ? 0 : v; }

int16_t medianSmall(int16_t* a, uint8_t n) {
  int16_t t[H_BUF_N];
  for (uint8_t i=0;i<n;i++) t[i]=a[i];
  for (uint8_t i=1;i<n;i++){ int16_t key=t[i]; int8_t j=i-1; while (j>=0 && t[j]>key){ t[j+1]=t[j]; j--; } t[j+1]=key; }
  return t[n/2];
}

int16_t updateHeightFilter(int16_t mm_raw){
  if (mm_raw < 0) return (h_count ? (int16_t)(h_ema + (h_ema>=0?0.5f:-0.5f)) : -1);

  // push to ring buffer
  h_buf[h_idx] = mm_raw;
  h_idx = (h_idx + 1) % H_BUF_N;
  if (h_count < H_BUF_N) h_count++;

  // median (robust to spikes)
  int16_t med = medianSmall(h_buf, h_count);

  // initialize EMA with first median
  if (h_ema < 0) h_ema = (float)med;

  // slew limit target before EMA
  float prev = h_ema;
  float target = (float)med;
  float diff = target - prev;
  if (diff >  H_SLEW_MM) target = prev + H_SLEW_MM;
  if (diff < -H_SLEW_MM) target = prev - H_SLEW_MM;

  // EMA
  h_ema = (1.0f - H_ALPHA)*h_ema + H_ALPHA*target;

  return (int16_t)(h_ema + (h_ema>=0?0.5f:-0.5f)); // rounded mm
}

// ===================== Odometry =====================
void updateOdometry(int32_t dx_counts, int32_t dy_counts, int height_mm, float dt_s) {
  if (height_mm < H_MIN || height_mm > H_MAX || dt_s <= 0.0005f) return;

  dx_counts = deband((int16_t)dx_counts);
  dy_counts = deband((int16_t)dy_counts);

  const float ang_x = dx_counts * K_pix_to_rad_x;
  const float ang_y = dy_counts * K_pix_to_rad_y;

  const float vx_cam = height_mm * (ang_x / dt_s);  // mm/s
  const float vy_cam = height_mm * (ang_y / dt_s);  // mm/s

  static float vx_f = 0.0f, vy_f = 0.0f;
  vx_f = (1.0f - alpha_v) * vx_f + alpha_v * vx_cam;
  vy_f = (1.0f - alpha_v) * vy_f + alpha_v * vy_cam;

  const float c = cosf(yaw_rad), s = sinf(yaw_rad);
  const float vx_w =  c * vx_f - s * vy_f;
  const float vy_w =  s * vx_f + c * vy_f;

  x_mm += vx_w * dt_s;
  y_mm += vy_w * dt_s;

  vx_dbg = vx_w; vy_dbg = vy_w;
}

// ===================== Move queue / parser =====================
// Use parallel arrays to avoid custom types in function params (Arduino preprocessor safe)
constexpr int MAX_MOVES = 16;
long q_dx[MAX_MOVES];
long q_dy[MAX_MOVES];
int  q_head = 0, q_tail = 0;

long target_start_x = 0, target_start_y = 0;
long target_dx = 0, target_dy = 0;
bool has_active = false;
int  MOVE_TOL_MM = 50;   // default tolerance

// FIFO helpers
bool qEmpty(){ return q_head == q_tail; }
bool qFull(){ return ((q_tail+1)%MAX_MOVES) == q_head; }

bool queuePush(long dx_mm, long dy_mm){
  if (qFull()) return false;
  q_dx[q_tail] = dx_mm;
  q_dy[q_tail] = dy_mm;
  q_tail = (q_tail + 1) % MAX_MOVES;
  return true;
}

bool queuePop(long *dx_mm, long *dy_mm){
  if (qEmpty()) return false;
  *dx_mm = q_dx[q_head];
  *dy_mm = q_dy[q_head];
  q_head = (q_head + 1) % MAX_MOVES;
  return true;
}

void startNextMove(){
  long dx, dy;
  if (!queuePop(&dx, &dy)) { has_active=false; return; }
  target_start_x = (long) x_mm;
  target_start_y = (long) y_mm;
  target_dx = dx;
  target_dy = dy;
  has_active = true;

  Serial.print("GO: ");
  if (dx != 0) { Serial.print("X "); Serial.print(dx); Serial.print(" mm "); }
  if (dy != 0) { Serial.print("Y "); Serial.print(dy); Serial.print(" mm "); }
  Serial.println();
}

// Call periodically to update progress & auto-advance
void updateMoveController(){
  if (!has_active) return;
  long moved_x = (long) x_mm - target_start_x;
  long moved_y = (long) y_mm - target_start_y;
  long rem_x = target_dx - moved_x;
  long rem_y = target_dy - moved_y;

  static uint32_t last_prog = 0;
  if (millis() - last_prog > 250) {
    Serial.print("Remaining: X ");
    Serial.print(rem_x); Serial.print(" mm, Y ");
    Serial.print(rem_y); Serial.print(" mm  (tol ");
    Serial.print(MOVE_TOL_MM); Serial.println(" mm)");
    last_prog = millis();
  }

  if (llabs(rem_x) <= MOVE_TOL_MM && llabs(rem_y) <= MOVE_TOL_MM) {
    Serial.println("DONE ✓");
    has_active = false;
    startNextMove();
  }
}

// ====== Serial parsing ======
String inLine;

long parseDistanceMm(const String& tokVal) {
  // supports mm, cm, m (default mm if unit missing)
  String s = tokVal;
  s.trim();
  s.toLowerCase();

  // sign & number
  long sign = 1;
  int i = 0;
  if (s.startsWith("-")) { sign=-1; i=1; }
  else if (s.startsWith("+")) { i=1; }

  int j = i;
  while (j < s.length() && (isDigit(s[j]) || s[j]=='.')) j++;
  float val = s.substring(i, j).toFloat();
  String unit = s.substring(j); unit.trim();

  long mm = 0;
  if (unit == "m") mm = (long) lroundf(val * 1000.0f);
  else if (unit == "cm") mm = (long) lroundf(val * 10.0f);
  else /* "mm" or empty */ mm = (long) lroundf(val);

  return sign * mm;
}

void enqueueMoveCmd(char axis, long mm) {
  long dx = 0, dy = 0;
  if (axis == 'x' || axis == 'X') dx = mm;
  else if (axis == 'y' || axis == 'Y') dy = mm;
  else return;

  if (!queuePush(dx, dy)) {
    Serial.println("Queue full! (max 16). Use 'c' to clear or execute first.");
  } else {
    Serial.print("Queued: Move "); Serial.print((char)toupper(axis));
    Serial.print(" "); Serial.print(mm); Serial.println(" mm");
  }
}

void parseAndEnqueue(const String& cmd) {
  String s = cmd;
  s.replace("\"", ""); s.replace(".", ""); s.trim();
  s.toLowerCase();

  if (s.length()==0) return;

  if (s=="r" || s=="reset") { x_mm=0; y_mm=0; Serial.println("Odom reset."); return; }
  if (s=="c" || s=="clear") { q_head=q_tail=0; has_active=false; Serial.println("Queue cleared."); return; }
  if (s=="status" || s=="s") {
    Serial.print("Pose: ("); Serial.print((long)x_mm); Serial.print(", "); Serial.print((long)y_mm);
    Serial.print(") mm, H="); Serial.print(last_height_mm); Serial.println(" mm");
    int cnt = (q_tail - q_head + MAX_MOVES) % MAX_MOVES;
    Serial.print("Queue items: "); Serial.println(cnt);
    if (has_active) Serial.println("Active move in progress.");
    return;
  }
  if (s=="help" || s=="?") {
    Serial.println("Commands:");
    Serial.println("  move x 3m, move y 2m, move x -0.5m   (commas/newlines ok)");
    Serial.println("  r=reset odom, c=clear queue, t 80=set tolerance 80 mm, status/s=show");
    return;
  }
  if (s.startsWith("t ")) {
    int sp = s.indexOf(' ');
    int tol = s.substring(sp+1).toInt();
    if (tol > 0) { MOVE_TOL_MM = tol; Serial.print("Tolerance set to "); Serial.print(MOVE_TOL_MM); Serial.println(" mm"); }
    return;
  }

  if (s.startsWith("move")) {
    int p1 = s.indexOf(' ');
    if (p1 > 0) {
      int p2 = s.indexOf(' ', p1+1);
      if (p2 > 0 && p2+1 < s.length()) {
        char axis = s[p1+1];
        String dist = s.substring(p2+1);
        long mm = parseDistanceMm(dist);
        enqueueMoveCmd(axis, mm);
        return;
      }
    }
  }

  if (s.length() > 2 && (s[0]=='x' || s[0]=='y')) {
    int sp = s.indexOf(' ');
    if (sp > 0) {
      char axis = s[0];
      String dist = s.substring(sp+1);
      long mm = parseDistanceMm(dist);
      enqueueMoveCmd(axis, mm);
      return;
    }
  }
}

void parseSerialInput() {
  while (Serial.available() > 0) {
    char ch = (char)Serial.read();
    if (ch == '\n' || ch == '\r') {
      int start = 0;
      while (start < inLine.length()) {
        int sep = inLine.indexOf(',', start);
        String piece = (sep < 0) ? inLine.substring(start) : inLine.substring(start, sep);
        parseAndEnqueue(piece);
        if (sep < 0) break;
        start = sep + 1;
      }
      inLine = "";
    } else {
      inLine += ch;
      if (inLine.length() > 200) inLine = "";
    }
  }

  if (!has_active && !qEmpty()) startNextMove();
}

// ===================== Setup / Loop =====================
void setup() {
  Serial.begin(115200);
  delay(200);

  // ---- I2C on custom pins ----
  Wire.setPins(PIN_SDA, PIN_SCL);
  Wire.begin();
  Wire.setClock(400000); // set 100000 if bus is long / many pull-ups

  // ---- VL53L1X init ----
  tof_ok = vl53.begin(0x29, &Wire);
  if (!tof_ok) {
    Serial.print("VL53L1X init failed, status="); Serial.println(vl53.vl_status);
  } else {
    vl53.setTimingBudget(50);   // ms per measurement (~20 Hz)
    vl53.startRanging();        // continuous mode
    Serial.println("VL53L1X ready (timing budget 50 ms, filtered height).");
  }

  // ---- PMW3901 init ----
  flow_ok = flow.begin();
  Serial.println(flow_ok ? "PMW3901 ready" : "PMW3901 init failed");

  const uint32_t now = millis();
  t_next_flow  = now + FLOW_DT_MS;
  t_next_tof   = now + TOF_DT_MS;
  t_next_print = now + PRINT_DT_MS;

  Serial.println("Origin (0,0) mm.");
  Serial.println("Type commands like: move x 3m, move y 2m, move x 2m");
  Serial.println("Other: r (reset), c (clear queue), t 80 (tolerance), status, help");
}

void loop() {
  const uint32_t now = millis();

  // parse commands & manage queue
  parseSerialInput();
  updateMoveController();

  // Flow accumulation
  if (flow_ok) {
    while ((int32_t)(now - t_next_flow) >= 0) {
      int16_t dx=0, dy=0;
      flow.readMotionCount(&dx, &dy);
      acc_dx += dx;
      acc_dy += dy;
      t_next_flow += FLOW_DT_MS;
    }
  }

  // ToF polling + filtering
  if (tof_ok) {
    while ((int32_t)(now - t_next_tof) >= 0) {
      if (vl53.dataReady()) {
        int16_t mm = vl53.distance(); // -1 on error
        vl53.clearInterrupt();
        if (mm >= 0) {
          int16_t hf = updateHeightFilter(mm);
          if (hf >= 0) last_height_mm = hf;
        }
      }
      t_next_tof += TOF_DT_MS;
    }
  }

  // Odometry update @ FLOW cadence
  static uint32_t t_next_odom = 0;
  if (t_next_odom == 0) t_next_odom = now + FLOW_DT_MS;
  while ((int32_t)(now - t_next_odom) >= 0) {
    uint32_t t_us = micros();
    static uint32_t last_us = 0;
    float dt_s = (last_us == 0) ? (FLOW_DT_MS / 1000.0f) : (t_us - last_us) * 1e-6f;
    last_us = t_us;

    int32_t dx = acc_dx, dy = acc_dy;
    acc_dx = 0; acc_dy = 0;

    updateOdometry(dx, dy, last_height_mm, dt_s);
    t_next_odom += FLOW_DT_MS;
  }

  // Serial print @ 10 Hz
  while ((int32_t)(now - t_next_print) >= 0) {
    Serial.print("H="); Serial.print(last_height_mm);
    Serial.print(" mm  Pose: (");
    Serial.print((long)x_mm); Serial.print(", ");
    Serial.print((long)y_mm); Serial.println(") mm");
    t_next_print += PRINT_DT_MS;
  }
}
