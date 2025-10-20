// ==== Combined Odometry (PMW3901) + Obstacle & Height (VL53L1X) ====
// Hardware: ESP32 (C3/S3) — adjust pins below
//
// Libraries needed:
//   - Bitcraze_PMW3901 (optical flow)  → provides class PMW3901
//   - VL53L1X by Pololu               → multi-sensor support with XSHUT readdressing
//
// Notes:
//   - Set your actual pinout in the CONFIG section.
//   - Tune K_pix_to_rad_* by running a known distance at a known height.
//   - Height is filtered (median + EMA + slew) to stabilise odometry.
//   - Four obstacle sensors + one height sensor shown; tweak cfgs[] to match your wiring.
// ------------------------------------------------------------------

#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <string.h>
#include <VL53L1X.h>

// --- PMW3901 optical-flow driver ---
#if __has_include("Bitcraze_PMW3901.h")
  #include "Bitcraze_PMW3901.h"
#elif __has_include("PMW3901.h")
  #include "PMW3901.h"
#else
  #error "PMW3901 library not found. In Arduino IDE: Tools → Manage Libraries → install 'Bitcraze_PMW3901' (or another PMW3901 driver)."
#endif

// ===================== CONFIG: Pins ===============================
// I2C pins
#ifndef PIN_SDA
#define PIN_SDA   21     // <-- set to your SDA (e.g., 8 on C3 Mini, 1 or 33 on S3)
#endif
#ifndef PIN_SCL
#define PIN_SCL   22     // <-- set to your SCL (e.g., 9 on C3 Mini, 2 or 36 on S3)
#endif

// SPI pins for PMW3901
#ifndef PIN_MISO
#define PIN_MISO  19     // e.g., C3 Mini: 5 ; S3 example: 13
#endif
#ifndef PIN_MOSI
#define PIN_MOSI  23     // e.g., C3 Mini: 6 ; S3 example: 11
#endif
#ifndef PIN_SCK
#define PIN_SCK   18     // e.g., C3 Mini: 4 ; S3 example: 12
#endif
#ifndef PIN_CS
#define PIN_CS     5     // e.g., C3 Mini: 7 ; S3 example: 10
#endif

// XSHUT pins for individual VL53L1X sensors
// Assign one as height sensor (is_height=true) and the others as obstacles.
struct SensorCfg {
  const char* role;
  uint8_t xshut_pin;
  uint8_t addr;      // unique I2C address to assign (0x30..0x37 typical)
  bool is_height;
};

// ---- EDIT THIS ARRAY to match your wiring ----
SensorCfg cfgs[] = {
  { "Left",    17,  0x30, false },
  { "Right",    1,  0x31, false },
  { "Front",   37,  0x32, false },
  { "Back",    36,  0x33, false },
  { "Height",  18,  0x34, true  },
};
// ------------------------------------------------

constexpr uint8_t NUM_SENSORS = sizeof(cfgs) / sizeof(cfgs[0]);
VL53L1X tof[NUM_SENSORS];

// ===================== CONFIG: Ranging ===========================
const VL53L1X::DistanceMode DIST_MODE = VL53L1X::DistanceMode::Short; // Short/Medium/Long
const uint16_t TIMING_BUDGET_MS = 20;     // 20 ms budget
const uint16_t MEAS_PERIOD_MS   = 20;     // startContinuous(period) — must be >= budget
const uint16_t OBSTACLE_LIMIT_MM = 800;   // threshold for obstacle detection (still available if needed)

// ===================== CONFIG: Flow/Odometry =====================
PMW3901 pmw;
constexpr uint32_t FLOW_DT_MS   = 10;     // read PMW every 10 ms (~100 Hz)
constexpr uint32_t PRINT_DT_MS  = 100;    // print @10 Hz

// Sign flips if your sensor axes are reversed (set to -1 to flip)
const int8_t FLOW_SIGN_X = +1;  // +1 normal, -1 flipped
const int8_t FLOW_SIGN_Y = -1;  // many setups need Y inverted

// Convert PMW3901 "pixel" motion to angle (radians). Tune these.
float K_pix_to_rad_x = 0.00153f; // example from your notes — tune!
float K_pix_to_rad_y = 0.00153f; // keep symmetric initially

// ===================== Height filtering ==========================
// Median-of-N (odd) + EMA + slew limiter
constexpr int    H_BUF_N     = 5;      // median window
constexpr float  H_ALPHA     = 0.55f;  // EMA weight
constexpr int    H_SLEW_MM   = 500;    // per-update limit (mm)

int16_t h_buf[H_BUF_N] = {0};
uint8_t h_idx = 0, h_count = 0;
float   h_ema = 400.0f;  // initial guess (mm)
int16_t last_height_mm = -1;

static int16_t median_of(int16_t* arr, int n) {
  // simple insertion sort copy
  int16_t tmp[H_BUF_N];
  for (int i=0;i<n;i++) tmp[i]=arr[i];
  for (int i=1;i<n;i++){
    int16_t key = tmp[i];
    int j = i-1;
    while (j>=0 && tmp[j] > key){ tmp[j+1]=tmp[j]; j--; }
    tmp[j+1]=key;
  }
  return tmp[n/2];
}

int16_t filter_height_mm(int16_t raw_mm){
  if (raw_mm <= 0) return last_height_mm > 0 ? last_height_mm : raw_mm;
  h_buf[h_idx] = raw_mm;
  h_idx = (h_idx + 1) % H_BUF_N;
  if (h_count < H_BUF_N) h_count++;

  int16_t med = (h_count==H_BUF_N) ? median_of(h_buf, H_BUF_N) : raw_mm;

  // EMA
  h_ema = H_ALPHA * med + (1.0f - H_ALPHA) * h_ema;

  // Slew
  int16_t cand = (int16_t)roundf(h_ema);
  if (last_height_mm >= 0) {
    int16_t diff = cand - last_height_mm;
    if (diff >  H_SLEW_MM) cand = last_height_mm + H_SLEW_MM;
    if (diff < -H_SLEW_MM) cand = last_height_mm - H_SLEW_MM;
  }
  last_height_mm = cand;
  return cand;
}

// ===================== Odometry state ============================
volatile long x_mm = 0;
volatile long y_mm = 0;
float vx_mmps = 0.0f, vy_mmps = 0.0f;

uint32_t t_next_flow  = 0;
uint32_t t_next_print = 0;

// ===================== Utils ====================================
void driveAllXShutLow() {
  for (uint8_t i = 0; i < NUM_SENSORS; i++) {
    pinMode(cfgs[i].xshut_pin, OUTPUT);
    digitalWrite(cfgs[i].xshut_pin, LOW);
  }
}

bool bringUpOne(uint8_t i) {
  // Bring one sensor up on default address 0x29, then readdress
  digitalWrite(cfgs[i].xshut_pin, HIGH);
  delay(3); // allow boot
  tof[i].setTimeout(100);
  if (!tof[i].init()) {
    Serial.printf("  FAIL: %s failed init at default 0x29\n", cfgs[i].role);
    digitalWrite(cfgs[i].xshut_pin, LOW);
    delay(2);
    return false;
  }
  tof[i].setAddress(cfgs[i].addr);
  tof[i].setDistanceMode(DIST_MODE);
  tof[i].setMeasurementTimingBudget(TIMING_BUDGET_MS);
  tof[i].startContinuous(MEAS_PERIOD_MS);
  Serial.printf("  OK: %s @ 0x%02X\n", cfgs[i].role, cfgs[i].addr);
  return true;
}

void i2cScanOnce() {
  Serial.println("I2C scan:");
  uint8_t count = 0;
  for (uint8_t addr = 1; addr < 127; addr++) {
    Wire.beginTransmission(addr);
    if (Wire.endTransmission() == 0) {
      Serial.printf("  0x%02X\n", addr);
      count++;
    }
  }
  if (count == 0) Serial.println("  (no devices)");
}

// ===================== SETUP =====================================
void setup() {
  Serial.begin(115200);
  delay(100);

  Serial.println("\n=== Combined PMW3901 Odom + VL53L1X Obstacles/Height ===");

  // I2C
  Wire.begin(PIN_SDA, PIN_SCL);
  Wire.setClock(400000);

  // SPI
  SPI.end(); // ensure clean re-config
  SPI.begin(PIN_SCK, PIN_MISO, PIN_MOSI, PIN_CS);

  // PMW3901
  if (!pmw.begin(PIN_CS)) {
    Serial.println("PMW3901 begin() FAILED — check wiring/pins.");
  } else {
    Serial.println("PMW3901 OK.");
  }

  // VL53L1X (readdress via XSHUT)
  driveAllXShutLow();
  delay(10);
  Serial.println("Bring-up VL53L1X sensors...");
  for (uint8_t i = 0; i < NUM_SENSORS; i++) {
    if (!bringUpOne(i)) {
      Serial.printf("Retry %s...\n", cfgs[i].role);
      delay(10);
      if (!bringUpOne(i)) {
        Serial.printf("GIVE UP: %s\n", cfgs[i].role);
      }
    }
  }

  i2cScanOnce();

  t_next_flow  = millis() + FLOW_DT_MS;
  t_next_print = millis() + PRINT_DT_MS;
}

// ===================== LOOP ======================================
void loop() {
  uint32_t now = millis();

  // ====== Read height & obstacles ======
  int16_t raw_height = -1;
  int16_t mm_left=-1, mm_right=-1, mm_front=-1, mm_back=-1;
  bool obstacle_left=false, obstacle_right=false, obstacle_front=false, obstacle_back=false;

  for (uint8_t i = 0; i < NUM_SENSORS; i++) {
    int16_t mm = -1;
    if (tof[i].dataReady()) {
      mm = tof[i].read();     // returns distance in mm; reading clears data-ready
    } else {
      mm = tof[i].ranging_data.range_mm; // last value (may be stale if no new data)
    }

    if (cfgs[i].is_height) {
      raw_height = mm;
    } else {
      bool is_obst = (mm > 0 && mm <= (int)OBSTACLE_LIMIT_MM);
      if      (strcmp(cfgs[i].role, "Left")  == 0) { obstacle_left  = is_obst; mm_left  = mm; }
      else if (strcmp(cfgs[i].role, "Right") == 0) { obstacle_right = is_obst; mm_right = mm; }
      else if (strcmp(cfgs[i].role, "Front") == 0) { obstacle_front = is_obst; mm_front = mm; }
      else if (strcmp(cfgs[i].role, "Back")  == 0) { obstacle_back  = is_obst; mm_back  = mm; }
    }
  }

  int16_t h_mm = filter_height_mm(raw_height);

  // ====== Odometry update from PMW3901 ======
  while ((int32_t)(now - t_next_flow) >= 0) {
    int16_t dx_pix = 0, dy_pix = 0;
    if (pmw.readMotionCount(&dx_pix, &dy_pix)) {
      // Guard for zero/invalid height
      int16_t h_guard = (h_mm > 1 ? h_mm : 1);
      float height_m = h_guard * 0.001f;

      // Convert pixel motion to ground displacement (mm) using small-angle approx:
      // ground_dx ≈ height * angle_x  where angle_x ≈ dx_pix * K_pix_to_rad_x
      float ang_x = (float)(FLOW_SIGN_X * dx_pix) * K_pix_to_rad_x;
      float ang_y = (float)(FLOW_SIGN_Y * dy_pix) * K_pix_to_rad_y;

      float dX_mm = height_m * ang_x * 1000.0f;
      float dY_mm = height_m * ang_y * 1000.0f;

      x_mm += (long)llround(dX_mm);
      y_mm += (long)llround(dY_mm);

      // simple velocity estimate (per FLOW_DT)
      vx_mmps = dX_mm * (1000.0f / FLOW_DT_MS);
      vy_mmps = dY_mm * (1000.0f / FLOW_DT_MS);
    }

    t_next_flow += FLOW_DT_MS;
  }

  // ====== Print @10 Hz ======
  if ((int32_t)(now - t_next_print) >= 0) {
    Serial.print("H="); Serial.print(h_mm); Serial.print(" mm  ");
    Serial.print("Pose=("); Serial.print((long)x_mm); Serial.print(", ");
    Serial.print((long)y_mm); Serial.print(") mm  ");
    Serial.print("v=("); Serial.print((int)vx_mmps); Serial.print(", ");
    Serial.print((int)vy_mmps); Serial.print(") mm/s  ");

    Serial.print("Obstacle Left ");  Serial.print(mm_left);  Serial.print("mm  ");
    Serial.print("Obstacle Right "); Serial.print(mm_right); Serial.print("mm  ");
    Serial.print("Obstacle Front "); Serial.print(mm_front); Serial.print("mm  ");
    Serial.print("Obstacle Back ");  Serial.print(mm_back);  Serial.print("mm");
    Serial.println();
    t_next_print += PRINT_DT_MS;
  }
}