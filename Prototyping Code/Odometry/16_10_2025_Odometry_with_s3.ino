/*
  ESP32-S3 Odometry:
    - PMW3901 (Bitcraze driver) over SPI for optical flow
    - VL53L1X (Pololu driver) over I2C for height

  Pins (ESP32-S3):
    PMW3901 SPI:
      CS  = 2
      MOSI= 42
      MISO= 41
      SCK = 40
    VL53L1X I2C (height):
      SDA = 8
      SCL = 9
      XSHUT (enable) = 18

  Libraries:
    - "Bitcraze_PMW3901" (Bitcraze)
    - "VL53L1X" (Pololu)
*/

#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <VL53L1X.h>
#include "Bitcraze_PMW3901.h"

// ---------------- Pins ----------------
constexpr int PIN_SDA        = 8;   // VL53L1X SDA
constexpr int PIN_SCL        = 9;   // VL53L1X SCL
constexpr int PIN_XSHUT_TOF  = 18;  // VL53L1X XSHUT (HIGH = enabled)

constexpr int PIN_MISO = 41;  // PMW3901
constexpr int PIN_MOSI = 42;
constexpr int PIN_SCK  = 40;
constexpr int PIN_CS   = 2;

// ---------------- Rates ----------------
constexpr uint32_t FLOW_DT_MS  = 10;   // ~100 Hz
constexpr uint32_t TOF_DT_MS   = 10;   // poll gate; new data comes every ~100 ms
constexpr uint32_t PRINT_DT_MS = 100;  // 10 Hz print

// ================ Flow scale =================
float K_pix_to_rad_x = 0.00193f;  // counts -> rad (X)
float K_pix_to_rad_y = 0.00193f;  // counts -> rad (Y)

constexpr int   DEAD   = 0;     // counts
constexpr int   H_MIN  = 60;    // mm
constexpr int   H_MAX  = 4000;  // mm
float alpha_v = 0.3f;           // velocity LPF

// ---------------- Globals ----------------
VL53L1X tof;                    // Pololu
Bitcraze_PMW3901 flow(PIN_CS);  // Bitcraze

int   last_height_mm = 200;     // mm
long  x_mm = 0, y_mm = 0;       // pose
float vx_filt = 0, vy_filt = 0; // mm/s

static inline void quickDelay(uint32_t ms) {
  uint32_t t0 = millis();
  while (millis() - t0 < ms) delay(1);
}

// ------------- PMW3901 init -------------
bool initFlow() {
  pinMode(PIN_CS, OUTPUT);
  digitalWrite(PIN_CS, HIGH);
  SPI.begin(PIN_SCK, PIN_MISO, PIN_MOSI, PIN_CS);

  if (!flow.begin()) {
    Serial.println("[PMW3901] begin() failed");
    return false;
  }
  Serial.println("[PMW3901] OK");
  return true;
}

// ------------- VL53L1X init -------------
bool initToF() {
  // Clean boot via XSHUT
  pinMode(PIN_XSHUT_TOF, OUTPUT);
  digitalWrite(PIN_XSHUT_TOF, LOW);  quickDelay(5);
  digitalWrite(PIN_XSHUT_TOF, HIGH); quickDelay(5);

  Wire.begin(PIN_SDA, PIN_SCL);
  Wire.setClock(400000);

  tof.setBus(&Wire);
  tof.setTimeout(150);
  if (!tof.init()) {
    Serial.println("[VL53L1X] init() failed");
    return false;
  }

  tof.setDistanceMode(VL53L1X::Long);
  tof.setMeasurementTimingBudget(80000); // 80 ms
  tof.startContinuous(100);              // >= budget, ~10 Hz dataReady()

  Serial.println("[VL53L1X] OK (Pololu)");
  return true;
}

// ------------- Height read (mm) -------------
// Returns a pair: (height_mm, new_sample_flag via out param)
int readHeightMM(bool &new_sample) {
  new_sample = false;

  // Only read when a *new* sample is ready
  if (!tof.dataReady()) return last_height_mm;

  uint16_t mm = tof.read();  // grabs the new sample and clears DRDY
  new_sample = true;

  if (mm == 0 || mm > H_MAX || (int)mm < H_MIN) return last_height_mm;
  return (int)mm;
}

// ------------- Flow read wrapper -------------
bool readFlowCounts(int16_t &dx, int16_t &dy) {
  flow.readMotionCount(&dx, &dy);  // your lib returns void
  return true;
}

// ------------- Setup -------------
void setup() {
  Serial.begin(115200);
  quickDelay(200);
  Serial.println("\n[Boot] PMW3901 odometry + VL53L1X height (Pololu, SDA=8 SCL=9; SPI CS=2/MOSI=42/MISO=41/SCK=40)");

  bool ok_flow = initFlow();
  bool ok_tof  = initToF();

  if (!ok_flow) Serial.println("! PMW3901 failed init");
  if (!ok_tof)  Serial.println("! VL53L1X failed init");
  if (!ok_flow || !ok_tof) Serial.println("Continuing anyway (check wiring if readings look off)");
}

// ------------- Main loop -------------
void loop() {
  static uint32_t t_toF=0, t_flow=0, t_pr=0;
  uint32_t now = millis();

  // ---- Height update (only when new data is ready) ----
  while ((int32_t)(now - t_toF) >= 0) {
    bool new_h = false;
    int h = readHeightMM(new_h);
    if (new_h) {
      last_height_mm = h;
    }
    t_toF += TOF_DT_MS;
  }

  // ---- Flow + odometry ----
  while ((int32_t)(now - t_flow) >= 0) {
    int16_t dx=0, dy=0;
    if (readFlowCounts(dx, dy)) {
      if (abs(dx) <= DEAD) dx = 0;
      if (abs(dy) <= DEAD) dy = 0;

      const float dt     = FLOW_DT_MS * 1e-3f;
      const float dps_x  = dx / dt;   // counts/s
      const float dps_y  = dy / dt;

      const float thdot_x = dps_x * K_pix_to_rad_x; // rad/s
      const float thdot_y = dps_y * K_pix_to_rad_y; // rad/s

      const float h_m = last_height_mm * 1e-3f;
      float vx =  h_m * thdot_x * 1000.0f; // mm/s
      float vy = -h_m * thdot_y * 1000.0f; // mm/s (flip sign to match your board)

      if (alpha_v > 0.0f && alpha_v < 1.0f) {
        vx_filt = alpha_v * vx + (1.0f - alpha_v) * vx_filt;
        vy_filt = alpha_v * vy + (1.0f - alpha_v) * vy_filt;
      } else {
        vx_filt = vx; vy_filt = vy;
      }

      x_mm += (long)(vx_filt * dt);
      y_mm += (long)(vy_filt * dt);
    }
    t_flow += FLOW_DT_MS;
  }

  // ---- Debug print ----
  while ((int32_t)(now - t_pr) >= 0) {
    // Mark with '*' if at least one new height sample arrived since last print
    bool fresh = tof.dataReady(); // quick peek; if true, another sample is queued
    Serial.print("H="); Serial.print(last_height_mm);
    Serial.print(" mm"); if (fresh) Serial.print(" *");
    Serial.print("  Pose=("); Serial.print(x_mm); Serial.print(", "); Serial.print(y_mm); Serial.print(") mm");
    Serial.print("  v(mm/s)=("); Serial.print((long)vx_filt); Serial.print(", "); Serial.print((long)vy_filt); Serial.println(")");
    t_pr += PRINT_DT_MS;
  }
}