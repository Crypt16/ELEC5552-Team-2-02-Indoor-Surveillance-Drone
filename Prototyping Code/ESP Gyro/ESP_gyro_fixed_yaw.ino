// T-Display S3 + GY-521 (MPU-6050) — reduced yaw drift
// Exact same TFT_eSPI drawing style as your screen test.

#include <SPI.h>
#include <TFT_eSPI.h>
#include <Wire.h>
#include <math.h>

TFT_eSPI tft = TFT_eSPI();

// ---- Pins ----
#define I2C_SDA 21
#define I2C_SCL 16
#define BTN_PIN 14   // press to zero yaw (active-LOW)

// ---- MPU-6050 registers ----
static const uint8_t MPU_ADDR_0 = 0x68;
static const uint8_t MPU_ADDR_1 = 0x69;
uint8_t MPU_ADDR = MPU_ADDR_0;

#define REG_SMPLRT_DIV   0x19
#define REG_CONFIG       0x1A
#define REG_GYRO_CONFIG  0x1B
#define REG_ACCEL_CONFIG 0x1C
#define REG_INT_ENABLE   0x38
#define REG_ACCEL_XOUT_H 0x3B
#define REG_PWR_MGMT_1   0x6B
#define REG_WHO_AM_I     0x75

// ---- scales ----
// Use ±250 dps for finer resolution:
static const float GYRO_SENS  = 131.0f;   // LSB/°/s  (±250 dps)
static const float ACCEL_SENS = 16384.0f; // LSB/g    (±2g)

// ---- complementary filter ----
float roll_f = 0.0f, pitch_f = 0.0f, yaw_f = 0.0f;
const float alpha = 0.98f; // 98% gyro / 2% accel for tilt

// Gyro bias in °/s (we’ll work in physical units to avoid confusion)
float gbx = 0.0f, gby = 0.0f, gbz = 0.0f;

// In-run yaw zero
float yaw_zero = 0.0f;

// Timing
uint32_t lastMicros = 0;
uint32_t lastDrawMs = 0;

// Button debounce
int lastBtn = HIGH;
uint32_t lastBtnMs = 0;

// ---- I2C helpers ----
bool i2cWrite8(uint8_t addr, uint8_t reg, uint8_t data) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.write(data);
  return Wire.endTransmission() == 0;
}

bool i2cReadBytes(uint8_t addr, uint8_t reg, uint8_t* buf, size_t n) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom((int)addr, (int)n) != (int)n) return false;
  for (size_t i=0;i<n;++i) buf[i] = Wire.read();
  return true;
}

bool mpuProbe(uint8_t addr) {
  uint8_t who=0;
  if (!i2cReadBytes(addr, REG_WHO_AM_I, &who, 1)) return false;
  return (who == 0x68);
}

bool mpuInit() {
  if      (mpuProbe(MPU_ADDR_0)) MPU_ADDR = MPU_ADDR_0;
  else if (mpuProbe(MPU_ADDR_1)) MPU_ADDR = MPU_ADDR_1;
  else return false;

  // Wake + config
  if (!i2cWrite8(MPU_ADDR, REG_PWR_MGMT_1, 0x00)) return false; // wake
  delay(100);
  i2cWrite8(MPU_ADDR, REG_SMPLRT_DIV, 9);      // 100 Hz
  i2cWrite8(MPU_ADDR, REG_CONFIG, 0x03);       // ~42 Hz DLPF
  i2cWrite8(MPU_ADDR, REG_GYRO_CONFIG, 0x00);  // FS_SEL=0 => ±250 dps
  i2cWrite8(MPU_ADDR, REG_ACCEL_CONFIG, 0x00); // ±2g
  i2cWrite8(MPU_ADDR, REG_INT_ENABLE, 0x00);
  return true;
}

bool mpuReadRaw(int16_t &ax,int16_t &ay,int16_t &az,int16_t &gx,int16_t &gy,int16_t &gz) {
  uint8_t b[14];
  if (!i2cReadBytes(MPU_ADDR, REG_ACCEL_XOUT_H, b, 14)) return false;
  ax=(int16_t)((b[0]<<8)|b[1]); ay=(int16_t)((b[2]<<8)|b[3]); az=(int16_t)((b[4]<<8)|b[5]);
  gx=(int16_t)((b[8]<<8)|b[9]); gy=(int16_t)((b[10]<<8)|b[11]); gz=(int16_t)((b[12]<<8)|b[13]);
  return true;
}

// Long, quiet calibration; keep sensor still
void calibrateGyro(uint16_t N=1500) {
  long sx=0, sy=0, sz=0;
  for (uint16_t i=0;i<N;++i) {
    int16_t ax,ay,az,gx,gy,gz;
    if (mpuReadRaw(ax,ay,az,gx,gy,gz)) {
      sx += gx; sy += gy; sz += gz;
    }
    delay(2);
  }
  // Convert to °/s biases
  gbx = (sx / (float)N) / GYRO_SENS;
  gby = (sy / (float)N) / GYRO_SENS;
  gbz = (sz / (float)N) / GYRO_SENS;
}

// Helpers
static inline float rad2deg(float r){ return r * 57.2957795f; }
static inline float deg2rad(float d){ return d * 0.01745329252f; }
static inline float wrap180(float a){ while (a>180) a-=360; while (a<-180) a+=360; return a; }

void updateFilter() {
  int16_t axr,ayr,azr,gxr,gyr,gzr;
  if (!mpuReadRaw(axr,ayr,azr,gxr,gyr,gzr)) return;

  // Convert to g and °/s
  float ax = axr / ACCEL_SENS;
  float ay = ayr / ACCEL_SENS;
  float az = azr / ACCEL_SENS;

  float gx_dps = (gxr / GYRO_SENS) - gbx;
  float gy_dps = (gyr / GYRO_SENS) - gby;
  float gz_dps = (gzr / GYRO_SENS) - gbz;

  // Small deadband to avoid integrating noise
  if (fabsf(gx_dps) < 0.03f) gx_dps = 0;
  if (fabsf(gy_dps) < 0.03f) gy_dps = 0;
  if (fabsf(gz_dps) < 0.03f) gz_dps = 0;

  // dt
  uint32_t now = micros();
  float dt = (now - lastMicros) * 1e-6f;
  lastMicros = now;
  if (dt <= 0 || dt > 0.2f) dt = 0.01f;

  // Accel tilt (deg)
  float rollAcc  = rad2deg(atan2f(ay, az));
  float pitchAcc = rad2deg(atan2f(-ax, sqrtf(ay*ay + az*az)));

  // Integrate tilt from gyro (deg)
  float rollGy  = roll_f  + gx_dps * dt;
  float pitchGy = pitch_f + gy_dps * dt;

  // Complementary blend for tilt
  roll_f  = alpha * rollGy  + (1.0f - alpha) * rollAcc;
  pitch_f = alpha * pitchGy + (1.0f - alpha) * pitchAcc;

  // ---- Better yaw rate using full kinematics ----
  // p,q,r in rad/s:
  float p = deg2rad(gx_dps), q = deg2rad(gy_dps), r = deg2rad(gz_dps);
  float phi = deg2rad(roll_f);
  float th  = deg2rad(pitch_f);
  float cth = cosf(th);
  float yawRate_rad;
  if (fabsf(cth) > 0.1f) {
    yawRate_rad = (q * sinf(phi) + r * cosf(phi)) / cth;
  } else {
    yawRate_rad = r; // avoid blow-up near ±90° pitch
  }
  float yawRate_dps = rad2deg(yawRate_rad);

  // Integrate yaw
  yaw_f += yawRate_dps * dt;

  // ---- In-run bias trimming when stationary ----
  // Gate: |acc| ~ 1g and small rates for a short window
  static float still_ms = 0.0f;
  float accNorm  = sqrtf(ax*ax + ay*ay + az*az);
  float gyroNorm = sqrtf(gx_dps*gx_dps + gy_dps*gy_dps + gz_dps*gz_dps);

  if (fabsf(accNorm - 1.0f) < 0.02f && gyroNorm < 0.5f) {
    still_ms += dt * 1000.0f;
    if (still_ms > 600.0f) {
      // Adapt biases very slowly toward current tiny rates
      float k = 0.0015f; // adaptation gain per update (~0.15%/s at 100 Hz)
      gbx += k * gx_dps;
      gby += k * gy_dps;
      gbz += k * gz_dps;
    }
  } else {
    still_ms = 0.0f;
  }
}

void drawPage(float yaw_show) {
  // Same drawing style as your test
  tft.fillScreen(TFT_BLACK);
  tft.setCursor(0, 0, 4);

  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.println("MPU-6050  Orientation\n");

  tft.setTextColor(TFT_GREEN, TFT_BLACK);
  tft.print("Roll : ");  tft.print(roll_f, 1);  tft.println(" deg");

  tft.setTextColor(TFT_CYAN, TFT_BLACK);
  tft.print("Pitch: ");  tft.print(pitch_f, 1); tft.println(" deg");

  tft.setTextColor(TFT_YELLOW, TFT_BLACK);
  tft.print("Yaw  : ");  tft.print(yaw_show, 1); tft.println(" deg*");

  tft.setTextColor(TFT_RED, TFT_BLACK);
  tft.println("\n* gyro-only (drifts)");
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.println("IO14: zero yaw");
}

void setup() {
  pinMode(BTN_PIN, INPUT_PULLUP);

  // Display (exact pattern as your screen test)
  tft.init();
  tft.fillScreen(TFT_BLACK);
  tft.setCursor(0, 0, 4);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.println("Initialised default\n");
  tft.println("Setting up IMU...");

  // I2C
  Wire.begin(I2C_SDA, I2C_SCL, 400000);

  if (!mpuInit()) {
    tft.setTextColor(TFT_RED, TFT_BLACK);
    tft.println("MPU not found at 0x68/0x69!");
    while (true) delay(100);
  }

  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.println("Calibrating gyro...");
  calibrateGyro(1500);

  // Seed tilt
  int16_t ax,ay,az,gx,gy,gz;
  if (mpuReadRaw(ax,ay,az,gx,gy,gz)) {
    float axg=ax/ACCEL_SENS, ayg=ay/ACCEL_SENS, azg=az/ACCEL_SENS;
    roll_f  = rad2deg(atan2f(ayg, azg));
    pitch_f = rad2deg(atan2f(-axg, sqrtf(ayg*ayg + azg*azg)));
    yaw_f   = 0.0f;
  }

  lastMicros = micros();
  lastDrawMs = millis();

  drawPage(0.0f);
}

void loop() {
  updateFilter();

  // IO14 → zero yaw (debounced)
  int b = digitalRead(BTN_PIN);
  uint32_t now = millis();
  if (b != lastBtn && (now - lastBtnMs) > 25) {
    lastBtnMs = now;
    if (b == LOW) yaw_zero = yaw_f;  // capture current as zero
    lastBtn = b;
  }

  if (now - lastDrawMs >= 100) {
    lastDrawMs = now;
    float yaw_show = wrap180(yaw_f - yaw_zero);
    drawPage(yaw_show);
  }

  delay(2);
}
