// main.cpp - Navigation System with Web Interface
#include <Wire.h>
#include <SPI.h>
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <VL53L1X.h>
#include "Bitcraze_PMW3901.h"
#include "web_ui.h"  // Separate HTML file

// ===================== WIFI CONFIG =====================
#ifndef WIFI_STA_SSID
  #define WIFI_STA_SSID ""
#endif
#ifndef WIFI_STA_PASS
  #define WIFI_STA_PASS ""
#endif
static const char* AP_SSID = "NavDrone";
static const char* AP_PASS = "12345678";

AsyncWebServer server(80);
AsyncWebSocket ws("/ws");

// ===================== PINS =====================
// I2C Bus 0 (Wire) - VL53L1X sensors
constexpr int PIN_SDA_VL  = 22;
constexpr int PIN_SCL_VL  = 21;

// I2C Bus 1 (Wire1) - MPU6500 IMU
constexpr int PIN_SDA_IMU = 21;
constexpr int PIN_SCL_IMU = 22;

// SPI - PMW3901 Optical Flow
constexpr int PIN_MISO = 19;
constexpr int PIN_MOSI = 23;
constexpr int PIN_SCK  = 18;
constexpr int PIN_CS   = 5;

// VL53L1X XSHUT map - 4 sensors (Bottom-Right is faulty)
// NOTE: If a pin doesn't work, try: 25, 26, 27, 33, 32, 35, 34
// AVOID: 0, 2, 15 (boot pins), 6-11 (flash), 34-39 (input only)
struct SensorCfg { const char* role; int xshut_pin; uint8_t addr; bool is_bottom; };
SensorCfg g_cfgs[] = {
  { "Left",        32, 0x30, false },  // Obstacle detection
  { "Right",       16, 0x31, false },  // Obstacle detection
  { "Forward",     17, 0x32, false },  // Obstacle detection
  { "Bottom-Left",  4, 0x33, true  },  // Height sensor (left)
  // { "Bottom-Right",13, 0x34, true  },  // FAULTY - Commented out
};
constexpr size_t NUM_SENSORS = sizeof(g_cfgs)/sizeof(g_cfgs[0]);

// ===================== OBSTACLE & VL53 SETTINGS =====================
constexpr uint16_t THRESH_LEFT   = 500;
constexpr uint16_t THRESH_RIGHT  = 500;
constexpr uint16_t THRESH_FWD    = 500;

// ToF Sensor Settings - Optimized for 0.1s (100ms) read rate
// Using longer timing for reliability since we only read every 100ms
constexpr uint16_t VL_TIMING_BUDGET_MS = 50;  // 50ms measurement time
constexpr uint16_t VL_INTER_MEAS_MS = 100;     // 100ms between measurements

// ===================== ERROR REJECTION PARAMETERS =====================
// Buffer size for all sensors
constexpr int BUFFER_SIZE = 5;
constexpr int MAX_CONSECUTIVE_REJECTIONS = 10;  // NEW: Threshold for auto-reset

// ToF Sensor Valid Ranges (mm)
constexpr uint16_t TOF_MIN_VALID = 30;      // Minimum valid distance (mm)
constexpr uint16_t TOF_MAX_VALID = 4000;    // Maximum valid distance (mm)
constexpr uint16_t TOF_MAX_CHANGE = 500;    // Maximum change between readings (mm)

// IMU Accelerometer Valid Ranges (g)
constexpr float ACCEL_MIN_VALID = -3.0f;    // Minimum valid acceleration
constexpr float ACCEL_MAX_VALID = 3.0f;     // Maximum valid acceleration
constexpr float ACCEL_MAX_CHANGE = 1.0f;    // Maximum change between readings

// IMU Gyroscope Valid Ranges (dps)
constexpr float GYRO_MIN_VALID = -500.0f;   // Minimum valid gyro rate
constexpr float GYRO_MAX_VALID = 500.0f;    // Maximum valid gyro rate
constexpr float GYRO_MAX_CHANGE = 100.0f;   // Maximum change between readings

// Optical Flow Valid Ranges (mm/s)
constexpr float VELOCITY_MIN_VALID = -2000.0f;  // Minimum valid velocity
constexpr float VELOCITY_MAX_VALID = 2000.0f;   // Maximum valid velocity
constexpr float VELOCITY_MAX_CHANGE = 500.0f;   // Maximum change between readings

// Cross-Validation: Optical Flow vs IMU Roll
// Reject jittery roll changes when optical flow shows no corresponding Y motion
constexpr float OPFLOW_Y_MOTION_THRESHOLD = 50.0f;  // mm/s - minimum Y motion to allow roll changes
constexpr float GYRO_X_REJECT_THRESHOLD = 5.0f;     // dps - reject gyro X above this if no opflow motion

// ===================== BUFFERED SENSOR CLASS =====================
template<typename T>
class BufferedSensor {
private:
  T buffer[BUFFER_SIZE];
  int count;
  T min_valid, max_valid, max_change;
  T last_valid;
  bool has_last_valid;
  int consecutive_rejections;  // NEW: Track consecutive rejections
  
public:
  BufferedSensor(T min_val, T max_val, T max_chg) 
    : count(0), min_valid(min_val), max_valid(max_val), max_change(max_chg), 
      last_valid(0), has_last_valid(false), consecutive_rejections(0) {}
  
  // Add new reading with error rejection
  bool update(T new_value) {
    // Check if value is in valid range
    if(new_value < min_valid || new_value > max_valid) {
      consecutive_rejections++;
      
      // NEW: If too many rejections, force reset (sensor may have moved to new valid range)
      if(consecutive_rejections >= MAX_CONSECUTIVE_REJECTIONS) {
        Serial.printf("[BUFFER] Force reset after %d rejections (out of range)\n", consecutive_rejections);
        clear();
        consecutive_rejections = 0;
        // Fall through to accept this value
      } else {
        return false;  // Reject out of range
      }
    }
    
    // Check if change is too large (skip for first reading)
    if(has_last_valid) {
      T change = (new_value > last_valid) ? (new_value - last_valid) : (last_valid - new_value);
      if(change > max_change) {
        consecutive_rejections++;
        
        // NEW: If too many rejections, assume legitimate rapid change occurred
        if(consecutive_rejections >= MAX_CONSECUTIVE_REJECTIONS) {
          Serial.print("[BUFFER] Force reset after ");
          Serial.print(consecutive_rejections);
          Serial.print(" rejections (large change: ");
          Serial.print((float)change);
          Serial.println(")");
          clear();
          consecutive_rejections = 0;
          // Fall through to accept this value as new baseline
        } else {
          return false;  // Reject large jump
        }
      }
    }
    
    // Value accepted - reset rejection counter
    consecutive_rejections = 0;
    
    // Add to buffer
    if(count < BUFFER_SIZE) {
      buffer[count++] = new_value;
    } else {
      // Shift buffer and add new value
      for(int i = 1; i < BUFFER_SIZE; i++) {
        buffer[i-1] = buffer[i];
      }
      buffer[BUFFER_SIZE-1] = new_value;
    }
    
    last_valid = new_value;
    has_last_valid = true;
    return true;
  }
  
  // Get filtered value (median for robustness)
  T getValue() {
    if(count == 0) return last_valid;
    
    // Create sorted copy
    T sorted[BUFFER_SIZE];
    for(int i = 0; i < count; i++) {
      sorted[i] = buffer[i];
    }
    
    // Bubble sort
    for(int i = 0; i < count-1; i++) {
      for(int j = 0; j < count-i-1; j++) {
        if(sorted[j] > sorted[j+1]) {
          T temp = sorted[j];
          sorted[j] = sorted[j+1];
          sorted[j+1] = temp;
        }
      }
    }
    
    // Return median
    return sorted[count/2];
  }
  
  bool hasValue() { return count > 0; }
  int getCount() { return count; }
  int getRejectionCount() { return consecutive_rejections; }  // NEW: For debugging
  void clear() { count = 0; has_last_valid = false; }
};

// ===================== PMW3901 ODOMETRY (ENABLED) =====================
constexpr uint32_t FLOW_DT_MS = 10;     // 100 Hz odometry updates
// Gains (adjust signs to match your drone orientation)
float K_pix_to_rad_x = 0.00193f;  // rad per count (sensor X -> world +Y)
float K_pix_to_rad_y = 0.00193f;  // rad per count (sensor Y -> world -X)
// Deadband on raw counts
constexpr int DEAD = 0;           // set non-zero if needed
// Velocity LPF
float alpha_v = 0.30f;            // 0..1, higher = snappier

// Height limits (for sanity checks)
constexpr int H_MIN = 60;         // mm
constexpr int H_MAX = 4000;       // mm

// ===================== MPU6500 IMU =====================
#define MPU_ADDR 0x68
#define REG_WHO_AM_I 0x75
#define REG_PWR_MGMT_1 0x6B
#define REG_CONFIG 0x1A
#define REG_GYRO_CONFIG 0x1B
#define REG_ACCEL_CONFIG 0x1C
#define REG_ACCEL_CONFIG2 0x1D
#define REG_SMPLRT_DIV 0x19
#define REG_ACCEL_XOUT_H 0x3B

static const float GYRO_SENS = 131.0f;     // ±250 dps
static const float ACCEL_SENS = 16384.0f;  // ±2g

// IMU state
static bool imu_ok = false;
static uint8_t mpu_address = 0x68;  // Will be updated if found at 0x69
static float gbx=0, gby=0, gbz=0;  // Gyro biases
static float roll_f=0, pitch_f=0, yaw_f=0;  // Filtered attitude
static float p_rate_dps=0, q_rate_dps=0, r_rate_dps=0;  // Gyro rates
static float ax_g=0, ay_g=0, az_g=0;  // Accelerometer in g's
static uint32_t lastIMUMicros = 0;
static const float alphaTilt = 0.98f;  // Complementary filter

// ===================== STATE TYPES =====================
struct Ranges{ uint16_t left,right,fwd,bottom_left; };  // 4 sensors (BR removed)
struct Step{ float dx_m; float dy_m; };

// ===================== GLOBAL STATE =====================
VL53L1X g_sensors[NUM_SENSORS];
Bitcraze_PMW3901 g_flow(PIN_CS);  // Optical flow sensor

// Buffered sensors for error rejection
BufferedSensor<uint16_t> buf_tof_left(TOF_MIN_VALID, TOF_MAX_VALID, TOF_MAX_CHANGE);
BufferedSensor<uint16_t> buf_tof_right(TOF_MIN_VALID, TOF_MAX_VALID, TOF_MAX_CHANGE);
BufferedSensor<uint16_t> buf_tof_forward(TOF_MIN_VALID, TOF_MAX_VALID, TOF_MAX_CHANGE);
BufferedSensor<uint16_t> buf_tof_bottom_left(TOF_MIN_VALID, TOF_MAX_VALID, TOF_MAX_CHANGE);

BufferedSensor<float> buf_accel_x(ACCEL_MIN_VALID, ACCEL_MAX_VALID, ACCEL_MAX_CHANGE);
BufferedSensor<float> buf_accel_y(ACCEL_MIN_VALID, ACCEL_MAX_VALID, ACCEL_MAX_CHANGE);
BufferedSensor<float> buf_accel_z(ACCEL_MIN_VALID, ACCEL_MAX_VALID, ACCEL_MAX_CHANGE);

BufferedSensor<float> buf_gyro_x(GYRO_MIN_VALID, GYRO_MAX_VALID, GYRO_MAX_CHANGE);
BufferedSensor<float> buf_gyro_y(GYRO_MIN_VALID, GYRO_MAX_VALID, GYRO_MAX_CHANGE);
BufferedSensor<float> buf_gyro_z(GYRO_MIN_VALID, GYRO_MAX_VALID, GYRO_MAX_CHANGE);

BufferedSensor<float> buf_vel_x(VELOCITY_MIN_VALID, VELOCITY_MAX_VALID, VELOCITY_MAX_CHANGE);
BufferedSensor<float> buf_vel_y(VELOCITY_MIN_VALID, VELOCITY_MAX_VALID, VELOCITY_MAX_CHANGE);

enum class SideIdx:int { LEFT=0, RIGHT=1, FORWARD=2, BOTTOM_LEFT=3, UNKNOWN=-1 };
uint16_t obstacleThresholdFor(const char* r){
  if(!strcmp(r,"Left"))return THRESH_LEFT;
  if(!strcmp(r,"Right"))return THRESH_RIGHT;
  if(!strcmp(r,"Forward"))return THRESH_FWD;
  return 0;  // No obstacle threshold for bottom sensors
}
SideIdx sideIndexFor(const char* r){
  if(!strcmp(r,"Left"))return SideIdx::LEFT;
  if(!strcmp(r,"Right"))return SideIdx::RIGHT;
  if(!strcmp(r,"Forward"))return SideIdx::FORWARD;
  if(!strcmp(r,"Bottom-Left"))return SideIdx::BOTTOM_LEFT;
  return SideIdx::UNKNOWN;
}

constexpr float STEP_TOLERANCE_M = 0.05f;
constexpr uint32_t PRINT_PERIOD_MS = 100;  // 10Hz update rate (0.1s)
constexpr uint32_t TOF_READ_PERIOD_MS = 100;  // Read ToF sensors every 0.1s
enum class Mode{ NAVIGATE, HOVER };
Mode g_mode = Mode::NAVIGATE;
bool g_hover_announced=false;

// Position tracking (optical flow enabled)
float g_pos_x=0, g_pos_y=0;
float vx_filt=0, vy_filt=0;
int   last_height_mm = 200;  // Will use bottom-left sensor
float g_height_m = 0.2f;

uint32_t g_last_print_ms=0, g_last_loop_ms=0, g_last_tof_read_ms=0;

// Waypoint queue
static const int MAX_STEPS=32;
Step q[MAX_STEPS]; int qi=0,qj=0;
float step_origin_x=0, step_origin_y=0;
bool  has_active=false; Step active{0,0};

bool qEmpty(){ return qi==qj; }
bool qFull(){ return ((qj+1)%MAX_STEPS)==qi; }
bool queue_push(const Step& s){ if(qFull()) return false; q[qj]=s; qj=(qj+1)%MAX_STEPS; return true; }
bool queue_pop(Step* s){ if(qEmpty()) return false; *s=q[qi]; qi=(qi+1)%MAX_STEPS; return true; }

const char* directionFor(const Step& s){
  if (fabs(s.dx_m)>=fabs(s.dy_m)) return (s.dx_m>0)?"Navigate Forward":(s.dx_m<0)?"Navigate Back":"Navigate Hold";
  return (s.dy_m>0)?"Navigate Right":(s.dy_m<0)?"Navigate Left":"Navigate Hold";
}
bool stepDone(const Step& s){
  float tx=step_origin_x + s.dx_m, ty=step_origin_y + s.dy_m;
  float ex=tx-g_pos_x, ey=ty-g_pos_y;
  return sqrtf(ex*ex+ey*ey) <= STEP_TOLERANCE_M;
}

// ===================== VL53 BRING-UP =====================
void powerDownAll(){ 
  for(size_t i=0;i<NUM_SENSORS;i++){ 
    pinMode(g_cfgs[i].xshut_pin,OUTPUT); 
    digitalWrite(g_cfgs[i].xshut_pin,LOW);
  } 
  delay(10); 
}

bool bringUpOne(size_t i){
  Serial.printf("  Powering sensor %s (XSHUT pin %d)...\n", g_cfgs[i].role, g_cfgs[i].xshut_pin);
  
  // Set pin mode and power up
  pinMode(g_cfgs[i].xshut_pin, OUTPUT);
  digitalWrite(g_cfgs[i].xshut_pin, HIGH); 
  delay(10);  // Give it time to boot
  
  // Verify pin is actually HIGH
  if(digitalRead(g_cfgs[i].xshut_pin) != HIGH) {
    Serial.printf("  !! WARNING: Pin %d not reading HIGH after setting!\n", g_cfgs[i].xshut_pin);
  }
  
  VL53L1X& s = g_sensors[i];
  s.setBus(&Wire);
  s.setTimeout(1000);  // 1 second timeout for reliability
  
  // Try init with retries
  bool init_ok = false;
  for(int retry = 0; retry < 3; retry++) {
    if(s.init()) {
      init_ok = true;
      break;
    }
    Serial.printf("  Retry %d...\n", retry + 1);
    delay(50);
  }
  
  if(!init_ok) { 
    Serial.printf("  !! init() failed for %s after 3 retries\n", g_cfgs[i].role);
    Serial.printf("  !! Check: 1) Sensor wiring 2) Pin %d conflicts 3) Sensor power\n", g_cfgs[i].xshut_pin);
    
    // Check if we can see it on default address
    Wire.beginTransmission(0x29);
    if(Wire.endTransmission() == 0) {
      Serial.println("  !! Sensor visible at default 0x29 - XSHUT may not be working!");
    }
    
    return false;
  }
  
  s.setAddress(g_cfgs[i].addr);
  
  // Use faster Short mode for obstacle sensors, Long mode for height sensors
  if(g_cfgs[i].is_bottom) {
    s.setDistanceMode(VL53L1X::Long);  // Long range for height
    Serial.printf("  Distance mode: Long (for height sensing)\n");
  } else {
    s.setDistanceMode(VL53L1X::Short);  // Short range = faster response for obstacles
    Serial.printf("  Distance mode: Short (for fast obstacle detection)\n");
  }
  
  s.setMeasurementTimingBudget(VL_TIMING_BUDGET_MS*1000UL);
  s.startContinuous(VL_INTER_MEAS_MS);
  
  // Give sensor time to take first measurement
  delay(VL_TIMING_BUDGET_MS + 10);
  
  Serial.printf("  OK: %s @ 0x%02X\n", g_cfgs[i].role, g_cfgs[i].addr);
  return true;
}

bool setupAllVL(){
  Serial.println("  Holding all XSHUT LOW (reset)...");
  powerDownAll();
  delay(20);
  
  bool ok=true;
  for(size_t i=0;i<NUM_SENSORS;i++){
    if(!bringUpOne(i)){ 
      ok=false; 
    }
    delay(20);  // Delay between sensors
  }
  
  // I2C scan for debugging on Wire (Bus 0)
  Serial.println("  I2C Bus 0 (Wire) scan after VL53L1X init:");
  for(uint8_t addr = 1; addr < 127; addr++){
    Wire.beginTransmission(addr);
    if(Wire.endTransmission() == 0){
      Serial.printf("    Found device at 0x%02X\n", addr);
    }
  }
  
  return ok;
}

Ranges readAllVL(){
  Ranges r{0xFFFF,0xFFFF,0xFFFF,0xFFFF};  // 4 sensors now
  
  for(size_t i=0;i<NUM_SENSORS;i++){
    VL53L1X& s = g_sensors[i];
    
    // Blocking read with timeout
    uint16_t mm = s.read(true);  // true = blocking
    
    if (s.timeoutOccurred()) {
      mm = 0xFFFF;  // Mark as invalid
    }
    
    const char* role=g_cfgs[i].role;
    SideIdx idx = sideIndexFor(role);
    
    // Apply buffering with error rejection
    if(mm != 0xFFFF) {
      bool accepted = false;
      
      switch(idx){
        case SideIdx::LEFT:
          accepted = buf_tof_left.update(mm);
          if(accepted || buf_tof_left.hasValue()) {
            r.left = buf_tof_left.getValue();
          }
          break;
          
        case SideIdx::RIGHT:
          accepted = buf_tof_right.update(mm);
          if(accepted || buf_tof_right.hasValue()) {
            r.right = buf_tof_right.getValue();
          }
          break;
          
        case SideIdx::FORWARD:
          accepted = buf_tof_forward.update(mm);
          if(accepted || buf_tof_forward.hasValue()) {
            r.fwd = buf_tof_forward.getValue();
          }
          break;
          
        case SideIdx::BOTTOM_LEFT:
          accepted = buf_tof_bottom_left.update(mm);
          if(accepted || buf_tof_bottom_left.hasValue()) {
            r.bottom_left = buf_tof_bottom_left.getValue();
            // Use buffered bottom-left height for odometry
            uint16_t filtered_mm = r.bottom_left;
            if(filtered_mm != 0xFFFF && filtered_mm > 0) {
              last_height_mm = filtered_mm;
              g_height_m = filtered_mm * 1e-3f;
            }
          }
          break;
          
        default: break;
      }
    }
  }
  return r;
}

// ===================== I2C COMMUNICATION =====================
bool i2cWrite8(TwoWire* wire, uint8_t addr, uint8_t reg, uint8_t data) { 
  wire->beginTransmission(addr); 
  wire->write(reg); 
  wire->write(data); 
  return wire->endTransmission() == 0;
}

bool i2cRead(TwoWire* wire, uint8_t addr, uint8_t reg, uint8_t* buf, size_t len) {
  wire->beginTransmission(addr); 
  wire->write(reg);
  if(wire->endTransmission(false) != 0) return false;
  if(wire->requestFrom((int)addr, (int)len) != (int)len) return false;
  for(size_t i = 0; i < len; i++) buf[i] = wire->read(); 
  return true;
}

// ===================== MPU6500 FUNCTIONS =====================
bool mpuInit() {
  Serial.println("  [IMU] Initializing MPU6500 on Wire1 (separate I2C bus)...");
  
  // Try both possible addresses (0x68 if AD0=LOW, 0x69 if AD0=HIGH)
  uint8_t addresses[] = {0x68, 0x69};
  bool found = false;
  uint8_t working_addr = 0;
  
  Serial.println("  [IMU] Scanning for MPU6500 on Wire1...");
  for(int i = 0; i < 2; i++) {
    Wire1.beginTransmission(addresses[i]);
    if(Wire1.endTransmission() == 0) {
      Serial.printf("  [IMU] Device found at 0x%02X on Wire1\n", addresses[i]);
      working_addr = addresses[i];
      found = true;
      break;
    }
  }
  
  if(!found) {
    Serial.println("  [IMU] ERROR: No device found at 0x68 or 0x69 on Wire1");
    Serial.println("  [IMU] Check wiring:");
    Serial.printf("    - SDA -> Pin %d\n", PIN_SDA_IMU);
    Serial.printf("    - SCL -> Pin %d\n", PIN_SCL_IMU);
    Serial.println("    - VCC -> 3.3V");
    Serial.println("    - GND -> GND");
    Serial.println("    - AD0/SAO -> GND (for 0x68) or 3.3V (for 0x69)");
    Serial.println("    - CSB -> 3.3V (to enable I2C mode)");
    return false;
  }
  
  // Use the found address
  mpu_address = working_addr;
  Serial.printf("  [IMU] Using address 0x%02X on Wire1\n", mpu_address);
  
  uint8_t whoami = 0;
  if(!i2cRead(&Wire1, mpu_address, REG_WHO_AM_I, &whoami, 1)) {
    Serial.printf("  [IMU] Failed to read WHO_AM_I from 0x%02X\n", mpu_address);
    return false;
  }
  Serial.printf("  [IMU] WHO_AM_I = 0x%02X (expect 0x70 for MPU6500, 0x71 for MPU9250)\n", whoami);
  
  if(whoami != 0x70 && whoami != 0x71) {
    Serial.printf("  [IMU] WARNING: Unexpected WHO_AM_I value. Continuing anyway...\n");
  }

  // Reset and wake
  Serial.println("  [IMU] Resetting device...");
  i2cWrite8(&Wire1, mpu_address, REG_PWR_MGMT_1, 0x80);
  delay(100);
  
  Serial.println("  [IMU] Waking device...");
  i2cWrite8(&Wire1, mpu_address, REG_PWR_MGMT_1, 0x00);
  delay(50);

  // Sample rate ~100 Hz
  Serial.println("  [IMU] Configuring sample rate...");
  i2cWrite8(&Wire1, mpu_address, REG_SMPLRT_DIV, 9);
  
  // DLPFs ~41 Hz on gyro and accel
  Serial.println("  [IMU] Configuring filters and ranges...");
  i2cWrite8(&Wire1, mpu_address, REG_CONFIG, 0x03);
  i2cWrite8(&Wire1, mpu_address, REG_GYRO_CONFIG, 0x00);  // ±250 dps
  i2cWrite8(&Wire1, mpu_address, REG_ACCEL_CONFIG, 0x00); // ±2g
  i2cWrite8(&Wire1, mpu_address, REG_ACCEL_CONFIG2, 0x03);

  // Verify configuration by reading back
  uint8_t test_read = 0;
  if(i2cRead(&Wire1, mpu_address, REG_PWR_MGMT_1, &test_read, 1)) {
    Serial.printf("  [IMU] PWR_MGMT_1 = 0x%02X (should be 0x00)\n", test_read);
  }

  Serial.println("  [IMU] MPU6500 initialized successfully on Wire1");
  return true;
}

void calibrateGyro() {
  Serial.println("  [IMU] Calibrating gyro (200 samples, keep still)...");
  long sx = 0, sy = 0, sz = 0;
  const int samples = 200;
  for(int i = 0; i < samples; i++) { 
    uint8_t buf[14];
    if(i2cRead(&Wire1, mpu_address, REG_ACCEL_XOUT_H, buf, 14)) {
      int16_t gx = (int16_t)((buf[8] << 8) | buf[9]);
      int16_t gy = (int16_t)((buf[10] << 8) | buf[11]);
      int16_t gz = (int16_t)((buf[12] << 8) | buf[13]);
      sx += gx; sy += gy; sz += gz;
    }
    delay(5);
  }
  gbx = (sx / (float)samples) / GYRO_SENS; 
  gby = (sy / (float)samples) / GYRO_SENS; 
  gbz = (sz / (float)samples) / GYRO_SENS;
  Serial.printf("  [IMU] Gyro biases: X=%.3f Y=%.3f Z=%.3f dps\n", gbx, gby, gbz);
}

void imuStep() {
  uint8_t buf[14]; 
  if(!i2cRead(&Wire1, mpu_address, REG_ACCEL_XOUT_H, buf, 14)) return;

  // Parse raw
  int16_t axr = (int16_t)((buf[0] << 8) | buf[1]);
  int16_t ayr = (int16_t)((buf[2] << 8) | buf[3]);
  int16_t azr = (int16_t)((buf[4] << 8) | buf[5]);
  int16_t gxr = (int16_t)((buf[8] << 8) | buf[9]);
  int16_t gyr = (int16_t)((buf[10] << 8) | buf[11]);
  int16_t gzr = (int16_t)((buf[12] << 8) | buf[13]);

  // Convert to physical units
  float ax = axr / ACCEL_SENS;
  float ay = ayr / ACCEL_SENS;
  float az = azr / ACCEL_SENS;
  float gx = (gxr / GYRO_SENS) - gbx;
  float gy = (gyr / GYRO_SENS) - gby;
  float gz = (gzr / GYRO_SENS) - gbz;
  
  // Cross-validation: Use optical flow to validate roll-axis sensors
  // If optical flow shows no Y motion but gyro shows roll change, it's likely jitter
  float opflow_y_mag = fabsf(vx_filt);  // vx_filt is optical flow Y-axis in our coordinate system
  bool opflow_shows_motion = (opflow_y_mag > OPFLOW_Y_MOTION_THRESHOLD);
  
  // If no optical flow motion detected, reject large gyro X changes (roll rate)
  if(!opflow_shows_motion && fabsf(gx) > GYRO_X_REJECT_THRESHOLD) {
    static uint32_t last_reject_msg = 0;
    if(millis() - last_reject_msg > 2000) {
      Serial.printf("[IMU] Rejecting jittery roll: gx=%.2f dps, opflow_y=%.1f mm/s\n", gx, opflow_y_mag);
      last_reject_msg = millis();
    }
    gx = 0;  // Zero out the jittery gyro reading
  }
  
  // Apply buffering with error rejection
  if(buf_accel_x.update(ax)) ax_g = buf_accel_x.getValue();
  else if(buf_accel_x.hasValue()) ax_g = buf_accel_x.getValue();
  else ax_g = ax;
  
  // For accel Y (affects roll calculation), also apply optical flow validation
  if(!opflow_shows_motion) {
    // Calculate what roll change this accel would cause
    float potential_roll = atan2f(ay, az) * 57.2957795f;
    float roll_change = fabsf(potential_roll - roll_f);
    
    // If roll would change significantly but no optical flow motion, be more strict
    if(roll_change > 5.0f) {
      // Don't update buffer, use last good value
      if(buf_accel_y.hasValue()) ay_g = buf_accel_y.getValue();
      else ay_g = ay;
    } else {
      // Normal buffering
      if(buf_accel_y.update(ay)) ay_g = buf_accel_y.getValue();
      else if(buf_accel_y.hasValue()) ay_g = buf_accel_y.getValue();
      else ay_g = ay;
    }
  } else {
    // Optical flow shows motion, trust the IMU
    if(buf_accel_y.update(ay)) ay_g = buf_accel_y.getValue();
    else if(buf_accel_y.hasValue()) ay_g = buf_accel_y.getValue();
    else ay_g = ay;
  }
  
  if(buf_accel_z.update(az)) az_g = buf_accel_z.getValue();
  else if(buf_accel_z.hasValue()) az_g = buf_accel_z.getValue();
  else az_g = az;
  
  // Use filtered accelerometer values
  ax = ax_g;
  ay = ay_g;
  az = az_g;

  // Calculate time step
  uint32_t nowMicros = micros();
  float dt = (nowMicros - lastIMUMicros) * 1e-6f; 
  lastIMUMicros = nowMicros;
  if(dt <= 0 || dt > 0.05f) dt = 0.01f;

  // Check accel validity
  float accel_mag = sqrtf(ax*ax + ay*ay + az*az);
  bool accel_ok = (accel_mag > 0.8f && accel_mag < 1.2f);

  // Attitude from accelerometer
  float rollAcc  = atan2f(ay, az) * 57.2957795f;
  float pitchAcc = atan2f(-ax, sqrtf(ay*ay + az*az)) * 57.2957795f;

  // Gyro integrate
  float rollGy  = roll_f  + gx * dt;
  float pitchGy = pitch_f + gy * dt;

  // Complementary filter
  if (accel_ok) {
    roll_f  = alphaTilt * rollGy  + (1.0f - alphaTilt) * rollAcc;
    pitch_f = alphaTilt * pitchGy + (1.0f - alphaTilt) * pitchAcc;
  } else {
    roll_f  = rollGy;
    pitch_f = pitchGy;
  }

  // Gyro deadzone
  if(fabsf(gx) < 0.05f) gx = 0; 
  if(fabsf(gy) < 0.05f) gy = 0; 
  if(fabsf(gz) < 0.05f) gz = 0;

  p_rate_dps = gx; 
  q_rate_dps = gy; 
  r_rate_dps = gz;

  // Yaw integration
  float phi   = roll_f * 0.01745329252f;
  float theta = pitch_f * 0.01745329252f;
  float cosTheta = cosf(theta);
  float yawRate_rad = (fabsf(cosTheta) > 0.1f) ? 
    ((gy * 0.01745329252f) * sinf(phi) + (gz * 0.01745329252f) * cosf(phi)) / cosTheta : 
    gz * 0.01745329252f;
  yaw_f += yawRate_rad * 57.2957795f * dt;

  // Normalize yaw
  while(yaw_f > 180.0f) yaw_f -= 360.0f;
  while(yaw_f < -180.0f) yaw_f += 360.0f;
}

// ===================== FLOW ODOMETRY (ENABLED) =====================
bool readFlowCounts(int16_t& dx, int16_t& dy){
  g_flow.readMotionCount(&dx,&dy);
  return true;
}

void updateOdom(){
  static uint32_t t_flow = 0;
  uint32_t now = millis();
  while ((int32_t)(now - t_flow) >= 0) {
    int16_t dx=0, dy=0;
    if (readFlowCounts(dx,dy)){
      if (abs(dx) <= DEAD) dx = 0;
      if (abs(dy) <= DEAD) dy = 0;

      const float dt = FLOW_DT_MS * 1e-3f;
      const float dps_x = dx / dt;  // counts/s
      const float dps_y = dy / dt;
      const float thdot_x = dps_x * K_pix_to_rad_x;  // rad/s
      const float thdot_y = dps_y * K_pix_to_rad_y;

      // Use bottom-left height for odometry (clamped)
      const float h_m = max(H_MIN, min(H_MAX, last_height_mm)) * 1e-3f;

      float vx =  h_m * thdot_x * 1000.0f;  // mm/s (world +Y from sensor X) - Also used to validate IMU roll
      float vy = -h_m * thdot_y * 1000.0f;  // mm/s (world +X from sensor Y)

      // Apply buffering with error rejection to velocities
      if(buf_vel_x.update(vx)) vx = buf_vel_x.getValue();
      else if(buf_vel_x.hasValue()) vx = buf_vel_x.getValue();
      
      if(buf_vel_y.update(vy)) vy = buf_vel_y.getValue();
      else if(buf_vel_y.hasValue()) vy = buf_vel_y.getValue();

      // LPF to smooth velocity
      if (alpha_v > 0.0f && alpha_v < 1.0f){
        vx_filt = alpha_v*vx + (1.0f-alpha_v)*vx_filt;
        vy_filt = alpha_v*vy + (1.0f-alpha_v)*vy_filt;
      } else {
        vx_filt = vx; 
        vy_filt = vy;
      }

      // Integrate pose (meters)
      g_pos_y += (vx_filt * dt) / 1000.0f;  // mm/s -> m
      g_pos_x += (vy_filt * dt) / 1000.0f;  // mm/s -> m
    }
    t_flow += FLOW_DT_MS;
  }
}

// ===================== SERIAL COMMANDS =====================
String inbuf;
void tryParseAndQueue(const String& line){
  int start=0;
  while(start < line.length()){
    int comma = line.indexOf(',', start);
    int end = (comma==-1)? line.length() : comma;
    String tok = line.substring(start,end); tok.trim(); start = (comma==-1)? line.length() : comma+1;
    if (tok.length()==0) continue;
    String s=tok; s.toLowerCase();
    if (s.startsWith("move")){
      s.remove(0,4); s.trim(); s.replace("m","");
      while (s.indexOf("  ")!=-1) s.replace("  "," ");
      int sp = s.indexOf(' ');
      if (sp<0){ Serial.println("Parse error: use 'move <dx> <dy>'"); continue; }
      float dx = s.substring(0,sp).toFloat();
      float dy = s.substring(sp+1).toFloat();
      if (!queue_push({dx,dy})) {
        Serial.println("Queue full, step ignored");
        ws.textAll("ERR Queue full");
      } else {
        char buf[64];
        snprintf(buf, sizeof(buf), "WAYPOINT %.2f %.2f", dx, dy);
        ws.textAll(buf);
      }
    } else {
      Serial.println("Unknown token: " + tok);
    }
  }
  if (!has_active && !qEmpty()){
    queue_pop(&active);
    step_origin_x = g_pos_x;
    step_origin_y = g_pos_y;
    has_active = true;
    Serial.println(directionFor(active));
    ws.textAll(directionFor(active));
  }
}

// ===================== WIFI SETUP =====================
void setupWiFi() {
  Serial.println("\n[WIFI] Starting WiFi setup...");
  
  if(strlen(WIFI_STA_SSID) > 0) {
    Serial.printf("[WIFI] Attempting STA mode: SSID=%s\n", WIFI_STA_SSID);
    WiFi.mode(WIFI_STA);
    WiFi.begin(WIFI_STA_SSID, WIFI_STA_PASS);
    uint32_t startTime = millis();
    while(WiFi.status() != WL_CONNECTED && millis() - startTime < 8000) { 
      delay(200);
      Serial.print(".");
    }
    Serial.println();
    
    if(WiFi.status() == WL_CONNECTED) {
      Serial.printf("[WIFI] Connected! IP: %s\n", WiFi.localIP().toString().c_str());
      return;
    }
    Serial.println("[WIFI] STA connection failed, falling back to AP mode");
  }
  
  Serial.printf("[WIFI] Starting AP mode: SSID=%s\n", AP_SSID);
  WiFi.mode(WIFI_AP);
  WiFi.softAP(AP_SSID, AP_PASS);
  delay(100);
  Serial.printf("[WIFI] AP started! IP: %s\n", WiFi.softAPIP().toString().c_str());
}

// ===================== WEBSOCKET HANDLER =====================
void onWsEvent(AsyncWebSocket *srv, AsyncWebSocketClient *client, AwsEventType type, 
               void *arg, uint8_t *data, size_t len) {
  if(type == WS_EVT_CONNECT) {
    Serial.printf("[WS] Client connected: ID=%d\n", client->id());
    client->text("[Navigation System Connected]");
  }
  else if(type == WS_EVT_DISCONNECT) {
    Serial.printf("[WS] Client disconnected: ID=%d\n", client->id());
  }
  else if(type == WS_EVT_DATA) {
    AwsFrameInfo *info = (AwsFrameInfo*)arg;
    if(info->final && info->index == 0 && info->len == len && info->opcode == WS_TEXT) {
      data[len] = 0;
      String msg = String((char*)data);
      msg.trim();
      
      if(msg.startsWith("WAYPOINT:")){
        msg.remove(0, 9);
        tryParseAndQueue(msg);
      }
    }
  }
}

// ===================== WEB SERVER SETUP =====================
void setupWebServer() {
  Serial.println("[WEB] Setting up web server...");
  
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *req) { 
    Serial.println("[WEB] Serving index page");
    req->send_P(200, "text/html", INDEX_HTML); 
  });
  
  ws.onEvent(onWsEvent);
  server.addHandler(&ws);
  
  server.begin();
  Serial.println("[WEB] Web server started successfully");
}

// ===================== SETUP =====================
void setup(){
  Serial.begin(115200); 
  delay(2000);
  
  Serial.println("\n========================================");
  Serial.println("ESP32 Navigation System with Web UI");
  Serial.println("========================================\n");
  
  Serial.println("[INIT] Initializing Dual I2C Buses...");
  Serial.printf("  Wire  (Bus 0): SDA=%d, SCL=%d (VL53L1X sensors)\n", PIN_SDA_VL, PIN_SCL_VL);
  Serial.printf("  Wire1 (Bus 1): SDA=%d, SCL=%d (MPU6500 IMU)\n", PIN_SDA_IMU, PIN_SCL_IMU);
  
  Wire.begin(PIN_SDA_VL, PIN_SCL_VL);
  Wire.setClock(100000);  // 100kHz for stability
  delay(50);
  
  Wire1.begin(PIN_SDA_IMU, PIN_SCL_IMU);
  Wire1.setClock(400000);  // 400kHz for IMU (faster)
  delay(50);
  
  Serial.println("[OK] Both I2C buses initialized");
  
  Serial.println("[INIT] Initializing SPI for optical flow...");
  Serial.printf("  SPI: MISO=%d, MOSI=%d, SCK=%d, CS=%d\n", PIN_MISO, PIN_MOSI, PIN_SCK, PIN_CS);
  SPI.begin(PIN_SCK, PIN_MISO, PIN_MOSI);
  Serial.println("[OK] SPI initialized");

  Serial.println("[INIT] Initializing PMW3901 optical flow sensor...");
  if (!g_flow.begin()) {
    Serial.println("[ERROR] PMW3901 begin FAILED - check wiring!"); 
  } else {
    Serial.println("[OK] PMW3901 optical flow sensor initialized");
  }
  
  Serial.println("[INIT] Initializing VL53L1X sensors...");
  if (!setupAllVL()) {
    Serial.println("[WARNING] One or more VL53L1X failed to init");
  } else {
    Serial.println("[OK] All VL53L1X sensors initialized");
  }

  Serial.println("[INIT] Initializing MPU6500 IMU...");
  
  // Scan Wire1 bus for IMU
  Serial.println("  [IMU] I2C Bus 1 (Wire1) scan:");
  bool found_any = false;
  for(uint8_t addr = 1; addr < 127; addr++){
    Wire1.beginTransmission(addr);
    if(Wire1.endTransmission() == 0){
      Serial.printf("    Found device at 0x%02X", addr);
      if(addr == 0x68) Serial.print(" <- MPU6500 (if AD0=LOW)");
      if(addr == 0x69) Serial.print(" <- MPU6500 (if AD0=HIGH)");
      Serial.println();
      found_any = true;
    }
  }
  if(!found_any) {
    Serial.println("    WARNING: No I2C devices found on Wire1!");
  }
  
  imu_ok = mpuInit();
  if(imu_ok) {
    calibrateGyro();
    
    // Initialize attitude from accelerometer
    uint8_t buf[14];
    if(i2cRead(&Wire1, mpu_address, REG_ACCEL_XOUT_H, buf, 14)) {
      int16_t ax = (int16_t)((buf[0] << 8) | buf[1]);
      int16_t ay = (int16_t)((buf[2] << 8) | buf[3]);
      int16_t az = (int16_t)((buf[4] << 8) | buf[5]);
      float axg = ax / ACCEL_SENS;
      float ayg = ay / ACCEL_SENS; 
      float azg = az / ACCEL_SENS;
      roll_f = atan2f(ayg, azg) * 57.2957795f;
      pitch_f = atan2f(-axg, sqrtf(ayg*ayg + azg*azg)) * 57.2957795f;
      yaw_f = 0.0f;
      Serial.printf("  [IMU] Initial attitude: R=%.1f P=%.1f Y=%.1f\n", roll_f, pitch_f, yaw_f);
    }
    lastIMUMicros = micros();
    Serial.println("[OK] MPU6500 initialized and calibrated");
  } else {
    Serial.println("[WARNING] MPU6500 initialization failed");
  }

  setupWiFi();
  setupWebServer();

  g_last_print_ms = millis(); 
  g_last_loop_ms = millis();
  g_last_tof_read_ms = millis();
  
  Serial.println("\n========================================");
  Serial.println("System Ready!");
  Serial.println("Update Rates:");
  Serial.println("  - ToF Sensors: 10 Hz (0.1s)");
  Serial.println("  - Optical Flow: 100 Hz");
  Serial.println("  - IMU: ~250 Hz");
  Serial.println("  - Serial/WebSocket: 10 Hz (0.1s)");
  Serial.println("\nError Rejection (5-item buffer):");
  Serial.printf("  ToF: %dmm - %dmm, max change: %dmm\n", 
    TOF_MIN_VALID, TOF_MAX_VALID, TOF_MAX_CHANGE);
  Serial.printf("  Accel: %.1fg - %.1fg, max change: %.1fg\n", 
    ACCEL_MIN_VALID, ACCEL_MAX_VALID, ACCEL_MAX_CHANGE);
  Serial.printf("  Gyro: %.0fdps - %.0fdps, max change: %.0fdps\n", 
    GYRO_MIN_VALID, GYRO_MAX_VALID, GYRO_MAX_CHANGE);
  Serial.printf("  Velocity: %.0fmm/s - %.0fmm/s, max change: %.0fmm/s\n", 
    VELOCITY_MIN_VALID, VELOCITY_MAX_VALID, VELOCITY_MAX_CHANGE);
  Serial.println("Buffer auto-reset: After 10 consecutive rejections");
  Serial.println("\nCross-Validation:");
  Serial.printf("  Roll axis: Validated by optical flow Y-axis\n");
  Serial.printf("  Rejects IMU roll jitter when opflow Y < %.0f mm/s\n", OPFLOW_Y_MOTION_THRESHOLD);
  Serial.println("========================================\n");
}

// ===================== LOOP =====================
void loop(){
  ws.cleanupClients();
  uint32_t now = millis();
  
  // Heartbeat every 10 seconds with buffer status
  static uint32_t last_heartbeat = 0;
  if (now - last_heartbeat >= 10000) {
    Serial.println("[Heartbeat] System running...");
    Serial.printf("  Buffer status - ToF: L:%d R:%d F:%d BL:%d | Accel: %d | Gyro: %d | Vel: %d\n",
      buf_tof_left.getCount(), buf_tof_right.getCount(), 
      buf_tof_forward.getCount(), buf_tof_bottom_left.getCount(),
      buf_accel_x.getCount(), buf_gyro_x.getCount(), buf_vel_x.getCount());
    Serial.printf("  Rejection count - ToF: L:%d R:%d F:%d BL:%d\n",
      buf_tof_left.getRejectionCount(), buf_tof_right.getRejectionCount(),
      buf_tof_forward.getRejectionCount(), buf_tof_bottom_left.getRejectionCount());
    last_heartbeat = now;
  }
  
  // Serial intake
  while(Serial.available()){
    char c=(char)Serial.read();
    if (c=='\n' || c=='\r'){ 
      if (inbuf.length()>0){ 
        tryParseAndQueue(inbuf); 
        inbuf=""; 
      } 
    } else { 
      inbuf += c; 
      if (inbuf.length()>300) inbuf.remove(0,100); 
    }
  }

  // Ranging + obstacle detection (throttled to 0.1s)
  static Ranges rng{0xFFFF,0xFFFF,0xFFFF,0xFFFF};  // Keep last reading
  
  if(now - g_last_tof_read_ms >= TOF_READ_PERIOD_MS) {
    g_last_tof_read_ms = now;
    rng = readAllVL();
  }
  
  // IMU update (fast as possible)
  if(imu_ok) {
    imuStep();
  }
  
  // Check for timeout errors
  static uint32_t last_timeout_msg = 0;
  for(size_t i=0; i<NUM_SENSORS; i++){
    if(g_sensors[i].timeoutOccurred() && now - last_timeout_msg >= 1000){
      Serial.printf("[WARNING] Sensor %s timed out\n", g_cfgs[i].role);
      last_timeout_msg = now;
    }
  }
  
  bool obstacle=false;
  auto chk=[&](uint16_t mm, const char* role){ 
    if(mm==0xFFFF) return; 
    uint16_t lim=obstacleThresholdFor(role); 
    if(lim && mm<=lim){ 
      obstacle=true; 
    } 
  };
  chk(rng.left,"Left"); 
  chk(rng.right,"Right"); 
  chk(rng.fwd,"Forward");

  Mode new_mode = obstacle ? Mode::HOVER : Mode::NAVIGATE;
  if (new_mode != g_mode){ 
    g_mode=new_mode; 
    g_hover_announced=false; 
  }
  if (g_mode==Mode::HOVER && !g_hover_announced){ 
    Serial.println("Entering Hover State"); 
    ws.textAll("ALERT Obstacle detected - Hovering");
    g_hover_announced=true; 
  }

  // Odometry update (optical flow enabled)
  updateOdom();

  // Waypoint progression (now fully functional with odometry)
  if (g_mode==Mode::NAVIGATE && has_active){
    if (stepDone(active)){
      Serial.println("Step Complete");
      ws.textAll("INFO Step Complete");
      step_origin_x += active.dx_m;
      step_origin_y += active.dy_m;
      if (queue_pop(&active)){ 
        Serial.println(directionFor(active)); 
        ws.textAll(directionFor(active));
        has_active=true; 
      } else { 
        has_active=false; 
        Serial.println("All Steps Complete"); 
        ws.textAll("INFO All waypoints complete");
      }
    }
  }

  // Web status updates
  if (now - g_last_print_ms >= PRINT_PERIOD_MS){
    g_last_print_ms = now;
    
    // Send data via WebSocket
    static char buf[256];
    
    // Send 4 sensors now (BR removed)
    snprintf(buf, sizeof(buf), "RANGES L:%d R:%d F:%d BL:%d", 
      rng.left==0xFFFF?9999:rng.left,
      rng.right==0xFFFF?9999:rng.right,
      rng.fwd==0xFFFF?9999:rng.fwd,
      rng.bottom_left==0xFFFF?9999:rng.bottom_left
    );
    ws.textAll(buf);
    
    snprintf(buf, sizeof(buf), "POS X:%.3f Y:%.3f H:%.3f VX:%d VY:%d",
      g_pos_x, g_pos_y, g_height_m, (int)vx_filt, (int)vy_filt
    );
    ws.textAll(buf);
    
    snprintf(buf, sizeof(buf), "MODE %s", g_mode==Mode::NAVIGATE?"NAVIGATE":"HOVER");
    ws.textAll(buf);
    
    snprintf(buf, sizeof(buf), "STATUS Active:%s Queue:%d",
      has_active?"Y":"N", (qj-qi+MAX_STEPS)%MAX_STEPS
    );
    ws.textAll(buf);
    
    // Send IMU data
    if(imu_ok) {
      snprintf(buf, sizeof(buf), "IMU R:%.1f P:%.1f Y:%.1f AX:%.2f AY:%.2f AZ:%.2f",
        roll_f, pitch_f, yaw_f, ax_g, ay_g, az_g
      );
      ws.textAll(buf);
    }
    
    // Serial debug with raw sensor data
    Serial.print("Sensors: L=");
    Serial.print(rng.left==0xFFFF ? -1 : rng.left);
    Serial.print("mm R=");
    Serial.print(rng.right==0xFFFF ? -1 : rng.right);
    Serial.print("mm F=");
    Serial.print(rng.fwd==0xFFFF ? -1 : rng.fwd);
    Serial.print("mm BL=");
    Serial.print(rng.bottom_left==0xFFFF ? -1 : rng.bottom_left);
    Serial.print("mm  |  ");
    
    Serial.print("Mode="); 
    Serial.print(g_mode==Mode::NAVIGATE?"NAVIGATE":"HOVER");
    Serial.print("  Pos=("); 
    Serial.print(g_pos_x,3); 
    Serial.print(", "); 
    Serial.print(g_pos_y,3); 
    Serial.print(") m");
    Serial.print("  H=");
    Serial.print(g_height_m,3);
    Serial.print(" m");
    Serial.print("  v(mm/s)=("); 
    Serial.print((long)vx_filt); 
    Serial.print(", "); 
    Serial.print((long)vy_filt); 
    Serial.print(")");
    Serial.print("  Active="); 
    Serial.print(has_active?"Y":"N");
    Serial.print("  Q="); 
    Serial.print((qj-qi+MAX_STEPS)%MAX_STEPS);
    
    // IMU data
    if(imu_ok) {
      Serial.print("  |  IMU: R=");
      Serial.print(roll_f, 1);
      Serial.print("° P=");
      Serial.print(pitch_f, 1);
      Serial.print("° Y=");
      Serial.print(yaw_f, 1);
      Serial.print("°  Accel: X=");
      Serial.print(ax_g, 2);
      Serial.print("g Y=");
      Serial.print(ay_g, 2);
      Serial.print("g Z=");
      Serial.print(az_g, 2);
      Serial.print("g");
    }
    
    Serial.println();
  }
  
  delay(10);  // 10ms delay for stable loop timing
}