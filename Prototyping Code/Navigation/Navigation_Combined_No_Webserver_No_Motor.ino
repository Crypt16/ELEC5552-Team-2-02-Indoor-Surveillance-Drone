/*
  Combined Navigation + Obstacle Detection (v3 - fixed for Arduino prototypes)
  - PMW3901 odometry logic lifted from your working 1.2M code:
      counts -> counts/s -> rad/s via K_pix_to_rad_x/y
      v = h * theta_dot, velocity LPF alpha_v, deadband on counts
  - Height pipeline: Median-of-N -> Slew-Rate Limiter -> EMA
  - Waypoints entered via Serial:  move <dx> <dy>, move <dx> <dy>, ...
  - Obstacle handling: print & enter Hover; do NOT reset queue.
  - Pololu VL53L1X for multi-sensor (XSHUT) + continuous ranging
*/

#include <Wire.h>
#include <SPI.h>
#include <VL53L1X.h>           // Pololu
#include "Bitcraze_PMW3901.h"  // Flow

// ===================== PINS (EDIT) =====================
constexpr int PIN_SDA  = 33;
constexpr int PIN_SCL  = 36;

constexpr int PIN_MISO = 31;
constexpr int PIN_MOSI = 37;
constexpr int PIN_SCK  = 30;
constexpr int PIN_CS   = 29;

// VL53L1X XSHUT map
struct SensorCfg { const char* role; int xshut_pin; uint8_t addr; bool is_height; };
SensorCfg g_cfgs[] = {
  { "Left",    26, 0x30, false },
  { "Right",    16, 0x31, false },
  { "Forward", 27, 0x32, false },
  { "Back",    28, 0x33, false },
  { "Height",  9, 0x34, true  },
};
constexpr size_t NUM_SENSORS = sizeof(g_cfgs)/sizeof(g_cfgs[0]);

// ===================== OBSTACLE & VL53 SETTINGS =====================
constexpr uint16_t THRESH_LEFT   = 500;
constexpr uint16_t THRESH_RIGHT  = 500;
constexpr uint16_t THRESH_FWD    = 500;
constexpr uint16_t THRESH_BACK   = 500;

constexpr uint16_t VL_TIMING_BUDGET_MS = 50;
constexpr uint16_t VL_INTER_MEAS_MS    = 0;

// ===================== PMW3901 ODOMETRY (from working 1.2M) =====================
constexpr uint32_t FLOW_DT_MS = 10;     // 100 Hz
// Gains (copy from your working 1.2M code; edit signs to match orientation)
float K_pix_to_rad_x = 0.00193f;  // rad per count  (sensor X -> world +Y)
float K_pix_to_rad_y = 0.00193f;  // rad per count  (sensor Y -> world -X)
// Deadband on raw counts
constexpr int DEAD = 0;           // set non-zero if needed
// Velocity LPF (same var name)
float alpha_v = 0.30f;            // 0..1, higher = snappier

// Height limits (for sanity / guards if needed)
constexpr int H_MIN = 60;         // mm
constexpr int H_MAX = 4000;       // mm

// ===================== HEIGHT FILTER CHAIN =====================
struct HeightFilter {
  int   N = 4;              // median buffer length
  float max_delta_m = 0.05; // slew limiter per update
  float alpha = 0.20f;      // EMA
  // state
  static const int MAXN = 10;
  float buf[MAXN]; int len=0;
  float ema=0; bool has=false;
  void clear(){ len=0; has=false; }
  float update(float h_m){
    if (N<1) N=1; if (N>MAXN) N=MAXN;
    if (len<N){ buf[len++]=h_m; } else { for(int i=1;i<N;i++) buf[i-1]=buf[i]; buf[N-1]=h_m; }
    // median
    float t[MAXN]; for(int i=0;i<len;i++) t[i]=buf[i];
    for(int i=1;i<len;i++){ float v=t[i]; int j=i-1; while(j>=0 && t[j]>v){ t[j+1]=t[j]; j--; } t[j+1]=v; }
    float med = (len==0)?h_m : ( (len&1)? t[len/2] : 0.5f*(t[len/2-1]+t[len/2]) );
    // slew
    float lim = med;
    if (has){
      float d = med - ema;
      if (d >  max_delta_m) lim = ema + max_delta_m;
      if (d < -max_delta_m) lim = ema - max_delta_m;
    }
    // ema
    if (!has){ ema=lim; has=true; } else { ema = alpha*lim + (1.0f-alpha)*ema; }
    return ema;
  }
};
HeightFilter g_hf;

// ===================== STATE TYPES (define BEFORE any function uses) =====================
struct Ranges{ uint16_t left,right,fwd,back,height; };
struct Step{ float dx_m; float dy_m; };

// ===================== GLOBAL STATE =====================
VL53L1X g_sensors[NUM_SENSORS];
Bitcraze_PMW3901 g_flow(PIN_CS);

enum class SideIdx:int { LEFT=0, RIGHT=1, FORWARD=2, BACK=3, HEIGHT=4, UNKNOWN=-1 };
uint16_t obstacleThresholdFor(const char* r){
  if(!strcmp(r,"Left"))return THRESH_LEFT;
  if(!strcmp(r,"Right"))return THRESH_RIGHT;
  if(!strcmp(r,"Forward"))return THRESH_FWD;
  if(!strcmp(r,"Back"))return THRESH_BACK;
  return 0;
}
SideIdx sideIndexFor(const char* r){
  if(!strcmp(r,"Left"))return SideIdx::LEFT;
  if(!strcmp(r,"Right"))return SideIdx::RIGHT;
  if(!strcmp(r,"Forward"))return SideIdx::FORWARD;
  if(!strcmp(r,"Back"))return SideIdx::BACK;
  if(!strcmp(r,"Height"))return SideIdx::HEIGHT;
  return SideIdx::UNKNOWN;
}

constexpr float STEP_TOLERANCE_M = 0.05f;
constexpr uint32_t PRINT_PERIOD_MS = 100;
enum class Mode{ NAVIGATE, HOVER };
Mode g_mode = Mode::NAVIGATE;
bool g_hover_announced=false;

// Pose (meters) and velocities (mm/s for debug parity with 1.2M)
float g_pos_x=0, g_pos_y=0;
float vx_filt=0, vy_filt=0; // mm/s (LPF as in 1.2M)
int   last_height_mm = 200; // raw-ish mm for odometry
float g_height_m = 0.2f;    // filtered meters for reporting

uint32_t g_last_print_ms=0, g_last_loop_ms=0;

// ===================== WAYPOINT QUEUE (Serial-provided) =====================
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

// ===================== PRINT HELPERS =====================
void printHeightLine(uint16_t mm){ Serial.print("Distance: "); Serial.print(mm); Serial.println("mm"); }
void printObstacleAlert(uint16_t lim, const char* side){
  Serial.print("Obstacle Within "); Serial.print(lim); Serial.print("mm "); Serial.println(side);
}

// ===================== VL53 BRING-UP =====================
void powerDownAll(){ for(size_t i=0;i<NUM_SENSORS;i++){ pinMode(g_cfgs[i].xshut_pin,OUTPUT); digitalWrite(g_cfgs[i].xshut_pin,LOW);} delay(5); }
bool bringUpOne(size_t i){
  digitalWrite(g_cfgs[i].xshut_pin,HIGH); delay(5);
  VL53L1X& s = g_sensors[i];
  if(!s.init()){ for(int k=0;k<4 && !s.init();k++) delay(5); }
  if(!s.init()) return false;
  s.setAddress(g_cfgs[i].addr);
  s.setDistanceMode(VL53L1X::Long);
  s.setMeasurementTimingBudget(VL_TIMING_BUDGET_MS*1000UL);
  s.startContinuous(VL_INTER_MEAS_MS);
  return true;
}
bool setupAllVL(){
  powerDownAll();
  bool ok=true;
  for(size_t i=0;i<NUM_SENSORS;i++){
    if(!bringUpOne(i)){ Serial.print(g_cfgs[i].role); Serial.println(": init FAILED"); ok=false; }
    else{ Serial.print(g_cfgs[i].role); Serial.print(": 0x"); Serial.println(g_cfgs[i].addr,HEX); }
  }
  return ok;
}

Ranges readAllVL(){
  Ranges r{0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF};
  for(size_t i=0;i<NUM_SENSORS;i++){
    VL53L1X& s = g_sensors[i];
    if(!s.dataReady()) continue;
    uint16_t mm = s.read();
    if (s.timeoutOccurred()) continue;
    const char* role=g_cfgs[i].role;
    switch(sideIndexFor(role)){
      case SideIdx::LEFT:   r.left=mm;   break;
      case SideIdx::RIGHT:  r.right=mm;  break;
      case SideIdx::FORWARD:r.fwd=mm;    break;
      case SideIdx::BACK:   r.back=mm;   break;
      case SideIdx::HEIGHT: r.height=mm; break;
      default: break;
    }
  }
  // Update height filters + print
  if (r.height != 0xFFFF){
    last_height_mm = r.height; // keep mm for odometry
    g_height_m = g_hf.update(r.height * 1e-3f);
    printHeightLine(r.height);
  }
  return r;
}

// ===================== FLOW ODOMETRY (1.2M logic) =====================
bool readFlowCounts(int16_t& dx, int16_t& dy){
  g_flow.readMotionCount(&dx,&dy);
  return true;
}
void updateOdom(){
  // integrate every FLOW_DT_MS using 1.2M method
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

      const float thdot_x = dps_x * K_pix_to_rad_x; // rad/s
      const float thdot_y = dps_y * K_pix_to_rad_y; // rad/s

      const float h_m = max(H_MIN, min(H_MAX, last_height_mm)) * 1e-3f; // clamp then convert

      float vx =  h_m * thdot_x * 1000.0f; // mm/s (world +Y from sensor X)
      float vy = -h_m * thdot_y * 1000.0f; // mm/s (world +X from sensor Y; note minus)

      // LPF to match 1.2M dynamics
      if (alpha_v > 0.0f && alpha_v < 1.0f){
        vx_filt = alpha_v*vx + (1.0f-alpha_v)*vx_filt;
        vy_filt = alpha_v*vy + (1.0f-alpha_v)*vy_filt;
      } else {
        vx_filt = vx; vy_filt = vy;
      }

      // Integrate pose (meters)
      g_pos_y += (vx_filt * dt) / 1000.0f; // mm/s -> m
      g_pos_x += (vy_filt * dt) / 1000.0f; // mm/s -> m
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
      if (!queue_push({dx,dy})) Serial.println("Queue full, step ignored");
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
  }
}

// ===================== SETUP / LOOP =====================
void setup(){
  Serial.begin(115200); delay(50);
  Serial.println("\n[ESP32] Nav+Obstacle v3 (1.2M odom + Serial waypoints)");
  Wire.begin(PIN_SDA,PIN_SCL);
  SPI.begin(PIN_SCK,PIN_MISO,PIN_MOSI);

  if (!g_flow.begin()) Serial.println("PMW3901 begin FAILED"); else Serial.println("PMW3901 OK");
  if (!setupAllVL())   Serial.println("One or more VL53L1X failed to init");

  g_last_print_ms = millis(); g_last_loop_ms = millis();
}

void loop(){
  uint32_t now = millis();
  // Serial intake
  while(Serial.available()){
    char c=(char)Serial.read();
    if (c=='\n' || c=='\r'){ if (inbuf.length()>0){ tryParseAndQueue(inbuf); inbuf=""; } }
    else { inbuf += c; if (inbuf.length()>300) inbuf.remove(0,100); }
  }

  // Ranging + height filters + obstacle
  Ranges rng = readAllVL();
  bool obstacle=false;
  auto chk=[&](uint16_t mm, const char* role){ if(mm==0xFFFF) return; uint16_t lim=obstacleThresholdFor(role); if(lim && mm<=lim){ printObstacleAlert(lim, role); obstacle=true; } };
  chk(rng.left,"Left"); chk(rng.right,"Right"); chk(rng.fwd,"Forward"); chk(rng.back,"Back");

  Mode new_mode = obstacle ? Mode::HOVER : Mode::NAVIGATE;
  if (new_mode != g_mode){ g_mode=new_mode; g_hover_announced=false; }
  if (g_mode==Mode::HOVER && !g_hover_announced){ Serial.println("Entering Hover State"); g_hover_announced=true; }

  // Odometry update (always running so pose is live, but step progression only in NAVIGATE)
  updateOdom();

  if (g_mode==Mode::NAVIGATE && has_active){
    if (stepDone(active)){
      Serial.println("Step Complete");
      step_origin_x += active.dx_m;
      step_origin_y += active.dy_m;
      if (queue_pop(&active)){ Serial.println(directionFor(active)); has_active=true; }
      else { has_active=false; Serial.println("All Steps Complete"); }
    }
  }

  if (now - g_last_print_ms >= PRINT_PERIOD_MS){
    g_last_print_ms = now;
    Serial.print("Mode="); Serial.print(g_mode==Mode::NAVIGATE?"NAVIGATE":"HOVER");
    Serial.print("  Pos=("); Serial.print(g_pos_x,3); Serial.print(", "); Serial.print(g_pos_y,3); Serial.print(") m");
    Serial.print("  H="); Serial.print(g_height_m,3); Serial.print(" m");
    Serial.print("  v(mm/s)=("); Serial.print((long)vx_filt); Serial.print(", "); Serial.print((long)vy_filt); Serial.print(")");
    Serial.print("  Active="); Serial.print(has_active?"Y":"N");
    Serial.print("  Q="); Serial.print((qj-qi+MAX_STEPS)%MAX_STEPS);
    Serial.println();
  }
}