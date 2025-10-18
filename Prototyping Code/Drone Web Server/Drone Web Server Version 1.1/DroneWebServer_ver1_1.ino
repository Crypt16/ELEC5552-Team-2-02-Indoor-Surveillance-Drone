#include "esp_camera.h"
#include <WiFi.h>
#include <WebServer.h>
#include <math.h>

// =================== Camera model ===================
#define CAMERA_MODEL_AI_THINKER
#include "camera_pins.h"

// =================== AP credentials =================
const char* AP_SSID     = "Drone2-02";
const char* AP_PASSWORD = "Group_2_02";   // 8+ chars required

// =============== Minimal HTTP server =================
WebServer server(80);

// =============== Touch config =======================
// Set touch to IO13 (T4). Wire your touch pad to GPIO13 and GND.
#define TOUCH_PAD   T4      // touch channel (T4)  -> GPIO13
#define TOUCH_GPIO  13      // for display/notes

volatile uint16_t g_baseline = 0;   // running baseline (no-touch average)
volatile uint16_t g_margin   = 18;  // how far below baseline counts as "active"

// ---------- MJPEG stream handler ----------
static const char* BOUNDARY = "frame";
void handle_mjpeg() {
  WiFiClient client = server.client();
  client.println("HTTP/1.1 200 OK");
  client.println("Content-Type: multipart/x-mixed-replace; boundary=frame");
  client.println("Cache-Control: no-cache");
  client.println("Pragma: no-cache");
  client.println();

  while (client.connected()) {
    camera_fb_t* fb = esp_camera_fb_get();
    if (!fb) break;

    client.printf("--%s\r\n", BOUNDARY);
    client.println("Content-Type: image/jpeg");
    client.printf("Content-Length: %u\r\n\r\n", fb->len);
    client.write(fb->buf, fb->len);
    client.print("\r\n");

    esp_camera_fb_return(fb);
    delay(5); // small pacing
  }
}

// ---------- Single still capture ----------
void handle_capture() {
  camera_fb_t* fb = esp_camera_fb_get();
  if (!fb) { server.send(503, "text/plain", "Camera Busy"); return; }
  server.sendHeader("Content-Type", "image/jpeg");
  server.sendHeader("Content-Length", String(fb->len));
  server.send(200);
  server.client().write(fb->buf, fb->len);
  esp_camera_fb_return(fb);
}

// ---------- Touch SSE stream ----------
// Sends JSON like: {"t":12345,"raw":57,"base":73,"th":55,"active":true}
void handle_touch_sse() {
  WiFiClient client = server.client();
  client.println("HTTP/1.1 200 OK");
  client.println("Content-Type: text/event-stream");
  client.println("Cache-Control: no-cache");
  client.println("Connection: keep-alive");
  client.println();

  float base = (g_baseline > 0) ? g_baseline : (float)touchRead(TOUCH_PAD);

  while (client.connected()) {
    uint16_t raw = touchRead(TOUCH_PAD);

    // ESP32 touch value drops when touched. Smooth baseline slowly.
    float diff = fabsf((float)raw - base);
    float alpha = (diff < 4.0f) ? 0.01f : 0.003f;
    base = (1.0f - alpha)*base + alpha*raw;

    g_baseline = (uint16_t)(base + 0.5f);
    uint16_t th = (g_baseline > g_margin) ? (g_baseline - g_margin) : 0;
    bool active = (raw < th);

    client.print("data: {");
    client.printf("\"t\":%lu,", (unsigned long)millis());
    client.printf("\"raw\":%u,", raw);
    client.printf("\"base\":%u,", (unsigned)g_baseline);
    client.printf("\"th\":%u,", (unsigned)th);
    client.printf("\"active\":%s", active ? "true" : "false");
    client.println("}\n");

    delay(100); // ~10 Hz
  }
}

// ---------- Recalibrate & Sensitivity ----------
void handle_touch_cal() {
  uint32_t sum = 0; const int N = 200;
  for (int i=0;i<N;i++) { sum += touchRead(TOUCH_PAD); delay(1); }
  g_baseline = (uint16_t)(sum / N);
  server.send(200, "text/plain", "OK");
}

void handle_touch_set() {
  if (!server.hasArg("margin")) { server.send(400, "text/plain", "missing margin"); return; }
  int m = server.arg("margin").toInt();
  if (m < 1) m = 1; if (m > 200) m = 200;
  g_margin = (uint16_t)m;
  server.send(200, "text/plain", "OK");
}

// ---------- Combined dashboard (camera + graph) ----------
void handle_root() {
  static const char* DASH_HTML = R"HTML(
<!doctype html>
<html>
<head>
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>ESP32-CAM Dashboard</title>
<style>
  body { font-family: system-ui,-apple-system,Segoe UI,Roboto,sans-serif; margin:16px; color:#222; }
  .row { display:flex; flex-wrap:wrap; gap:16px; }
  .card { border:1px solid #ddd; border-radius:12px; padding:12px; box-shadow:0 1px 4px rgba(0,0,0,.05); }
  #cnv { width:100%; max-width:760px; height:280px; display:block; }
  .pill { display:inline-block; padding:6px 10px; border-radius:999px; background:#eee; }
  .pill.active { background:#00c85322; border:1px solid #00c853; }
  button,input[type=range] { cursor:pointer; }
  .mono { font-family: ui-monospace, SFMono-Regular, Menlo, Consolas, monospace; }
  label { font-size:12px; color:#555; }
</style>
</head>
<body>
  <h2>ESP32-CAM – Camera + Touch Graph</h2>
  <div class="row">
    <div class="card">
      <div>State: <span id="state" class="pill">—</span></div>
      <div class="mono">Raw: <span id="raw">—</span> | Baseline: <span id="base">—</span> | Threshold: <span id="th">—</span></div>
      <div style="margin-top:8px; display:flex; gap:12px; align-items:center;">
        <button id="recal">Recalibrate</button>
        <div>
          <label>Margin <span id="mval">18</span></label><br>
          <input id="margin" type="range" min="4" max="80" value="18">
        </div>
      </div>
      <canvas id="cnv" width="760" height="280"></canvas>
      <div class="mono" style="margin-top:6px;">Touch pad: <b>T4</b> on <b>GPIO13</b>. Wire a pad/wire to IO13 and GND.</div>
    </div>
    <div class="card">
      <h3 style="margin-top:0;">Live Camera</h3>
      <img id="stream" src="/mjpeg" style="max-width:360px; width:100%; border-radius:8px; border:1px solid #ddd;">
      <div style="margin-top:8px;"><a href="/capture" target="_blank">Open snapshot</a></div>
    </div>
  </div>

<script>
(() => {
  const cnv = document.getElementById('cnv');
  const ctx = cnv.getContext('2d');
  const stateEl = document.getElementById('state');
  const rawEl = document.getElementById('raw');
  const baseEl = document.getElementById('base');
  const thEl = document.getElementById('th');
  const recalBtn = document.getElementById('recal');
  const margin = document.getElementById('margin');
  const mval = document.getElementById('mval');

  const MAX = 500;
  const pts = [];
  let yMin = 0, yMax = 120;

  function draw() {
    const W = cnv.width, H = cnv.height;
    ctx.clearRect(0,0,W,H);

    let minV = Infinity, maxV = -Infinity;
    for (const p of pts) { minV = Math.min(minV, p.raw, p.base, p.th); maxV = Math.max(maxV, p.raw, p.base, p.th); }
    if (minV !== Infinity) {
      const pad = Math.max(5, (maxV - minV) * 0.2);
      yMin = Math.max(0, Math.floor(minV - pad));
      yMax = Math.ceil(maxV + pad);
    }

    ctx.globalAlpha = 0.3;
    ctx.beginPath();
    for (let i=0;i<=5;i++){ const y = H*i/5; ctx.moveTo(0,y); ctx.lineTo(W,y); }
    ctx.strokeStyle = '#ccc'; ctx.stroke();
    ctx.globalAlpha = 1;

    function yMap(v){ return H - ( (v - yMin) / Math.max(1,(yMax - yMin)) ) * H; }
    function plot(field, strokeStyle, lineWidth){
      ctx.beginPath();
      pts.forEach((p,i)=> {
        const x = (i/(Math.max(1,pts.length-1))) * W;
        const y = yMap(p[field]);
        if (i===0) ctx.moveTo(x,y); else ctx.lineTo(x,y);
      });
      ctx.lineWidth = lineWidth||1.5;
      ctx.strokeStyle = strokeStyle;
      ctx.stroke();
    }
    plot('th',   '#999', 1);
    plot('base', '#555', 1.25);
    plot('raw',  '#000', 1.8);
  }

  const ev = new EventSource('/touch-sse');
  ev.onmessage = (e) => {
    const d = JSON.parse(e.data);
    pts.push({raw:d.raw, base:d.base, th:d.th, active:d.active ? 1 : 0});
    if (pts.length > MAX) pts.shift();

    rawEl.textContent  = d.raw;
    baseEl.textContent = d.base;
    thEl.textContent   = d.th;
    stateEl.textContent = d.active ? 'ACTIVE' : 'idle';
    stateEl.className = 'pill' + (d.active ? ' active' : '');

    draw();
  };

  recalBtn.onclick = async () => { try { await fetch('/touch-cal'); } catch(e){} };
  margin.oninput = async () => {
    mval.textContent = margin.value;
    try { await fetch('/touch-set?margin='+encodeURIComponent(margin.value)); } catch(e){}
  };
})();
</script>
</body>
</html>
)HTML";
  server.send(200, "text/html", DASH_HTML);
}

void startMinimalServer() {
  server.on("/",           HTTP_GET, handle_root);       // combined dashboard
  server.on("/mjpeg",      HTTP_GET, handle_mjpeg);
  server.on("/capture",    HTTP_GET, handle_capture);
  server.on("/touch-sse",  HTTP_GET, handle_touch_sse);
  server.on("/touch-cal",  HTTP_GET, handle_touch_cal);
  server.on("/touch-set",  HTTP_GET, handle_touch_set);
  server.begin();
}

// =============== Setup LED flash OFF ===============
static inline void flashOff() {
#ifdef LED_GPIO_NUM
  pinMode(LED_GPIO_NUM, OUTPUT);
  digitalWrite(LED_GPIO_NUM, LOW);
#else
  // AI-Thinker flash LED is GPIO4
  pinMode(4, OUTPUT);
  digitalWrite(4, LOW);
#endif
}

static void calibrateTouchStartup() {
  uint32_t sum = 0; const int N = 200;
  for (int i=0;i<N;i++) { sum += touchRead(TOUCH_PAD); delay(1); }
  g_baseline = sum / N;
}

void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println("\nBooting (AP mode)…");

  flashOff();

  // -------- Camera config (tuned for smooth fps) --------
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer   = LEDC_TIMER_0;
  config.pin_d0       = Y2_GPIO_NUM;
  config.pin_d1       = Y3_GPIO_NUM;
  config.pin_d2       = Y4_GPIO_NUM;
  config.pin_d3       = Y5_GPIO_NUM;
  config.pin_d4       = Y6_GPIO_NUM;
  config.pin_d5       = Y7_GPIO_NUM;
  config.pin_d6       = Y8_GPIO_NUM;
  config.pin_d7       = Y9_GPIO_NUM;
  config.pin_xclk     = XCLK_GPIO_NUM;
  config.pin_pclk     = PCLK_GPIO_NUM;
  config.pin_vsync    = VSYNC_GPIO_NUM;
  config.pin_href     = HREF_GPIO_NUM;
  config.pin_sscb_sda = SIOD_GPIO_NUM;
  config.pin_sscb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn     = PWDN_GPIO_NUM;
  config.pin_reset    = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;      // streaming
  config.frame_size   = FRAMESIZE_QVGA;      // QVGA = balance
  config.jpeg_quality = 16;                  // 18–24 faster; smaller = better quality
  config.fb_count     = psramFound() ? 2 : 1;
  config.grab_mode    = psramFound() ? CAMERA_GRAB_LATEST : CAMERA_GRAB_WHEN_EMPTY;
  config.fb_location  = CAMERA_FB_IN_PSRAM;

  if (psramFound()) {
    Serial.printf("PSRAM OK: %d bytes\n", ESP.getPsramSize());
  } else {
    Serial.println("PSRAM NOT FOUND; reducing features.");
    config.frame_size   = FRAMESIZE_SVGA;
    config.fb_location  = CAMERA_FB_IN_DRAM;
    config.fb_count     = 1;
    config.grab_mode    = CAMERA_GRAB_WHEN_EMPTY;
  }

  Serial.println("Initializing camera…");
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed: 0x%x\n", err);
    while (true) delay(1000);
  }

  // ---- Sensor tweaks (brightness/exposure) ----
  sensor_t * s = esp_camera_sensor_get();
  if (s) {
    s->set_exposure_ctrl(s, 1);
    s->set_aec2(s, 1);
    s->set_ae_level(s, -1);
    s->set_gain_ctrl(s, 1);
    s->set_brightness(s, 0);
    s->set_contrast(s, 1);
    s->set_saturation(s, 1);
    s->set_whitebal(s, 1);
    s->set_awb_gain(s, 1);
    s->set_lenc(s, 1);
    s->set_dcw(s, 1);
    s->set_framesize(s, FRAMESIZE_VGA);
    s->set_quality(s, 14);
  }

  // --------- AP mode ----------
  WiFi.mode(WIFI_AP);
  WiFi.softAP(AP_SSID, AP_PASSWORD);           // WPA2 AP
  IPAddress apIP = WiFi.softAPIP();            // default 192.168.4.1

  Serial.print("AP SSID: "); Serial.println(AP_SSID);
  Serial.print("AP IP:   "); Serial.println(apIP);

  // Touch baseline
  calibrateTouchStartup();

  startMinimalServer();
  Serial.println("HTTP server started. Open:");
  Serial.print("  http://"); Serial.println(apIP);
  Serial.println("  http://192.168.4.1/        (Dashboard: camera + touch graph)");
  Serial.println("  http://192.168.4.1/mjpeg   (MJPEG stream)");
  Serial.println("  http://192.168.4.1/capture (single JPEG)");
}

void loop() {
  server.handleClient();
}
