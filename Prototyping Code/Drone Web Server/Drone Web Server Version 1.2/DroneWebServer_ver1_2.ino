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

// =============== HTTP (UI/API) on :80 ===============
WebServer server(80);

// =============== MJPEG stream on :81 ===============
WiFiServer mjpegServer(81);
WiFiClient mjpegClient;      // single viewer keeps it responsive
bool       mjpegActive = false;
uint32_t   lastFrameAt = 0;

// =============== Graph settings =====================
volatile uint8_t g_sensPct = 8;   // percent above baseline to mark "active" (1..30)

// ----------------------------------------------------
// UI: dashboard (camera + sine-wave graph) on :80
// ----------------------------------------------------
void handle_root() {
  static const char* DASH_HTML = R"HTML(
<!doctype html><html><head>
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>ESP32-CAM Dashboard</title>
<style>
body{font-family:system-ui,-apple-system,Segoe UI,Roboto,sans-serif;margin:16px;color:#222}
.row{display:flex;flex-wrap:wrap;gap:16px}
.card{border:1px solid #ddd;border-radius:12px;padding:12px;box-shadow:0 1px 4px rgba(0,0,0,.05)}
#cnv{width:100%;max-width:760px;height:280px;display:block}
.pill{display:inline-block;padding:6px 10px;border-radius:999px;background:#eee}
.pill.active{background:#00c85322;border:1px solid #00c853}
button,input[type=range]{cursor:pointer}
.mono{font-family:ui-monospace,Menlo,Consolas,monospace}
</style>
</head><body>
<h2>ESP32-CAM – Camera + Live Clock Graph</h2>
<div class="row">
  <div class="card">
    <div>State: <span id="state" class="pill">—</span></div>
    <div class="mono">Raw: <span id="raw">—</span> | Baseline: <span id="base">—</span> | Threshold: <span id="th">—</span></div>
    <div style="margin-top:8px;display:flex;gap:12px;align-items:center;">
      <div><label>Trigger % <span id="pval">8</span>%</label><br><input id="pct" type="range" min="1" max="30" value="8"></div>
    </div>
    <canvas id="cnv" width="760" height="280"></canvas>
  </div>
  <div class="card">
    <h3 style="margin-top:0;">Live Camera</h3>
    <img id="stream" style="max-width:360px;width:100%;border-radius:8px;border:1px solid #ddd">
    <div style="margin-top:8px;"><a href="/capture" target="_blank">Open snapshot</a></div>
  </div>
</div>
<script>
(() => {
  const cnv=document.getElementById('cnv'), ctx=cnv.getContext('2d');
  const stateEl=document.getElementById('state'), rawEl=document.getElementById('raw'),
        baseEl=document.getElementById('base'), thEl=document.getElementById('th');
  const pct=document.getElementById('pct'), pval=document.getElementById('pval');
  const img=document.getElementById('stream');

  // Point <img> to port 81 MJPEG
  img.src = location.protocol + '//' + location.hostname + ':81/mjpeg';

  const MAX=500, pts=[]; let yMin=0,yMax=120;
  function draw(){
    const W=cnv.width,H=cnv.height; ctx.clearRect(0,0,W,H);
    let mn=Infinity,mx=-Infinity;
    for(const p of pts){ mn=Math.min(mn,p.raw,p.base,p.th); mx=Math.max(mx,p.raw,p.base,p.th); }
    if(mn!==Infinity){ const pad=Math.max(5,(mx-mn)*.2); yMin=Math.max(0,Math.floor(mn-pad)); yMax=Math.ceil(mx+pad); }
    ctx.globalAlpha=.3; ctx.beginPath(); for(let i=0;i<=5;i++){const y=H*i/5;ctx.moveTo(0,y);ctx.lineTo(W,y);} ctx.strokeStyle='#ccc'; ctx.stroke(); ctx.globalAlpha=1;
    const yMap=v=>H-((v-yMin)/Math.max(1,(yMax-yMin)))*H;
    function plot(f,st,w){ ctx.beginPath(); pts.forEach((p,i)=>{const x=(i/Math.max(1,pts.length-1))*W, y=yMap(p[f]); if(i===0)ctx.moveTo(x,y); else ctx.lineTo(x,y);}); ctx.lineWidth=w||1.5; ctx.strokeStyle=st; ctx.stroke(); }
    plot('th','#999',1); plot('base','#555',1.25); plot('raw','#000',1.8);
  }

  const ev=new EventSource('/sse');
  ev.onmessage=e=>{
    const d=JSON.parse(e.data);
    pts.push({raw:d.raw,base:d.base,th:d.th,active:d.active?1:0});
    if(pts.length>MAX) pts.shift();
    rawEl.textContent=d.raw; baseEl.textContent=d.base; thEl.textContent=d.th;
    stateEl.textContent=d.active?'ACTIVE':'idle';
    stateEl.className='pill'+(d.active?' active':'');
    draw();
  };

  pct.oninput=async()=>{ pval.textContent=pct.value; try{ await fetch('/sens?pct='+encodeURIComponent(pct.value)); }catch(e){} };
  pval.textContent=pct.value;
})();
</script>
</body></html>
)HTML";
  server.send(200, "text/html", DASH_HTML);
}

// ----------------------------------------------------
// Single still capture on :80
// ----------------------------------------------------
void handle_capture() {
  camera_fb_t* fb = esp_camera_fb_get();
  if (!fb) { server.send(503, "text/plain", "Camera Busy"); return; }
  server.sendHeader("Content-Type", "image/jpeg");
  server.sendHeader("Content-Length", String(fb->len));
  server.send(200);
  server.client().write(fb->buf, fb->len);
  esp_camera_fb_return(fb);
}

// ----------------------------------------------------
// SSE: live sine-wave data on :80
// ----------------------------------------------------
void handle_sse() {
  WiFiClient client = server.client();
  client.println("HTTP/1.1 200 OK");
  client.println("Content-Type: text/event-stream");
  client.println("Cache-Control: no-cache");
  client.println("Connection: keep-alive");
  client.println();

  float base = 1000.0f; // initial baseline (auto-adjusts)
  while (client.connected()) {
    uint32_t t = millis();
    // 4-second sine wave: raw ~ 400..1600
    float phase = (t % 4000) / 4000.0f;
    float s = sinf(phase * 2.0f * 3.1415926f);
    float raw = 1000.0f + 600.0f * s;

    // Baseline smoothing (track drift, not oscillation)
    float diff  = fabsf(raw - base);
    float alpha = (diff < 4.0f) ? 0.02f : 0.004f;
    base = (1.0f - alpha)*base + alpha*raw;

    // Threshold = base * (1 + sens%)
    float pct = ((float)g_sensPct) * 0.01f;    // 0.01 .. 0.30
    float th  = base * (1.0f + pct);
    bool active = (raw > th);

    client.print("data: {");
    client.printf("\"t\":%lu,", t);
    client.printf("\"raw\":%lu,", (unsigned long)(raw + 0.5f));
    client.printf("\"base\":%lu,", (unsigned long)(base + 0.5f));
    client.printf("\"th\":%lu,", (unsigned long)(th + 0.5f));
    client.printf("\"active\":%s", active ? "true" : "false");
    client.println("}\n");

    delay(100); // ~10 Hz
  }
}

// ----------------------------------------------------
// Adjust sensitivity (Trigger %) on :80
// ----------------------------------------------------
void handle_sens() {
  if (!server.hasArg("pct")) { server.send(400, "text/plain", "pct=1..30"); return; }
  int p = server.arg("pct").toInt();
  if (p < 1) p = 1; if (p > 30) p = 30;
  g_sensPct = (uint8_t)p;
  server.send(200, "text/plain", "OK");
}

// ----------------------------------------------------
// Route table (port 80)
// ----------------------------------------------------
void startHttpServer() {
  server.on("/",        HTTP_GET, handle_root);
  server.on("/capture", HTTP_GET, handle_capture);
  server.on("/sse",     HTTP_GET, handle_sse);
  server.on("/sens",    HTTP_GET, handle_sens);
  server.begin();
}

// ----------------------------------------------------
// MJPEG server (port 81) — non-blocking streamer
// ----------------------------------------------------
static const char* BOUNDARY = "frame";

void startMjpegServer() {
  mjpegServer.begin();
}

// Minimal accept for "GET /mjpeg"
bool acceptMjpegClient(WiFiClient& c) {
  c.setTimeout(2000);
  String req = c.readStringUntil('\n');  // request line
  // Drain headers
  while (c.connected() && c.available()) {
    String h = c.readStringUntil('\n');
    if (h == "\r" || h == "") break;
  }
  if (req.indexOf("GET /mjpeg") >= 0) {
    c.setNoDelay(true);
    c.println("HTTP/1.1 200 OK");
    c.println("Content-Type: multipart/x-mixed-replace; boundary=frame");
    c.println("Cache-Control: no-cache");
    c.println("Pragma: no-cache");
    c.println();
    return true;
  }
  c.println("HTTP/1.1 404 Not Found\r\n\r\n");
  c.stop();
  return false;
}

// Stream one frame per tick
void mjpegTick() {
  if (!mjpegClient || !mjpegClient.connected()) {
    if (mjpegClient) { mjpegClient.stop(); mjpegActive = false; }
    WiFiClient incoming = mjpegServer.available();
    if (incoming) {
      if (!mjpegActive && acceptMjpegClient(incoming)) {
        mjpegClient = incoming;
        mjpegActive = true;
        lastFrameAt = 0;
      } else {
        incoming.println("HTTP/1.1 503 Busy\r\n\r\n");
        incoming.stop();
      }
    }
    return;
  }

  // Pace (~30 fps ceiling)
  uint32_t now = millis();
  if (now - lastFrameAt < 30) return;
  lastFrameAt = now;

  camera_fb_t* fb = esp_camera_fb_get();
  if (!fb) return;

  if (mjpegClient.connected()) {
    mjpegClient.printf("--%s\r\n", BOUNDARY);
    mjpegClient.println("Content-Type: image/jpeg");
    mjpegClient.printf("Content-Length: %u\r\n\r\n", fb->len);
    mjpegClient.write(fb->buf, fb->len);
    mjpegClient.print("\r\n");
  }
  esp_camera_fb_return(fb);

  if (!mjpegClient.connected()) {
    mjpegClient.stop();
    mjpegActive = false;
  }
}

// ================= Flash LED OFF ====================
static inline void flashOff() {
#ifdef LED_GPIO_NUM
  pinMode(LED_GPIO_NUM, OUTPUT);
  digitalWrite(LED_GPIO_NUM, LOW);
#else
  pinMode(4, OUTPUT); // AI-Thinker flash on GPIO4
  digitalWrite(4, LOW);
#endif
}

void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println("\nBooting (AP mode)…");
  flashOff();

  // -------- Camera config (balanced/fast) --------
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
  config.pixel_format = PIXFORMAT_JPEG;
  config.frame_size   = FRAMESIZE_QVGA;   // keep modest for speed
  config.jpeg_quality = 16;               // higher number = lower quality = faster
  config.fb_count     = psramFound() ? 2 : 1;
  config.grab_mode    = psramFound() ? CAMERA_GRAB_LATEST : CAMERA_GRAB_WHEN_EMPTY;
  config.fb_location  = CAMERA_FB_IN_PSRAM;

  if (!psramFound()) {
    Serial.println("PSRAM NOT FOUND; falling back.");
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

  // Optional sensor tweaks
  sensor_t* s = esp_camera_sensor_get();
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
    s->set_framesize(s, FRAMESIZE_QVGA);
    s->set_quality(s, 16);
  }

  // -------- Wi-Fi AP (perf-friendly) --------
  WiFi.mode(WIFI_AP);
  WiFi.setSleep(false);
  WiFi.setTxPower(WIFI_POWER_19_5dBm);
  WiFi.softAP(AP_SSID, AP_PASSWORD, 6, 0, 2);  // ch6, visible, allow up to 2 clients

  IPAddress apIP = WiFi.softAPIP();
  Serial.print("AP SSID: "); Serial.println(AP_SSID);
  Serial.print("AP IP:   "); Serial.println(apIP);
  Serial.println("Open these URLs:");
  Serial.print("  http://"); Serial.println(apIP);                 // dashboard on :80
  Serial.print("  http://"); Serial.print(apIP); Serial.println(":81/mjpeg"); // stream on :81

  // Start servers
  server.onNotFound([](){ server.send(404, "text/plain", "Not found"); });
  startHttpServer();
  mjpegServer.begin();
}

void loop() {
  server.handleClient();  // responsive UI on :80
  mjpegTick();            // video stream on :81 (non-blocking)
}
