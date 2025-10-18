#include "esp_camera.h"
#include <WiFi.h>
#include <WebServer.h>

// =================== Camera model ===================
#define CAMERA_MODEL_AI_THINKER
#include "camera_pins.h"

// =================== STA (hotspot/router) credentials =================
const char* WIFI_SSID = "Orbi25";
const char* WIFI_PASS = "redpotato035";

// =============== Minimal HTTP server =================
WebServer server(80);

// ---------- helper: safe chunked write to avoid stalls ----------
static bool writeAll(WiFiClient &client, const uint8_t* buf, size_t len, uint32_t timeout_ms = 1000) {
  size_t sent = 0;
  uint32_t stallStart = millis();
  while (sent < len) {
    if (!client.connected()) return false;
    size_t avail = client.availableForWrite();
    if (avail == 0) {
      if (millis() - stallStart > timeout_ms) return false;
      delay(1);
      continue;
    }
    size_t n = min((size_t)1024, min(avail, len - sent));  // 1 KB chunks
    size_t w = client.write(buf + sent, n);
    if (w == 0) {
      if (millis() - stallStart > timeout_ms) return false;
      delay(1);
      continue;
    }
    sent += w;
    stallStart = millis(); // progress made
  }
  return true;
}

// ---------- MJPEG stream handler ----------
static const char* BOUNDARY = "frame";
void handle_mjpeg() {
  WiFiClient client = server.client();
  client.setNoDelay(true);  // reduce Nagle latency

  client.println("HTTP/1.1 200 OK");
  client.println("Content-Type: multipart/x-mixed-replace; boundary=frame");
  client.println("Cache-Control: no-cache, no-store, must-revalidate");
  client.println("Pragma: no-cache");
  client.println("Connection: close");
  client.println();

  char header[160];

  while (client.connected()) {
    camera_fb_t* fb = esp_camera_fb_get();
    if (!fb) break;

    int hdrLen = snprintf(header, sizeof(header),
                          "--%s\r\nContent-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n",
                          BOUNDARY, fb->len);

    bool ok = writeAll(client, (const uint8_t*)header, hdrLen) &&
              writeAll(client, fb->buf, fb->len) &&
              writeAll(client, (const uint8_t*)"\r\n", 2);

    esp_camera_fb_return(fb);

    if (!ok) break;
    // small pacing to avoid buffer bursts; adjust 0–5 ms as needed
    delay(1);
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

// ---------- Root page ----------
void handle_root() {
  server.send(200, "text/html",
    "<!doctype html><html><body>"
    "<h3>ESP32-CAM (STA mode)</h3>"
    "<p><a href=\"/mjpeg\">Open MJPEG stream</a></p>"
    "<p><a href=\"/capture\">Take snapshot</a></p>"
    "</body></html>");
}

void startMinimalServer() {
  server.on("/",        HTTP_GET, handle_root);
  server.on("/mjpeg",   HTTP_GET, handle_mjpeg);
  server.on("/capture", HTTP_GET, handle_capture);
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

void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println("\nBooting (STA mode)…");

  flashOff();

  // -------- Camera config (start modest for smoothness) --------
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
  config.frame_size   = FRAMESIZE_QVGA;      // 320x240 = fluid starter
  config.jpeg_quality = 24;                  // higher number = smaller JPEG
  config.fb_count     = psramFound() ? 2 : 1;
  config.grab_mode    = psramFound() ? CAMERA_GRAB_LATEST : CAMERA_GRAB_WHEN_EMPTY;
  config.fb_location  = CAMERA_FB_IN_PSRAM;

  if (psramFound()) {
    Serial.printf("PSRAM OK: %d bytes\n", ESP.getPsramSize());
  } else {
    Serial.println("PSRAM NOT FOUND; reducing features.");
    config.frame_size   = FRAMESIZE_QVGA;   // keep small if no PSRAM
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
    s->set_exposure_ctrl(s, 1);   // AE on
    s->set_aec2(s, 1);            // balanced AE
    s->set_ae_level(s, -1);
    s->set_gain_ctrl(s, 1);
    s->set_brightness(s, 0);
    s->set_contrast(s, 1);
    s->set_saturation(s, 1);
    s->set_whitebal(s, 1);
    s->set_awb_gain(s, 1);
    s->set_lenc(s, 1);
    s->set_dcw(s, 1);
    s->set_framesize(s, FRAMESIZE_QVGA);  // keep small to start (change to HVGA/VGA later)
    s->set_quality(s, 24);                // smaller JPEGs → smoother
  }

  // --------- STA mode (join hotspot/router) ----------
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);  // important for smooth streaming
  WiFi.begin(WIFI_SSID, WIFI_PASS);

  Serial.print("Connecting to Wi-Fi");
  while (WiFi.status() != WL_CONNECTED) { delay(250); Serial.print("."); }
  Serial.println();
  Serial.print("STA IP: "); Serial.println(WiFi.localIP());

  startMinimalServer();
  Serial.println("HTTP server started. Open:");
  Serial.print("  http://"); Serial.print(WiFi.localIP()); Serial.println("/mjpeg   (MJPEG stream)");
  Serial.print("  http://"); Serial.print(WiFi.localIP()); Serial.println("/capture (single JPEG)");
}

void loop() {
  server.handleClient();
}
