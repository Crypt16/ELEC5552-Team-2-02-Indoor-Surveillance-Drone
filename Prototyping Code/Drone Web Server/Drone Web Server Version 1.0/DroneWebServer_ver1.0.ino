#include "esp_camera.h"
#include <WiFi.h>
#include <WebServer.h>

// =================== Camera model ===================
#define CAMERA_MODEL_AI_THINKER
#include "camera_pins.h"

// =================== AP credentials =================
const char* AP_SSID     = "Drone2-02";
const char* AP_PASSWORD = "Group_2_02";   // 8+ chars required

// =============== Minimal HTTP server =================
WebServer server(80);

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

// ---------- Root page ----------
void handle_root() {
  server.send(200, "text/html",
    "<!doctype html><html><body>"
    "<h3>ESP32-CAM (AP mode)</h3>"
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
  config.frame_size   = FRAMESIZE_QVGA;      // QQVGA for max fps; QVGA = balance
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
  // Core exposure/AGC: keep auto on, but less aggressive
  s->set_exposure_ctrl(s, 1);   // AE on
  s->set_aec2(s, 1);            // disable the stronger AE (reduces washout)
  s->set_ae_level(s, -1);       // -2..+2, slight underexposure helps contrast
  s->set_gain_ctrl(s, 1);       // AGC on
  // s->set_agc_gain(s, 8);     // (optional) cap manual gain if you turn AGC off

  // Contrast & color
  s->set_brightness(s, 0);      // was 2; bring back to neutral
  s->set_contrast(s, 1);        // -2..+2
  s->set_saturation(s, 1);      // -2..+2 (a touch more color)

  // White balance
  s->set_whitebal(s, 1);
  s->set_awb_gain(s, 1);
  // s->set_wb_mode(s, 1);      // 0=Auto, 1=Sunny, 2=Cloudy, 3=Office, 4=Home

  // Sharpen/optics helpers
  s->set_lenc(s, 1);            // lens correction ON
  s->set_dcw(s, 1);             // downsize enable (helps at >QVGA)

  // Make sure we’re at the new size even if config was smaller
  s->set_framesize(s, FRAMESIZE_VGA);
  s->set_quality(s, 14);
}


  // --------- AP mode ----------
  WiFi.mode(WIFI_AP);
  WiFi.softAP(AP_SSID, AP_PASSWORD);           // WPA2 AP
  // Optionally fix channel for stability (1/6/11):
  // WiFi.softAP(AP_SSID, AP_PASSWORD, 6);

  IPAddress apIP = WiFi.softAPIP();            // default 192.168.4.1
  Serial.print("AP SSID: "); Serial.println(AP_SSID);
  Serial.print("AP IP:   "); Serial.println(apIP);

  startMinimalServer();
  Serial.println("HTTP server started. Open:");
  Serial.print("  http://"); Serial.println(apIP);
  Serial.println("  http://192.168.4.1/mjpeg   (MJPEG stream)");
  Serial.println("  http://192.168.4.1/capture (single JPEG)");
}

void loop() {
  server.handleClient();
}
