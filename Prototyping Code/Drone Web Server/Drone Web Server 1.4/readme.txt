ESP32-CAM MJPEG Web Stream (AI Thinker Module)
==============================================

OVERVIEW
--------
This sketch sets up an ESP32-CAM (AI Thinker model) as a Wi-Fi client (STA mode)
and hosts a lightweight HTTP server. The camera serves live MJPEG video and still
JPEG captures over a web browser.

FEATURES
--------
- Streams live MJPEG video at 320x240 resolution.
- Captures single still images (JPEG).
- Simple HTML interface at the root page.
- Handles slow or bursty Wi-Fi safely with chunked writes.
- Automatically disables the onboard flash LED at startup.

HARDWARE
--------
Board:    ESP32-CAM (AI Thinker)
Camera:   OV2640 (default)
LED Flash: GPIO4 (disabled at startup)
Connections:  
  - SDA: GPIO21  
  - SCL: GPIO16  
  - Flash LED: GPIO4  

PSRAM is used if available for smoother streaming.

WIFI SETTINGS
-------------
Edit the Wi-Fi credentials at the top of the sketch:

    const char* WIFI_SSID = "YourWiFiName";
    const char* WIFI_PASS = "YourWiFiPassword";

When connected, the ESP32 prints its local IP address to Serial Monitor.

EXAMPLE SERIAL OUTPUT:
----------------------
    Booting (STA mode)…
    PSRAM OK: 4194304 bytes
    Initializing camera…
    STA IP: 192.168.1.125
    HTTP server started. Open:
      http://192.168.1.125/mjpeg   (MJPEG stream)
      http://192.168.1.125/capture (single JPEG)

ENDPOINTS
---------
1. **/**  
   Displays a minimal HTML page with links to /mjpeg and /capture.

2. **/mjpeg**  
   Streams an MJPEG feed (multipart/x-mixed-replace) for real-time viewing.

3. **/capture**  
   Captures and returns a single JPEG image.

CAMERA CONFIGURATION
--------------------
- Pixel Format: JPEG
- Frame Size:   QVGA (320x240)
- JPEG Quality: 24 (higher = smaller file)
- Frame Buffers: 2 (if PSRAM available)
- Exposure, gain, and color correction enabled for balanced image.

PERFORMANCE NOTES
-----------------
- Streaming uses 1 KB chunks to avoid socket stalls.
- If performance drops, lower frame size or increase JPEG quality value.
- Wi-Fi sleep is disabled for smoother throughput.

TROUBLESHOOTING
---------------
- **Camera init failed:** Ensure correct board selection and 5V stable supply.
- **No image / browser stuck:** Try lower resolution (QVGA), ensure PSRAM is detected.
- **Upload errors:** Hold GPIO0 → GND during upload, release and reset afterward.
- **Brownouts / resets:** Use a proper 5V/1A power source.

SECURITY
--------
No password or HTTPS protection is implemented.
Use only on trusted LANs.

FILES USED
----------
- `esp_camera.h`        : ESP32 camera driver.
- `camera_pins.h`       : Pin definitions for AI Thinker model.
- `app_httpd.cpp`       : Example reference from Espressif (not used here).
- `partitions.csv`      : Default memory layout for ESP32-CAM.

AUTHOR -NC
------
UWA Indoor Drone Project — Demonstration of ESP32-CAM integration.
Implements lightweight HTTP server for live vision streaming.
