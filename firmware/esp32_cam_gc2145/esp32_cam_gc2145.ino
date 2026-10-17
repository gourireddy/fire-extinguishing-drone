#include <Arduino.h>
#include <WiFi.h>
#include <Wire.h>
#include "esp_camera.h"
#include "img_converters.h"
#include "esp_http_server.h"
#include "camera_pins.h"
#include "wifi_credentials.h"

#define PART_BOUNDARY "123456789000000000000987654321"
static const char* _STREAM_CONTENT_TYPE = "multipart/x-mixed-replace;boundary=" PART_BOUNDARY;
static const char* _STREAM_BOUNDARY = "\r\n--" PART_BOUNDARY "\r\n";
static const char* _STREAM_PART = "Content-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n";

httpd_handle_t stream_httpd = NULL;
bool camera_initialized = false;

// HTML page for full-screen video viewing
static const char PROGMEM INDEX_HTML[] = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>ESP32 Fire Drone Camera Stream</title>
  <style>
    body {
      margin: 0;
      padding: 0;
      background: #111;
      color: #fff;
      font-family: -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, sans-serif;
      display: flex;
      flex-direction: column;
      align-items: center;
      justify-content: center;
      min-height: 100vh;
      overflow: hidden;
    }
    .header {
      position: absolute;
      top: 10px;
      left: 15px;
      z-index: 10;
      font-size: 14px;
      color: #aaa;
      background: rgba(0,0,0,0.6);
      padding: 6px 12px;
      border-radius: 6px;
    }
    .container {
      position: relative;
      width: 100vw;
      height: 100vh;
      display: flex;
      align-items: center;
      justify-content: center;
    }
    img#stream {
      width: 100vw;
      height: 100vh;
      object-fit: contain;
      display: block;
      image-rendering: crisp-edges;
    }
    .controls {
      position: absolute;
      bottom: 20px;
      display: flex;
      gap: 12px;
      z-index: 10;
      background: rgba(0,0,0,0.6);
      padding: 8px 16px;
      border-radius: 30px;
      backdrop-filter: blur(4px);
    }
    button {
      background: #007bff;
      color: #fff;
      border: none;
      padding: 8px 16px;
      border-radius: 20px;
      cursor: pointer;
      font-size: 14px;
      font-weight: 500;
      transition: background 0.2s;
    }
    button:hover {
      background: #0056b3;
    }
    .status-badge {
      display: flex;
      align-items: center;
      gap: 6px;
      font-size: 13px;
      color: #4cd964;
    }
    .indicator {
      width: 8px;
      height: 8px;
      border-radius: 50%;
      background: #4cd964;
      animation: pulse 1.5s infinite;
    }
    @keyframes pulse {
      0% { opacity: 0.4; }
      50% { opacity: 1; }
      100% { opacity: 0.4; }
    }
  </style>
</head>
<body>
  <div class="header">
    <strong>Drone Camera Feed</strong> | Sensor: GC2145 (RGB565 &rarr; MJPEG)
  </div>

  <div class="container" id="stream-container">
    <img id="stream" src="/stream" alt="ESP32 Live Stream" />
    <div class="controls">
      <div class="status-badge">
        <span class="indicator"></span> LIVE
      </div>
      <button onclick="toggleFullScreen()">Full Screen</button>
      <button onclick="reloadStream()">Reconnect</button>
    </div>
  </div>

  <script>
    function toggleFullScreen() {
      const container = document.getElementById('stream-container');
      if (!document.fullscreenElement) {
        if (container.requestFullscreen) {
          container.requestFullscreen();
        } else if (container.webkitRequestFullscreen) {
          container.webkitRequestFullscreen();
        }
      } else {
        if (document.exitFullscreen) {
          document.exitFullscreen();
        }
      }
    }

    function reloadStream() {
      const img = document.getElementById('stream');
      img.src = '/stream?t=' + new Date().getTime();
    }
  </script>
</body>
</html>
)rawliteral";

// Web root handler
static esp_err_t index_handler(httpd_req_t *req) {
  httpd_resp_set_type(req, "text/html");
  return httpd_resp_send(req, INDEX_HTML, HTTPD_RESP_USE_STRLEN);
}

// MJPEG stream handler
static esp_err_t stream_handler(httpd_req_t *req) {
  camera_fb_t *fb = NULL;
  esp_err_t res = ESP_OK;
  size_t _jpg_buf_len = 0;
  uint8_t *_jpg_buf = NULL;
  char part_buf[64];

  res = httpd_resp_set_type(req, _STREAM_CONTENT_TYPE);
  if (res != ESP_OK) return res;

  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");

  // Target 10 FPS pacing: 100ms per frame
  const int64_t target_frame_time_us = 100000;

  while (true) {
    int64_t frame_start = esp_timer_get_time();

    fb = esp_camera_fb_get();
    if (!fb) {
      Serial.println("[ERROR] Failed to capture camera frame");
      res = ESP_FAIL;
      break;
    }

    if (fb->format == PIXFORMAT_JPEG) {
      _jpg_buf = fb->buf;
      _jpg_buf_len = fb->len;
    } else {
      // GC2145 outputs RGB565 -> convert to JPEG buffer for HTTP client
      // Quality 65 provides clear imagery while significantly speeding up CPU encoding
      bool converted = frame2jpg(fb, 65, &_jpg_buf, &_jpg_buf_len);
      if (!converted) {
        Serial.println("[ERROR] frame2jpg conversion failed");
        esp_camera_fb_return(fb);
        res = ESP_FAIL;
        break;
      }
    }

    if (res == ESP_OK) {
      res = httpd_resp_send_chunk(req, _STREAM_BOUNDARY, strlen(_STREAM_BOUNDARY));
    }
    if (res == ESP_OK) {
      size_t hlen = snprintf(part_buf, 64, _STREAM_PART, _jpg_buf_len);
      res = httpd_resp_send_chunk(req, part_buf, hlen);
    }
    if (res == ESP_OK) {
      res = httpd_resp_send_chunk(req, (const char *)_jpg_buf, _jpg_buf_len);
    }

    // Free converted JPEG buffer if allocated
    if (fb->format != PIXFORMAT_JPEG && _jpg_buf != NULL) {
      free(_jpg_buf);
      _jpg_buf = NULL;
    }

    esp_camera_fb_return(fb);
    fb = NULL;

    if (res != ESP_OK) {
      break;
    }

    // Maintain consistent 10 FPS pacing to prevent network queue buildup
    int64_t elapsed_us = esp_timer_get_time() - frame_start;
    if (elapsed_us < target_frame_time_us) {
      int delay_ms = (int)((target_frame_time_us - elapsed_us) / 1000);
      vTaskDelay(pdMS_TO_TICKS(delay_ms));
    } else {
      taskYIELD();
    }
  }

  return res;
}

// Start HTTP server
void startCameraServer() {
  httpd_config_t config = HTTPD_DEFAULT_CONFIG();
  config.server_port = 80;
  config.ctrl_port = 32768;

  httpd_uri_t index_uri = {
    .uri = "/",
    .method = HTTP_GET,
    .handler = index_handler,
    .user_ctx = NULL
  };

  httpd_uri_t stream_uri = {
    .uri = "/stream",
    .method = HTTP_GET,
    .handler = stream_handler,
    .user_ctx = NULL
  };

  Serial.printf("[INFO] Starting HTTP server on port %d...\n", config.server_port);
  if (httpd_start(&stream_httpd, &config) == ESP_OK) {
    httpd_register_uri_handler(stream_httpd, &index_uri);
    httpd_register_uri_handler(stream_httpd, &stream_uri);
    Serial.println("[INFO] HTTP server successfully started.");
  } else {
    Serial.println("[ERROR] Failed to start HTTP server.");
  }
}

// I2C bus scanner helper
void scanI2CBus(int sda, int scl) {
  Wire.begin(sda, scl);
  Serial.printf("[INFO] Scanning I2C bus on SDA=%d, SCL=%d...\n", sda, scl);
  int count = 0;
  for (byte address = 1; address < 127; address++) {
    Wire.beginTransmission(address);
    byte error = Wire.endTransmission();
    if (error == 0) {
      Serial.printf("  Found I2C device at 0x%02X", address);
      if (address == 0x3C) {
        Serial.print(" (GC2145 SCCB Address!)");
      }
      Serial.println();
      count++;
    }
  }
  if (count == 0) {
    Serial.println("  No I2C devices found on this bus.");
  }
}

void setup() {
  Serial.begin(115200);
  delay(1500);
  Serial.println("\n==========================================");
  Serial.println(" ESP32-S3 GC2145 Drone Camera Streamer   ");
  Serial.println("==========================================");

  // PSRAM Diagnostic
  if (psramFound()) {
    Serial.printf("[INFO] PSRAM detected: Total %d KB, Free %d KB\n",
                  ESP.getPsramSize() / 1024, ESP.getFreePsram() / 1024);
  } else {
    Serial.println("[WARN] No PSRAM detected! Memory will be tightly constrained.");
  }

  // Pre-probe SCCB bus
  scanI2CBus(SIOD_GPIO_NUM, SIOC_GPIO_NUM);

  // Camera configuration
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sccb_sda = SIOD_GPIO_NUM;
  config.pin_sccb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;

  // Use 20MHz XCLK for GC2145
  config.xclk_freq_hz = 20000000;

  // GC2145 outputs RGB565 / YUV422 - No native hardware JPEG
  config.pixel_format = PIXFORMAT_RGB565;
  config.frame_size = FRAMESIZE_QVGA; // 320x240 for reliable low-latency streaming
  config.jpeg_quality = 12;
  config.fb_count = 2;
  config.fb_location = CAMERA_FB_IN_PSRAM;
  config.grab_mode = CAMERA_GRAB_LATEST;

  Serial.println("[INFO] Initializing camera with GC2145 config...");
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("[ERROR] esp_camera_init failed with error code: 0x%x\n", err);
    Serial.println("Diagnostics:");
    if (err == ESP_ERR_NOT_SUPPORTED) {
      Serial.println("  ESP_ERR_NOT_SUPPORTED: Format/resolution combination rejected.");
    } else if (err == ESP_ERR_NOT_FOUND) {
      Serial.println("  ESP_ERR_NOT_FOUND: Camera sensor not detected on SCCB bus.");
    }
    camera_initialized = false;
  } else {
    Serial.println("[INFO] Camera initialized successfully!");
    camera_initialized = true;

    sensor_t *s = esp_camera_sensor_get();
    if (s != NULL) {
      Serial.printf("[INFO] Sensor PID: 0x%04X, VER: 0x%04X\n", s->id.PID, s->id.VER);
    }
  }

  // Connect to Wi-Fi
  Serial.printf("[INFO] Connecting to Wi-Fi SSID: %s\n", WIFI_SSID);
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  WiFi.setSleep(false);

  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 40) {
    delay(500);
    Serial.print(".");
    attempts++;
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\n[INFO] Wi-Fi connected successfully!");
    Serial.print("[INFO] ESP32 Camera IP Address: http://");
    Serial.println(WiFi.localIP());
    Serial.print("[INFO] Live MJPEG Stream URL:  http://");
    Serial.print(WiFi.localIP());
    Serial.println("/stream");

    // Start HTTP server
    startCameraServer();
  } else {
    Serial.println("\n[WARN] Wi-Fi connection timed out. Camera server not started.");
  }
}

void loop() {
  // Monitor Wi-Fi connection and reconnect if dropped
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("[WARN] Wi-Fi disconnected. Reconnecting...");
    WiFi.reconnect();
    unsigned long start = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - start < 10000) {
      delay(500);
      Serial.print(".");
    }
    if (WiFi.status() == WL_CONNECTED) {
      Serial.print("\n[INFO] Reconnected! IP: ");
      Serial.println(WiFi.localIP());
    }
  }
  delay(5000);
}
