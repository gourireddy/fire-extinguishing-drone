#include <Arduino.h>
#include <WiFi.h>
#include <Wire.h>
#include "esp_camera.h"
#include "img_converters.h"
#include "esp_http_server.h"
#include "driver/ledc.h"

#include "hardware_config.h"
#include "camera_pins.h"
#include "wifi_credentials.h"
#include "safety_interlock.h"

// =============================================================================
// STAGE 4: UNIFIED CAMERA STREAMING, WEBSOCKET TELEMETRY & SAFETY CONTROL
// Target: ESP32-S3-WROOM-1 (N16R8) | Arduino Core 3.3.12 (ESP-IDF 5.5.5)
// =============================================================================

// Safety Interlock & Pump State Machine
SafetyInterlockController safety;

// Server Handle
httpd_handle_t httpd_server = NULL;
bool camera_initialized = false;

// --- Servo Subsystem Parameters ---
uint32_t active_pan_min_us  = ACTIVE_PAN_MIN_US;   // 1333 us
uint32_t active_pan_max_us  = ACTIVE_PAN_MAX_US;   // 1667 us
uint32_t active_tilt_min_us = ACTIVE_TILT_MIN_US;  // 1333 us
uint32_t active_tilt_max_us = ACTIVE_TILT_MAX_US;  // 1667 us

float cur_pan_us  = SERVO_PULSE_NEUTRAL_US; // 1500 us
float cur_tilt_us = SERVO_PULSE_NEUTRAL_US; // 1500 us
float tgt_pan_us  = SERVO_PULSE_NEUTRAL_US; // 1500 us
float tgt_tilt_us = SERVO_PULSE_NEUTRAL_US; // 1500 us

float slew_us_per_tick = SERVO_DEFAULT_SLEW_US_PER_TICK; // 10 us / 20ms tick
unsigned long last_servo_update = 0;

// Serial input buffer
String serial_input_buffer = "";

// Multipart boundary for MJPEG streaming
#define PART_BOUNDARY "123456789000000000000987654321"
static const char* _STREAM_CONTENT_TYPE = "multipart/x-mixed-replace;boundary=" PART_BOUNDARY;
static const char* _STREAM_BOUNDARY = "\r\n--" PART_BOUNDARY "\r\n";
static const char* _STREAM_PART = "Content-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n";

// Convert microseconds to 14-bit LEDC duty ticks (50 Hz / 20 ms period)
inline uint32_t usToDuty(float us) {
  return (uint32_t)(us * 0.8192f + 0.5f);
}

float usToNominalAngle(float us) {
  us = constrain(us, (float)SERVO_PHYSICAL_MIN_US, (float)SERVO_PHYSICAL_MAX_US);
  return ((us - SERVO_PHYSICAL_MIN_US) / (float)(SERVO_PHYSICAL_MAX_US - SERVO_PHYSICAL_MIN_US)) * 180.0f;
}

bool isValidPositiveInt(const String &str, int &out_val) {
  String s = str;
  s.trim();
  if (s.length() == 0 || s.length() > 5) return false;
  for (unsigned int i = 0; i < s.length(); i++) {
    if (!isDigit(s.charAt(i))) return false;
  }
  long parsed = s.toInt();
  if (parsed < 0 || parsed > 65535) return false;
  out_val = (int)parsed;
  return true;
}

// Initialize LEDC Hardware Timer for Servos
void initServos() {
  ledc_timer_config_t servo_timer = {
    .speed_mode       = LEDC_LOW_SPEED_MODE,
    .duty_resolution  = (ledc_timer_bit_t)SERVO_PWM_RES_BITS,
    .timer_num        = SERVO_LEDC_TIMER,
    .freq_hz          = SERVO_PWM_FREQ_HZ,
    .clk_cfg          = LEDC_AUTO_CLK
  };
  ESP_ERROR_CHECK(ledc_timer_config(&servo_timer));

  ledc_channel_config_t pan_ch = {
    .gpio_num   = PIN_SERVO_PAN,
    .speed_mode = LEDC_LOW_SPEED_MODE,
    .channel    = SERVO_PAN_CHANNEL,
    .intr_type  = LEDC_INTR_DISABLE,
    .timer_sel  = SERVO_LEDC_TIMER,
    .duty       = usToDuty(SERVO_PULSE_NEUTRAL_US),
    .hpoint     = 0
  };
  ESP_ERROR_CHECK(ledc_channel_config(&pan_ch));

  ledc_channel_config_t tilt_ch = {
    .gpio_num   = PIN_SERVO_TILT,
    .speed_mode = LEDC_LOW_SPEED_MODE,
    .channel    = SERVO_TILT_CHANNEL,
    .intr_type  = LEDC_INTR_DISABLE,
    .timer_sel  = SERVO_LEDC_TIMER,
    .duty       = usToDuty(SERVO_PULSE_NEUTRAL_US),
    .hpoint     = 0
  };
  ESP_ERROR_CHECK(ledc_channel_config(&tilt_ch));

  cur_pan_us  = SERVO_PULSE_NEUTRAL_US;
  tgt_pan_us  = SERVO_PULSE_NEUTRAL_US;
  cur_tilt_us = SERVO_PULSE_NEUTRAL_US;
  tgt_tilt_us = SERVO_PULSE_NEUTRAL_US;
}

void setTargetPanUs(float us) {
  float clamped = constrain(us, (float)active_pan_min_us, (float)active_pan_max_us);
  if (clamped != tgt_pan_us) {
    tgt_pan_us = clamped;
    safety.notifyServoMotion(true);
  }
}

void setTargetTiltUs(float us) {
  float clamped = constrain(us, (float)active_tilt_min_us, (float)active_tilt_max_us);
  if (clamped != tgt_tilt_us) {
    tgt_tilt_us = clamped;
    safety.notifyServoMotion(true);
  }
}

void updateServos() {
  unsigned long now = millis();
  if (now - last_servo_update < 20) return;
  last_servo_update = now;

  bool was_slewing = false;

  // Slew Pan
  if (cur_pan_us < tgt_pan_us) {
    cur_pan_us = min(cur_pan_us + slew_us_per_tick, tgt_pan_us);
    was_slewing = true;
  } else if (cur_pan_us > tgt_pan_us) {
    cur_pan_us = max(cur_pan_us - slew_us_per_tick, tgt_pan_us);
    was_slewing = true;
  }

  // Slew Tilt
  if (cur_tilt_us < tgt_tilt_us) {
    cur_tilt_us = min(cur_tilt_us + slew_us_per_tick, tgt_tilt_us);
    was_slewing = true;
  } else if (cur_tilt_us > tgt_tilt_us) {
    cur_tilt_us = max(cur_tilt_us - slew_us_per_tick, tgt_tilt_us);
    was_slewing = true;
  }

  safety.notifyServoMotion(was_slewing);

  ledc_set_duty(LEDC_LOW_SPEED_MODE, SERVO_PAN_CHANNEL, usToDuty(cur_pan_us));
  ledc_update_duty(LEDC_LOW_SPEED_MODE, SERVO_PAN_CHANNEL);

  ledc_set_duty(LEDC_LOW_SPEED_MODE, SERVO_TILT_CHANNEL, usToDuty(cur_tilt_us));
  ledc_update_duty(LEDC_LOW_SPEED_MODE, SERVO_TILT_CHANNEL);
}

// Camera Initialization
bool initCamera() {
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_4;
  config.ledc_timer   = LEDC_TIMER_2;
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
  config.pin_sccb_sda = SIOD_GPIO_NUM;
  config.pin_sccb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn     = PWDN_GPIO_NUM;
  config.pin_reset    = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_RGB565;
  config.frame_size   = FRAMESIZE_QVGA;  // 320x240
  config.jpeg_quality = 12;
  config.fb_count     = 2;
  config.fb_location  = CAMERA_FB_IN_PSRAM;
  config.grab_mode    = CAMERA_GRAB_LATEST;

  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("[CAMERA] Driver initialization failed: 0x%x\n", err);
    return false;
  }

  sensor_t *s = esp_camera_sensor_get();
  if (s != NULL) {
    s->set_vflip(s, 1);
    s->set_hmirror(s, 0);
  }
  return true;
}

// Web root handler
static const char PROGMEM INDEX_HTML[] = R"rawliteral(
<!DOCTYPE html><html><head><meta charset="utf-8"><title>Drone Camera & Telemetry</title>
<style>body{margin:0;background:#111;color:#fff;font-family:sans-serif;text-align:center;}
img{max-width:100vw;max-height:90vh;object-fit:contain;}</style></head>
<body><h2>Indoor Fire Extinguishing Drone Live Stream</h2><img src="/stream"/></body></html>
)rawliteral";

static esp_err_t index_handler(httpd_req_t *req) {
  httpd_resp_set_type(req, "text/html");
  return httpd_resp_send(req, INDEX_HTML, HTTPD_RESP_USE_STRLEN);
}

// MJPEG stream handler (converts RGB565 to JPEG in PSRAM, 10 FPS paced)
static esp_err_t stream_handler(httpd_req_t *req) {
  camera_fb_t *fb = NULL;
  esp_err_t res = ESP_OK;
  size_t _jpg_buf_len = 0;
  uint8_t *_jpg_buf = NULL;
  char part_buf[64];

  res = httpd_resp_set_type(req, _STREAM_CONTENT_TYPE);
  if (res != ESP_OK) return res;
  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");

  const int64_t target_frame_time_us = 100000; // 10 FPS = 100ms

  while (true) {
    int64_t frame_start = esp_timer_get_time();

    fb = esp_camera_fb_get();
    if (!fb) {
      vTaskDelay(pdMS_TO_TICKS(50));
      continue;
    }

    if (fb->format == PIXFORMAT_JPEG) {
      _jpg_buf = fb->buf;
      _jpg_buf_len = fb->len;
    } else {
      bool converted = frame2jpg(fb, 65, &_jpg_buf, &_jpg_buf_len);
      if (!converted) {
        esp_camera_fb_return(fb);
        continue;
      }
    }

    if (res == ESP_OK) res = httpd_resp_send_chunk(req, _STREAM_BOUNDARY, strlen(_STREAM_BOUNDARY));
    if (res == ESP_OK) {
      size_t hlen = snprintf(part_buf, 64, _STREAM_PART, _jpg_buf_len);
      res = httpd_resp_send_chunk(req, part_buf, hlen);
    }
    if (res == ESP_OK) res = httpd_resp_send_chunk(req, (const char *)_jpg_buf, _jpg_buf_len);

    if (fb->format != PIXFORMAT_JPEG && _jpg_buf != NULL) {
      free(_jpg_buf);
      _jpg_buf = NULL;
    }
    esp_camera_fb_return(fb);
    fb = NULL;

    if (res != ESP_OK) break;

    int64_t elapsed_us = esp_timer_get_time() - frame_start;
    if (elapsed_us < target_frame_time_us) {
      vTaskDelay(pdMS_TO_TICKS((target_frame_time_us - elapsed_us) / 1000));
    } else {
      taskYIELD();
    }
  }
  return res;
}

// WebSocket JSON Message Parser & Dispatcher
void processWebSocketCommand(const char *json_str, httpd_req_t *req) {
  String msg = String(json_str);

  // 1. Authentication Token Check
  if (msg.indexOf(AUTH_SHARED_TOKEN) < 0) {
    char err_buf[128];
    snprintf(err_buf, sizeof(err_buf), "{\"error\":\"UNAUTHORIZED_TOKEN\"}");
    httpd_ws_frame_t ws_reply;
    memset(&ws_reply, 0, sizeof(ws_reply));
    ws_reply.payload = (uint8_t*)err_buf;
    ws_reply.len = strlen(err_buf);
    ws_reply.type = HTTPD_WS_TYPE_TEXT;
    httpd_ws_send_frame(req, &ws_reply);
    return;
  }

  // Extract sequence number
  uint32_t seq = 0;
  int seq_idx = msg.indexOf("\"seq\":");
  if (seq_idx >= 0) {
    seq = msg.substring(seq_idx + 6).toInt();
    if (seq <= safety.getLastSeqProcessed() && seq != 0) {
      // Replayed or stale sequence packet
      return;
    }
    safety.setLastSeqProcessed(seq);
  }

  safety.feedHeartbeat();

  // Parse Command Types
  if (msg.indexOf("\"type\":\"arm\"") >= 0) {
    safety.arm();
  } else if (msg.indexOf("\"type\":\"disarm\"") >= 0) {
    safety.disarm();
  } else if (msg.indexOf("\"type\":\"estop\"") >= 0) {
    safety.emergencyStop();
  } else if (msg.indexOf("\"type\":\"target\"") >= 0) {
    // Extract target parameters
    bool detected = (msg.indexOf("\"detected\":true") >= 0);
    float conf = 0.0f;
    float pan_err = 0.0f;
    float tilt_err = 0.0f;
    float dist = 1.0f;

    int c_idx = msg.indexOf("\"conf\":");
    if (c_idx >= 0) conf = msg.substring(c_idx + 7).toFloat();

    int p_idx = msg.indexOf("\"pan_err\":");
    if (p_idx >= 0) pan_err = msg.substring(p_idx + 10).toFloat();

    int t_idx = msg.indexOf("\"tilt_err\":");
    if (t_idx >= 0) tilt_err = msg.substring(t_idx + 11).toFloat();

    int d_idx = msg.indexOf("\"dist_m\":");
    if (d_idx >= 0) dist = msg.substring(d_idx + 9).toFloat();

    safety.updateTargetData(detected, conf, pan_err, tilt_err, dist);
  } else if (msg.indexOf("\"type\":\"servo\"") >= 0) {
    int p_idx = msg.indexOf("\"pan_us\":");
    if (p_idx >= 0) {
      float pus = msg.substring(p_idx + 9).toFloat();
      setTargetPanUs(pus);
    }
    int t_idx = msg.indexOf("\"tilt_us\":");
    if (t_idx >= 0) {
      float tus = msg.substring(t_idx + 10).toFloat();
      setTargetTiltUs(tus);
    }
  } else if (msg.indexOf("\"type\":\"discharge\"") >= 0) {
    uint32_t dur = PUMP_BURST_DEFAULT_MS;
    int d_idx = msg.indexOf("\"duration_ms\":");
    if (d_idx >= 0) dur = msg.substring(d_idx + 14).toInt();
    safety.requestDischarge(dur);
  }

  // Construct JSON Telemetry Response
  char resp[256];
  InterlockStatus eval = safety.evaluateDischargeConditions(PUMP_BURST_DEFAULT_MS);
  snprintf(resp, sizeof(resp),
    "{\"ack\":%u,\"state\":\"%s\",\"fault\":\"%s\",\"interlock\":\"%s\",\"pan_us\":%.0f,\"tilt_us\":%.0f,\"bursting\":%s,\"simulated\":%s}",
    seq,
    safety.getStateStr(),
    safety.getFaultReasonStr(),
    SafetyInterlockController::getInterlockFailStr(eval),
    cur_pan_us,
    cur_tilt_us,
    safety.isBursting() ? "true" : "false",
    STAGE4_SIMULATE_PUMP_ONLY ? "true" : "false"
  );

  httpd_ws_frame_t ws_reply;
  memset(&ws_reply, 0, sizeof(ws_reply));
  ws_reply.payload = (uint8_t*)resp;
  ws_reply.len = strlen(resp);
  ws_reply.type = HTTPD_WS_TYPE_TEXT;
  httpd_ws_send_frame(req, &ws_reply);
}

// WebSocket handler
static esp_err_t ws_handler(httpd_req_t *req) {
  if (req->method == HTTP_GET) return ESP_OK;

  httpd_ws_frame_t ws_pkt;
  memset(&ws_pkt, 0, sizeof(httpd_ws_frame_t));
  ws_pkt.type = HTTPD_WS_TYPE_TEXT;

  esp_err_t ret = httpd_ws_recv_frame(req, &ws_pkt, 0);
  if (ret != ESP_OK) return ret;

  if (ws_pkt.len) {
    uint8_t *buf = (uint8_t *)calloc(1, ws_pkt.len + 1);
    if (!buf) return ESP_ERR_NO_MEM;
    ws_pkt.payload = buf;
    ret = httpd_ws_recv_frame(req, &ws_pkt, ws_pkt.len);
    if (ret == ESP_OK) {
      processWebSocketCommand((char*)buf, req);
    }
    free(buf);
  }
  return ret;
}

void startHttpServer() {
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
  httpd_uri_t ws_uri = {
    .uri = "/ws",
    .method = HTTP_GET,
    .handler = ws_handler,
    .user_ctx = NULL,
    .is_websocket = true
  };

  if (httpd_start(&httpd_server, &config) == ESP_OK) {
    httpd_register_uri_handler(httpd_server, &index_uri);
    httpd_register_uri_handler(httpd_server, &stream_uri);
    httpd_register_uri_handler(httpd_server, &ws_uri);
    Serial.println("[HTTP/WS] Server initialized on port 80.");
  }
}

void setup() {
  // STEP 0: Safety clamping
  safety.init();

  Serial.begin(115200);
  delay(1200);

  Serial.println("\n==============================================================");
  Serial.println("  STAGE 4: UNIFIED CAMERA STREAMING & WEBSOCKET SAFETY SYSTEM ");
  Serial.println("  Core: Arduino-ESP32 3.3.12 (ESP-IDF 5.5.5)                 ");
  Serial.println("==============================================================");
  Serial.println("[SAFETY] Physical pump drive strictly DISABLED (Simulation Mode).");
  Serial.println("[SAFETY] GPIO 21 locked LOW (0.0V).");
  Serial.println("[SAFETY] 8-Point autonomous discharge interlock active.");

  // Init servos
  initServos();
  Serial.println("[ACTUATOR] Servos centered at 1500 us.");

  // Init camera
  camera_initialized = initCamera();
  if (camera_initialized) {
    Serial.println("[CAMERA] GC2145 initialized (QVGA RGB565).");
  } else {
    Serial.println("[WARN] Camera init failed or not connected. Continuing networking.");
  }

  // Connect to Wi-Fi
  Serial.printf("[WIFI] Connecting to '%s'...\n", WIFI_SSID);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 20) {
    delay(500);
    Serial.print(".");
    attempts++;
  }
  if (WiFi.status() == WL_CONNECTED) {
    Serial.printf("\n[WIFI] Connected! IP Address: %s\n", WiFi.localIP().toString().c_str());
    startHttpServer();
  } else {
    Serial.println("\n[WARN] Wi-Fi connection timeout. Standalone bench mode active.");
  }
}

void loop() {
  // Update safety interlock engine & timers
  safety.update();

  // Deterministic 50 Hz servo position update
  updateServos();
}
