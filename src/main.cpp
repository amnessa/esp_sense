#include <Arduino.h>
#include <WiFi.h>
#include "esp_camera.h"
#include "esp_http_server.h"

#define CAMERA_MODEL_XIAO_ESP32S3
#include "camera_pins.h"

const char* ssid = "SUPERONLINE_Wi-Fi_A662";
const char* password = "FReSBNCQy4";

// Multipart MJPEG framing: the browser keeps one connection open and we
// push a new JPEG after every boundary marker.
#define PART_BOUNDARY "frameboundary"
static const char* STREAM_CONTENT_TYPE = "multipart/x-mixed-replace;boundary=" PART_BOUNDARY;
static const char* STREAM_BOUNDARY = "\r\n--" PART_BOUNDARY "\r\n";
static const char* STREAM_PART = "Content-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n";

static httpd_handle_t index_httpd = NULL;
static httpd_handle_t stream_httpd = NULL;

static const char INDEX_HTML[] = R"rawliteral(
<!doctype html><html><head><meta name="viewport" content="width=device-width,initial-scale=1">
<title>XIAO ESP32S3 Sense</title>
<style>body{background:#111;color:#eee;font-family:sans-serif;text-align:center;margin:0;padding:1rem}
img{max-width:100%;border-radius:8px}</style></head>
<body><h2>XIAO ESP32S3 Sense</h2>
<img src="" id="s"><script>
document.getElementById('s').src = 'http://' + location.hostname + ':81/stream';
</script></body></html>
)rawliteral";

static esp_err_t index_handler(httpd_req_t* req) {
  httpd_resp_set_type(req, "text/html");
  return httpd_resp_send(req, INDEX_HTML, HTTPD_RESP_USE_STRLEN);
}

static esp_err_t capture_handler(httpd_req_t* req) {
  camera_fb_t* fb = esp_camera_fb_get();
  if (!fb) {
    httpd_resp_send_500(req);
    return ESP_FAIL;
  }

  httpd_resp_set_type(req, "image/jpeg");
  httpd_resp_set_hdr(req, "Content-Disposition", "inline; filename=capture.jpg");
  esp_err_t res = httpd_resp_send(req, (const char*)fb->buf, fb->len);

  esp_camera_fb_return(fb);
  return res;
}

static esp_err_t stream_handler(httpd_req_t* req) {
  esp_err_t res = httpd_resp_set_type(req, STREAM_CONTENT_TYPE);
  if (res != ESP_OK) return res;

  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");

  char part_buf[64];
  while (true) {
    camera_fb_t* fb = esp_camera_fb_get();
    if (!fb) {
      Serial.println("Camera capture failed");
      res = ESP_FAIL;
      break;
    }

    size_t hlen = snprintf(part_buf, sizeof(part_buf), STREAM_PART, fb->len);

    res = httpd_resp_send_chunk(req, STREAM_BOUNDARY, strlen(STREAM_BOUNDARY));
    if (res == ESP_OK) res = httpd_resp_send_chunk(req, part_buf, hlen);
    if (res == ESP_OK) res = httpd_resp_send_chunk(req, (const char*)fb->buf, fb->len);

    esp_camera_fb_return(fb);

    // Client closed the tab; stop pushing frames.
    if (res != ESP_OK) break;
  }

  return res;
}

static void start_servers() {
  httpd_config_t config = HTTPD_DEFAULT_CONFIG();
  config.server_port = 80;
  config.ctrl_port = 32768;

  httpd_uri_t index_uri = {"/", HTTP_GET, index_handler, NULL};
  httpd_uri_t capture_uri = {"/capture", HTTP_GET, capture_handler, NULL};

  if (httpd_start(&index_httpd, &config) == ESP_OK) {
    httpd_register_uri_handler(index_httpd, &index_uri);
    httpd_register_uri_handler(index_httpd, &capture_uri);
  }

  // The stream occupies its worker for as long as a client is watching, so it
  // gets its own server on port 81 and leaves the page/capture responsive.
  config.server_port = 81;
  config.ctrl_port = 32769;

  httpd_uri_t stream_uri = {"/stream", HTTP_GET, stream_handler, NULL};

  if (httpd_start(&stream_httpd, &config) == ESP_OK) {
    httpd_register_uri_handler(stream_httpd, &stream_uri);
  }
}

static bool init_camera() {
  camera_config_t config = {};

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
  config.xclk_freq_hz = 20000000;

  // The OV2640 emits JPEG itself, so we forward its bytes untouched.
  config.pixel_format = PIXFORMAT_JPEG;
  config.frame_size = FRAMESIZE_VGA;  // 640x480
  config.jpeg_quality = 12;           // 10-63, lower is better quality
  config.fb_count = 2;                // double buffering needs PSRAM
  config.fb_location = CAMERA_FB_IN_PSRAM;
  config.grab_mode = CAMERA_GRAB_LATEST;

  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed: 0x%x\n", err);
    return false;
  }

  sensor_t* s = esp_camera_sensor_get();
  s->set_vflip(s, 1);  // the Sense expansion board mounts the lens inverted

  return true;
}

void setup() {
  Serial.begin(115200);
  delay(2000);

  Serial.println("\n--- XIAO ESP32S3 Sense Video Stream ---");

  if (!psramFound()) {
    Serial.println("WARNING: PSRAM not detected, streaming will be unstable");
  }

  if (!init_camera()) {
    Serial.println("Halting.");
    while (true) delay(1000);
  }

  Serial.print("Connecting to WiFi network: ");
  Serial.println(ssid);

  WiFi.begin(ssid, password);
  WiFi.setSleep(false);  // modem sleep adds latency spikes to the stream

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("\nConnected to WiFi network!");

  start_servers();

  Serial.print("Viewer:   http://");
  Serial.println(WiFi.localIP());
  Serial.print("Stream:   http://");
  Serial.print(WiFi.localIP());
  Serial.println(":81/stream");
  Serial.print("Snapshot: http://");
  Serial.print(WiFi.localIP());
  Serial.println("/capture");
}

void loop() {
  delay(10000);
}
