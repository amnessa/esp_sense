#include "detector.h"

#include <Arduino.h>
#include <list>
#include <vector>

#include "esp_camera.h"
#include "esp_timer.h"
#include "img_converters.h"

#include "human_face_detect_msr01.hpp"
#if DETECT_TWO_STAGE
#include "human_face_detect_mnp01.hpp"
#endif

// The stream stays a raw JPEG passthrough; detection works from its own decoded
// copy. jpg2rgb565() downscales during decode, so asking for half size costs
// less than decoding full frames and shrinking afterwards.
#define DECODE_SCALE JPG_SCALE_2X
#define DECODE_DIVISOR 2

// Sized for VGA/2. Larger camera framesizes are skipped rather than truncated.
#define MAX_DETECT_W 320
#define MAX_DETECT_H 240

static DetectionState g_state = {};
static SemaphoreHandle_t g_lock = NULL;
static uint8_t* g_rgb = NULL;
static int64_t g_last_valid_us = 0;

// jpg2rgb565() writes each pixel in the opposite byte order to the one esp-dl
// reads, which scrambles red and blue and shifts green. The image stays
// structurally valid, so this fails silently rather than erroring. Measured on
// this board: 0 hits in 80 frames as decoded, 21 in 80 after swapping.
static void swap_bytes(uint8_t* buf, size_t pixels) {
  uint16_t* p = (uint16_t*)buf;
  for (size_t i = 0; i < pixels; i++) p[i] = (uint16_t)((p[i] >> 8) | (p[i] << 8));
}

static void publish(const DetectionState& s) {
  xSemaphoreTake(g_lock, portMAX_DELAY);
  g_state = s;
  xSemaphoreGive(g_lock);
}

void detector_get(DetectionState* out) {
  xSemaphoreTake(g_lock, portMAX_DELAY);
  *out = g_state;
  xSemaphoreGive(g_lock);

  // Recomputed on read so a caller polling slowly still sees the true age.
  int64_t last = g_last_valid_us;
  out->age_ms = last ? (uint32_t)((esp_timer_get_time() - last) / 1000) : UINT32_MAX;
}

static void detector_task(void*) {
  // DETECT_RESIZE_SCALE is the knob that decides the smallest detectable face:
  // the network runs on the frame scaled by this factor, so 0.4 on a 320x240
  // input means a 128x96 image. Raising it costs roughly quadratic time and
  // buys recall on smaller and more oblique faces.
  HumanFaceDetectMSR01 s1(DETECT_SCORE, 0.5F, 10, DETECT_RESIZE_SCALE);
#if DETECT_TWO_STAGE
  HumanFaceDetectMNP01 s2(0.5F, 0.3F, 5);
#endif

  DetectionState st = {};

#if DETECT_STATS
  uint32_t hits = 0, runs = 0, ms_sum = 0;
#endif

  while (true) {
    camera_fb_t* fb = esp_camera_fb_get();
    if (!fb) {
      vTaskDelay(pdMS_TO_TICKS(10));
      continue;
    }

    const int w = fb->width / DECODE_DIVISOR;
    const int h = fb->height / DECODE_DIVISOR;

    if (w > MAX_DETECT_W || h > MAX_DETECT_H) {
      esp_camera_fb_return(fb);
      vTaskDelay(pdMS_TO_TICKS(100));
      continue;
    }

    bool decoded = jpg2rgb565(fb->buf, fb->len, g_rgb, DECODE_SCALE);
    esp_camera_fb_return(fb);  // released before inference; we own g_rgb now

    if (!decoded) {
      Serial.println("Detector: jpg2rgb565 failed");
      vTaskDelay(pdMS_TO_TICKS(10));
      continue;
    }

    swap_bytes(g_rgb, (size_t)w * h);

    const int64_t t0 = esp_timer_get_time();

    std::vector<int> shape = {h, w, 3};
#if DETECT_TWO_STAGE
    std::list<dl::detect::result_t>& candidates = s1.infer((uint16_t*)g_rgb, shape);
    std::list<dl::detect::result_t>& results = s2.infer((uint16_t*)g_rgb, shape, candidates);
#else
    std::list<dl::detect::result_t>& results = s1.infer((uint16_t*)g_rgb, shape);
#endif

    st.infer_ms = (uint32_t)((esp_timer_get_time() - t0) / 1000);
    st.frame_w = w;
    st.frame_h = h;
    st.seq++;

#if DETECT_STATS
    runs++;
    ms_sum += st.infer_ms;
    if (!results.empty()) hits++;

    if (runs >= 50) {
      Serial.printf("STATS  %u/%u frames with a face   %u ms avg   scale %.2f\n", hits, runs,
                    ms_sum / runs, (double)DETECT_RESIZE_SCALE);
      hits = runs = ms_sum = 0;
    }
#endif

    // Largest box wins: for a pan-tilt tracker that means following the nearest
    // face rather than flicking between people in the background.
    const dl::detect::result_t* best = NULL;
    int best_area = 0;
    for (const auto& r : results) {
      const int area = (r.box[2] - r.box[0]) * (r.box[3] - r.box[1]);
      if (area > best_area) {
        best_area = area;
        best = &r;
      }
    }

    if (best) {
      const float cx = (best->box[0] + best->box[2]) * 0.5f;
      const float cy = (best->box[1] + best->box[3]) * 0.5f;

      st.valid = true;
      st.score = best->score;
      st.err_x = (cx - w * 0.5f) / (w * 0.5f);
      st.err_y = (cy - h * 0.5f) / (h * 0.5f);
      for (int i = 0; i < 4; i++) st.box[i] = best->box[i];

      g_last_valid_us = esp_timer_get_time();
    } else {
      // err_x/err_y and box[] keep their previous values on purpose.
      st.valid = false;
      st.score = 0.0f;
    }

    publish(st);

    vTaskDelay(1);  // keep the idle task fed
  }
}

bool detector_start() {
  g_lock = xSemaphoreCreateMutex();
  if (!g_lock) {
    Serial.println("Detector: mutex alloc failed");
    return false;
  }

  g_rgb = (uint8_t*)heap_caps_malloc(MAX_DETECT_W * MAX_DETECT_H * 2, MALLOC_CAP_SPIRAM);
  if (!g_rgb) {
    Serial.println("Detector: RGB565 buffer alloc failed (needs PSRAM)");
    return false;
  }

  // Core 0: Arduino and the HTTP servers run on core 1 for this board.
  BaseType_t ok = xTaskCreatePinnedToCore(detector_task, "detect", 10240, NULL, 1, NULL, 0);
  if (ok != pdPASS) {
    Serial.println("Detector: task create failed");
    return false;
  }

  return true;
}
