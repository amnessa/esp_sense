#pragma once

#include <stdint.h>

// Snapshot of the tracker, safe to read from any task.
//
// err_x / err_y are the quantities a pan-tilt loop consumes: the face centre's
// offset from the image centre, normalised to [-1, +1] on each axis. They are
// deliberately *held* at their last good values when a face is lost, so a
// controller reading this struct coasts instead of snapping back to centre.
// Use `age_ms` to decide when a track has gone stale enough to give up on.
struct DetectionState {
  bool valid;        // a face was found in the most recent inference
  float err_x;       // [-1, +1], positive = face is right of centre
  float err_y;       // [-1, +1], positive = face is below centre
  float score;
  int box[4];        // x1, y1, x2, y2 in detector-frame pixels
  int frame_w;       // detector frame size, for scaling box[] to other frames
  int frame_h;
  uint32_t seq;      // increments once per completed inference
  uint32_t age_ms;   // since the last *valid* detection
  uint32_t infer_ms; // duration of the last inference
};

// Spawns the detection task. Call after esp_camera_init().
bool detector_start();

void detector_get(DetectionState* out);
