#pragma once

// Pan/tilt visual-servoing loop. Consumes DetectionState from detector.h and
// drives two hobby servos over LEDC.
//
// Call tracker_start() AFTER esp_camera_init(): the camera driver claims LEDC
// timer 0 / channel 0 for XCLK, and attaching first would let it reconfigure
// the timer out from under us.

#include <stdint.h>

bool tracker_start();

// Advances the ramp and, on detector updates, the control law. Cheap and
// non-blocking; call it as fast as loop() can (target ~100Hz, see TICK_MS).
void tracker_tick();

// Current commanded pulses, for status printing.
void tracker_pulses(float* pan_us, float* tilt_us);

// Stop emitting pulses and let the head go limp.
void tracker_relax();
