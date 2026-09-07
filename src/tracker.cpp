#include "tracker.h"

#include <Arduino.h>
#include <math.h>

#include "detector.h"

// --- hardware map ----------------------------------------------------------
// Same pins and LEDC allocation the bench validated. Timer 1 / channels 2-3;
// the camera holds timer 0 / channel 0 for XCLK.
static const int PIN_PAN   = 1;   // D0
static const int PIN_TILT  = 2;   // D1
static const int CH_PAN    = 2;
static const int CH_TILT   = 3;

static const int      SERVO_HZ  = 50;
static const int      DUTY_BITS = 14;   // the S3's ceiling; 1.22us per step
static const uint32_t PERIOD_US = 1000000UL / SERVO_HZ;

// Hard clamp, identical to the bench's. Nothing below ever escapes it.
static const uint16_t US_MIN = 500;
static const uint16_t US_MAX = 2500;
static const uint16_t US_MID = 1500;

// --- travel limits ---------------------------------------------------------
// Conservative until the bench's measured endpoints are filled into the
// README table. Pull real measurements in by 50-100us before using them: a
// controller that commands into a mechanical stop sits there buzzing until
// the gears strip. Tilt is kept tighter than pan because it carries the head
// against gravity and has further to fall.
static const float PAN_LO  = 1200.0f, PAN_HI  = 1800.0f;
static const float TILT_LO = 1250.0f, TILT_HI = 1750.0f;

// --- control law -----------------------------------------------------------
// S = error-units per microsecond of servo travel: how much the face moves in
// the image for one us of pulse. Measured, not derived -- it absorbs the lens
// FOV, the gear ratio and the us/degree in one number. Procedure: settle on a
// stationary face, note err_x, step pan +100us, note err_x again,
// S_PAN = d(err_x) / d(us). These defaults assume ~11us/deg and the OV2640's
// ~65deg horizontal field, and are a starting point, not a calibration.
static const float S_PAN  = 0.0028f;
static const float S_TILT = 0.0037f;

// KP == 1.0 with a perfect S centres the face in one step. 0.4 leaves margin
// for the one-inference lag between the box and the video.
static const float KP = 0.4f;

// Below this the error is treated as zero. Without it the head dithers on
// detector noise even with a perfectly still face, and dither is what kills
// micro servos.
static const float DEADBAND = 0.04f;

// A track older than this is abandoned; the head holds where it is rather
// than snapping to centre (detector.h holds err_* at their last good values,
// so age_ms is the only thing that can tell us the track is stale).
static const uint32_t LOST_MS = 800;

// --- per-axis motion shaping ----------------------------------------------
// slewUsPerSec rate-limits the setpoint so the head does not shake the camera
// hard enough to wreck the next detection.
//
// stepUs is the tilt fix. A hobby servo has a 5-10us deadband, and near its
// edge the drive pulse is too short to break stiction on a gravity-loaded
// axis -- which is exactly why the bench's tilt servo only tracked at v3
// (2200us/s) and stalled at v1/v2. Quantising the *output* onto a coarse grid
// decouples smoothness from step size: the setpoint still ramps continuously,
// but every pulse actually emitted differs from the last by a full stepUs, so
// each one commands real torque. Pan is horizontal and barely loaded, so it
// gets a fine grid and stays smooth.
struct Axis {
  const char* name;
  int   pin;
  int   ch;
  float lo, hi;
  float slewUsPerSec;
  float stepUs;
  float setpoint;     // where the control law wants to be
  float current;      // slew-limited command
  int32_t written;    // last pulse actually emitted, quantised (-1 = none yet)
  bool  attached;
};

static Axis pan  = {"pan",  PIN_PAN,  CH_PAN,  PAN_LO,  PAN_HI,
                    900.0f,  4.0f, US_MID, US_MID, -1, false};
static Axis tilt = {"tilt", PIN_TILT, CH_TILT, TILT_LO, TILT_HI,
                    2200.0f, 12.0f, US_MID, US_MID, -1, false};

// Flip a sign if that axis runs *away* from the face -- the horn is bolted on
// the other way round. Cheaper than rewiring, and the README says so.
static const float PAN_SIGN  = +1.0f;  // +err_x (face right) -> longer pulse
static const float TILT_SIGN = +1.0f;  // +err_y (face low)   -> longer pulse

// ---------------------------------------------------------------------------
static uint32_t usToDuty(float us) {
  if (us < US_MIN) us = US_MIN;
  if (us > US_MAX) us = US_MAX;
  return (uint32_t)((us * (1UL << DUTY_BITS)) / PERIOD_US + 0.5f);
}

static bool axisAttach(Axis& a) {
  // ledcSetup returns the frequency it actually configured, or 0 if it could
  // not honour the request. Ignoring that turns a config error into a servo
  // that looks dead.
  double got = ledcSetup(a.ch, SERVO_HZ, DUTY_BITS);
  if (got == 0) {
    Serial.printf("[%s] ledcSetup FAILED (ch %d, %d bits) -- no pulses.\n",
                  a.name, a.ch, DUTY_BITS);
    return false;
  }
  ledcAttachPin(a.pin, a.ch);
  a.attached = true;
  a.written = -1;               // force the first write through
  Serial.printf("[%s] GPIO%d ch%d @ %.0fHz, %.0f-%.0fus, %.0fus/s, %.0fus grid\n",
                a.name, a.pin, a.ch, got, a.lo, a.hi, a.slewUsPerSec, a.stepUs);
  return true;
}

// Ramp toward the setpoint, then emit only if the quantised command changed.
static void axisTick(Axis& a, uint32_t dtMicros) {
  if (!a.attached) return;

  float d = a.setpoint - a.current;
  if (d != 0.0f) {
    // One tick can never stand for more than a frame's worth of time, or a
    // blocking print banks up dt and the ramp lurches.
    if (dtMicros > 20000) dtMicros = 20000;
    float step = a.slewUsPerSec * (dtMicros / 1000000.0f);
    a.current = (fabsf(d) <= step) ? a.setpoint
                                   : a.current + (d > 0 ? step : -step);
  }

  int32_t q = (int32_t)(roundf(a.current / a.stepUs) * a.stepUs);
  if (q == a.written) return;
  a.written = q;
  ledcWrite(a.ch, usToDuty((float)q));
}

// ---------------------------------------------------------------------------
bool tracker_start() {
  bool ok = axisAttach(pan);
  ok &= axisAttach(tilt);
  // Park at centre. Both axes start believing they are already there, so this
  // is a single instant write rather than a ramp from an unknown pose.
  pan.current = pan.setpoint = US_MID;
  tilt.current = tilt.setpoint = US_MID;
  axisTick(pan, 0);
  axisTick(tilt, 0);
  return ok;
}

void tracker_tick() {
  static uint32_t lastMicros = 0;
  static uint32_t lastSeq = 0;

  uint32_t now = micros();
  uint32_t dt = lastMicros ? (now - lastMicros) : 0;
  lastMicros = now;

  DetectionState d;
  detector_get(&d);

  // The control law runs once per *inference*, not once per tick. KP is sized
  // for one correction per detector update; applying it at loop rate would
  // multiply the effective gain by however many ticks fit in an inference and
  // the head would hunt violently.
  if (d.seq != lastSeq) {
    lastSeq = d.seq;

    // err_* are held at their last good values when a face is lost, so this
    // coasts instead of snapping to centre. age_ms is what ends the track --
    // and it is UINT32_MAX until the first valid detection, so this also
    // covers "never seen a face".
    if (d.age_ms <= LOST_MS) {
      float ex = fabsf(d.err_x) < DEADBAND ? 0.0f : d.err_x;
      float ey = fabsf(d.err_y) < DEADBAND ? 0.0f : d.err_y;

      pan.setpoint  = constrain(pan.setpoint  + PAN_SIGN  * KP * ex / S_PAN,
                                pan.lo,  pan.hi);
      tilt.setpoint = constrain(tilt.setpoint + TILT_SIGN * KP * ey / S_TILT,
                                tilt.lo, tilt.hi);
    }
  }

  axisTick(pan, dt);
  axisTick(tilt, dt);
}

void tracker_pulses(float* pan_us, float* tilt_us) {
  if (pan_us) *pan_us = pan.current;
  if (tilt_us) *tilt_us = tilt.current;
}

void tracker_relax() {
  ledcDetachPin(pan.pin);
  ledcDetachPin(tilt.pin);
  pan.attached = tilt.attached = false;
}
