// Standalone servo bench for the XIAO ESP32S3 + 2x GH-S37D pan/tilt rig.
//
// Build/upload this and nothing else:  pio run -e servo_test -t upload -t monitor
// The camera project in src/ is untouched.
//
// ---------------------------------------------------------------------------
// WIRING  (do this before powering anything)
//
//   servo brown/black  -> external supply GND  AND  XIAO GND   (common ground,
//                         non-negotiable: without it the pulse has no reference)
//   servo red          -> external 5V supply +  (NOT the XIAO 3V3 pin)
//   pan  orange/white  -> XIAO D0 (GPIO1)
//   tilt orange/white  -> XIAO D1 (GPIO2)
//
// Two micro servos can pull ~1A between them when they stall. The XIAO's 3V3
// regulator cannot do that, and a brownout mid-move looks exactly like "the
// servo is broken". Feed them from a separate 5V source (4xAA, a UBEC, or a
// 5V bench supply) and drop a 470-1000uF electrolytic across the servo rail
// near the connectors. 3.3V signal into a 5V micro servo is fine; these read
// the pulse with a logic input, not an analog threshold.
//
// ---------------------------------------------------------------------------
// ABOUT THE "90 or 180 degrees" QUESTION
//
// Neither is a property of the servo alone -- it is a property of the pulse
// range you feed it. The convention is 1000us = one end, 1500us = middle,
// 2000us = the other end, and that convention only covers ~90-120deg on most
// micro servos. The same horn will keep going out to roughly 180deg if you
// push the pulse toward 500us / 2500us, until the internal pot hits its
// mechanical stop.
//
// So the real answer is measured, not looked up: crawl outward from center in
// small steps until the servo stops responding or starts buzzing (that buzz is
// it straining against a stop -- back off immediately, it will cook the gears
// and brown out your rail). Mark that pulse as the limit. This sketch is built
// around exactly that procedure; see 'l' and 'u' below.
// ---------------------------------------------------------------------------

#include <Arduino.h>

// --- hardware map ----------------------------------------------------------
static const int PIN_PAN  = 1;   // D0
static const int PIN_TILT = 2;   // D1

// LEDC: timer 1 + channels 2/3. The esp_camera driver in src/main.cpp takes
// timer 0 / channel 0 for XCLK, so these stay out of its way when the two
// sketches eventually merge.
static const int CH_PAN  = 2;
static const int CH_TILT = 3;

static const int   SERVO_HZ      = 50;
// 14 is the ESP32-S3's ceiling: its LEDC timers are 14-bit, unlike the original
// ESP32's 20. Ask for more and ledcSetup() refuses outright and emits nothing,
// which looks exactly like a dead servo. 14 bits over a 20ms period still gives
// 1.22us per step -- comfortably finer than any hobby servo's deadband (5-10us).
static const int   DUTY_BITS     = 14;
static const uint32_t PERIOD_US  = 1000000UL / SERVO_HZ;

// Hard safety clamp. Nothing in this sketch will ever emit a pulse outside
// this window, no matter what you type. 500-2500 is the widest range any
// hobby servo tolerates; going beyond it just confuses the decoder.
static const uint16_t US_MIN = 500;
static const uint16_t US_MAX = 2500;
static const uint16_t US_MID = 1500;

// --- per-servo state -------------------------------------------------------
struct Servo {
  const char* name;
  int   pin;
  int   ch;
  bool  attached;
  float current;      // pulse actually being emitted, us (float: the ramp is smooth)
  float target;       // where we are ramping to, us
  uint16_t lowUs;     // measured endpoint, filled in by the 'l' command
  uint16_t highUs;    // measured endpoint, filled in by the 'u' command
  float lowDeg;       // protractor reading at lowUs  (see 'k')
  float highDeg;      // protractor reading at highUs
  bool  calibrated;
};

static Servo pan  = {"pan",  PIN_PAN,  CH_PAN,  false, US_MID, US_MID, 1000, 2000, 0, 0, false};
static Servo tilt = {"tilt", PIN_TILT, CH_TILT, false, US_MID, US_MID, 1000, 2000, 0, 0, false};
static Servo* sel = &pan;

// --- speed presets ---------------------------------------------------------
// These are *our* slew limits, not the servo's. The servo always moves at its
// own maximum; we simply refuse to move the setpoint faster than this, which
// is what makes a tracking rig look smooth instead of twitchy. INSTANT (0)
// removes the limit entirely and lets the servo show you its true top speed.
struct Speed { const char* name; float usPerSec; };
static const Speed SPEEDS[] = {
  {"instant", 0.0f},     // step -- servo goes as fast as it physically can
  {"slow",    250.0f},   // ~4s across a 1000us span
  {"medium",  800.0f},   // ~1.25s
  {"fast",   2200.0f},   // ~0.45s, still visibly ramped
};
static int speedIdx = 2;

static uint16_t nudgeStep = 25;

// ---------------------------------------------------------------------------
static uint32_t usToDuty(float us) {
  if (us < US_MIN) us = US_MIN;
  if (us > US_MAX) us = US_MAX;
  return (uint32_t)((us * (1UL << DUTY_BITS)) / PERIOD_US + 0.5f);
}

static void servoAttach(Servo& s) {
  if (s.attached) return;
  // ledcSetup returns the frequency it actually configured, or 0 if it could
  // not honour the request. Silently ignoring that is what cost an afternoon.
  double got = ledcSetup(s.ch, SERVO_HZ, DUTY_BITS);
  if (got == 0) {
    Serial.printf("[%s] ledcSetup FAILED (ch %d, %d bits) -- no pulses will be\n"
                  "       generated. Channel taken, or bit width over the S3's 14.\n",
                  s.name, s.ch, DUTY_BITS);
    return;
  }
  ledcAttachPin(s.pin, s.ch);
  ledcWrite(s.ch, usToDuty(s.current));
  s.attached = true;
  Serial.printf("[%s] attached on GPIO%d @ %.0fus (ch %d, %.0fHz, duty %u/%lu)\n",
                s.name, s.pin, s.current, s.ch, got,
                (unsigned)ledcRead(s.ch), (unsigned long)(1UL << DUTY_BITS));
}

// Detaching stops the pulse train. A servo with no signal goes limp and draws
// almost nothing -- the fastest way to tell a stalling servo from a dead one,
// and the right thing to do the moment you hear buzzing.
//
// The line is driven hard LOW rather than left as a floating input. A floating
// signal wire is an antenna: it picks up mains hum and the servo's own motor
// noise, decodes the garbage as pulses, and twitches. Held LOW it sees a clean
// "no signal" and stays still.
static void servoDetach(Servo& s) {
  if (s.attached) ledcDetachPin(s.pin);
  pinMode(s.pin, OUTPUT);
  digitalWrite(s.pin, LOW);
  s.attached = false;
  Serial.printf("[%s] detached (line held low, servo limp)\n", s.name);
}

static void writeUs(Servo& s, float us) {
  s.current = constrain(us, (float)US_MIN, (float)US_MAX);
  if (s.attached) ledcWrite(s.ch, usToDuty(s.current));
}

static void setTarget(Servo& s, float us) {
  s.target = constrain(us, (float)US_MIN, (float)US_MAX);
  if (SPEEDS[speedIdx].usPerSec == 0.0f) writeUs(s, s.target);
}

// Ramp engine. Called every loop; advances current toward target at the
// selected slew rate. Returns true while still moving.
static bool rampTick(Servo& s, uint32_t dtMicros) {
  if (s.current == s.target) return false;
  float rate = SPEEDS[speedIdx].usPerSec;
  if (rate == 0.0f) { writeUs(s, s.target); return false; }
  // One tick can never represent more than a frame's worth of time. Without
  // this, any blocking print or delay banks up dt and the "slow" ramp lurches.
  if (dtMicros > 20000) dtMicros = 20000;
  float step = rate * (dtMicros / 1000000.0f);
  float d = s.target - s.current;
  writeUs(s, (fabsf(d) <= step) ? s.target : s.current + (d > 0 ? step : -step));
  return true;
}

// us -> degrees, only meaningful once you have entered protractor readings.
static float usToDeg(const Servo& s, float us) {
  if (!s.calibrated || s.highUs == s.lowUs) return NAN;
  float f = (us - s.lowUs) / (float)(s.highUs - s.lowUs);
  return s.lowDeg + f * (s.highDeg - s.lowDeg);
}

// ---------------------------------------------------------------------------
static void report() {
  Serial.println(F("\n--------------------------------------------------"));
  Serial.printf("speed : %s (%.0f us/s)   nudge step: %uus   selected: %s\n",
                SPEEDS[speedIdx].name, SPEEDS[speedIdx].usPerSec, nudgeStep, sel->name);
  Servo* all[2] = {&pan, &tilt};
  for (Servo* s : all) {
    Serial.printf("[%-4s] GPIO%-2d %-8s now %6.1fus  limits %u..%uus  span %uus\n",
                  s->name, s->pin, s->attached ? "attached" : "DETACHED",
                  s->current, s->lowUs, s->highUs, s->highUs - s->lowUs);
    if (s->calibrated) {
      float total = s->highDeg - s->lowDeg;
      Serial.printf("        travel %.1fdeg over that span  ->  %.2f us/deg  |  now %.1fdeg\n",
                    total, (s->highUs - s->lowUs) / total, usToDeg(*s, s->current));
      for (int i = 1; i < 4; i++) {
        Serial.printf("        %-7s = %.0f deg/s\n", SPEEDS[i].name,
                      SPEEDS[i].usPerSec / ((s->highUs - s->lowUs) / total));
      }
    } else {
      Serial.println(F("        not calibrated -- run 'k <lowDeg> <highDeg>' after measuring"));
    }
  }
  Serial.println(F("--------------------------------------------------"));
}

static void help() {
  Serial.println(F(
    "\n=== servo bench ===================================================\n"
    "  p | t          select pan / tilt\n"
    "  <number>       move selected to that pulse in us (ramped)\n"
    "  + | -          nudge selected by the step size\n"
    "  = <us>         set nudge step (default 25)\n"
    "  c              center selected at 1500us\n"
    "  cc             center BOTH at 1500us\n"
    "  v0..v3         speed: 0 instant, 1 slow, 2 medium, 3 fast\n"
    "  l              mark current pulse as the LOW limit of selected\n"
    "  u              mark current pulse as the HIGH limit of selected\n"
    "  k <lo> <hi>    enter protractor degrees at those two limits\n"
    "  w [n]          sweep selected low<->high n times (default 3), timed\n"
    "  ww [n]         sweep both together\n"
    "  f              full-speed step low->high, timed (true max speed)\n"
    "  j              jitter test: hold still 5s, watch for hunting/buzz\n"
    "  x              PIN SELF-TEST: is this GPIO driving at all? (multimeter)\n"
    "  m              METER TEST: measure real pulse width as DC volts\n"
    "  d | a          detach / attach selected\n"
    "  z              PANIC: detach both immediately\n"
    "  r              report state\n"
    "  ?              this help\n"
    "\n"
    "FINDING THE REAL RANGE (the 90-vs-180 answer):\n"
    "  1) 'c' to center, confirm the horn sits mid-travel.\n"
    "  2) '-' repeatedly. Watch the horn AND listen. When it stops moving or\n"
    "     starts to buzz/strain, you are at the stop: press '+' once to back\n"
    "     off, then 'l' to record it.\n"
    "  3) '+' outward the same way, back off one step, then 'u'.\n"
    "  4) Put a protractor on the horn, note the angle at each limit, feed\n"
    "     both to 'k'. 'r' then prints the true travel and the deg/s for\n"
    "     each speed preset.\n"
    "  Any sweep or press aborts on the next key -- keep a hand on Enter.\n"
    "===================================================================\n"));
}

// Sweeps between the recorded limits, honouring the current speed preset, and
// times it. Abort by sending any character.
static bool sweep(Servo** list, int n, int cycles) {
  for (int i = 0; i < n; i++) {
    if (list[i]->highUs <= list[i]->lowUs) {
      Serial.printf("[%s] limits not set sensibly (%u..%u)\n",
                    list[i]->name, list[i]->lowUs, list[i]->highUs);
      return false;
    }
    servoAttach(*list[i]);
  }
  Serial.printf("sweeping %d cycle(s) at %s -- send any key to abort\n",
                cycles, SPEEDS[speedIdx].name);

  uint32_t last = micros();
  for (int c = 0; c < cycles; c++) {
    for (int leg = 0; leg < 2; leg++) {
      for (int i = 0; i < n; i++)
        setTarget(*list[i], leg == 0 ? list[i]->highUs : list[i]->lowUs);

      uint32_t t0 = millis();
      last = micros();          // discard the gap spent printing / in delay()
      bool moving = true;
      while (moving) {
        uint32_t now = micros();
        uint32_t dt = now - last;
        last = now;
        moving = false;
        for (int i = 0; i < n; i++) moving |= rampTick(*list[i], dt);
        if (Serial.available()) {
          while (Serial.available()) Serial.read();
          Serial.println(F("aborted"));
          return false;
        }
        delayMicroseconds(500);
      }
      // With speed 'instant' the setpoint jumps in one tick, so this number is
      // the command time, not the mechanical time -- the servo is still moving
      // after it prints. Give it a moment before the return leg.
      Serial.printf("  cycle %d %s leg: %lums commanded\n",
                    c + 1, leg == 0 ? "out " : "back", (unsigned long)(millis() - t0));
      if (SPEEDS[speedIdx].usPerSec == 0.0f) delay(600);
      else delay(150);
    }
  }
  Serial.println(F("sweep done"));
  return true;
}

static void fullSpeedStep() {
  if (sel->highUs <= sel->lowUs) { Serial.println(F("set limits first")); return; }
  servoAttach(*sel);
  int saved = speedIdx;
  speedIdx = 0;
  Serial.printf("[%s] parking at %uus...\n", sel->name, sel->lowUs);
  setTarget(*sel, sel->lowUs);
  delay(1200);
  Serial.printf("[%s] STEP -> %uus now. Time the horn (phone slow-mo works well).\n",
                sel->name, sel->highUs);
  uint32_t t0 = millis();
  setTarget(*sel, sel->highUs);
  delay(1500);
  Serial.printf("held for %lums. A GH-S37D-class micro servo should have\n"
                "settled well inside that; if it was still crawling, your 5V\n"
                "rail is sagging under load.\n", (unsigned long)(millis() - t0));
  speedIdx = saved;
}

// Takes the servo protocol out of the picture entirely and just asks whether
// the pin moves. Set a multimeter to DC volts, black probe on GND, red probe on
// the servo's signal wire AT THE SERVO END -- probing there tests the jumper and
// the breadboard row too, which is exactly where these faults live.
static void pinSelfTest() {
  servoDetach(*sel);
  Serial.printf("\n[%s] self-test on GPIO%d. Multimeter in DC volts, black on GND,\n"
                "red on the SIGNAL wire where it enters the servo plug.\n\n",
                sel->name, sel->pin);

  pinMode(sel->pin, OUTPUT);
  digitalWrite(sel->pin, LOW);
  Serial.println(F("  step 1/3: driving LOW for 5s   -> expect 0.0V"));
  delay(5000);

  digitalWrite(sel->pin, HIGH);
  Serial.println(F("  step 2/3: driving HIGH for 5s  -> expect ~3.3V"));
  delay(5000);

  digitalWrite(sel->pin, LOW);
  if (ledcSetup(sel->ch, SERVO_HZ, DUTY_BITS) == 0) {
    Serial.println(F("  ledcSetup failed -- cannot run step 3"));
    return;
  }
  ledcAttachPin(sel->pin, sel->ch);
  ledcWrite(sel->ch, (1UL << DUTY_BITS) / 2);
  Serial.println(F("  step 3/3: 50% PWM for 5s       -> expect ~1.65V"));
  delay(5000);

  Serial.println(F(
    "\nReading it:\n"
    "  all three correct  -> the pin is fine; the fault is the servo, its plug,\n"
    "                        or the signal wire's colour (signal is usually the\n"
    "                        pale one: orange/yellow/white, NEVER red or brown).\n"
    "  all three ~0V      -> that wire is not on this pin. On the XIAO, D0 is the\n"
    "                        top-left pad next to the USB port and D1 is directly\n"
    "                        below it. Touching a wire makes a servo twitch even\n"
    "                        when the wire connects to nothing, so the twitch you\n"
    "                        saw is not proof the pin is right.\n"
    "  drifting / floaty  -> the wire is loose in the breadboard row.\n"));

  servoDetach(*sel);
}

// A multimeter on DC volts averages the pulse train, which means the reading is
// a direct measurement of pulse width: avg = 3.3V * width / 20000us. So each
// setpoint has its own predictable voltage, and the meter becomes a crude
// oscilloscope. If these readings track, the ESP32 side is proven correct and
// the fault is downstream, in the servo or its connector.
static void meterTest() {
  servoAttach(*sel);
  static const uint16_t PTS[] = {500, 1000, 1500, 2000, 2500};

  Serial.printf("\n[%s] pulse-width check on GPIO%d. DC volts, black on GND, red on\n"
                "the signal wire at the servo plug. 6s per step:\n\n", sel->name, sel->pin);

  for (uint8_t i = 0; i < sizeof(PTS) / sizeof(PTS[0]); i++) {
    // Straight to the pin -- the ramp would smear the setpoint across the hold.
    sel->target = PTS[i];
    writeUs(*sel, PTS[i]);
    delay(50);
    Serial.printf("  %4uus  duty reg %5u/%5lu  -> expect %.2fV\n",
                  PTS[i], (unsigned)ledcRead(sel->ch),
                  (unsigned long)(1UL << DUTY_BITS),
                  3.3f * PTS[i] / (float)PERIOD_US);
    delay(6000);
  }
  writeUs(*sel, US_MID);
  sel->target = US_MID;

  Serial.println(F(
    "\nReading it:\n"
    "  voltages track (0.08 / 0.17 / 0.25 / 0.33 / 0.41V)\n"
    "      -> the pulse train is correct and the ESP32 has done its job. The\n"
    "         fault is the servo end. See the connector note below.\n"
    "  stuck near 1.65V at every step\n"
    "      -> the duty register really is jammed at 50%; that is a firmware bug\n"
    "         and the reg numbers above will say so too.\n"
    "  all ~0V -> the pin stopped driving between the self-test and now.\n"));
}

static void jitterTest() {
  servoAttach(*sel);
  Serial.printf("[%s] holding %.0fus for 5s -- listen. Continuous buzzing means\n"
                "it is fighting a mechanical stop or hunting on a noisy rail.\n",
                sel->name, sel->current);
  delay(5000);
  Serial.println(F("done"));
}

// ---------------------------------------------------------------------------
static void handleLine(char* line) {
  while (*line == ' ') line++;
  if (!*line) return;
  // The help text writes arguments as <lo> <hi>; typing the brackets literally
  // is an easy mistake, so just treat them as whitespace.
  for (char* q = line; *q; q++) if (*q == '<' || *q == '>') *q = ' ';

  // Bare number: move the selected servo there.
  if (isdigit((unsigned char)line[0])) {
    long us = atol(line);
    if (us < US_MIN || us > US_MAX) {
      Serial.printf("out of the %u..%uus safety window\n", US_MIN, US_MAX);
      return;
    }
    servoAttach(*sel);
    setTarget(*sel, (float)us);
    Serial.printf("[%s] -> %ldus\n", sel->name, us);
    return;
  }

  char c = tolower((unsigned char)line[0]);
  char c1 = tolower((unsigned char)line[1]);
  char* arg = line + 1;

  if (c == 'c' && c1 == 'c') {
    servoAttach(pan); servoAttach(tilt);
    setTarget(pan, US_MID); setTarget(tilt, US_MID);
    Serial.println(F("both -> 1500us"));
    return;
  }
  if (c == 'w' && c1 == 'w') {
    int n = atoi(line + 2); if (n <= 0) n = 3;
    Servo* both[2] = {&pan, &tilt};
    sweep(both, 2, n);
    return;
  }

  switch (c) {
    case '?': help(); break;
    case 'r': report(); break;
    case 'p': sel = &pan;  Serial.println(F("selected pan"));  break;
    case 't': sel = &tilt; Serial.println(F("selected tilt")); break;

    case '+': case '-': {
      servoAttach(*sel);
      setTarget(*sel, sel->target + (c == '+' ? nudgeStep : -nudgeStep));
      Serial.printf("[%s] -> %.0fus\n", sel->name, sel->target);
      break;
    }
    case '=': {
      int n = atoi(arg);
      if (n > 0 && n <= 200) { nudgeStep = n; Serial.printf("nudge step %uus\n", nudgeStep); }
      else Serial.println(F("step must be 1..200"));
      break;
    }
    case 'c': servoAttach(*sel); setTarget(*sel, US_MID);
              Serial.printf("[%s] -> 1500us\n", sel->name); break;

    case 'v': {
      int n = atoi(arg);
      if (n >= 0 && n <= 3) { speedIdx = n; Serial.printf("speed: %s\n", SPEEDS[n].name); }
      else Serial.println(F("v0..v3"));
      break;
    }
    case 'l': sel->lowUs = (uint16_t)lroundf(sel->current);
              Serial.printf("[%s] LOW limit = %uus\n", sel->name, sel->lowUs); break;
    case 'u': sel->highUs = (uint16_t)lroundf(sel->current);
              Serial.printf("[%s] HIGH limit = %uus\n", sel->name, sel->highUs); break;

    case 'k': {
      // Deliberately integer-only: this newlib build does not convert floats
      // in strtof/sscanf("%f") -- both silently refuse. strtol always works,
      // and whole degrees off a protractor is all the precision there is.
      char* end1 = NULL; char* end2 = NULL;
      long lo = strtol(arg, &end1, 10);
      long hi = (end1 != arg) ? strtol(end1, &end2, 10) : 0;
      if (end1 != arg && end2 != end1 && lo != hi) {
        sel->lowDeg = (float)lo; sel->highDeg = (float)hi; sel->calibrated = true;
        Serial.printf("[%s] calibrated: %uus=%lddeg, %uus=%lddeg\n",
                      sel->name, sel->lowUs, lo, sel->highUs, hi);
        report();
      } else Serial.println(F("usage: k <degAtLow> <degAtHigh>  whole degrees, e.g. k 0 175"));
      break;
    }
    case 'w': { int n = atoi(arg); if (n <= 0) n = 3; Servo* one[1] = {sel}; sweep(one, 1, n); break; }
    case 'f': fullSpeedStep(); break;
    case 'j': jitterTest(); break;
    case 'x': pinSelfTest(); break;
    case 'm': meterTest(); break;
    case 'd': servoDetach(*sel); break;
    case 'a': servoAttach(*sel); break;
    case 'z': servoDetach(pan); servoDetach(tilt); Serial.println(F("PANIC: both limp")); break;
    default:  Serial.printf("unknown '%c' -- '?' for help\n", c);
  }
}

// ---------------------------------------------------------------------------
void setup() {
  // First thing, before anything slow: park both signal lines low. Between
  // reset and here the pins are floating inputs, which is what makes servos
  // dance during an upload. This shortens that window to a few milliseconds;
  // a 10k resistor from each signal pin to GND closes it completely.
  pinMode(PIN_PAN, OUTPUT);  digitalWrite(PIN_PAN, LOW);
  pinMode(PIN_TILT, OUTPUT); digitalWrite(PIN_TILT, LOW);

  Serial.begin(115200);
  uint32_t t0 = millis();
  while (!Serial && millis() - t0 < 3000) delay(10);   // USB CDC needs a moment
  delay(300);

  Serial.println(F("\n\nXIAO ESP32S3 servo bench"));
  Serial.println(F("Servos start DETACHED. Check your wiring and 5V rail, then"));
  Serial.println(F("press 'a' (selected) or 'cc' (both, centered) to energise."));
  help();
  report();
  Serial.print(F("\n> "));
}

void loop() {
  static char buf[64];
  static uint8_t len = 0;
  static uint32_t last = micros();

  uint32_t now = micros();
  uint32_t dt = now - last;
  last = now;
  rampTick(pan, dt);
  rampTick(tilt, dt);

  while (Serial.available()) {
    char ch = Serial.read();
    if (ch == '\r') continue;
    if (ch == '\n') { buf[len] = 0; handleLine(buf); len = 0; Serial.print(F("\n> ")); }
    else if (len < sizeof(buf) - 1) buf[len++] = ch;
  }
}
