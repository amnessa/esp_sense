# esp_sense — face-tracking pan/tilt head

XIAO ESP32S3 Sense (OV2640 + on-chip face detection) driving a two-axis servo
head that keeps a detected face centred in frame.

| env | builds | what it is |
|---|---|---|
| `seeed_xiao_esp32s3` | `src/` | camera, MJPEG stream, face detector, web overlay |
| `servo_test` | `servo_test/` | standalone servo bench — range, speed, wiring diagnostics |

The two never compile together; `default_envs` keeps plain `pio run` on the
camera project. The bench is opt-in:

```
pio run -e servo_test -t upload -t monitor
```

---

## Coordinate frame

Right-handed, ROS REP-103 convention, origin at the camera:

```
            z  up
            |
            |            x = the camera normal: straight out of the lens.
            |                This is the direction the head points.
            o------ y  left
           /
          /
         x  forward (out of the lens, toward the face)
```

| axis | direction | rotation about it | servo | pin |
|---|---|---|---|---|
| x | out of the lens | roll — *not actuated* | — | — |
| y | camera's left | **pitch / tilt** (nose up-down) | tilt | D1 / GPIO2 |
| z | up | **yaw / pan** (turn left-right) | pan | D0 / GPIO1 |

Positive yaw turns the camera **left**. Positive pitch noses it **down**.

`DetectionState` reports the face offset already normalised to this frame:
`err_x` is +1 at the right edge of the image, `err_y` is +1 at the bottom.
So a face at positive `err_x` needs *negative* yaw to centre it, and a face at
positive `err_y` needs *positive* pitch. Whether that matches your servo's
rotation direction depends on which way the horn is bolted on — check it once
with the bench (`p`, then `1300`, then `1700`, and watch) and flip the gain
sign rather than rewiring.

---

## Wiring

Servos are powered from their **own 5V supply**. The ESP32 is powered from USB.

```
   5V PSU  +5V  ─────────────┬──────────────┐
                             │              │
                          [pan servo]   [tilt servo]
                           red           red
                             │              │
   5V PSU  GND  ─────┬───────┴──────────────┘     (brown/black)
                     │
                     └────────  XIAO GND  pin      <-- COMMON GROUND
                                                       not optional

   XIAO D0 (GPIO1) ────┬──── pan  signal   (orange/yellow/white)
                     [10k]
                       └──── GND  (optional)

   XIAO D1 (GPIO2) ────┬──── tilt signal
                     [10k]
                       └──── GND (optional)
```

**The common ground is the whole ballgame.** A servo pulse is a voltage
*relative to the servo's ground*. If the ESP32's ground and the PSU's ground
are not bonded, there is no shared reference, the servo sees noise instead of
a pulse, and it buzzes and twitches without ever positioning. Every voltage
you can measure will look correct while this is wrong, because a multimeter
measures relative to whichever ground you clip onto.

**Do not also bridge the PSU +5V to the XIAO's 5V pad while USB is plugged
in.** That ties your PSU against the host's USB rail. Ground is the only
connection the two supplies need. If you later want to run untethered, remove
USB first, then feed the 5V pad.

Other notes:

- **470–1000 µF** across the servo rail, physically near the connectors. Micro
  servos pull large inrush currents; breadboard contact resistance turns that
  into a voltage sag that resets the servo's decoder mid-move.
- **10k from each signal pin to GND.** Between reset and the first line of
  `setup()`, and throughout every upload, those pins are floating inputs — and
  a floating servo signal wire is an antenna. The resistors hold the line down
  when nothing is driving it.
- Servo plug order is GND (brown) · **+5V (red, centre)** · signal (pale).
  Reversed, the centre pin is still +5V, so nothing burns out and the fault
  hides: the servo's ground pin gets the PWM and it buzzes without moving.
  Check by metering the servo's brown wire against the PSU negative — it must
  read 0.00 V.

### Why these pins

The camera occupies GPIO 10–18, 38–40, 47, 48. D0/D1 are free, are not
strapping pins, and are not shared with the Sense expansion board. The servo
PWM uses **LEDC timer 1, channels 2/3**; the camera driver claims timer 0 /
channel 0 for XCLK (`src/main.cpp`), so the two coexist.

The ESP32-S3's LEDC timers are **14-bit** — unlike the original ESP32's 20.
`ledcSetup()` refuses a larger width outright and emits nothing at all, which
looks exactly like a dead servo. 14 bits over a 20 ms period is 1.22 µs per
step, finer than any hobby servo's 5–10 µs deadband.

---

## Servo calibration

Run the bench and record the results here. `?` lists commands.

1. `p` (or `t`) to select, `c` to centre, confirm the horn sits mid-travel.
2. `-` repeatedly. When it stops moving **or starts to buzz**, that is the
   mechanical stop — back off one step with `+`, then `l` to mark it.
3. `+` outward the same way, back off, `u` to mark.
4. Protractor on the horn, read the angle at each limit, then `k <lo> <hi>`
   in whole degrees. `r` prints the true travel and the deg/s of each speed.

Diagnostics if something is off: `x` proves the GPIO drives at all (0 V /
3.3 V / 1.65 V), and `m` measures the real pulse width as a DC average
(3.3 V × width ÷ 20000 µs), turning a multimeter into a crude scope.

**Measured (fill in):**

| | low µs | high µs | travel ° | µs/° |
|---|---|---|---|---|
| pan (D0) | | | | |
| tilt (D1) | | | | |

Pull both endpoints in by 50–100 µs before using them in the tracker. A
controller that commands into a mechanical stop will sit there buzzing
indefinitely.

> On "is it 90° or 180°?": neither is a property of the servo. The usual
> 1000–2000 µs range only covers ~90–120°; the same servo keeps going out
> toward ~180° as the pulse approaches 500/2500 µs, until the internal
> potentiometer hits its stop. It is measured, not looked up.

---

## Control: why there is no kinematics here

The camera sits at the **end** of the kinematic chain and the error is measured
in the camera's own frame. Yaw moves the face horizontally in the image; pitch
moves it vertically. The Jacobian relating joint angles to image error is
diagonal, so there is nothing to invert — each axis needs exactly one scalar:
**how much image error does one microsecond of servo travel buy?**

Don't derive that from the lens FOV. Measure it, and it absorbs the FOV, the
gear ratio and the µs/degree all at once:

1. Point the head at a stationary face and let it settle.
2. Note `err_x`, step pan by a known amount (say +100 µs), note `err_x` again.
3. `S_pan = Δerr_x / Δµs`, in error-units per microsecond. Repeat for tilt.

### The loop

```cpp
// Commands a *velocity* proportional to error, integrated into position.
// This is the standard visual-servoing form: because the camera moves with
// the servo, position feeds back through the error itself, so a position
// increment proportional to error gives a stable first-order convergence.
//
// KP == 1.0 with a perfect S would centre the face in a single step. Use
// 0.3-0.5: the detector's box lags the video by one inference, and a gain
// that assumes zero latency will hunt.
static const float KP        = 0.4f;
static const float DEADBAND  = 0.04f;   // ignore <4% error: stops idle dither
static const uint32_t LOST_MS = 800;    // give up on a stale track

void tracker_tick() {
  DetectionState d;
  detector_get(&d);

  // err_x/err_y are held at their last good values when a face is lost, so
  // this coasts rather than snapping to centre. age_ms is what decides.
  if (d.age_ms > LOST_MS) return;

  float ex = fabsf(d.err_x) < DEADBAND ? 0.0f : d.err_x;
  float ey = fabsf(d.err_y) < DEADBAND ? 0.0f : d.err_y;

  panUs  = constrain(panUs  - KP * ex / S_PAN,  PAN_LO,  PAN_HI);
  tiltUs = constrain(tiltUs + KP * ey / S_TILT, TILT_LO, TILT_HI);
}
```

Signs assume positive yaw turns left and positive pitch noses down. If the
head runs *away* from the face, flip the gain sign for that axis — the servo
horn is mounted the other way round.

### What to watch for

- **Rate-limit the setpoint.** Feed the output through the bench's ramp
  (`v1`/`v2`/`v3` speeds) rather than jumping. A servo slewing at full speed
  shakes the camera enough to wreck the next detection, so the loop fights
  itself.
- **A deadband is not optional.** Without it the head dithers on detector
  noise even with a perfectly still face, and dither is what burns out micro
  servos.
- **P is enough; don't reach for I.** A face is a moving target, and integral
  action on a moving target winds up and overshoots. If there's steady-state
  droop, raise KP.
- **Coupling is negligible here.** Pan's effect on the image scales with
  cos(tilt) — 13% at 30° of tilt, far inside what a P controller absorbs.
  Divide the pan gain by cos(tilt) only if you go to steep angles.
