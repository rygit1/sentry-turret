/*
 * CV Sentry Turret - ESP32 firmware (Phase 1 aim + gel-blaster fire)
 *
 * Receives aim commands from the Mac/Pi brain (turret_brain.py --serial) over USB and
 * drives the pan/tilt servos with a paced sub-degree glide so motion is one smooth
 * sweep instead of per-command hops. FIRE pulls the gel
 * blaster's OWN trigger with a third servo and holds it for a short burst; the blaster
 * runs its own flywheels off its own battery, so this build has NO flywheel MOSFET and
 * NO dart pusher. A PHYSICAL arm switch gates all firing in hardware as well as software.
 *
 * Protocol (one line per update, newline-terminated):
 *   P094 T081        -> pan = 94 deg, tilt = 81 deg
 *   P094 T081 FIRE   -> same, plus fire a burst (only if physically armed)
 *
 * Wiring (ESP32 dev board):
 *   PAN servo  signal -> GPIO13      TILT servo signal -> GPIO14
 *   TRIGGER servo sig -> GPIO27      ARM switch -> GPIO33 to GND (INPUT_PULLUP)
 *   Servo V+ (red)    -> EXTERNAL 5V supply (NOT the ESP32 5V pin)
 *   Servo GND (brown) -> external 5V GND, tied to ESP32 GND (COMMON GROUND)
 *   1000uF cap across the external 5V rail, close to the servos.
 *   ARM switch closed (to GND) = ARMED.
 *
 * Servos are DS3218 270-degree: attached with a 500-2500us pulse for full travel.
 * Arduino UNO instead of ESP32: include <Servo.h>, drop the ESP32PWM/allocateTimer
 * lines, use pins PAN=9 TILT=10 TRIGGER=5 ARM=2; the parser is identical.
 */

#include <ESP32Servo.h>

const int PAN_PIN     = 13;
const int TILT_PIN    = 14;     // GPIO14 (GPIO12 is a boot-strapping pin, do not use it)
const int TRIGGER_PIN = 27;
const int ARM_PIN     = 33;

const int PAN_MIN = 0,  PAN_MAX = 180;    // chip allows FULL range (manual jog is unrestricted)
const int TILT_MIN = 0, TILT_MAX = 180;   // safe crash-limits (tilt 60-85, pan 38-170) live in the BRAIN's auto loop only

// Motion: the brain sends a fresh target about every camera frame (50-80 ms apart). The old code
// hopped to each target at full servo speed in whole-degree steps, then WAITED for the next
// command: hop-wait-hop-wait = visible stutter. Now each new target is GLIDED to over ~GLIDE_MS
// (a hair over one camera frame) in fine sub-degree steps at a fast tick, so the head is still
// moving when the next command lands = one continuous glide at the commanded average speed.
const int   TICK_MS     = 5;                       // 200 Hz motion tick (was 15 ms / 66 Hz)
const int   GLIDE_MS    = 70;                      // spread each brain hop across about one camera frame
const int   GLIDE_TICKS = GLIDE_MS / TICK_MS;
const float PAN_TICK_MAX  = 133.0 * TICK_MS / 1000.0;  // speed ceiling per tick (133 deg/s, matches --slew-rate)
const float TILT_TICK_MAX =  66.0 * TICK_MS / 1000.0;  // tilt stays GENTLER (loose tilt joint, missing a screw)

// Trigger servo: REST = not touching the trigger, PULL = trigger fully squeezed.
// CALIBRATE both to your linkage in Step 6 (rotate by hand to find the two angles).
const int TRIG_REST = 142;   // arm just clear of the trigger (measured on the real linkage 7/12)
const int TRIG_PULL = 127;   // trigger squeezed all the way in (PULL sits BELOW rest on this linkage)
const int BURST_MS  = 400;      // hold time per FIRE = one short full-auto burst (400 lets the flywheels reach full speed so each burst is a steady ~3, not the 2/1/2/1 alternation of a 250ms squeeze)

const unsigned long FIRE_COOLDOWN_MS = 800;
const unsigned long FAILSAFE_MS      = 1500;  // no serial this long -> hold, never fire

Servo panServo, tiltServo, triggerServo;

float panCur = 90, tiltCur = 72;   // boot to a tilt inside [60,85] so it never slams the cap on reset
int   panTgt = 90, tiltTgt = 72;
float panVel = 0, tiltVel = 0;     // glide speed (deg per tick), recomputed on every aim command
bool  fireReq = false;
unsigned long lastTick = 0, lastFire = 0, lastRx = 0;
String line;

bool isArmed() { return digitalRead(ARM_PIN) == LOW; }  // switch to GND = armed

void setup() {
  Serial.begin(115200);
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  panServo.setPeriodHertz(50);     panServo.attach(PAN_PIN, 500, 2500);
  tiltServo.setPeriodHertz(50);    tiltServo.attach(TILT_PIN, 500, 2500);
  triggerServo.setPeriodHertz(50); triggerServo.attach(TRIGGER_PIN, 500, 2500);
  pinMode(ARM_PIN, INPUT_PULLUP);
  panServo.write((int)panCur);
  tiltServo.write((int)tiltCur);
  triggerServo.write(TRIG_REST);
  lastRx = millis();
  Serial.println("TURRET DEBUG BUILD ready (prints why each FIRE does/doesn't fire)");
}

void parseLine(const String &s) {
  int p = s.indexOf('P');
  int t = s.indexOf('T');
  if (p >= 0 && t > p) {
    panTgt  = constrain(s.substring(p + 1, p + 4).toInt(), PAN_MIN, PAN_MAX);
    tiltTgt = constrain(s.substring(t + 1, t + 4).toInt(), TILT_MIN, TILT_MAX);
    panVel  = (panTgt  - panCur)  / (float)GLIDE_TICKS;   // pace the move to land with the next command
    tiltVel = (tiltTgt - tiltCur) / (float)GLIDE_TICKS;
    lastRx = millis();
  }
  if (s.indexOf("FIRE") >= 0) fireReq = true;

  int g = s.indexOf('G');   // G### = set trigger servo angle directly (calibration; ungated, no gel)
  if (g >= 0) {
    int ang = constrain(s.substring(g + 1, g + 4).toInt(), 0, 180);
    triggerServo.write(ang);
    lastRx = millis();
    Serial.print("TRIG set to "); Serial.println(ang);
  }
}

void fireBurst() {
  triggerServo.write(TRIG_PULL);   // squeeze the blaster's trigger (it runs its own flywheels)
  delay(BURST_MS);                 // hold for one short full-auto burst
  triggerServo.write(TRIG_REST);   // release
}

void glide(float &cur, int tgt, float &vel, Servo &s, float tickMax) {
  float rem = (float)tgt - cur;
  float step = vel;
  if ((rem > 0 && step <= 0) || (rem < 0 && step >= 0))   // stale/zero pace but not there yet:
    step = (rem > 0) ? tickMax : -tickMax;                // fall back to the speed ceiling
  if (step >  tickMax) step =  tickMax;                   // hardware speed ceiling per axis
  if (step < -tickMax) step = -tickMax;
  if ((rem >= 0 && step >= rem) || (rem <= 0 && step <= rem)) { cur = tgt; vel = 0; }  // arrive, never pass
  else cur += step;
  // sub-degree writes: attach(500,2500) maps 0-180 deg over 2000 us, so 1 us = 0.09 deg.
  // plain write() rounds to whole degrees, which reads as tiny clicks at glide speeds.
  s.writeMicroseconds(500 + (int)(cur * (2000.0 / 180.0) + 0.5));
}

void loop() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n')                         { parseLine(line); line = ""; }
    else if (c != '\r' && line.length() < 32) line += c;
  }

  unsigned long now = millis();
  if (now - lastTick >= TICK_MS) {
    lastTick = now;
    glide(panCur, panTgt, panVel, panServo, PAN_TICK_MAX);
    glide(tiltCur, tiltTgt, tiltVel, tiltServo, TILT_TICK_MAX);
  }

  if (fireReq) {
    fireReq = false;
    bool linkAlive = (now - lastRx) < FAILSAFE_MS;
    bool armed = isArmed();
    bool coolOk = (now - lastFire) > FIRE_COOLDOWN_MS;
    Serial.print("FIRE rx | armed="); Serial.print(armed);
    Serial.print(" link="); Serial.print(linkAlive);
    Serial.print(" cooldownOk="); Serial.println(coolOk);
    if (armed && linkAlive && coolOk) {
      fireBurst();
      lastFire = millis();
      Serial.println("  -> FIRING");
    }
  }
}
