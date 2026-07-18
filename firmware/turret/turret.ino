/*
 * CV Sentry Turret - ESP32 firmware (Phase 1 aim + gel-blaster fire)
 *
 * Receives aim commands from the Mac/Pi brain (turret_brain.py --serial) over USB and
 * drives the pan/tilt servos, slew-limited so motion is smooth. FIRE pulls the gel
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

const int PAN_SLEW_STEP  = 2;   // max degrees moved per control tick (smoothness)
const int TILT_SLEW_STEP = 1;   // tilt moves GENTLER than pan (loose tilt joint, missing a screw)
const int TICK_MS   = 15;       // ~66 Hz control loop

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

void slew(float &cur, int tgt, Servo &s, int stepMax) {
  if (cur < tgt)      cur += min((float)stepMax, (float)tgt - cur);
  else if (cur > tgt) cur -= min((float)stepMax, cur - (float)tgt);
  s.write((int)cur);
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
    slew(panCur, panTgt, panServo, PAN_SLEW_STEP);
    slew(tiltCur, tiltTgt, tiltServo, TILT_SLEW_STEP);
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
