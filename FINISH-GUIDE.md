# 🔫 SENTRY TURRET, FINISH GUIDE (everything from here to done)

Written 2026-07-12. Work top to bottom. You've got this, the hard parts are behind you.

**EVERY command below assumes you FIRST run this (fresh terminals start in your home folder):**
```
cd ~/sentry-turret
```
If a command ever says `no such file or directory: .venv/bin/python`, you forgot that line. Just run it and try again.

---

## ⚠️ 0. SAFETY, read once, live by it (the gun is real now)
1. **Goggles ON** any time gel is loaded. They're in the Mythic box. Gel stings, eyes are eyes.
2. **Never aim at a face.** Body only. **Never at anyone who didn't say yes.**
3. **Disarm when you're not actively testing** = flip the toggle OFF, or unplug the pin-33 jumper (details in Step 1). A disarmed turret physically cannot fire.
4. **Test on a cardboard box first**, never a person, until you trust it.
5. Start every session with the gun's own **orange safety LOCKED** and no gel, prove the motion, THEN load gel.

---

## 1. WHERE YOU ARE  ✅ = done
- ✅ Pan, tilt, trigger servos mounted + wired
- ✅ Trigger calibrated: **REST 142, PULL 127** (baked into the firmware)
- ✅ Trigger dry-fires (armed, arm swings, snaps back)
- ✅ Brain software done (tracking + face recognition + auto-fire logic)
- ⬜ **Step 1:** arm toggle switch
- ⬜ **Step 2:** strain-relief the wires (stop them popping out)
- ⬜ **Step 3:** hydrate gel (start 4 hrs early!)
- ⬜ **Step 4:** live-fire test on a target
- ⬜ **Step 5:** zero the aim
- ⬜ **Step 6:** run the full auto-aim system

---

## 2. 📋 COMMAND CHEAT SHEET (copy-paste)
Every one needs `cd ~/sentry-turret` first (or paste the whole line).

**Flash the chip** (after any firmware change; close other programs first):
```
cd ~/sentry-turret && arduino-cli compile --upload -p /dev/cu.usbserial-0001 --fqbn esp32:esp32:esp32 firmware/turret/turret.ino
```

**Jog all 3 servos by hand** (arrows = pan/tilt, A/D = trigger):
```
cd ~/sentry-turret && .venv/bin/python trigger_jog.py
```

**Dry-fire test the trigger** (ENTER = one pull, shows the chip's readout):
```
cd ~/sentry-turret && .venv/bin/python trigger_test.py
```

**Find which camera number is the eMeet** (look at the /tmp/cam_N.jpg it saves):
```
cd ~/sentry-turret && .venv/bin/python camera_probe.py
```

**RUN THE FULL SYSTEM, solo test, tracks only YOU:**
```
cd ~/sentry-turret && .venv/bin/python turret_brain.py --serial /dev/cu.usbserial-0001 --camera 0 --lock-me
```

**RUN THE FULL SYSTEM, guard mode, targets everyone EXCEPT you:**
```
cd ~/sentry-turret && .venv/bin/python turret_brain.py --serial /dev/cu.usbserial-0001 --camera 0 --target-others
```

**Add more face photos of you** (do this from your own Terminal, then add `--reenroll` to ONE run after):
```
cd ~/sentry-turret && .venv/bin/python enroll_capture.py --camera 0
```

⚠️ **Always run `turret_brain.py` and `enroll_capture.py` from YOUR OWN Terminal app** (not through me). First run pops a "Terminal wants the camera" box → click **Allow**.

---

## 3. 🔌 WIRING REFERENCE (if any wire pops out, plug it back like this)

**Golden rules:** servo power/ground go to the edge rails fed by the 5V brick, NEVER the chip. Red (+) and blue (−) rails must never touch. Go by LOCATION, not color (colors repeat).

**PAN servo:**
| servo hole | wire | other end |
|---|---|---|
| WHITE (signal) | BLUE | chip pin **13** |
| RED (power) | ORANGE spare | red (+) rail |
| BLACK (ground) | YELLOW spare | blue (−) rail |

**TILT servo:**
| servo hole | wire | other end |
|---|---|---|
| WHITE (signal) | GREEN | chip pin **14** |
| RED (power) | ORANGE spare | red (+) rail |
| BLACK (ground) | GRAY spare | blue (−) rail |

**TRIGGER servo:**
| servo hole | wire | other end |
|---|---|---|
| WHITE (signal) | RED (longer) | chip pin **27** (next to green) |
| RED (power) | GREEN | red (+) rail |
| BLACK (ground) | BLUE | blue (−) rail |

**ARM (the fire safety):** chip pin **33** → blue (−) rail (through the toggle, Step 1).
Remember: `armed=0` in the readout = pin 33 isn't reaching ground.

**Power:** 5V brick → barrel adapter (LEFT screw = +, RIGHT screw = −) → red jumper to red (+) rail, blue jumper to blue (−) rail. 1000uF cap across the rails (long leg → red, short/striped leg → blue). Chip GND (orange) → blue (−) rail = common ground.

⚠️ **Always unplug power (brick from wall + USB) before reseating any power/ground wire.** Swapping a power and ground can cook a servo.

---

## 4. 🛑 STEP 1, WIRE THE ARM TOGGLE

The toggle just breaks the pin-33 → ground wire. ON = grounded = can fire. OFF = floating = chip refuses to fire.

**No soldering needed.** Your blue toggle has **2 metal legs** (a simple on/off):
1. Jumper from **chip pin 33 → one leg.**
2. Jumper from **the other leg → the blue (−) rail.**
3. One flip connects the two legs (grounded = armed), the other flip breaks the connection (safe). The `trigger_test.py` readout confirms which way: whichever position shows `armed=1` is your ARMED side.
4. No-solder connection: hook the bare wire tip through each leg's little hole and twist it tight, or press and tape it firm. Tug-test it, it must not fall off. (The threaded barrel + hex nut are just for mounting it through a hole later, ignore them for now.)

Now: **toggle ON → armed. Toggle OFF → safe.** Test it: run `trigger_test.py`, press ENTER with the toggle OFF (readout says `armed=0`, no fire), flip it ON, press ENTER (`armed=1`, arm swings). 

**Dead-simple fallback if the toggle is fussy:** skip it and just **unplug the pin-33 jumper from the rail to disarm, plug it in to arm.** Not as slick, but it's a real physical safety and takes zero parts.

---

## 5. 🧷 STEP 2, STRAIN-RELIEF (this is why wires keep popping)

Every time the turret swings, tight wires get yanked out. Fix it once:
1. Give each servo's 3 wires a little **slack loop** (a finger's worth of extra) so motion doesn't pull them.
2. **Tape or zip-tie the wire bundle down** to the wood bed near each servo, so the pull happens on the tape, not the plug.
3. Route the wires along the arm so nothing snags when it moves through its full range.

Test: jog pan/tilt through their whole range (`trigger_jog.py`) and watch, no wire should tug at its plug.

---

## 6. 💧 STEP 3, HYDRATE THE GEL (START THIS 4 HOURS EARLY)

The Mythic shoots gel beads that must **soak in water ~4 hours first** (they swell up firm). Do this before anything below:
1. Pour the little beads into a bowl of water.
2. Wait ~4 hours (overnight is fine). They grow to ~7mm and firm up.
3. Toss any that are split or mushy, and drain them before loading.
4. Load them into the gun's **top hopper** (it feeds by gravity, so the gun must sit upright, hopper up).

⚠️ Under-soaked (hard) or over-soaked (mushy) beads are the #1 cause of jams. Firm and round is the target.

---

## 7. 🎯 STEP 4, LIVE-FIRE TEST (on a target, not a person)

1. **Goggles on.** Cardboard box ~2-3 meters away.
2. Charge the gun (USB-C), load hydrated gel, turn the **gun ON**, flip its **orange safety OFF**.
3. Toggle **ON** (armed).
4. Run the dry-fire test to fire a burst at the box:
```
cd ~/sentry-turret && .venv/bin/python trigger_test.py
```
Press ENTER. The servo squeezes the trigger (142→127→142) and the gun fires a **2-3 bead burst** (that's the right amount, you tuned it). 
5. If it fires clean, live fire works. 🎉

**If it doesn't shoot gel** but the arm moves: gun not on / safety still locked / hopper empty or jammed / beads too dry. If the arm strains or buzzes at the squeeze: 127 is a hair too far, see Troubleshooting to back it off.

---

## 8. 🔧 STEP 5, ZERO THE AIM (make it hit where the red dot points)

The red reticle on screen is where the brain THINKS the gun points. Zeroing lines up where gel actually LANDS with that dot.
1. Run the solo system with the camera **mounted ON the moving head** (see Step 6 note):
```
cd ~/sentry-turret && .venv/bin/python turret_brain.py --serial /dev/cu.usbserial-0001 --camera 0 --lock-me
```
2. Aim at the cardboard box, press **SPACE** to arm (screen shows ARMED), toggle ON, gun on/safety off.
3. Let it lock and fire. See where the gel hits vs the red dot.
4. **Nudge the aim with the keys** `I / J / K / L` (up / left / down / right), `0` resets. Fire again, repeat until gel lands on the dot.
5. Read the boresight numbers off the screen (bottom HUD, shows `boresight P +x.x  T +y.y`).
6. **Bake them in** so it remembers next time, add them to your run command:
```
cd ~/sentry-turret && .venv/bin/python turret_brain.py --serial /dev/cu.usbserial-0001 --camera 0 --lock-me --aim-pan X --aim-tilt Y
```
(replace X and Y with your two numbers).

Optional, distance-aware aim (compensates for drop at different ranges): add `--auto-calib`. If shots land low, add `--gel-mps 35` (lower = aim higher); if high, raise it toward 45.

---

## 9. 🤖 STEP 6, RUN THE FULL AUTO-AIM SYSTEM

**Camera rule (important):** the eMeet webcam MUST ride **on the moving head** with the gun (it's "camera-on-gun", centered in frame = aimed). If the camera sits still on the desk, the aim loop cranks to the rail and overshoots. Also, the camera sees whatever's in front of it, so put the turret at **face/chest height** or it'll track legs.

**Two arming layers for a live shot (both required, this is your safety):**
- **SPACE** in the brain window = software armed (brain shows ARMED, only then does it send fire).
- **Toggle ON** = hardware armed (the chip only acts on fire when pin 33 is grounded).
- Plus gun on + gel loaded + orange safety off.
Miss any one and it tracks but won't fire, which is exactly what you want while setting up.

**Solo test (tracks only you, good for tuning):**
```
cd ~/sentry-turret && .venv/bin/python turret_brain.py --serial /dev/cu.usbserial-0001 --camera 0 --lock-me
```

**Guard mode (your real deploy, targets everyone EXCEPT you):**
```
cd ~/sentry-turret && .venv/bin/python turret_brain.py --serial /dev/cu.usbserial-0001 --camera 0 --target-others
```

**How it behaves:** it scans (patrol), locks onto a target, aims, and once locked for a moment it sends a fire burst, then waits 1 second before it can fire again. It won't hose the room. `Q` quits. `SPACE` toggles armed/safe at any time.

**On-screen keys:** `SPACE` arm/safe · `I/J/K/L` nudge aim · `0` reset aim · `Q` quit.

---

## 10. 🧠 FACIAL RECOGNITION (who it tracks / avoids)

It already knows your face (you enrolled ~60 photos). Modes:
- `--lock-me` = tracks ONLY you.
- `--target-others` = tracks everyone EXCEPT you (your guard mode).

**If it doesn't recognize you well** (loses lock, or grabs you in guard mode):
1. Add more photos from the real angles it struggles with, run from your Terminal:
```
cd ~/sentry-turret && .venv/bin/python enroll_capture.py --camera 0
```
Pose the weak angle (standing, looking down, side). It auto-snaps ~24 shots.
2. Then rebuild once by adding `--reenroll` to a single run:
```
cd ~/sentry-turret && .venv/bin/python turret_brain.py --serial /dev/cu.usbserial-0001 --camera 0 --lock-me --reenroll
```
3. Drop `--reenroll` after that one run.

**Stop it confusing your brother as you** (guard mode should target him): enroll HIM as a separate person, then rebuild:
```
cd ~/sentry-turret && .venv/bin/python enroll_capture.py --camera 0 --name aiden --count 40
cd ~/sentry-turret && .venv/bin/python turret_brain.py --serial /dev/cu.usbserial-0001 --camera 0 --lock-me --reenroll
```

**Biggest recognition lever = LIGHTING.** Face the light, don't stand with a bright window behind you (it darkens your face and kills the match).

---

## 11. 🩺 TROUBLESHOOTING (symptom → fix)

**`no such file or directory: .venv/bin/python`** → you're in the wrong folder. Run `cd ~/sentry-turret` first.

**Nothing moves at all** → power. Brick blue light ON? Both plugged (brick in wall AND USB in laptop)? Motors need the brick, chip needs the USB.

**A servo won't move / went limp** → its power, ground, or signal wire popped. Re-seat per the Wiring Reference (Step 3). Power off first.

**Trigger won't fire, arm doesn't swing** → run `trigger_test.py`, press ENTER, read the `[chip]` line:
- `armed=0` → toggle is OFF or pin-33 jumper isn't grounded. Flip toggle ON / re-seat the jumper to the blue (−) rail.
- `armed=1` and `-> FIRING` but no move → arm is jammed. Give it room to swing.
- no `FIRE rx` line at all → the program isn't reaching the chip. Close other programs using the port, re-run.

**Trigger buzzes/strains at the squeeze** → PULL (127) pushes a hair past the trigger's stop. Open `firmware/turret/turret.ino`, change `TRIG_PULL = 127` to `129` or `130`, save, and re-flash (cheat sheet). Higher number = squeezes less far.

**Gun fires too few / too many beads** → open the firmware, find `BURST_MS = 250`. 100 ≈ 1 bead, 250 ≈ 2-3 (now), 450 ≈ 4-5. Change, save, re-flash.

**"port busy" / "resource busy"** → another program has the connection. Close the jog/test/brain window (Ctrl-C or Q) first.

**Camera not found / "none detected"** → run `camera_probe.py`, open the /tmp/cam_N.jpg files, find which N is the eMeet's view, use `--camera N`. And you MUST run from your own Terminal (camera permission), not through me.

**It tracks legs, not face** → raise the turret to face/chest height, or kneel in front of it.

**It swings wildly / overshoots on a small move** → the camera isn't ON the moving head. Mount the eMeet on the gimbal so it turns with the gun.

**It jitters/buzzes at rest** → power. Make sure it's on the 5V brick (not just laptop USB) with the 1000uF cap in.

**It won't lock (green box but never fires)** → it's close; the defaults are already tuned. Make sure you pressed SPACE to arm. If it hunts, the aim's just easing on, give it a second on a still target.

**Gun jams (beads don't feed)** → beads too dry/mushy or hopper low. Re-hydrate to firm, keep the hopper at least half full, gun upright.

---

## 12. 📑 REFERENCE (the numbers, in one place)

**Chip pins:** PAN signal = 13 · TILT signal = 14 · TRIGGER signal = 27 · ARM = 33 (to ground).
**Trigger angles:** REST **142** (clear), PULL **127** (fires). Burst = 400ms (~3 beads, steady; was 250ms which alternated 2/1 because the flywheels hadn't spun up).
**Serial port:** `/dev/cu.usbserial-0001` · **Baud:** 115200.
**Camera:** eMeet, usually `--camera 0` (re-probe if it moved).
**Tuned brain defaults (already baked, no need to type):** kp 0.10, max-step 2.0, deadzone 3.0, coast 1.5, slew-rate 133, det-score 0.6, id-threshold 0.40, id-margin 0.06, patrol-tilt 82, aim-below-face 1.5.
**Firmware file:** `firmware/turret/turret.ino` (edit angles/burst here, then re-flash).
**Scripts:** `trigger_jog.py` (jog all 3) · `trigger_test.py` (dry/live fire + readout) · `camera_probe.py` (find camera) · `enroll_capture.py` (add face photos) · `turret_brain.py` (the full system).

**The finish line:** guard mode running (`--target-others`), aim zeroed, toggle armed, goggles on, hitting a target it locks onto. When that works, you've built a real facial-recognition sentry turret. 🎉
```
```
Save this. You have everything you need. Go finish it. 💪
