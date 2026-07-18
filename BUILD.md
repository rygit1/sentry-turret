# Sentry Turret - Build Guide (open this when the parts arrive)

Written for a true beginner. We build this **together, live, one step at a time**. You never work ahead from this page. Every step is just three beats:

- **Grab:** I name the one part to pick up and show you a photo of it, so you never guess which piece.
- **Do:** one simple action, with exactly where it goes.
- **Show me:** you snap a photo, I check it before you touch power. Nothing gets energized until I have traced it.

**Two rules that keep it safe and simple:**
- If anything ever smells hot, buzzes, smokes, or the board keeps resetting, type **STOP**. I say unplug first, explain after.
- Numbers in **bold (mm / pin / color)** are exact. Where a number depends on your exact part, it says **measure** and how.

**Already done, so you can relax about these:** all the software is installed and tested, and **the ESP32 is already flashed** with the turret firmware. Tomorrow we go straight to building, wiring, and the motion test.

This page is the map. The live walk-through is the real thing.

---

## BUILD-TIME FIXES (my notes, you can skip)
**Beginner: skip this section.** These are engineering notes, not your to-do list. The only one that touches build day is FIX 2 (servo power comes straight from the supply, never through the breadboard), and I walk you through it live at Step 3.
Caught in the pre-order audit (2026-06-22) and the cart review (2026-06-23). None is something you buy; all happen while we build. **Status 2026-06-23 (firmware re-read):** FIX 1 is ALREADY done in `turret.ino`, and FIX 3's firmware half is done too, so only **FIX 2** plus a couple of build-time calibrations actually remain.

**FIX 1 - Firmware fire path: ALREADY DONE (verified 2026-06-23), nothing to change before flashing.** `firmware/turret/turret.ino` is already on the gel layout: **TILT on GPIO14** (not the GPIO12 strapping pin), and FIRE = `fireBurst()`, which pulls the GPIO27 trigger servo (`TRIG_REST 60` -> `TRIG_PULL 110`), holds `BURST_MS = 250 ms`, then releases. No flywheel-MOSFET, no dart-pusher, no `fireOneDart()`. The ONLY trigger work left is at **Step 6**: calibrate `TRIG_REST`/`TRIG_PULL` to your actual linkage (rotate the servo by hand to find the two angles), and optionally tune `BURST_MS` (250 ms is one short burst, roughly 1-2 beads/pull after servo-travel + flywheel spin-up; raise toward 400-500 ms for a fuller 4-5 bead burst).

**FIX 2 - Servo power does NOT run through the breadboard (Step 3).** The breadboard's thin spring rails sag and buzz above ~1-2 A. Run each servo's **red (V+) and brown/black (GND) straight from the 5V supply screw terminal** (or a small terminal block); use the breadboard ONLY for the ESP32, the 1000uF cap, the signal wires, and the common-ground tie. This is the real version of golden rule "servos on the 5V supply only" in Step 3.

**FIX 3 - 270deg servos.** The DS3218 4-pack is the **270deg** version, so a default `write(0..180)` only sweeps part way and each commanded degree moves ~1.5 physical degrees. **Firmware half is DONE:** `turret.ino` already attaches all three servos with `attach(pin, 500, 2500)` for full travel. **Still to do at build:** calibrate the pan/tilt endpoints (plus `CAM_HFOV`/`CAM_VFOV`) in `turret_brain.py`. Pan/tilt only need ~180deg of travel and the trigger only a short pull, so 270deg is fine once calibrated.

---

## SAFETY (read once, obey always)
- **Gel beads only.** Hydrate beads ~4 hours before use. They sting, never aim at a face, eyes, or any person/pet who has not said yes.
- **Goggles on** whenever the blaster has power (the kit includes them).
- **Keep the blaster's trigger lock ON** until the moment you test-fire.
- The turret boots **SAFE** (disarmed). It only fires after you press SPACE to arm AND it is locked on a target.
- Keep the blaster's own power button OFF while wiring and while it is on the bench near your hands.

---

## PART A - What you have (tick each off)
> ASINs are the REVIEW-VERIFIED picks (live-checked 2026-06-22, revised after the cart audit 2026-06-23).
| # | Part | ASIN | Key size to know |
|---|------|------|------------------|
| 1 | DS3218 digital servo **4-pack, 270deg**, ANNIMOS (pan + tilt + trigger + 1 spare) | B07TKTQ2NZ (pick the **20kg x 4PCS 270deg** variant, $48.36) | body **40.5 x 20 x 40 mm**, 25-tooth shaft. 270deg = wider sweep; **calibrate the range in firmware**, it is not a 180 servo |
| 2 | elechawk 2-DOF metal pan/tilt bracket (333) | B07PQ12TXS | holds 2 servos, includes horns + screws. **Inventory the bag on arrival** (missing-part reports are common) |
| 3 | ESP32 dev board, HiLetgo 38-pin (ESP-WROOM-32), CP2102 | B0718T232Z | **53 x 28 mm**. NOTE: this is the **micro-USB** version (not USB-C); use the micro-USB data cable in row 7 |
| 4 | 5V 10A power supply, ALITOVE (enclosed, EMI filter + overload/short protection) + screw-terminal adapter | B0852HL336 | powers the SERVOS only. **5V** (never 12V); meter ~5V under load before wiring |
| 5 | breadboard + jumper wires, BOJACK (718) | B08Y59P6D1 | 830-point, +/- rails down both sides |
| 6 | 1000uF 16V capacitor, Tnisesm 20pk | B089R4QWR3 | has a **stripe = minus leg** |
| 7 | **USB-C to Micro-USB DATA cable** (Mac USB-C to the ESP32's micro-USB) | Cable Matters **B0746NHSCZ** (Micro-B 2.0; grab the 3 ft length) | MUST do **data**, not charge-only. NOT the wide USB-3 Micro-B connector (it will not fit) |
| 8 | bamboo cutting board **8x10 in**, Farberware | **B08BGB422K** | the BASE. Full 8x10 for stability (the gun overhangs the front). Orient the 10in side left-right |
| 9 | Gorilla double-sided mounting tape (23,223) | B082TQ3KB5 | sticks servo + breadboard down. **Wipe the bamboo with rubbing alcohol first** |
| 10 | Nerf Pro Gelfire Mythic gel blaster (2,037) | B09W6DXY4N | ~**0.5-0.7 kg** with barrel+stock off. **Includes the safety eyewear** |
| 11 | VELCRO Brand straps (36,796) | B001E1Y5O6 | strap the blaster to the plate (zip ties also work) |
| 12 | zip ties 8in, TR Industrial (27,604) | B01018DC96 | trigger linkage + tidy wires |
| 13 | SPST mini toggle switch 10-pk, Taiss (965) | B0799LBFNY | ARM cutoff, inline on the trigger servo's V+ wire (no soldering) |
| - | eMeet C980 Pro webcam (you own it) | - | **331 g**, USB-A (use your A-to-C adapter) |

## PART B - Tools
- JOREST 52-in-1 precision screwdriver set (B0D633J3C7, 4,562 reviews)
- A ruler with mm, and a pencil
- Optional: a small zip-lock of coins or steel nuts for the counterweight

---

## STEP 1 - Assemble the pan-tilt bracket
You are building the two-servo neck. Refer to `mockups/pan_mechanism.png`.

1. **Pan servo (the one that spins the whole turret):** mount it into the bracket's **lower** servo pocket with the brass output shaft pointing **UP**. Use 4 of the kit's black self-tap screws through the bracket into the servo's mounting ears. Snug, not gorilla-tight (plastic ears strip).
2. Press a **round horn** onto the pan servo's brass shaft but **do not screw it down yet** (we set the center in Step 5).
3. **Tilt servo:** mount it sideways into the **U-bracket arms**, shaft through the **big round hole in one arm**, and the included **bearing/screw** through the big hole in the **other arm** as the free pivot. Use the kit screws.
4. Press the U's top plate onto the tilt servo's horn, **do not screw down yet**.
5. Leave both horns loose. The servos must still spin freely by hand.

**CHECK:** the U tilts up/down smoothly on the tilt servo, and the whole U can rotate on the pan servo. No binding.

---

## STEP 2 - Mount the bracket to the bamboo base
The gun points forward and sticks out, so the pan axis sits a bit **back of center** to keep weight over the board.

1. On the bamboo board (203 x 254 mm), measure and pencil a dot at **127 mm from each side (centered left-right)** and **90 mm from the FRONT edge**. That dot = the pan axis.
2. Stick the pan servo's base bracket down over that dot with **two strips of Gorilla mounting tape**, then add a **zip-tie collar** if your bracket has slots, for insurance. (No drill needed. If you have a drill, two 3 mm screws are even better.)
3. Make sure the gun will point toward the **front edge** (the 90 mm side).

**CHECK:** push the bracket sideways hard, it does not peel or slide.

---

## STEP 3 - Build the brain board (wiring)
This is the part people fear. Go one wire at a time. **Three golden rules:**
- **Servos get power ONLY from the 5V 5A supply.** Never from the ESP32's 5V pin (it cannot source 3 servos and will brown out/reset).
- **Common ground:** the ESP32's GND and the power supply's minus MUST connect, or the signal means nothing.
- **The 1000uF capacitor goes across the servo power rails**, stripe-leg to minus. It is the shock absorber for servo current spikes.

**Breadboard orientation:** long way left-to-right. The two lines down each side are the **power rails** (red = +, blue = -). The middle has rows numbered 1-30, columns a-e and f-j, split by the center trench.

**Wire it in this exact order:**

> Your servo wires (confirmed): **red = power (to the + rail)**, **black = ground (to the minus rail)**, **white = signal (to the GPIO pin)**. Same on all three servos; only the white signal wire changes pin.
| Step | From | To | Wire |
|------|------|----|----|
| 3.1 | 5V supply **+** (via screw terminal) | breadboard **red (+) rail** | red |
| 3.2 | 5V supply **-** | breadboard **blue (-) rail** | black |
| 3.3 | 1000uF cap **+ leg** | red (+) rail | - |
| 3.4 | 1000uF cap **- leg (stripe side)** | blue (-) rail | - |
| 3.5 | ESP32 plugged into the board straddling the center trench, **USB port facing OFF the board's edge** | rows ~10-28 | - |
| 3.6 | ESP32 **GND** pin | blue (-) rail | black (THE common-ground wire) |
| 3.7 | Pan servo: **red** wire to red rail, **brown/black** to blue rail, **orange/white signal** to **GPIO13** | - | - |
| 3.8 | Tilt servo: **red** to red rail, **brown/black** to blue rail, **signal** to **GPIO14** | - | - |
| 3.9 | Trigger servo: **red** to red rail, **brown/black** to blue rail, **signal** to **GPIO27** | - | - |

Pins to use (already SAFE, avoid the strapping pins): **PAN = GPIO13, TILT = GPIO14, TRIGGER = GPIO27.** Do not use GPIO12 (boot-strapping pin).

**CHECK:** trace each servo's 3 wires with your finger: signal to its GPIO, red to red rail, dark to blue rail. The cap stripe is on the blue rail. The ESP32 GND reaches the blue rail. **Do not power on yet.**

---

## STEP 4 - Flash the ESP32  (ALREADY DONE 2026-06-24)
We did this together ahead of time, so there is nothing to do here on build day.
- Port confirmed: **`/dev/cu.usbserial-0001`** (your Mac sees it natively, no driver to install).
- The cable is a real data cable, the firmware uploaded and hash-verified, and the board auto-reset and is running.
- The firmware lives in permanent memory, so unplugging it changed nothing.

**Skip straight to Step 5.** (If we ever need to re-flash: `arduino-cli compile --upload -p /dev/cu.usbserial-0001 --fqbn esp32:esp32:esp32 firmware/turret/turret.ino`.)

---

## STEP 5 - First motion test (NO gun yet)
1. Now turn the **5V supply ON**.
2. Run, from `~/sentry-turret`:
   `.venv/bin/python turret_brain.py --serial /dev/cu.usbserial-0001 --no-patrol`
3. Both servos should jump to **center (90 deg)**. With power on and centered:
   - Pop each horn off and re-seat it so the bracket sits **square/level at center**, THEN screw the horns down tight.
4. Wave your face around. Pan should turn the gun toward you, tilt should follow up/down. If an axis moves the **wrong way**, tell Claude, we flip that servo's direction in firmware (one line).

**CHECK:** the bare bracket tracks your face left/right and up/down, smoothly, no buzzing. Center looks level.

---

## STEP 6 - Mount the camera + blaster (the BALANCE step)
Refer to `mockups/balance_layout.png`. Goal: combined center of mass sits **over the tilt axis** so the servo holds almost nothing.

1. **Strip the blaster:** pull off the **removable barrel and stock**. This drops it from 680 g to ~**500 g** and shortens the forward weight.
2. **Mount the camera ON TOP of the gun, toward the FRONT**, directly above the barrel, lens looking forward. This is the clear-view spot: an unobstructed view straight down the line of fire and the smallest parallax. The hopper sits behind it, so nothing blocks the view. (A rear-mounted camera gets blocked by the gun, do not do that.)
3. **Mount the trigger servo** at the trigger now too, so the whole head is one assembled unit.
4. **Balance the WHOLE assembly as one unit:** rest the entire gun + camera + trigger servo across a pencil, find where it balances, mark it. Strap it to the U top plate with **2 Velcro straps** so that **combined mark sits over the tilt axis**. Because the forward camera pulls the balance forward, you slide the whole gun a few cm BACK of centered to bring the combined balance point onto the axis. If it still cannot reach, add a small counterweight (steel nuts or a coin stack, ~7 g each) at the very REAR.
5. **Lift test (the real judge):** power the tilt servo OFF (unplug its signal), set the gun at **45 deg up**, let go. It should **stay put**. If the muzzle drops, slide mass back or add **steel nuts at the far rear** (each ~7 g; a stack 80 mm back fixes a small lean). If it falls backward, nudge forward.
6. **Trigger linkage:** mount the trigger servo beside the trigger guard. Zip-tie a short arm from the servo horn to the trigger so that rotating the servo **pulls the trigger**. The Mythic is full-auto and USB-C powered, so the servo only has to *pull and hold*, the blaster runs its own flywheels.
7. **Measure cam_dy:** with a ruler, measure the height of the camera lens **above the barrel center line**, in meters (e.g. 60 mm = **0.06**). You will type this into `--cam-dy`.

**CHECK:** lift test holds at 45 deg up and 45 deg down. Trigger servo arm clearly pulls the trigger when you rotate it by hand.

---

## STEP 7 - Go live (tracking)
1. Charge the blaster (USB-C), load hydrated gel beads, but keep its **trigger lock ON** and **power button OFF** for now.
2. Select the camera by NAME: add `--camera-name eMeet` (survives macOS index reshuffling, no number to hunt for). The numeric `--camera <n>` index shifts with whatever is plugged in, so prefer the name. 
3. Run:
   `.venv/bin/python turret_brain.py --lock-me --serial /dev/cu.usbserial-0001 --camera-name eMeet`
4. Confirm it tracks **only you** and the gun follows. Patrol scan kicks in when you leave the frame, it snaps back when you return.
5. If swivel is too slow, raise speed: tell Claude to bump `slew` (7 -> 10-12) and `kp` (0.40 -> 0.5). If it jitters/overshoots, lower them.

**CHECK:** smooth lock on your face, gun physically aimed at you, no oscillation.

---

## STEP 8 - Zero the gun (make it actually hit)
Two layers, do them in order.

**8a. Boresight (fixed zero):**
1. Tape a paper target at about **2-3 m**, same height as the turret.
2. Goggles on, beads loaded, trigger lock OFF, **arm with SPACE**, let it lock + fire a short burst.
3. Watch where gel lands vs the on-screen reticle. **Nudge live with I / J / K / L** (0 resets) until hits land on the crosshair.
4. Read the HUD `boresight P+x.x T+y.y` and start it pre-zeroed next time with `--aim-pan x.x --aim-tilt y.y`.

**8b. Distance-aware auto-calibrate (hits at any range):**
1. Add `--auto-calib --cam-dy 0.06` (use YOUR measured cam height) `--target-width 0.15`.
2. Fire at **~1.5 m** and **~4 m**. If far shots land **low**, lower `--gel-mps` (default is now **40**, the Mythic's real ~130-150 FPS muzzle; try ~34); if they land high, raise it. Two distances pin it.
3. The HUD shows `auto-cal d=2.3m aim up +1.8` updating with range. Squat, stand, step back, it re-aims correctly (range comes from your face width, which posture does not change; the rifleman's-rule term keeps steep down-shots true).

Full live command once zeroed:
`.venv/bin/python turret_brain.py --lock-me --serial /dev/cu.usbserial-0001 --camera-name eMeet --auto-calib --ballistic-lead --cam-dy 0.06 --aim-pan <x> --aim-tilt <y>`

Add `--ballistic-lead` (lead by the bead's time-of-flight, better on movers) and optionally `--body-fusion` (keeps the lock when you turn away; needs `pip install ultralytics`). The **camera-on-gun aim law turns on automatically with `--serial`** (use `--fixed-camera` only if the camera is NOT on the gun).

**CHECK:** beads land on target at 1.5 m AND 4 m, standing and squatting.

**8c. Max accuracy (optional, the best it gets).** The brain already aims with the exact lens-tangent projection, ranges off your **eye spacing** (steadier than the face box), and leads + **coasts** through brief detection dropouts. For the last bit, run a one-time calibration to correct your webcam's real lens distortion:
1. Print a 9x6 checkerboard, then run `.venv/bin/python calibrate_camera.py --camera 1`.
2. Hold it at many angles + distances, press SPACE ~15-20 times, then Q. It saves `models/intrinsics.npz` and prints your true HFOV/VFOV.
3. Add `--intrinsics models/intrinsics.npz` to the live command. The bore now uses distortion-corrected rays and your real focal length for ranging.

---

## Troubleshooting
| Symptom | Cause | Fix |
|--------|-------|-----|
| ESP32 keeps resetting when servos move | servos drawing from ESP32 / no cap | servos on the 5V supply only; add the 1000uF cap |
| Servos twitch / random | no common ground | join ESP32 GND to supply minus |
| One axis moves backward | servo direction | flip that axis in firmware (one line) |
| Gun droops / tilt buzzes hot | unbalanced load | redo the Step 6 lift test, add rear counterweight |
| No serial port | missing driver | install CP210x, replug |
| Shots always low at distance | gel speed too high in model | lower `--gel-mps` |
| Loses you when you turn | (already handled) identity hysteresis | keep `--lock-me`, default threshold |

## Reference card (every number in one place)
- Pins: **PAN 13, TILT 14, TRIGGER 27** (avoid GPIO12)
- Pan axis on base: **127 mm from sides, 90 mm from front**
- Servo body: **40.5 x 20 x 40 mm**; ESP32: **53 x 28 mm**; board: **203 x 254 mm** (8x10 in)
- Servos: **DS3218 270deg** 4-pack (calibrate range, pulse 500-2500). Cable: **USB-C to micro-USB DATA** (the ESP32 is micro-USB)
- Weights: camera **331 g**, blaster stripped **~500 g**, servo **60 g**
- Balance: camera on top-FRONT (clear view); balance the WHOLE gun+camera+trigger assembly over the tilt axis (slide the gun back); rear nuts only if needed
- Servo torque budget: DS3218 **19 kg·cm @ 5V**; balanced load ~2 kg·cm (easy)
- Boresight keys: **I/J/K/L** nudge, **0** reset; flags `--aim-pan --aim-tilt`
- Auto-calib: `--auto-calib --cam-dy <m> --target-width 0.15 --gel-mps 40` (Mythic ~130-150 FPS muzzle)
- Lead = full intercept solver (`--ballistic-lead`): couples slew-time (`--slew-rate` deg/s, MEASURE by timing a 0->180), drag flight (`--drag-k` 1/m, ~0.04, 2-range fit), future range, and `--lead-latency`. Body-fusion: `--body-fusion` (needs `pip install ultralytics`)
- Aim law auto-switches to camera-on-gun when `--serial` is set; `--fixed-camera` forces the sim law
- cam_dy = camera lens height above the barrel, in METERS
