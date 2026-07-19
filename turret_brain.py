#!/usr/bin/env python3
"""Phase 0/1 brain for the CV sentry turret.

Detects a target on the webcam, runs the same aim controller the real turret
will use, and (Phase 0) prints / (Phase 1) streams over serial the command it
sends to the servos. No hardware needed for Phase 0: a virtual bore reticle
chases the target on screen so you can watch the loop converge and lock.

  Live (YuNet face):  python turret_brain.py
  Lock onto ME:       python turret_brain.py --lock-me            # only tracks Ryan
  Guard (all but me): python turret_brain.py --target-others      # tracks anyone but Ryan
  Check recognition:  python turret_brain.py --id-eval            # scores you vs everyone, no camera
  Full body (YOLO):   python turret_brain.py --detector person   # pip install ultralytics
  Drive real turret:  python turret_brain.py --serial /dev/cu.usbserial-XXXX
  Self-test:          python turret_brain.py --selftest
  Keys:               SPACE = arm / disarm,   Q = quit

--lock-me / --target-others identify a specific face (default 'ryan') with SFace
embeddings built from the room-security enrollment photos (~/room-security/known_faces).
YuNet stays the detector; recognition only labels the faces it already finds.

Default face detector is YuNet (DNN) when models/face_detection_yunet_2023mar.onnx
is present, which holds onto turned / profile / edge-of-frame faces. Falls back
to the Haar cascade automatically if the model is missing.
"""

import argparse
import math
import os
import sys
import time

try:
    import numpy as np  # noqa: F401  (cv2 returns numpy arrays)
    import cv2
except ImportError:
    sys.exit("Missing deps. Run:  pip install opencv-python numpy")

HERE = os.path.dirname(os.path.abspath(__file__))
YUNET_PATH = os.path.join(HERE, "models", "face_detection_yunet_2023mar.onnx")
YOLO_FACE_PATH = os.path.join(HERE, "models", "yolov11n-face.pt")

# --- Webcam + servo geometry (tune HFOV/VFOV to your camera) ---------------
CAM_HFOV = 82.0          # horizontal FOV, degrees (eMeet C980 Pro, 90 deg diagonal -> ~82 H on 16:9)
CAM_VFOV = 52.0          # vertical FOV, degrees (90 deg diagonal -> ~52 V on 16:9); refine w/ calibrate_camera.py
PAN_CENTER = 90          # servo degrees pointing straight ahead
TILT_CENTER = 72         # neutral inside the safe tilt band [60,85] (was 90, now out of range)
SERVO_MIN, SERVO_MAX = 0, 180
# Safe crash-limits enforced ONLY in the automatic loop (tracking + patrol); manual jog stays free.
PAN_LIMIT_MIN, PAN_LIMIT_MAX = 38, 170
TILT_LIMIT_MIN, TILT_LIMIT_MAX = 60, 85
LOCK_TOL_DEG = 5.0       # bore within this of target = LOCKED (looser = locks on more easily). 5.0 is demo-forgiving: fine for a gel body shot, and it locks even when the aim is settling. Tighten toward 3 for precision.
FIRE_HOLD_FRAMES = 5     # consecutive locked frames before a FIRE
FIRE_COOLDOWN_S = 1.0

# --- Predictive lead + idle patrol -----------------------------------------
LEAD_SMOOTH = 0.35       # EMA weight on the target-velocity estimate (higher = snappier, noisier)
PATROL_AFTER_S = 3.0     # seconds with no target before idle scanning kicks in (holds on you through a blur/turn)
PATROL_SPEED = 35.0      # patrol sweep speed, degrees/second
PATROL_MIN, PATROL_MAX = 45.0, 160.0   # pan sweep limits while scanning, degrees (inside hard cap 38-170)
PATROL_TILT_MIN, PATROL_TILT_MAX = 60.0, 83.0   # tilt band for patrol (inside hard cap 60-85)
PATROL_TILT_STEP = 0.0   # 0 = FLAT patrol (no tilt bump = no shake); held at the --patrol-tilt value
PATROL_TILT_PARK = 82.0  # patrol park tilt. Ryan: HIGH number = aim UP (80 up, 60 down); verify live w/ --patrol-tilt
SEARCH_LOCAL_S = 2.5     # after losing a target, first scan TIGHT around where it vanished (smart search)
SEARCH_LOCAL_DEG = 20.0  # half-size (deg) of that local scan box before widening to the full sweep

# --- Distance-aware auto-calibration (parallax + gel drop) -----------------
CAM_BARREL_DY = 0.15     # camera sits this many METERS from the barrel line (Ryan measured ~6 in BELOW)
CAM_BARREL_DX = 0.0      # camera left/right offset from the barrel, meters
GEL_MPS = 40.0           # gel-bead effective speed, m/s. The Mythic chronographs ~130-150 FPS
                         # muzzle (~40-46 m/s, avg ~132 FPS); default near the low muzzle value.
                         # Air drag slows a light gel bead, so the empirically-tuned value (BUILD.md,
                         # 2 ranges) may settle a bit LOWER. Shots land low -> lower it; high -> raise.
REAL_TARGET_WIDTH = 0.15 # real width of what you track, meters (~0.15 a face, ~0.45 a body)
REAL_EYE_DIST = 0.063    # real inter-ocular (eye-to-eye) distance, meters (~63mm adult)
GRAVITY = 9.81
DIST_MIN, DIST_MAX = 0.3, 12.0   # clamp the distance estimate to sane meters


def clamp(v, lo, hi):
    return lo if v < lo else hi if v > hi else v


SPRINT_RAMP = 0.2   # extra deg/frame of sprint speed per degree of error beyond the handoff point


class PID:
    """Per-axis controller. Output is a slew-limited angle delta (degrees/frame).

    `sprint` (deg/frame, 0 = off): far-target speed cap. Inside the handoff point (the error
    where kp*err == slew, i.e. the WHOLE close-in zone) the output is bit-identical to the
    plain controller, so the tuned settle feel is untouched. Beyond it the step ramps up
    (SPRINT_RAMP per degree of extra error) toward `sprint`, so a cross-room error is crossed
    at highway speed instead of crawling at the close-in cap the whole way. The ramp starts at
    exactly `slew`, so speed is continuous at the handoff (no gear-clunk). P-only paths use this."""

    def __init__(self, kp, ki, kd, slew, sprint=0.0):
        self.kp, self.ki, self.kd, self.slew = kp, ki, kd, slew
        self.sprint = sprint
        self.i = 0.0
        self.prev = 0.0

    def step(self, err, dt):
        self.i += err * dt
        d = (err - self.prev) / dt if dt > 0 else 0.0
        self.prev = err
        out = self.kp * err + self.ki * self.i + self.kd * d
        lim = self.slew
        if self.sprint > self.slew:
            e = abs(err)
            start = self.slew / max(self.kp, 1e-6)
            if e > start:
                lim = min(self.slew + SPRINT_RAMP * (e - start), self.sprint)
                out = math.copysign(lim, err)   # far away: travel AT the sprint speed
        return clamp(out, -lim, lim)

    def reset(self):
        self.i = self.prev = 0.0


def pixel_to_angle(cx, cy, w, h):
    """Target pixel -> absolute (pan, tilt) servo angle, exact pinhole (rectilinear)
    model: angle = atan(normalized_offset * tan(FOV/2)). A real lens maps angle to
    pixels by x = f*tan(angle), so the old linear `ex*FOV/2` under-read the true angle
    everywhere but the center and edge (off by ~2 deg mid-frame at 70 deg HFOV). The
    tangent form is exact across the whole frame, which is the bulk of the aim error."""
    ex = (cx - w / 2.0) / (w / 2.0)          # -1..1 left/right
    ey = (cy - h / 2.0) / (h / 2.0)          # -1..1 top/bottom
    pan = PAN_CENTER + math.degrees(math.atan(ex * math.tan(math.radians(CAM_HFOV / 2.0))))
    tilt = TILT_CENTER - math.degrees(math.atan(ey * math.tan(math.radians(CAM_VFOV / 2.0))))
    return clamp(pan, SERVO_MIN, SERVO_MAX), clamp(tilt, SERVO_MIN, SERVO_MAX)


def angle_to_pixel(pan, tilt, w, h):
    """Inverse of pixel_to_angle (exact tangent model)."""
    ex = math.tan(math.radians(pan - PAN_CENTER)) / math.tan(math.radians(CAM_HFOV / 2.0))
    ey = -math.tan(math.radians(tilt - TILT_CENTER)) / math.tan(math.radians(CAM_VFOV / 2.0))
    return int(w / 2.0 + ex * (w / 2.0)), int(h / 2.0 + ey * (h / 2.0))


def load_intrinsics(path):
    """Load camera matrix K (3x3) and distortion coeffs from an .npz with keys 'K' and
    'dist' (make one with calibrate_camera.py). Returns (None, None) if no path. This is
    the true max-accuracy path: real intrinsics + lens-distortion correction beat the
    single-number FOV pinhole model, especially toward the frame edges."""
    if not path:
        return None, None
    data = np.load(path)
    return data["K"].astype(np.float64), data["dist"].astype(np.float64)


def pixel_to_angle_K(cx, cy, K, dist):
    """Pixel -> (pan, tilt) using real camera intrinsics + lens-distortion correction.
    undistortPoints returns normalized image coords (x', y'), which equal tan(angle),
    so the ray angles come out exact and free of barrel distortion."""
    pts = np.array([[[float(cx), float(cy)]]], dtype=np.float64)
    norm = cv2.undistortPoints(pts, K, dist)
    xn, yn = float(norm[0, 0, 0]), float(norm[0, 0, 1])
    pan = PAN_CENTER + math.degrees(math.atan(xn))
    tilt = TILT_CENTER - math.degrees(math.atan(yn))
    return clamp(pan, SERVO_MIN, SERVO_MAX), clamp(tilt, SERVO_MIN, SERVO_MAX)


def lead_target(cx, cy, vx, vy, lead, w, h):
    """Project the target pixel forward by its velocity so we aim where it is GOING,
    not where it was. lead is seconds; vx, vy are pixels/second."""
    return clamp(cx + vx * lead, 0, w - 1), clamp(cy + vy * lead, 0, h - 1)


def patrol_step(pan, tilt, pan_dir, tilt_dir, dt,
                pan_min=PATROL_MIN, pan_max=PATROL_MAX,
                tilt_min=PATROL_TILT_MIN, tilt_max=PATROL_TILT_MAX):
    """2D idle scan. Sweep pan across [pan_min, pan_max]; at each horizontal end, bump tilt by one
    band and reverse pan, so the gun RASTERS over a whole area instead of a single horizontal line.
    A target slightly above or below the current view still gets swept through. Bounces at the tilt
    limits too. Returns (pan, tilt, pan_dir, tilt_dir)."""
    pan += pan_dir * PATROL_SPEED * dt
    if pan >= pan_max or pan <= pan_min:
        pan = clamp(pan, pan_min, pan_max)
        pan_dir = -pan_dir
        tilt += tilt_dir * PATROL_TILT_STEP        # drop/raise to the next band at each horizontal end
        if tilt >= tilt_max or tilt <= tilt_min:
            tilt = clamp(tilt, tilt_min, tilt_max)
            tilt_dir = -tilt_dir
    return pan, tilt, pan_dir, tilt_dir


def focal_px(w):
    """Pinhole focal length in pixels, from the camera's horizontal FOV."""
    return (w / 2.0) / math.tan(math.radians(CAM_HFOV / 2.0))


def estimate_distance(box_w_px, frame_w, real_width=REAL_TARGET_WIDTH, focal=None):
    """Distance to the target in meters from its apparent width (similar triangles).
    Pass `focal` (px) from real calibration to override the FOV-derived focal length."""
    if box_w_px <= 0:
        return DIST_MAX
    f = focal if focal is not None else focal_px(frame_w)
    return clamp(real_width * f / box_w_px, DIST_MIN, DIST_MAX)


def estimate_distance_iod(iod_px, frame_w, real_iod=REAL_EYE_DIST, focal=None):
    """Distance from the inter-ocular (eye-to-eye) pixel distance. Eye landmarks are far
    more stable than the face box and vary ~5% person-to-person vs ~15% for face width,
    so this is the more accurate range cue whenever landmarks are available."""
    if iod_px <= 0:
        return DIST_MAX
    f = focal if focal is not None else focal_px(frame_w)
    return clamp(real_iod * f / iod_px, DIST_MIN, DIST_MAX)


def face_aim(row):
    """From a YuNet detection row (box + 5 landmarks) return (aim_x, aim_y, inter_ocular_px),
    or None. The aim point is the CENTROID of the available facial landmarks (eyes + nose +
    mouth corners), i.e. dead center of the face, a more natural turret target than the box
    center or the brow. The inter-ocular eye spacing is kept as the range cue. Row layout:
    [x, y, w, h, rEyeX, rEyeY, lEyeX, lEyeY, noseX, noseY, rMouthX, rMouthY, lMouthX, lMouthY, ...]."""
    if row is None or len(row) < 8:
        return None
    rex, rey, lex, ley = float(row[4]), float(row[5]), float(row[6]), float(row[7])
    iod = math.hypot(lex - rex, ley - rey)
    xs, ys = [rex, lex], [rey, ley]
    for i in range(8, min(14, len(row)) - 1, 2):     # add nose + mouth corners when present
        xs.append(float(row[i]))
        ys.append(float(row[i + 1]))
    return sum(xs) / len(xs), sum(ys) / len(ys), iod


def aim_correction(d, cam_dy=CAM_BARREL_DY, cam_dx=CAM_BARREL_DX, gel_mps=GEL_MPS, elev_deg=0.0):
    """Distance-dependent aim offset in degrees to ADD: parallax (the camera sits
    above/beside the barrel, so aim up, shrinks with range) + gravity drop of the
    gel bead (aim up, grows with range). elev_deg is the barrel's firing elevation
    from horizontal; the drop term scales by cos(elev) (the rifleman's rule) so steep
    shots -- e.g. tracking someone who squats low and close -- stay zeroed.
    Returns (pan_deg, tilt_deg)."""
    parallax_pan = math.degrees(math.atan2(cam_dx, d))
    parallax_tilt = math.degrees(math.atan2(cam_dy, d))
    # Superelevation to cancel gravity drop. Horizontal range governs the drop
    # (rifleman's rule), and we take the exact atan(drop/range) instead of the
    # small-angle shortcut, so steep/long shots stay true.
    horiz = d * math.cos(math.radians(elev_deg))
    drop_tilt = math.degrees(math.atan(0.5 * GRAVITY * horiz / (gel_mps ** 2)))
    return parallax_pan, parallax_tilt + drop_tilt


def serial_cmd(pan, tilt, fire):
    return f"P{int(round(pan)):03d} T{int(round(tilt)):03d}" + (" FIRE" if fire else "")


def drive(pid, cur, tgt, dt, on_gun, center, deadzone=0.0):
    """Step one servo axis toward `tgt` (an absolute servo angle) and report the error driven.

    The right error depends on where the camera is mounted:
      * FIXED camera (Phase 0 sim = the Mac webcam): a target pixel maps to a FIXED world angle, so
        the loop drives the servo to that absolute angle.  error = tgt - cur.
      * CAMERA-ON-GUN (the real turret): the camera rides on the gun, so turning the servo turns the
        view; the controllable signal is the target's offset from frame CENTER, and the loop nulls
        it (image-based visual servoing).  error = tgt - center.
    Running the fixed-camera law on the on-gun hardware settles a stationary off-center target at the
    HALFWAY point (a persistent under-aim of half the offset) and reports LOCKED while mis-aimed, so
    the law must match the mount.

    `deadzone` (deg): when the aim error is within this band, HOLD the servo instead of issuing a
    tiny correction. A real servo buzzes/hunts on sub-degree commands it can't physically resolve;
    holding inside the deadzone kills that jitter and the gear wear while staying well inside the
    lock tolerance. 0 = off (the sim default, so convergence checks are unchanged).
    Returns (new_angle, error_deg)."""
    err = (tgt - center) if on_gun else (tgt - cur)
    if abs(err) <= deadzone:
        return cur, err                      # on target enough: hold, don't hunt
    return clamp(cur + pid.step(err, dt), SERVO_MIN, SERVO_MAX), err


def coast_step(rem, kp, max_step, deadzone):
    """One blind coast frame: keep walking toward where the target was LAST SEEN, by the plain
    (no-sprint) P step. `rem` is the aim travel still owed from the last real detection, so blind
    motion is bounded by a place the camera actually saw: it can finish the move but never run
    off to a rail the way velocity extrapolation did. Returns (delta_deg, remaining_after)."""
    if abs(rem) <= deadzone:
        return 0.0, rem
    step = clamp(kp * rem, -max_step, max_step)
    return step, rem - step


def body_aim(box, frac):
    """Aim point for a person (body) box when no face is visible: horizontal center, `frac` of the
    way down from the top (~0.13 = head height, ~0.5 = center mass). Head height keeps the aim point
    near where the face was so the face<->body hand-off does not jump."""
    x, y, bw, bh = box
    return x + bw // 2, int(round(y + frac * bh))


def flight_time(d_m, v0, drag_k):
    """Gel-bead time-of-flight to range d (m) with quadratic drag. drag_k (1/m) is the drag constant;
    drag_k=0 reduces to no-drag d/v0. With drag the bead keeps slowing, so flight time grows faster
    than linearly with range -- which is exactly why ONE fixed speed can't be right at every distance."""
    v0 = max(v0, 1e-3)
    if drag_k <= 1e-9:
        return d_m / v0
    return (math.exp(drag_k * d_m) - 1.0) / (drag_k * v0)


def speed_at(d_m, v0, drag_k):
    """Bead speed still remaining at range d -- the velocity that governs gravity drop way out there."""
    return max(v0, 1e-3) * math.exp(-max(drag_k, 0.0) * d_m)


def intercept(tx, ty, vx, vy, d, d_rate, pan, tilt, w, h,
              v0, drag_k, slew_rate, latency, on_gun, iters=4):
    """Fixed-point fire-control solve: aim where the target will BE when the bead arrives, accounting
    for BOTH the time to swing the gun onto the lead point (angular move / measured slew rate, + a
    fixed pipeline latency) AND the bead's drag-aware time-of-flight to the FUTURE range. Lateral
    motion (vx, vy px/s), radial motion (d_rate m/s) and the slew time are all coupled and re-solved
    a few times, so the lead scales with distance and with how far the gun must travel. The horizon is
    short (tens to a few hundred ms), so constant velocity over it is a fair model.
    Returns (aim_x, aim_y, total_lead_s, d_future)."""
    T = flight_time(d, v0, drag_k)            # seed: just the bead flight to the present range
    px, py, d_fut = float(tx), float(ty), d
    for _ in range(max(iters, 1)):
        px = clamp(tx + vx * T, 0.0, w - 1.0)
        py = clamp(ty + vy * T, 0.0, h - 1.0)
        fpan, ftilt = pixel_to_angle(px, py, w, h)
        # how far the gun must still rotate to reach the lead point: on-gun it nulls the offset from
        # frame CENTER; fixed-camera (sim) it moves from its current angle to the absolute lead angle.
        dtheta = math.hypot(fpan - PAN_CENTER, ftilt - TILT_CENTER) if on_gun \
            else math.hypot(fpan - pan, ftilt - tilt)
        t_aim = max(latency, 0.0) + dtheta / max(slew_rate, 1e-3)
        d_fut = clamp(d + d_rate * T, DIST_MIN, DIST_MAX)
        T = clamp(t_aim + flight_time(d_fut, v0, drag_k), 0.0, 2.0)
    return px, py, T, d_fut


# --- Detectors -------------------------------------------------------------
class YuNetFaceDetector:
    name = "face (yunet)"

    def __init__(self, model_path, score=0.5):
        # cv2.FaceDetectorYN holds onto turned / profile / tilted faces far better
        # than Haar and still runs fast on CPU. score=0.5 keeps a grip on faces
        # that turn or blur as you move (raise it if it boxes non-faces).
        self.det = cv2.FaceDetectorYN.create(model_path, "", (320, 320), score, 0.3, 5000)
        self._size = None
        self.last_faces = None       # raw Nx15 rows (box + 5 landmarks); used for recognition

    def detect(self, frame):
        h, w = frame.shape[:2]
        if self._size != (w, h):
            self.det.setInputSize((w, h))
            self._size = (w, h)
        _, faces = self.det.detect(frame)
        self.last_faces = faces      # aligned 1:1 with the boxes returned below
        out = []
        if faces is not None:
            for f in faces:
                x, y, bw, bh = f[:4]
                out.append((int(x), int(y), int(bw), int(bh)))
        return out


class FaceDetector:
    """Haar fallback: frontal + profile (and a mirrored pass for the other side)."""
    name = "face (haar)"

    def __init__(self):
        base = cv2.data.haarcascades
        self.frontal = cv2.CascadeClassifier(base + "haarcascade_frontalface_default.xml")
        self.profile = cv2.CascadeClassifier(base + "haarcascade_profileface.xml")
        if self.frontal.empty():
            raise RuntimeError("could not load haar cascade")

    def detect(self, frame):
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        boxes = list(self.frontal.detectMultiScale(gray, 1.1, 5, minSize=(60, 60)))
        if not self.profile.empty():
            boxes += list(self.profile.detectMultiScale(gray, 1.1, 5, minSize=(60, 60)))
            flipped = cv2.flip(gray, 1)       # profile cascade is one-sided; flip for the other
            w = gray.shape[1]
            for (x, y, bw, bh) in self.profile.detectMultiScale(flipped, 1.1, 5, minSize=(60, 60)):
                boxes.append((w - x - bw, y, bw, bh))
        return [tuple(int(v) for v in b) for b in boxes]


class PersonDetector:
    name = "person (yolo11n)"

    def __init__(self):
        from ultralytics import YOLO
        self.model = YOLO("yolo11n.pt")

    def detect(self, frame):
        res = self.model(frame, classes=[0], verbose=False)[0]
        out = []
        for x1, y1, x2, y2 in res.boxes.xyxy.cpu().numpy()[:, :4]:
            out.append((int(x1), int(y1), int(x2 - x1), int(y2 - y1)))
        return out


class YoloFaceDetector:
    """YOLO11 trained on WIDERFACE. Heavier (needs torch) but stays in the YOLO
    stack, can use the Mac GPU (MPS), and is the bridge to face recognition."""
    name = "face (yolo11)"

    def __init__(self, weights):
        from ultralytics import YOLO
        self.model = YOLO(weights)

    def detect(self, frame):
        res = self.model(frame, verbose=False)[0]
        out = []
        for x1, y1, x2, y2 in res.boxes.xyxy.cpu().numpy()[:, :4]:
            out.append((int(x1), int(y1), int(x2 - x1), int(y2 - y1)))
        return out


def make_detector(kind, score=0.6):
    if kind == "person":
        return PersonDetector()
    if kind == "yolo-face":
        if not os.path.exists(YOLO_FACE_PATH):
            sys.exit("YOLO face weights missing. Put yolov11n-face.pt in models/ "
                     "(huggingface.co/AdamCodd/YOLOv11n-face-detection or "
                     "github.com/akanametov/yolo-face), then: pip install ultralytics")
        return YoloFaceDetector(YOLO_FACE_PATH)
    if kind == "haar":
        return FaceDetector()
    if os.path.exists(YUNET_PATH):          # default "face"
        try:
            return YuNetFaceDetector(YUNET_PATH, score)
        except Exception as e:
            print(f"[warn] YuNet failed ({e}), using Haar", file=sys.stderr)
    return FaceDetector()


def pick_target(boxes, prev_center):
    """Threat priority: pick the CLOSEST target (largest box = nearest = highest threat), with a
    stickiness bonus for the one we're already tracking so it doesn't flip-flop between two people of
    similar size. A clearly closer/bigger newcomer (>~40% larger) still steals focus. On the first
    acquisition (no prev) it's simply the largest. Returns (x, y, w, h, cx, cy)."""
    if not boxes:
        return None
    if prev_center is not None:
        px, py = prev_center

        def threat(b):
            cx, cy = b[0] + b[2] / 2, b[1] + b[3] / 2
            d2 = (cx - px) ** 2 + (cy - py) ** 2
            sticky = 1.4 if d2 < (max(b[2], b[3]) * 0.75) ** 2 else 1.0   # likely the same target
            return b[2] * b[3] * sticky        # box area = closeness = threat level
        x, y, w, h = max(boxes, key=threat)
    else:
        x, y, w, h = max(boxes, key=lambda b: b[2] * b[3])
    return x, y, w, h, x + w // 2, y + h // 2


# --- Rendering -------------------------------------------------------------
def draw(frame, dets, target, bore_px, pan, tilt, armed, locked, fire, cmd, fps, dname, id_line="", extra=""):
    h, w = frame.shape[:2]
    cv2.drawMarker(frame, (w // 2, h // 2), (90, 90, 90), cv2.MARKER_CROSS, 18, 1)
    tgt_center = (target[4], target[5]) if target else None
    # when recognition is on, label every detected face (green = me, red = not me)
    if id_line:
        for d in dets:
            x, y, bw, bh = d["box"]
            cx, cy = x + bw // 2, y + bh // 2
            if tgt_center is not None and abs(cx - tgt_center[0]) <= 2 and abs(cy - tgt_center[1]) <= 2:
                continue                                   # the target gets the bold box below
            col = (0, 220, 120) if d.get("is_me") else (0, 0, 255)
            cv2.rectangle(frame, (x, y), (x + bw, y + bh), col, 1)
            lbl = f"{d.get('name') or '?'} {d.get('score', 0.0):.2f}"
            cv2.putText(frame, lbl, (x, max(12, y - 6)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, col, 1, cv2.LINE_AA)
    if target:
        x, y, bw, bh, cx, cy = target
        cv2.rectangle(frame, (x, y), (x + bw, y + bh), (0, 220, 0), 2)
        cv2.line(frame, bore_px, (cx, cy), (0, 140, 255), 1)
    reticle = (0, 255, 255) if locked else (0, 0, 255)
    cv2.circle(frame, bore_px, 16, reticle, 2)
    cv2.drawMarker(frame, bore_px, reticle, cv2.MARKER_CROSS, 22, 1)
    hud = [
        f"{dname}   {fps:4.1f} fps",
        f"PAN {pan:5.1f}  TILT {tilt:5.1f}   (270deg servo {pan * 1.5:.0f}/{tilt * 1.5:.0f})",
        ("ARMED" if armed else "SAFE") + ("   LOCKED" if locked else ""),
        f"-> {cmd}",
    ]
    if id_line:
        hud.insert(1, id_line)
    if extra:
        hud.append(extra)
    for i, t in enumerate(hud):
        org = (10, 24 + i * 22)
        cv2.putText(frame, t, org, cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 3, cv2.LINE_AA)
        col = (0, 0, 255) if t.startswith("ARMED") else (60, 255, 60)
        cv2.putText(frame, t, org, cv2.FONT_HERSHEY_SIMPLEX, 0.6, col, 1, cv2.LINE_AA)
    if fire:
        cv2.putText(frame, "FIRE", (w // 2 - 60, h // 2 - 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 1.4, (0, 0, 255), 3, cv2.LINE_AA)
    cv2.putText(frame, "SPACE arm   I/J/K/L zero gun   0 reset   Q quit", (10, h - 12),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1, cv2.LINE_AA)
    return frame


def open_camera(index):
    for backend in (cv2.CAP_AVFOUNDATION, cv2.CAP_ANY):
        cap = cv2.VideoCapture(index, backend)
        if cap.isOpened():
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
            return cap
        cap.release()
    return None


def resolve_camera_name(substr):
    """Map a camera NAME substring (e.g. 'eMeet') to its OpenCV index using ffmpeg's
    AVFoundation device list, which enumerates in the same order OpenCV uses on macOS.
    Returns (index, matched_name) on success or (None, error_message)."""
    import subprocess, re
    try:
        out = subprocess.run(
            ["ffmpeg", "-f", "avfoundation", "-list_devices", "true", "-i", ""],
            capture_output=True, text=True, timeout=10).stderr
    except FileNotFoundError:
        return None, "ffmpeg not found (install: brew install ffmpeg), or use --camera <index>"
    except Exception as e:
        return None, f"could not list cameras: {e}"
    devices, in_video = [], False
    for line in out.splitlines():
        if "AVFoundation video devices" in line:
            in_video = True
            continue
        if "AVFoundation audio devices" in line:
            break
        if in_video:
            m = re.search(r"\[(\d+)\]\s+(.*\S)\s*$", line)
            if m:
                devices.append((int(m.group(1)), m.group(2)))
    for idx, name in devices:
        if substr.lower() in name.lower():
            return idx, name
    avail = ", ".join(f"[{i}] {n}" for i, n in devices) or "none detected"
    return None, f"no camera matching '{substr}'. Available: {avail}"


def open_serial(port, baud):
    try:
        import serial
    except ImportError:
        sys.exit("Serial output needs pyserial:  pip install pyserial")
    link = serial.Serial(port, baud, timeout=0)
    time.sleep(2.0)   # let the ESP32 finish resetting after the port opens
    return link


def manual(args):
    """Drive the servos BY HAND over serial, no camera. Built for hardware bring-up and zeroing.
    Uses SMOOTH continuous control (hold keys, move both axes at once) when pygame is installed,
    else falls back to the simple one-key step jogger. Needs --serial."""
    if not args.serial:
        sys.exit("--manual needs --serial <port> (it drives the real servos directly)")
    try:
        import pygame  # noqa: F401
    except ImportError:
        return _manual_cv2(args)
    return _manual_pygame(args)


def _manual_pygame(args):
    """Smooth jog: reads LIVE key-state every frame, so holding a key moves CONTINUOUSLY and you
    can hold two keys for a diagonal (both servos at once). No discrete steps = no chunkiness."""
    import pygame
    link = open_serial(args.serial, args.baud)
    pan, tilt = float(PAN_CENTER), float(TILT_CENTER)
    speed = 90.0                       # deg/s while a key is held; [ / ] adjusts
    pygame.init()
    pygame.display.set_mode((540, 168))
    pygame.display.set_caption("Sentry Turret - smooth jog")
    screen = pygame.display.get_surface()
    big = pygame.font.SysFont("menlo", 30)
    small = pygame.font.SysFont("menlo", 15)
    clock = pygame.time.Clock()
    last = None

    def send():
        nonlocal last
        cur = (round(pan, 1), round(tilt, 1))
        if cur != last:
            link.write((serial_cmd(pan, tilt, False) + "\n").encode())
            last = cur

    send()
    print("[manual] hold WASD/arrows = smooth move (multi-axis), SPACE center, [ ] speed, Q quit",
          file=sys.stderr)
    running = True
    while running:
        dt = clock.tick(60) / 1000.0
        for e in pygame.event.get():
            if e.type == pygame.QUIT:
                running = False
            elif e.type == pygame.KEYDOWN:
                if e.key in (pygame.K_q, pygame.K_ESCAPE):
                    running = False
                elif e.key == pygame.K_SPACE:
                    pan, tilt = float(PAN_CENTER), float(TILT_CENTER)
                elif e.key == pygame.K_LEFTBRACKET:
                    speed = clamp(speed - 15, 15, 200)
                elif e.key == pygame.K_RIGHTBRACKET:
                    speed = clamp(speed + 15, 15, 200)
        k = pygame.key.get_pressed()
        if k[pygame.K_a] or k[pygame.K_LEFT]:
            pan = clamp(pan - speed * dt, SERVO_MIN, SERVO_MAX)
        if k[pygame.K_d] or k[pygame.K_RIGHT]:
            pan = clamp(pan + speed * dt, SERVO_MIN, SERVO_MAX)
        if k[pygame.K_w] or k[pygame.K_UP]:
            tilt = clamp(tilt + speed * dt, SERVO_MIN, SERVO_MAX)
        if k[pygame.K_s] or k[pygame.K_DOWN]:
            tilt = clamp(tilt - speed * dt, SERVO_MIN, SERVO_MAX)
        send()
        screen.fill((24, 24, 24))
        screen.blit(big.render(f"PAN {pan:5.1f}   TILT {tilt:5.1f}", True, (0, 235, 0)), (16, 38))
        screen.blit(small.render("hold WASD/arrows  SPACE center  [ ] speed  Q quit  "
                                 f"(speed {speed:.0f}/s)", True, (185, 185, 185)), (16, 104))
        pygame.display.flip()
    link.write(b"P090 T072\n")
    link.close()
    pygame.quit()


def _manual_cv2(args):
    """Fallback one-key step jogger (used only if pygame isn't installed)."""
    link = open_serial(args.serial, args.baud)
    pan, tilt = float(PAN_CENTER), float(TILT_CENTER)
    step = 3.0

    def send():
        link.write((serial_cmd(pan, tilt, False) + "\n").encode())

    send()
    print("[manual] A/D pan, W/S tilt, SPACE center, [ / ] step, Q quit", file=sys.stderr)
    panel = np.zeros((170, 470, 3), dtype=np.uint8)
    while True:
        panel[:] = (24, 24, 24)
        cv2.putText(panel, f"PAN {pan:5.1f}   TILT {tilt:5.1f}", (16, 64),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.95, (0, 235, 0), 2)
        cv2.putText(panel, f"step {step:.0f}   A/D pan  W/S tilt  SPACE center  [ ] step  Q quit",
                    (16, 116), cv2.FONT_HERSHEY_SIMPLEX, 0.46, (185, 185, 185), 1)
        cv2.imshow("Sentry Turret - manual jog", panel)
        k = cv2.waitKey(30) & 0xFF
        if k == 255:
            continue
        if k == ord('q'):
            break
        elif k in (ord('a'), ord('A')):
            pan = clamp(pan - step, SERVO_MIN, SERVO_MAX)
        elif k in (ord('d'), ord('D')):
            pan = clamp(pan + step, SERVO_MIN, SERVO_MAX)
        elif k in (ord('w'), ord('W')):
            tilt = clamp(tilt + step, SERVO_MIN, SERVO_MAX)
        elif k in (ord('s'), ord('S')):
            tilt = clamp(tilt - step, SERVO_MIN, SERVO_MAX)
        elif k == ord(' '):
            pan, tilt = float(PAN_CENTER), float(TILT_CENTER)
        elif k == ord('['):
            step = clamp(step - 1, 1, 30)
        elif k == ord(']'):
            step = clamp(step + 1, 1, 30)
        else:
            continue
        send()
    link.write(b"P090 T072\n")
    link.close()
    cv2.destroyAllWindows()


def run(args):
    try:
        det = make_detector(args.detector, args.det_score)
    except ImportError:
        sys.exit("YOLO detectors need ultralytics:  pip install ultralytics")

    cam_index = args.camera
    if args.camera_name:
        idx, info = resolve_camera_name(args.camera_name)
        if idx is None:
            sys.exit(f"--camera-name: {info}")
        print(f"[cam] '{args.camera_name}' -> index {idx} ({info})", file=sys.stderr)
        cam_index = idx
    cap = open_camera(cam_index)
    if cap is None:
        sys.exit("Camera not available. On macOS, grant Camera access to your "
                 "terminal in System Settings > Privacy & Security > Camera.")

    link = open_serial(args.serial, args.baud) if args.serial else None
    # the camera rides ON the gun on the real turret (driven over serial) but is FIXED in the Phase 0
    # sim (the Mac webcam); the aim law in drive() must match the mount, so derive it from the link.
    on_gun = (link is not None) and not args.fixed_camera

    person_det = None
    if args.body_fusion:
        try:
            person_det = PersonDetector()
            print("[turret] body-fusion ON -> hold the lock on the body when the face turns away "
                  "(YOLO person runs only while the face is hidden).", file=sys.stderr)
        except Exception as e:
            print(f"[warn] body-fusion needs ultralytics ({e}); continuing face-only.", file=sys.stderr)

    # optional face recognition: lock onto me, or guard against everyone but me
    identifier = None
    id_mode = "me" if args.lock_me else ("not_me" if args.target_others else None)
    if id_mode:
        try:
            from face_id import FaceIdentifier, DEFAULT_THRESHOLD
        except ImportError as e:
            sys.exit(f"Recognition needs face_id.py beside this file ({e}).")
        thr = args.id_threshold if args.id_threshold is not None else DEFAULT_THRESHOLD
        identifier = FaceIdentifier(name=args.id_name, faces_dir=args.faces_dir,
                                    threshold=thr, smooth=args.id_smooth, force=args.reenroll,
                                    margin=args.id_margin)
        if args.detector in ("person",):
            print("[warn] recognition works best with the face detector; "
                  "--detector person crops whole bodies.", file=sys.stderr)
        tgt = "ME only" if id_mode == "me" else f"everyone EXCEPT {args.id_name}"
        print(f"[turret] recognition ON -> targeting {tgt}", file=sys.stderr)

    pan, tilt = float(PAN_CENTER), float(TILT_CENTER)
    pid_p = PID(args.kp, 0.0, 0.0, slew=args.max_step, sprint=args.sprint)  # pan sprints when far
    pid_t = PID(args.kp, 0.0, 0.0, slew=args.max_step)                      # tilt stays gentle (loose joint)
    armed = False
    lock_count = 0
    last_fire = last_print = 0.0
    prev_center = None
    latched = False             # recently locked on the RIGHT target via a face? (gates body-fusion)
    vel_x = vel_y = 0.0          # smoothed target pixel velocity, for predictive lead
    vlast = None                # last center used for the velocity estimate
    coast_rem = [0.0, 0.0]      # aim travel (pan, tilt deg) still owed if detection blinks mid-move
    patrol_dir = 1              # current pan sweep direction while idle-scanning
    patrol_tilt_dir = 1         # current tilt-band direction for the 2D raster scan
    last_tgt_angle = None       # (pan, tilt) where a target was last seen -> smart search starts there
    last_seen = time.time()     # when we last had a target (drives patrol)
    t_prev = time.time()
    fps = 0.0
    seen = 0
    window = not args.headless

    K, dist_coef = load_intrinsics(args.intrinsics)
    if K is not None:
        print(f"[turret] camera intrinsics loaded from {args.intrinsics} "
              f"(distortion-corrected aim)", file=sys.stderr)
    dist_ema = None             # smoothed distance estimate, meters
    dist_rate = 0.0             # smoothed d(range)/dt, m/s (approach/retreat) -- feeds the intercept solver

    def aim_to_angle(px, py, fw, fh):
        return pixel_to_angle_K(px, py, K, dist_coef) if K is not None else pixel_to_angle(px, py, fw, fh)

    while True:
        ok, frame = cap.read()
        if not ok:
            break
        frame = cv2.flip(frame, 1)            # mirror = intuitive to face
        h, w = frame.shape[:2]
        now = time.time()
        dt = now - t_prev
        t_prev = now
        fps = 0.9 * fps + 0.1 * (1.0 / dt if dt > 0 else 0.0)

        boxes = det.detect(frame)
        rows = getattr(det, "last_faces", None)
        dets = [{"box": b, "row": (rows[i] if rows is not None and i < len(rows) else None),
                 "name": None, "score": 0.0, "is_me": False} for i, b in enumerate(boxes)]
        id_line = ""
        if identifier is not None:
            for d, idr in zip(dets, identifier.classify(frame, boxes, rows)):
                d.update(idr)
            cand = [d for d in dets if (d["is_me"] if id_mode == "me" else not d["is_me"])]
        else:
            cand = dets
        target = pick_target([d["box"] for d in cand], prev_center)
        iod_px = None
        target_is_body = False
        if target is not None:                               # FACE available: precise aim + range + id
            latched = True
            trow = next((d.get("row") for d in dets if d["box"] == target[:4]), None)
            ea = face_aim(trow)
            if ea is not None:
                target = (target[0], target[1], target[2], target[3],
                          int(round(ea[0])), int(round(ea[1])))
                iod_px = ea[2]
        elif person_det is not None and len(boxes) == 0 and (id_mode is None or latched):
            # FACE genuinely hidden (no face in view, not a DIFFERENT face): hold the lock on the body.
            # YOLO runs only here, so its cost is paid only while the face is gone.
            persons = person_det.detect(frame)
            ptgt = pick_target(persons, prev_center)
            if ptgt is not None:
                bx, by = body_aim(ptgt[:4], args.aim_frac)
                target = (ptgt[0], ptgt[1], ptgt[2], ptgt[3], bx, by)
                target_is_body = True
        if target is None:
            latched = False
        fire = False
        patrolling = False
        dist = None
        cor_tilt = 0.0
        fx = float(K[0][0]) if K is not None else focal_px(w)
        if target:
            tx, ty = target[4], target[5]
            if not target_is_body and args.aim_below_face and target[3]:
                off = args.aim_below_face * target[3]
                # Cap the drop so the FACE box stays in frame at aim-equilibrium. Without this a close/
                # large face pushes the synthetic chest point off the BOTTOM of the frame; the on-gun loop
                # then drives tilt into its down-limit (60) chasing an off-screen point, and the face stays
                # half-visible so patrol never rescues it = the "sticks at tilt 60" bug. The cap only bites
                # when the face is large/close; full chest aim is preserved at normal range.
                off = min(off, max(0.0, 0.40 * h - 0.5 * target[3]))
                ty = clamp(ty + off, 0, h - 1)   # aim at the CHEST/neck, not the face
            if vlast is not None and dt > 0:                # estimate how fast it's moving
                vel_x = (1 - LEAD_SMOOTH) * vel_x + LEAD_SMOOTH * (tx - vlast[0]) / dt
                vel_y = (1 - LEAD_SMOOTH) * vel_y + LEAD_SMOOTH * (ty - vlast[1]) / dt
            vlast = (tx, ty)
            prev_center = (tx, ty)
            last_seen = now
            if args.auto_calib or args.ballistic_lead:       # range estimate (drives lead AND drop)
                if iod_px:                                   # eyes -> steadier range than the box
                    dist_raw = estimate_distance_iod(iod_px, w, args.eye_dist, focal=fx)
                elif target_is_body:                         # body box -> use shoulder width
                    dist_raw = estimate_distance(target[2], w, args.body_width, focal=fx)
                else:
                    dist_raw = estimate_distance(target[2], w, args.target_width, focal=fx)
                prev = dist_ema
                dist_ema = dist_raw if dist_ema is None else \
                    (1 - args.dist_smooth) * dist_ema + args.dist_smooth * dist_raw
                dist = dist_ema
                if prev is not None and dt > 0:              # range-rate: is the target closing or fleeing?
                    dist_rate = (1 - args.dist_smooth) * dist_rate + args.dist_smooth * (dist_ema - prev) / dt
            # lead: full fire-control intercept (couples slew time + drag flight + future range) when
            # ballistic + ranged; else the simple fixed --lead seconds.
            lead_s, d_aim = args.lead, dist
            if args.ballistic_lead and dist:
                aim_x, aim_y, lead_s, d_aim = intercept(tx, ty, vel_x, vel_y, dist, dist_rate, pan, tilt,
                                                        w, h, args.gel_mps, args.drag_k, args.slew_rate,
                                                        args.lead_latency, on_gun)
            else:
                aim_x, aim_y = lead_target(tx, ty, vel_x, vel_y, lead_s, w, h)
            tgt_pan, tgt_tilt = aim_to_angle(aim_x, aim_y, w, h)
            tgt_pan = clamp(tgt_pan + args.aim_pan, SERVO_MIN, SERVO_MAX)       # boresight zero
            tgt_tilt = clamp(tgt_tilt + args.aim_tilt, SERVO_MIN, SERVO_MAX)
            if args.auto_calib and d_aim:                    # parallax + drop, at the PREDICTED range
                v_drop = d_aim / flight_time(d_aim, args.gel_mps, args.drag_k)  # avg speed -> exact drag-aware drop
                cor_pan, cor_tilt = aim_correction(d_aim, args.cam_dy, args.cam_dx, v_drop,
                                                   elev_deg=tilt - TILT_CENTER)
                tgt_pan = clamp(tgt_pan + cor_pan, SERVO_MIN, SERVO_MAX)
                tgt_tilt = clamp(tgt_tilt + cor_tilt, SERVO_MIN, SERVO_MAX)
            pan0, tilt0 = pan, tilt
            pan, epan = drive(pid_p, pan, tgt_pan, dt, on_gun, PAN_CENTER, args.deadzone)
            tilt, etilt = drive(pid_t, tilt, tgt_tilt, dt, on_gun, TILT_CENTER, args.deadzone)
            # travel still owed after this step: if detection blinks next frame, coast FINISHES
            # this walk to the last-seen spot instead of freezing mid-stride.
            coast_rem = [epan - (pan - pan0), etilt - (tilt - tilt0)]
            # normal lock = target centered on the bore. ALSO count it locked when the gun is craned fully UP
            # against its tilt cap and the target sits just above it: a LOW camera can't tilt up enough to
            # center your face, but the bore is then resting on your CHEST = a valid body shot. This is why it
            # tracked you but never fired (tilt pinned at 85, face above reach, so it never "locked"). Pan must
            # still be centered on you. The real cure is raising the camera so tilt stops saturating.
            tilt_maxed_on_body = TILT_LIMIT_MAX <= tgt_tilt <= TILT_LIMIT_MAX + 30
            locked = abs(epan) < LOCK_TOL_DEG and (abs(etilt) < LOCK_TOL_DEG or tilt_maxed_on_body)
            last_tgt_angle = (pan, tilt)        # remember where we were aimed -> smart search on loss
            lock_count = lock_count + 1 if locked else 0
            if armed and lock_count >= FIRE_HOLD_FRAMES and now - last_fire > FIRE_COOLDOWN_S:
                fire = True
                last_fire = now
                lock_count = 0
        else:
            locked = False
            lock_count = 0
            gap = now - last_seen
            if gap < args.coast and vlast is not None:
                # brief dropout (blur/turn): do NOT extrapolate with velocity. On a LOST target the
                # velocity is stale, so `vlast + vel*gap` flung the gun to a rail (pan 170 / tilt 60)
                # every time detection blinked, which IS the wild "nodding" swing seen in the test clip.
                # But a dead FREEZE mid-swing turned a cross-room move into move-stop-move stutter, so:
                # FINISH THE WALK. Keep gliding toward the spot the target was last SEEN (coast_rem, a
                # real bounded distance, no guessing ahead), then hold there and wait for re-detect.
                dstep, coast_rem[0] = coast_step(coast_rem[0], args.kp, args.max_step, args.deadzone)
                pan = clamp(pan + dstep, SERVO_MIN, SERVO_MAX)
                dstep, coast_rem[1] = coast_step(coast_rem[1], args.kp, args.max_step, args.deadzone)
                tilt = clamp(tilt + dstep, SERVO_MIN, SERVO_MAX)
            else:
                vel_x = vel_y = 0.0
                vlast = None
                coast_rem = [0.0, 0.0]
                dist_ema = None
                dist_rate = 0.0
                pid_p.reset()
                pid_t.reset()
                if args.patrol and gap > PATROL_AFTER_S:     # idle -> HUNT for a target
                    patrolling = True
                    # smart search: for the first SEARCH_LOCAL_S after a loss, scan TIGHT around where
                    # the target vanished; then widen to a full 2D area sweep (pan AND tilt).
                    local = (gap - PATROL_AFTER_S) < SEARCH_LOCAL_S and last_tgt_angle is not None
                    if local:
                        lp, _ = last_tgt_angle
                        tilt = args.patrol_tilt          # PIN tilt to the patrol height (no dipping low)
                        pan, tilt, patrol_dir, patrol_tilt_dir = patrol_step(
                            pan, tilt, patrol_dir, patrol_tilt_dir, dt,
                            pan_min=clamp(lp - SEARCH_LOCAL_DEG, SERVO_MIN, SERVO_MAX),
                            pan_max=clamp(lp + SEARCH_LOCAL_DEG, SERVO_MIN, SERVO_MAX),
                            tilt_min=args.patrol_tilt,
                            tilt_max=args.patrol_tilt)
                    else:
                        tilt = args.patrol_tilt          # look UP at people, not down at the floor (low # = up)
                        pan, tilt, patrol_dir, patrol_tilt_dir = patrol_step(
                            pan, tilt, patrol_dir, patrol_tilt_dir, dt)

        if identifier is not None:
            tscore = next((d["score"] for d in cand if d["box"] == target[:4]), None) if target else None
            mode_txt = "LOCK ME" if id_mode == "me" else "GUARD !"
            id_line = f"{mode_txt} {identifier.name}" + (f"  {tscore:.2f}" if tscore is not None else "")

        pan = clamp(pan, PAN_LIMIT_MIN, PAN_LIMIT_MAX)    # auto-only safe limits (manual jog bypasses this)
        tilt = clamp(tilt, TILT_LIMIT_MIN, TILT_LIMIT_MAX)
        cmd = serial_cmd(pan, tilt, fire)
        if link is not None:
            link.write((cmd + "\n").encode())          # every frame for responsive tracking
        if now - last_print > 0.2:
            print(cmd, flush=True)
            last_print = now

        if window:
            # on-gun: the bore is fixed relative to the camera (rigidly attached), so it sits at the
            # boresight offset from frame center. fixed camera (sim): the bore is the virtual gun angle.
            bore_px = (angle_to_pixel(PAN_CENTER + args.aim_pan, TILT_CENTER + args.aim_tilt, w, h)
                       if on_gun else angle_to_pixel(pan, tilt, w, h))
            # live STATE readout, so the nod (if any) is visible: which mode each frame is in.
            if target:
                extra = "TRACK (body)" if target_is_body else "TRACK"
            elif patrolling:
                extra = f"PATROL {args.patrol_tilt:.0f} (scanning L-R)"
            elif (now - last_seen) < args.coast:
                extra = ("COAST (gliding to your last spot)"
                         if abs(coast_rem[0]) > args.deadzone or abs(coast_rem[1]) > args.deadzone
                         else "COAST (holding your last spot)")
            else:
                extra = "HOLD (lost you, about to patrol)"
            if args.auto_calib and dist is not None:
                extra += f"   d={dist:.1f}m  aim up {cor_tilt:+.1f}"
            elif args.aim_pan or args.aim_tilt:
                extra += f"   boresight P {args.aim_pan:+.1f} T {args.aim_tilt:+.1f}"
            draw(frame, dets, target, bore_px, pan, tilt, armed, locked, fire, cmd, fps, det.name, id_line, extra)
            cv2.imshow("Sentry Turret - brain", frame)
            k = cv2.waitKey(1) & 0xFF
            if k == ord('q'):
                break
            elif k == ord(' '):
                armed = not armed
            elif k == ord('j'):
                args.aim_pan -= 0.5          # live boresight zeroing: nudge where the gun points
            elif k == ord('l'):
                args.aim_pan += 0.5
            elif k == ord('i'):
                args.aim_tilt += 0.5
            elif k == ord('k'):
                args.aim_tilt -= 0.5
            elif k == ord('0'):
                args.aim_pan = args.aim_tilt = 0.0

        seen += 1
        if args.frames and seen >= args.frames:
            break

    if link is not None:
        link.write(b"P090 T072\n")
        link.close()
    cap.release()
    if window:
        cv2.destroyAllWindows()


def selftest():
    ok = True

    def check(name, cond):
        nonlocal ok
        print(f"[{'PASS' if cond else 'FAIL'}] {name}")
        ok = ok and cond

    p, t = pixel_to_angle(640, 360, 1280, 720)
    check("center pixel -> bore center", abs(p - PAN_CENTER) < 1e-6 and abs(t - TILT_CENTER) < 1e-6)

    pr, _ = pixel_to_angle(1280, 360, 1280, 720)
    check("right edge -> +HFOV/2", abs(pr - (PAN_CENTER + CAM_HFOV / 2)) < 1e-6)

    px = angle_to_pixel(*pixel_to_angle(900, 300, 1280, 720), 1280, 720)
    check("angle<->pixel round trip", abs(px[0] - 900) <= 1 and abs(px[1] - 300) <= 1)

    # exact tangent projection: the edge still maps to +HFOV/2, but a mid-frame offset
    # reads a LARGER angle than the old linear guess (the lens stretches angle near center)
    pmid, _ = pixel_to_angle(960, 360, 1280, 720)            # ex = +0.5
    check("tangent edge == +HFOV/2",
          abs(pixel_to_angle(1280, 360, 1280, 720)[0] - (PAN_CENTER + CAM_HFOV / 2)) < 1e-6)
    check("tangent mid-offset > old linear guess", pmid - PAN_CENTER > 0.5 * (CAM_HFOV / 2.0))

    fa = face_aim([100, 100, 40, 40, 110.0, 130.0, 150.0, 130.0, 130.0, 160.0, 115.0, 190.0, 145.0, 190.0])
    check("face_aim centroid x = face center", abs(fa[0] - 130.0) < 1e-6)
    check("face_aim centroid y = mid-face", abs(fa[1] - 160.0) < 1e-6)
    check("face_aim inter-ocular px", abs(fa[2] - 40.0) < 1e-6)
    check("face_aim eyes-only -> eye midpoint",
          abs(face_aim([0, 0, 9, 9, 110.0, 130.0, 150.0, 132.0])[0] - 130.0) < 1e-6)
    check("face_aim None without landmarks", face_aim([1, 2, 3, 4]) is None)
    check("wider eye spacing = nearer", estimate_distance_iod(200, 1280) < estimate_distance_iod(50, 1280))
    check("iod distance honors a real focal length",
          estimate_distance_iod(100, 1280, focal=1000.0) == clamp(REAL_EYE_DIST * 1000.0 / 100, DIST_MIN, DIST_MAX))

    cur, tgt = 90.0, 130.0
    pid = PID(0.30, 0.0, 0.0, 7.0)
    for _ in range(300):
        cur = clamp(cur + pid.step(tgt - cur, 1 / 30), 0, 180)
    check("PID converges to target", abs(cur - tgt) < 0.5)
    check("PID slew limited", PID(5, 0, 0, 7.0).step(1000, 1 / 30) == 7.0)

    # sprint-far / tiptoe-near: bit-identical close in, ramps beyond the kp*err==slew handoff
    sp = PID(0.10, 0, 0, 2.0, sprint=6.0)
    check("sprint: close-in identical to plain kp (err 10)", abs(sp.step(10, 1 / 30) - 1.0) < 1e-9)
    check("sprint: continuous at the handoff (err 20)", abs(sp.step(20, 1 / 30) - 2.0) < 1e-9)
    check("sprint: ramps when far (err 30 -> 4.0)", abs(sp.step(30, 1 / 30) - 4.0) < 1e-9)
    check("sprint: caps at its ceiling (err 90 -> 6.0)", abs(sp.step(90, 1 / 30) - 6.0) < 1e-9)
    check("sprint: signed (err -90 -> -6.0)", abs(sp.step(-90, 1 / 30) + 6.0) < 1e-9)
    check("sprint 0 (default-off) = the old controller exactly", PID(0.10, 0, 0, 2.0).step(90, 1 / 30) == 2.0)

    # coast finishes the walk: blind travel converges to the last-seen spot, never past it, then holds
    rem, moved = 12.0, 0.0
    for _ in range(200):
        dstep, rem = coast_step(rem, 0.10, 2.0, 1.0)
        moved += dstep
    check("coast walk converges inside the deadzone", abs(rem) <= 1.0)
    check("coast walk never overshoots the owed travel", 0 < moved <= 12.0)
    check("coast holds once inside the deadzone", coast_step(0.5, 0.10, 2.0, 1.0)[0] == 0.0)

    check("serial format", serial_cmd(94, 81, False) == "P094 T081")
    check("serial fire flag", serial_cmd(94, 81, True) == "P094 T081 FIRE")

    check("threat priority picks the closer (bigger) target", pick_target(
        [(0, 0, 50, 50), (600, 0, 160, 160)], (20, 20))[4] == 680)
    check("stickiness holds a similar-size target", pick_target(
        [(0, 0, 80, 80), (600, 0, 82, 82)], (20, 20))[4] == 40)

    lx, ly = lead_target(100, 100, 200, 0, 0.1, 1280, 720)
    check("lead aims ahead of a moving target", lx > 100 and abs(ly - 100) < 1e-6)
    check("lead 0 = aim where it is now", lead_target(100, 100, 200, 0, 0.0, 1280, 720) == (100, 100))
    check("lead clamps inside the frame", lead_target(1270, 100, 99999, 0, 1.0, 1280, 720)[0] == 1279)
    _hp, _ht, _hpd, _htd = patrol_step(PATROL_MAX - 0.1, PATROL_TILT_MIN, 1, 1, 1.0)
    check("patrol bounces pan at the high limit", _hp == PATROL_MAX and _hpd == -1)
    check("patrol holds a flat tilt (no bump = no shake)", _ht == PATROL_TILT_MIN + PATROL_TILT_STEP)
    _lp, _lt, _lpd, _ltd = patrol_step(PATROL_MIN + 0.1, 90.0, -1, 1, 1.0)
    check("patrol bounces pan at the low limit", _lp == PATROL_MIN and _lpd == 1)
    _pp, _pt, _pd2, _ptd = patrol_step(90.0, 90.0, 1, 1, 0.1)
    check("patrol sweeps pan while inside limits", PATROL_MIN < _pp < PATROL_MAX and _pd2 == 1)
    check("patrol holds tilt while pan is mid-sweep", _pt == 90.0)

    check("bigger face box = closer", estimate_distance(400, 1280) < estimate_distance(60, 1280))
    check("distance clamps to sane range",
          estimate_distance(1, 1280) == DIST_MAX and estimate_distance(10 ** 6, 1280) == DIST_MIN)
    check("parallax correction shrinks with distance",
          aim_correction(0.5, cam_dy=0.05, gel_mps=1e9)[1] > aim_correction(6.0, cam_dy=0.05, gel_mps=1e9)[1])
    check("gel drop correction grows with distance",
          aim_correction(6.0, cam_dy=0.0)[1] > aim_correction(0.5, cam_dy=0.0)[1])
    check("steep shots scale drop down (rifleman's rule for squatting/elevation)",
          aim_correction(5.0, cam_dy=0.0, elev_deg=0)[1] > aim_correction(5.0, cam_dy=0.0, elev_deg=45)[1])

    # drag-aware time-of-flight (#2 / dynamic speed): no-drag is exactly d/v0; drag makes it longer
    check("no-drag flight time = range/speed", abs(flight_time(4.0, 40, 0.0) - 4.0 / 40) < 1e-9)
    check("drag lengthens flight time", flight_time(6.0, 40, 0.05) > flight_time(6.0, 40, 0.0))
    check("flight time grows with range", flight_time(6.0, 40, 0.04) > flight_time(1.0, 40, 0.04))
    check("bead speed decays with range under drag", speed_at(6.0, 40, 0.05) < speed_at(1.0, 40, 0.05))
    check("no drag = constant speed", abs(speed_at(8.0, 40, 0.0) - 40) < 1e-9)

    # fire-control intercept: couples slew time + drag flight + future range, re-solved to a fixed point
    sx, sy, sT, sd = intercept(640, 360, 0, 0, 3.0, 0.0, 90, 90, 1280, 720, 40, 0.0, 130, 0.0, False)
    check("intercept of a still target aims at it", abs(sx - 640) < 1 and abs(sy - 360) < 1)
    mx, _, mT, _ = intercept(640, 360, 300, 0, 3.0, 0.0, 90, 90, 1280, 720, 40, 0.0, 130, 0.0, False)
    check("intercept leads a rightward mover to the right", mx > 640)
    _, _, near_T, _ = intercept(640, 360, 300, 0, 1.0, 0.0, 90, 90, 1280, 720, 40, 0.0, 130, 0.0, False)
    _, _, far_T, _ = intercept(640, 360, 300, 0, 8.0, 0.0, 90, 90, 1280, 720, 40, 0.0, 130, 0.0, False)
    check("intercept lead grows with range (longer flight)", far_T > near_T)
    _, _, slow_T, _ = intercept(900, 360, 300, 0, 4.0, 0.0, 90, 90, 1280, 720, 40, 0.0, 40, 0.0, False)
    _, _, fast_T, _ = intercept(900, 360, 300, 0, 4.0, 0.0, 90, 90, 1280, 720, 40, 0.0, 400, 0.0, False)
    check("intercept accounts for slew time (slower gun -> more lead)", slow_T > fast_T)
    _, _, _, close_d = intercept(640, 360, 0, 0, 5.0, -2.0, 90, 90, 1280, 720, 40, 0.0, 130, 0.0, False)
    check("intercept future range tracks a closing target", close_d < 5.0)

    # body aim point (#6): horizontal center, frac of the way down from the box top
    bax, bay = body_aim((100, 200, 40, 160), 0.25)
    check("body aim x = box center", bax == 120)
    check("body aim y = frac down from the top", bay == 240)

    # control law must match the camera mount. Simulate one pan axis closing on an off-center target.
    def _sim(on_gun, fixed_cam, target_world, steps=500):
        pidx = PID(0.40, 0.0, 0.0, 7.0)
        servo = 90.0
        for _ in range(steps):
            beta = target_world - servo                         # target's optical offset (camera on gun)
            tgt = target_world if fixed_cam else (PAN_CENTER + beta)
            servo, _ = drive(pidx, servo, tgt, 1 / 30, on_gun, PAN_CENTER)
        return servo
    check("on-gun aim law converges ONTO an off-center target", abs(_sim(True, False, 130.0) - 130.0) < 0.5)
    check("fixed-camera sim law converges to the target angle", abs(_sim(False, True, 130.0) - 130.0) < 0.5)
    check("the OLD fixed law on camera-on-gun settles HALFWAY (the bug this fixes)",
          abs(_sim(False, False, 130.0) - (90.0 + 130.0) / 2) < 0.5)

    if os.path.exists(YUNET_PATH):
        try:
            YuNetFaceDetector(YUNET_PATH)
            check("yunet detector loads", True)
        except Exception as e:
            check(f"yunet detector loads ({e})", False)
    else:
        print("[INFO] yunet model missing, will use haar fallback")
    try:
        FaceDetector()
        check("haar fallback loads", True)
    except Exception as e:
        check(f"haar fallback loads ({e})", False)

    cap = open_camera(0)
    if cap is not None:
        r, _ = cap.read()
        cap.release()
        print(f"[INFO] camera 0 opened, frame read = {r}")
    else:
        print("[INFO] camera not accessible headless (fine - grant permission, then run live)")

    try:
        import face_id
        print("\n--- face_id (recognition) checks ---")
        ok = face_id.selftest() and ok
    except Exception as e:
        print(f"[INFO] face_id checks skipped ({e})")

    print("\nSELFTEST", "PASSED" if ok else "FAILED")
    return 0 if ok else 1


def main():
    ap = argparse.ArgumentParser(description="CV sentry turret brain")
    ap.add_argument("--detector", choices=["face", "haar", "yolo-face", "person"], default="face")
    ap.add_argument("--det-score", type=float, default=0.6,
                    help="face-detector confidence 0-1; higher = fewer false 'faces' on junk (try 0.7-0.8)")
    ap.add_argument("--patrol-tilt", type=float, default=PATROL_TILT_PARK,
                    help="tilt the flat patrol holds; per Ryan HIGH=up (85=most up, 60=down). Default 82.")
    ap.add_argument("--camera", type=int, default=0)
    ap.add_argument("--camera-name", default=None,
                    help="pick the camera by NAME substring (e.g. 'eMeet') instead of a numeric "
                         "--camera index; survives macOS camera-index reshuffling. macOS only (ffmpeg).")
    ap.add_argument("--serial", default=None, help="ESP32 serial port, e.g. /dev/cu.usbserial-XXXX")
    ap.add_argument("--fixed-camera", action="store_true",
                    help="force the FIXED-camera aim law even when driving servos (only if the camera "
                         "is NOT mounted on the gun; default assumes camera-on-gun over serial)")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--headless", action="store_true", help="no GUI window, print commands only")
    ap.add_argument("--frames", type=int, default=0, help="stop after N frames (0 = until Q)")
    ap.add_argument("--manual", action="store_true",
                    help="hand-drive the servos over serial with the keyboard (no camera); for wiring "
                         "bring-up and zeroing the boresight. A/D pan, W/S tilt, SPACE center. Needs --serial")
    ap.add_argument("--deadzone", type=float, default=3.0,
                    help="hold the servo when the aim error is within this many degrees (kills the "
                         "buzz/jitter when basically on target; 0 = off)")
    ap.add_argument("--kp", type=float, default=0.10,
                    help="aim gain: how hard it corrects. Lower = smoother/gentler but slower to "
                         "catch up; higher = snappier but shakier/overshooty. (0.10: Ryan prefers the smooth "
                         "swing; 0.13 felt choppy 7/18, reverted)")
    ap.add_argument("--max-step", type=float, default=2.0,
                    help="max degrees the aim command can move per frame. Lower = smoother, less "
                         "jerky; higher = faster but choppier. (2.0: reverted from 3.5 on 7/18, the faster "
                         "swing hurt smoothness and Ryan wanted the smooth motion back)")
    ap.add_argument("--sprint", type=float, default=6.0,
                    help="far-target speed cap, deg/frame (PAN only). When the target is way off to the "
                         "side the step ramps up to this so it crosses the room fast, then hands back to "
                         "the untouched kp/max-step glide to settle. Close-in motion is bit-identical to "
                         "before (the smooth swing is preserved). 0 = off (pre-7/19 behavior)")
    ap.add_argument("--lead", type=float, default=0.12,
                    help="seconds to lead a moving target (0 = aim where it is now)")
    ap.add_argument("--ballistic-lead", action="store_true",
                    help="lead by the bead's time-of-flight (range / --gel-mps) instead of a fixed "
                         "--lead; needs a range estimate (auto from face/body size)")
    ap.add_argument("--lead-latency", type=float, default=0.0,
                    help="fixed pipeline/processing latency (s) added to the intercept lead time")
    ap.add_argument("--slew-rate", type=float, default=133.0,
                    help="measured servo slew speed in deg/s; the intercept solver uses it to estimate "
                         "how long the gun takes to swing onto the lead point. MEASURE on hardware: "
                         "command 0->180 and time it (firmware default is ~130 deg/s)")
    ap.add_argument("--drag-k", type=float, default=0.0,
                    help="gel-bead drag constant (1/m). >0 makes speed + flight-time range-dependent "
                         "(the bead slows); ~0.04 is a physical estimate for a 7mm bead, calibrate from "
                         "2 ranges. 0 = no drag (constant speed)")
    ap.add_argument("--aim-pan", type=float, default=0.0,
                    help="boresight: degrees added to pan so the GUN hits where the camera centers")
    ap.add_argument("--aim-tilt", type=float, default=0.0,
                    help="boresight: degrees added to tilt (positive = aim higher)")
    ap.add_argument("--auto-calib", action="store_true",
                    help="distance-aware aim: correct camera-barrel parallax + gel drop per range")
    ap.add_argument("--cam-dy", type=float, default=CAM_BARREL_DY,
                    help="meters the camera sits ABOVE the barrel (measure on your build)")
    ap.add_argument("--cam-dx", type=float, default=CAM_BARREL_DX,
                    help="meters the camera is offset sideways from the barrel")
    ap.add_argument("--gel-mps", type=float, default=GEL_MPS,
                    help="gel-bead muzzle speed m/s (shots land low -> lower this)")
    ap.add_argument("--target-width", type=float, default=REAL_TARGET_WIDTH,
                    help="real width of the tracked target, meters (~0.15 face, ~0.45 body)")
    ap.add_argument("--body-fusion", action="store_true",
                    help="when the face turns away or is occluded, keep tracking the body (YOLO "
                         "person) and aim the box instead of dropping the lock to patrol")
    ap.add_argument("--body-width", type=float, default=0.45,
                    help="real shoulder width, meters, used for ranging while tracking the body")
    ap.add_argument("--aim-frac", type=float, default=0.13,
                    help="body aim height as a fraction down from the top of the person box "
                         "(0.13 ~ head, to match the face hand-off; 0.5 = center mass)")
    ap.add_argument("--aim-below-face", type=float, default=0.3,
                    help="aim this many FACE-HEIGHTS below the face center. 0.3 = jaw/upper-neck: keeps your FACE "
                         "near frame-center so the lock HOLDS. A LOW camera looking UP loses the face if it aims "
                         "lower (at the chest), which strands the gun looking away from your face = the nodding "
                         "loop. Upward shot angle + gel drop still land it on the body. Was 1.5 then 0.7; 0 = "
                         "dead-center on the face.")
    ap.add_argument("--patrol", action=argparse.BooleanOptionalAction, default=True,
                    help="sweep to scan for targets when idle (--no-patrol to disable)")
    ap.add_argument("--eye-dist", type=float, default=REAL_EYE_DIST,
                    help="real inter-ocular distance, meters, for landmark ranging (default 0.063)")
    ap.add_argument("--dist-smooth", type=float, default=0.3,
                    help="EMA weight on each new distance estimate (lower = smoother)")
    ap.add_argument("--coast", type=float, default=1.5,
                    help="seconds to keep aiming along the last velocity through a brief dropout")
    ap.add_argument("--intrinsics", default=None,
                    help="camera-calibration .npz (keys K, dist) for distortion-corrected max-accuracy aim; make one with calibrate_camera.py")
    ap.add_argument("--selftest", action="store_true", help="run logic checks and exit")

    g = ap.add_mutually_exclusive_group()
    g.add_argument("--lock-me", action="store_true",
                   help="track ONLY the recognized person (default 'ryan')")
    g.add_argument("--target-others", action="store_true",
                   help="track everyone EXCEPT the recognized person")
    ap.add_argument("--id-name", default="ryan", help="whose face is 'me' (a folder in --faces-dir)")
    ap.add_argument("--faces-dir", default=os.path.expanduser("~/room-security/known_faces"),
                    help="enrollment photos (reuses the room-security faces)")
    ap.add_argument("--id-threshold", type=float, default=0.40, help="SFace cosine match cutoff (tuned 0.40; clears Aiden 0.378)")
    ap.add_argument("--id-margin", type=float, default=0.06,
                    help="must beat the best look-alike (sibling/family) by this cosine margin to count as you; raise if a sibling still matches")
    ap.add_argument("--id-smooth", type=float, default=0.5, help="EMA weight on the new frame (1.0 = none)")
    ap.add_argument("--reenroll", action="store_true", help="rebuild embeddings even if cached")
    ap.add_argument("--id-eval", action="store_true",
                    help="score you vs every enrolled person, then exit (no camera)")

    args = ap.parse_args()
    if args.selftest:
        sys.exit(selftest())
    if args.manual:
        manual(args)
        return
    if args.id_eval:
        from face_id import FaceIdentifier, DEFAULT_THRESHOLD
        thr = args.id_threshold if args.id_threshold is not None else DEFAULT_THRESHOLD
        FaceIdentifier(name=args.id_name, faces_dir=args.faces_dir,
                       threshold=thr, force=args.reenroll).evaluate(args.faces_dir)
        return
    run(args)


if __name__ == "__main__":
    main()
