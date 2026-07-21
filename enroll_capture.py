#!/usr/bin/env python3
"""Capture eMeet face photos for enrollment (pose front / left / right / up / down).

Saves frames into <faces-dir>/<name>/ so `turret_brain.py --reenroll` picks them up.
Only saves frames where a face is actually detected, so the reference set stays clean.
RUN FROM YOUR OWN Terminal (needs camera permission), e.g.:

    cd ~/sentry-turret && .venv/bin/python enroll_capture.py --camera 0
"""
import argparse
import os
import sys
import time

import cv2

from turret_brain import YuNetFaceDetector, YUNET_PATH


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--camera", type=int, default=0)
    ap.add_argument("--name", default="ryan")
    ap.add_argument("--faces-dir", default=os.path.expanduser("~/room-security/known_faces"))
    ap.add_argument("--count", type=int, default=24, help="how many photos to capture")
    ap.add_argument("--interval", type=float, default=0.7, help="seconds between auto-snaps")
    ap.add_argument("--det-score", type=float, default=0.55, help="min face confidence to save a frame")
    args = ap.parse_args()

    outdir = os.path.join(args.faces_dir, args.name)
    os.makedirs(outdir, exist_ok=True)
    existing = len([f for f in os.listdir(outdir) if f.lower().endswith((".jpg", ".jpeg", ".png"))])

    det = YuNetFaceDetector(YUNET_PATH, args.det_score)

    # identity guard: when this person already has a profile, auto-snap refuses frames that
    # don't match it (same is-it-really-them rule the turret uses, margin veto included).
    # This is what was missing on 6/28, when a capture session for one brother auto-saved
    # the other brother's face into his folder and poisoned recognition for three weeks.
    ident = None
    try:
        from face_id import FaceIdentifier
        ident = FaceIdentifier(name=args.name, faces_dir=args.faces_dir, threshold=0.40, smooth=1.0)
        print(f"[enroll] guard ON: auto-snap only saves faces that match the existing "
              f"'{args.name}' profile (SPACE still force-saves)")
    except Exception as e:
        print(f"[enroll] no usable '{args.name}' profile yet ({e}); saving unguarded")

    cap = cv2.VideoCapture(args.camera)
    if not cap.isOpened():
        sys.exit(f"could not open camera {args.camera} (is the eMeet plugged in? run camera_probe.py)")

    poses = ["FACE FORWARD", "turn LEFT a bit", "turn RIGHT a bit",
             "look UP (standing)", "chin DOWN a bit", "FORWARD again"]
    print(f"[enroll] saving up to {args.count} clean face photos into {outdir}")
    print("[enroll] slowly follow the on-screen pose. It snaps automatically when it sees your face.")
    print("[enroll] SPACE = snap now, Q = done.")

    saved = 0
    last = 0.0
    while saved < args.count:
        ok, frame = cap.read()
        if not ok:
            continue
        now = time.time()
        boxes = det.detect(frame)
        has_face = bool(boxes)
        pose = poses[min((saved * len(poses)) // max(args.count, 1), len(poses) - 1)]

        # a frame is auto-saveable only when there is exactly ONE face and (if a profile
        # exists) it actually matches this person; anything else waits or needs SPACE.
        auto_ok = has_face
        tag = pose if has_face else "no face seen - face the camera"
        if len(boxes) > 1:
            auto_ok = False
            tag = f"{len(boxes)} faces in view - only {args.name} please"
        elif has_face and ident is not None:
            idr = ident.classify(frame, boxes, getattr(det, "last_faces", None))[0]
            if not idr["is_me"]:
                auto_ok = False
                tag = f"doesn't look like {args.name} ({idr['score']:.2f}) - not auto-saving"

        disp = frame.copy()
        for (x, y, bw, bh) in boxes:
            cv2.rectangle(disp, (int(x), int(y)), (int(x + bw), int(y + bh)),
                          (0, 255, 0) if auto_ok else (0, 0, 255), 2)
        cv2.putText(disp, f"{tag}   {saved}/{args.count}", (20, 40),
                    cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 0) if auto_ok else (0, 0, 255), 2)
        cv2.imshow("enroll  (SPACE = snap, Q = quit)", disp)

        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break
        auto = auto_ok and (now - last) >= args.interval
        if has_face and (key == ord(' ') or auto):
            fn = os.path.join(outdir, f"emeet_{existing + saved:03d}.jpg")
            cv2.imwrite(fn, frame)
            saved += 1
            last = now

    cap.release()
    cv2.destroyAllWindows()
    print(f"[enroll] saved {saved} photos to {outdir}")
    print("[enroll] now run turret_brain.py ONCE with --reenroll to rebuild your face profile.")


if __name__ == "__main__":
    main()
