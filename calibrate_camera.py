#!/usr/bin/env python3
"""One-time camera calibration for the sentry turret (the max-accuracy aim path).

Print a checkerboard (default 9x6 INNER corners), hold it at many angles + distances in
front of the SAME webcam the turret uses, and press SPACE to capture each frame (grab
15-25). It solves the camera matrix K + lens-distortion coefficients and saves them to
models/intrinsics.npz, which:

  .venv/bin/python turret_brain.py --intrinsics models/intrinsics.npz ...

then uses for distortion-corrected aiming and true-focal-length ranging. It also prints
the measured HFOV/VFOV, so even without --intrinsics you can paste those into
CAM_HFOV/CAM_VFOV for a better FOV model.

  .venv/bin/python calibrate_camera.py --camera 1 --cols 9 --rows 6 --square 0.025
"""
import argparse
import os
import sys

import numpy as np
import cv2

HERE = os.path.dirname(os.path.abspath(__file__))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--camera", type=int, default=0)
    ap.add_argument("--cols", type=int, default=9, help="inner corners across the board")
    ap.add_argument("--rows", type=int, default=6, help="inner corners down the board")
    ap.add_argument("--square", type=float, default=0.025, help="checker square size, meters")
    ap.add_argument("--out", default=os.path.join(HERE, "models", "intrinsics.npz"))
    ap.add_argument("--need", type=int, default=15, help="captures wanted before solving")
    a = ap.parse_args()

    pattern = (a.cols, a.rows)
    objp = np.zeros((a.cols * a.rows, 3), np.float32)
    objp[:, :2] = np.mgrid[0:a.cols, 0:a.rows].T.reshape(-1, 2) * a.square
    objpoints, imgpoints = [], []

    cap = cv2.VideoCapture(a.camera, cv2.CAP_AVFOUNDATION) if sys.platform == "darwin" \
        else cv2.VideoCapture(a.camera)
    if not cap.isOpened():
        cap = cv2.VideoCapture(a.camera)
    if not cap.isOpened():
        sys.exit("camera not available")
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

    print(f"SPACE = capture when the board outline is GREEN, Q = solve + quit. Need ~{a.need}.")
    shape = None
    while True:
        ok, frame = cap.read()
        if not ok:
            break
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        shape = gray.shape[::-1]                       # (w, h)
        found, corners = cv2.findChessboardCorners(gray, pattern, None)
        view = frame.copy()
        if found:
            cv2.drawChessboardCorners(view, pattern, corners, found)
        cv2.putText(view, f"{len(objpoints)}/{a.need}", (10, 34),
                    cv2.FONT_HERSHEY_SIMPLEX, 1.1, (0, 220, 0) if found else (0, 0, 255), 2)
        cv2.imshow("calibrate (SPACE capture, Q solve)", view)
        k = cv2.waitKey(1) & 0xFF
        if k == ord('q'):
            break
        if k == ord(' ') and found:
            corners = cv2.cornerSubPix(
                gray, corners, (11, 11), (-1, -1),
                (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001))
            objpoints.append(objp.copy())
            imgpoints.append(corners)
            print("captured", len(objpoints))
    cap.release()
    cv2.destroyAllWindows()

    if len(objpoints) < 5:
        sys.exit("need at least ~5 good captures; run again")
    rms, K, dist, _, _ = cv2.calibrateCamera(objpoints, imgpoints, shape, None, None)
    os.makedirs(os.path.dirname(a.out), exist_ok=True)
    np.savez(a.out, K=K, dist=dist)

    fx, fy, cx, cy = K[0, 0], K[1, 1], K[0, 2], K[1, 2]
    hfov = 2 * np.degrees(np.arctan(shape[0] / (2 * fx)))
    vfov = 2 * np.degrees(np.arctan(shape[1] / (2 * fy)))
    print(f"\nsaved {a.out}   (reprojection RMS {rms:.3f} px; under ~0.5 is good)")
    print(f"fx={fx:.1f} fy={fy:.1f} cx={cx:.1f} cy={cy:.1f}")
    print(f"measured HFOV={hfov:.1f} VFOV={vfov:.1f}  (you can also paste these as CAM_HFOV/CAM_VFOV)")
    print(f"\nUse it:  .venv/bin/python turret_brain.py --intrinsics {a.out} ...")


if __name__ == "__main__":
    main()
