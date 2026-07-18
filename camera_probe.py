#!/usr/bin/env python3
"""Open camera indices 0..4, grab a frame from each, save to /tmp/cam_<i>.jpg.
Run it, click ALLOW on the macOS camera prompt, then run it ONCE MORE so every
index captures cleanly. Then we look at the images to find the eMeet."""
import cv2
for i in range(5):
    cap = cv2.VideoCapture(i, cv2.CAP_AVFOUNDATION)
    ok = cap.isOpened()
    frame = None
    if ok:
        for _ in range(5):           # toss a few frames so it's not black
            r, frame = cap.read()
    w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    if frame is not None:
        cv2.imwrite(f"/tmp/cam_{i}.jpg", frame)
        print(f"index {i}: OPENED  {w}x{h}  -> saved /tmp/cam_{i}.jpg", flush=True)
    else:
        print(f"index {i}: no frame (open={ok})", flush=True)
    cap.release()
print("done. If index 0 said 'no frame' the first time, run this script again now that permission is granted.")
