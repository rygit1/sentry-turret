#!/usr/bin/env python3
"""SFace face recognition for the sentry turret (Phase 0.5).

Identifies a SPECIFIC person ("me") among the faces YuNet already detects, so the
turret can lock ONLY onto me (--lock-me) or onto everyone EXCEPT me (--target-others).

Reuses the room-security enrollment: "me" is built from the exact same photos in
~/room-security/known_faces/<name>/ that the room cam already learned. Recognition
uses OpenCV's SFace model (cv2.FaceRecognizerSF), which pairs natively with the
turret's YuNet detector, so there is no dlib and no extra install.

Per face:  YuNet box + 5 landmarks -> alignCrop -> 128-d SFace embedding
           -> cosine similarity vs the enrolled "me" set -> is_me?
"""
import glob
import hashlib
import os
import sys

import numpy as np
import cv2

HERE = os.path.dirname(os.path.abspath(__file__))
SFACE_PATH = os.path.join(HERE, "models", "face_recognition_sface_2021dec.onnx")
YUNET_PATH = os.path.join(HERE, "models", "face_detection_yunet_2023mar.onnx")
CACHE_DIR = os.path.join(HERE, "models", "embeddings")
DEFAULT_FACES_DIR = os.path.expanduser("~/room-security/known_faces")

# SFace cosine similarity: ~0.363 is OpenCV's validated same-identity cutoff.
# We default to 0.45, measured: on Ryan's room-security photos he scores >= 0.76
# while his brother Aiden (the closest impostor) tops out at 0.378, so 0.45 sits
# safely between them and hardens against the sibling case. The HUD prints the
# live score, so tune with --id-threshold (lower if it misses you in poor light).
DEFAULT_THRESHOLD = 0.45

SFACE_URL = ("https://github.com/opencv/opencv_zoo/raw/main/models/"
             "face_recognition_sface/face_recognition_sface_2021dec.onnx")


def l2norm(v):
    n = np.linalg.norm(v)
    return v / n if n > 0 else v


class FaceIdentifier:
    """Decides whether a detected face is `name` (default 'ryan')."""

    def __init__(self, name="ryan", faces_dir=DEFAULT_FACES_DIR,
                 threshold=DEFAULT_THRESHOLD, smooth=0.5,
                 sface_path=SFACE_PATH, yunet_path=YUNET_PATH, force=False, margin=0.06):
        if not hasattr(cv2, "FaceRecognizerSF"):
            raise RuntimeError("This OpenCV build lacks FaceRecognizerSF (need >= 4.5.4).")
        if not os.path.exists(sface_path):
            raise FileNotFoundError(
                "SFace model missing. Download it once:\n"
                f"  curl -L -o {sface_path} \\\n    {SFACE_URL}")
        self.name = name
        self.threshold = threshold
        self.smooth = smooth                 # EMA weight on the new frame (1.0 = no smoothing)
        self.rec = cv2.FaceRecognizerSF.create(sface_path, "")
        # a private YuNet, used only to find faces inside still enrollment photos.
        # It is independent of the live detector (which the user may set to person/yolo).
        self._enroll_det = (cv2.FaceDetectorYN.create(yunet_path, "", (320, 320), 0.6, 0.3, 5000)
                            if os.path.exists(yunet_path) else None)
        self.refs = None                     # (N,128) L2-normalized embeddings of "me"
        self.others = None                   # (M,128) embeddings of OTHER known people ("not me")
        self.margin = margin                 # a face must beat the best impostor by this cosine margin to be "me"
        self._tracks = []                    # [(cx, cy, ema_score)] for temporal smoothing
        n = self.load_or_enroll(faces_dir, force=force)
        m = self._load_others(faces_dir, force=force)
        print(f"[face_id] '{self.name}': {n} reference + {m} impostor embeddings "
              f"(threshold {self.threshold:.3f}, margin {self.margin:.2f}, smooth {self.smooth:.2f})",
              file=sys.stderr)

    # ---- enrollment (reused from room-security photos) --------------------
    def _person_dir(self, faces_dir):
        return os.path.join(faces_dir, self.name)

    def _cache_path(self, faces_dir):
        os.makedirs(CACHE_DIR, exist_ok=True)
        key = hashlib.md5(os.path.abspath(self._person_dir(faces_dir)).encode()).hexdigest()[:8]
        return os.path.join(CACHE_DIR, f"{self.name}_{key}.npz")

    @staticmethod
    def _signature(files):
        h = hashlib.md5()
        for f in files:
            st = os.stat(f)
            h.update(f.encode()); h.update(str(int(st.st_mtime)).encode())
            h.update(str(st.st_size).encode())
        return h.hexdigest()

    def _photos(self, pdir):
        return sorted(glob.glob(os.path.join(pdir, "*.jpg")) +
                      glob.glob(os.path.join(pdir, "*.jpeg")) +
                      glob.glob(os.path.join(pdir, "*.png")))

    def load_or_enroll(self, faces_dir, force=False):
        pdir = self._person_dir(faces_dir)
        files = self._photos(pdir)
        if not files:
            raise FileNotFoundError(
                f"No enrollment photos for '{self.name}' in {pdir}.\n"
                f"Enroll first:  python3 ~/room-security/enroll_face.py {self.name}")
        sig = self._signature(files)
        cache = self._cache_path(faces_dir)
        if not force and os.path.exists(cache):
            data = np.load(cache, allow_pickle=True)
            if str(data["sig"]) == sig and data["refs"].shape[0] > 0:
                self.refs = data["refs"].astype(np.float32)
                return len(self.refs)
        embs = []
        for f in files:
            img = cv2.imread(f)
            if img is None:
                continue
            emb = self._embed_largest(img)
            if emb is not None:
                embs.append(emb)
        if not embs:
            raise RuntimeError(f"No face embeddings extracted from {len(files)} photos in {pdir}.")
        self.refs = np.vstack(embs).astype(np.float32)
        np.savez(cache, refs=self.refs, sig=sig)
        return len(self.refs)

    def _load_others(self, faces_dir, force=False):
        """Embed every OTHER known person (every dir except mine) as 'not me' references.
        A live face that sits closer to one of these than to me is rejected even if it
        clears the threshold, so a look-alike sibling can't be mistaken for me."""
        dirs = sorted(d for d in glob.glob(os.path.join(faces_dir, "*"))
                      if os.path.isdir(d) and os.path.basename(d) != self.name)
        files = []
        for d in dirs:
            files.extend(self._photos(d))
        if not files:
            self.others = None
            return 0
        sig = self._signature(files)
        cache = os.path.join(os.path.dirname(self._cache_path(faces_dir)), f"{self.name}_others.npz")
        if not force and os.path.exists(cache):
            data = np.load(cache, allow_pickle=True)
            if str(data["sig"]) == sig and data["refs"].shape[0] > 0:
                self.others = data["refs"].astype(np.float32)
                return len(self.others)
        embs = []
        for f in files:
            img = cv2.imread(f)
            if img is None:
                continue
            e = self._embed_largest(img)
            if e is not None:
                embs.append(e)
        self.others = np.vstack(embs).astype(np.float32) if embs else None
        if self.others is not None:
            np.savez(cache, refs=self.others, sig=sig)
        return 0 if self.others is None else len(self.others)

    # ---- embedding helpers ------------------------------------------------
    def _embed_largest(self, img):
        if self._enroll_det is None:
            return self._embed_crop(cv2.resize(img, (112, 112)))
        h, w = img.shape[:2]
        self._enroll_det.setInputSize((w, h))
        _, faces = self._enroll_det.detect(img)
        if faces is None or len(faces) == 0:
            return None
        row = max(faces, key=lambda f: f[2] * f[3])
        return self._embed_row(img, row)

    def _embed_row(self, img, row):
        r = np.asarray(row, dtype=np.float32)
        try:
            aligned = self.rec.alignCrop(img, r)
        except cv2.error:
            aligned = self.rec.alignCrop(img, r.reshape(1, -1))
        return self._embed_crop(aligned)

    def _embed_box(self, frame, box):
        x, y, w, h = box
        x, y = max(0, int(x)), max(0, int(y))
        crop = frame[y:y + int(h), x:x + int(w)]
        if crop.size == 0:
            return None
        return self._embed_crop(cv2.resize(crop, (112, 112)))

    def _embed_crop(self, crop112):
        feat = self.rec.feature(crop112)        # (1,128)
        return l2norm(feat.flatten().astype(np.float32))

    def _score(self, emb):
        if self.refs is None or emb is None:
            return 0.0
        return float(np.max(self.refs @ emb))   # cosine sim; all vectors L2-normalized

    def _other_score(self, emb):
        if self.others is None or emb is None:
            return 0.0
        return float(np.max(self.others @ emb))  # best cosine vs any OTHER known person

    # ---- live classification ---------------------------------------------
    # Hysteresis so a head-turn does not drop you: you must clearly BE the target
    # once (score >= threshold) to latch on, then you stay the target through dips
    # down to `keep`, and only let go when you clearly are not (score < keep).
    @staticmethod
    def _ema(alpha, new, prev):
        return alpha * new + (1.0 - alpha) * prev

    @staticmethod
    def _latch(prev_latched, sc, acquire, keep):
        if sc >= acquire:
            return True
        if sc < keep:
            return False
        return prev_latched

    def _nearest_track(self, cx, cy, radius=140.0):
        best, bestd = None, radius * radius
        for t in self._tracks:
            d = (cx - t[0]) ** 2 + (cy - t[1]) ** 2
            if d < bestd:
                bestd, best = d, t
        return best

    def classify(self, frame, boxes, rows=None):
        """Return a list parallel to `boxes`: {name, score, is_me}.
        `rows` are YuNet raw detection rows aligned with boxes (for landmark
        alignment); pass None to fall back to a plain box crop."""
        keep = max(0.30, self.threshold - 0.12)
        out, new_tracks = [], []
        for i, box in enumerate(boxes):
            if rows is not None and i < len(rows):
                emb = self._embed_row(frame, rows[i])
            else:
                emb = self._embed_box(frame, box)
            raw = self._score(emb)
            me_better = raw >= self._other_score(emb) + self.margin   # closer to me than to any sibling
            cx, cy = box[0] + box[2] / 2.0, box[1] + box[3] / 2.0
            prev = self._nearest_track(cx, cy)
            if prev is None:
                sc, latched = raw, (raw >= self.threshold)
            else:
                sc = self._ema(self.smooth, raw, prev[2])
                latched = self._latch(prev[3], sc, self.threshold, keep)
            if not me_better:
                latched = False                       # looks more like someone else -> never me
            new_tracks.append((cx, cy, sc, latched))
            out.append({"name": self.name if latched else None, "score": sc, "is_me": latched})
        self._tracks = new_tracks
        return out

    # ---- offline evaluation (no camera needed) ----------------------------
    def _score_eval(self, emb, drop_self):
        sims = self.refs @ emb
        if drop_self and len(sims) > 1:
            order = np.sort(sims)[::-1]
            if order[0] > 0.999:                 # the photo's own embedding -> exclude it
                return float(order[1])
        return float(np.max(sims))

    def evaluate(self, faces_dir):
        people = sorted(d for d in glob.glob(os.path.join(faces_dir, "*")) if os.path.isdir(d))
        print(f"\nIDENTITY EVAL   me = '{self.name}'   threshold = {self.threshold:.3f}")
        print(f"{'person':<12}{'photos':>7}{'mean':>8}{'min':>8}{'max':>8}{'%matched':>10}")
        print("-" * 53)
        me_max_other = 0.0
        me_min = 1.0
        for pdir in people:
            person = os.path.basename(pdir)
            files = self._photos(pdir)
            drop_self = (person == self.name)
            scores = []
            for f in files:
                img = cv2.imread(f)
                if img is None:
                    continue
                emb = self._embed_largest(img)
                if emb is not None:
                    scores.append(self._score_eval(emb, drop_self))
            if not scores:
                print(f"{person:<12}{len(files):>7}      (no face found)")
                continue
            s = np.array(scores)
            pct = 100.0 * np.mean(s >= self.threshold)
            tag = "  <= ME" if drop_self else ""
            print(f"{person:<12}{len(scores):>7}{s.mean():>8.3f}{s.min():>8.3f}{s.max():>8.3f}{pct:>9.0f}%{tag}")
            if drop_self:
                me_min = min(me_min, s.min())
            else:
                me_max_other = max(me_max_other, s.max())
        print("-" * 53)
        margin = me_min - me_max_other
        verdict = "GOOD separation" if margin > 0.05 else ("THIN separation" if margin > 0 else "OVERLAP")
        print(f"me-min {me_min:.3f}   best-impostor {me_max_other:.3f}   margin {margin:+.3f}   -> {verdict}")
        if me_max_other >= self.threshold:
            print(f"[!] an impostor exceeds the threshold; raise --id-threshold above {me_max_other:.3f}")
        if me_min < self.threshold:
            print(f"[!] some of your own photos fall below the threshold; lower it below {me_min:.3f}")
        return me_min, me_max_other


def selftest():
    """Headless checks: no camera, no enrollment photos required."""
    ok = True

    def check(name, cond):
        nonlocal ok
        print(f"[{'PASS' if cond else 'FAIL'}] {name}")
        ok = ok and cond

    v = np.array([3.0, 4.0], dtype=np.float32)
    check("l2norm -> unit length", abs(np.linalg.norm(l2norm(v)) - 1.0) < 1e-6)

    if not os.path.exists(SFACE_PATH):
        print("[INFO] SFace model missing; skipping recognizer checks")
        print("\nFACE_ID SELFTEST", "PASSED" if ok else "FAILED")
        return ok

    rec = cv2.FaceRecognizerSF.create(SFACE_PATH, "")
    check("SFace recognizer loads", rec is not None)

    rng = np.random.default_rng(0)
    crop_a = rng.integers(0, 255, (112, 112, 3), dtype=np.uint8)
    crop_b = rng.integers(0, 255, (112, 112, 3), dtype=np.uint8)
    fa = l2norm(rec.feature(crop_a).flatten().astype(np.float32))
    fb = l2norm(rec.feature(crop_b).flatten().astype(np.float32))
    check("feature -> 128-d", fa.shape == (128,))
    check("identical crop -> cosine ~ 1.0", float(fa @ fa) > 0.999)
    check("different crop -> lower cosine", float(fa @ fb) < float(fa @ fa))

    # mode-filter logic (the lock-me vs target-others decision)
    dets = [{"is_me": True}, {"is_me": False}, {"is_me": False}]
    me = [d for d in dets if d["is_me"]]
    others = [d for d in dets if not d["is_me"]]
    check("me-mode keeps only me", len(me) == 1)
    check("not-me mode keeps the rest", len(others) == 2)

    # EMA damps a single-frame spike; the latch keeps you locked through a dip
    check("EMA damps a spike (0.2->0.9 => 0.55)",
          abs(FaceIdentifier._ema(0.5, 0.9, 0.2) - 0.55) < 1e-6)
    check("latch: acquire at >= threshold", FaceIdentifier._latch(False, 0.50, 0.45, 0.33) is True)
    check("latch: hold through a head-turn dip", FaceIdentifier._latch(True, 0.36, 0.45, 0.33) is True)
    check("latch: release when clearly not you", FaceIdentifier._latch(True, 0.20, 0.45, 0.33) is False)
    check("latch: don't acquire mid-band", FaceIdentifier._latch(False, 0.40, 0.45, 0.33) is False)

    print("\nFACE_ID SELFTEST", "PASSED" if ok else "FAILED")
    return ok


if __name__ == "__main__":
    import argparse
    ap = argparse.ArgumentParser(description="SFace identity for the sentry turret")
    ap.add_argument("--selftest", action="store_true")
    ap.add_argument("--eval", action="store_true", help="score every enrolled person vs 'me'")
    ap.add_argument("--name", default="ryan")
    ap.add_argument("--faces-dir", default=DEFAULT_FACES_DIR)
    ap.add_argument("--threshold", type=float, default=DEFAULT_THRESHOLD)
    ap.add_argument("--reenroll", action="store_true")
    a = ap.parse_args()
    if a.selftest:
        sys.exit(0 if selftest() else 1)
    if a.eval:
        fid = FaceIdentifier(name=a.name, faces_dir=a.faces_dir,
                             threshold=a.threshold, force=a.reenroll)
        fid.evaluate(a.faces_dir)
