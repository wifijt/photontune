#!/usr/bin/env python3
"""Measure how much motion blur AprilTag detection actually tolerates.

Records PhotonVision's own detected corners over NetworkTables while you wave a
tag. Frame-to-frame corner displacement gives image-plane speed (px/s); multiply
by exposure to get blur (px). Then we can ask: at how many pixels of smear does
detection and pose quality actually fall apart?

This is the one thing photontune cannot see, because it measures a static camera.
"""
import sys, time, math, json
import numpy as np, ntcore
from photonlibpy.photonCamera import PhotonCamera

import argparse
_ap = argparse.ArgumentParser(
    description="Measure how much motion blur AprilTag detection actually tolerates.")
_ap.add_argument("host", nargs="?", default="photonvision.local")
_ap.add_argument("tag", nargs="?", type=int, default=0, help="tag ID to track")
_ap.add_argument("exposure", nargs="?", type=float, default=None,
                 help="current exposure in us, for the blur figure")
_ap.add_argument("seconds", nargs="?", type=float, default=30.0)
_ap.add_argument("--camera", default=None, help="camera nickname (default: auto-detect)")
_a = _ap.parse_args()
HOST, TAG, EXP, DUR = _a.host, _a.tag, _a.exposure, _a.seconds

inst = ntcore.NetworkTableInstance.getDefault()
inst.startClient4("blurtest")
inst.setServer(HOST, ntcore.NetworkTableInstance.kDefaultPort4)
if _a.camera is None:
    import urllib.request, json as _j, zipfile as _z, io as _io, sqlite3 as _s, tempfile as _t, os as _o
    raw = urllib.request.urlopen("http://%s:5800/api/settings/photonvision_config.zip" % HOST,
                                 timeout=60).read()
    with _t.TemporaryDirectory() as _td, _z.ZipFile(_io.BytesIO(raw)) as _zz:
        _n = [n for n in _zz.namelist() if n.endswith("photon.sqlite")][0]
        _zz.extract(_n, _td)
        _c = _s.connect(_o.path.join(_td, _n))
        _a.camera = _j.loads(_c.execute("select config_json from cameras").fetchone()[0]).get("nickname")
        _c.close()
    print("auto-detected camera: %s" % _a.camera)
cam = PhotonCamera(_a.camera)

samples = []          # (t, centroid, size_px, ambiguity, range_m)
gaps = 0              # frames where the tag vanished
frames = 0
t0 = time.time()
last_seen = None
print("recording %.0fs - wave the tag" % DUR)
while time.time() - t0 < DUR:
    r = cam.getLatestResult()
    ts = r.getTargets()
    frames += 1
    hit = None
    for t in ts:
        if int(t.fiducialId) == TAG and t.detectedCorners and len(t.detectedCorners) == 4:
            hit = t
            break
    now = time.time() - t0
    if hit is None:
        gaps += 1
        last_seen = None
    else:
        c = np.array([[p.x, p.y] for p in hit.detectedCorners])
        size = float(np.mean([np.linalg.norm(c[i] - c[(i + 1) % 4]) for i in range(4)]))
        p = hit.bestCameraToTarget
        rng = math.sqrt(p.X()**2 + p.Y()**2 + p.Z()**2) if p else float("nan")
        samples.append((now, c.mean(axis=0), size, float(hit.poseAmbiguity), rng))
    time.sleep(0.01)

print("frames %d, tag seen %d (%.0f%%), missed %d"
      % (frames, len(samples), 100.0 * len(samples) / max(frames, 1), gaps))
if len(samples) < 20:
    sys.exit("not enough detections - was the tag in view?")

# image-plane speed between consecutive detections
vel = []
for a, b in zip(samples, samples[1:]):
    dt = b[0] - a[0]
    if 0.004 < dt < 0.15:
        vel.append((b[0], np.linalg.norm(b[1] - a[1]) / dt, b[2], b[3], b[4]))
v = np.array([x[1] for x in vel])
print("\nimage-plane speed of the tag (px/s):")
for q in (50, 75, 90, 95, 99):
    print("   p%-3d %8.1f px/s" % (q, np.percentile(v, q)))
print("   max  %8.1f px/s" % v.max())

if EXP:
    blur = v * (EXP / 1e6)
    print("\nblur at exposure %.0f us  =  speed x %.4f s" % (EXP, EXP / 1e6))
    for q in (50, 90, 99):
        print("   p%-3d %6.2f px" % (q, np.percentile(blur, q)))
    print("   max  %6.2f px" % blur.max())
    # does ambiguity degrade as blur rises?
    amb = np.array([x[3] for x in vel])
    ok = amb >= 0
    if ok.sum() > 20:
        lo = blur < np.percentile(blur, 33)
        hi = blur > np.percentile(blur, 67)
        print("\n   ambiguity when nearly still (blur p<33): %.3f" % np.median(amb[ok & lo]))
        print("   ambiguity when moving fast (blur p>67) : %.3f" % np.median(amb[ok & hi]))
    rng = np.array([x[4] for x in vel])
    good = np.isfinite(rng)
    if good.sum() > 20:
        lo = blur < np.percentile(blur, 33)
        hi = blur > np.percentile(blur, 67)
        print("   range still  %.3f m   range moving %.3f m   (should be equal)"
              % (np.median(rng[good & lo]), np.median(rng[good & hi])))
json.dump([[s[0], float(s[2]), s[3], s[4]] for s in samples], open("/tmp/blur_samples.json", "w"))
