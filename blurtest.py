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

HOST = sys.argv[1] if len(sys.argv) > 1 else "192.168.1.202"
TAG  = int(sys.argv[2]) if len(sys.argv) > 2 else 6
EXP  = float(sys.argv[3]) if len(sys.argv) > 3 else None   # us, for the blur figure
DUR  = float(sys.argv[4]) if len(sys.argv) > 4 else 30.0

inst = ntcore.NetworkTableInstance.getDefault()
inst.startClient4("blurtest")
inst.setServer(HOST, ntcore.NetworkTableInstance.kDefaultPort4)
cam = PhotonCamera("OV9281")

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
