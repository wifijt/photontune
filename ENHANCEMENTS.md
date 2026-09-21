# Filed, not done

Measured, sized, and deliberately not built. Each says what it would buy and
what it would cost.

---

## 1. Frame-driven dwell — roughly halves a tune on a fast transport

`--dwell` is 4.0 s of wall clock per trial. The decision does not care about
seconds: it needs a frame count, because the gates are `rate_upper_bound(solves,
frames)` and a tag-count standard error. Time was a proxy for evidence.

Measured on this rig, both transports read simultaneously, same camera, same
moment:

```
transport        frames/12 s   rate     mean tags   multitag
websocket            105       8.8/s      2.00        100%
networktables        363      30.2/s      2.00        100%
```

Identical data, 3.4x the rate. A 4.0 s dwell yields ~35 frames on the websocket
and ~121 on NetworkTables — 86 of them surplus to the decision.

The change: collect until the frame count the slow path already yields (~35),
then stop; keep `--dwell` as a CEILING so the slow path is unchanged and a
stalled pipeline still cannot hang. `MIN_SAMPLE_FRAMES` and the existing
extension for slow transports stay exactly as they are - this trims only the
surplus.

Sized on a real run (8 trials: reference, gain 0, gain 20 and the applied-point
check, per camera):

```
now     8 x 4.0 s dwell = 32 s + 5.6 s settle + overhead = 44 s measured
after   8 x ~1.2 s      = 10 s + 5.6 s settle + overhead ~ 20 s
```

Same measurements, same confidence, same answer. Self-scaling: faster on a
roboRIO at ~43/s, and it lengthens back toward the cap on a degraded link.

Not done because 44 s is acceptable for the field workflow and the change
touches the code path every gate depends on.

## 2. Settle is 2x the measured apply latency

0.7 s per trial against a measured 0.204-0.277 s (median 0.246, n=12) for a gain
change and 0.25-0.36 s for exposure. 8 trials x 0.7 s = 5.6 s per run. Worth
~2 s. Smaller than (1) and higher risk, since settle is the ONLY thing
separating a trial from the previous setting's frames - the `sequenceID` gate
that looks like a second line of defence is inert. Do (1) first, if either.

---

# Correcting the record: why NT-server management was removed

**The stated reason in REDESIGN.md is wrong.** It says photontune must never
toggle PhotonVision's NT server because doing so destroys its own control
channel, citing `stopServer()` and `NetworkManager.reinitialize()` restarting
the web server.

That was inferred from source and never tested on hardware. The symptom it
explained - the daemon's NT status freezing during a run and never resuming -
turned out to be something else entirely: importing `photonlibpy` inside the
first tune moved `wpi::Now()` from epoch microseconds to process-relative
microseconds, and ntcore silently drops any write whose timestamp is not newer
than the topic's current one. Fixed in `eae5769`; reproduced in nine lines.

Tested afterwards by the owner, by hand: `runNTServer` enabled from the
dashboard, a full two-camera tune sampling over NetworkTables, 44 s, clean
completion, nothing torn down.

So the capability was given up on a theory that explained a symptom with a
different cause. The COMPETITION rule still stands and is unrelated: a stray NT
server on the coprocessor will fight the roboRIO, so it must be off before that
Pi is on a robot. That is an argument for turning it off deliberately, not for
refusing to use it.

**Recommended practice, no code change:** turn `runNTServer` on from the
PhotonVision dashboard for bench work - one click, no API call, no NIC bounce -
and off before a competition. photontune finds the server if it is there and
falls back to the websocket if it is not.
