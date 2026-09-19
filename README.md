# photontune

Exposure and gain for [PhotonVision](https://photonvision.org) AprilTag pipelines, set
from a **motion-blur budget** rather than searched for on a bench.

> *The least gain that still sees every tag, at the exposure a spinning robot allows.*

```
── OV9281 ──  current exposure 1500.0, gain 100
   baseline: already correct
   fx 1105.9 from this camera's own calibration; 6 px at 360 deg/s allows 863 us
   reference  gain 100  exposure    863  multitag 100%  reproj   0.460  tags 3.70
       gain 0    exposure    863  multitag   5%  reproj   1.137  tags 1.09   <- saw 1.09 tags
              against the reference's 3.70 - 2.61 short, over the 0.22 that 3 sigma of this
              sample's own scatter allows, so it is a real loss and not noise
   ok  gain 20   exposure    863  multitag  98%  reproj   0.869  tags 3.93
   lowest gain that sees every tag: 20. Applying 40 - one grid step of margin.
   applied gain 40 / exposure 863 - confirmed. 6.0 px of smear at 360 deg/s (budget 6)
```

Two cameras in **43 s**, measured on a Pi 5 with two OV9281s.

## Why the exposure is not searched for

This tool measures a **stationary** camera, so motion blur is exactly zero in everything
it can observe. Every plateau it can find is a plateau in the one condition that does not
matter. Blur scales linearly with exposure:

```
blur_px = rotation_rate x exposure x focal_length

rotation rate              1000us   2000us   4000us   8000us  15000us  25000us
45 deg/s  gentle              0.9      1.7      3.5      6.9     13.0     21.7
90 deg/s  moderate            1.7      3.5      6.9     13.9     26.1     43.4
180 deg/s brisk               3.5      6.9     13.9     27.8     52.1     86.9
360 deg/s fast spin           6.9     13.9     27.8     55.6    104.2    173.7
```

(fx = 1105.9 px at 1280x800. A tag edge is only about 70 px across at 2.4 m, and good pose
accuracy rests on roughly 0.2 px of corner precision.)

A tuner that picked "the middle of the plateau" would happily choose 12000 — about 104 px
of smear on a 70 px tag during a fast spin. The tag is erased. So turn it around: the
exposure is whatever the budget allows, and the only thing left to decide is gain.

```
t_budget = max_blur_px / (radians(blur_rate) * fx)
```

`fx` comes from the camera's **own active calibration**, not from a flag — on this rig it
is 1105.9 at 1280x800 and 570.5 at 640x400, so a constant carried across a resolution
change overstates blur by 1.94x.

### The budget is a proxy, and is labelled as one

`--max-blur-px` defaults to **6**, derived rather than measured. PhotonVision's own
Gaussian `blur` swept at fixed exposure and gain:

```
blur   tags   multitag        blur   tags   multitag
0.0    3.96     100%          2.0    1.00       0%
0.5    3.47     100%          2.5    1.00       0%
1.0    2.00     100%          3.0    0.76       0%
1.5    1.50      50%          4.0    0.00       0%
```

Detection degrades immediately — 12% of tags gone by sigma 0.5 — and multi-tag collapses
between sigma 1.5 and 2.0. A box smear of length L matches a Gaussian of sigma L/sqrt(12)
by equal high-frequency attenuation, so sigma 1.0 is about 3.5 px of smear.

That proxy **overstates the damage by roughly 2x**: Gaussian blur degrades edges in every
direction, motion smear only degrades the ones perpendicular to the motion, and a tag has
edges in two orthogonal directions. Hence 6 px rather than 3.5.

`blurtest.py` measures the real thing but needs a human waving a tag. Two 30-second runs,
same tag on a 3 m string, same motion:

```
                          4735 us        15000 us
detection while moving      99%            76%      <- 553 frames lost
median blur                0.70 px        2.03 px
max blur                   8.39 px       30.74 px
ambiguity, moving          0.357          0.431
```

Both scored 100% on a static sweep. Run it when someone is available and correct the
number.

```sh
python3 blurtest.py <host> <tagId> <exposure_us> <seconds>
```

## How it chooses the gain

Per camera:

1. **Assert the baseline** — the structural settings that have one right answer
   (see below), then check the camera has a calibration for the resolution it is
   actually running.
2. **Fix the exposure** at `t_budget`, clamped to the camera's own reported
   `minExposureRaw` / `maxExposureRaw`.
3. **Reference count**: sample at the top of the gain grid. That is the best the scene can
   do without exceeding the budget.
4. **Walk gain up** through a six-point grid (0, 20, 40, 60, 80, 100), sampling each.
   A point passes if the multi-tag solve rate's upper confidence bound clears 90% **and**
   its tag count is not significantly below the reference. **Stop at the first pass and
   apply that gain plus one grid step** — a field is not the pit.
5. If the image turns out to be **saturated** (fails at gain 0, and the tag count *falls*
   as gain rises) walk exposure **down** instead. If even maximum gain cannot see the tags
   at the budget exposure, walk exposure **up** and raise an `over_blur_budget` warning:
   the answer stands, but you are outside the budget and a human has to know.
6. Write it, poll until the camera agrees, then re-read the whole camera at the end of the
   tune and check it is still where the tune left it.

Reprojection error is measured, printed and stored in the JSON. **It never chooses
anything.** It is OpenCV's RMS residual over the corners of the multi-tag fit: it measures
internal consistency of a fit, not pose accuracy, and it *improves* when distant tags drop
out of the solve. Optimising it rewarded losing tags; six identical runs of one camera
chose gain 60, 80, 60, 80, 100, 100.

### "Significantly below", not a fraction

A fraction cannot decide whether a tag count dropped. At 0.85 a reference of 4.00 sets the
floor at 3.40, so a real loss to 3.73 passes; at 0.96 a reference of 2.00 sets it at 1.92
and a reading of exactly 1.92 — one frame in 25 short — changed the answer. The test is
whether the shortfall exceeds three standard errors of *this sample's own* tag count,
computed from the per-frame counts already in hand. It tightens with more evidence rather
than loosening.

## The baseline

Everything with one right answer is asserted before anything is measured, because tuning
on top of a crushed brightness just answers with a long, blurry exposure and calls it
success. `inputImageRotationMode=0`, `blur=0`, `cameraAutoExposure=off`, red/blue gain 0,
`targetModel=7` (6.5 in), `tagFamily=0`, plus measured defaults for `decisionMargin`,
`hammingDist`, `threads`, `decimate`, `cameraBrightness`, `numIterations`, `refineEdges`,
`doMultiTarget` and `solvePNPEnabled`. Each carries its reason in the source.

Three of those had never once applied, for two weeks, because they were sent as enum names
where PhotonVision requires ordinals — and the tool logged its own failure and carried on.
`sabotage_test.py` exists for that reason: it breaks 15 settings on purpose and checks each
one is repaired.

```sh
python3 sabotage_test.py --photontune "python3 /opt/photontune/photontune.py --host 127.0.0.1"
python3 sabotage_test.py --verdict-matrix   # offline: exit code per recorded problem
python3 sabotage_test.py --gain-walk        # offline: recorded samples, real pass rule
python3 sabotage_test.py --sample-floor     # offline: a 3-frame sample cannot "pass"
python3 sabotage_test.py --cli-smoke        # offline: the real main(), every flag
```

## Install

Needs Python 3 and:

```sh
pip install msgpack websockets        # both modes
pip install pyntcore                  # daemon mode only
```

On a PhotonVision Raspberry Pi image, `pip3` is not present and the package lists are
stale:

```sh
sudo apt-get update
sudo apt-get install -y --no-install-recommends python3-msgpack python3-websockets python3-pip
sudo pip3 install --break-system-packages pyntcore
```

## Usage — CLI

```sh
# tune every camera
python3 photontune.py --host photonvision.local

# specific cameras
python3 photontune.py --cameras Front,Back

# assert the structural settings and stop
python3 photontune.py --baseline-only

# run it on the coprocessor itself - no laptop setup required
python3 photontune.py --host 127.0.0.1
```

Exits `0` only if every camera ended in the state the run says it did. **No path exits 0
on a failure the tool detected** — that is asserted per problem by
`sabotage_test.py --verdict-matrix`, which runs the real `main()` in a subprocess and
checks the process exit status.

## Usage — NetworkTables daemon

```sh
sudo cp photontune.py /opt/photontune/
sudo cp photontune.service /etc/systemd/system/
sudo systemctl enable --now photontune
```

| Topic | Type | Meaning |
|---|---|---|
| `PhotonTune/run` | bool | set true to start; auto-clears when finished |
| `PhotonTune/busy` | bool | true while tuning |
| `PhotonTune/progress` | double | 0..1, spans all cameras |
| `PhotonTune/ok` | bool | **go / no-go for the last run** |
| `PhotonTune/summary` | string | `OV9281=863@g40`, or `FAILED: ...` |
| `PhotonTune/warnings` | string | not failures, but a human must be told |
| `PhotonTune/status` | string | live line |
| `PhotonTune/heartbeat` | double | increments continuously — proves the service is alive |
| `PhotonTune/result` | string | full JSON: every trial, every problem |
| `PhotonTune/camera` | string | camera being tuned, e.g. `Front (2 of 3)` |
| `PhotonTune/bootBaselineRan` | bool | the boot baseline actually executed |
| `PhotonTune/bootBaselineOk` | bool | it succeeded on **every** camera |
| `PhotonTune/bootBaselineSummary` | string | what it changed, or why it did not run |

**`heartbeat` matters.** If the daemon dies, `ok` and `busy` hold their last values forever
and the button silently does nothing. A frozen heartbeat means the service is down — not
that tuning failed. Check it before pressing.

### Boot asserts the baseline; tuning is on demand

At startup the daemon waits for PhotonVision to answer, asserts the baseline on every
camera, and stops. That takes seconds and cannot leave a camera mid-search.

It does **not** tune at boot, deliberately:

- PhotonVision persists every websocket write to SQLite, so tune-once-and-persist is the
  platform's own default. Re-deriving the same answer every power cycle buys nothing.
- The boot tune this replaces spent ~131 s driving both cameras through blind exposures
  while the robot may be on the cart about to be enabled.
- Lighting differs between the pit and the field. Tune where you will play.

The daemon refuses to start a tune while the robot is enabled (`FMSInfo/FMSControlData`
bit 0), and **re-checks between every step during one**, aborting with a restore if the
robot goes enabled mid-tune.

### photontune never starts a NetworkTables server

`--nt-server` points at one that already **exists** — the roboRIO on a robot. Toggling
PhotonVision's own `runNTServer` posts to `/api/settings/general`, which calls
`stopServer()` (orphaning every NT client) *and* `NetworkManager.reinitialize()`, which
restarts the web server and drops every websocket regardless of `shouldManage`. The tool
used to do that twice per run and destroy its own control channel.

With no NT server reachable, sampling falls back to PhotonVision's websocket at ~9–11
results/s, which the algorithm tolerates: a 4 s dwell is ~40 frames.

## Options

| Flag | Default | |
|---|---|---|
| `--host` | `photonvision.local` | PhotonVision host |
| `--cameras` | all | comma-separated nicknames |
| `--max-blur-px` | 6 | **the budget that sets the exposure.** Proxy-derived |
| `--blur-rate` | 360 | deg/s the budget is judged at |
| `--max-gain` | 100 | top of the six-point grid; also sets the margin step |
| `--dwell` / `--settle` | 4.0 / 0.7 | seconds collecting / waiting after a change |
| `--max-exposure` | 25000 | ceiling for the too-dark walk only |
| `--fx` | 1105.9 | fallback only; the camera's own calibration wins |
| `--min-tags` / `--max-ambiguity` | 2.0 / 0.20 | bars when multi-tag is unavailable |
| `--baseline-only`, `--no-baseline`, `--brightness` | | the structural settings |
| `--nt-server`, `--no-nt`, `--json` | | |

## Limitations

- **The blur budget is a proxy.** It comes from Gaussian blur on a stationary camera,
  halved by an argument about edge orientation. It is the weakest number in the tool.
  `blurtest.py` measures the real one.
- **The 90% solve-rate bar and the 3-sigma tag test are judgement calls** — better ones
  than the fractions they replace, but still calls.
- **Gain is chosen on a six-point grid**, so the answer is granular by construction. That
  is deliberate: a finer grid would decide on differences smaller than the measurement.
- **Multi-camera is sequential**, so cameras cannot perturb each other — but N cameras
  takes N times as long.
- The right answer genuinely moves as lighting changes. Retune where you will play.

## See also

[photonvision-tools](https://github.com/wifijt/photonvision-tools) — surveying a custom
AprilTag layout, and scripted PhotonVision configuration.

## License

GPLv3. See [LICENSE](LICENSE).
