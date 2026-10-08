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

Run it from a laptop over the network, or on the coprocessor itself. It talks to
PhotonVision over its websocket — nothing to build, no PhotonVision changes.

---

## Install

### On a laptop

```sh
pip install msgpack websockets
python3 photontune.py --host photonvision.local
```

That is everything for CLI use. `pyntcore` and `photonlibpy` are optional here (see
below); without them, sampling uses the websocket.

### On the coprocessor — a PhotonVision Raspberry Pi image

**None of these ship on the image**, including `pip3` itself, and the package lists are
stale on a fresh boot. Verified on PhotonVision v2026.3.4 / Debian 12 / Python 3.11:

```sh
sudo apt-get update
sudo apt-get install -y --no-install-recommends \
    python3-msgpack python3-websockets python3-pip
sudo pip3 install --break-system-packages pyntcore photonlibpy
```

| package | needed for | if missing |
|---|---|---|
| `msgpack`, `websockets` | everything | exits immediately |
| `pyntcore` | daemon mode | `--daemon` exits with a message; CLI is unaffected |
| `photonlibpy` | decoding detections **off NetworkTables** | logs it and falls back to the websocket — **4.8x fewer frames per trial** |

`photonlibpy` is easy to skip and costs the most: over NT the tool samples ~43 results/s
against ~9 on the websocket, so every trial is built on 4.8x the evidence.

Then install the files:

```sh
sudo mkdir -p /opt/photontune
sudo cp photontune.py sabotage_test.py /opt/photontune/
```

Check it runs before making it a service:

```sh
python3 /opt/photontune/photontune.py --host 127.0.0.1 --baseline-only
```

### As a service

```sh
sudo cp photontune.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable --now photontune
journalctl -u photontune -f
```

**Edit `--nt-server` in the unit first.** It must point at a NetworkTables server that
already exists — the roboRIO on a robot (`--team <number>`, or `10.TE.AM.2`). photontune
never starts one. On a bench with no roboRIO, either point it at PhotonVision's own NT
server (`127.0.0.1`, if you have enabled `runNTServer` in the dashboard) or leave it and
accept the websocket fallback.

At startup the daemon asserts the baseline on every camera and stops — seconds, and it
cannot leave a camera mid-sweep. It does **not** tune at boot; see below.

---

## Usage — CLI

```sh
# tune every camera
python3 photontune.py --host photonvision.local

# specific cameras
python3 photontune.py --cameras Front,Back

# assert the structural settings and stop
python3 photontune.py --baseline-only

# on the coprocessor itself
python3 photontune.py --host 127.0.0.1
```

Exits `0` only if every camera ended in the state the run says it did. **No path exits 0
on a failure the tool detected** — asserted per problem by `sabotage_test.py
--verdict-matrix`, which runs the real `main()` in a subprocess and checks the exit status.

## Usage — NetworkTables daemon

Set `PhotonTune/run` true from your dashboard.

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

The daemon refuses to start a tune while the robot is enabled (`FMSInfo/FMSControlData`
bit 0), and **re-checks between every step during one**, aborting with a restore if the
robot goes enabled mid-tune.

### Why it does not tune at boot

- PhotonVision persists every websocket write to SQLite, so tune-once-and-persist is the
  platform's own default. Re-deriving the same answer every power cycle buys nothing.
- The boot tune this replaces spent ~131 s driving both cameras through blind exposures
  while the robot may be on the cart about to be enabled.
- Lighting differs between the pit and the field. **Tune where you will play.**

## Testing

```sh
python3 sabotage_test.py --photontune "python3 /opt/photontune/photontune.py --host 127.0.0.1"
python3 sabotage_test.py --verdict-matrix   # offline: exit code per recorded problem
python3 sabotage_test.py --gain-walk        # offline: recorded samples, real pass rule
python3 sabotage_test.py --sample-floor     # offline: a 3-frame sample cannot "pass"
python3 sabotage_test.py --cli-smoke        # offline: the real main(), every flag
```

The first breaks 15 settings on purpose, checks each is repaired, and leaves the cameras
as it found them. It exists because three baseline settings were sent as enum names where
PhotonVision requires ordinals, so they had never once applied — and the tool logged its
own failure and carried on.

---

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

`--max-blur-px` defaults to **6**, derived rather than measured. Sweeping PhotonVision's
own Gaussian `blur` at fixed exposure and gain, detection degrades immediately — 12% of
tags gone by sigma 0.5 — and multi-tag collapses between sigma 1.5 and 2.0. A box smear of
length L matches a Gaussian of sigma L/sqrt(12), so sigma 1.0 is about 3.5 px of smear.

That proxy **overstates the damage by roughly 2x**: Gaussian blur degrades edges in every
direction, motion smear only the ones perpendicular to the motion, and a tag has edges in
two orthogonal directions. Hence 6 px rather than 3.5.

`blurtest.py` measures the real thing but needs a human waving a tag. Two 30-second runs,
same tag on a 3 m string, same motion: at 4735 us, 99% detected while moving and 0.70 px
median blur; at 15000 us, 76% and 2.03 px. Both scored 100% on a static sweep.

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

"Significantly below" is not a fraction. A fixed fraction either lets a real loss through
or turns one frame in 25 into a different answer, depending on the reference. The test is
whether the shortfall exceeds three standard errors of *this sample's own* tag count,
computed from the per-frame counts already in hand — so it tightens with more evidence
rather than loosening.

Reprojection error is measured, printed and stored in the JSON. **It never chooses
anything.** It is OpenCV's RMS residual over the corners of the multi-tag fit: it measures
internal consistency of a fit, not pose accuracy, and it *improves* when distant tags drop
out of the solve. Optimising it rewarded losing tags — six identical runs of one camera
chose gain 60, 80, 60, 80, 100, 100.

## The baseline

Everything with one right answer is asserted before anything is measured, because tuning
on top of a crushed brightness just answers with a long, blurry exposure and calls it
success. `inputImageRotationMode=0`, `blur=0`, `cameraAutoExposure=off`, red/blue gain 0,
`targetModel=7` (6.5 in), `tagFamily=0`, plus measured defaults for `decisionMargin`,
`hammingDist`, `threads`, `decimate`, `cameraBrightness`, `numIterations`, `refineEdges`,
`doMultiTarget` and `solvePNPEnabled`. Each carries its reason in the source.

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
| `--nt-server`, `--team`, `--no-nt`, `--json` | | |

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

[DEFECTS.md](DEFECTS.md) lists what is known to be wrong, the conditions it appears
under, and the workaround for each.

## See also

[photonvision-tools](https://github.com/wifijt/photonvision-tools) — surveying a custom
AprilTag layout, scripted PhotonVision configuration, and
[SETUP.md](https://github.com/wifijt/photonvision-tools/blob/main/SETUP.md): the whole
route from a fresh PhotonVision install to a tuned rig.

## License

GPLv3. See [LICENSE](LICENSE).
