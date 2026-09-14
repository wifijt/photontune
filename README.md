# photontune

Automatic exposure tuning for [PhotonVision](https://photonvision.org) AprilTag pipelines,
scored on **detection quality** rather than image brightness — and deliberately biased
toward **short exposures** to limit motion blur.

Built for the case where there is no time at a competition to calmly tune camera settings.
A full sweep takes about 40 seconds per camera, needs no service restart, and can be
triggered from the driver station by anyone on the team.

```
exposure  1584   no tags                      <- cliff
exposure  2508   multitag  82%
exposure  3973   multitag 100%  reproj 0.178  ok
exposure  6292   multitag 100%  reproj 0.190  ok
exposure 25000   multitag 100%  reproj 0.669  ok

cliff ~3157 (bracketed), bias 1.50  ->  4735
predicted blur: 8.2 px @90deg/s, 32.9 px @360deg/s (tag edge ~70 px)
applied 4735 - verified: multitag 100%  reproj 0.131
```

## Why not just pick the middle of the range?

Because every setting above the cliff looks identical on a bench, and they are wildly
different on a moving robot.

This tool measures a **stationary** camera, so motion blur is exactly zero in everything
it can observe. Left to itself it would always drift toward longer exposures. But blur
scales linearly with exposure time:

```
blur_px = rotation_rate x exposure x focal_length

rotation rate              1000us   2000us   4000us   8000us  15000us  25000us
45 deg/s  gentle              0.9      1.7      3.5      6.9     13.0     21.7
90 deg/s  moderate            1.7      3.5      6.9     13.9     26.1     43.4
180 deg/s brisk               3.5      6.9     13.9     27.8     52.1     86.9
360 deg/s fast spin           6.9     13.9     27.8     55.6    104.2    173.7
```

(fx = 1105.9 px, 1280x800. A tag edge is only about 70 px across at 2.4 m, and good pose
accuracy rests on roughly 0.2 px of corner precision.)

A tuner that picked "the middle of the plateau" would happily choose 12000 — about 104 px
of smear on a 70 px tag during a fast spin. The tag is erased.

So `photontune` finds where detection actually *fails*, and sits a small safety factor
above that. It is correcting for a cost that is real, known in direction, and invisible
to the instrument.

## How it chooses

1. Sweeps exposure geometrically across the requested range.
2. Scores each point on **multi-tag solve rate** (>= 90%), falling back to mean tags per
   frame plus ambiguity when multi-tag is unavailable.
3. Brackets the failure cliff between the highest failing and lowest passing sample and
   takes their geometric mean. (Biasing off the shortest *tested* pass double-counts
   margin — with a log-spaced sweep that sample already sits up to one full step high.)
4. Applies `cliff x bias`, then **verifies** it, falling back to the shortest known-good
   value if the interpolated pick does not hold.

Settings are changed live over PhotonVision's websocket, so the camera never drops out.

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
# measure and report, change nothing
python3 photontune.py --host photonvision.local --dry-run

# tune every camera
python3 photontune.py --host photonvision.local

# specific cameras
python3 photontune.py --cameras Front,Back

# run it on the coprocessor itself - no laptop setup required
python3 photontune.py --host 127.0.0.1
```

Exits `0` on success, `1` if any camera failed. Dry runs never exit non-zero.

## Usage — NetworkTables daemon

Install the service, pointing `--nt-server` at the roboRIO:

```sh
sudo cp photontune.py /opt/photontune/
sudo cp photontune.service /etc/systemd/system/
sudo systemctl enable --now photontune
```

Then anyone can trigger it from the dashboard:

| Topic | Type | Meaning |
|---|---|---|
| `PhotonTune/run` | bool | set true to start; auto-clears when finished |
| `PhotonTune/busy` | bool | true while sweeping |
| `PhotonTune/progress` | double | 0..1, spans all cameras |
| `PhotonTune/ok` | bool | **go / no-go for the last run** |
| `PhotonTune/summary` | string | `OV9281=4735`, or `FAILED: ...` |
| `PhotonTune/status` | string | live line, updates each sweep point |
| `PhotonTune/heartbeat` | double | increments continuously — proves the service is alive |
| `PhotonTune/result` | string | full JSON: every sample, ranges, warnings |
| `PhotonTune/camera` | string | camera being tuned, e.g. `Front (2 of 3)` |
| `PhotonTune/referenceTag` | double | **write before running**: held tag ID, `-1` = use field tags |
| `PhotonTune/referenceRange` | double | **write before running**: string length in metres |
| `PhotonTune/holdCard` | bool | true while a card must be held steady |
| `PhotonTune/holdFor` | string | **which camera** to hold it in front of |

Press the button, watch the bar, wait for the green box.

**`heartbeat` matters.** If the daemon dies, `ok` and `busy` hold their last values
forever and the button silently does nothing. A frozen heartbeat means the service is
down — not that tuning failed. Check it before pressing.

The daemon **refuses to run while the robot is enabled** (`FMSInfo/FMSControlData` bit 0).

## Reference-tag mode

Hold a tag in front of the camera on a string of known length and tune in the pit, before
you ever get field access:

```sh
python3 photontune.py --reference-tag 8 --reference-range 3.0
```

From the dashboard, write `referenceTag` and `referenceRange` first, then press `run`.
Leave `referenceTag` at `-1` to tune against whatever field tags are in view instead.

### With more than one camera

Cameras point in different directions, so a held card is only ever visible to one of them.
The tool tunes cameras **sequentially** and tells the person holding it where to stand:

```
camera  = "Front (1 of 2)"     holdFor = "Front"
   ... 40 s ...
   move the card to 'Rear' - 8s        <- --move-pause
camera  = "Rear (2 of 2)"      holdFor = "Rear"
```

Put `holdFor` somewhere large on the dashboard. Budget roughly
`40s x cameras + 8s x (cameras - 1)`. Tune `--move-pause` to however long it actually takes
to walk between them.

Scoring switches to that tag's **detection rate** — a single held tag cannot multi-tag,
and its ambiguity is driven by viewing angle rather than exposure.

The string length matters, so the tool checks it. It measures the range independently from
tag size and your calibration, and warns if reality disagrees:

```
reference tag 8 measured at 2.53 m (string says 1.00 m)
WARNING: measured range is 253% of the stated range.
         Tuning at the wrong distance biases exposure badly -
         too close under-exposes you for real field tags.
```

A tag at 1 m is large and bright and passes at almost any exposure, so the cliff you would
measure sits far below the real one against field tags at 3–6 m. Use a representative
distance.

## Options

| Flag | Default | |
|---|---|---|
| `--host` | `photonvision.local` | PhotonVision host |
| `--cameras` | all | comma-separated nicknames |
| `--min-exposure` / `--max-exposure` | 1000 / 25000 | sweep range |
| `--steps` | 8 | sweep points |
| `--bias` | 1.5 | safety factor above the estimated cliff |
| `--gain` | unchanged | fixed gain for the sweep |
| `--dwell` / `--settle` | 2.5 / 1.5 | seconds collecting / waiting after a change |
| `--max-ambiguity` | 0.20 | ambiguity bar when multi-tag is unavailable |
| `--fx` | 1105.9 | focal length in px, for the blur estimate |
| `--reference-tag` / `--reference-range` | — | held-tag mode |
| `--dry-run`, `--json` | | |

## Limitations

- **It cannot measure motion blur.** A stationary bench is blind to the dominant cost of
  exposure. The short bias compensates for that, but the final word belongs to a moving
  robot.
- **The pass thresholds (90% solve rate, 95% reference detection) are judgement calls**
  and deserve checking against a real field.
- **It tunes exposure, not gain.** Gain is held fixed during a sweep. If nothing passes,
  raise `--gain` and run again.
- **Multi-camera is sequential**, so cameras cannot perturb each other — but N cameras
  takes N times as long.
- The cliff genuinely moves as lighting changes. Retune when conditions change materially.

## License

GPLv3. See [LICENSE](LICENSE).
