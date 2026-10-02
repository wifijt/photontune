# Known defects, the conditions that trigger them, and what to do

photontune as deployed 2026-09-19, md5 `f950fe75…`. Everything here was found by
forcing the failure on real hardware, not by reading code. If a defect is listed
as unmeasured, that is stated.

Ordered by whether it can bite you at a competition.

---

## 1. A hard kill can leave a camera blind, and a reboot will not fix it

**Condition.** `SIGKILL`, a power cut, or a parent process dying while photontune
is mid-tune. A clean stop (`systemctl stop`, Ctrl-C, SIGTERM) restores correctly
— this is only the uncatchable cases.

**What happens.** Forced: a SIGKILL timed to land during the gain-0 trial left a
camera at 863 µs / gain 0, which measured 0.34–1.70 tags and 8–11% multi-tag.
Effectively blind. **Boot does not repair it**, because boot asserts only the
structural baseline and the baseline does not own exposure or gain.

**Why it is not fixed.** 863 µs / gain 0 is a legal-looking pair — no sanity
check can distinguish it from a deliberate setting. The only real fix is
persisting an intended operating point that boot restores, which is a design
change, not a patch.

**Workaround.** If a camera stops seeing tags after an unclean shutdown, run a
tune: `python3 /opt/photontune/photontune.py`. Takes ~52 s and puts it right.
Before a match, glance at the dashboard: a camera showing no targets at a range
that should work is this.

---

## 2. The "never tune during a match" guard fails open

**Condition.** NetworkTables unreachable when a tune is triggered.

**What happens.** `robot_is_enabled` returns "not enabled" when it cannot ask.
With no NT server, the read returns its default. So the guard that is supposed
to refuse to tune during a match is **inert exactly when the link is down** —
which is a plausible field state. It is now tri-state internally (unknown is
distinguishable from false), but the decision still proceeds.

**Workaround.** Do not trigger a tune from the pit once queued. The boot path is
baseline-only and safe; it is only the on-demand tune that carries this.

---

## 3. The blur budget is right at ~5 m and optimistic beyond it

**Condition.** Tags further than about 5 m, at high angular rate.

**What we know, measured.** Tolerance is **relative to tag size**, not absolute:
`σ_fail ≈ 0.050 × tag side in px` (worst tag; mean 0.061, spread 1.39×). That is
about 0.4 of a bit-cell. Converting to linear smear, `L_fail ≈ 0.173 × side px`:

| range | 2 m | 4 m | 5 m | 7 m | 8 m |
|---|---|---|---|---|---|
| tag side, px | 91 | 46 | 36 | 26 | 23 |
| smear tolerated | 15.8 | 7.9 | **6.3** | 4.5 | 3.9 |

The default budget is **6 px**, so it is conservative inside 5 m and too loose
past it.

**What is still unmeasured.** Whether motion blur being *one-directional* buys
back a factor of two. PhotonVision's blur is isotropic and physically cannot
test it. This is the single largest remaining uncertainty and it scales every
answer the tool gives.

**Workaround.** If you rely on tags past 5 m, pass `--max-range 7` (or your real
worst case) and the budget is computed from tag size instead of a constant.
`fx` cancels out, so it is the same answer on either camera. To settle the 2×,
run `blurtest.py` with someone waving a tag.

---

## 4. Two settings only the settle timer protects

**Condition.** Always, but with margin.

**What happens.** A trial is separated from the previous setting's frames only by
`--settle` (0.7 s). Measured apply latency is 0.204–0.277 s, median 0.246 — a
**2.5× margin**, not the 3× once claimed. The `sequenceID` gate that looks like
a second line of defence is **inert**: it cannot fire, because the reader drains
continuously and sequence numbers are monotonic, so every frame reaching the
collector is already newer than the mark. It is retained as a labelled backstop.

**Workaround.** None needed at defaults. Do not lower `--settle` below ~0.5 s.

---

## 5. `rescue_tags_short` cannot fire on the case that motivated it

**Condition.** The scene is too dark for any gain, so the rescue path walks
exposure up.

**What happens.** The check compares the applied tag count against the best any
trial saw — but on a too-dark walk *every* trial is darker than the answer, so
the run never measures the count it fell short of. `reference_unusable` being
HARD is what actually closes that hole; this key is belt-and-braces that rarely
engages.

**Workaround.** None. Documented so it is not mistaken for coverage.

---

## 6. The tuned gain legitimately changes with light — this is not a bug

**Condition.** Different time of day, different room, lights on or off.

**What happens.** The same rig answered gain 40 in the morning and gain 60 that
evening. Verified as the room, not the code: replaying the recorded frame counts
through the current build reproduces the original answer in all combinations.
Within one session and one light, 14 consecutive runs gave identical answers.

**What to do.** Tune where you will play. That is the whole reason boot no longer
tunes: pit light is not field light.

---

## 7. PhotonVision can abort on a geometry-changing write

**Condition.** Writing `inputImageRotationMode`, resolution, or anything that
restarts libcamera — including one click in the dashboard. Seen once in three
days, on a JVM with 3d 14h uptime after many camera restarts.

**What happens.** `Failed to get v3d handle for dmabuf 75` → `EGL error 0x3003`
→ `std::terminate` → SIGABRT. systemd restarted PhotonVision and it came back
clean; the identical write immediately afterwards succeeded.

**Not caused by photontune** — it is a dmabuf leak in PhotonVision's libcamera
path. Attribution is plausible, not proven; it was not reproducible.

**Workaround.** If PhotonVision vanishes after a settings change, check
`systemctl status photonvision` — it has probably already restarted itself.
Reboot the Pi before an event rather than relying on a long-uptime JVM.

---

## 8. Two PhotonVision traps that will cost you an evening

**Never select pipeline index 0** if it is an unused placeholder. On this rig it
put PhotonVision into throwing `NullPointerException: "src" is null` on every
dispatch, and **it stopped broadcasting camera settings entirely**. Recovered by
selecting another pipeline; no service restart needed.

**`/api/settings/general` resets the network interface.** It is the only way to
toggle PhotonVision's NT server, and it bounces the NIC *and* restarts the web
server — dropping every websocket — regardless of `shouldManage`. It took this
Pi off the network entirely once. photontune no longer calls it at all, for
exactly this reason. If you must, do it from the console, never over the link
it is resetting, and never right before you need the Pi.

---

## 9. The survey places weakly-constrained tags with false confidence

**Condition.** A tag that is only ever visible together with one other tag.

**What happens.** Tag 3 on the bench layout is seen only with tag 2. A bundle
adjustment cannot validate a constraint it has once, but `solve_layout.py` emits
a position anyway with no signal attached. Downstream this showed up as the
two-camera mount transform bending 0.77° and 24 mm between rig positions.

The tag cannot be moved — the bench mirrors an FRC field, where FIRST fixes tag
positions. The fix belongs in the tool. Parked; see `survey/README-survey.md`.

**Workaround.** On a real field, use the official `AprilTagFieldLayout` rather
than a survey. Survey only a practice field, and distrust any tag you cannot see
alongside two others.

---

## 10. Unfiled upstream issue: streams cost a third of your framerate

PhotonVision prepares camera streams every frame **with no client connected**.
Measured on a fresh service: 60.1 → 80.5 fps, 42.0 → 32.7 ms latency, frame
period 17.42 → 8.71 ms. The dashboard cannot reach a "no streams" state.

Write-up ready at `photonvision-tools/docs/upstream-issue-stream-encoding.md`.
**Not filed** — PhotonVision's repo rules forbid an AI opening issues, so this
one is yours to post, with the AI Disclosure box ticked.

**Workaround.** `photonvision-tools/config/set_streams.py --off` between matches.

---

## 11. Exposure units differ between USB and CSI cameras, and nothing says so

**Condition:** any USB (UVC) camera. Every value photontune computes, applies and
reports is wrong by the ratio between the camera's raw unit and a microsecond -
100x for a standard UVC camera.

**What is true.** PhotonVision reads the exposure bounds straight off the V4L2
property and writes the value straight back, with **no unit conversion**, while
its own log line calls the number microseconds
(`GenericUSBCameraSettables.setExposureRaw`, v2026.3.4):

```java
int propVal = (int) MathUtil.clamp(exposureRaw, minExposure, maxExposure);
logger.debug("Setting property " + exposureAbsProp.getName() + " to " + propVal
             + " (user requested " + exposureRaw + " μs)");
```

Measured on one Pi carrying both, same OV9281 sensor behind both drivers:

```
CSI  (libcamera)  minExposureRaw 7      maxExposureRaw 80000    -> genuinely us
USB  (UVC)        exposure_time_absolute  min=1  max=5000  default=157
```

UVC specifies `CT_EXPOSURE_TIME_ABSOLUTE_CONTROL` in units of 100 us. The
`default=157` confirms it: 157 x 100 us = 15.7 ms, a conventional default,
where 157 us would be absurd and a 5 ms ceiling would be absurd for a maximum.

**Consequence.** photontune derives `t_budget` in microseconds and writes it as
`cameraExposureRaw`. On a USB camera that number is interpreted as hundreds of
microseconds:

```
intended    863.5 us  ->    6.0 px of smear at 360 deg/s   (the budget)
actual    86350.0 us  ->  600.0 px of smear
```

against a tag edge about 70 px across at 2.4 m. The tag is erased many times
over. In practice 86 ms is grossly overexposed indoors, so the saturation branch
walks exposure back down and the run still converges on something usable - which
is why this does not look like a failure. **The blur budget is simply not being
enforced, and the reported exposure is not comparable to a CSI camera's.**

photontune has no backend awareness at all - it never reads `isCSICamera`, which
PhotonVision does publish on the camera object.

**Workaround today:** on USB cameras treat the exposure number as a raw device
value, not microseconds, and do not compare it against a CSI camera's. Divide
the reported figure by 100 to get approximate microseconds before reasoning
about motion blur. `--max-blur-px` is not meaningful on USB until this is fixed.

**FIXED 2026-10-01.** The unit cannot be known with certainty - PhotonVision
accepts any of `raw_exposure_absolute`, `raw_exposure_time_absolute`, `exposure`
or `raw_Exposure`, and they do not share a scale - so photontune infers it and
says so. No camera has a maximum exposure of 20 ms, so a `maxExposureRaw` below
20000 is not microseconds; `exposure_unit_us()` returns 100 for those and 1
otherwise, `--exposure-unit-us` overrides, and a WARN-level
`exposure_units_inferred` names the assumption in the verdict so nobody compares
the printed number against a libcamera camera's by mistake.

The budget is still computed in microseconds and converted ONCE, where it is
set. Every read, write and verify downstream stays in the camera's own units,
which is why this is one conversion and not the eight the boundary would have
needed.

Verified by forcing the case, not by watching the CSI pair succeed - the CSI
pair cannot show the bug, its factor is 1. `sabotage_test.py --exposure-units`,
7 cases:

```
USB OV9281, UVC - 100 us per unit      unit 100   writes 8.6   -> 5.56 px
CSI OV9281, libcamera                  unit   1   writes 863.5 -> 6.00 px
what the bug did:  USB wrote 863 raw as if us    -> 600.0 px of smear
```

**Residual limitation.** A UVC camera quantises exposure to 100 us steps, so the
budget can only be hit to within one step - about 12% at 863 us. The error lands
on the safe side (8 steps, not 9), but a USB camera cannot track a blur budget
as finely as a CSI one.

## Deferred, deliberately

These were found and consciously not fixed, because the tune's correctness was
prioritised over crash edge cases:

- ~~the daemon's NT status may still freeze for the duration of a run~~ **FIXED
  2026-09-21, and the cause was neither of the two things guessed at here.**
  Importing `photonlibpy` pulls in `wpilib` → `hal`, and `hal/_initialize.py`
  runs `HAL_Initialize` at import time, which reinstalls `wpi::Now()` as FPGA
  time. Measured on the Pi: `ntcore._now()` goes from `1790007715324377` to
  `656`. ntcore then silently drops every write to a topic that already held a
  value stamped with the old clock — `setString()` still returns True. So
  `status`, `progress`, `summary`, `ok`, `busy` and `run` froze the instant the
  first tune reached `_open_readers`, while `camera` and `result`, written for
  the first time after it, kept working. The daemon now loads that decoder
  before it publishes anything, and `NTOut` stamps every write with a
  forced-monotonic timestamp. `sabotage_test.py --nt-clock` forces the jump.
- `Photon.cameras()` waits its full 12 s timeout against a PhotonVision that
  connects but never broadcasts; the boot probe uses 3 s
- `--verdict-matrix` and `--cli-smoke` do not cover this round's behaviour
  changes: four deliberate reversions of the headline fixes all scored green.
  `--nt-clock` is the exception — three planted reversions of the NT fix
  (the whole old file, `NTOut` without its timestamp, and the decoder load
  moved back after the NT client) each exit 1 against it
