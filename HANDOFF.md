# photontune — state of play

Written 2026-09-18, updated 2026-09-19 after the rebuild. Read this before
changing anything.

## Where the code is

Branch `feat/baseline-and-blur-budget`. **Commits past the last audited point are
NOT pushed.** GitHub PR #9 (and photonvision-tools #4, #5) reflect the earlier,
audited state only. Do not push a round of fixes until it has survived an
independent adversarial audit — see below for why.

## The two principles everything is judged against

1. **It should just work without human interaction.**
2. **Tune to competition-ready settings as fast as possible.**

Runtime went 233 s → 214 s → 125 s → **42 s** for a two-camera run. There is no
unattended boot tune any more; boot asserts the baseline only, in 8-10 s.

## The working loop

Write the fix spec → hand it to a fresh agent → follow it with an **independent**
adversarial agent that must force each failure itself.

This is not ceremony. Every round so far has passed the implementer's own
checking and failed the independent audit:

- Round 1 claimed 10 fixes. The audit **refuted 2**, found a **regression**, and
  found **6 paths that exited 0 on a failure the tool had itself detected** —
  including one where it logged `*** MISMATCH ***` on its own write and then
  reported success.
- An earlier round's "fix" for enum settings made the comparison accept both
  forms of the value. That silenced the symptom and left the write broken for two
  weeks.

**Only ONE agent may hold the Pi at a time.** Two agents with write access
corrupt each other's measurements; that cost a whole timing run.

## The rule that matters

**Verify a fix by FORCING the failure, never by watching a success.**

Everything that turned out to be broken had been "verified" on the happy path:
exit codes that always returned 0, a SIGTERM restore confirmed exactly once, a
baseline assertion that had never once applied. `sabotage_test.py` exists for
this reason — it breaks 15 settings on purpose and checks each is repaired.

A related trap: the **gain ratchet has now returned twice through different code
paths**. Fix that class structurally (per-camera argument state), not door by
door.

## Things that are true about the hardware, learned the hard way

- **Enums must be SENT as int ordinals**, not names. PhotonVision's
  `VisionModuleChangeSubscriber.setProperty` does `getEnumConstants()[(int) v]`.
  A name throws `ClassCastException`, which PhotonVision logs and swallows.
- **`inputImageRotationMode` must stay `DEG_0`.** Non-zero corrupts the multi-tag
  pose by ~0.4 m while the published corners stay correct — it fails silently.
  A sideways mount belongs in `robotToCamera`.
- **`cameraSettings` lags 1–2 s.** Reading a value straight back proves nothing;
  poll until it agrees or a timeout expires.
- **PhotonVision stores unclamped values** but clamps the hardware call, so a
  write past the limit reads back "verified" while the camera sits at the bound.
- **`/api/settings/general` resets the network interface.** It is the only way to
  toggle `runNTServer`, and it bounces the NIC regardless of how complete the
  body is. It also calls `stopServer()`, orphaning every NT client, and
  `NetworkManager.reinitialize()`, which drops every websocket regardless of
  `shouldManage`. **photontune no longer posts to it at all** - a tool that
  bounces its own control channel twice per run cannot be made reliable.
- **Do not select pipeline index 0** on this rig (named "UNUSED - do not
  select"). It put PhotonVision into throwing `NullPointerException: "src" is
  null` on every dispatch, and it stopped broadcasting `cameraSettings`
  entirely. Recovered by switching back, no service restart.
- `vcgencmd` does not work (`/dev/vcio` missing). Use sysfs, or PhotonVision's
  own `metrics` over the websocket, which carries `cpuTemp` and `cpuThr`.

## Measurements worth not repeating

- Setting-apply latency: **0.25–0.36 s** for exposure (six trials, timed on the
  coprocessor against PhotonVision's own capture timestamps). Gain-change
  latency was never measured — that gap is still open.
- True horizontal FOV is **69.4°**, not the 60.1° that `2·atan(w/2fx)` gives.
  These OV9281s have k1 = −0.42 and the pinhole formula invents a blind spot.
- Streams are prepared every frame with **nobody watching**, costing about a
  third of the pipeline's framerate. Write-up is in
  `photonvision-tools/docs/upstream-issue-stream-encoding.md`, still unfiled.
- Case thermals: ~7.5 °C of headroom to the Pi 5's soft limit above whatever the
  room is. The fan curve is pinned in `/boot/firmware/config.txt`.

**All gain measurements come from one dim room.** Reprojection improved
monotonically to the top of the range there, which is part of why it was
abandoned as an objective. The gain walk's answer (40 on both cameras) has
still only been validated in that one room.

## Open, not started

- MIPI/CSI camera-setup tool. The trick is `camera_auto_detect=0` plus
  `dtoverlay=ov9281,cam0` and `dtoverlay=ov9281,cam1` in
  `/boot/firmware/config.txt` — auto-detect handles the official Pi cameras and
  fails on an OV9281.
- ChArUco capture tool. A draft exists in the session scratchpad as
  `calibrate3.py` (guided, 12 phases, reports observations PhotonVision KEPT).
- `photonvision-tools/calib/calib_view.py` — mirrored calibration viewer, built
  and working, **uncommitted**. It mirrors the DISPLAY only, via CSS; the frames
  PhotonVision calibrates from are untouched.

## Still unresolved from the field survey

**Tag 3 is misplaced in the layout**, shown three independent ways: its
cross-wall solve has the worst reprojection of any configuration despite being
the easiest fit and the closest range; the two-camera mount transform has to
bend 0.77° and 24 mm between positions to reconcile it; and it puts the room at
12 ft 10¼ in against a 12 ft tape. It is tied to the layout through tag 2 alone,
and a bundle adjustment cannot check a constraint it only has once. Moving it so
a camera can see it alongside tags 7/8/9 is the fix; re-surveying from where it
sits is not.

## Round 4 outcome (2026-09-19): the audit's defects, fixed and forced

An independent adversarial audit of round 3 proved nine defects on hardware.
Eight are fixed on this branch, each forced before and after. **F7 is NOT
DONE** - see below.

### The blocker, F1: the reference measurement was contaminated

`sample()` passed `after_capture` only on the NetworkTables path and
`Photon.collect()` had no stale-frame gate at all, so deleting NT-server
management left the gate inoperative on the transport that is now the default.

Measured here before the fix, one camera: steady state 8.3 results/s, but a 4 s
read after 10 s unread yields **117** frames. In a real run the reference trial
collected **79** frames in a 4.0 s dwell against 41-48 for every trial it is
compared to. Forced two ways and both now correct:

| forced case | 9796175 | now |
|---|---|---|
| `--max-blur-px 0.05` (7 us, camera sees nothing at any gain) | reference `multitag 49%  tags 1.95` | `tags 0.00` |
| hostile pre-state, 60 us / gain 0 | reference `multitag 50%  tags 2.00` | `multitag 100%  tags 4.00` |

Frames per trial after: 37/37/35 and 36/36/34 - flat.

The fix: the websocket now has a reader task that consumes it continuously, so
nothing queues in front of a measurement, and `collect()` gates on
`sequenceID`. **The websocket carries no `captureTimestampMicros`** - verified
by dumping a message: `multitagResult, latency, fps, classNames, sequenceID,
targets`. `sequenceID` is a per-camera frame counter, measured strictly
monotonic. **Draining to empty was measured and rejected:** with two cameras
interleaved the largest gap between messages at steady state is 0.114 s, so no
`recv()` timeout separates "empty" from "idle".

**`--dwell 4.0` keeps its value and loses its justification.** It was
attributed to solve-rate statistics; it was really outlasting the backlog. The
floor is `MIN_SAMPLE_FRAMES` (20, 2.4 s at 8.3/s), but a trial landing near it
triggers the extension in `collect()` and is then judged on a different sample
size - and so a different effective gate - than its neighbours. 1.5x the floor
is 3.6 s; 4.0 is that with 10% margin.

### Stability, re-measured on the gated path

Eight consecutive two-camera runs, default flags, 43-48 s each:

- **camera 1: gain 40, 8 of 8.** camera 2: gain 40 in 7, **gain 20 in run 6**.
- Every reference read exactly `tags 2.000 / rate 1.00` or `4.000 / 1.00`,
  n = 33-38. The reference variance the audit saw (one run reading 2.850 for a
  camera reading 3.99-4.00) did **not** recur.

**The 40->20 shift survives, and it is not a measurement artefact.** Camera 2's
gain-0 trial read 1.000-1.054 tags at a 0.00-0.05 solve rate in seven runs and
**1.892 tags at 0.892** in run 6 - 0.9 tags apart, far outside sampling error on
37 frames. The scene genuinely changed: the second tag became detectable at
gain 0 for that run. The walk then correctly picked gain 0 and applied 20.

It is not a leak in the new gate either, and the frame count is what rules that
out: contamination INFLATES the count (79 against 41-48 before the fix), and
camera 2's gain-0 trial collected 37 frames in run 6 - the same 37 it collected
in runs 1-5 and 8, and 38 in run 7. A sample cannot be 89% borrowed from the
previous setting and still be exactly the right size.

**Do not make this stickier.** The fragility is in the scene - one tag sits at
the edge of detectability at gain 0 - not in the tool.

**The `--dwell 2` sensitivity is gone.** The audit found camera 2 answering
gain 20 in 3/3 runs at `--dwell 2` while the default answered 40. On the gated
path, 3/3 runs answer **gain 40**, and gain 0 reads 0.00 tags at dwell 2 exactly
as it does at dwell 4. It was the backlog: a short dwell was mostly frames from
the previous, higher-gain setting.

### The other fixes

- **F2 `over_blur_budget` is now HARD**, and the too-dark walk is capped at
  `OVER_BUDGET_LIMIT` (2.0) times the budget rather than at `--max-exposure`.
  Forced: `--max-blur-px 0.05` applied 188 us (1.3 px, 26x) exit 0, now exits 1
  with the camera put back; `--max-blur-px 0.5` applied 232 us (1.6 px) exit 0,
  now applies 109 us and exits 1. The camera is still LEFT at what the walk
  found - best available beats nothing - and the exit code says it is not the
  answer that was asked for.
- **F9 the reference is validated and the applied point is measured.** The
  reference now runs `why_failed()` against itself and drops the tag-count
  floor rather than setting it from a sample that sees nothing
  (`reference_unusable`, WARN). `tags_significantly_below` combines the
  reference's own standard error in quadrature instead of treating it as
  exact. And one trial now runs **at the gain that was applied** - nothing ever
  had. Forced with a competing writer parking the camera at 7 us after the
  final confirm: 9796175 exits 0, this exits 1 with `applied_point_failed`.
  Cost: one dwell per camera, and a two-camera run is 43-48 s.
- **F3** no exit path returns 0 with a camera it could not put back.
  daemon + SIGTERM + an unrestorable camera was exit 0, now 1; a clean daemon
  stop is 143 and `photontune.service` declares that a success.
- **F4** `NTResults.available()` reads the shutdown flag and `_open_readers`
  runs in an executor. TERM at 2 s: **18.8 s -> 0.3 s**, at 5 s: 15.7 -> 0.3,
  at 20 s: 7.2 -> 2.2. Before, the 2 s case wrote `cameraBrightness 5 -> 40`
  to the camera AFTER the signal.
- **F5** `--verdict-matrix` takes expected severity from `MATRIX_SEVERITY` in
  the test file, not from the table under test. Downgrading `apply_mismatch`
  HARD->WARN on 9796175 reports "all 14 cases correct" under the old matrix and
  `<-- WRONG` under this one.
- **F6** `--cli-smoke`'s stub answers the way `tune_camera` answers, so the
  two `--baseline-only` cases stop printing `CAM exposure -> 864, gain 40`, and
  a fourth column asserts what the output must say. Two planted regressions
  are caught that the old harness scored "all 14 correct". `sabotage_test`
  phase 1 now asserts the sabotage (15/15) instead of counting it.
- **F8** `robot_is_enabled` no longer fails open. `robot_state()` answers
  True/False/**None**, and None is reported as `robot_state_unknown` (WARN) on
  the daemon path. Verified against a disconnected ntcore client: 9796175
  returns `False` - "not enabled" - where this returns `None` and says "the
  NetworkTables client is not connected to any server".

### NOT DONE

- **F7, SIGKILL leaving a blind camera that boot does not repair.** Not
  started. The right fix is to persist the intended operating point and have
  the boot baseline restore it: a pure sanity check cannot work, because the
  state SIGKILL left (863.5 us / gain 0) is a perfectly legal-looking pair and
  only a record of what was intended can tell it apart.

### The 6 px blur budget: an unattended attempt to bound it, and why it failed

`blurscale.py` sweeps `blur` at two `decimate` settings and finds the 50%
detection point **per tag**, using the six tags in view (apparent side 45.5 to
86.3 px) as the lever. It did not settle the number, and how it failed matters:

```
sigma50 at the deployed decimate=2, all six tags, same scene, same session
    side 45.5 px -> 1.75      side 67.9 px -> 3.00
    side 54.7 px -> 3.00      side 74.9 px -> 1.63
    side 55.4 px -> 1.75      side 86.3 px -> 3.52
```

A **2.2x spread between tags at the same moment**, with no relationship to
apparent size. So the sweep the budget rests on never measured a property of
the detector - it measured whichever tag in that scene was worst. REDESIGN's
"multi-tag collapses between sigma 1.5 and 2.0" reproduces exactly, as camera 2
at 1.63, while camera 1's tags survive to 3.0.

The decimate test was inconsistent - the ratio `sigma50(dec=1)/sigma50(dec=2)`
is 2.16 for the one tag measured cleanly at both and 1.02 for the other camera
- so whether `blur` is in full-resolution or decimated pixels is **still
unresolved**, and that is a factor of two sitting directly under the budget.
Both cameras also collapse totally at exactly sigma 4.0 at decimate=1, which
looks like an implementation limit rather than anything physical.

One caution the data supports: upstream apriltag blurs only the image used for
**quad** detection and decodes bit cells from the unblurred one, which is how
tags here survive a sigma larger than their own bit cell. Motion blur gets no
such exemption. If that is right, the proxy measures the more robust half of
the detector and the 2x that raised the budget from 3.5 to 6 pushes against a
correction that probably needs to go the other way.

**6 px is unchanged and relabelled: an estimate that may be OPTIMISTIC.**
`blurtest.py`, with a human waving a tag, is still the only thing that settles
it.

## Round 3 outcome (2026-09-19): the rebuild in REDESIGN.md, implemented

`REDESIGN.md` is now the tool. Exposure is FIXED at the blur budget and gain is
the only search: measure a reference at the top of a six-point gain grid, walk
the grid up from 0, stop at the first point that sees every tag the reference
saw, apply that plus one grid step. Reprojection is reported and never used to
choose anything.

Measured on this rig, both cameras, no NetworkTables server anywhere. The
before column is `33951b3` run under exactly those conditions
(`--no-manage-nt-server`, so it does not create the server the new build
refuses to create); the 125-143 s in the round-2 notes was measured WITH
PhotonVision's NT server running, which that build started for itself:

| | before | after |
|---|---|---|
| two-camera run | **130 s**, measured here on `33951b3` | **43.5-43.7 s** |
| the answer it gives | 1500 us at gain 80 and 100 - **10.4 px of blur, over its own budget**, and the two cameras disagree in the same room | 863/860 us at gain 40 on both - 6.0 px, inside budget |
| baseline-only (the boot path) | ~131 s, a full tune | **8-10 s** |
| six consecutive runs, two cameras | gain 60/80/60/80/100/100 on one camera | **the same gain twelve times out of twelve** |
| `photontune.py` | 3229 lines | 2642 |

Deleted: the gain scan and its decision machinery, NT-server management,
held-card mode, `--dry-run`, `baseline_contradiction`, sweep narrowing and gain
escalation, `merge_failures`/`FAILURE_KEYS`, and fourteen flags. The ratchet is
closed by construction rather than by a third promise: `Config` is a frozen
dataclass, per-camera state lives on a `Search` object built inside the loop,
and `tune_camera` is a loop that returns one record.

### Two real defects, both found by forcing rather than by watching

- **A SIGTERM that did not interrupt.** Killing a live two-camera tune 22 s in
  left it running to completion - "completed in 42 s", exit 0 - in 2 of 4
  attempts at that position, in a plain shell with no test harness involved.
  22 s is the boundary between the two cameras, where the code is inside
  `Photon.confirm()` -> `cameras_fresh()`. The plausible mechanism is
  `asyncio.wait_for` converting an outer cancellation into `TimeoutError` when
  its own timeout fires in the same instant, which `except asyncio.TimeoutError`
  then swallows. **It was closed as a class, not as an instance:** both signal
  handlers now set a `SHUTDOWN` flag the moment the signal is SEEN, and every
  long-running loop asks it directly - the websocket pump, `cameras_fresh`,
  `confirm`, the NetworkTables collector (which runs in an executor thread a
  cancellation cannot reach at all), and the per-step guard in the tune. 6 of 6
  kills at the same position interrupt and restore afterwards.
  **The restore path deliberately does not consult the flag** - aborting the
  thing that puts the camera back is the opposite of what the signal is for.
- **`ss -K` is a no-op on this kernel.** It lists the socket and kills nothing,
  so "kill the websocket" tests that use it prove nothing. Kill the transport
  from inside the process instead (`self.ws.transport.abort()`); `wskill.py` in
  the session scratchpad does it.

### PhotonVision aborted itself, once

Mid-`sabotage_test`, writing `inputImageRotationMode=1` restarted libcamera and
the native layer died:

```
Failed to get v3d handle for dmabuf 75
EGL error detected on line 387: 0x3003        <- EGL_BAD_ALLOC
terminate called after throwing an instance of 'std::runtime_error'
photonvision.service: Main process exited, code=killed, status=6/ABRT
```

systemd restarted it (`NRestarts=1`) and it came back clean. **Not caused by
photontune** - any settings write that changes frame geometry restarts
libcamera, and one click in the dashboard does the same - and NOT reproducible:
the identical sabotage run immediately afterwards, including the same rotation
write and the repair back to 0, passed 16/16 with no restart. The JVM had 3d 14h
of uptime and today had put dozens of destroy/create cycles through libcamera,
so the likely cause is a dmabuf/GPU handle leak across camera restarts rather
than the rotation value. Worth watching; worth reporting upstream if it recurs.

### Honest residue

- **Gain 20 passes on both cameras by a small margin.** Over six runs the
  closest call was camera 2 at a 90% multi-tag solve rate against a 90% gate
  (it clears on the upper confidence bound). One camera drifting below that
  would move the answer from gain 40 to gain 60. The decision is stable in
  THIS light; it has not been tested in another.
- **The 6 px blur budget is still a proxy**, halved from a Gaussian-blur sweep
  by an argument about edge orientation. It is the weakest number in the tool
  and `blurtest.py` measures the real one.
- **~8 s of every run is the NetworkTables probe** finding no server (4 s per
  candidate host). It now says so. `--no-nt` skips it.
- **The whole tune is `raise_if_shutdown`-guarded but not `robot_is_enabled`-
  guarded on the CLI path**, because there is no NT client there to ask. Only
  the daemon re-checks between steps.

## Round 2 outcome (2026-09-18, late)

All 8 audit defects fixed and forced; one extra found and fixed. Two structural
changes matter more than the individual fixes:

- **A `PROBLEMS` registry IS the verdict.** Every problem carries a deliberate
  HARD/WARN decision and `note_problem()` RAISES on an unregistered key — so a
  problem the verdict cannot see is no longer expressible. That closes the class,
  not the six instances. `sabotage_test.py --verdict-matrix` asserts the real
  `main()`'s exit status per problem: 17/17. Reverting only the old three-key
  list makes the same matrix report exactly 6 WRONG — the audit's finding,
  reproduced as a test.
- **`_tune_all` rebinds `args` to a per-camera copy** (the parameter is now
  `outer_args`), so no camera can inherit another's state and later-added lines
  cannot reintroduce it. That is the gain ratchet's third door, closed by
  construction.

Honest residue, all from the agent's own report:
- Runtime went **125 s → 130–143 s** (+4% to +14%): 2 extra gain repeats per
  camera, end-of-tune verification, baseline-exposure restore.
- The 35% reproj swing at the deciding gain **could not be forced live** — the
  room was quiet (0.2% run-to-run). The mechanism is proven by replaying the
  audit's numbers, not by live noise. **Re-test in different light.**
- At the default `--tag-fraction 0.85`, the 3.5–3.8 tag drop that motivated D5
  is still inside tolerance and allowed.
- Briefly made worse then fixed: the new polling opened a websocket every 0.4 s
  and killed three runs; it now backs off 0.5→2.0 s.
- **Pre-existing, NOT fixed:** the daemon's NT status freezes for the duration of
  a run (it stops the instant `NTResults` creates its private instance and
  resumes at the end). Proven identical before and after, so not introduced here.

`photontune.service` is **inactive** — deliberately, to avoid lock contention
during verification. Starting it runs the OLD `/opt` copy and immediately
retunes both cameras:  `ssh photonvision sudo systemctl start photontune`.
Deploy the new build first if that matters.

**Next: an independent adversarial audit of round 2, in different light. Then
push.**
