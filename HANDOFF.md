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
