# photontune — state of play

Written 2026-09-18. Read this before changing anything.

## Where the code is

Branch `feat/baseline-and-blur-budget`. **Commits past the last audited point are
NOT pushed.** GitHub PR #9 (and photonvision-tools #4, #5) reflect the earlier,
audited state only. Do not push a round of fixes until it has survived an
independent adversarial audit — see below for why.

## The two principles everything is judged against

1. **It should just work without human interaction.**
2. **Tune to competition-ready settings as fast as possible.**

Runtime went 233 s → 214 s → 125 s for a two-camera run. The unattended boot
autorun completes in ~131 s.

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
  body is. Drive it from the console or a second interface, never over the link
  it resets, and never right before you need the Pi.
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

**All gain measurements come from one dim room**, where reprojection improved
monotonically to the top of the range. Re-validate the gain-reproducibility fix
in different light before trusting it.

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
