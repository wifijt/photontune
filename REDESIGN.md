# photontune, rebuilt around what the measurements actually support

Spec, 2026-09-19. Supersedes the gain-search design. Read `HANDOFF.md` first.

## Why

Four rounds of fix-and-audit did not reduce the defect rate. The recurrence was
structural, and a design critique found the reason: **the tool optimises a
quantity it cannot resolve, and that it should not be optimising.**

`bestReprojErr` is OpenCV's RMS residual over the corners in the multi-tag fit.
It measures internal consistency of a fit, NOT pose accuracy — and it *improves*
when distant tags drop out of the solve, because fewer, closer corners fit
better. That is why a tag-count guard had to be bolted on to stop the optimiser
rewarding tag loss. We were fighting our own objective.

Measured consequences:

| | |
|---|---|
| back-to-back repeat noise | 0.0–0.9% |
| drift across one scan (~10 min) | ~2.0% |
| margin decisions actually turned on | 0.005 px |
| six identical runs, one camera | gain 60, 80, 60, 80, 100, 100 |
| in a dim room | monotone to the top of the range; "optimum" = the ceiling |
| cost | ~135 s for two cameras |

The gain ratchet was fixed three times in three code paths. It kept coming back
because **the objective reopens it every run**: with nothing in the objective
penalising gain, the only thing that ever stopped it choosing maximum was a
tie-break floored at an arbitrary constant.

## The measurements this design rests on

**Apply latency** (timed on the coprocessor against PhotonVision's own
`captureTimestampMicros`):

- exposure: **0.25–0.36 s** (6 trials)
- gain: **0.234 s median, 0.197–0.241, n=8**, symmetric in both directions

A 0.7 s settle covers both with ~3x margin. Changing gain is not more expensive
than changing exposure, which is what makes a gain-walk affordable.

*(An earlier attempt at the gain figure returned 0.011 s — faster than one frame
period. The light had shifted so gain 0 still saw tags, the edge never fired,
and the timer caught a frame already in flight. Rejected. The retry first probes
for an exposure where gain 0 genuinely sees nothing, found 600 µs.)*

**Blur tolerance** — MEASURED, as of 2026-09-19, except for one factor of two
that is named at the end. This replaces the proxy reasoning that stood here.

What `blur` is, from PhotonVision's and upstream apriltag's source:

- It is apriltag's `quad_sigma`, applied **after** `quad_decimate`, to the
  **decimated** image (`AprilTagPipeline.java:93-94`; `apriltag.c:1024-1031`).
  At the deployed `decimate=2`, a setting of σ means **2σ full-resolution
  pixels** — a factor of two that was missing from the old budget.
- At `decimate ≥ 2` the payload decode reads the **unblurred** image
  (`apriltag.c:1150`); at `decimate == 1` the blur is in place and the decode
  sees it. Confirmed on hardware: at decimate=2 every tag's decision-margin
  bracket is flat in blur, while at decimate=1 it falls monotonically
  (`/tmp/aud/margin.log`, six tags, two cameras). A decimate=2 sweep therefore
  measures only the quad detector; motion blur gets no such exemption, so all
  scaling below is taken from the **decimate=1** sweep.
- **σ < 0.5 is a complete no-op**: `ksz = 4*sigma`, gated on `ksz > 1`, so
  σ 0.4 runs no filter at all. Corroborated on hardware — blur 0.4 and blur
  0.0 give identical margin brackets for all six tags at both decimates.
- **There is no clamp at σ 4.0.** The earlier "both cameras collapse at
  exactly 4.0, an implementation limit" was the sweep's last grid point plus
  multi-tag needing two tags. Extended to σ 7, two tags are still detected at
  100% at σ 4.0 and one survives to 5.0.

How tolerance scales. The earlier "2.2× spread with no relationship to
apparent size" was **a data-reading error**: `blurscale.json` omits a row
when a tag scores zero, and absent rows were read as end-of-series rather
than as zero, truncating each series at a different place. Re-read with
absent = 0, at decimate=1, interpolated to the 50% crossing:

```
tag side   σ_fail    σ_fail/side
  45.5 px    2.48       0.0545
  54.6 px    3.26       0.0596
  55.2 px    3.75       0.0680
  67.9 px    4.75       0.0700
  74.8 px    3.76       0.0503
  86.3 px    5.51       0.0639
```

Tolerance **is** proportional to tag size. Worst tag σ_fail = **0.050 × side**
(about 0.4 of a bit cell — a 36h11 tag is 8 cells across); mean 0.061; spread
across the six **1.39×**. *(An earlier reading of this same data reported the
spread as 1.18×; it is 1.39×. The worst-tag coefficient is unaffected, and a
budget should be built on the worst tag.)*

Converting by equal variance, `L = σ√12`, taking **no** credit for motion blur
being one-dimensional:

**L_fail ≈ 0.173 × (tag side in px)**

For a 6.5 in tag at fx 1105.9: 15.9 px at 2 m, 7.9 at 4 m, **6.4 at 5 m**,
4.5 at 7 m, 4.0 at 8 m.

**So 6 px is conservative inside ~5 m and optimistic beyond it.** A single
number cannot be right at both ends. The right shape is `0.17 × side_px`
(~1.4 bit-cell widths) pinned to the longest range the team needs — which is
what `--max-range` does. It is **off by default**, because turning it on
changes the answer.

**Still unmeasured, and now the largest remaining lever:** the 2× credit for
motion blur being one-dimensional. A Gaussian blurs every edge; a smear only
blurs edges perpendicular to it, and a tag has edges in two orthogonal
directions. PhotonVision's blur is isotropic and cannot test this, so no sweep
of it ever will. Note the consequence: `L_fail = 0.173 × side` takes no such
credit, so using `0.17 × side` as the **budget** puts the operating point at
the measured failure point of the worst tag, and the entire margin is that
unmeasured 2×. That is a deliberate, stated bet, not a conservative choice.
`blurtest.py`, with a human waving a tag, is the only thing that settles it.

## The algorithm

Per camera. Exposure is FIXED at the blur budget; gain is the only search.

1. **Assert the baseline** (`assert_baseline`, unchanged). Read `fx` from the
   ACTIVE calibration. Refuse — or switch mode — if uncalibrated.
2. **`t_budget = max_blur_px / (radians(blur_rate) * fx)`**, clamped to the
   camera's reported `minExposureRaw`/`maxExposureRaw`. At 6 px, 360 deg/s and
   fx 1105.9 that is **864 µs**. Set gain 0, exposure `t_budget`.
3. **Reference count:** gain 100 at `t_budget`, sample 2 s, record `best_tags`
   and the multi-tag solve rate. This is the best the scene can do without
   exceeding the budget.
4. **Walk gain UP** from 0 through the existing 6-point grid. At each point
   sample 2 s (>= 20 frames even at the websocket's 10 Hz). Pass if
   `rate_upper_bound(solves) >= 0.90` AND not `tags_significantly_below(best_tags)`.
   **Stop at the first pass. Apply that gain plus ONE grid step** as margin — a
   field is not the pit.
5. **If gain 0 fails because the image is saturated** — defined as: fails at
   gain 0 AND fails at gain 100 with `mean_tags` FALLING as gain rises — walk
   exposure DOWN from `t_budget` at gain 0 until it passes.
6. **If gain 100 at `t_budget` fails:** walk exposure UP at gain 100 to the
   shortest that passes, apply it, and raise `over_blur_budget` — **HARD**,
   since 2026-09-19. It was WARN, and a camera at 26x the budget exited 0.
   The camera is still LEFT at what the walk found (the best available answer
   beats none); the exit code says it is not the answer that was asked for.
7. **Confirm** the write, run `verify_final_state`, publish the verdict.

Note on steps 5 and 6: they are reachable **only when the reference is
unusable**. If the reference passes, gain 100 IS the reference, is judged
against its own mean, and passes — so the walk always picks at least gain
100 and never falls through. Every rescue is therefore a run whose tag-count
floor was dropped, which is why `reference_unusable` is **HARD** rather than
WARN: without a reference nothing in the run checks that the chosen settings
see *every* tag, and two tags satisfy the 90% multi-tag gate as well as four.
Forced on hardware: a rescue landed on 2.00 tags in a scene showing 4.00 and
exited 0.

Cost: 1 + 6 + confirm ~= 8 trials x ~2.7 s ~= **25 s per camera**, deterministic,
no reprojection in the decision, no recursion.

One sentence a student can repeat: *the least gain that still sees every tag, at
the exposure a spinning robot allows.*

Reprojection is still REPORTED as a diagnostic. It is simply not permitted to
choose anything.

## Keep

`BASELINE_ALWAYS` / `BASELINE_DEFAULT` and `assert_baseline` — the highest-value
thing in the repo; every entry is a one-click way a student breaks the camera,
and three of them were silently never applied for two weeks. `sabotage_test.py`
and its 15-setting matrix. The `PROBLEMS` registry and `note_problem` raising on
an unregistered key. The blur budget. `tags_significantly_below` and
`rate_upper_bound`. `RunLock`. `PENDING_RESTORE` replay on every exit.
`verify_final_state`. The calibration gate and `camera_fx`.

## Delete

- **The gain scan and its decision machinery** — `optimise_gain` phases 2–3,
  `gain_decision`, `GAIN_PROBE_WINDOW/MAX`, `GAIN_BAND_FLOOR`,
  `--gain-search-steps`, `--reproj-tolerance`, `--gain-tag-sigma`,
  `sabotage_test.py --gain-decision`.
- **NT-server management** — `read_network_config`, `set_nt_server`,
  `nt_server_reachable`, the toggle in `_run_locked`, `nt_server_not_stopped`,
  `_NTLink.resync/republish`. *Verified from PhotonVision's source:* stopping
  the server calls `stopServer()`, orphaning every client, AND
  `NetworkManager.reinitialize()` restarts the web server, dropping every
  websocket **regardless of `shouldManage`**. The tool destroys its own control
  channel by design. Point `--nt-server` at whatever server PhotonVision is
  already a client of; never create one. Cost: bench sampling drops to 10 Hz,
  which the new algorithm tolerates.
- **Held-card mode** — `--reference-tag/-range/-bias`,
  `--calibrate-reference-bias`, `--move-pause`. It requires a human holding a
  card, which contradicts principle 1, and measures a different cliff corrected
  by a 1.6 fudge that a second mode exists to calibrate. Replace with "any tags
  in view"; a printed multi-tag board in the pit costs $2.
- **`--dry-run`** — it writes exposure across the whole sweep and then restores,
  i.e. every risk of a real run plus a restore path that needed its own audit,
  and which was found exiting 0 having left a camera at 10x over-exposure.
- **`baseline_contradiction`** — it guards against corruption by a competing
  reader, which `RunLock` now prevents, and it is what hard-failed a second tune
  when a stale-frame baseline disagreed with itself.
- **`_escalate_to` and sweep narrowing**, `--gain-steps`, `--bias`,
  `--tag-fraction`, `--steps`, `--fast`.

## Structural rules, so the recurrence stops

- **No shared mutable `args`.** Config is a frozen dataclass. Per-camera search
  state is an explicit object created inside the loop. The ratchet returned
  three times through shared state; this is what makes it impossible rather
  than absent.
- **No recursion.** `tune_camera` escalated by `return await tune_camera(...)`,
  discarding the outer frame — which is why `carry`, `merge_failures`,
  save/restore and `copy.copy` all exist: four layers protecting one shared
  object. A loop returning one record needs none of them.
- **Every problem goes through `note_problem`**, which raises on an unregistered
  key. Already true; keep it true.

## Boot model

**Boot asserts the baseline only** — seconds, safe, no camera left mid-sweep.
**Tuning is on demand**, triggered from the dashboard at the field.

Rationale, verified: PhotonVision already persists every websocket write to
SQLite, so tune-once-and-persist is the platform's default. Re-deriving the same
answer every power cycle buys nothing and spends ~131 s sweeping both cameras
through blind exposures while the robot may be on the cart about to be enabled.
`robot_is_enabled` is checked once before the run and never again during it, so
a match starting mid-sweep leaves a camera wherever the sweep was. That violates
"a match must never be interrupted" more directly than any bug found in four
audits.

**Re-check `robot_is_enabled` between steps** and abort with a restore.

Lighting differs between pit and field, which is the argument FOR on-demand and
AGAINST boot: you want to tune where you will play, not where you parked.

## Acceptance

- Six consecutive two-camera runs choose the **same gain per camera**, or report
  a tie and break it deterministically.
- Two-camera run **<= 60 s**.
- `sabotage_test.py` 16/0; the verdict matrix still complete.
- A tune interrupted at any point leaves the camera usable.
- No path exits 0 on a detected failure.
