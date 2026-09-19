#!/usr/bin/env python3
#
# photontune - detection-quality exposure tuning for PhotonVision
# Copyright (C) 2026  wifijt
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <https://www.gnu.org/licenses/>.
"""
photontune - find a safe exposure for PhotonVision AprilTag pipelines, fast.

Scores each candidate on DETECTION QUALITY (multi-tag solve rate, reprojection,
ambiguity) rather than image brightness, then deliberately picks the SHORTEST
exposure that clears the failure cliff.

Why shortest: this tool measures a stationary camera, so motion blur is zero in
everything it can observe. On a moving robot blur scales linearly with exposure
(blur_px = omega * t * fx), and at 360 deg/s with fx=1100 a 12 ms exposure smears
a corner ~100 px across a ~70 px tag. The plateau this tool measures is real but
it is a plateau in the one condition that does not matter, so we bias short.

Modes:
  CLI     python photontune.py --host photonvision.local
  Daemon  python photontune.py --daemon --nt-server 10.TE.AM.2
"""
import argparse, asyncio, copy, json, math, sys, time
from collections import defaultdict

try:
    import msgpack, websockets
except ImportError:
    sys.exit("need: pip install msgpack websockets")


# ───────────────────────── scoring ─────────────────────────

# One-sided 95% confidence. Used to ask "can this sample RULE OUT meeting the
# threshold?" rather than "is the point estimate above it?".
_RATE_Z = 1.645


def rate_upper_bound(hits, n, z=_RATE_Z):
    """Wilson upper confidence bound on a rate measured as hits out of n.

    A rate measured over ~60 frames carries several points of sampling error, so
    a hard cut on the point estimate decides on noise. MEASURED on this rig: 88%
    at 748 us and 89% at 1000 us both produced "NOTHING PASSED... That is a
    LIGHTING or CONFIG problem" on a camera seeing 3.5-3.7 tags, while two
    independent 60 s measurements of the steady-state solve rate on the same rig
    gave 93.5% and 82.9%. The 90% threshold sits INSIDE the normal operating
    range, so a sample cannot be failed for landing just under it.

    Wilson rather than the normal approximation because the interesting region is
    near 1.0, where the normal approximation runs off the end of the scale. At
    ~64 frames this turns a 90% floor into an effective ~85% floor; at 600 frames
    it tightens back towards 90%, which is the correct behaviour - more evidence,
    less benefit of the doubt.
    """
    if n <= 0:
        return 1.0
    p = float(hits) / n
    denom = 1.0 + z * z / n
    centre = p + z * z / (2.0 * n)
    half = z * math.sqrt(p * (1.0 - p) / n + z * z / (4.0 * n * n))
    return min(1.0, (centre + half) / denom)


# Below this many frames, rate_upper_bound is too generous to mean anything and
# the sample is not classified at all.
#
# MEASURED, by asking for each n what the SMALLEST hit rate is that still clears
# the nominal 0.90 gate through the upper bound:
#
#     n =  3  ->  0.667      n = 13  ->  0.769      n = 26  ->  0.808
#     n =  4  ->  0.750      n = 19  ->  0.789      n = 33  ->  0.818
#     n =  7  ->  0.714      n = 20  ->  0.800      n = 64  ->  0.844
#
# At the old n>=3 floor, 2 frames out of 3 - 67% - was a "pass" against a 90%
# threshold, reachable on --no-nt --fast. The sequence is not monotonic (it is
# granularity, not statistics), so the floor has to be the point from which
# EVERY larger n behaves: from n=20 upward the effective floor never drops below
# 0.80 again, for the 0.95 reference-tag gate as well as the 0.90 multi-tag one.
#
# Sampling extends itself to reach this rather than failing (see sample()), so
# the cost lands only on the slow websocket path: over NetworkTables a 1.5 s
# dwell already carries ~60 frames.
MIN_SAMPLE_FRAMES = 20

# How long a collect may run PAST its dwell to reach MIN_SAMPLE_FRAMES. Bounded
# on purpose: a dead pipeline must not add this to all eight sweep points, which
# is why the extension also requires at least one frame to have arrived.
SAMPLE_EXTEND_S = 4.0

# How long a read-back may poll for the camera to agree before it is called a
# failure. PhotonVision's cameraSettings broadcast lags a write by 1-2 s, so
# anything under a few seconds cries wolf; 12 s is long enough that "still not
# agreeing" means it never will.
CONFIRM_TIMEOUT_S = 12.0

# Extra measurements of the gain that is about to win, to estimate the noise
# WHERE THE DECISION IS MADE. Two, for three readings in total: one repeat gives
# a spread that is itself a coin toss, and the audit's three runs of the same
# gain (0.378 / 0.393 / 0.510) show the outlier need not be adjacent. Costs
# 2 x (settle + dwell) ~ 4.4 s per camera.
GAIN_REPEATS = 2


class Sample:
    """Detection quality at one exposure, for one camera."""
    def __init__(self, exposure):
        self.exposure = exposure
        self.frames = 0
        self.multitag_solves = 0
        self.reproj = []
        self.tag_counts = []
        self.ambiguities = []
        self.ref_seen = 0          # frames containing the reference tag
        self.ref_ranges = []       # measured range to it, metres

    @property
    def solve_rate(self):
        return self.multitag_solves / self.frames if self.frames else 0.0

    @property
    def mean_tags(self):
        return sum(self.tag_counts) / len(self.tag_counts) if self.tag_counts else 0.0

    @property
    def med_reproj(self):
        return _median(self.reproj)

    @property
    def med_ambiguity(self):
        return _median([a for a in self.ambiguities if a >= 0])

    @property
    def ref_rate(self):
        return self.ref_seen / self.frames if self.frames else 0.0

    @property
    def ref_range(self):
        return _median(self.ref_ranges)

    def passes(self, min_tags, max_ambiguity, ref_tag=None, tag_target=None):
        if self.frames < MIN_SAMPLE_FRAMES:
            return False
        # Multi-tag solves happily on a subset, so "it solved" is not the same as
        # "it saw everything available". Require most of the tags that the best
        # exposure in this sweep managed to find - otherwise we settle for a short
        # exposure that quietly drops the hardest (usually farthest) tags.
        if tag_target is not None and self.mean_tags < tag_target:
            return False
        # Reference-tag mode: a single held tag cannot multi-tag, and its
        # ambiguity is dominated by viewing angle rather than exposure, so
        # detection RATE is the honest metric.
        if ref_tag is not None:
            return rate_upper_bound(self.ref_seen, self.frames) >= 0.95
        # Prefer the multi-tag criterion when multi-tag is actually running.
        # Judged on the upper confidence bound, not the point estimate: a rate
        # measured over ~60 frames cannot separate 88% from 90%.
        if self.multitag_solves:
            return rate_upper_bound(self.multitag_solves, self.frames) >= 0.90
        # Otherwise: enough tags, seen reliably, at trustworthy ambiguity.
        if self.mean_tags < min_tags:
            return False
        amb = self.med_ambiguity
        return amb is not None and amb <= max_ambiguity

    def why_failed(self, min_tags, max_ambiguity, ref_tag=None, tag_target=None):
        """One short phrase saying why this sample did not pass, or None.

        The gain search reported EVERY outcome as "no passing exposure": a dead
        connection, a calibration refusal, --baseline-only and a genuinely dark
        room were indistinguishable, and six "sweeps" were seen completing in
        0.00 s total with nothing noticing.
        """
        if self.passes(min_tags, max_ambiguity, ref_tag, tag_target):
            return None
        if self.frames == 0:
            return "no frames arrived - nothing was being published"
        if self.frames < MIN_SAMPLE_FRAMES:
            return ("only %d frames - under the %d needed to classify (at %d "
                    "frames the 90%% gate lets %.0f%% through, so a 'pass' here "
                    "would be noise). Raise --dwell, or drop --fast."
                    % (self.frames, MIN_SAMPLE_FRAMES, self.frames,
                       100 * _effective_floor(self.frames)))
        if tag_target is not None and self.mean_tags < tag_target:
            return "saw %.2f tags, needed %.2f" % (self.mean_tags, tag_target)
        if ref_tag is not None:
            return ("reference tag %d in %.0f%% of %d frames (at best %.0f%%, "
                    "under the 95%% floor)"
                    % (ref_tag, 100 * self.ref_rate, self.frames,
                       100 * rate_upper_bound(self.ref_seen, self.frames)))
        if self.multitag_solves:
            return ("multi-tag solved in %.0f%% of %d frames (at best %.0f%%, "
                    "under the 90%% floor)"
                    % (100 * self.solve_rate, self.frames,
                       100 * rate_upper_bound(self.multitag_solves, self.frames)))
        if self.mean_tags < min_tags:
            return "no multi-tag solve, and only %.2f tags" % self.mean_tags
        amb = self.med_ambiguity
        if amb is None:
            return "no multi-tag solve and no usable ambiguity"
        return "ambiguity %.3f, over the %.2f limit" % (amb, max_ambiguity)

    def summary(self, ref_tag=None):
        if ref_tag is not None:
            r = self.ref_range
            return "tag %d seen %3.0f%%  range %s" % (
                ref_tag, 100 * self.ref_rate, ("%.2f m" % r) if r else "n/a")
        if self.multitag_solves:
            return "multitag %3.0f%%  reproj %7.3f  tags %.2f" % (
                100 * self.solve_rate, self.med_reproj or float("nan"), self.mean_tags)
        amb = self.med_ambiguity
        return "tags %.2f  ambiguity %s  (no multitag)" % (
            self.mean_tags, "%.3f" % amb if amb is not None else "n/a")


def _effective_floor(n, threshold=0.90):
    """The smallest hit rate that still clears `threshold` through the bound at n.

    Only used to SAY how weak a small sample is, in the message that refuses it.
    """
    for h in range(n + 1):
        if rate_upper_bound(h, n) >= threshold:
            return h / float(n) if n else 1.0
    return 1.0


# The noise band never goes below this. One set of near-identical readings must
# not make every 1% difference "significant".
GAIN_BAND_FLOOR = 1.05


def gain_decision(good, repeats, leader_gain, reproj_tolerance,
                  band_floor=GAIN_BAND_FLOOR):
    """Which gain wins, given [(gain, reproj)] and repeat readings of the leader.

    Pure arithmetic, deliberately separated from the measuring so it can be
    driven with recorded numbers - see sabotage_test.py --gain-decision, which
    replays the audit's own three runs (gain 100 read 0.378 / 0.393 / 0.510 and
    gain 60 read 0.478 / 0.479 / 0.478) and shows the old rule choosing 100,
    100, 60 while this one chooses the same gain all three times.

    Returns (pick, best, band, noise, good) where pick and best are (gain,
    reproj) pairs and `good` is the input with the leader's reprojection
    replaced by the median of its repeats.
    """
    noise = None
    if len(repeats) > 1:
        # The spread OBSERVED AT THE DECIDING POINT, not at the quietest gain in
        # the scan. And the median of the repeats, so one lucky reading cannot
        # win the scan on its own.
        noise = abs(math.log(max(repeats) / min(repeats)))
        leader_rp = _median(repeats)
        good = [(g, leader_rp if g == leader_gain else rp) for g, rp in good]
    best_rp = min(rp for _g, rp in good)
    # --reproj-tolerance keeps its own job: REJECT a gain whose fit is clearly
    # worse than the best. It is not a noise estimate and must not be used as
    # one - at its 1.5 default it declares a 28% difference insignificant.
    usable = [t for t in good if t[1] <= best_rp * float(reproj_tolerance)]
    band = max(math.exp(noise) if noise is not None else 1.0, band_floor)
    contenders = [t for t in usable if t[1] <= best_rp * band]
    pick = min(contenders, key=lambda t: t[0])       # LOWEST gain wins a tie
    best = min(good, key=lambda t: t[1])
    return pick, best, band, noise, good


def _median(xs):
    xs = sorted(xs)
    if not xs:
        return None
    n = len(xs)
    return xs[n // 2] if n % 2 else 0.5 * (xs[n // 2 - 1] + xs[n // 2])


def geometric_sweep(lo, hi, steps):
    if steps < 2:
        return [lo]
    r = (hi / lo) ** (1.0 / (steps - 1))
    return [lo * (r ** i) for i in range(steps)]


def sweep_holes(samples):
    """Find physically impossible gaps in an exposure sweep.

    Brightness rises monotonically with exposure, so detection cannot come back
    after vanishing. A sample with tags, then one with none, then tags again, is
    a measurement artefact - not a property of the scene. Seen in the wild when
    another process was reading PhotonVision's websocket concurrently and
    starving individual samples: tags went 0.25, 0, 0.03, 0, 0, 1.08, 3.00 and
    the cliff estimate landed 4x too long.

    Catches a STRONG detection, then none, then strong again. Note it does NOT
    catch every corrupted sweep: a run starved by a competing websocket reader
    produced 0.25, 0, 0.03, 0, 0, 1.08, 3.00 - wrong at specific points but
    still rising, so nothing here fires. baseline_contradiction() is the check
    for that.

    Returns the list of (exposure, mean_tags) that sit in a hole.
    """
    seq = sorted(samples, key=lambda s: s.exposure)
    holes = []
    for j in range(1, len(seq) - 1):
        if seq[j].mean_tags > 0.5:
            continue
        before = any(s.mean_tags > 0.5 for s in seq[:j])
        after = any(s.mean_tags > 0.5 for s in seq[j + 1:])
        if before and after:
            holes.append((seq[j].exposure, seq[j].mean_tags))
    return holes


def baseline_contradiction(samples, base_exposure, base_tags):
    """Does the sweep contradict what the camera was already doing?

    Before sweeping we record detection at the exposure the camera came in on.
    If the sweep then claims a NEARBY exposure sees far fewer tags, the sweep is
    wrong - the scene did not change, the measurement did. This is what catches a
    sweep starved by another process reading PhotonVision's websocket, where the
    numbers stay plausibly ordered but are individually false.

    Returns (exposure, swept_tags) of the worst contradicting sample, or None.
    """
    if base_tags < 1.0 or base_exposure <= 0:
        return None                      # nothing to contradict
    worst = None
    for s in samples:
        # Asymmetric on purpose. A SHORTER exposure seeing fewer tags is just the
        # detection cliff - that is the whole point of the sweep. A LONGER one
        # seeing fewer is impossible until over-exposure, so cap the window
        # rather than flag the bright tail.
        if not (0.95 * base_exposure <= s.exposure <= 2.5 * base_exposure):
            continue
        if s.mean_tags < 0.4 * base_tags:
            if worst is None or s.mean_tags < worst[1]:
                worst = (s.exposure, s.mean_tags)
    return worst


def choose_exposure(samples, bias, min_tags, max_ambiguity, ref_tag=None,
                    tag_fraction=0.85):
    """Estimate where detection actually fails, then sit a safety factor above it.

    Biasing off the shortest *tested* passing value double-counts margin: with a
    geometric sweep, adjacent points differ by the step ratio, so that value is
    already up to one full step above the true cliff. Instead we bracket the cliff
    between the highest failing and lowest passing sample and take the geometric
    mean, which is the best single estimate from a log-spaced sweep.

    Returns (chosen, shortest_passing_sample, cliff_estimate, tag_target).
    All four are returned on every path - the caller unpacks four, and a
    3-tuple here silently killed the gain-escalation path for dark rooms.
    """
    # How many tags did the BEST exposure in this sweep see? Anything much below
    # that is leaving detections on the table.
    best_tags = max((s.mean_tags for s in samples), default=0.0)
    tag_target = best_tags * tag_fraction if (best_tags >= 2 and ref_tag is None) else None
    passing = [s for s in samples
               if s.passes(min_tags, max_ambiguity, ref_tag, tag_target)]
    if not passing:
        # chosen=None tells the caller to escalate gain. tag_target still comes
        # back so it can say WHY nothing passed.
        return None, None, None, tag_target
    shortest = min(passing, key=lambda s: s.exposure)
    failing_below = [s for s in samples
                     if s.exposure < shortest.exposure
                     and not s.passes(min_tags, max_ambiguity, ref_tag, tag_target)]
    if failing_below:
        highest_fail = max(failing_below, key=lambda s: s.exposure).exposure
        cliff = math.sqrt(highest_fail * shortest.exposure)
    else:
        # Everything we tested passed - the cliff is at or below our range.
        cliff = shortest.exposure
    ceiling = max(s.exposure for s in samples)
    return min(cliff * bias, ceiling), shortest, cliff, tag_target


# ───────────────────────── PhotonVision link ─────────────────────────

class Photon:
    def __init__(self, host, port=5800):
        self.uri = "ws://%s:%d/websocket_data" % (host, port)
        self.ws = None

    async def __aenter__(self):
        try:
            # ping_interval=None: no client-side keepalive. Once sampling moved to
            # NetworkTables this socket is only used for set_setting, so it sits
            # idle for tens of seconds and the default 20 s ping/pong killed it
            # mid-tune - "sent 1011 (unexpected error) keepalive ping timeout".
            # It is a local connection with an explicit lifetime; we do not need
            # the library policing it.
            self.ws = await websockets.connect(self.uri, open_timeout=10,
                                               max_size=80_000_000,
                                               ping_interval=None, close_timeout=5)
        except Exception as exc:
            raise ConnectionError(
                "could not reach PhotonVision at %s - check the host and that it is running (%s)"
                % (self.uri, type(exc).__name__)) from None
        return self

    async def __aexit__(self, *a):
        if self.ws:
            await self.ws.close()

    async def _pump(self, seconds, on_message, stop_when=None):
        t0 = time.time()
        while time.time() - t0 < seconds:
            remaining = seconds - (time.time() - t0)
            try:
                raw = await asyncio.wait_for(self.ws.recv(), timeout=max(0.2, remaining))
            except asyncio.TimeoutError:
                return
            if not isinstance(raw, bytes):
                continue
            msg = msgpack.unpackb(raw, raw=False)
            if isinstance(msg, dict):
                on_message(msg)
            if stop_when is not None and stop_when():
                return

    async def cameras(self, timeout=12):
        """[{uniqueName, nickname, settings, calibrations, formats, bounds}].

        Returns as soon as the first cameraSettings arrives - PhotonVision sends
        it on connect (measured ~0.14 s). Without the early exit this waited out
        the whole 12 s timeout on every single run, because updatePipelineResult
        keeps arriving so the recv never times out.

        Keeps the calibration and bounds data. It costs nothing to retain - it is
        in the same message - and discarding it was why the tool could not warn
        about a missing calibration, and why it guessed fx from a CLI default
        instead of reading the camera's own intrinsics.
        """
        found = {}
        def handle(msg):
            for cam in msg.get("cameraSettings", []) or []:
                found[cam["uniqueName"]] = {
                    "uniqueName": cam["uniqueName"],
                    "nickname": cam.get("nickname", "?"),
                    "settings": cam["currentPipelineSettings"],
                    "calibrations": cam.get("calibrations") or [],
                    "formats": cam.get("videoFormatList") or [],
                    "minExposureRaw": cam.get("minExposureRaw"),
                    "maxExposureRaw": cam.get("maxExposureRaw"),
                }
        await self._pump(timeout, handle, stop_when=lambda: bool(found))
        return list(found.values())

    # PhotonVision discards a float sent to an integer-typed setting: setProperty
    # does propField.setInt(settings, (Integer) value), so a Double raises
    # ClassCastException and the write does not happen. It is NOT silent - it is
    # logged at ERROR in PhotonVision's journal ("Unknown exception when setting
    # PSC prop!") - but the websocket API returns nothing, so the only way a
    # client learns of it is to read the coprocessor's log. Coerce them.
    INT_SETTINGS = {"cameraGain", "cameraBrightness", "decimate", "numIterations",
                    "threads", "decisionMargin", "cameraRedGain", "cameraBlueGain",
                    "cameraVideoModeIndex", "pipelineIndex"}

    async def set_setting(self, unique_name, **kw):
        payload = {}
        for k, v in kw.items():
            if k in self.INT_SETTINGS and isinstance(v, float):
                v = int(round(v))
            payload[k] = v
        payload["cameraUniqueName"] = unique_name
        await self.ws.send(msgpack.packb({"changePipelineSetting": payload}))

    async def cameras_fresh(self, timeout=8):
        """Camera list over a NEW connection.

        PhotonVision sends cameraSettings ONCE, on connect - measured: 1 in 25 s
        against 441 updatePipelineResult. Pumping the long-lived socket for it
        therefore never succeeds after the first read, which silently broke the
        readback in set_and_verify. A fresh connection re-triggers the broadcast.
        """
        found = {}
        uri = self.uri
        async with websockets.connect(uri, max_size=80_000_000, open_timeout=10,
                                      ping_interval=None) as ws:
            t0 = time.time()
            while time.time() - t0 < timeout:
                try:
                    raw = await asyncio.wait_for(ws.recv(), timeout=2.0)
                except asyncio.TimeoutError:
                    break
                if not isinstance(raw, bytes):
                    continue
                msg = msgpack.unpackb(raw, raw=False)
                if not isinstance(msg, dict):
                    continue
                for cam in msg.get("cameraSettings", []) or []:
                    found[cam["uniqueName"]] = {
                        "uniqueName": cam["uniqueName"],
                        "nickname": cam.get("nickname", "?"),
                        "settings": cam["currentPipelineSettings"],
                    }
                if found:
                    break
        return list(found.values())

    async def set_and_verify(self, unique_name, settle=1.5, **kw):
        """Write settings and confirm the camera took them. Returns list of failures."""
        await self.set_setting(unique_name, **kw)
        await asyncio.sleep(settle)
        cams = await self.cameras_fresh(timeout=8)
        got = next((c for c in cams if c["uniqueName"] == unique_name), None)
        if not got:
            # Could not read the camera back at all - that is NOT the same as a
            # setting being rejected, and must not raise the "unreliable" banner.
            return [(None, None, None)]
        live = got["settings"]
        bad = []
        for k, want in kw.items():
            if k in self.INT_SETTINGS and isinstance(want, float):
                want = int(round(want))
            have = live.get(k)
            try:
                same = abs(float(have) - float(want)) < 1e-6
            except (TypeError, ValueError):
                same = (have == want)
            if not same:
                bad.append((k, want, have))
        return bad

    async def confirm(self, unique_name, expect, timeout=CONFIRM_TIMEOUT_S,
                      settle=1.2, log=None):
        """Poll cameraSettings until the camera agrees, or the timeout expires.

        Returns (mismatches, read_ok). read_ok is False only if we never managed
        to read this camera AT ALL - which is NOT "the setting was rejected", and
        is not a pass either. Both of those confusions were live:

          - ONE look, 1.5 s after the write, was the whole read-back. Measured:
            photontune's own NT-server toggle bounced a competing writer offline
            inside that window, the single look saw the value it wanted, no
            failure was recorded, the run exited 0 - and the camera ended it with
            cameraRedGain=50. A snapshot cannot tell a settled value from one
            that is about to be overwritten; only re-reading can.
          - When PhotonVision stopped broadcasting cameraSettings the read
            returned None, which was logged and then treated as success.

        Polling rather than one longer sleep because PhotonVision's cameraSettings
        broadcast lags the write by 1-2 s and is not on a fixed schedule: an
        earlier attempt at this cried wolf on writes that had in fact succeeded.
        On the happy path this returns after the first look, so it costs what the
        old single read cost; the extra time is only ever spent on a failure.
        """
        deadline = time.time() + timeout
        await asyncio.sleep(min(settle, timeout))
        bad, read_ok, gap = None, False, 0.5
        while True:
            try:
                cams = await self.cameras_fresh(timeout=6)
            except Exception:
                cams = []
            got = next((c for c in cams if c["uniqueName"] == unique_name), None)
            if got is not None:
                read_ok = True
                bad = self._diff(got["settings"], expect)
                if not bad:
                    return [], True
            if time.time() >= deadline:
                break
            # Back off. Every poll is a NEW websocket connection - cameraSettings
            # is only broadcast on connect - and a fixed 0.4 s interval means ~25
            # connect/disconnect cycles in a 12 s timeout. MEASURED: three runs
            # died with "no close frame received or sent" while polling hard
            # across a video-mode change, which restarts the capture pipeline and
            # is exactly when a read-back disagrees for several seconds. Slower
            # polling still confirms in 1-2 s on the happy path (the first look
            # almost always agrees) and stops the tool from knocking over the
            # server it is asking.
            await asyncio.sleep(gap)
            gap = min(gap * 1.6, 2.0)
        if not read_ok and log:
            log("   !! PhotonVision did not broadcast cameraSettings once in "
                "%.0f s. It is up (the websocket connected) but is not "
                "reporting camera state." % timeout)
            log("   !! Seen after selecting pipeline 0 on this rig: every "
                "DataChangeService dispatch threw NullPointerException and the "
                "broadcast stopped. Select a different pipeline.")
        return (bad if bad is not None else []), read_ok

    async def set_and_verify(self, unique_name, settle=1.5,
                             timeout=CONFIRM_TIMEOUT_S, log=None, **kw):
        """Write settings and confirm the camera took them. Returns list of failures.

        A single (None, None, None) entry means "could not read the camera back
        at all" - different from a rejection, and treated as a HARD failure by
        the caller rather than as silence.
        """
        await self.set_setting(unique_name, **kw)
        bad, read_ok = await self.confirm(unique_name, kw, timeout=timeout,
                                          settle=settle, log=log)
        if not read_ok:
            return [(None, None, None)]
        return bad

    async def collect(self, unique_name, seconds, ref_tag=None, min_frames=0,
                      extend=SAMPLE_EXTEND_S):
        """Gather pipeline results for one camera.

        min_frames: keep collecting past `seconds` until this many frames have
        arrived. The websocket is throttled to ~9 fps, so a 0.9 s --fast dwell
        carries ~8 frames - below MIN_SAMPLE_FRAMES, which would refuse to
        classify anything in that mode. Extending is the cheap half of the fix.
        Only extends when frames ARE arriving: at zero the pipeline is dead or
        the scene is black, and waiting longer buys nothing but a slower sweep.
        """
        out = {"frames": 0, "solves": 0, "reproj": [], "tags": [], "amb": [],
               "ref_seen": 0, "ref_ranges": []}
        def handle(msg):
            res = msg.get("updatePipelineResult")
            if not res or unique_name not in res:
                return
            c = res[unique_name]
            out["frames"] += 1
            targets = c.get("targets") or []
            out["tags"].append(len(targets))
            for t in targets:
                out["amb"].append(t.get("ambiguity", -1))
                if ref_tag is not None and t.get("fiducialId") == ref_tag:
                    out["ref_seen"] += 1
                    p = t.get("pose") or {}
                    if "x" in p:
                        out["ref_ranges"].append(
                            math.sqrt(p["x"] ** 2 + p["y"] ** 2 + p["z"] ** 2))
            mt = c.get("multitagResult")
            if mt:
                out["solves"] += 1
                out["reproj"].append(mt.get("bestReprojectionError", float("nan")))
        await self._pump(seconds, handle)
        t_end = time.time() + extend
        while (min_frames and 0 < out["frames"] < min_frames
               and time.time() < t_end):
            await self._pump(0.5, handle)
        return out


# ───────────────────────── the sweep ─────────────────────────

async def sample(pv, cam, args, seconds, ref_tag, after_capture=None):
    """Collect detections, preferring NetworkTables over the throttled websocket."""
    reader = getattr(args, "_nt", {}).get(cam["nickname"])
    if reader is not None:
        # ntcore blocks; keep the event loop free so nothing else stalls.
        return await asyncio.get_event_loop().run_in_executor(
            None, reader.collect, seconds, ref_tag, after_capture,
            MIN_SAMPLE_FRAMES)
    return await pv.collect(cam["uniqueName"], seconds, ref_tag,
                            min_frames=MIN_SAMPLE_FRAMES)


def capture_mark(args, cam):
    """Newest capture timestamp right now, to gate out pre-change frames."""
    reader = getattr(args, "_nt", {}).get(cam["nickname"])
    return reader.newest_capture() if reader is not None else None



def active_format(cam):
    """The {width,height,fps} PhotonVision is currently running on this camera."""
    fmts = cam.get("formats") or {}
    idx = cam.get("settings", {}).get("cameraVideoModeIndex")
    if idx is None:
        return None
    for key in (idx, str(idx), int(idx) if str(idx).lstrip("-").isdigit() else idx):
        if isinstance(fmts, dict) and key in fmts:
            return fmts[key]
    if isinstance(fmts, list):
        try:
            return fmts[int(idx)]
        except (IndexError, ValueError, TypeError):
            return None
    return None


def calibration_for(cam):
    """The calibration matching the ACTIVE resolution, or None.

    PhotonVision stores a calibration per resolution. Having "a calibration" is
    not enough - it has to be for the mode the camera is actually running.
    """
    fmt = active_format(cam)
    if not fmt:
        return None
    w, h = fmt.get("width"), fmt.get("height")
    for cal in cam.get("calibrations") or []:
        res = cal.get("resolution") or {}
        if abs(float(res.get("width", -1)) - float(w)) < 1 and \
           abs(float(res.get("height", -1)) - float(h)) < 1:
            return cal
    return None


def camera_fx(cam, fallback=None):
    """Focal length in pixels from the camera's OWN intrinsics.

    The blur budget is blur_px = omega * t * fx, so fx sets the whole scale of
    it. A hard-coded default is only right for one camera at one resolution -
    on this rig 1105.9 at 1280x800 becomes 570.5 at 640x400, so a default
    carried across a resolution change overstates blur by 1.94x and buys gain
    nobody needed.
    """
    cal = calibration_for(cam)
    if cal:
        ci = cal.get("cameraIntrinsics") or {}
        data = ci.get("data") or ci.get("dataValue")
        if data and len(data) >= 1 and data[0]:
            return float(data[0])
    return fallback


def calibrated_modes(cam):
    """[(videoFormatList index, format, calibration)] for every calibrated mode."""
    fmts = cam.get("formats") or {}
    items = list(fmts.items()) if isinstance(fmts, dict) else list(enumerate(fmts))
    out = []
    for idx, fmt in items:
        if not isinstance(fmt, dict):
            continue
        w, h = fmt.get("width"), fmt.get("height")
        for cal in cam.get("calibrations") or []:
            res = cal.get("resolution") or {}
            try:
                if abs(float(res.get("width", -1)) - float(w)) < 1 and \
                   abs(float(res.get("height", -1)) - float(h)) < 1:
                    out.append((int(idx), fmt, cal))
                    break
            except (TypeError, ValueError):
                continue
    return out


async def ensure_calibrated_mode(pv, cam, args, log=print):
    """Move the camera onto a calibrated video mode if it is not on one.

    Returns (cam, problem). When a calibrated resolution EXISTS in
    videoFormatList but the active one is uncalibrated, switching to it is the
    difference between a tool that tells a human what to do and a tool that just
    works - which is the whole point of running unattended. Only refuses when no
    calibrated mode exists at all, because then there is genuinely nothing to do
    but calibrate.

    Picks the calibrated mode closest in pixel count to the one in use, so the
    switch changes the camera's CPU cost and framerate as little as possible.
    """
    problem = calibration_problem(cam, assume_solvepnp=True)
    if not problem:
        return cam, None
    options = calibrated_modes(cam)
    fmt = active_format(cam) or {}
    if not options:
        return cam, problem
    now_px = float(fmt.get("width") or 0) * float(fmt.get("height") or 0)

    def closeness(o):
        px = float(o[1].get("width", 0)) * float(o[1].get("height", 0))
        return (abs(math.log(px / now_px)) if now_px and px else 0.0, -px)

    idx, newfmt, cal = min(options, key=closeness)
    log("   !! the active video mode %gx%g has NO calibration. With solvePNP on,"
        % (fmt.get("width", 0), fmt.get("height", 0)))
    log("   !! PhotonVision publishes NOTHING in this state, so every exposure")
    log("   !! would look equally dead and the tune would blame the lighting.")
    log("   !! SWITCHING the pipeline to video mode %d (%gx%g), which IS "
        "calibrated." % (idx, newfmt.get("width", 0), newfmt.get("height", 0)))
    log("   !! That is a real change to your pipeline, made deliberately: it is "
        "the difference between a camera that works and one that needs a human.")
    bad = await pv.set_and_verify(cam["uniqueName"], settle=max(args.settle, 2.0),
                                  cameraVideoModeIndex=int(idx))
    fresh = await pv.cameras_fresh(timeout=8)
    newer = next((c for c in fresh if c["uniqueName"] == cam["uniqueName"]), None)
    if newer:
        cam = dict(cam)
        cam["settings"] = newer["settings"]
    problem = calibration_problem(cam, assume_solvepnp=True)
    if problem:
        log("   !! the switch did not take (%s) - %s"
            % ([b for b in bad if b[0]] or "no rejection reported", problem))
        return cam, problem
    ci = (cal.get("cameraIntrinsics") or {})
    data = ci.get("data") or ci.get("dataValue") or []
    log("   now on a calibrated mode: %gx%g, fx %s"
        % (newfmt.get("width", 0), newfmt.get("height", 0),
           ("%.1f" % float(data[0])) if data else "?"))
    # A resolution change restarts the capture pipeline, and the settings
    # read-back comes back long before the frames do. MEASURED: sweeping
    # immediately after the switch read 0.00 tags at EVERY exposure on a camera
    # that sees 2.00 at the same settings once settled, so the tune reported a
    # lighting problem it had caused itself. Wait for detections to come back.
    t0 = time.time()
    while time.time() - t0 < 20:
        raw = await sample(pv, cam, args, 1.0, args.reference_tag)
        if raw["tags"] and (sum(raw["tags"]) / len(raw["tags"])) > 0:
            log("   detections resumed %.0f s after the mode change" % (time.time() - t0))
            break
        await asyncio.sleep(1.0)
    else:
        log("   NOTE: no tags seen in the %.0f s after the mode change - "
            "continuing anyway, but the sweep below may be measuring a camera "
            "that is still restarting." % (time.time() - t0))
    return cam, None


def calibration_problem(cam, assume_solvepnp=False):
    """Why this camera cannot produce a 3D pose, or None if it can.

    assume_solvepnp: judge as though solvePNP were on. The gate used to return
    None whenever solvePNPEnabled was false - which is PhotonVision's DEFAULT -
    and was then checked BEFORE the baseline turned solvePNP on, so on a
    genuinely fresh camera it saw nothing wrong and six fake "no passing
    exposure" lines printed before the real message. photontune's own baseline
    asserts solvePNPEnabled=True, so when the baseline is going to run, the
    honest question is whether the camera will work AFTER it.

    With solvePNP on and no calibration for the active resolution, PhotonVision
    publishes nothing at all, so every exposure scores zero, gain escalates to
    the ceiling, and the tool blames the lighting. The one config problem it
    actually is never gets named.
    """
    st = cam.get("settings", {})
    if not (assume_solvepnp or st.get("solvePNPEnabled")):
        return None
    fmt = active_format(cam)
    if fmt is None:
        return ("cannot tell which video mode is active (cameraVideoModeIndex=%r)"
                % st.get("cameraVideoModeIndex"))
    if calibration_for(cam) is None:
        have = ["%gx%g" % ((c.get("resolution") or {}).get("width", 0),
                           (c.get("resolution") or {}).get("height", 0))
                for c in (cam.get("calibrations") or [])]
        return ("no calibration for the active mode %gx%g (calibrated: %s). "
                "With solvePNP enabled PhotonVision emits NOTHING in this state - "
                "not a 2D fallback - so every exposure will look equally dead and "
                "no amount of tuning can help. Calibrate this resolution, or "
                "switch to one that is calibrated."
                % (fmt.get("width", 0), fmt.get("height", 0),
                   ", ".join(have) if have else "none"))
    return None


def _escalate_to(args, gain):
    """Next gain to try, and whether that is the ONLY further try.

    With a fixed-exposure gain scan following (--optimise-gain), jump straight to
    the ceiling: the scan re-measures every gain from the bottom afterwards and
    picks the lowest one that is statistically as good, so overshooting here
    costs nothing and saves a whole exposure sweep per step skipped. Without a
    scan to follow (--no-optimise-gain) the escalation IS the final answer, so it
    must still climb gently and stop at the first gain that works.
    """
    if getattr(args, "_gain_scan_follows", False):
        return float(args.max_gain), True
    return min(float(args.max_gain), max(gain * 1.5, gain + 10)), False


# ───────────────── every problem the tool can record ─────────────────
#
# THE REGISTRY IS THE VERDICT. There is no other list.
#
# Before this, tune_failed() consulted three keys while the code recorded ten,
# so six detected failures exited 0 - the worst being apply_mismatch, where the
# tool verified its own final write, printed "*** MISMATCH ***", and then set
# result["applied"] anyway. A tool that detects its own write failed and reports
# success is worse than one that never checked.
#
# HARD: the run did not do what it says it did. exit 1, and ok=false in NT.
# WARN: the answer stands, but a human has to be told. exit 0, and the text MUST
#       appear in the CLI summary and in the daemon's NT status.
#
# Adding a problem without adding it here is not possible: note_problem() is the
# only way to record one and it raises on an unregistered key.
HARD, WARN = "hard", "warn"

PROBLEMS = {
    # ---- hard ----
    "baseline_failed": (HARD,
        lambda v: "structural settings did not take: %s"
                  % ", ".join(str(f[0]) for f in v)),
    "baseline_unconfirmed": (HARD,
        lambda v: "could not confirm the baseline took (%s)" % v),
    "setting_rejected": (HARD,
        lambda v: "the camera rejected %s"
                  % ", ".join(str(b[0]) for b in v)),
    "unverified": (HARD,
        lambda v: "could not read the camera back to confirm the gain (%s)" % v),
    "calibration_problem": (HARD, lambda v: str(v)),
    "apply_mismatch": (HARD,
        lambda v: "the FINAL write did not take: wanted gain %s / exposure %s, "
                  "camera reports gain %s / exposure %s"
                  % (v["wanted"][0], v["wanted"][1], v["got"][0], v["got"][1])),
    "apply_unconfirmed": (HARD,
        lambda v: "could not confirm the final write (%s)" % v),
    "final_state_wrong": (HARD,
        lambda v: "the camera did not END the run in the state we set: %s"
                  % ", ".join("%s is %s, should be %s" % (k, h, w)
                              for k, w, h in v)),
    "gain_scan_error": (HARD, lambda v: "the gain scan did not finish: %s" % v),
    "nt_server_not_stopped": (HARD, lambda v: str(v)),
    # ---- warn ----
    "video_mode_switched": (WARN,
        lambda v: "VIDEO MODE CHANGED %s -> %s - this camera is now running a "
                  "different RESOLUTION than it was. It had no calibration for "
                  "the old one." % (v[0], v[1])),
    "would_switch_video_mode": (WARN,
        lambda v: "would switch the video mode %s -> %s (not done: --dry-run)"
                  % (v[0], v[1])),
    "over_blur_budget": (WARN,
        lambda v: "%.1f px of motion blur, over the budget - fine on a bench, "
                  "smeared on a moving robot. Add light, or raise --max-gain." % v),
    "range_warning": (WARN,
        lambda v: "reference tag: %s"
                  % (("measured %.2f m against a stated %.2f m"
                      % (v["measured"], v["stated"])) if isinstance(v, dict) else v)),
    "fellback": (WARN,
        lambda v: "the chosen exposure failed its own verification; fell back to "
                  "the shortest exposure that had already passed"),
    "would_baseline": (WARN,
        lambda v: "would change %s (not done: --dry-run)" % ", ".join(sorted(v))),
}

HARD_KEYS = tuple(k for k, (sev, _) in PROBLEMS.items() if sev == HARD)
WARN_KEYS = tuple(k for k, (sev, _) in PROBLEMS.items() if sev == WARN)


def note_problem(rec, key, value=True):
    """Record a problem. The ONLY way to record one.

    Raises on an unregistered key rather than accepting it: an unregistered key
    is a problem the verdict cannot see, which is the whole bug class this
    registry exists to close.
    """
    if key not in PROBLEMS:
        raise KeyError("unregistered problem %r - add it to PROBLEMS, with a "
                       "deliberate HARD/WARN decision" % key)
    rec[key] = value
    return rec


def problem_notes(rec, severity=None):
    """[(key, severity, text)] for every problem recorded on this record."""
    out = []
    for key, (sev, describe) in PROBLEMS.items():
        v = rec.get(key)
        if not v:
            continue
        if severity is not None and sev != severity:
            continue
        try:
            text = describe(v)
        except Exception:
            text = "%s=%r" % (key, v)
        out.append((key, sev, text))
    return out


# Problems must survive being handed up through a recursion or a search.
#
# Every one of these was recorded and then lost. tune_camera escalates gain with
# `return await tune_camera(...)`, which discards the outer frame's dict, and the
# gain search calls tune_camera with skip_baseline=True, so the frame that ran the
# baseline is not the frame that returns. The record therefore belongs to the
# CALLER, which is the only party present for the whole camera.
FAILURE_KEYS = tuple(PROBLEMS)


def merge_failures(rec, r):
    """Fold a tune result into the caller's per-camera record, keeping problems.

    A plain dict.update() is not enough in either direction: the callee may know
    about a rejected setting the caller does not, and the caller may know about a
    baseline failure the callee (skip_baseline=True) never saw. Warnings need the
    same protection as failures - video_mode_switched is recorded by the caller
    and would otherwise be overwritten by a callee that never saw the switch.
    """
    keep = {k: rec[k] for k in FAILURE_KEYS if rec.get(k)}
    rec.update(r or {})
    rec.update(keep)
    return rec


def tune_failed(r, args=None):
    """Did this camera's tune fail? One definition, used by the CLI and the daemon.

    The daemon used to judge on `applied` alone, so a camera that came back
    rotated 90 degrees with a gain that never took still reported ok=true to the
    dashboard - the one signal a team actually trusts.
    """
    if any(r.get(k) for k in HARD_KEYS):
        return True
    if getattr(args, "baseline_only", False):
        return bool(r.get("error")) and "baseline only" not in str(r.get("error"))
    if getattr(args, "dry_run", False):
        # --dry-run applies nothing by design, so `applied` cannot be the test.
        # It used to skip the check altogether, which is how a dry run that
        # measured nothing at all still exited 0.
        return not r.get("would_apply")
    return not r.get("applied")


def run_problems(args):
    """Problems that belong to the RUN rather than to any one camera.

    "!! COULD NOT STOP the NetworkTables server" was printed and then dropped:
    the run exited 0 with PhotonVision still serving NetworkTables against the
    roboRIO, which is the single most disruptive state this tool can leave
    behind. It has no camera to hang off, so it lives here.
    """
    return dict(getattr(args, "_run_problems", {}) or {})


def note_run_problem(args, key, value=True):
    if not hasattr(args, "_run_problems") or args._run_problems is None:
        args._run_problems = {}
    note_problem(args._run_problems, key, value)
    return args._run_problems


def run_verdict(results, args):
    """(failed, warnings, exit_code) for a whole run. The CLI and daemon agree.

    Factored out of main() so the verdict can be tested directly - see
    sabotage_test.py --verdict-matrix, which asserts the exit code for every key
    in PROBLEMS one at a time.
    """
    rp = run_problems(args)
    failed = [r for r in (results or []) if tune_failed(r, args)]
    warnings = []
    for r in (results or []):
        for _k, _sev, text in problem_notes(r, WARN):
            warnings.append((r.get("camera", "?"), text))
    for _k, _sev, text in problem_notes(rp, WARN):
        warnings.append(("run", text))
    hard_run = problem_notes(rp, HARD)
    code = 1 if (failed or hard_run or not results) else 0
    return failed, warnings, code


async def tune_camera(pv, cam, args, log=print, progress=None, original=None,
                      skip_baseline=False, carry=None):
    name = cam["nickname"]
    unique = cam["uniqueName"]
    # `original` is threaded through gain re-sweeps. Re-deriving it there read
    # post-sweep state, so a later failure "restored" the camera to whatever the
    # last swept exposure happened to be.
    if original is None:
        original = {
            "cameraExposureRaw": cam["settings"]["cameraExposureRaw"],
            "cameraGain": cam["settings"]["cameraGain"],
            "cameraAutoExposure": cam["settings"]["cameraAutoExposure"],
        }
    log("── %s ──  current exposure %s, gain %s" % (name, original["cameraExposureRaw"], original["cameraGain"]))

    # Start from the BASELINE gain, not from whatever the last run left behind.
    # `original` stays the restore target - a failure must put back what the user
    # had - but it must not be the starting point, or tuning is a ratchet.
    if args.gain is not None:
        gain = args.gain
    elif skip_baseline:
        gain = original["cameraGain"]          # a re-sweep, already reset
    else:
        gain = getattr(args, "start_gain", BASELINE_START_GAIN)
        if abs(float(original["cameraGain"]) - float(gain)) > 1e-6:
            log("   gain: resetting to baseline %g (camera was at %g)"
                % (gain, original["cameraGain"]))
    result = {"camera": name, "uniqueName": unique, "original": original,
              "applied": None, "samples": [], "cliff": None, "gain": gain}
    # A gain escalation re-enters here and RETURNS the inner dict, so anything the
    # outer frame had already recorded has to be carried in explicitly.
    for _k in FAILURE_KEYS:
        if (carry or {}).get(_k):
            result[_k] = carry[_k]

    try:
        # Structural settings FIRST. Tuning exposure against a crushed brightness
        # just answers with a long, blurry exposure and calls it a success.
        if getattr(args, "baseline", True) and not skip_baseline:
            _changed, _failed = await assert_baseline(pv, cam, args, log)
            if _changed:
                fresh = await pv.cameras_fresh(timeout=8)
                newer = next((c for c in fresh if c["uniqueName"] == unique), None)
                if newer:
                    cam = dict(cam)
                    cam["settings"] = newer["settings"]
            result["baseline_changed"] = list(_changed)
            if _failed:
                result["baseline_failed"] = [list(map(str, f)) for f in _failed]
        # AFTER the baseline: solvePNPEnabled is PhotonVision's default-off, and
        # checking first meant the gate was blind on exactly the fresh camera it
        # exists for. Judge on what the camera will be, not what it was.
        _was = (cam.get("settings") or {}).get("cameraVideoModeIndex")
        cam, problem = await ensure_calibrated_mode(pv, cam, args, log)
        if (cam.get("settings") or {}).get("cameraVideoModeIndex") != _was:
            result["video_mode_switched"] = [_was,
                                             cam["settings"].get("cameraVideoModeIndex")]
        if problem:
            log("   !! CALIBRATION: %s" % problem)
            result["error"] = "no calibration for the active resolution"
            note_problem(result, "calibration_problem", problem)
            return result
        if getattr(args, "baseline_only", False):
            result["applied"] = None
            result["error"] = "baseline only - not tuned"
            return result

        bad = await pv.set_and_verify(unique, settle=max(args.settle, 1.5),
                                      cameraAutoExposure=False, cameraGain=gain)
        rejected = [b for b in bad if b[0] is not None]
        unverified = [b for b in bad if b[0] is None]
        if rejected:
            for k, want, have in rejected:
                log("   WARNING: %s did not take (wanted %s, camera has %s)" % (k, want, have))
            log("   the sweep below is NOT at the gain it claims - results are unreliable")
            note_problem(result, "setting_rejected",
                         [list(map(str, b)) for b in rejected])
        if unverified:
            # NOT a shrug any more. This is reached only after polling for the
            # whole confirm timeout, which means PhotonVision stopped answering
            # with cameraSettings at all - the state a bad pipeline selection
            # puts it in. Everything measured after it is unattributable: the
            # sweep cannot say what gain it ran at.
            log("   !! could not read the camera back to confirm the gain, after"
                " polling for %.0f s." % CONFIRM_TIMEOUT_S)
            log("   !! PhotonVision is not reporting cameraSettings. Nothing"
                " below can be attributed to a known gain.")
            note_problem(result, "unverified",
                         "no cameraSettings from PhotonVision for %.0f s"
                         % CONFIRM_TIMEOUT_S)

        # What is the camera doing RIGHT NOW, before we touch the exposure? Used
        # afterwards to tell a bad measurement from a genuine detection cliff.
        base_exposure = float(original.get("cameraExposureRaw") or 0.0)
        base_raw = await sample(pv, cam, args, max(1.5, args.dwell * 0.6), args.reference_tag)
        base_tags = (sum(base_raw["tags"]) / len(base_raw["tags"])) if base_raw["tags"] else 0.0
        log("   baseline: at its current %.0f the camera sees %.2f tags"
            % (base_exposure, base_tags))

        PENDING_RESTORE[unique] = dict(original)
        samples = []
        blanks = 0
        # Clamp to the camera's OWN reported bounds. PhotonVision clamps the
        # hardware call but stores the unclamped value, so a sweep past the limit
        # reads back "verified" while every step sits at the same real exposure
        # and scores identically. Bounds differ per model: this CSI OV9281 reports
        # 7-80000 us, while USB variants report entirely different ranges.
        lo, hi = args.min_exposure, args.max_exposure
        cmin, cmax = cam.get("minExposureRaw"), cam.get("maxExposureRaw")
        if cmin is not None and lo < float(cmin):
            log("   raising sweep floor %.0f -> %.0f (camera minimum)" % (lo, float(cmin)))
            lo = float(cmin)
        if cmax is not None and hi > float(cmax):
            log("   lowering sweep ceiling %.0f -> %.0f (camera maximum)" % (hi, float(cmax)))
            hi = float(cmax)
        sweep = geometric_sweep(lo, hi, args.steps)
        for step_i, exposure in enumerate(sweep):
            if progress:
                progress(step_i / float(len(sweep)))
            mark = capture_mark(args, cam)
            await pv.set_setting(unique, cameraExposureRaw=float(exposure))
            await asyncio.sleep(args.settle)
            raw = await sample(pv, cam, args, args.dwell, args.reference_tag, mark)

            s = Sample(exposure)
            s.frames = raw["frames"]
            s.multitag_solves = raw["solves"]
            s.reproj = raw["reproj"]
            s.tag_counts = raw["tags"]
            s.ambiguities = raw["amb"]
            s.ref_seen = raw["ref_seen"]
            s.ref_ranges = raw["ref_ranges"]
            samples.append(s)
            ok = s.passes(args.min_tags, args.max_ambiguity, args.reference_tag)
            mark = "ok " if ok else "   "
            log("   %s exposure %8.0f   %s" % (mark, exposure, s.summary(args.reference_tag)))

            # Overexposure is monotonic: once the image is too bright to detect a
            # tag, every LONGER exposure is worse. The sweep climbs, so a run of
            # dead steps at the top is physics, not information. Stop after two
            # consecutive blanks - two rather than one, so a single dropped frame
            # or a hand passing the lens cannot truncate the sweep.
            # Count FAILING steps, not blank ones. A dying exposure usually still
            # reports 1 tag rather than 0, so gating on "no tags at all" almost
            # never fires and the sweep runs to the top anyway.
            if not ok:
                blanks += 1
                if blanks >= 2 and any(x.passes(args.min_tags, args.max_ambiguity,
                                                args.reference_tag) for x in samples):
                    skipped = len(sweep) - step_i - 1
                    if skipped > 0:
                        log("   (stopping: 2 failed steps, %d longer exposure%s skipped "
                            "- they can only be worse)"
                            % (skipped, "" if skipped == 1 else "s"))
                    break
            else:
                blanks = 0

        result["samples"] = [
            {"exposure": s.exposure, "frames": s.frames, "solve_rate": s.solve_rate,
             "mean_tags": s.mean_tags, "med_reproj": s.med_reproj,
             "med_ambiguity": s.med_ambiguity,
             "ref_rate": s.ref_rate, "ref_range": s.ref_range,
             "passes": s.passes(args.min_tags, args.max_ambiguity, args.reference_tag)}
            for s in samples
        ]

        # Sanity-check the held tag really is at the distance the string says.
        if args.reference_tag is not None and args.reference_range:
            seen = [s for s in samples if s.ref_range]
            if seen:
                measured = _median([s.ref_range for s in seen])
                ratio = measured / args.reference_range
                log("   reference tag %d measured at %.2f m (string says %.2f m)"
                    % (args.reference_tag, measured, args.reference_range))
                if not 0.8 <= ratio <= 1.25:
                    log("   WARNING: measured range is %.0f%% of the stated range."
                        % (100 * ratio))
                    log("            Tuning at the wrong distance biases exposure badly -")
                    log("            too close under-exposes you for real field tags.")
                    note_problem(result, "range_warning",
                                 {"measured": measured,
                                  "stated": args.reference_range})
            else:
                log("   WARNING: reference tag %d never seen at any exposure." % args.reference_tag)
                note_problem(result, "range_warning", "never detected")

        # A single held tag is detected at shorter exposures than a full multi-tag
        # solve needs, so held-card mode measures a LOWER cliff than field
        # conditions require. Carry extra margin to compensate.
        contra = baseline_contradiction(samples, base_exposure, base_tags)
        if contra:
            log("   *** SWEEP CONTRADICTS THE CAMERA'S OWN STARTING STATE - NOT APPLYING ***")
            log("   At its incoming exposure %.0f the camera saw %.2f tags, but the"
                % (base_exposure, base_tags))
            log("   sweep claims %.0f - a LONGER exposure - saw only %.2f."
                % (contra[0], contra[1]))
            log("   The scene did not change that much; the measurement is wrong.")
            log("   Usual cause: something else reading PhotonVision's websocket at")
            log("   the same time. Stop it and re-run. Settings left untouched.")
            await pv.set_setting(unique, **original)
            result["error"] = ("sweep contradicts baseline (%.0f saw %.2f tags vs %.2f at %.0f)"
                               % (contra[0], contra[1], base_tags, base_exposure))
            return result

        holes = sweep_holes(samples)
        if holes:
            log("   *** THIS SWEEP IS NOT PHYSICALLY POSSIBLE - NOT APPLYING ***")
            log("   Detection vanished and came back as exposure increased:")
            for e, t in holes:
                log("     exposure %8.0f saw %.2f tags, but both shorter AND longer"
                    " exposures saw tags" % (e, t))
            log("   Brightness only goes up with exposure, so these samples are")
            log("   measurement artefacts, not the scene. Usual cause: something")
            log("   else reading PhotonVision's websocket at the same time, or the")
            log("   scene changed mid-sweep (someone walked in front of the camera).")
            log("   Re-run with nothing else talking to PhotonVision.")
            await pv.set_setting(unique, **original)
            result["error"] = "non-monotonic sweep (%d impossible samples)" % len(holes)
            return result

        bias = args.bias
        if args.reference_tag is not None:
            bias *= args.reference_bias
            log("   held-card mode: bias %.2f x %.2f = %.2f (single-tag detection is "
                "easier than a multi-tag solve)" % (args.bias, args.reference_bias, bias))
        chosen, shortest, cliff, tag_target = choose_exposure(
            samples, bias, args.min_tags, args.max_ambiguity, args.reference_tag,
            args.tag_fraction)
        if tag_target:
            log("   best exposure saw %.2f tags - requiring >= %.2f (%.0f%%)"
                % (max(s.mean_tags for s in samples), tag_target, 100*args.tag_fraction))
        if chosen is None:
            # Exposure alone could not get there. Raising gain buys brightness
            # WITHOUT buying motion blur, so escalate gain rather than accept a
            # long exposure - which is what a naive tuner would do.
            if args.gain_steps and gain < args.max_gain:
                nxt, one_jump = _escalate_to(args, gain)
                log("   nothing passed at gain %g - retrying at gain %g" % (gain, nxt))
                log("   (gain costs noise; exposure costs blur - prefer gain)")
                args.gain = nxt
                args.gain_steps = 0 if one_jump else args.gain_steps - 1
                return await tune_camera(pv, cam, args, log, progress,
                                         original=original, skip_baseline=True,
                                         carry=result)
            log("   NOTHING PASSED even at gain %g." % gain)
            log("   That is a LIGHTING or CONFIG problem, not an exposure one:")
            log("     - check cameraBrightness (a low value crushes the image to black)")
            log("     - check the camera is actually pointed at tags")
            log("     - add light, or raise --max-gain")
            await pv.set_setting(unique, **original)
            PENDING_RESTORE.pop(unique, None)
            result["error"] = "no passing exposure even at max gain"
            return result

        result["cliff"] = cliff
        log("   cliff ~%.0f (bracketed), shortest verified pass %.0f, bias %.2f  ->  %.0f"
            % (cliff, shortest.exposure, bias, chosen))
        fx = camera_fx(cam, args.fx)
        if abs(fx - args.fx) > 1.0:
            log("   fx %.1f from this camera's calibration (default was %.1f)"
                % (fx, args.fx))
        blur = lambda deg: math.radians(deg) * (chosen / 1e6) * fx
        log("   predicted blur: %.1f px @90deg/s, %.1f px @%.0fdeg/s"
            % (blur(90), blur(args.blur_rate), args.blur_rate))

        # Blur is a BUDGET, not a footnote. Until now it was printed and ignored,
        # so a camera at low gain could be handed an 18814 us exposure - 131 px of
        # smear across a ~70 px tag, i.e. blind the moment the robot moves - and
        # the tool would call it a pass because the bench was stationary.
        # Gain costs noise; exposure costs blur. Trade them.
        predicted = blur(args.blur_rate)
        # Only worth raising gain if there is room to go SHORTER. When the cliff
        # already sits at the bottom of the sweep, more gain cannot buy a shorter
        # exposure - it just re-sweeps to the same answer until gain_steps runs
        # out, a minute a time. Seen looping at 10.4 px against a 10 px budget.
        at_floor = shortest.exposure <= args.min_exposure * 1.25
        if predicted > args.max_blur_px * 1.15 and not at_floor:
            if args.gain_steps and gain < args.max_gain:
                nxt, one_jump = _escalate_to(args, gain)
                log("   %.0f px of blur at %.0f deg/s exceeds the %.0f px budget"
                    % (predicted, args.blur_rate, args.max_blur_px))
                log("   raising gain %g -> %g and re-sweeping (gain costs noise, "
                    "exposure costs blur)" % (gain, nxt))
                if one_jump:
                    # Straight to the top, and only once. If the most gain
                    # available cannot buy a shorter exposure then no value in
                    # between can either, and each intermediate step costs a
                    # whole sweep - measured 23 s. Climbing 0 -> 10 -> 20 cost
                    # two extra sweeps per camera and ended at the same 1500 us.
                    # Which gain is actually APPLIED is decided afterwards by the
                    # fixed-exposure scan, so overshooting here is free.
                    log("   (straight to the top of the range; the gain finally "
                        "applied is chosen by the fixed-exposure scan)")
                # More gain moves the cliff DOWN or leaves it - it can never need
                # a LONGER exposure - so bracket the previous answer instead of
                # re-exploring the whole range.
                args.max_exposure = min(args.max_exposure,
                                        max(args.min_exposure * 1.5,
                                            shortest.exposure * 1.5))
                args.steps = max(3, min(args.steps, 4))
                args.gain = nxt
                args.gain_steps = 0 if one_jump else args.gain_steps - 1
                return await tune_camera(pv, cam, args, log, progress,
                                         original=original, skip_baseline=True,
                                         carry=result)
            log("   !! %.1f px of blur at %.0f deg/s, over the %.0f px budget, and"
                % (predicted, args.blur_rate, args.max_blur_px))
            log("   !! gain is already at %g. Works on a BENCH; will smear on a"
                % gain)
            log("   !! moving robot. Add light, or raise --max-gain.")
            note_problem(result, "over_blur_budget", round(predicted, 1))
        elif predicted > args.max_blur_px and at_floor:
            log("   note: %.1f px of blur at %.0f deg/s is over the %.0f px budget,"
                % (predicted, args.blur_rate, args.max_blur_px))
            log("   but the cliff is already at the bottom of the sweep (%.0f us) -"
                % args.min_exposure)
            log("   more gain cannot buy a shorter exposure. Lower --min-exposure"
                " to explore further, or add light.")
            note_problem(result, "over_blur_budget", round(predicted, 1))

        if args.dry_run:
            log("   dry run - would set exposure %.0f at gain %g; restoring original"
                % (chosen, gain))
            # Record the choice where the harvester looks. The gain search
            # harvested on r["applied"], which dry-run never sets, so
            # --optimise-gain --dry-run logged "no passing exposure" for every
            # candidate while the suppressed trial log showed each one
            # succeeding, then fell through to yet another full sweep.
            result["would_apply"] = chosen
            await pv.set_setting(unique, **original)
        else:
            mark = capture_mark(args, cam)
            await pv.set_setting(unique, cameraExposureRaw=float(chosen), cameraGain=gain,
                                 cameraAutoExposure=False)
            await asyncio.sleep(args.settle)
            raw = await sample(pv, cam, args, args.dwell, args.reference_tag, mark)
            check = Sample(chosen)
            check.frames = raw["frames"]; check.multitag_solves = raw["solves"]
            check.reproj = raw["reproj"]; check.tag_counts = raw["tags"]
            check.ambiguities = raw["amb"]
            check.ref_seen = raw["ref_seen"]; check.ref_ranges = raw["ref_ranges"]
            if check.passes(args.min_tags, args.max_ambiguity, args.reference_tag):
                log("   applied %.0f - verified: %s" % (chosen, check.summary(args.reference_tag)))
                result["applied"] = chosen
            else:
                log("   %.0f FAILED verification (%s)" % (chosen, check.summary(args.reference_tag)))
                log("   falling back to shortest verified pass: %.0f" % shortest.exposure)
                await pv.set_setting(unique, cameraExposureRaw=float(shortest.exposure))
                await asyncio.sleep(args.settle)
                result["applied"] = shortest.exposure
                note_problem(result, "fellback", True)
        PENDING_RESTORE.pop(unique, None)
        return result

    except BaseException as exc:
        # BaseException, not Exception. Ctrl-C raises KeyboardInterrupt, which is
        # NOT an Exception - so an interrupt during the sweep skipped this restore
        # and left the camera sitting at whatever exposure was being tested.
        # asyncio.CancelledError is the same family. Restore, then re-raise those
        # two so the interrupt still ends the program.
        log("   %s (%s) - restoring original settings"
            % ("INTERRUPTED" if isinstance(exc, (KeyboardInterrupt, asyncio.CancelledError, Terminated))
               else "ERROR", exc or type(exc).__name__))
        restored = False
        try:
            await pv.set_setting(unique, **original)
            await asyncio.sleep(0.4)          # let the write reach PhotonVision
            restored = True
        except BaseException:
            pass
        if restored:
            PENDING_RESTORE.pop(unique, None)
        else:
            # The socket is gone, so nothing was restored - keep the record so
            # main() can replay it on a fresh connection. Dropping it here meant
            # a dropped websocket mid-sweep left the camera parked at whatever
            # exposure was being tested, while the log claimed a restore.
            PENDING_RESTORE[unique] = dict(original)
            log("   restore did NOT reach the camera - will retry on a fresh connection")
        if isinstance(exc, (KeyboardInterrupt, asyncio.CancelledError, Terminated)):
            PENDING_RESTORE[unique] = dict(original)   # loop is dying; main() retries
            raise
        result["error"] = str(exc)
        return result


async def calibrate_reference_bias(args, log=print):
    """Measure YOUR reference-bias by running both modes back to back.

    A single held tag is an easier detection problem than a full multi-tag solve,
    so held-card mode measures a lower cliff than field conditions require. The
    ratio is NOT a universal constant - it depends on how many field tags your
    camera sees and how far away the hardest one is - so it has to be measured
    per camera, once, with representative tags in view.
    """
    if args.reference_tag is None:
        sys.exit("--calibrate-reference-bias needs --reference-tag (the card you will hold)")

    held_tag, held_range = args.reference_tag, args.reference_range
    dry, bias_in = args.dry_run, args.reference_bias
    args.dry_run = True          # never disturb the camera's settings while calibrating
    args.reference_bias = 1.0    # measure the raw cliffs, unbiased

    log("=" * 66)
    log("STEP 1/2  held card - hold tag %d steady%s" % (
        held_tag, (" at %.2f m" % held_range) if held_range else ""))
    log("=" * 66)
    held = await run(args, log=log)

    log("")
    log("=" * 66)
    log("STEP 2/2  field tags - you can put the card down now")
    log("=" * 66)
    args.reference_tag, args.reference_range = None, None
    field = await run(args, log=log)

    args.dry_run, args.reference_bias = dry, bias_in
    args.reference_tag, args.reference_range = held_tag, held_range

    log("")
    log("=" * 66)
    log("RESULT")
    log("=" * 66)
    by_name = {r["camera"]: r for r in field}
    out = []
    for h in held:
        f = by_name.get(h["camera"])
        hc, fc = h.get("cliff"), (f or {}).get("cliff")
        if not hc or not fc:
            log("  %-14s could not measure both modes - skipped" % h["camera"])
            continue
        ratio = fc / hc
        out.append((h["camera"], hc, fc, ratio))
        log("  %-14s held-card cliff %7.0f   field cliff %7.0f   ratio %.2f"
            % (h["camera"], hc, fc, ratio))
        if ratio < 1.0:
            log("     SUSPECT: the held card needed MORE exposure than the field tags.")
            log("     A single close tag should be EASIER, so this usually means the card")
            log("     was not presented consistently - drooping, tilted, glare, or moved")
            log("     mid-sweep. Re-run holding it square and steady, or tape it up.")
    if not out:
        log("  nothing measured. Are field tags visible, and was the card held throughout?")
        return out
    worst = max(r for _, _, _, r in out)
    log("")
    if len(out) > 1:
        log("  Cameras differ; using the largest ratio so no camera is under-exposed.")
    log("  Put this in your systemd unit / command line:")
    log("")
    log("      --reference-bias %.2f" % worst)
    log("")
    log("  Valid while your camera geometry stays similar. Re-measure if you move")
    log("  or re-aim a camera, or if the tags you rely on change distance.")
    return out


async def _gain_trial(pv, cam, args, gain, exposure, settle_extra=0.0):
    """Detection quality at ONE gain, with the exposure held where it is."""
    mark = capture_mark(args, cam)
    await pv.set_setting(cam["uniqueName"], cameraAutoExposure=False,
                         cameraGain=int(round(gain)),
                         cameraExposureRaw=float(exposure))
    await asyncio.sleep(args.settle + settle_extra)
    raw = await sample(pv, cam, args, args.dwell, args.reference_tag, mark)
    s = Sample(exposure)
    s.frames = raw["frames"]
    s.multitag_solves = raw["solves"]
    s.reproj = raw["reproj"]
    s.tag_counts = raw["tags"]
    s.ambiguities = raw["amb"]
    s.ref_seen = raw["ref_seen"]
    s.ref_ranges = raw["ref_ranges"]
    return s


async def optimise_gain(pv, cam, args, log=print, progress=None, skip_baseline=False):
    """ONE exposure sweep, then a gain scan with that exposure held fixed.

    The old shape ran a whole tune_camera - a complete exposure sweep - once per
    gain candidate, six times over. MEASURED on this rig: 91.6 s per camera, and
    what it bought was a decision smaller than its own noise. Holding exposure
    fixed and repeating the gain scan three times, reprojection at gain 40 read
    1.100 / 0.683 / 0.431 px - a 2.55x spread at ONE gain - while the decision
    the tool actually made was gain 80 (0.471) over gain 40 (0.563), a 1.19x
    difference sitting well inside that spread. It chose 60, 80 and 100 on
    different runs in the same room and the same light.

    MEASURED costs of the two halves: one 8-step exposure sweep 23 s; a
    fixed-exposure scan over all six gain candidates 13.2 s (three repeats:
    13.3 / 13.2 / 13.2). 36 s per camera instead of 91.6 s.

    Re-sweeping exposure per gain was never needed anyway: across every run on
    this rig the cliff sat at 1500 us for every gain from 20 to 100.
    """
    unique = cam["uniqueName"]
    nick = cam["nickname"]

    # ---- phase 1: find the exposure, once, with the log VISIBLE ----------
    # The old code passed log=lambda m: None here, which is why a connection
    # death, a calibration refusal and a dark room all printed as the same
    # "no passing exposure".
    saved_gain, saved_steps = args.gain, args.gain_steps
    saved_bounds = (args.min_exposure, args.max_exposure, args.steps)
    saved_follows = getattr(args, "_gain_scan_follows", False)
    args._gain_scan_follows = True      # lets phase 1 escalate in one jump
    # Pin the starting gain explicitly. tune_camera's skip_baseline branch falls
    # back to the camera's CURRENT gain, which is the ratchet this tool exists to
    # avoid: measured, a first pass at a camera left on gain 100 swept at 100 and
    # then scanned only [100], so the search could never go back down.
    if args.gain is None:
        args.gain = float(getattr(args, "start_gain", BASELINE_START_GAIN))
    try:
        r = await tune_camera(
            pv, cam, args, log=log,
            progress=(lambda f: progress(0.55 * f)) if progress else None,
            skip_baseline=skip_baseline)
    finally:
        # tune_camera writes args.gain and narrows the sweep bounds during an
        # escalation, and args is shared across cameras - camera 1's answer used
        # to become camera 2's start.
        args.gain, args.gain_steps = saved_gain, saved_steps
        args.min_exposure, args.max_exposure, args.steps = saved_bounds
        args._gain_scan_follows = saved_follows

    exposure = r.get("applied") or r.get("would_apply")
    if not exposure:
        return r                 # the sweep already said why, in the real log

    original = r.get("original") or {}

    # ---- phase 2: scan gain with that exposure held ----------------------
    # The FULL range, not "from whatever the sweep ended at". Phase 1 may have
    # jumped to the ceiling just to find an exposure; the whole point of the scan
    # is to walk that back down, and a trial that cannot see the tags at a low
    # gain drops out on its own merits.
    lo = float(saved_gain if saved_gain is not None
               else getattr(args, "start_gain", BASELINE_START_GAIN))
    hi = float(args.max_gain)
    n = max(1, int(args.gain_search_steps))
    if n == 1 or hi <= lo:
        gains = [lo]
    else:
        gains = [lo + (hi - lo) * i / float(n - 1) for i in range(n)]
    gains = sorted({int(round(g)) for g in gains})
    log("   gain scan at exposure %.0f over %s" % (exposure, gains))

    # Phase 1 popped this on success, but the camera is about to be moved again.
    if original:
        PENDING_RESTORE[unique] = dict(original)

    steps = len(gains) + (1 if len(gains) > 1 else 0)
    trials = []
    reasons = {}
    for i, gv in enumerate(gains):
        if progress:
            progress(0.55 + 0.45 * i / steps)
        try:
            s = await _gain_trial(pv, cam, args, gv, exposure)
        except Exception as exc:
            # Abort the SCAN, not the run. Running the remaining trials against a
            # dead socket produced five more silent no-ops and then blamed the
            # lighting; the sweep's own answer is still good and is kept.
            log("       gain %-5d  ABORTED: %s (%s)"
                % (gv, type(exc).__name__, exc or "no detail"))
            log("   stopping the gain scan with %d candidate(s) untried - a dead "
                "connection is not a dark room. Keeping the sweep's answer."
                % (len(gains) - i))
            r["gain_scan_error"] = "%s during the gain scan" % type(exc).__name__
            break
        why = s.why_failed(args.min_tags, args.max_ambiguity, args.reference_tag)
        trials.append((gv, s.med_reproj if why is None else None, s))
        if why is None:
            log("   ok  gain %-5d  %s" % (gv, s.summary(args.reference_tag)))
        else:
            reasons[gv] = why
            log("       gain %-5d  %s  - %s" % (gv, s.summary(args.reference_tag), why))

    # ---- phase 3: how much of that spread is just measurement noise? -----
    # Re-measure the FIRST candidate at the end of the scan. That is the honest
    # unit to compare the others against: it is the same quantity, measured
    # twice, across exactly the interval the scan itself spans.
    noise = None
    # Repeat a gain that actually PASSED, not blindly the first candidate.
    # Repeating a candidate that saw nothing yields no reprojection and so no
    # noise estimate at all - measured: gain 0 failed on OV9281 (1), the repeat
    # was wasted, and the fallback band then picked gain 20 (reproj 2.052) over
    # gain 100 (1.597), calling a 28% difference insignificant.
    repeatable = next((g for g, rp, _ in trials if rp is not None), None)
    if len(gains) > 1 and repeatable is not None and not r.get("gain_scan_error"):
        if progress:
            progress(0.55 + 0.45 * len(gains) / steps)
        try:
            again = await _gain_trial(pv, cam, args, repeatable, exposure)
        except Exception as exc:
            log("   repeat of gain %d failed: %s - no noise estimate this run"
                % (repeatable, type(exc).__name__))
            r["gain_scan_error"] = "%s during the repeat" % type(exc).__name__
            again = None
        first_rp = next(rp for g, rp, _ in trials if g == repeatable)
        rp2 = (again.med_reproj
               if (again is not None
                   and again.passes(args.min_tags, args.max_ambiguity, args.reference_tag))
               else None)
        if first_rp and rp2:
            noise = abs(math.log(rp2 / first_rp))
            log("   repeat of gain %d: reproj %.3f then %.3f - this run's own "
                "repeat noise is %.0f%%"
                % (repeatable, first_rp, rp2, 100 * (math.exp(noise) - 1)))

    good = [t for t in trials if t[1] is not None]
    if not good:
        log("   no gain in the scan measured usable reprojection - keeping the "
            "sweep's own gain %g. Reasons: %s"
            % (lo, "; ".join("gain %d: %s" % (g, w) for g, w in sorted(reasons.items()))
               or r.get("gain_scan_error", "none recorded")))
        PENDING_RESTORE.pop(unique, None)
        return r

    best_rp = min(t[1] for t in good)
    # --reproj-tolerance keeps its own job: REJECT a gain whose fit is clearly
    # worse than the best. It is not a noise estimate and must not be used as
    # one - at its 1.5 default it declares a 28% difference insignificant.
    usable = [t for t in good if t[1] <= best_rp * float(args.reproj_tolerance)]
    # One repeat is a crude estimate, so floor the band at 5%: a fluke pair of
    # near-identical readings must not make every 1% difference "significant".
    # With no repeat at all, 5% is also the fallback - claiming a wider band
    # without having measured it is exactly the dishonesty this replaces.
    band = max(math.exp(noise) if noise is not None else 1.0, 1.05)
    contenders = [t for t in usable if t[1] <= best_rp * band]
    pick = min(contenders, key=lambda t: t[0])       # LOWEST gain wins a tie
    best = min(good, key=lambda t: t[1])

    log("      %s" % ", ".join("gain %d -> %.3f" % (g, rp) for g, rp, _ in good))
    if len(good) == 1:
        log("   only gain %d measured usable reprojection (%.3f) at exposure %.0f"
            % (pick[0], pick[1], exposure))
    elif pick[0] == best[0]:
        log("   chose gain %d at exposure %.0f - reproj %.3f, the best measured "
            "and outside this run's %.0f%% noise band"
            % (pick[0], exposure, pick[1], 100 * (band - 1)))
        if pick[0] >= max(t[0] for t in trials) and len(trials) > 1:
            log("      NOTE: that is the TOP of the search range - the optimum "
                "may be higher. Raise --max-gain to find it.")
    else:
        log("   chose gain %d (reproj %.3f) over gain %d (reproj %.3f) at "
            "exposure %.0f" % (pick[0], pick[1], best[0], best[1], exposure))
        log("      the difference is INSIDE this run's %.0f%% measurement noise, "
            "so it is NOT significant - and the lower gain is the one with less "
            "sensor noise." % (100 * (band - 1)))

    r["gain_trials"] = [{"gain": g, "reproj": rp, "frames": s.frames,
                         "mean_tags": s.mean_tags, "solve_rate": s.solve_rate,
                         "why_failed": reasons.get(g)}
                        for g, rp, s in trials]
    r["gain_noise_pct"] = None if noise is None else round(100 * (math.exp(noise) - 1), 1)
    r["gain_significant"] = (pick[0] == best[0])

    # Explicitly apply the winner. The scan leaves whichever gain was tried LAST
    # on the camera - the repeat, in fact - which is the winner only by luck.
    if not args.dry_run:
        # Guarded for the same reason as the trials: if the socket died during
        # the scan, the apply cannot succeed, and a traceback out of here loses
        # the whole run's result. PENDING_RESTORE is deliberately left in place
        # on failure so the exit-path replay puts the camera back.
        try:
            await pv.set_setting(unique, cameraAutoExposure=False,
                                 cameraGain=int(pick[0]), cameraExposureRaw=float(exposure))
            await asyncio.sleep(max(args.settle, 1.5))
            # cameraSettings lags a second or two, so a plain read here reports
            # the PREVIOUS value and cries MISMATCH on a write that succeeded.
            live = await pv.cameras_fresh(timeout=10)
            got = next((c for c in live if c["uniqueName"] == unique), None)
        except Exception as exc:
            log("   could not apply gain %d / exposure %.0f: %s"
                % (pick[0], exposure, type(exc).__name__))
            r["error"] = "could not apply the tuned settings (%s)" % type(exc).__name__
            r["applied"] = None
            return r
        if got:
            gs = got["settings"]
            ok = (abs(float(gs["cameraGain"]) - pick[0]) < 0.51
                  and abs(float(gs["cameraExposureRaw"]) - exposure) < 1e-6)
            log("   applied gain %d / exposure %.0f - camera reports %s / %s  %s"
                % (pick[0], exposure, gs["cameraGain"], gs["cameraExposureRaw"],
                   "confirmed" if ok else "*** MISMATCH ***"))
            if not ok:
                r["apply_mismatch"] = {"wanted": [pick[0], exposure],
                                       "got": [gs["cameraGain"], gs["cameraExposureRaw"]]}
        r["applied"] = exposure
        r["gain"] = pick[0]
        PENDING_RESTORE.pop(unique, None)
    else:
        log("   dry run - would set gain %d at exposure %.0f; restoring original"
            % (pick[0], exposure))
        r["would_apply"] = exposure
        r["would_gain"] = pick[0]
        await pv.set_setting(unique, **original)
        await asyncio.sleep(0.4)
        PENDING_RESTORE.pop(unique, None)
    if progress:
        progress(1.0)
    return r


class AlreadyRunning(Exception):
    """Another photontune already holds the run lock."""


class RunLock:
    """Advisory lock, so two photontunes cannot fight over the same cameras.

    This ships as a daemon AND a CLI on the same box, so a human running the CLI
    while the daemon's autorun fires is normal rather than exotic, and nothing
    detected it. Overlapping runs are not merely wasteful: MEASURED on this rig,
    the second run's NetworkTables-server toggle calls
    NetworkManager.reinitialize(), which killed the first run's websocket with
    "no close frame received or sent" and left the camera parked mid-sweep at
    9966 us on gain 0.

    Takes BOTH paths rather than the first one that works. /run is the
    conventional home but is root-only here (drwxr-xr-x root root), and the
    daemon runs as root while a human's CLI does not - so choosing one path by
    availability would put the two parties on DIFFERENT files and exclude
    nothing at all. /tmp is world-writable and is what actually guarantees
    exclusion; /run is taken as well when we can get it, and created 0666 so the
    unprivileged side can lock it too.
    """
    PATHS = ("/run/photontune.lock", "/tmp/photontune.lock")

    def __init__(self):
        self._held = []
        self.holder = None

    @staticmethod
    def _open(path):
        import os
        try:
            fd = os.open(path, os.O_RDWR | os.O_CREAT, 0o666)
        except OSError:
            try:
                # flock() works on a read-only descriptor, unlike fcntl.lockf -
                # so an unprivileged run can still contend for a root-owned file.
                fd = os.open(path, os.O_RDONLY)
            except OSError:
                return None
        try:
            os.fchmod(fd, 0o666)
        except OSError:
            pass
        return fd

    @staticmethod
    def _who(fd):
        import os
        try:
            os.lseek(fd, 0, os.SEEK_SET)
            return os.read(fd, 400).decode("utf-8", "replace").strip()
        except OSError:
            return ""

    def acquire(self):
        import fcntl, os
        got = []
        for path in self.PATHS:
            fd = self._open(path)
            if fd is None:
                continue
            try:
                fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
            except OSError:
                self.holder = self._who(fd) or "pid unknown"
                os.close(fd)
                for f in got:
                    try:
                        fcntl.flock(f, fcntl.LOCK_UN)
                        os.close(f)
                    except OSError:
                        pass
                return False
            try:
                os.ftruncate(fd, 0)
                os.write(fd, ("pid %d: %s\n"
                              % (os.getpid(), " ".join(sys.argv[1:]))).encode())
            except OSError:
                pass              # read-only descriptor; the lock still holds
            got.append(fd)
        self._held = got
        return True

    def release(self):
        import fcntl, os
        for fd in self._held:
            try:
                fcntl.flock(fd, fcntl.LOCK_UN)
            except OSError:
                pass
            try:
                os.close(fd)
            except OSError:
                pass
        self._held = []

    def message(self):
        return ("another photontune is already running (%s).\n"
                "  Two at once fight over the same cameras: the second one's "
                "NetworkTables-server\n"
                "  toggle bounces the network stack and kills the first one's "
                "websocket mid-sweep,\n"
                "  leaving a camera at whatever exposure it was testing. Wait "
                "for it, or stop it." % (self.holder or "pid unknown"))


async def run(args, log=print, progress=None, on_camera=None):
    lock = RunLock()
    if not lock.acquire():
        raise AlreadyRunning(lock.message())
    try:
        return await _run_locked(args, log, progress, on_camera)
    finally:
        lock.release()


async def _run_locked(args, log=print, progress=None, on_camera=None):
    args._nt_we_started = False
    args._nt_cfg = None
    args._run_problems = {}      # per RUN, not per camera; reset every trigger
    if not getattr(args, "no_nt", False) and getattr(args, "manage_nt_server", True):
        # Must happen BEFORE the Photon websocket is opened. Toggling the NT
        # server posts to /api/settings/general, which also calls
        # NetworkManager.reinitialize() and bounces the interface - killing any
        # connection we are already holding ("no close frame received or sent").
        cfg = await read_network_config(args.host, args.port)
        if cfg and not cfg.get("runNTServer"):
            target = cfg.get("ntServerAddress") or args.host
            if not nt_server_reachable(target):
                log("no NetworkTables server at %s - starting PhotonVision's"
                    " temporarily (stopped again afterwards)" % target)
                args._nt_cfg = cfg
                # Record that we ASKED before checking whether it worked: if it
                # half-worked we still own turning it off again.
                args._nt_we_started = True
                if not await set_nt_server(args.host, cfg, True, args.port, log):
                    log("   continuing without it; sampling will fall back to "
                        "the throttled websocket")
    try:
        return await _run_inner(args, log, progress, on_camera)
    finally:
        if args._nt_we_started:
            # Log AFTER the read-back, not before it. This line used to print
            # unconditionally and was the only record anyone had.
            if await set_nt_server(args.host, args._nt_cfg, False, args.port, log):
                log("stopped the NetworkTables server we started")
            else:
                log("!! COULD NOT STOP the NetworkTables server we started - "
                    "PhotonVision is still serving NetworkTables and will fight "
                    "the roboRIO. Turn runNTServer off in the dashboard.")
                # Recorded, not merely printed. This exited 0: the tune was fine
                # and the coprocessor was left serving NetworkTables against the
                # roboRIO, which is the most disruptive state this tool can
                # create and the one a human most needs to be told about.
                note_run_problem(
                    args, "nt_server_not_stopped",
                    "PhotonVision is STILL serving NetworkTables - photontune "
                    "started it and could not stop it again. It will fight the "
                    "roboRIO. Turn runNTServer off in the dashboard.")


async def _run_inner(args, log=print, progress=None, on_camera=None):
    async with Photon(args.host, args.port) as pv:
        cams = await pv.cameras()
        if not cams:
            raise LookupError("no cameras reported by PhotonVision at %s" % args.host)
        if args.cameras:
            want = {c.strip().lower() for c in args.cameras.split(",")}
            matched = [c for c in cams
                       if c["nickname"].lower() in want or c["uniqueName"].lower() in want]
            if not matched:
                raise LookupError(
                    "no camera matched %r. Available: %s"
                    % (args.cameras, ", ".join(c["nickname"] for c in cams)))
            cams = matched
        log("tuning %d camera(s): %s" % (len(cams), ", ".join(c["nickname"] for c in cams)))
        args._nt = {}
        if not getattr(args, "no_nt", False):
            # Try the given server, then loopback. Running ON the coprocessor,
            # --host defaults to photonvision.local, and resolving its own mDNS
            # name does not connect - so a bare invocation silently fell back to
            # the slow websocket path. 127.0.0.1 is harmless to try elsewhere:
            # if nothing is listening, available() just returns False.
            cands = [args.nt_server or args.host]
            if "127.0.0.1" not in cands:
                cands.append("127.0.0.1")
            for ntsrv in cands:
                try:
                    readers = {}
                    for c in cams:
                        r = NTResults(ntsrv, c["nickname"])
                        if not r.available():
                            r.close()
                            for done in readers.values():
                                done.close()
                            readers = {}
                            break
                        readers[c["nickname"]] = r
                    if readers:
                        args._nt = readers
                        log("   NetworkTables server: %s" % ntsrv)
                        break
                except Exception as exc:
                    log("   NT unavailable via %s (%s)" % (ntsrv, exc))
        if args._nt:
            log("   sampling over NetworkTables (~4.8x the frames of the websocket)")
        else:
            log("   sampling over the websocket - it is throttled to ~9 fps, so each")
            log("   sweep point is scored on ~30 frames. Run with an NT server"
                " reachable for better data.")
        results = []
        return await _tune_all(pv, cams, args, log, progress, on_camera, results)


async def _tune_all(pv, cams, outer_args, log, progress, on_camera, results):
        for idx, cam in enumerate(cams):   # sequential: avoids cameras perturbing each other
            # EVERY camera gets its own argument state. Not a save/restore around
            # each call - a copy, so there is no door left to leave open.
            #
            # This bug arrived twice through two different doors. The first fix
            # put a save/restore inside optimise_gain; _tune_all then called
            # tune_camera DIRECTLY on the --no-optimise-gain path with no restore
            # at all, and the same bug came straight back. FORCED: camera 1
            # escalated 0 -> 10 -> 20 -> 30 and narrowed its sweep to a 4-point
            # 200/471/1111/2620 grid; camera 2 then logged "gain is already at
            # 30", never printed "resetting to baseline 0", and swept camera 1's
            # leftover grid instead of its own range. Exit 0.
            #
            # tune_camera mutates args.gain, args.gain_steps, args.min_exposure,
            # args.max_exposure and args.steps as it escalates. A shallow copy
            # keeps those per camera while sharing _nt (the readers are meant to
            # be shared) - so a third door cannot open.
            # Rebinding `args` itself, not introducing a second name: every
            # line below - and every line anyone adds below - uses the
            # per-camera copy automatically. A `cam_args` alias standing next to
            # a live `args` would leave the same door open for the next edit to
            # walk through, and this bug has already come back once that way.
            args = copy.copy(outer_args)
            def cam_progress(f, idx=idx):
                if progress:
                    progress((idx + f) / float(len(cams)))
            if on_camera:
                on_camera(cam["nickname"], idx, len(cams))
            # THE per-camera record. It is built here and not by tune_camera,
            # because this frame is the only one present for the whole camera:
            # the gain search calls tune_camera with skip_baseline=True and a gain
            # escalation returns a fresh inner dict, so a failure recorded down
            # there had no way back up. Measured: a run whose baseline did not
            # apply exited 0.
            rec = {"camera": cam["nickname"], "uniqueName": cam["uniqueName"],
                   "applied": None}
            if args.optimise_gain:
                # Structural settings BEFORE the gain search, not inside it.
                # optimise_gain runs tune_camera with its logging discarded, so a
                # baseline failure was both invisible and too late: measured, a
                # camera left at inputImageRotationMode=1 made all six gain trials
                # report "no passing exposure" - six full sweeps burned against a
                # camera that was broken in a way the tool already knew how to fix,
                # and the fix only landed afterwards in the fallback path.
                if getattr(args, "baseline", True):
                    _ch, _fl = await assert_baseline(pv, cam, args, log)
                    if _ch:
                        fresh = await pv.cameras_fresh(timeout=8)
                        newer = next((c for c in fresh
                                      if c["uniqueName"] == cam["uniqueName"]), None)
                        if newer:
                            cam = dict(cam)
                            cam["settings"] = newer["settings"]
                        rec["baseline_changed"] = list(_ch)
                    if _fl:
                        log("   !! baseline did not fully apply: %s"
                            % ", ".join(str(f[0]) for f in _fl))
                        log("   !! tuning on top of settings that are still wrong")
                        rec["baseline_failed"] = [list(map(str, f)) for f in _fl]
                # AFTER the baseline, because the baseline is what turns solvePNP
                # on, and the gate was silent while it was off.
                was = (cam.get("settings") or {}).get("cameraVideoModeIndex")
                cam, _prob = await ensure_calibrated_mode(pv, cam, args, log)
                if (cam.get("settings") or {}).get("cameraVideoModeIndex") != was:
                    rec["video_mode_switched"] = [was,
                                                  cam["settings"].get("cameraVideoModeIndex")]
                if _prob:
                    log("   !! CALIBRATION: %s" % _prob)
                    rec["error"] = "no calibration for the active resolution"
                    note_problem(rec, "calibration_problem", _prob)
                    results.append(rec)
                    continue
                r = await optimise_gain(pv, cam, args, log, cam_progress,
                                        skip_baseline=True)
                if r is not None:
                    results.append(merge_failures(rec, r))
                    continue
            # Held-card mode: the card is only visible to one camera at a time,
            # so give whoever is holding it a chance to move before we sweep.
            if args.reference_tag is not None and idx > 0 and args.move_pause > 0:
                log("   move the card to '%s' - %.0fs" % (cam["nickname"], args.move_pause))
                await asyncio.sleep(args.move_pause)
            results.append(merge_failures(
                rec, await tune_camera(pv, cam, args, log, cam_progress, carry=rec)))
        if progress:
            progress(1.0)
        return results


# ───────────────────────── B: NetworkTables trigger ─────────────────────────

class Terminated(BaseException):
    """SIGTERM arrived. BaseException so it unwinds like KeyboardInterrupt does."""


# Cameras whose settings we have changed and not yet put back. Ctrl-C during a
# sweep leaves the camera at whatever exposure was being tested - blind, if that
# was the short end - so the restore must survive an interrupt. It cannot run on
# the interrupted event loop: asyncio is already tearing it down and the await
# fails silently. main() replays this on a FRESH loop instead.
PENDING_RESTORE = {}


async def restore_pending(host, port=5800):
    """Put back anything a crash or Ctrl-C left changed. Safe to call twice.

    Returns [(uniqueName, original, verified)]. The read-back is the point: the
    failure this exists for is a DEAD websocket, and set_setting on a socket that
    is already dying returns without raising - so "we sent it" was never evidence
    the camera took it, and the log said "restored" either way. Only a verified
    entry is forgotten, so a caller that runs again still retries the rest.
    """
    if not PENDING_RESTORE:
        return []
    done = []
    async with Photon(host, port) as pv:
        for unique, original in list(PENDING_RESTORE.items()):
            try:
                await pv.set_setting(unique, **original)
            except Exception:
                continue
            ok = False
            # Read back more than once. PhotonVision's cameraSettings broadcast
            # lags a second or two behind a write, so a single look 0.5 s later
            # reports the PREVIOUS value: measured on a systemctl stop mid-sweep,
            # this logged "!! UNVERIFIED ... check these by hand" while the camera
            # was in fact already back at 1500 us / gain 100. A restore that cries
            # wolf is worse than no check at all.
            for _try in range(3):
                try:
                    await asyncio.sleep(1.5)
                    live = await pv.cameras_fresh(timeout=8)
                    got = next((c for c in live if c["uniqueName"] == unique), None)
                    if got and all(_matches(got["settings"].get(k), v)
                                   for k, v in original.items()):
                        ok = True
                        break
                except Exception:
                    pass
            done.append((unique, dict(original), ok))
            if ok:
                PENDING_RESTORE.pop(unique, None)
    return done


def replay_pending(host, port=5800, log=print):
    """Replay PENDING_RESTORE on a FRESH event loop. Never raises.

    Belongs in a `finally`, not in an interrupt handler. The commonest way to
    leave a camera mid-sweep is not an interrupt at all: a websocket death inside
    tune_camera is caught, records PENDING_RESTORE, logs "will retry on a fresh
    connection" and then returns NORMALLY - so the ordinary exit path ran, nobody
    retried, and the camera sat at whatever exposure was being tested, which can
    be the blind end of the range. Observed in 4 of 8 runs on this rig and in
    both of the daemon's production boot runs.
    """
    if not PENDING_RESTORE:
        return []
    log("restoring %d camera(s) left mid-tune" % len(PENDING_RESTORE))
    try:
        done = asyncio.run(restore_pending(host, port))
    except BaseException as exc:
        log("  !! COULD NOT RESTORE (%s). Check the exposure by hand." % exc)
        return []
    for unique, orig, ok in done:
        log("  %s %s -> exposure %s, gain %s"
            % ("restored" if ok else "!! UNVERIFIED", unique[:8],
               orig.get("cameraExposureRaw"), orig.get("cameraGain")))
    if PENDING_RESTORE:
        log("  !! still not put back: %s - check these by hand"
            % ", ".join(u[:8] for u in PENDING_RESTORE))
    return done


NETCFG_FIELDS = ["ntServerAddress", "connectionType", "staticIp", "hostname",
                 "runNTServer", "shouldManage", "shouldPublishProto",
                 "networkManagerIface", "setStaticCommand", "setDHCPcommand"]


async def read_network_config(host, port=5800):
    """PhotonVision's network settings, off the websocket broadcast."""
    uri = "ws://%s:%d/websocket_data" % (host, port)
    async with websockets.connect(uri, max_size=None, open_timeout=10) as ws:
        for _ in range(60):
            m = msgpack.unpackb(await asyncio.wait_for(ws.recv(), 10), raw=False)
            if not isinstance(m, dict):
                continue
            st = m.get("settings")
            if not isinstance(st, dict):
                continue
            nw = st.get("networkSettings") or st.get("network")
            if nw:
                return {k: nw[k] for k in NETCFG_FIELDS if k in nw}
    return None


async def set_nt_server(host, cfg, enabled, port=5800, log=None, timeout=20.0):
    """Turn PhotonVision's own NT server on or off, live, and CONFIRM it.

    Applied by NetworkTablesManager.setConfig() without restarting PhotonVision.
    Send the WHOLE config with one field changed - the endpoint also calls
    NetworkManager.reinitialize(), and with shouldManage=true a partial body
    would let Jackson default-fill fields and could take the coprocessor off the
    network. Verified: posting the config back unchanged is a clean no-op.

    The POST's own reply proves nothing either way: the HTTP connection drops
    without a response while the network stack bounces, so a failed read is
    expected. That was used as licence to swallow EVERY exception and return, and
    the caller then logged "stopping the NetworkTables server we started"
    unconditionally - while runNTServer has been found left True after runs that
    all claimed to have stopped it. Read the config back instead, and say what it
    actually says. Returns True only if the observed state is the one asked for.

    The wait is polled rather than two hard-coded 4 s sleeps - 8 s of every run
    spent whether or not anything had happened yet.
    """
    import urllib.request, urllib.error
    body = dict(cfg)
    body["runNTServer"] = bool(enabled)
    req = urllib.request.Request(
        "http://%s:%d/api/settings/general" % (host, port),
        data=json.dumps(body).encode(), method="POST",
        headers={"Content-Type": "application/json"})

    def _post():
        try:
            urllib.request.urlopen(req, timeout=15).read()
            return None
        except Exception as exc:
            return exc            # expected: the stack restarts under the reply

    posted = await asyncio.get_event_loop().run_in_executor(None, _post)
    t0 = time.time()
    seen = None
    while time.time() - t0 < timeout:
        await asyncio.sleep(0.5)
        try:
            cur = await asyncio.wait_for(read_network_config(host, port), 8)
        except Exception:
            continue              # websocket is down while the stack bounces
        if not cur:
            continue
        seen = cur.get("runNTServer")
        if bool(seen) == bool(enabled):
            if log:
                log("   PhotonVision's NT server is %s - confirmed after %.1f s"
                    % ("ON" if enabled else "OFF", time.time() - t0))
            return True
    if log:
        log("   !! PhotonVision's NT server did NOT go %s: runNTServer still "
            "reads %r after %.0f s%s"
            % ("on" if enabled else "off", seen, time.time() - t0,
               " (POST raised %s)" % type(posted).__name__ if posted else ""))
    return False


def nt_server_reachable(server, wait=4.0):
    """Is anything serving NetworkTables at `server`?"""
    try:
        import ntcore
    except ImportError:
        return False
    # A PRIVATE instance. This used to call startClient4 on the DEFAULT instance -
    # the same singleton the daemon publishes its status table on - under a
    # different identity, which orphaned the daemon's publishers. Measured: after
    # a clean tune the daemon's own log said ok=True while the NT table a team
    # reads still said "running..." and ok=False, forever. Only `heartbeat`, which
    # is written every loop, kept moving.
    inst = ntcore.NetworkTableInstance.create()
    try:
        inst.startClient4("photontune-probe")
        inst.setServer(server, ntcore.NetworkTableInstance.kDefaultPort4)
        end = time.time() + wait
        while time.time() < end:
            if inst.isConnected():
                return True
            time.sleep(0.2)
    finally:
        try:
            inst.stopClient()
            ntcore.NetworkTableInstance.destroy(inst)
        except Exception:
            pass
    return False


class NTResults:
    """Read PhotonVision's detections off NetworkTables instead of its websocket.

    The websocket is the DASHBOARD feed and is throttled for the UI: measured
    ~9 results/s on a pipeline running 42. A 2.5 s dwell therefore scored each
    sweep point on ~30 frames, and at a 90% pass threshold that makes 32/36 vs
    33/36 - one frame - decide a 2x exposure change. NT carries every frame:
    42.9 results/s measured, ~107 frames per dwell, 4.8x the samples.

    Imports ONLY the decoder. photonlibpy.PhotonCamera pulls in
    photonlibpy/timesync/timeSyncServer.py, which creates a TimeSyncServer at
    module scope (line 94) and binds a UDP port PhotonVision already owns -
    "OSError: [Errno 98] Address already in use" on the coprocessor itself.
    Packet + PhotonPipelineResult start no threads.

    Needs an NT server to exist somewhere. On a robot that is the roboRIO; on a
    bench it is PhotonVision itself when runNTServer is on. If there is none,
    callers fall back to the websocket.
    """

    def __init__(self, server, nickname):
        import ntcore
        from photonlibpy.packet import Packet
        from photonlibpy.targeting.photonPipelineResult import PhotonPipelineResult
        self._Packet = Packet
        self._Result = PhotonPipelineResult
        # Private instance, for the same reason as nt_server_reachable: sampling
        # must not re-point or re-identify the connection the daemon publishes on.
        self.inst = ntcore.NetworkTableInstance.create()
        self._own_inst = True
        self.inst.startClient4("photontune-nt")
        self.inst.setServer(server, ntcore.NetworkTableInstance.kDefaultPort4)
        self._tbl_name = nickname
        tbl = self.inst.getTable("photonvision").getSubTable(nickname)
        self.sub = tbl.getRawTopic("rawBytes").subscribe(
            "rawBytes", b"", ntcore.PubSubOptions(periodic=0.005, sendAll=True,
                                                  keepDuplicates=True))

    def close(self):
        """Release the private NT client. Without this every reader leaks one."""
        try:
            self.inst.stopClient()
            import ntcore as _nt
            _nt.NetworkTableInstance.destroy(self.inst)
        except Exception:
            pass

    def newest_capture(self):
        """captureTimestampMicros of the latest frame, or None."""
        raw = self.sub.get()
        if not raw:
            return None
        try:
            r = self._Result.photonStruct.unpack(self._Packet(raw))
            return r.metadata.captureTimestampMicros
        except Exception:
            return None

    def available(self, wait=4.0):
        """True if results are actually arriving - not merely that NT connected."""
        end = time.time() + wait
        while time.time() < end:
            if self.sub.get():
                return True
            time.sleep(0.1)
        return False

    def collect(self, seconds, ref_tag=None, after_capture=None, min_frames=0,
                extend=SAMPLE_EXTEND_S):
        """after_capture: discard frames CAPTURED at or before this timestamp.

        Waiting a settle period is not enough. The pipeline runs ~125 ms behind,
        so frames arriving just after an exposure change were exposed BEFORE it.
        Sampled without this gate, the first sweep point inherited frames from
        the baseline collect at a much longer exposure and read 0.96 tags where
        the truth was 0.17 - which then tripped the sanity check. Capture
        timestamps are stamped at the sensor, so gating on them is exact.
        """
        out = {"frames": 0, "solves": 0, "reproj": [], "tags": [], "amb": [],
               "ref_seen": 0, "ref_ranges": [], "stale_dropped": 0}
        seen = set()
        end = time.time() + seconds
        # Keep ONE dedupe set across the extension rather than calling collect
        # twice: sub.get() returns the latest value, so a second call would
        # re-count the frame that straddles the boundary.
        hard_end = end + (extend if min_frames else 0.0)
        # ntcore's readQueue() returned nothing here regardless of pollStorage,
        # so poll and dedupe on sequenceID. At 500 Hz nothing is missed at 24 ms.
        while (time.time() < end
               or (0 < out["frames"] < min_frames and time.time() < hard_end)):
            raw = self.sub.get()
            if raw:
                try:
                    r = self._Result.photonStruct.unpack(self._Packet(raw))
                except Exception:
                    r = None
                if r is not None and r.metadata.sequenceID not in seen:
                    seen.add(r.metadata.sequenceID)
                    if (after_capture is not None
                            and r.metadata.captureTimestampMicros <= after_capture):
                        out["stale_dropped"] += 1
                        time.sleep(0.002)
                        continue
                    out["frames"] += 1
                    targets = r.getTargets()
                    out["tags"].append(len(targets))
                    for t in targets:
                        out["amb"].append(getattr(t, "poseAmbiguity", -1))
                        if ref_tag is not None and int(getattr(t, "fiducialId", -1)) == ref_tag:
                            out["ref_seen"] += 1
                            p = getattr(t, "bestCameraToTarget", None)
                            if p is not None:
                                out["ref_ranges"].append(
                                    math.sqrt(p.X() ** 2 + p.Y() ** 2 + p.Z() ** 2))
                    mt = getattr(r, "multitagResult", None)
                    if mt is not None and hasattr(mt, "isPresent"):
                        mt = mt.get() if mt.isPresent() else None
                    if mt is not None and getattr(mt, "estimatedPose", None) is not None:
                        out["solves"] += 1
                        out["reproj"].append(float(mt.estimatedPose.bestReprojErr))
            time.sleep(0.002)
        return out


# ─────────────────── settings photontune asserts ───────────────────
#
# The split matters more than the values. ALWAYS has one right answer and is
# applied without argument. DEFAULT is a measured default for this class of
# hardware which you may legitimately disagree with. Everything else - exposure,
# gain, decisionMargin - is scene-dependent and gets TUNED, not asserted.
#
# Leaving these to a separate tool is what let a camera sit at brightness 67
# while its neighbour sat at 40, producing two tunes that could not be compared.

BASELINE_ALWAYS = {
    "inputImageRotationMode": (0, 0,
        "Non-zero rotation corrupts the multi-tag pose by ~0.4 m while the "
        "published corners stay correct - it fails silently. PhotonVision #2613. "
        "A sideways mount belongs in robotToCamera, not here."),
    "blur": (0.0, 0.0,
        "Gaussian blur BEFORE detection. Any non-zero value softens the tag "
        "edges the detector depends on; at 3.5 detection stops entirely and no "
        "exposure or gain recovers it."),
    "cameraAutoExposure": (False, False,
        "Auto-exposure optimises for the whole scene, not the tags, and drifts. "
        "It also makes tuning meaningless - the camera would fight every step."),
    "cameraRedGain": (0, 0, "Mono sensor; white balance is meaningless."),
    "cameraBlueGain": (0, 0, "Mono sensor; white balance is meaningless."),
    "targetModel": (7, 7,
        "Physical tag size. Sets the SCALE of every distance measured. Wrong "
        "here and all ranges are proportionally wrong while looking perfectly "
        "self-consistent. 6.5 in = 165.1 mm is the FRC standard."),
    "tagFamily": (0, 0, "The family FRC uses."),
}

# The gain a tune STARTS from, unless --gain says otherwise.
#
# ZERO, deliberately. Gain is not a setting with a good value - it is a cost you
# pay to buy a SHORTER exposure, and you should only pay it when the exposure the
# scene wants would smear more than the blur budget allows. So the tune starts at
# the bottom and climbs only when the blur budget forces it. Whatever it lands on
# is then the least noise that meets the budget, which is the actual goal.
#
# Two things this fixes. Starting from the camera's CURRENT gain made the tool
# non-deterministic and, because escalation is one-way, a ratchet: observed on one
# rig, same room and same light, one camera starting at 28 and the other at 100
# purely from run history - and the noisy one reprojected at 1.8-2.1 px against
# the other's 0.5. Running unattended at boot, that walks toward maximum gain over
# a season with nothing reporting it. Starting from a fixed GUESS (this was 25 for
# one revision) is better but still wrong: it silently pays for noise the scene may
# not need, and it cannot be justified from a measurement.
#
# The cost of starting at 0 is an extra sweep in a genuinely dark venue, where the
# first pass finds nothing and escalation raises gain. That path is loop-guarded
# and it is the correct answer arrived at honestly.
BASELINE_START_GAIN = 0

BASELINE_DEFAULT = {
    "decisionMargin": (35, 35,
        "Detection confidence floor - the goal's 'cutoff'. PhotonVision drops any "
        "detection below it before anything else runs, so a human who sets it to "
        "100 makes every tag vanish and the tuner reports a lighting problem. 50 "
        "was measured rejecting a valid tag that 35 recovered; lower admits false "
        "positives, which multi-tag rejects anyway."),
    "hammingDist": (0, 0,
        "Bit errors tolerated when decoding. Above 0 admits misreads, and a "
        "misread tag poisons the multi-tag solve with a confident wrong position."),
    "threads": (1, 1,
        "PER CAMERA. Matching physical cores sounds right and is wrong: the "
        "detector's pool competes with capture, streaming and the JVM. Measured "
        "on a Pi 5 - one camera 14.90 ms at 1 vs 15.58 at 4; two cameras "
        "84.0 fps total at 1 vs 73.7 at 4, and ~20 ms less latency on both."),
    "decimate": (2, 2,
        "Search-stage downsampling. Costs RANGE, not accuracy - corner "
        "refinement always runs at full resolution. Measured: identical "
        "reprojection at 1/2/3/4 but detection range ~18/9/6/4.6 m."),
    "cameraBrightness": (40, 40,
        "Sensor black level, NOT a scene property - which is why it is asserted "
        "rather than tuned. Set low it crushes the image to black and nothing "
        "downstream recovers it; a tuner will just answer with a long, blurry "
        "exposure and report success. Adjust with --brightness if your sensor "
        "differs, but do not leave it unowned."),
    "numIterations": (40, 40,
        "Single-tag pose refinement only - AprilTagPoseEstimatorPipe. "
        "MultiTargetPNPPipe never reads it, so with multi-tag on it does not "
        "touch the pose you actually use. A CPU knob, not an accuracy one."),
    "refineEdges": (True, True,
        "Sub-pixel corner refinement. A real accuracy-vs-CPU tradeoff, not a "
        "constant - but off, corners land on whole pixels and pose precision "
        "collapses. This is also what makes decimate cheap."),
    "doMultiTarget": (True, True,
        "Multi-tag PnP. THE fix for pose flipping - single-tag PnP on a face-on "
        "tag is genuinely ambiguous. Needs a loaded AprilTagFieldLayout."),
    "solvePNPEnabled": (True, True,
        "3D pose output. With no calibration for the ACTIVE resolution this "
        "throws every frame and emits nothing at all - not a 2D fallback."),
}


def _matches(have, expect):
    try:
        return abs(float(have) - float(expect)) < 1e-6
    except (TypeError, ValueError):
        return have == expect


async def assert_baseline(pv, cam, args, log=print):
    """Put the structural settings where they must be, before tuning anything.

    Reports every change and its reason, so 'blindly applied' is visible rather
    than implicit. Returns (changed, failed).
    """
    unique = cam["uniqueName"]
    # send-form vs readback-form. PhotonVision accepts "DEG_0" and reports 0;
    # comparing what we sent against what it reports made three settings look
    # permanently failed, which re-ran the baseline on every gain escalation and
    # corrupted the restore target with post-sweep state.
    want = {}
    expect = {}
    for tbl in (BASELINE_ALWAYS, BASELINE_DEFAULT):
        for k, (v, e, _why) in tbl.items():
            want[k] = v
            expect[k] = e
    if getattr(args, "brightness", None) is not None:
        want["cameraBrightness"] = int(args.brightness)
        expect["cameraBrightness"] = int(args.brightness)

    live = cam.get("settings", {})
    todo = {}
    for k, v in want.items():
        if _matches(live.get(k), expect[k]):
            continue
        todo[k] = v
    if not todo:
        log("   baseline: already correct")
        return [], []

    for k, v in todo.items():
        why = (BASELINE_ALWAYS.get(k) or BASELINE_DEFAULT.get(k))[2]
        log("   baseline: %s %s -> %s" % (k, live.get(k), v))
        log("             %s" % why.split(". ")[0] + ".")
    await pv.set_setting(unique, **todo)
    await asyncio.sleep(max(args.settle, 1.5))

    after = await pv.cameras_fresh(timeout=8)
    got = next((c for c in after if c["uniqueName"] == unique), None)
    failed = []
    if got is None:
        log("   baseline: could not read back to confirm")
    else:
        for k, v in todo.items():
            have = got["settings"].get(k)
            if not _matches(have, expect[k]):
                failed.append((k, expect[k], have))
                log("   !! baseline %s did NOT take (wanted %s, camera has %s)" % (k, v, have))
    return list(todo), failed


def robot_is_enabled(inst):
    """True if the FMS/DS says enabled. Never retune during a match."""
    try:
        tbl = inst.getTable("FMSInfo")
        control = tbl.getEntry("FMSControlData").getInteger(0)
        return bool(control & 0x01)          # bit 0 = enabled
    except Exception:
        return False


async def daemon(args, log=print):
    try:
        import ntcore
    except ImportError:
        sys.exit("daemon mode needs: pip install pyntcore")

    inst = ntcore.NetworkTableInstance.getDefault()
    inst.startClient4("photontune")
    if args.nt_server:
        inst.setServer(args.nt_server, ntcore.NetworkTableInstance.kDefaultPort4)
    else:
        inst.setServerTeam(args.team)

    tbl = inst.getTable(args.nt_table)
    run_entry = tbl.getEntry("run")
    run_entry.setDefaultBoolean(False)
    status = tbl.getEntry("status")          # live human-readable line
    busy = tbl.getEntry("busy")              # true while sweeping
    ok_entry = tbl.getEntry("ok")            # <- go / no-go for the last run
    summary = tbl.getEntry("summary")        # <- one line, what it did
    warnings_e = tbl.getEntry("warnings")    # <- things that are not failures but
                                             #    that a human must still be told
    progress_e = tbl.getEntry("progress")    # <- 0..1, for a progress bar
    heartbeat = tbl.getEntry("heartbeat")    # <- proves the service is alive
    result_entry = tbl.getEntry("result")    # full JSON
    # Held-card mode, settable from the dashboard before pressing run.
    ref_tag_e = tbl.getEntry("referenceTag")      # tag ID, -1 = off (use field tags)
    ref_range_e = tbl.getEntry("referenceRange")  # string length, metres
    hold_e = tbl.getEntry("holdCard")             # <- true while you must hold it steady
    hold_for_e = tbl.getEntry("holdFor")          # <- WHICH camera to hold it in front of
    camera_e = tbl.getEntry("camera")             # <- camera being tuned, "2 of 3"
    # Automatic tune shortly after boot, so nobody has to remember to press run.
    boot_ran = tbl.getEntry("bootTuneRan")        # <- did the automatic run happen at all
    boot_ok = tbl.getEntry("bootTuneOk")          # <- did it succeed
    boot_sum = tbl.getEntry("bootTuneSummary")    # <- what it did, or why it did not
    ref_tag_e.setDefaultDouble(-1 if args.reference_tag is None else args.reference_tag)
    ref_range_e.setDefaultDouble(args.reference_range or 0.0)
    hold_e.setBoolean(False)
    status.setString("idle")
    summary.setString("never run")
    warnings_e.setString("")
    hold_e.setBoolean(False)
    busy.setBoolean(False)
    ok_entry.setBoolean(False)
    progress_e.setDouble(0.0)
    boot_ran.setBoolean(False)
    boot_ok.setBoolean(False)
    boot_sum.setString("pending" if args.autorun else "disabled")

    # Fire one tune after PhotonVision is actually answering - a fixed sleep from
    # service start is not enough, the pipeline comes up well after the process does.
    auto = {"fire": False, "done": not args.autorun}
    async def _autorun():
        t0 = time.time()
        ready = False
        while time.time() - t0 < args.autorun_timeout:
            try:
                async with Photon(args.host, args.port) as probe:
                    if await probe.cameras():
                        ready = True
                        break
            except Exception:
                pass
            await asyncio.sleep(2.0)
        if not ready:
            auto["done"] = True
            why = "PhotonVision not ready within %.0fs" % args.autorun_timeout
            boot_ran.setBoolean(False); boot_ok.setBoolean(False); boot_sum.setString(why)
            log("autorun skipped: " + why)
            return
        log("autorun: PhotonVision up, tuning in %.0fs" % args.autorun_delay)
        await asyncio.sleep(args.autorun_delay)
        auto["fire"] = True
    if args.autorun:
        asyncio.ensure_future(_autorun())

    log("daemon up. table /%s  - set 'run' true to tune." % args.nt_table)
    last = False
    beat = 0.0
    while True:
        beat += 1.0
        heartbeat.setDouble(beat)
        trigger = run_entry.getBoolean(False)
        boot_fire = auto["fire"] and not auto["done"]
        if boot_fire:
            auto["fire"] = False
            auto["done"] = True
            trigger = True
        if trigger and (not last or boot_fire):
            if robot_is_enabled(inst):
                status.setString("refused: robot enabled")
                log("trigger ignored - robot is enabled")
                run_entry.setBoolean(False)
                if boot_fire:
                    boot_ran.setBoolean(False); boot_ok.setBoolean(False)
                    boot_sum.setString("skipped: robot was enabled at boot")
            else:
                # Read held-card settings written by the dashboard.
                nt_tag = int(ref_tag_e.getDouble(-1))
                nt_range = ref_range_e.getDouble(0.0)
                args.reference_tag = nt_tag if nt_tag >= 0 else None
                args.reference_range = nt_range if nt_range > 0 else None

                run_ok = {"value": False, "summary": ""}
                busy.setBoolean(True)
                warnings_e.setString("")
                ok_entry.setBoolean(False)
                progress_e.setDouble(0.0)
                if args.reference_tag is not None:
                    hold_e.setBoolean(True)
                    msg = ("HOLD tag %d steady at %.2f m" % (args.reference_tag, args.reference_range)
                           if args.reference_range else
                           "HOLD tag %d steady" % args.reference_tag)
                    status.setString(msg)
                    summary.setString(msg)
                    log(msg)
                else:
                    status.setString("tuning on field tags...")
                    summary.setString("running...")
                lines = []
                def cap(msg):
                    lines.append(str(msg)); log(msg); status.setString(str(msg)[:120])
                def announce(nick, idx, total):
                    camera_e.setString("%s (%d of %d)" % (nick, idx + 1, total))
                    if args.reference_tag is not None:
                        hold_for_e.setString(nick)
                        note = "HOLD tag %d in front of %s" % (args.reference_tag, nick)
                        if args.reference_range:
                            note += " at %.2f m" % args.reference_range
                        status.setString(note); summary.setString(note); cap(note)

                try:
                    res = await run(args, log=cap, progress=lambda f: progress_e.setDouble(f),
                                    on_camera=announce)
                    failed, warns, code = run_verdict(res, args)
                    applied = [r for r in res if r not in failed and r.get("applied")]
                    line = "; ".join(
                        "%s=%.0f%s" % (r["camera"], r["applied"], " (fallback)" if r.get("fellback") else "")
                        for r in applied)
                    if failed:
                        line += ("; FAILED: " + ", ".join(
                            "%s (%s)" % (r["camera"], r.get("error", "?")) for r in failed))
                    for _k, _s, _t in problem_notes(run_problems(args), HARD):
                        line += "; FAILED: %s" % _t
                    # Warnings reach the dashboard, not just the log. A camera
                    # whose RESOLUTION this tool changed, or one applied over the
                    # blur budget, published ok=true and said nothing - so the
                    # one signal a team reads could not tell them their camera is
                    # no longer running the mode they set.
                    wline = " | ".join("%s: %s" % (who, t) for who, t in warns)
                    warnings_e.setString(wline[:500])
                    if wline:
                        line += "  [WARN] " + wline
                        log("WARNINGS: " + wline)
                    every_camera_ok = bool(applied) and code == 0
                    run_ok["value"] = every_camera_ok
                    run_ok["summary"] = line or "no cameras tuned"
                    ok_entry.setBoolean(every_camera_ok)
                    summary.setString(line or "no cameras tuned")
                    status.setString(("DONE - " if every_camera_ok else "DONE WITH ERRORS - ") + line)
                    result_entry.setString(json.dumps(res))
                    progress_e.setDouble(1.0)
                    log("done: " + line)
                except AlreadyRunning as exc:
                    # SKIP the trigger, do not fail it. A human at the CLI while
                    # the daemon's autorun fires is the ordinary case, and the
                    # human's run is the one that should win - they are standing
                    # there watching it. A failed boot tune would also latch
                    # bootTuneOk=false for the rest of the session.
                    run_ok["summary"] = "skipped: %s" % str(exc).splitlines()[0]
                    summary.setString(run_ok["summary"])
                    status.setString(run_ok["summary"])
                    log(run_ok["summary"])
                except Exception as exc:
                    run_ok["value"] = False
                    run_ok["summary"] = "error: %s" % exc
                    ok_entry.setBoolean(False)
                    summary.setString("error: %s" % exc)
                    status.setString("error: %s" % exc)
                    log("error: %s" % exc)
                finally:
                    # The daemon is the DEPLOYED mode, and the replay used to sit
                    # in the `except` branch only - which is the branch a dropped
                    # websocket does NOT take, because tune_camera swallows it and
                    # returns normally. Every exit path, or it is not a restore.
                    try:
                        for unique, orig, ok in await restore_pending(args.host, args.port):
                            log("%s %s to %s"
                                % ("restored" if ok else "UNVERIFIED restore of",
                                   unique, orig))
                    except Exception as rexc:
                        log("restore replay failed: %s" % rexc)
                    busy.setBoolean(False)
                    hold_e.setBoolean(False)
                    hold_for_e.setString("")
                    camera_e.setString("")
                    run_entry.setBoolean(False)
                    if boot_fire:
                        boot_ran.setBoolean(True)
                        # Publish what we COMPUTED. Reading it back through NT
                        # here returned the default False, because the run has
                        # already stopped PhotonVision's NT server by this point
                        # and this client has disconnected - so a tune where every
                        # camera succeeded still reported ok=False at boot, which
                        # is the one signal a team would actually trust.
                        boot_ok.setBoolean(bool(run_ok["value"]))
                        boot_sum.setString(str(run_ok["summary"])[:200])
                        log("autorun done: ok=%s %s"
                            % (run_ok["value"], run_ok["summary"]))
        last = trigger
        await asyncio.sleep(0.2)


# ───────────────────────── A: CLI ─────────────────────────

def build_parser():
    p = argparse.ArgumentParser(
        description="Tune PhotonVision exposure by detection quality, biased short to limit motion blur.")
    p.add_argument("--host", default="photonvision.local", help="PhotonVision host")
    p.add_argument("--port", type=int, default=5800)
    p.add_argument("--cameras", default=None,
                   help="comma-separated nicknames to tune (default: all)")
    p.add_argument("--min-exposure", type=float, default=1000.0)
    p.add_argument("--max-exposure", type=float, default=25000.0)
    p.add_argument("--steps", type=int, default=8)
    p.add_argument("--bias", type=float, default=1.5,
                   help="safety factor above the shortest passing exposure (default 1.5)")
    p.add_argument("--gain", type=float, default=None,
                   help="force this exact starting gain, overriding the baseline")
    p.add_argument("--start-gain", type=float, default=BASELINE_START_GAIN,
                   help="gain every tune STARTS from (default %(default)g). A tune must "
                        "not start from the previous tune's answer: escalation is one-way, "
                        "so that ratchets gain upward a little more on every run.")
    p.add_argument("--gain-steps", type=int, default=3,
                   help="how many times to escalate gain if no exposure passes (0 = never)")
    p.add_argument("--max-gain", type=float, default=100.0,
                   help="ceiling for gain escalation")
    p.add_argument("--no-optimise-gain", "--no-optimize-gain", dest="optimise_gain",
                   action="store_false", default=True,
                   help="skip the gain search and sweep exposure at a single gain. "
                        "Faster, but gain is venue-dependent: measured on an OV9281 "
                        "in a dim room, reprojection improved monotonically from "
                        "1.231 px at gain 0 to 0.496 px at gain 100, all at the same "
                        "1500 us exposure. A constant cannot be right in both a dim "
                        "workshop and a lit field.")
    p.add_argument("--optimise-gain", "--optimize-gain", dest="optimise_gain",
                   action="store_true",
                   help="search gain AND exposure together for the shortest exposure that "
                        "still sees all the tags (slower, but exposure is the costly one)")
    p.add_argument("--gain-search-steps", type=int, default=6,
                   help="how many gain values to try with --optimise-gain")
    p.add_argument("--reproj-tolerance", type=float, default=1.5,
                   help="reject a gain whose reprojection exceeds this multiple of the "
                        "best seen - stops noise being traded for exposure indefinitely")
    p.add_argument("--fast", action="store_true",
                   help="shorter settle and dwell. Over NetworkTables the default 2.5 s "
                        "dwell is ~100 results per candidate, which is far more than is "
                        "needed to tell a pass from a blank.")
    p.add_argument("--dwell", type=float, default=1.5,
                   help="seconds of data per candidate (default %(default)s). Over "
                        "NetworkTables that is ~60 results, well past what is needed "
                        "to separate a pass from a failure.")
    p.add_argument("--settle", type=float, default=0.7,
                   help="seconds to wait after changing a setting (default %(default)s). "
                        "MEASURED on a Pi 5 / OV9281: the pipeline reflects a new "
                        "exposure in 0.25-0.36 s, worst case 0.36 s over six trials, "
                        "timed on the coprocessor against PhotonVision's own capture "
                        "timestamps. This is that worst case roughly doubled. The old "
                        "1.5 s default was 4x the measurement and was multiplied by "
                        "every step of every sweep.")
    p.add_argument("--min-tags", type=float, default=2.0,
                   help="mean tags per frame required when multi-tag is unavailable")
    p.add_argument("--max-ambiguity", type=float, default=0.20)
    p.add_argument("--tag-fraction", type=float, default=0.85,
                   help="require at least this fraction of the tags the best exposure "
                        "in the sweep saw (default 0.85); 0 disables")
    p.add_argument("--fx", type=float, default=1105.9, help="focal length in px, for the blur estimate")
    p.add_argument("--reference-tag", type=int, default=None,
                   help="tag ID held in front of the camera; score on its detection rate")
    p.add_argument("--reference-bias", type=float, default=1.6,
                   help="extra margin in held-card mode; a single tag is detected at "
                        "shorter exposures than a multi-tag solve needs (default 1.6)")
    p.add_argument("--move-pause", type=float, default=8.0,
                   help="seconds to move the held card between cameras (held-card mode)")
    p.add_argument("--reference-range", type=float, default=None,
                   help="string length in metres - the tool verifies the tag really is there")
    p.add_argument("--calibrate-reference-bias", action="store_true",
                   help="measure YOUR held-card vs field-tag ratio (needs --reference-tag "
                        "and representative field tags in view). Changes no settings.")
    p.add_argument("--dry-run", action="store_true", help="measure and report, change nothing")
    p.add_argument("--json", action="store_true", help="emit JSON results")
    p.add_argument("--daemon", action="store_true", help="NT-triggered service mode")
    p.add_argument("--nt-server", default=None, help="NT server host (daemon mode)")
    p.add_argument("--team", type=int, default=0, help="team number for NT (daemon mode)")
    p.add_argument("--nt-table", default="PhotonTune")
    p.add_argument("--max-blur-px", type=float, default=10.0,
                   help="blur budget in pixels at --blur-rate. If the chosen exposure "
                        "exceeds it, raise gain and re-sweep rather than accept a "
                        "blurry answer. JUDGEMENT, not a measurement - blurtest.py "
                        "can measure the real tolerance on your rig.")
    p.add_argument("--blur-rate", type=float, default=360.0,
                   help="angular rate the blur budget is judged at, deg/s. 360 is what "
                        "robots actually do while aiming (CTRE default 270, REV 360, "
                        "Limelight rejects vision above 360); 900 is never-exceed.")
    p.add_argument("--no-baseline", dest="baseline", action="store_false", default=True,
                   help="do NOT assert the structural settings first. They are "
                        "asserted by default: brightness, rotation, blur and the tag "
                        "model have a right answer, and leaving them unowned is what "
                        "lets a bad brightness silently ruin a tune.")
    p.add_argument("--brightness", type=int, default=None,
                   help="override the asserted cameraBrightness (default 40)")
    p.add_argument("--baseline-only", action="store_true",
                   help="assert the structural settings and stop - do not tune")
    p.add_argument("--no-nt", action="store_true",
                   help="always sample from the websocket, never NetworkTables")
    p.add_argument("--no-manage-nt-server", dest="manage_nt_server",
                   action="store_false", default=True,
                   help="do NOT start PhotonVision's NT server when none is found. "
                        "By default photontune starts it, tunes, and stops it again, "
                        "so a stray server is never left to fight the roboRIO.")
    p.add_argument("--autorun", action="store_true",
                   help="daemon: tune once automatically after PhotonVision comes up")
    p.add_argument("--autorun-delay", type=float, default=5.0,
                   help="seconds to wait after PhotonVision answers, before the automatic tune")
    p.add_argument("--autorun-timeout", type=float, default=90.0,
                   help="give up waiting for PhotonVision after this many seconds")
    return p


def _install_signal_handlers():
    """Turn SIGTERM into an exception so `finally` blocks actually run.

    systemd stops a unit with SIGTERM. Python's default handler exits the
    interpreter immediately - no `finally`, no restore - so under the deployed
    systemd configuration the camera was left at whatever exposure the sweep was
    testing, which can be the blind end of the range. Raising instead lets the
    same unwinding that Ctrl-C already used do its job.
    """
    import signal

    def _raise(signum, _frame):
        raise Terminated("signal %d" % signum)

    for sig in (signal.SIGTERM, signal.SIGHUP):
        try:
            signal.signal(sig, _raise)
        except (ValueError, OSError, AttributeError):
            pass          # not the main thread, or not supported here


async def _guard_signals(coro):
    """Run `coro` with SIGTERM/SIGHUP cancelling it ON the event loop.

    signal.signal() is kept as the fallback for the window before the loop
    exists and for the restore afterwards, but it is not the mechanism: its
    handler raises from whatever bytecode the main thread happens to be running,
    which can be inside websockets' own cleanup or inside a `finally`.
    loop.add_signal_handler is the documented asyncio path - the callback runs
    between loop callbacks and cancels the task at an await point, so
    tune_camera's handler gets a LIVE loop to restore on rather than one that is
    already unwinding.

    MEASURED, because the bug report for this was wrong and the wrong diagnosis
    is worth recording. signal.signal was not broken: kill -TERM to the PYTHON
    process restored the camera 14 times out of 14 on this rig (8 different kill
    positions through a sweep, 4 on the --optimise-gain path, 2 repeats), on both
    this revision and dc49bcd. The reported "exit 143, no INTERRUPTED line,
    camera left at 6292 us" is a different failure, and it reproduces on demand:
    if photontune is launched under a shell wrapper - `sh -c "python3
    photontune.py ..." &` - and the WRAPPER is killed, the wrapper exits 143
    (default disposition, no Python involved at all), python is never signalled,
    survives as an orphan, and abandons the sweep mid-flight. Verified: wrapper
    EXIT=143, `pgrep` still listing the python pid afterwards, log ending at
    "exposure 6292". Nothing inside the process can fix that; kill the python
    process, or use `systemctl stop`, which does.
    The missing INTERRUPTED line has its own separate explanation: on the
    --optimise-gain path tune_camera is called with log=lambda m: None, so that
    line is suppressed even when the restore works perfectly.
    """
    import signal
    loop = asyncio.get_event_loop()
    task = asyncio.ensure_future(coro)
    hit = {}

    def _cancel(signum):
        hit["sig"] = signum
        task.cancel()

    installed = []
    for sig in (signal.SIGTERM, signal.SIGHUP):
        try:
            loop.add_signal_handler(sig, _cancel, sig)
            installed.append(sig)
        except (NotImplementedError, RuntimeError, ValueError, AttributeError, OSError):
            pass                  # Windows, or not the main thread
    try:
        return await task
    except asyncio.CancelledError:
        if hit:
            raise Terminated("signal %d" % hit["sig"]) from None
        raise
    finally:
        for sig in installed:
            try:
                loop.remove_signal_handler(sig)
            except Exception:
                pass
        # remove_signal_handler puts the disposition back to SIG_DFL, not back to
        # ours - so without this the restore that runs AFTER the loop would die
        # on a second SIGTERM with no cleanup at all.
        _install_signal_handlers()


def main():
    _install_signal_handlers()
    args = build_parser().parse_args()
    if getattr(args, "fast", False):
        args.dwell = min(args.dwell, 0.9)
        args.settle = min(args.settle, 0.5)
    if args.min_exposure <= 0 or args.max_exposure <= args.min_exposure:
        sys.exit("--max-exposure must exceed --min-exposure, both > 0")
    if args.calibrate_reference_bias:
        try:
            asyncio.run(calibrate_reference_bias(args))
        except (AlreadyRunning, ConnectionError, LookupError) as exc:
            sys.exit("photontune: %s" % exc)
        return
    if args.daemon:
        try:
            try:
                asyncio.run(_guard_signals(daemon(args)))
            except (KeyboardInterrupt, Terminated):
                # systemd stops the daemon with SIGTERM, and the daemon is the
                # DEPLOYED mode. Without this the signal unwound to the top with a
                # traceback and, worse, any camera left mid-sweep stayed there.
                print("photontune: stopping - restoring any camera left mid-tune")
                # and never leave PhotonVision's NT server running behind us
                try:
                    if getattr(args, "_nt_we_started", False):
                        cfg = asyncio.run(read_network_config(args.host, args.port))
                        if cfg:
                            ok = asyncio.run(set_nt_server(args.host, cfg, False,
                                                           args.port, print))
                            print("  turned PhotonVision's NT server back off" if ok
                                  else "  !! PhotonVision's NT server is STILL ON")
                except Exception:
                    pass
                sys.exit(0)
        finally:
            replay_pending(args.host, args.port)
        return
    t0 = time.time()
    # The `finally` is the whole fix. An interrupt was never the common case -
    # a websocket death is caught inside tune_camera and returns NORMALLY, so
    # the restore has to hang off the exit, not off an exception.
    try:
        try:
            results = asyncio.run(_guard_signals(run(args)))
        except (AlreadyRunning, ConnectionError, LookupError) as exc:
            sys.exit("photontune: %s" % exc)
        except (KeyboardInterrupt, Terminated):
            # The interrupted loop could not complete the restore. Do it on a new one.
            print("\ninterrupted - putting camera settings back...")
            sys.exit(130)
        # --baseline-only never sets `applied` by design, so "no applied" cannot
        # mean failure in that mode. A baseline that did NOT take, or a rejected
        # setting, MUST mean failure in every mode - previously both were recorded
        # and never consulted, so the tool reported success with a 90-degree
        # rotated image and a gain that never applied.
        failed, warnings, code = run_verdict(results, args)
        if args.json:
            print(json.dumps(results, indent=2))
        else:
            print("\ncompleted in %.0f s" % (time.time() - t0))
            for r in results:
                if r.get("applied"):
                    print("  %-14s exposure -> %.0f, gain %s%s"
                          % (r["camera"], r["applied"], r.get("gain"),
                             "  (fallback)" if r.get("fellback") else ""))
                elif r.get("would_apply"):
                    print("  %-14s would set exposure -> %.0f, gain %s  (dry run)"
                          % (r["camera"], r["would_apply"],
                             r.get("would_gain", r.get("gain"))))
                else:
                    print("  %-14s unchanged (%s)" % (r["camera"], r.get("error", "dry run")))
            # Warnings are NOT failures and are NOT footnotes. A camera left on a
            # resolution nobody chose, or applied with 17.9 px of blur against a
            # 1 px budget, both used to exit 0 with nothing in the summary at all.
            if warnings:
                print("")
                print("WARNINGS - the tune stands, but these need a human to know:")
                for who, text in warnings:
                    print("  %-14s %s" % (who, text))
        for r in failed:
            notes = [t for _k, _s, t in problem_notes(r, HARD)]
            # The recorded problem first, not `error`: on --baseline-only the
            # error field says the benign "baseline only - not tuned" while the
            # actual failure is the read-back that could not confirm.
            why = "; ".join(notes) or r.get("error") or "did not apply anything"
            print("FAILED %s: %s" % (r.get("camera"), why))
            for text in notes:
                if text not in why:
                    print("       %s" % text)
        for _k, _sev, text in problem_notes(run_problems(args), HARD):
            print("FAILED run: %s" % text)
        sys.exit(code)
    finally:
        # EVERY exit path, including the ordinary one and sys.exit above.
        replay_pending(args.host, args.port)


if __name__ == "__main__":
    main()
