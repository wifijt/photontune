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
photontune - set PhotonVision AprilTag exposure and gain from a blur budget.

THE LEAST GAIN THAT STILL SEES EVERY TAG, AT THE EXPOSURE A SPINNING ROBOT
ALLOWS.

Exposure is not searched for. This tool measures a STATIONARY camera, so
motion blur is zero in everything it can observe, and every plateau it can
find is a plateau in the one condition that does not matter. On a moving robot
blur is blur_px = omega * t * fx, so the exposure follows from a budget:

    t_budget = max_blur_px / (radians(blur_rate) * fx)

At 6 px, 360 deg/s and fx 1105.9 that is 864 us. Gain is then the only free
variable, and the answer is the least of it that still sees every tag the
scene can offer - measured by walking a six-point grid upward and stopping at
the first point that passes, plus one grid step of margin.

Reprojection is REPORTED and never used to choose. It is OpenCV's RMS residual
over the corners of the multi-tag fit: it measures internal consistency of a
fit, not pose accuracy, and it IMPROVES when distant tags drop out of the
solve. Optimising it rewards losing tags, which is why a tag-count guard had
to be bolted on to stop it, and why six identical runs on one camera chose
gain 60, 80, 60, 80, 100, 100.

Modes:
  CLI     python photontune.py --host photonvision.local
  Daemon  python photontune.py --daemon --nt-server 10.TE.AM.2
          (asserts the baseline at boot; tunes only when told to)
"""
import argparse, asyncio, dataclasses, json, math, sys, time

try:
    import msgpack, websockets
except ImportError:
    sys.exit("need: pip install msgpack websockets")


# ───────────────────────── shutdown ─────────────────────────

class Terminated(BaseException):
    """SIGTERM arrived. BaseException so it unwinds like KeyboardInterrupt does."""


# Set the instant a SIGTERM or SIGHUP is SEEN, by both the plain signal handler
# and the event loop's, and never cleared.
#
# The cancellation is still the mechanism; this is the belt to its braces, and
# it exists because of a failure that was observed and NOT explained. One
# SIGTERM in a sweep of kill positions through a live two-camera tune did not
# interrupt anything: the process ran the remaining 20 s to completion, printed
# "completed in 42 s" and exited 0. Nine repeats at the same and later
# positions all interrupted correctly, so it is a race, not a dead path.
#
# The plausible mechanism is asyncio.wait_for: when its timeout fires it
# cancels the inner future and converts THAT cancellation into TimeoutError, so
# an outer task cancellation landing in the same instant can come out as a
# timeout - and Photon._pump's `except asyncio.TimeoutError: return` then
# swallows it and the tune carries on. Every sample runs a _pump with a 0.2 s
# floor on its timeout, so there are hundreds of chances per run to hit it.
#
# Rather than chase one instance through the library, every loop that can run
# for a while asks this question directly. A flag cannot be raced away.
SHUTDOWN = {}


def note_shutdown(signum):
    SHUTDOWN.setdefault("sig", int(signum))


def raise_if_shutdown(where=""):
    """Abort promptly if a signal has been seen, whatever became of the cancel."""
    sig = SHUTDOWN.get("sig")
    if sig:
        raise Terminated("signal %d%s" % (sig, (" during %s" % where) if where else ""))


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

# Exposure bounds to use when a camera does not report its own. Only a
# fallback: every camera on this rig reports 7-80000 us and those numbers are
# what get used. A camera that reports nothing is rare enough that guessing a
# wide, safe range beats refusing to tune it.
EXPOSURE_FLOOR_FALLBACK = 100.0
EXPOSURE_CEILING_FALLBACK = 25000.0

# How long a read-back may poll for the camera to agree before it is called a
# failure. PhotonVision's cameraSettings broadcast lags a write by 1-2 s, so
# anything under a few seconds cries wolf; 12 s is long enough that "still not
# agreeing" means it never will.
CONFIRM_TIMEOUT_S = 12.0

# How many standard errors of a candidate's OWN tag count it may fall below the
# reference count before we call it a real loss of tags. See
# Sample.tags_significantly_below for the measured table this is set from.
TAG_DROP_Z = 3.0

# The gain grid. Six points, evenly spaced from 0 to --max-gain, which is the
# grid the deleted gain scan used and the grid every gain measurement in this
# file was taken on. The walk stops at the first point that passes and then
# adds ONE more step as margin, so the grid spacing IS the margin: at the
# default --max-gain 100 that is 20.
GAIN_GRID_STEPS = 6

# How many points the exposure walks may try before giving up. Both walks are
# exceptional paths - the scene is saturated, or too dark to work inside the
# blur budget - and both must stay bounded, because an unbounded walk is how
# the old escalation turned a 25 s tune into a two-minute one.
EXPOSURE_WALK_STEPS = 5


class Sample:
    """Detection quality at one (gain, exposure), for one camera."""
    def __init__(self, exposure, gain=None):
        self.exposure = exposure
        self.gain = gain
        self.frames = 0
        self.multitag_solves = 0
        self.reproj = []
        self.tag_counts = []
        self.ambiguities = []

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

    def tag_standard_error(self):
        """Sampling error of this sample's own mean tag count, or None."""
        n = len(self.tag_counts)
        if n < 2:
            return None
        m = self.mean_tags
        var = sum((t - m) ** 2 for t in self.tag_counts) / float(n - 1)
        return math.sqrt(var / n)

    def tags_significantly_below(self, reference, z=TAG_DROP_Z):
        """Is this sample missing tags the reference found, beyond noise?

        A FRACTION cannot do this job, and trying one failed twice in opposite
        directions on the same rig:

          - at 0.85, a reference of 4.00 tags sets the floor at 3.40, so 3.50,
            3.73 and 3.80 all pass. Live runs marked "tags 3.73" and "tags
            3.81" as ok. That is the case the tag guard was written for, and it
            never caught it.
          - at 0.96, a reference of 2.00 tags sets the floor at 1.92 - and a
            re-measurement reading exactly 1.92 tags, one frame in 25 short,
            disqualified the winning gain and changed the answer.

        The scale-free question is whether the shortfall is bigger than the
        SAMPLING ERROR of this sample's own tag count, which we can measure
        from the per-frame counts we already have. At the 30-60 frames a dwell
        carries, 3 sigma separates the two cases cleanly on every real example
        from this rig:

            ref    this   short   3 sigma   verdict
            4.00   3.50    0.50     0.21    reject - half a tag gone
            4.00   3.73    0.27     0.19    reject
            4.00   3.80    0.20     0.17    reject
            4.00   3.93    0.07     0.11    keep   - 7% of frames, noise
            2.00   1.92    0.08     0.12    keep
            2.00   1.52    0.48     0.21    reject - a real dropout

        It tightens with more evidence rather than loosening, which is the same
        behaviour rate_upper_bound() has and for the same reason.
        """
        if reference is None or len(self.tag_counts) < 2:
            return False
        short = reference - self.mean_tags
        if short <= 0:
            return False
        se = self.tag_standard_error()
        if not se:
            # Every frame is short by the same amount. That is not sampling
            # error, it is the tag being gone.
            return True
        return short > z * se

    def passes(self, min_tags, max_ambiguity, tag_reference=None,
               tag_z=TAG_DROP_Z):
        if self.frames < MIN_SAMPLE_FRAMES:
            return False
        # Multi-tag solves happily on a subset, so "it solved" is not the same
        # as "it saw everything available". A gain that loses tags buys nothing
        # at all - the exposure is fixed, so there is no blur being traded for
        # them - which is why this is a significance test and not a fraction.
        if self.tags_significantly_below(tag_reference, tag_z):
            return False
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

    def why_failed(self, min_tags, max_ambiguity, tag_reference=None,
                   tag_z=TAG_DROP_Z):
        """One short phrase saying why this sample did not pass, or None.

        The gain search reported EVERY outcome as "no passing exposure": a dead
        connection, a calibration refusal, --baseline-only and a genuinely dark
        room were indistinguishable, and six "sweeps" were seen completing in
        0.00 s total with nothing noticing.
        """
        if self.passes(min_tags, max_ambiguity, tag_reference, tag_z):
            return None
        if self.frames == 0:
            return "no frames arrived - nothing was being published"
        if self.frames < MIN_SAMPLE_FRAMES:
            return ("only %d frames - under the %d needed to classify (at %d "
                    "frames the 90%% gate lets %.0f%% through, so a 'pass' here "
                    "would be noise). Raise --dwell."
                    % (self.frames, MIN_SAMPLE_FRAMES, self.frames,
                       100 * _effective_floor(self.frames)))
        if self.tags_significantly_below(tag_reference, tag_z):
            return ("saw %.2f tags against the reference's %.2f - %.2f short, "
                    "over the %.2f that %g sigma of this sample's own scatter "
                    "allows, so it is a real loss and not noise"
                    % (self.mean_tags, tag_reference,
                       tag_reference - self.mean_tags,
                       tag_z * (self.tag_standard_error() or 0.0), tag_z))
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

    def summary(self):
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


def _median(xs):
    xs = sorted(xs)
    if not xs:
        return None
    n = len(xs)
    return xs[n // 2] if n % 2 else 0.5 * (xs[n // 2 - 1] + xs[n // 2])


# ───────────────────────── the blur budget ─────────────────────────
#
# This is the one thing in the tool that knows about the robot rather than
# about the bench, and it is now what SETS the exposure rather than what
# comments on it.
#
# A camera rotating at omega smears a point across blur_px = omega * t * fx.
# Turn that around and the exposure is not a thing to search for at all: it is
# whatever the blur budget allows, and the only free variable left is gain.

def blur_px(exposure_us, blur_rate_deg_s, fx):
    """Motion smear, in pixels, at this exposure and angular rate."""
    return math.radians(blur_rate_deg_s) * (exposure_us / 1e6) * fx


def blur_budget_exposure(max_blur_px, blur_rate_deg_s, fx):
    """The longest exposure that stays inside the budget, in microseconds."""
    return 1e6 * max_blur_px / (math.radians(blur_rate_deg_s) * fx)


def geometric_sweep(lo, hi, steps):
    if steps < 2:
        return [lo]
    r = (hi / lo) ** (1.0 / (steps - 1))
    return [lo * (r ** i) for i in range(steps)]


def gain_with_margin(picked, margin_step, max_gain):
    """One grid step above the lowest gain that works, capped at the ceiling.

    A field is not the pit. The light changes, the tags get further away, and
    the cost of one step is sensor noise on a solve that is already passing -
    while the cost of being one step short is a camera that stops seeing tags
    in the middle of a match.
    """
    return int(min(float(max_gain), picked + margin_step))


def gain_grid(max_gain, steps=GAIN_GRID_STEPS):
    """The gains the walk visits, lowest first, and the margin step.

    Returns (gains, step). Integers, deduplicated, so the walk cannot visit the
    same operating point twice and read two different answers for it.
    """
    hi = float(max_gain)
    n = max(2, int(steps))
    if hi <= 0:
        return [0], 0
    step = hi / float(n - 1)
    gains = sorted({int(round(step * i)) for i in range(n)})
    return gains, int(round(step))


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
            # Asked here rather than trusted to the cancellation: see SHUTDOWN.
            # The restore paths do not go through _pump, so aborting here
            # cannot stop the camera being put back.
            raise_if_shutdown("sampling")
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

    @classmethod
    def _diff(cls, live, expect):
        """[(key, wanted, have)] for every expectation the camera disagrees with."""
        bad = []
        for k, want in expect.items():
            if k in cls.INT_SETTINGS and isinstance(want, float):
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

    async def collect(self, unique_name, seconds, min_frames=0,
                      extend=SAMPLE_EXTEND_S):
        """Gather pipeline results for one camera.

        min_frames: keep collecting past `seconds` until this many frames have
        arrived. The websocket is throttled to ~9 fps, so a 0.9 s --fast dwell
        carries ~8 frames - below MIN_SAMPLE_FRAMES, which would refuse to
        classify anything in that mode. Extending is the cheap half of the fix.
        Only extends when frames ARE arriving: at zero the pipeline is dead or
        the scene is black, and waiting longer buys nothing but a slower sweep.
        """
        out = {"frames": 0, "solves": 0, "reproj": [], "tags": [], "amb": []}
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


# ──────────────────── sampling and the camera ────────────────────

async def sample(pv, cam, cfg, seconds, after_capture=None):
    """Collect detections, preferring NetworkTables over the throttled websocket."""
    reader = cfg.readers.get(cam["nickname"])
    if reader is not None:
        # ntcore blocks; keep the event loop free so nothing else stalls.
        return await asyncio.get_event_loop().run_in_executor(
            None, reader.collect, seconds, after_capture, MIN_SAMPLE_FRAMES)
    return await pv.collect(cam["uniqueName"], seconds,
                            min_frames=MIN_SAMPLE_FRAMES)


def capture_mark(cfg, cam):
    """Newest capture timestamp right now, to gate out pre-change frames."""
    reader = cfg.readers.get(cam["nickname"])
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


async def ensure_calibrated_mode(pv, cam, cfg, log=print):
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
    bad = await pv.set_and_verify(cam["uniqueName"], settle=max(cfg.settle, 2.0),
                                  log=log, cameraVideoModeIndex=int(idx))
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
        raw = await sample(pv, cam, cfg, 1.0)
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
    "search_error": (HARD, lambda v: "the gain walk did not finish: %s" % v),
    "no_workable_settings": (HARD, lambda v: str(v)),
    "robot_enabled_midrun": (HARD,
        lambda v: "the robot was ENABLED during the tune (%s) - aborted and put "
                  "the camera back. A match must never be interrupted." % v),
    # ---- warn ----
    "video_mode_switched": (WARN,
        lambda v: "VIDEO MODE CHANGED %s -> %s - this camera is now running a "
                  "different RESOLUTION than it was. It had no calibration for "
                  "the old one." % (v[0], v[1])),
    "over_blur_budget": (WARN,
        lambda v: "%.1f px of motion blur at --blur-rate, over the budget - "
                  "fine on a bench, smeared on a moving robot. The scene needed "
                  "a longer exposure than the budget allows even at maximum "
                  "gain. Add light." % v),
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


# There is no merge_failures and no FAILURE_KEYS any more, and that is the
# point. They existed because tune_camera escalated with
# `return await tune_camera(...)`, discarding the outer frame's dict, and
# because the gain search re-entered it with skip_baseline=True - so the frame
# that ran the baseline was not the frame that returned, and four layers
# (`carry`, merge_failures, save/restore and copy.copy) existed to carry
# problems back across that gap. tune_camera is now a LOOP that builds one
# record and returns it, so there is no gap to carry anything across.


def tune_failed(r, cfg=None):
    """Did this camera's tune fail? One definition, used by the CLI and the daemon.

    The daemon used to judge on `applied` alone, so a camera that came back
    rotated 90 degrees with a gain that never took still reported ok=true to the
    dashboard - the one signal a team actually trusts.
    """
    if any(r.get(k) for k in HARD_KEYS):
        return True
    if cfg is not None and cfg.baseline_only:
        # --baseline-only never sets `applied`, by design, so "no applied"
        # cannot be the test in that mode. A baseline that did NOT take is
        # caught by HARD_KEYS above, in every mode.
        return bool(r.get("error")) and "baseline only" not in str(r.get("error"))
    return not r.get("applied")


def run_verdict(results, cfg=None):
    """(failed, warnings, exit_code) for a whole run. The CLI and daemon agree.

    Factored out of main() so the verdict can be tested directly - see
    sabotage_test.py --verdict-matrix, which asserts the exit code for every key
    in PROBLEMS one at a time.

    No results at all is a FAILURE. The tool was asked to tune something and
    tuned nothing; exiting 0 on that is how six "sweeps" were once seen
    completing in 0.00 s with nothing noticing.
    """
    failed = [r for r in (results or []) if tune_failed(r, cfg)]
    warnings = []
    for r in (results or []):
        for _k, _sev, text in problem_notes(r, WARN):
            warnings.append((r.get("camera", "?"), text))
    return failed, warnings, (1 if (failed or not results) else 0)


# ───────────────────────── configuration ─────────────────────────
#
# FROZEN, and that is the entire point of it.
#
# The gain ratchet came back three times. Every recurrence was a value written
# onto a shared argparse namespace and read by the next camera or the next
# pass - args.gain, args.gain_steps, args.min_exposure, args.max_exposure,
# args.steps. It was fixed with a save/restore, then a second save/restore, then
# a per-camera copy.copy, and each fix left the shared object in place and
# promised the next edit would not misuse it. The third fix's own comment says
# "so a third door cannot open".
#
# A frozen dataclass cannot be written to at all, so the bug is not absent, it
# is inexpressible. Search state has exactly one home - the Search object
# tune_camera builds inside its own frame - and that object is thrown away when
# the camera is done.

@dataclasses.dataclass(frozen=True)
class Config:
    host: str = "photonvision.local"
    port: int = 5800
    cameras: object = None          # comma-separated nicknames, or None for all

    # what the tune decides
    max_gain: float = 100.0
    max_blur_px: float = 6.0
    blur_rate: float = 360.0
    fx: float = 1105.9              # fallback only; camera_fx() reads the real one
    max_exposure: float = 25000.0   # ceiling for the over-budget walk only

    # how a measurement is taken
    dwell: float = 4.0
    settle: float = 0.7
    min_tags: float = 2.0
    max_ambiguity: float = 0.20

    # the baseline
    baseline: bool = True
    baseline_only: bool = False
    brightness: object = None

    # sampling transport
    no_nt: bool = False
    nt_server: object = None

    # wiring, filled in once by _run_inner and by the daemon. Not policy.
    readers: dict = dataclasses.field(default_factory=dict)
    nt_inst: object = None

    @classmethod
    def from_args(cls, a):
        return cls(host=a.host, port=a.port, cameras=a.cameras,
                   max_gain=a.max_gain, max_blur_px=a.max_blur_px,
                   blur_rate=a.blur_rate, fx=a.fx, max_exposure=a.max_exposure,
                   dwell=a.dwell, settle=a.settle, min_tags=a.min_tags,
                   max_ambiguity=a.max_ambiguity, baseline=a.baseline,
                   baseline_only=a.baseline_only, brightness=a.brightness,
                   no_nt=a.no_nt, nt_server=a.nt_server)


class Search:
    """One camera's search state. Created INSIDE the per-camera loop, never shared.

    The gain ratchet came back three times through three different doors, and
    every one of them was a value written onto the shared `args` namespace and
    read by the next camera or the next pass: args.gain, args.gain_steps,
    args.min_exposure, args.max_exposure, args.steps. The fixes were a
    save/restore, then a second save/restore, then a per-camera copy.copy - all
    of them keeping the shared object and promising not to misuse it.

    This is the structural version of that promise. Config is frozen (see
    Config) and therefore cannot hold search state at all; search state has one
    home, this object, and it is constructed fresh per camera. A line added
    later cannot reintroduce the bug, because there is nowhere to write it.
    """
    __slots__ = ("cam", "unique", "nickname", "original", "fx", "t_budget",
                 "exposure", "gain", "reference", "trials")

    def __init__(self, cam):
        self.cam = cam
        self.unique = cam["uniqueName"]
        self.nickname = cam["nickname"]
        st = cam["settings"]
        self.original = {
            "cameraExposureRaw": st["cameraExposureRaw"],
            "cameraGain": st["cameraGain"],
            "cameraAutoExposure": st["cameraAutoExposure"],
        }
        self.fx = None
        self.t_budget = None       # the blur budget, in microseconds
        self.exposure = None       # the exposure actually being tried
        self.gain = None           # the gain actually being tried
        self.reference = None      # Sample at max gain: the best the scene can do
        self.trials = []           # every Sample taken, in the order taken

    def exposure_bounds(self):
        """The camera's OWN reported exposure limits, as floats.

        PhotonVision clamps the hardware call but STORES the unclamped value,
        so an exposure written past the limit reads back "verified" while the
        sensor sits at the bound. Every exposure this tool writes is clamped
        here first so the number in the log is the number on the sensor.
        Measured on this rig's CSI OV9281: 7 to 80000 us. USB variants report
        entirely different ranges, which is why this is read and not assumed.
        """
        lo = self.cam.get("minExposureRaw")
        hi = self.cam.get("maxExposureRaw")
        return (float(lo) if lo is not None else EXPOSURE_FLOOR_FALLBACK,
                float(hi) if hi is not None else EXPOSURE_CEILING_FALLBACK)


async def trial(pv, cam, cfg, gain, exposure, log=None):
    """Measure detection quality at ONE (gain, exposure). The only measurement.

    Writes the pair, waits one settle, then samples from the first frame
    CAPTURED after the write - see capture_mark(). MEASURED apply latency on
    this rig: exposure 0.25-0.36 s over six trials, gain 0.234 s median
    (0.197-0.241, n=8), symmetric in both directions. The 0.7 s default settle
    covers both with about 3x margin, which is what makes walking gain as cheap
    as walking exposure and therefore makes this design affordable at all.
    """
    mark = capture_mark(cfg, cam)
    await pv.set_setting(cam["uniqueName"], cameraAutoExposure=False,
                         cameraGain=int(round(gain)),
                         cameraExposureRaw=float(exposure))
    await asyncio.sleep(cfg.settle)
    raw = await sample(pv, cam, cfg, cfg.dwell, mark)
    s = Sample(float(exposure), int(round(gain)))
    s.frames = raw["frames"]
    s.multitag_solves = raw["solves"]
    s.reproj = raw["reproj"]
    s.tag_counts = raw["tags"]
    s.ambiguities = raw["amb"]
    return s


class RobotEnabled(Exception):
    """The robot went enabled mid-tune. Abort and put the camera back."""


def _check_abort(cfg, where):
    """Both reasons to stop, asked BETWEEN steps and never inside one.

    robot_is_enabled was checked once before the run and never again during
    it, so a match starting mid-tune left a camera wherever the walk had got
    to. That violates "a match must never be interrupted" more directly than
    any bug four audits found.

    Both exceptions unwind through tune_camera's handler, which puts the
    camera back to what the user had before returning or re-raising.

    cfg.nt_inst is None on the CLI path, where there is no NetworkTables
    client to ask - a human at a terminal is not a match.
    """
    raise_if_shutdown(where)
    if cfg.nt_inst is not None and robot_is_enabled(cfg.nt_inst):
        raise RobotEnabled(where)


async def tune_camera(pv, cam, cfg, log=print, progress=None):
    """Fix the exposure at the blur budget, then find the least gain that works.

    One sentence a student can repeat: THE LEAST GAIN THAT STILL SEES EVERY
    TAG, AT THE EXPOSURE A SPINNING ROBOT ALLOWS.

    A LOOP, returning one record. The previous version escalated by
    `return await tune_camera(...)`, which discarded the frame that had run the
    baseline; `carry`, merge_failures, save/restore and copy.copy were four
    layers protecting one shared object from that. None of them are needed by a
    loop, so none of them are here.

    Why this shape at all: the old objective was bestReprojErr, which is
    OpenCV's RMS residual over the corners of the multi-tag fit. It measures
    internal consistency of a fit, NOT pose accuracy, and it IMPROVES when
    distant tags drop out of the solve - fewer, closer corners fit better. So
    the optimiser was rewarded for losing tags, a tag-count guard had to be
    bolted on to stop it, and with nothing in the objective penalising gain the
    only thing that ever stopped it choosing maximum was a tie-break floored at
    an arbitrary constant. MEASURED: 0.0-0.9% repeat noise, ~2.0% drift across
    one scan, decisions turning on 0.005 px, and six identical runs on one
    camera choosing gain 60, 80, 60, 80, 100, 100.

    Reprojection is still RECORDED and printed. It is simply not permitted to
    choose anything.
    """
    st = Search(cam)
    rec = {"camera": st.nickname, "uniqueName": st.unique,
           "original": dict(st.original), "applied": None, "gain": None,
           "trials": [], "budget": None}
    log("── %s ──  current exposure %s, gain %s"
        % (st.nickname, st.original["cameraExposureRaw"], st.original["cameraGain"]))

    try:
        # ---- 1. the baseline, and the calibration it depends on -----------
        if cfg.baseline:
            changed, failed, unconfirmed = await assert_baseline(pv, cam, cfg, log)
            if changed:
                fresh = await pv.cameras_fresh(timeout=8)
                newer = next((c for c in fresh if c["uniqueName"] == st.unique), None)
                if newer:
                    cam = dict(cam)
                    cam["settings"] = newer["settings"]
                    st.cam = cam
                rec["baseline_changed"] = list(changed)
            if failed:
                note_problem(rec, "baseline_failed",
                             [list(map(str, f)) for f in failed])
            if unconfirmed:
                note_problem(rec, "baseline_unconfirmed", unconfirmed)
        # AFTER the baseline: solvePNPEnabled is PhotonVision's default-off, so
        # checking first left the gate blind on exactly the fresh camera it
        # exists for. Judge on what the camera will be, not what it was.
        was_mode = (cam.get("settings") or {}).get("cameraVideoModeIndex")
        cam, problem = await ensure_calibrated_mode(pv, cam, cfg, log)
        st.cam = cam
        if (cam.get("settings") or {}).get("cameraVideoModeIndex") != was_mode:
            note_problem(rec, "video_mode_switched",
                         [was_mode, cam["settings"].get("cameraVideoModeIndex")])
        if problem:
            log("   !! CALIBRATION: %s" % problem)
            rec["error"] = "no calibration for the active resolution"
            note_problem(rec, "calibration_problem", problem)
            return rec
        if cfg.baseline_only:
            rec["error"] = "baseline only - not tuned"
            return rec

        # ---- 2. the exposure, which is not searched for -------------------
        own_fx = camera_fx(cam)
        st.fx = own_fx if own_fx else cfg.fx
        lo_e, hi_e = st.exposure_bounds()
        want = blur_budget_exposure(cfg.max_blur_px, cfg.blur_rate, st.fx)
        st.t_budget = min(max(want, lo_e), hi_e)
        rec["budget"] = {"fx": st.fx, "fx_from_calibration": bool(own_fx),
                         "max_blur_px": cfg.max_blur_px,
                         "blur_rate": cfg.blur_rate, "wanted": want,
                         "exposure": st.t_budget}
        # Say WHERE fx came from. It sets the whole scale of the budget, and a
        # line claiming "from this camera's calibration" over a --fx fallback
        # would be the tool asserting something it had not checked.
        log("   fx %.1f %s; %g px at %.0f deg/s allows %.0f us"
            % (st.fx,
               "from this camera's active calibration" if own_fx
               else "from --fx (this camera reports NO calibration for its "
                    "active mode, so the budget is a guess)",
               cfg.max_blur_px, cfg.blur_rate, want))
        if abs(st.t_budget - want) > 1.0:
            log("   clamped to the camera's reported range %.0f-%.0f us -> %.0f us"
                % (lo_e, hi_e, st.t_budget))
        st.exposure = st.t_budget

        PENDING_RESTORE[st.unique] = dict(st.original)
        gains, margin_step = gain_grid(cfg.max_gain)

        # ---- 3. the reference: the best this scene can do inside the budget
        _check_abort(cfg, "before the reference measurement")
        top = gains[-1]
        st.reference = await trial(pv, cam, cfg, top, st.exposure, log)
        st.trials.append(st.reference)
        log("   reference  gain %-4d exposure %6.0f  %s"
            % (top, st.exposure, st.reference.summary()))
        best_tags = st.reference.mean_tags

        # ---- 4. walk gain UP, stop at the first pass ----------------------
        #
        # UP from zero, not down from the top and not out from a previous
        # answer. Gain is not a setting with a good value - it is a cost paid
        # in sensor noise to buy a shorter exposure, and the exposure is
        # already fixed, so the right amount is the least that works. Starting
        # anywhere else is what made this a ratchet: escalation is one-way, so
        # a tune that starts from the last tune's answer walks toward maximum
        # gain over a season with nothing reporting it.
        picked = None
        for i, g in enumerate(gains):
            if progress:
                progress(0.1 + 0.7 * i / float(len(gains)))
            _check_abort(cfg, "walking gain at %d" % g)
            if g == top:
                s = st.reference            # already measured; same operating point
            else:
                s = await trial(pv, cam, cfg, g, st.exposure, log)
                st.trials.append(s)
            why = s.why_failed(cfg.min_tags, cfg.max_ambiguity, best_tags)
            log("   %s gain %-4d exposure %6.0f  %s%s"
                % ("ok " if why is None else "   ", g, st.exposure, s.summary(),
                   "" if why is None else "   <- " + why))
            if why is None:
                picked = g
                break

        if picked is not None:
            # ONE grid step of margin. A field is not the pit: the light
            # changes, the tags get further away, and the cost of one step is
            # sensor noise on a solve that is already passing, while the cost
            # of being one step short is a camera that stops seeing tags.
            st.gain = gain_with_margin(picked, margin_step, cfg.max_gain)
            if st.gain != picked:
                log("   lowest gain that sees every tag: %d. Applying %d - one "
                    "grid step of margin, because a field is not the pit."
                    % (picked, st.gain))
            else:
                log("   lowest gain that sees every tag: %d, which is already "
                    "the top of the range - no margin left to add." % picked)
        else:
            # ---- 5/6. nothing in the grid worked at the budget exposure ----
            st.gain, st.exposure = await _rescue(pv, cam, cfg, st, log)
            if st.gain is None:
                msg = ("no gain from %s saw the tags at %.0f us, and neither a "
                       "shorter nor a longer exposure did. That is a LIGHTING "
                       "or CONFIG problem, not a tuning one: check "
                       "cameraBrightness, check the camera is pointed at tags, "
                       "add light." % (gains, st.t_budget))
                log("   !! " + msg)
                note_problem(rec, "no_workable_settings", msg)
                rec["error"] = "nothing worked at any gain or exposure"
                await pv.set_setting(st.unique, **st.original)
                await asyncio.sleep(0.4)
                PENDING_RESTORE.pop(st.unique, None)
                return rec

        # ---- 7. apply, confirm, and report the blur we ended up with ------
        _check_abort(cfg, "before the final write")
        rec["gain"] = st.gain
        want_state = {"cameraGain": int(st.gain),
                      "cameraExposureRaw": float(st.exposure)}
        await pv.set_setting(st.unique, cameraAutoExposure=False, **want_state)
        # cameraSettings lags a write by 1-2 s, so a plain read here reports the
        # PREVIOUS value and cries MISMATCH on a write that succeeded. Poll.
        bad, read_ok = await pv.confirm(st.unique, want_state,
                                        settle=max(cfg.settle, 1.5), log=log)
        if not read_ok:
            log("   !! applied gain %d / exposure %.0f but the camera never "
                "reported back - the write is UNCONFIRMED."
                % (st.gain, st.exposure))
            note_problem(rec, "apply_unconfirmed",
                         "no cameraSettings for %.0f s after the final write"
                         % CONFIRM_TIMEOUT_S)
            rec["error"] = "could not confirm the final write"
            return rec
        if bad:
            got = {k: h for k, _w, h in bad}
            log("   *** MISMATCH *** wanted gain %d / exposure %.0f, camera "
                "reports %s" % (st.gain, st.exposure,
                                ", ".join("%s=%s" % (k, v) for k, v in got.items())))
            # applied stays None. The camera is NOT at the settings this run
            # chose, and saying "exposure -> 864" about a camera that is not at
            # 864 is the lie this tool most needed to stop telling: it used to
            # print MISMATCH and then set applied on the next line anyway.
            note_problem(rec, "apply_mismatch",
                         {"wanted": [st.gain, st.exposure],
                          "got": [got.get("cameraGain", st.gain),
                                  got.get("cameraExposureRaw", st.exposure)]})
            rec["error"] = "the final write did not take"
            return rec

        px = blur_px(st.exposure, cfg.blur_rate, st.fx)
        log("   applied gain %d / exposure %.0f - confirmed. %.1f px of smear "
            "at %.0f deg/s (budget %g)"
            % (st.gain, st.exposure, px, cfg.blur_rate, cfg.max_blur_px))
        if px > cfg.max_blur_px * 1.001:
            note_problem(rec, "over_blur_budget", round(px, 1))
        rec["applied"] = st.exposure
        PENDING_RESTORE.pop(st.unique, None)
        if progress:
            progress(1.0)
        return rec

    except BaseException as exc:
        # BaseException, not Exception. Ctrl-C raises KeyboardInterrupt, which
        # is NOT an Exception, so an interrupt during the walk skipped this
        # restore and left the camera at whatever it was testing.
        # asyncio.CancelledError is the same family.
        interrupted = isinstance(exc, (KeyboardInterrupt, asyncio.CancelledError,
                                       Terminated))
        # str(exc), not `exc or ...`: an exception object is always truthy, so
        # the old form printed "INTERRUPTED ()" for CancelledError, which
        # carries no message. The TYPE is the only information there is.
        log("   %s (%s) - restoring original settings"
            % ("INTERRUPTED" if interrupted else "ERROR",
               str(exc) or type(exc).__name__))
        restored = False
        try:
            await pv.set_setting(st.unique, **st.original)
            await asyncio.sleep(0.4)          # let the write reach PhotonVision
            restored = True
        except BaseException:
            pass
        if restored:
            PENDING_RESTORE.pop(st.unique, None)
        else:
            # The socket is gone, so nothing was restored - keep the record so
            # the exit path can replay it on a fresh connection. Dropping it
            # here meant a dropped websocket mid-walk left the camera parked at
            # whatever it was testing, while the log claimed a restore.
            PENDING_RESTORE[st.unique] = dict(st.original)
            log("   restore did NOT reach the camera - will retry on a fresh "
                "connection")
        if interrupted:
            PENDING_RESTORE[st.unique] = dict(st.original)   # loop is dying
            raise
        if isinstance(exc, RobotEnabled):
            note_problem(rec, "robot_enabled_midrun", str(exc))
            rec["error"] = "aborted: the robot went enabled (%s)" % exc
            return rec
        note_problem(rec, "search_error",
                     "%s: %s" % (type(exc).__name__, str(exc) or "no detail"))
        rec["error"] = str(exc) or type(exc).__name__
        return rec
    finally:
        rec["trials"] = [{"gain": s.gain, "exposure": s.exposure,
                          "frames": s.frames, "mean_tags": s.mean_tags,
                          "solve_rate": s.solve_rate,
                          "med_reproj": s.med_reproj,
                          "med_ambiguity": s.med_ambiguity}
                         for s in st.trials]


async def _rescue(pv, cam, cfg, st, log):
    """Nothing in the gain grid passed at the budget exposure. Which way is out?

    Two possibilities, and they need opposite answers, so guessing is not an
    option:

      SATURATED - the image is already too bright, so gain makes it worse. The
        signature is failing at gain 0 AND at maximum gain with the tag count
        FALLING as gain rises. Judged with the same significance test the walk
        uses, not with a fraction. The way out is a SHORTER exposure, which
        also costs nothing in blur.

      TOO DARK - even maximum gain cannot make the budget exposure work. The
        only way out is a LONGER exposure, which is over the blur budget by
        definition, so it is applied and reported as a warning rather than
        silently accepted.

    Returns (gain, exposure), or (None, None) if neither walk found anything.
    """
    gains, _step = gain_grid(cfg.max_gain)
    lo_e, hi_e = st.exposure_bounds()
    at_zero = next((s for s in st.trials if s.gain == gains[0]), None)
    at_top = st.reference
    saturated = (at_zero is not None
                 and at_top.tags_significantly_below(at_zero.mean_tags))

    if saturated:
        log("   tags FALL as gain rises (%.2f at gain %d, %.2f at gain %d) - "
            "the image is saturated, not dark. Walking exposure DOWN at gain %d."
            % (at_zero.mean_tags, gains[0], at_top.mean_tags, gains[-1], gains[0]))
        floor = max(lo_e, st.t_budget / 32.0)
        ladder = geometric_sweep(st.t_budget, floor, EXPOSURE_WALK_STEPS + 1)[1:]
        gain = gains[0]
    else:
        ceiling = min(hi_e, cfg.max_exposure)
        if ceiling <= st.t_budget * 1.001:
            log("   even gain %d cannot work at %.0f us, and there is no longer "
                "exposure available (ceiling %.0f us)."
                % (gains[-1], st.t_budget, ceiling))
            return None, None
        log("   even gain %d cannot see the tags at %.0f us - the scene is too "
            "dark for the blur budget. Walking exposure UP at gain %d; whatever "
            "this lands on is over budget by definition."
            % (gains[-1], st.t_budget, gains[-1]))
        ladder = geometric_sweep(st.t_budget, ceiling, EXPOSURE_WALK_STEPS + 1)[1:]
        gain = gains[-1]

    for i, e in enumerate(ladder):
        _check_abort(cfg, "walking exposure at %.0f us" % e)
        e = min(max(e, lo_e), hi_e)
        s = await trial(pv, cam, cfg, gain, e, log)
        st.trials.append(s)
        # No tag reference on this walk. The reference is "the best the scene
        # can do INSIDE the budget", and this walk has already left the budget -
        # comparing against a count that was measured in a state we have just
        # abandoned would reject the only settings that work.
        why = s.why_failed(cfg.min_tags, cfg.max_ambiguity)
        log("   %s gain %-4d exposure %6.0f  %s%s"
            % ("ok " if why is None else "   ", gain, e, s.summary(),
               "" if why is None else "   <- " + why))
        if why is None:
            return gain, e
    return None, None


async def verify_final_state(pv, cam, rec, cfg, log):
    """Re-read the camera AFTER its tune and check it is where we left it.

    Every read-back before this one was a snapshot taken seconds before the run
    ended, and a snapshot only proves the value was right at that instant.
    MEASURED: a competing writer was bounced offline inside the baseline
    read-back window; the read-back saw the value it wanted, recorded no
    failure, the run exited 0 - and the camera ended that run with
    cameraRedGain=50, which the baseline exists to set to 0. Nothing in the
    tool looked again.

    Only runs when something was actually applied: on a failure path the camera
    is deliberately put back to the user's own settings, which legitimately do
    not match the baseline.
    """
    if not rec.get("applied") and not cfg.baseline_only:
        return
    expect = {}
    if cfg.baseline:
        for tbl in (BASELINE_ALWAYS, BASELINE_DEFAULT):
            for k, (_v, e, _why) in tbl.items():
                expect[k] = e
        if cfg.brightness is not None:
            expect["cameraBrightness"] = int(cfg.brightness)
    if rec.get("applied"):
        expect["cameraExposureRaw"] = float(rec["applied"])
        if rec.get("gain") is not None:
            expect["cameraGain"] = int(round(float(rec["gain"])))
    if not expect:
        return
    bad, read_ok = await pv.confirm(cam["uniqueName"], expect, settle=0.6, log=log)
    if not read_ok:
        log("   !! could not read %s back at the end of its tune - the state it "
            "is in is unknown." % cam["nickname"])
        note_problem(rec, "apply_unconfirmed",
                     "no cameraSettings at the end of the tune")
    elif bad:
        log("   !! the camera is NOT in the state this tune left it in:")
        for k, want, have in bad:
            log("      %s is %s, should be %s" % (k, have, want))
        log("   !! something else is writing to this camera, or a write was "
            "rolled back after it was confirmed.")
        note_problem(rec, "final_state_wrong",
                     [[k, want, have] for k, want, have in bad])


class AlreadyRunning(Exception):
    """Another photontune already holds the run lock."""


class RunLock:
    """Advisory lock, so two photontunes cannot fight over the same cameras.

    This ships as a daemon AND a CLI on the same box, so a human running the CLI
    while the daemon fires is normal rather than exotic, and nothing detected
    it. Overlapping runs are not merely wasteful: two writers driving the same
    camera's gain and exposure interleave, so each one measures the other's
    settings and both answers are meaningless. MEASURED on this rig with the
    NetworkTables-server toggle that used to live here, the second run bounced
    the network stack, killed the first run's websocket with "no close frame
    received or sent" and left the camera parked mid-sweep at 9966 us on gain 0.

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
                "  Two at once fight over the same cameras: each one measures "
                "the other's writes,\n"
                "  so both answers are meaningless and whichever finishes last "
                "leaves the camera\n"
                "  wherever it happened to be. Wait for it, or stop it."
                % (self.holder or "pid unknown"))


async def run(cfg, log=print, progress=None, on_camera=None):
    lock = RunLock()
    if not lock.acquire():
        raise AlreadyRunning(lock.message())
    try:
        return await _run_inner(cfg, log, progress, on_camera)
    finally:
        lock.release()


async def _run_inner(cfg, log=print, progress=None, on_camera=None):
    async with Photon(cfg.host, cfg.port) as pv:
        cams = await pv.cameras()
        if not cams:
            raise LookupError("no cameras reported by PhotonVision at %s" % cfg.host)
        if cfg.cameras:
            want = {c.strip().lower() for c in cfg.cameras.split(",")}
            matched = [c for c in cams
                       if c["nickname"].lower() in want or c["uniqueName"].lower() in want]
            if not matched:
                raise LookupError(
                    "no camera matched %r. Available: %s"
                    % (cfg.cameras, ", ".join(c["nickname"] for c in cams)))
            cams = matched
        log("%s %d camera(s): %s"
            % ("baseline for" if cfg.baseline_only else "tuning", len(cams),
               ", ".join(c["nickname"] for c in cams)))
        readers = {} if cfg.baseline_only else _open_readers(cfg, cams, log)
        # The ONE place the frozen config gains its sampling wiring. After this
        # line nothing may write to cfg at all, which is what makes the gain
        # ratchet inexpressible rather than merely absent.
        cfg = dataclasses.replace(cfg, readers=readers)
        if readers:
            log("   sampling over NetworkTables (~4.8x the frames of the websocket)")
        elif not cfg.baseline_only:
            log("   sampling over the websocket - it is throttled to ~9 results/s")
            log("   per camera (measured on this rig, both cameras streaming:")
            log("   8.9 and 9.0), so a %.1f s dwell is ~%d frames."
                % (cfg.dwell, int(cfg.dwell * 9)))
        try:
            return await _tune_all(pv, cams, cfg, log, progress, on_camera)
        finally:
            for r in readers.values():
                r.close()


def _open_readers(cfg, cams, log):
    """NetworkTables readers for every camera, or {} if there is no server.

    photontune never CREATES a server - see NTResults. --nt-server points at
    one that already exists; on a robot that is the roboRIO. Loopback is tried
    as well because running ON the coprocessor, --host defaults to
    photonvision.local and resolving its own mDNS name does not connect, so a
    bare invocation silently fell back to the slow path.
    """
    if cfg.no_nt:
        return {}
    cands = [cfg.nt_server or cfg.host]
    if "127.0.0.1" not in cands:
        cands.append("127.0.0.1")
    for ntsrv in cands:
        readers = {}
        try:
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
                log("   NetworkTables server: %s" % ntsrv)
                return readers
        except Exception as exc:
            for done in readers.values():
                done.close()
            log("   NT unavailable via %s (%s)" % (ntsrv, exc))
    return {}


async def _tune_all(pv, cams, cfg, log, progress, on_camera):
    """Tune each camera in turn. Sequential: cameras perturb each other's light.

    There is no per-camera copy of anything here, and no save/restore, because
    there is nothing mutable to copy: cfg is frozen and every value the search
    writes lives on the Search object tune_camera builds inside its own frame.
    That is the gain ratchet's three doors closed by construction rather than
    by three promises.
    """
    results = []
    for idx, cam in enumerate(cams):
        def cam_progress(f, idx=idx):
            if progress:
                progress((idx + f) / float(len(cams)))
        if on_camera:
            on_camera(cam["nickname"], idx, len(cams))
        rec = await tune_camera(pv, cam, cfg, log, cam_progress)
        await verify_final_state(pv, cam, rec, cfg, log)
        results.append(rec)
    if progress:
        progress(1.0)
    return results


# ───────────────────────── B: NetworkTables trigger ─────────────────────────

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

    Needs an NT server to ALREADY exist somewhere - on a robot, the roboRIO.
    photontune never creates one. Verified from PhotonVision's source: toggling
    runNTServer posts to /api/settings/general, which calls stopServer() (which
    orphans every NT client) AND NetworkManager.reinitialize(), and the latter
    restarts the web server and drops every websocket regardless of
    shouldManage. Managing the server destroys the tool's own control channel.
    With no server reachable, callers fall back to the websocket at ~9 Hz.
    """

    def __init__(self, server, nickname):
        import ntcore
        from photonlibpy.packet import Packet
        from photonlibpy.targeting.photonPipelineResult import PhotonPipelineResult
        self._Packet = Packet
        self._Result = PhotonPipelineResult
        # A PRIVATE instance. Sampling must not re-point or re-identify the
        # connection the daemon publishes its status table on: sharing the
        # default singleton orphaned the daemon's publishers, and its NT table
        # then said "running...", ok=false, forever.
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

    def collect(self, seconds, after_capture=None, min_frames=0,
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
               "stale_dropped": 0}
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
            # This runs in an executor thread, where a task cancellation cannot
            # reach it at all - the await in sample() unblocks but this loop
            # keeps polling to the end of its dwell. The flag can reach it.
            raise_if_shutdown("sampling over NetworkTables")
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


async def assert_baseline(pv, cam, cfg, log=print):
    """Put the structural settings where they must be, before tuning anything.

    Reports every change and its reason, so 'blindly applied' is visible rather
    than implicit. Returns (changed, failed, unconfirmed).
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
    if cfg.brightness is not None:
        want["cameraBrightness"] = int(cfg.brightness)
        expect["cameraBrightness"] = int(cfg.brightness)

    live = cam.get("settings", {})
    todo = {}
    for k, v in want.items():
        if _matches(live.get(k), expect[k]):
            continue
        todo[k] = v
    if not todo:
        log("   baseline: already correct")
        return [], [], None

    for k, v in todo.items():
        why = (BASELINE_ALWAYS.get(k) or BASELINE_DEFAULT.get(k))[2]
        log("   baseline: %s %s -> %s" % (k, live.get(k), v))
        log("             %s" % why.split(". ")[0] + ".")

    await pv.set_setting(unique, **todo)
    # Poll, do not snapshot. A single look 1.5 s later saw a value that a
    # competing writer then took back, and reported it as applied.
    bad, read_ok = await pv.confirm(unique, {k: expect[k] for k in todo},
                                    settle=max(cfg.settle, 1.5), log=log)
    failed, unconfirmed = [], None
    if not read_ok:
        # NOT silence. This used to log one line, record nothing, and let the
        # run continue and exit 0 while the baseline state was unknown.
        log("   !! baseline: could not read the camera back to confirm, after "
            "polling %.0f s. The baseline state is UNKNOWN." % CONFIRM_TIMEOUT_S)
        unconfirmed = ("no cameraSettings for %.0f s after writing %s"
                       % (CONFIRM_TIMEOUT_S, ", ".join(sorted(todo))))
    else:
        for k, want, have in bad:
            failed.append((k, want, have))
            log("   !! baseline %s did NOT take (wanted %s, camera has %s)"
                % (k, want, have))
    return list(todo), failed, unconfirmed


def robot_is_enabled(inst):
    """True if the FMS/DS says enabled. Never retune during a match."""
    try:
        tbl = inst.getTable("FMSInfo")
        control = tbl.getEntry("FMSControlData").getInteger(0)
        return bool(control & 0x01)          # bit 0 = enabled
    except Exception:
        return False


async def daemon(args, log=print):
    """Assert the baseline once at boot; tune only when asked.

    BOOT ASSERTS THE BASELINE ONLY - seconds, safe, and it cannot leave a
    camera mid-walk. TUNING IS ON DEMAND, from the dashboard, at the field.

    Why, verified rather than assumed:

      - PhotonVision already persists every websocket write to SQLite, so
        tune-once-and-persist is the platform's own default. Re-deriving the
        same answer every power cycle buys nothing.
      - The boot autorun this replaces spent ~131 s sweeping both cameras
        through blind exposures while the robot may be on the cart about to be
        enabled, and robot_is_enabled was checked once before the run and never
        again during it. A match starting mid-sweep left a camera wherever the
        sweep had got to. That violates "a match must never be interrupted"
        more directly than any bug four audits found.
      - Lighting differs between the pit and the field. You want to tune where
        you will play, not where you parked - which is an argument FOR on
        demand and AGAINST boot.

    The baseline is different in kind: it is idempotent, it writes settings
    that have ONE right answer, it finishes in seconds, and three of its
    entries were silently never applied for two weeks. That is worth doing
    every boot.
    """
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

    base_cfg = dataclasses.replace(Config.from_args(args), nt_inst=inst)

    tbl = inst.getTable(args.nt_table)
    run_entry = tbl.getEntry("run")
    run_entry.setDefaultBoolean(False)
    status = tbl.getEntry("status")          # live human-readable line
    busy = tbl.getEntry("busy")              # true while tuning
    ok_entry = tbl.getEntry("ok")            # <- go / no-go for the last run
    summary = tbl.getEntry("summary")        # <- one line, what it did
    warnings_e = tbl.getEntry("warnings")    # <- things that are not failures but
                                             #    that a human must still be told
    progress_e = tbl.getEntry("progress")    # <- 0..1, for a progress bar
    heartbeat = tbl.getEntry("heartbeat")    # <- proves the service is alive
    result_entry = tbl.getEntry("result")    # full JSON
    camera_e = tbl.getEntry("camera")        # <- camera being tuned, "2 of 3"
    boot_ran = tbl.getEntry("bootBaselineRan")      # <- did the boot baseline happen
    boot_ok = tbl.getEntry("bootBaselineOk")        # <- did it succeed
    boot_sum = tbl.getEntry("bootBaselineSummary")  # <- what it did, or why not
    status.setString("idle")
    summary.setString("never run")
    warnings_e.setString("")
    busy.setBoolean(False)
    ok_entry.setBoolean(False)
    progress_e.setDouble(0.0)
    boot_ran.setBoolean(False)
    boot_ok.setBoolean(False)
    boot_sum.setString("pending")

    async def boot_baseline():
        """Wait for PhotonVision to answer, then assert the baseline. Seconds."""
        t0 = time.time()
        ready = False
        while time.time() - t0 < args.boot_timeout:
            try:
                async with Photon(args.host, args.port) as probe:
                    if await probe.cameras():
                        ready = True
                        break
            except Exception:
                pass
            await asyncio.sleep(2.0)
        if not ready:
            why = "PhotonVision not ready within %.0fs" % args.boot_timeout
            boot_ran.setBoolean(False); boot_ok.setBoolean(False)
            boot_sum.setString(why)
            log("boot baseline skipped: " + why)
            return
        # The baseline is safe with the robot enabled in a way a tune is not -
        # it writes only settings that have one right answer and it does not
        # move exposure or gain - but it still changes the pipeline, so it
        # waits rather than interrupting a match.
        if robot_is_enabled(inst):
            boot_ran.setBoolean(False); boot_ok.setBoolean(False)
            boot_sum.setString("skipped: robot was enabled at boot")
            log("boot baseline skipped: robot is enabled")
            return
        status.setString("asserting the baseline...")
        t1 = time.time()
        try:
            res = await run(dataclasses.replace(base_cfg, baseline_only=True),
                            log=log)
        except Exception as exc:
            boot_ran.setBoolean(True); boot_ok.setBoolean(False)
            boot_sum.setString("error: %s" % exc)
            status.setString("boot baseline error: %s" % exc)
            log("boot baseline error: %s" % exc)
            return
        failed, warns, code = run_verdict(
            res, dataclasses.replace(base_cfg, baseline_only=True))
        line = "; ".join(
            "%s: %s" % (r["camera"],
                        ("%d changed" % len(r["baseline_changed"]))
                        if r.get("baseline_changed") else "already correct")
            for r in res)
        if failed:
            line += "; FAILED: " + ", ".join(
                "%s (%s)" % (r["camera"], r.get("error", "?")) for r in failed)
        boot_ran.setBoolean(True)
        boot_ok.setBoolean(code == 0)
        boot_sum.setString(line[:200])
        status.setString("idle - baseline asserted in %.0f s" % (time.time() - t1))
        log("boot baseline done in %.0f s: ok=%s %s"
            % (time.time() - t1, code == 0, line))

    asyncio.ensure_future(boot_baseline())

    log("daemon up. table /%s  - set 'run' true to tune." % args.nt_table)
    last = False
    beat = 0.0
    while True:
        beat += 1.0
        heartbeat.setDouble(beat)
        trigger = run_entry.getBoolean(False)
        if trigger and not last:
            if robot_is_enabled(inst):
                status.setString("refused: robot enabled")
                log("trigger ignored - robot is enabled")
                run_entry.setBoolean(False)
            else:
                busy.setBoolean(True)
                warnings_e.setString("")
                ok_entry.setBoolean(False)
                progress_e.setDouble(0.0)
                status.setString("tuning on the tags in view...")
                summary.setString("running...")
                def cap(msg):
                    log(msg); status.setString(str(msg)[:120])
                def announce(nick, idx, total):
                    camera_e.setString("%s (%d of %d)" % (nick, idx + 1, total))

                try:
                    res = await run(base_cfg, log=cap,
                                    progress=lambda f: progress_e.setDouble(f),
                                    on_camera=announce)
                    failed, warns, code = run_verdict(res, base_cfg)
                    applied = [r for r in res if r not in failed and r.get("applied")]
                    line = "; ".join("%s=%.0f@g%s" % (r["camera"], r["applied"],
                                                      r.get("gain"))
                                     for r in applied)
                    if failed:
                        line += ("; FAILED: " + ", ".join(
                            "%s (%s)" % (r["camera"], r.get("error", "?"))
                            for r in failed))
                    # Warnings reach the dashboard, not just the log. A camera
                    # whose RESOLUTION this tool changed, or one applied over
                    # the blur budget, published ok=true and said nothing - so
                    # the one signal a team reads could not tell them their
                    # camera is no longer running the mode they set.
                    wline = " | ".join("%s: %s" % (who, t) for who, t in warns)
                    warnings_e.setString(wline[:500])
                    if wline:
                        line += "  [WARN] " + wline
                        log("WARNINGS: " + wline)
                    every_camera_ok = bool(applied) and code == 0
                    ok_entry.setBoolean(every_camera_ok)
                    summary.setString(line or "no cameras tuned")
                    status.setString(("DONE - " if every_camera_ok
                                      else "DONE WITH ERRORS - ") + line)
                    result_entry.setString(json.dumps(res))
                    progress_e.setDouble(1.0)
                    log("done: " + line)
                except AlreadyRunning as exc:
                    # SKIP the trigger, do not fail it. A human at the CLI while
                    # the daemon is triggered is the ordinary case, and the
                    # human's run is the one that should win - they are standing
                    # there watching it.
                    msg = "skipped: %s" % str(exc).splitlines()[0]
                    summary.setString(msg); status.setString(msg); log(msg)
                except Exception as exc:
                    ok_entry.setBoolean(False)
                    summary.setString("error: %s" % exc)
                    status.setString("error: %s" % exc)
                    log("error: %s" % exc)
                finally:
                    # The daemon is the DEPLOYED mode, and the replay used to
                    # sit in the `except` branch only - which is the branch a
                    # dropped websocket does NOT take, because tune_camera
                    # swallows it and returns normally. Every exit path, or it
                    # is not a restore.
                    try:
                        for unique, orig, ok in await restore_pending(args.host,
                                                                      args.port):
                            log("%s %s to %s"
                                % ("restored" if ok else "UNVERIFIED restore of",
                                   unique, orig))
                    except Exception as rexc:
                        log("restore replay failed: %s" % rexc)
                    busy.setBoolean(False)
                    camera_e.setString("")
                    run_entry.setBoolean(False)
        last = trigger
        await asyncio.sleep(0.2)


# ───────────────────────── A: CLI ─────────────────────────

def build_parser():
    p = argparse.ArgumentParser(
        description="Set PhotonVision exposure from a motion-blur budget, then "
                    "find the least gain that still sees every tag.")
    p.add_argument("--host", default="photonvision.local", help="PhotonVision host")
    p.add_argument("--port", type=int, default=5800)
    p.add_argument("--cameras", default=None,
                   help="comma-separated nicknames to tune (default: all)")

    p.add_argument("--max-blur-px", type=float, default=6.0,
                   help="motion-blur budget in pixels at --blur-rate (default "
                        "%(default)g). This SETS the exposure: "
                        "t = max_blur_px / (radians(blur_rate) * fx). "
                        "PROXY-DERIVED, not measured on a moving robot. "
                        "PhotonVision's Gaussian blur swept at fixed exposure "
                        "loses 12%% of tags by sigma 0.5 and all multi-tag "
                        "between sigma 1.5 and 2.0; converting by equal "
                        "high-frequency attenuation (sigma = L/sqrt(12)) puts "
                        "the cliff near 3.5 px of smear. Gaussian blur damages "
                        "edges in every direction while motion smear only "
                        "damages the ones perpendicular to it, and a tag has "
                        "edges in two orthogonal directions, so the proxy "
                        "overstates the damage by roughly 2x - hence 6 rather "
                        "than 3.5. blurtest.py measures the real thing, but it "
                        "needs a human waving a tag.")
    p.add_argument("--blur-rate", type=float, default=360.0,
                   help="angular rate the blur budget is judged at, deg/s. 360 is "
                        "what robots actually do while aiming (CTRE default 270, "
                        "REV 360, Limelight rejects vision above 360); 900 is "
                        "never-exceed.")
    p.add_argument("--max-gain", type=float, default=100.0,
                   help="top of the gain grid. The walk visits six points from 0 "
                        "to here and stops at the first that sees every tag, so "
                        "this also sets the margin step (default %(default)g -> "
                        "steps of 20).")
    p.add_argument("--fx", type=float, default=1105.9,
                   help="focal length in px, used ONLY if the camera's active "
                        "calibration does not report one. The blur budget scales "
                        "directly with fx, and on this rig fx is 1105.9 at "
                        "1280x800 but 570.5 at 640x400, so a default carried "
                        "across a resolution change overstates blur by 1.94x.")
    p.add_argument("--max-exposure", type=float, default=25000.0,
                   help="ceiling for the too-dark exposure walk ONLY. The tuned "
                        "exposure is the blur budget; this bounds how far past it "
                        "the tool may go when even maximum gain cannot see the "
                        "tags, and anything it lands on is reported as over "
                        "budget.")

    p.add_argument("--dwell", type=float, default=4.0,
                   help="seconds of data per trial (default %(default)s). Over the "
                        "websocket that is ~36 results at the ~9/s measured on "
                        "this rig - the sampling error on the tag count is what "
                        "decides a marginal gain, so this is not a knob to "
                        "shorten casually.")
    p.add_argument("--settle", type=float, default=0.7,
                   help="seconds to wait after changing a setting (default "
                        "%(default)s). MEASURED on a Pi 5 / OV9281 against "
                        "PhotonVision's own capture timestamps: exposure applies "
                        "in 0.25-0.36 s (6 trials), gain in 0.234 s median "
                        "(0.197-0.241, n=8). This is the worst case roughly "
                        "doubled.")
    p.add_argument("--min-tags", type=float, default=2.0,
                   help="mean tags per frame required when multi-tag is unavailable")
    p.add_argument("--max-ambiguity", type=float, default=0.20)

    p.add_argument("--no-baseline", dest="baseline", action="store_false", default=True,
                   help="do NOT assert the structural settings first. They are "
                        "asserted by default: brightness, rotation, blur and the tag "
                        "model have a right answer, and leaving them unowned is what "
                        "lets a bad brightness silently ruin a tune.")
    p.add_argument("--brightness", type=int, default=None,
                   help="override the asserted cameraBrightness (default 40)")
    p.add_argument("--baseline-only", action="store_true",
                   help="assert the structural settings and stop - do not tune. "
                        "This is what the daemon does at boot.")

    p.add_argument("--no-nt", action="store_true",
                   help="always sample from the websocket, never NetworkTables")
    p.add_argument("--nt-server", default=None,
                   help="host of an EXISTING NetworkTables server - the roboRIO on "
                        "a robot. photontune never starts one: toggling "
                        "PhotonVision's own server calls stopServer() and "
                        "NetworkManager.reinitialize(), which orphans every NT "
                        "client and drops every websocket, including this tool's.")
    p.add_argument("--json", action="store_true", help="emit JSON results")

    p.add_argument("--daemon", action="store_true", help="NT-triggered service mode")
    p.add_argument("--team", type=int, default=0, help="team number for NT (daemon mode)")
    p.add_argument("--nt-table", default="PhotonTune")
    p.add_argument("--boot-timeout", type=float, default=90.0,
                   help="daemon: give up waiting for PhotonVision to answer after "
                        "this many seconds (default %(default)s)")
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
        note_shutdown(signum)
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
        note_shutdown(signum)
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
        # A Terminated raised by raise_if_shutdown() inside a sampling loop the
        # cancellation never reached needs no clause here: it is already the
        # exception main() looks for, and it propagates on its own.
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
    if args.max_blur_px <= 0:
        sys.exit("--max-blur-px must be > 0: it is what SETS the exposure")
    if args.blur_rate <= 0:
        sys.exit("--blur-rate must be > 0")
    if args.daemon:
        try:
            try:
                asyncio.run(_guard_signals(daemon(args)))
            except (KeyboardInterrupt, Terminated):
                # systemd stops the daemon with SIGTERM, and the daemon is the
                # DEPLOYED mode. Without this the signal unwound to the top with a
                # traceback and, worse, any camera left mid-sweep stayed there.
                print("photontune: stopping - restoring any camera left mid-tune")
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
            results = asyncio.run(_guard_signals(run(Config.from_args(args))))
        except (AlreadyRunning, ConnectionError, LookupError) as exc:
            sys.exit("photontune: %s" % exc)
        except (KeyboardInterrupt, Terminated) as exc:
            # The interrupted loop could not complete the restore. Do it on a
            # new one - see the `finally` below. Exit 130 for Ctrl-C and 143
            # for SIGTERM, the shell's own conventions (128 + signal), so a
            # script that launched this can tell WHICH interruption it was.
            print("\ninterrupted - putting camera settings back...")
            sys.exit(143 if isinstance(exc, Terminated) else 130)
        failed, warnings, code = run_verdict(results, Config.from_args(args))
        if args.json:
            print(json.dumps(results, indent=2))
        else:
            print("\ncompleted in %.0f s" % (time.time() - t0))
            for r in results:
                if r.get("applied"):
                    print("  %-14s exposure -> %.0f, gain %s"
                          % (r["camera"], r["applied"], r.get("gain")))
                elif args.baseline_only:
                    print("  %-14s baseline asserted%s"
                          % (r["camera"],
                             (" (%d changed)" % len(r["baseline_changed"]))
                             if r.get("baseline_changed") else " (already correct)"))
                else:
                    print("  %-14s unchanged (%s)"
                          % (r["camera"], r.get("error", "nothing applied")))
            # Warnings are NOT failures and are NOT footnotes. A camera left on
            # a resolution nobody chose, or applied over the blur budget, both
            # used to exit 0 with nothing in the summary at all.
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
        sys.exit(code)
    finally:
        # EVERY exit path, including the ordinary one and sys.exit above.
        replay_pending(args.host, args.port)


if __name__ == "__main__":
    main()
