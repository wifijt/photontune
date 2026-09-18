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
import argparse, asyncio, json, math, sys, time
from collections import defaultdict

try:
    import msgpack, websockets
except ImportError:
    sys.exit("need: pip install msgpack websockets")


# ───────────────────────── scoring ─────────────────────────

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
        if self.frames < 3:
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
            return self.ref_rate >= 0.95
        # Prefer the multi-tag criterion when multi-tag is actually running.
        if self.multitag_solves:
            return self.solve_rate >= 0.90
        # Otherwise: enough tags, seen reliably, at trustworthy ambiguity.
        if self.mean_tags < min_tags:
            return False
        amb = self.med_ambiguity
        return amb is not None and amb <= max_ambiguity

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

    async def collect(self, unique_name, seconds, ref_tag=None):
        """Gather pipeline results for one camera."""
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
        return out


# ───────────────────────── the sweep ─────────────────────────

async def sample(pv, cam, args, seconds, ref_tag, after_capture=None):
    """Collect detections, preferring NetworkTables over the throttled websocket."""
    reader = getattr(args, "_nt", {}).get(cam["nickname"])
    if reader is not None:
        # ntcore blocks; keep the event loop free so nothing else stalls.
        return await asyncio.get_event_loop().run_in_executor(
            None, reader.collect, seconds, ref_tag, after_capture)
    return await pv.collect(cam["uniqueName"], seconds, ref_tag)


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


def calibration_problem(cam):
    """Why this camera cannot produce a 3D pose, or None if it can.

    Checked BEFORE tuning because the failure is otherwise indistinguishable
    from darkness: with solvePNP on and no calibration for the active
    resolution, PhotonVision publishes nothing at all, so every exposure scores
    zero, gain escalates to the ceiling, and the tool blames the lighting. The
    one config problem it actually is never gets named.
    """
    st = cam.get("settings", {})
    if not st.get("solvePNPEnabled"):
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


async def tune_camera(pv, cam, args, log=print, progress=None, original=None,
                      skip_baseline=False):
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

    problem = calibration_problem(cam)
    if problem:
        log("   !! CALIBRATION: %s" % problem)
        result["error"] = "no calibration for the active resolution"
        result["calibration_problem"] = problem
        return result

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
            result["setting_rejected"] = [list(map(str, b)) for b in rejected]
        if unverified:
            log("   NOTE: could not read the camera back to confirm the gain. The"
                " settings were sent; this is a readback timeout, not a rejection.")
            result["unverified"] = True

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
                    result["range_warning"] = {"measured": measured, "stated": args.reference_range}
            else:
                log("   WARNING: reference tag %d never seen at any exposure." % args.reference_tag)
                result["range_warning"] = "reference tag never detected"

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
                nxt = min(args.max_gain, max(gain * 1.5, gain + 10))
                log("   nothing passed at gain %g - retrying at gain %g" % (gain, nxt))
                log("   (gain costs noise; exposure costs blur - prefer gain)")
                args.gain = nxt
                args.gain_steps -= 1
                return await tune_camera(pv, cam, args, log, progress,
                                         original=original, skip_baseline=True)
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
                nxt = min(args.max_gain, max(gain * 1.5, gain + 10))
                log("   %.0f px of blur at %.0f deg/s exceeds the %.0f px budget"
                    % (predicted, args.blur_rate, args.max_blur_px))
                log("   raising gain %g -> %g and re-sweeping (gain costs noise, "
                    "exposure costs blur)" % (gain, nxt))
                args.gain = nxt
                args.gain_steps -= 1
                return await tune_camera(pv, cam, args, log, progress,
                                         original=original, skip_baseline=True)
            log("   !! %.1f px of blur at %.0f deg/s, over the %.0f px budget, and"
                % (predicted, args.blur_rate, args.max_blur_px))
            log("   !! gain is already at %g. Works on a BENCH; will smear on a"
                % gain)
            log("   !! moving robot. Add light, or raise --max-gain.")
            result["over_blur_budget"] = round(predicted, 1)
        elif predicted > args.max_blur_px and at_floor:
            log("   note: %.1f px of blur at %.0f deg/s is over the %.0f px budget,"
                % (predicted, args.blur_rate, args.max_blur_px))
            log("   but the cliff is already at the bottom of the sweep (%.0f us) -"
                % args.min_exposure)
            log("   more gain cannot buy a shorter exposure. Lower --min-exposure"
                " to explore further, or add light.")
            result["over_blur_budget"] = round(predicted, 1)

        if args.dry_run:
            log("   dry run - restoring original")
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
                result["fellback"] = True
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


async def optimise_gain(pv, cam, args, log=print, progress=None, skip_baseline=False):
    """Find the (gain, exposure) pair giving the SHORTEST exposure that still sees
    all the tags - because exposure costs motion blur and gain only costs noise.

    Sweeping exposure at a fixed gain answers "what is the shortest exposure at THIS
    gain", which is not the question. Raising gain lowers the exposure needed; the
    limit is that noise eventually degrades corner precision, so we stop when
    reprojection error starts getting worse rather than simply maximising gain.
    """
    # same reasoning as tune_camera: a search must not start from its own last answer
    start = (args.gain if args.gain is not None
             else getattr(args, "start_gain", BASELINE_START_GAIN))
    # Span start..max_gain evenly. The old rule stepped by max(g*1.7, g+8), which
    # overshoots the ceiling near the top: starting at 60 with a max of 100, the
    # next candidate was 102 and the "search" ran with a single value. It also
    # sampled the low end finely, which is where the measurements say the answer
    # is NOT - on an OV9281 in a dim room reprojection improved monotonically from
    # gain 0 to 79, so the interesting region is the top.
    n = max(1, int(args.gain_search_steps))
    lo, hi = float(start), float(args.max_gain)
    if n == 1 or hi <= lo:
        gains = [lo]
    else:
        gains = [lo + (hi - lo) * i / float(n - 1) for i in range(n)]
    gains = sorted({round(g) for g in gains})
    log("── %s ──  gain search over %s" % (cam["nickname"], [round(x) for x in gains]))

    trials = []
    saved_gain = args.gain
    saved_steps, args.gain_steps = args.gain_steps, 0      # no escalation inside a trial
    saved_min, saved_max, saved_n = args.min_exposure, args.max_exposure, args.steps
    for i, gv in enumerate(gains):
        args.gain = gv
        if progress:
            progress(i / float(len(gains)))
        # After the first gain has located the cliff, stop re-sweeping the whole
        # 1000-25000 range to rediscover it. Raising gain moves the cliff DOWN or
        # leaves it alone - it cannot need a longer exposure - so a bracket around
        # the previous answer is sufficient. Measured across every run on this rig:
        # the cliff sat at 1500 us for every gain from 20 to 100. The full sweep was
        # 8 steps; the bracket is 4, which is most of the runtime of a gain search.
        if trials:
            prev = min(t[1] for t in trials)
            args.min_exposure = max(saved_min, prev / 2.5)
            args.max_exposure = min(saved_max, prev * 2.5)
            args.steps = max(3, min(saved_n, 4))
        r = await tune_camera(pv, cam, args, log=lambda m: None, progress=None,
                              skip_baseline=skip_baseline)
        exp = r.get("applied") or (r.get("cliff") and None)
        if r.get("applied") and r.get("samples"):
            best = min((x for x in r["samples"] if x["passes"]),
                       key=lambda x: x["exposure"], default=None)
            rp = best.get("med_reproj") if best else None
            trials.append((gv, r["applied"], rp, r))
            log("   gain %-5.0f -> exposure %7.0f   reproj %s"
                % (gv, r["applied"], ("%.3f" % rp) if rp else "n/a"))
        else:
            log("   gain %-5.0f -> no passing exposure" % gv)
    args.gain_steps = saved_steps
    args.gain = saved_gain
    args.min_exposure, args.max_exposure, args.steps = saved_min, saved_max, saved_n

    if not trials:
        return None
    good = [t for t in trials if t[2] is not None]
    best_rp = min((t[2] for t in good), default=None)
    if best_rp is not None:
        # Reject gains where noise has visibly hurt the fit.
        usable = [t for t in trials if t[2] is None or t[2] <= best_rp * args.reproj_tolerance]
    else:
        usable = trials
    # Shortest exposure first - that is what buys motion tolerance. But among the
    # gains that all reach that same exposure, take the one that REPROJECTS best,
    # not the lowest number.
    #
    # Measured on an OV9281 in a dim room, every gain from 8 up reached 1500 us and
    # reprojection improved the whole way: 1.225 -> 1.036 -> 0.841 px. Picking the
    # lowest gain discarded a 32% better fit for no gain in shutter speed. "Gain
    # only costs noise" is wrong at the dark end: too little gain means a
    # low-contrast image, and corner refinement needs contrast more than it needs
    # a low noise floor.
    trials = [(round(g), e, rp, r) for (g, e, rp, r) in trials]
    usable = [(round(g), e, rp, r) for (g, e, rp, r) in usable]
    best_exp = min(t[1] for t in usable)
    tied = [t for t in usable if t[1] <= best_exp * 1.05]
    pick = min(tied, key=lambda t: (t[2] if t[2] is not None else float("inf")))
    log("   chose gain %.0f with exposure %.0f "
        "(shortest exposure; best reproj %s of %d gain(s) that reached it)"
        % (pick[0], pick[1],
           ("%.3f" % pick[2]) if pick[2] is not None else "n/a", len(tied)))
    if len(tied) > 1:
        log("      tied on exposure: %s"
            % ", ".join("gain %.0f -> %s" % (t[0], ("%.3f" % t[2]) if t[2] else "n/a")
                        for t in sorted(tied, key=lambda x: x[0])))
    if pick[0] >= max(t[0] for t in trials) and len(trials) > 1:
        log("      NOTE: the best gain is the TOP of the search range - the optimum "
            "may be higher. Raise --gain-search-steps or --max-gain to find it.")
    # Deliberately NOT written back into args.gain: this object is shared across
    # cameras, and camera 1's answer became camera 2's search START - which made
    # camera 2 "search" a single candidate and silently inherit camera 1's gain.
    # Each camera gets its own light; each gets its own search.

    # Explicitly apply the winner. The trials leave whichever pair was tried LAST
    # on the camera, which is only the winner by luck - so write it, then confirm
    # the camera actually took it rather than assuming.
    if not args.dry_run:
        await pv.set_setting(cam["uniqueName"], cameraAutoExposure=False,
                             cameraGain=int(round(pick[0])),
                             cameraExposureRaw=float(pick[1]))
        await asyncio.sleep(args.settle)
        # cameraSettings lags a second or two, so a plain read here reports the
        # PREVIOUS value and cries MISMATCH on a write that actually succeeded.
        live = await (pv.cameras_fresh(timeout=10) if hasattr(pv, "cameras_fresh")
                      else pv.cameras(timeout=8))
        got = next((c for c in live if c["uniqueName"] == cam["uniqueName"]), None)
        if got:
            gs = got["settings"]
            ok = (abs(float(gs["cameraGain"]) - round(pick[0])) < 0.51
                  and abs(float(gs["cameraExposureRaw"]) - pick[1]) < 1e-6)
            log("   applied gain %.0f / exposure %.0f - camera reports %s / %s  %s"
                % (pick[0], pick[1], gs["cameraGain"], gs["cameraExposureRaw"],
                   "confirmed" if ok else "*** MISMATCH ***"))
            if not ok:
                pick[3]["apply_mismatch"] = {"wanted": [pick[0], pick[1]],
                                             "got": [gs["cameraGain"], gs["cameraExposureRaw"]]}
        pick[3]["applied"] = pick[1]
        pick[3]["gain"] = pick[0]
    return pick[3]


async def run(args, log=print, progress=None, on_camera=None):
    args._nt_we_started = False
    args._nt_cfg = None
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
                await set_nt_server(args.host, cfg, True, args.port)
                args._nt_cfg = cfg
                args._nt_we_started = True
    try:
        return await _run_inner(args, log, progress, on_camera)
    finally:
        if args._nt_we_started:
            log("stopping the NetworkTables server we started")
            await set_nt_server(args.host, args._nt_cfg, False, args.port)


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


async def _tune_all(pv, cams, args, log, progress, on_camera, results):
        for idx, cam in enumerate(cams):   # sequential: avoids cameras perturbing each other
            def cam_progress(f, idx=idx):
                if progress:
                    progress((idx + f) / float(len(cams)))
            if on_camera:
                on_camera(cam["nickname"], idx, len(cams))
            if args.optimise_gain:
                # Structural settings BEFORE the gain search, not inside it.
                # optimise_gain runs tune_camera with its logging discarded, so a
                # baseline failure was both invisible and too late: measured, a
                # camera left at inputImageRotationMode=1 made all six gain trials
                # report "no passing exposure" - six full sweeps burned against a
                # camera that was broken in a way the tool already knew how to fix,
                # and the fix only landed afterwards in the fallback path.
                _prob = calibration_problem(cam)
                if _prob:
                    log("── %s ──" % cam["nickname"])
                    log("   !! CALIBRATION: %s" % _prob)
                    results.append({"camera": cam["nickname"],
                                    "uniqueName": cam["uniqueName"],
                                    "applied": None,
                                    "error": "no calibration for the active resolution",
                                    "calibration_problem": _prob})
                    continue
                if getattr(args, "baseline", True):
                    _ch, _fl = await assert_baseline(pv, cam, args, log)
                    if _ch:
                        fresh = await pv.cameras_fresh(timeout=8)
                        newer = next((c for c in fresh
                                      if c["uniqueName"] == cam["uniqueName"]), None)
                        if newer:
                            cam = dict(cam)
                            cam["settings"] = newer["settings"]
                    if _fl:
                        log("   !! baseline did not fully apply: %s"
                            % ", ".join(str(f[0]) for f in _fl))
                        log("   !! tuning on top of settings that are still wrong")
                r = await optimise_gain(pv, cam, args, log, cam_progress,
                                        skip_baseline=True)
                if r is not None:
                    results.append(r)
                    continue
            # Held-card mode: the card is only visible to one camera at a time,
            # so give whoever is holding it a chance to move before we sweep.
            if args.reference_tag is not None and idx > 0 and args.move_pause > 0:
                log("   move the card to '%s' - %.0fs" % (cam["nickname"], args.move_pause))
                await asyncio.sleep(args.move_pause)
            results.append(await tune_camera(pv, cam, args, log, cam_progress))
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
    """Put back anything a crash or Ctrl-C left changed. Safe to call twice."""
    if not PENDING_RESTORE:
        return []
    done = []
    async with Photon(host, port) as pv:
        for unique, original in list(PENDING_RESTORE.items()):
            try:
                await pv.set_setting(unique, **original)
                await asyncio.sleep(0.5)
                done.append((unique, original))
                PENDING_RESTORE.pop(unique, None)
            except Exception:
                pass
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


async def set_nt_server(host, cfg, enabled, port=5800):
    """Turn PhotonVision's own NT server on or off, live.

    Applied by NetworkTablesManager.setConfig() without restarting PhotonVision.
    Send the WHOLE config with one field changed - the endpoint also calls
    NetworkManager.reinitialize(), and with shouldManage=true a partial body
    would let Jackson default-fill fields and could take the coprocessor off the
    network. Verified: posting the config back unchanged is a clean no-op.

    The HTTP connection drops without a response while the network stack bounces,
    so a failed read is expected and is NOT an error.
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
        except Exception:
            pass                  # connection dropped as the stack restarts
    await asyncio.get_event_loop().run_in_executor(None, _post)
    await asyncio.sleep(4.0)


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

    def collect(self, seconds, ref_tag=None, after_capture=None):
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
        # ntcore's readQueue() returned nothing here regardless of pollStorage,
        # so poll and dedupe on sequenceID. At 500 Hz nothing is missed at 24 ms.
        while time.time() < end:
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
                    applied = [r for r in res if r.get("applied")]
                    failed = [r for r in res if not r.get("applied")]
                    line = "; ".join(
                        "%s=%.0f%s" % (r["camera"], r["applied"], " (fallback)" if r.get("fellback") else "")
                        for r in applied)
                    if failed:
                        line += ("; FAILED: " + ", ".join(
                            "%s (%s)" % (r["camera"], r.get("error", "?")) for r in failed))
                    every_camera_ok = bool(applied) and not failed
                    run_ok["value"] = every_camera_ok
                    run_ok["summary"] = line or "no cameras tuned"
                    ok_entry.setBoolean(every_camera_ok)
                    summary.setString(line or "no cameras tuned")
                    status.setString(("DONE - " if every_camera_ok else "DONE WITH ERRORS - ") + line)
                    result_entry.setString(json.dumps(res))
                    progress_e.setDouble(1.0)
                    log("done: " + line)
                except Exception as exc:
                    run_ok["value"] = False
                    run_ok["summary"] = "error: %s" % exc
                    ok_entry.setBoolean(False)
                    summary.setString("error: %s" % exc)
                    status.setString("error: %s" % exc)
                    log("error: %s" % exc)
                    # The daemon is the DEPLOYED mode, and it never replayed the
                    # restore - so a tune that died mid-sweep left the camera at
                    # whatever exposure was being tested until a human noticed.
                    try:
                        for unique, orig in await restore_pending(args.host, args.port):
                            log("restored %s to %s" % (unique, orig))
                    except Exception as rexc:
                        log("restore after error also failed: %s" % rexc)
                finally:
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
        except (ConnectionError, LookupError) as exc:
            sys.exit("photontune: %s" % exc)
        return
    if args.daemon:
        try:
            asyncio.run(daemon(args))
        except (KeyboardInterrupt, Terminated):
            # systemd stops the daemon with SIGTERM, and the daemon is the
            # DEPLOYED mode. Without this the signal unwound to the top with a
            # traceback and, worse, any camera left mid-sweep stayed there.
            print("photontune: stopping - restoring any camera left mid-tune")
            try:
                for unique, orig in asyncio.run(restore_pending(args.host, args.port)):
                    print("  restored %s to %s" % (unique, orig))
            except Exception as exc:
                print("  restore failed: %s" % exc)
            # and never leave PhotonVision's NT server running behind us
            try:
                if getattr(args, "_nt_we_started", False):
                    cfg = asyncio.run(read_network_config(args.host, args.port))
                    if cfg:
                        asyncio.run(set_nt_server(args.host, cfg, False, args.port))
                        print("  turned PhotonVision's NT server back off")
            except Exception:
                pass
            sys.exit(0)
        return
    t0 = time.time()
    try:
        results = asyncio.run(run(args))
    except (ConnectionError, LookupError) as exc:
        sys.exit("photontune: %s" % exc)
    except (KeyboardInterrupt, Terminated):
        # The interrupted loop could not complete the restore. Do it on a new one.
        if PENDING_RESTORE:
            print("\ninterrupted - putting camera settings back...")
            try:
                for unique, orig in asyncio.run(restore_pending(args.host, args.port)):
                    print("  restored %s -> exposure %s"
                          % (unique[:8], orig.get("cameraExposureRaw")))
            except Exception as exc:
                print("  !! COULD NOT RESTORE (%s). Check the exposure by hand." % exc)
        sys.exit(130)
    if args.json:
        print(json.dumps(results, indent=2))
    else:
        print("\ncompleted in %.0f s" % (time.time() - t0))
        for r in results:
            if r.get("applied"):
                print("  %-14s exposure -> %.0f%s" % (r["camera"], r["applied"],
                                                      "  (fallback)" if r.get("fellback") else ""))
            else:
                print("  %-14s unchanged (%s)" % (r["camera"], r.get("error", "dry run")))
    if not args.dry_run:
        # --baseline-only never sets `applied` by design, so "no applied" cannot
        # mean failure in that mode. A baseline that did NOT take, or a rejected
        # setting, MUST mean failure in every mode - previously both were recorded
        # and never consulted, so the tool reported success with a 90-degree
        # rotated image and a gain that never applied.
        def _bad(r):
            if r.get("calibration_problem") or r.get("baseline_failed") \
                    or r.get("setting_rejected"):
                return True
            if getattr(args, "baseline_only", False):
                return bool(r.get("error")) and "baseline only" not in str(r.get("error"))
            return not r.get("applied")
        failed = [r for r in results if _bad(r)]
        if failed:
            for r in failed:
                why = (r.get("calibration_problem") or r.get("error")
                       or ("baseline did not apply: %s" % r.get("baseline_failed")))
                print("FAILED %s: %s" % (r.get("camera"), why))
        sys.exit(1 if (failed or not results) else 0)


if __name__ == "__main__":
    main()
