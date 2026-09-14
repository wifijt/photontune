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


def choose_exposure(samples, bias, min_tags, max_ambiguity, ref_tag=None,
                    tag_fraction=0.85):
    """Estimate where detection actually fails, then sit a safety factor above it.

    Biasing off the shortest *tested* passing value double-counts margin: with a
    geometric sweep, adjacent points differ by the step ratio, so that value is
    already up to one full step above the true cliff. Instead we bracket the cliff
    between the highest failing and lowest passing sample and take the geometric
    mean, which is the best single estimate from a log-spaced sweep.

    Returns (chosen, shortest_passing_sample, cliff_estimate).
    """
    # How many tags did the BEST exposure in this sweep see? Anything much below
    # that is leaving detections on the table.
    best_tags = max((s.mean_tags for s in samples), default=0.0)
    tag_target = best_tags * tag_fraction if (best_tags >= 2 and ref_tag is None) else None
    passing = [s for s in samples
               if s.passes(min_tags, max_ambiguity, ref_tag, tag_target)]
    if not passing:
        return None, None, None
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
            self.ws = await websockets.connect(self.uri, open_timeout=10, max_size=80_000_000)
        except Exception as exc:
            raise ConnectionError(
                "could not reach PhotonVision at %s - check the host and that it is running (%s)"
                % (self.uri, type(exc).__name__)) from None
        return self

    async def __aexit__(self, *a):
        if self.ws:
            await self.ws.close()

    async def _pump(self, seconds, on_message):
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

    async def cameras(self, timeout=12):
        """[{uniqueName, nickname, settings}] - PhotonVision may have several."""
        found = {}
        def handle(msg):
            for cam in msg.get("cameraSettings", []) or []:
                found[cam["uniqueName"]] = {
                    "uniqueName": cam["uniqueName"],
                    "nickname": cam.get("nickname", "?"),
                    "settings": cam["currentPipelineSettings"],
                }
        await self._pump(timeout, handle)
        return list(found.values())

    # PhotonVision SILENTLY DISCARDS a float sent to an integer-typed setting.
    # No error, no log line - the write simply does not happen. Coerce them.
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

    async def set_and_verify(self, unique_name, settle=1.5, **kw):
        """Write settings and confirm the camera took them. Returns list of failures."""
        await self.set_setting(unique_name, **kw)
        await asyncio.sleep(settle)
        cams = await self.cameras(timeout=8)
        got = next((c for c in cams if c["uniqueName"] == unique_name), None)
        if not got:
            return [("<camera>", None, None)]
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

async def tune_camera(pv, cam, args, log=print, progress=None):
    name = cam["nickname"]
    unique = cam["uniqueName"]
    original = {
        "cameraExposureRaw": cam["settings"]["cameraExposureRaw"],
        "cameraGain": cam["settings"]["cameraGain"],
        "cameraAutoExposure": cam["settings"]["cameraAutoExposure"],
    }
    log("── %s ──  current exposure %s, gain %s" % (name, original["cameraExposureRaw"], original["cameraGain"]))

    gain = args.gain if args.gain is not None else original["cameraGain"]
    result = {"camera": name, "uniqueName": unique, "original": original,
              "applied": None, "samples": [], "cliff": None, "gain": gain}

    try:
        bad = await pv.set_and_verify(unique, settle=max(args.settle, 1.5),
                                      cameraAutoExposure=False, cameraGain=gain)
        if bad:
            for k, want, have in bad:
                log("   WARNING: %s did not take (wanted %s, camera has %s)" % (k, want, have))
            log("   the sweep below is NOT at the gain it claims - results are unreliable")
            result["setting_rejected"] = [list(map(str, b)) for b in bad]

        samples = []
        sweep = geometric_sweep(args.min_exposure, args.max_exposure, args.steps)
        for step_i, exposure in enumerate(sweep):
            if progress:
                progress(step_i / float(len(sweep)))
            await pv.set_setting(unique, cameraExposureRaw=float(exposure))
            await asyncio.sleep(args.settle)
            raw = await pv.collect(unique, args.dwell, args.reference_tag)

            s = Sample(exposure)
            s.frames = raw["frames"]
            s.multitag_solves = raw["solves"]
            s.reproj = raw["reproj"]
            s.tag_counts = raw["tags"]
            s.ambiguities = raw["amb"]
            s.ref_seen = raw["ref_seen"]
            s.ref_ranges = raw["ref_ranges"]
            samples.append(s)
            mark = "ok " if s.passes(args.min_tags, args.max_ambiguity, args.reference_tag) else "   "
            log("   %s exposure %8.0f   %s" % (mark, exposure, s.summary(args.reference_tag)))

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
                return await tune_camera(pv, cam, args, log, progress)
            log("   NOTHING PASSED even at gain %g." % gain)
            log("   That is a LIGHTING or CONFIG problem, not an exposure one:")
            log("     - check cameraBrightness (a low value crushes the image to black)")
            log("     - check the camera is actually pointed at tags")
            log("     - add light, or raise --max-gain")
            await pv.set_setting(unique, **original)
            result["error"] = "no passing exposure even at max gain"
            return result

        result["cliff"] = cliff
        log("   cliff ~%.0f (bracketed), shortest verified pass %.0f, bias %.2f  ->  %.0f"
            % (cliff, shortest.exposure, bias, chosen))
        blur = lambda deg: math.radians(deg) * (chosen / 1e6) * args.fx
        log("   predicted blur: %.1f px @90deg/s, %.1f px @360deg/s (tag edge ~70 px)"
            % (blur(90), blur(360)))

        if args.dry_run:
            log("   dry run - restoring original")
            await pv.set_setting(unique, **original)
        else:
            await pv.set_setting(unique, cameraExposureRaw=float(chosen), cameraGain=gain,
                                 cameraAutoExposure=False)
            await asyncio.sleep(args.settle)
            raw = await pv.collect(unique, args.dwell, args.reference_tag)
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
        return result

    except Exception as exc:
        log("   ERROR (%s) - restoring original settings" % exc)
        try:
            await pv.set_setting(unique, **original)
        except Exception:
            pass
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


async def optimise_gain(pv, cam, args, log=print, progress=None):
    """Find the (gain, exposure) pair giving the SHORTEST exposure that still sees
    all the tags - because exposure costs motion blur and gain only costs noise.

    Sweeping exposure at a fixed gain answers "what is the shortest exposure at THIS
    gain", which is not the question. Raising gain lowers the exposure needed; the
    limit is that noise eventually degrades corner precision, so we stop when
    reprojection error starts getting worse rather than simply maximising gain.
    """
    start = args.gain if args.gain is not None else cam["settings"]["cameraGain"]
    gains, g = [], float(start)
    while len(gains) < args.gain_search_steps and g <= args.max_gain:
        gains.append(g)
        g = max(g * 1.7, g + 8)
    log("── %s ──  gain search over %s" % (cam["nickname"], [round(x) for x in gains]))

    trials = []
    saved_steps, args.gain_steps = args.gain_steps, 0      # no escalation inside a trial
    for i, gv in enumerate(gains):
        args.gain = gv
        if progress:
            progress(i / float(len(gains)))
        r = await tune_camera(pv, cam, args, log=lambda m: None, progress=None)
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

    if not trials:
        args.gain = start
        return None
    good = [t for t in trials if t[2] is not None]
    best_rp = min((t[2] for t in good), default=None)
    if best_rp is not None:
        # Reject gains where noise has visibly hurt the fit.
        usable = [t for t in trials if t[2] is None or t[2] <= best_rp * args.reproj_tolerance]
    else:
        usable = trials
    pick = min(usable, key=lambda t: t[1])
    log("   chose gain %.0f with exposure %.0f (shortest exposure with acceptable reproj)"
        % (pick[0], pick[1]))
    args.gain = pick[0]

    # Explicitly apply the winner. The trials leave whichever pair was tried LAST
    # on the camera, which is only the winner by luck - so write it, then confirm
    # the camera actually took it rather than assuming.
    if not args.dry_run:
        await pv.set_setting(cam["uniqueName"], cameraAutoExposure=False,
                             cameraGain=pick[0], cameraExposureRaw=float(pick[1]))
        await asyncio.sleep(args.settle)
        live = await pv.cameras(timeout=8)
        got = next((c for c in live if c["uniqueName"] == cam["uniqueName"]), None)
        if got:
            gs = got["settings"]
            ok = (abs(float(gs["cameraGain"]) - pick[0]) < 1e-6
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
        results = []
        for idx, cam in enumerate(cams):   # sequential: avoids cameras perturbing each other
            def cam_progress(f, idx=idx):
                if progress:
                    progress((idx + f) / float(len(cams)))
            if on_camera:
                on_camera(cam["nickname"], idx, len(cams))
            if args.optimise_gain:
                r = await optimise_gain(pv, cam, args, log, cam_progress)
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
    ref_tag_e.setDefaultDouble(-1 if args.reference_tag is None else args.reference_tag)
    ref_range_e.setDefaultDouble(args.reference_range or 0.0)
    hold_e.setBoolean(False)
    status.setString("idle")
    summary.setString("never run")
    hold_e.setBoolean(False)
    busy.setBoolean(False)
    ok_entry.setBoolean(False)
    progress_e.setDouble(0.0)

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
                # Read held-card settings written by the dashboard.
                nt_tag = int(ref_tag_e.getDouble(-1))
                nt_range = ref_range_e.getDouble(0.0)
                args.reference_tag = nt_tag if nt_tag >= 0 else None
                args.reference_range = nt_range if nt_range > 0 else None

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
                    ok_entry.setBoolean(every_camera_ok)
                    summary.setString(line or "no cameras tuned")
                    status.setString(("DONE - " if every_camera_ok else "DONE WITH ERRORS - ") + line)
                    result_entry.setString(json.dumps(res))
                    progress_e.setDouble(1.0)
                    log("done: " + line)
                except Exception as exc:
                    ok_entry.setBoolean(False)
                    summary.setString("error: %s" % exc)
                    status.setString("error: %s" % exc)
                    log("error: %s" % exc)
                finally:
                    busy.setBoolean(False)
                    hold_e.setBoolean(False)
                    hold_for_e.setString("")
                    camera_e.setString("")
                    run_entry.setBoolean(False)
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
    p.add_argument("--gain", type=float, default=None, help="starting gain (default: leave as-is)")
    p.add_argument("--gain-steps", type=int, default=3,
                   help="how many times to escalate gain if no exposure passes (0 = never)")
    p.add_argument("--max-gain", type=float, default=100.0,
                   help="ceiling for gain escalation")
    p.add_argument("--optimise-gain", "--optimize-gain", dest="optimise_gain",
                   action="store_true",
                   help="search gain AND exposure together for the shortest exposure that "
                        "still sees all the tags (slower, but exposure is the costly one)")
    p.add_argument("--gain-search-steps", type=int, default=4,
                   help="how many gain values to try with --optimise-gain")
    p.add_argument("--reproj-tolerance", type=float, default=1.5,
                   help="reject a gain whose reprojection exceeds this multiple of the "
                        "best seen - stops noise being traded for exposure indefinitely")
    p.add_argument("--dwell", type=float, default=2.5, help="seconds of data per candidate")
    p.add_argument("--settle", type=float, default=1.5, help="seconds to wait after changing a setting")
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
    return p


def main():
    args = build_parser().parse_args()
    if args.min_exposure <= 0 or args.max_exposure <= args.min_exposure:
        sys.exit("--max-exposure must exceed --min-exposure, both > 0")
    if args.calibrate_reference_bias:
        try:
            asyncio.run(calibrate_reference_bias(args))
        except (ConnectionError, LookupError) as exc:
            sys.exit("photontune: %s" % exc)
        return
    if args.daemon:
        asyncio.run(daemon(args))
        return
    t0 = time.time()
    try:
        results = asyncio.run(run(args))
    except (ConnectionError, LookupError) as exc:
        sys.exit("photontune: %s" % exc)
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
        failed = [r for r in results if not r.get("applied")]
        sys.exit(1 if (failed or not results) else 0)


if __name__ == "__main__":
    main()
