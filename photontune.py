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

    def passes(self, min_tags, max_ambiguity, ref_tag=None):
        if self.frames < 3:
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


def choose_exposure(samples, bias, min_tags, max_ambiguity, ref_tag=None):
    """Estimate where detection actually fails, then sit a safety factor above it.

    Biasing off the shortest *tested* passing value double-counts margin: with a
    geometric sweep, adjacent points differ by the step ratio, so that value is
    already up to one full step above the true cliff. Instead we bracket the cliff
    between the highest failing and lowest passing sample and take the geometric
    mean, which is the best single estimate from a log-spaced sweep.

    Returns (chosen, shortest_passing_sample, cliff_estimate).
    """
    passing = [s for s in samples if s.passes(min_tags, max_ambiguity, ref_tag)]
    if not passing:
        return None, None, None
    shortest = min(passing, key=lambda s: s.exposure)
    failing_below = [s for s in samples
                     if s.exposure < shortest.exposure
                     and not s.passes(min_tags, max_ambiguity, ref_tag)]
    if failing_below:
        highest_fail = max(failing_below, key=lambda s: s.exposure).exposure
        cliff = math.sqrt(highest_fail * shortest.exposure)
    else:
        # Everything we tested passed - the cliff is at or below our range.
        cliff = shortest.exposure
    ceiling = max(s.exposure for s in samples)
    return min(cliff * bias, ceiling), shortest, cliff


# ───────────────────────── PhotonVision link ─────────────────────────

class Photon:
    def __init__(self, host, port=5800):
        self.uri = "ws://%s:%d/websocket_data" % (host, port)
        self.ws = None

    async def __aenter__(self):
        self.ws = await websockets.connect(self.uri, open_timeout=10, max_size=80_000_000)
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

    async def set_setting(self, unique_name, **kw):
        payload = dict(kw)
        payload["cameraUniqueName"] = unique_name
        await self.ws.send(msgpack.packb({"changePipelineSetting": payload}))

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
    result = {"camera": name, "uniqueName": unique, "original": original, "applied": None, "samples": []}

    try:
        await pv.set_setting(unique, cameraAutoExposure=False, cameraGain=gain)
        await asyncio.sleep(0.4)

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

        chosen, shortest, cliff = choose_exposure(samples, args.bias, args.min_tags,
                                                  args.max_ambiguity, args.reference_tag)
        if chosen is None:
            log("   NOTHING PASSED - restoring original. Try more light or a higher --gain.")
            await pv.set_setting(unique, **original)
            result["error"] = "no passing exposure"
            return result

        log("   cliff ~%.0f (bracketed), shortest verified pass %.0f, bias %.2f  ->  %.0f"
            % (cliff, shortest.exposure, args.bias, chosen))
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


async def run(args, log=print, progress=None):
    async with Photon(args.host, args.port) as pv:
        cams = await pv.cameras()
        if not cams:
            log("no cameras reported by PhotonVision at %s" % args.host)
            return []
        if args.cameras:
            want = {c.strip().lower() for c in args.cameras.split(",")}
            cams = [c for c in cams if c["nickname"].lower() in want or c["uniqueName"].lower() in want]
            if not cams:
                log("none of the requested cameras matched")
                return []
        log("tuning %d camera(s): %s" % (len(cams), ", ".join(c["nickname"] for c in cams)))
        results = []
        for idx, cam in enumerate(cams):   # sequential: avoids cameras perturbing each other
            def cam_progress(f, idx=idx):
                if progress:
                    progress((idx + f) / float(len(cams)))
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
    status.setString("idle")
    summary.setString("never run")
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
                busy.setBoolean(True)
                ok_entry.setBoolean(False)
                progress_e.setDouble(0.0)
                status.setString("tuning...")
                summary.setString("running...")
                lines = []
                def cap(msg):
                    lines.append(str(msg)); log(msg); status.setString(str(msg)[:120])
                try:
                    res = await run(args, log=cap, progress=lambda f: progress_e.setDouble(f))
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
    p.add_argument("--gain", type=float, default=None, help="fixed gain (default: leave as-is)")
    p.add_argument("--dwell", type=float, default=2.5, help="seconds of data per candidate")
    p.add_argument("--settle", type=float, default=1.5, help="seconds to wait after changing a setting")
    p.add_argument("--min-tags", type=float, default=2.0,
                   help="mean tags per frame required when multi-tag is unavailable")
    p.add_argument("--max-ambiguity", type=float, default=0.20)
    p.add_argument("--fx", type=float, default=1105.9, help="focal length in px, for the blur estimate")
    p.add_argument("--reference-tag", type=int, default=None,
                   help="tag ID held in front of the camera; score on its detection rate")
    p.add_argument("--reference-range", type=float, default=None,
                   help="string length in metres - the tool verifies the tag really is there")
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
    if args.daemon:
        asyncio.run(daemon(args))
        return
    t0 = time.time()
    results = asyncio.run(run(args))
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
