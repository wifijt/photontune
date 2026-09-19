#!/usr/bin/env python3
#
# Copyright (C) 2026  wifijt
#
# This program is free software: you can redistribute it and/or modify it under
# the terms of the GNU General Public License as published by the Free Software
# Foundation, either version 3 of the License, or (at your option) any later
# version.  This program is distributed WITHOUT ANY WARRANTY; see the GNU
# General Public License at <https://www.gnu.org/licenses/> for details.
#
"""Break the camera on purpose, then check photontune puts it back.

    python3 sabotage_test.py                 # on the coprocessor
    python3 sabotage_test.py --host 1.2.3.4  # or remotely

photontune's stated goal includes "take fully human-tweaked settings and
optimize them - fix them if they are whacked". That is a claim about behaviour
under hostile input, and the only way to know it holds is to BE the hostile
input. Every case below is a setting a person can reach from the dashboard in
one click.

This exists because reading the code was not enough: the code LOOKED like it
asserted a baseline, and three of those settings had in fact never applied once,
because they were sent as enum names where PhotonVision requires ordinals. It
logged its own failure and carried on. Nothing but running it found that.

The harness snapshots every setting it touches and restores them at the end,
including after a failure or an interrupt.
"""
import argparse, asyncio, json, os, subprocess, sys, time

try:
    import msgpack, websockets
except ImportError:
    sys.exit("need: pip install msgpack websockets")

# setting -> (sabotage value, expected value after repair, why a human might set it)
STRUCTURAL = {
    "inputImageRotationMode": (1, 0, "camera mounted sideways, rotated the preview"),
    "targetModel": (0, 7, "picked the wrong tag size from the dropdown"),
    "tagFamily": (1, 0, "chose 16h5 - looks plausible, detects nothing on an FRC field"),
    "blur": (3.5, 0.0, "tried to 'clean up' a noisy image"),
    "decimate": (4, 2, "chased framerate"),
    "threads": (4, 1, "matched it to the core count, which is the wrong instinct"),
    "numIterations": (222, 40, "turned it up for 'accuracy'"),
    "decisionMargin": (100, 35, "raised it to reject false positives"),
    "hammingDist": (2, 0, "allowed bit errors to 'see more tags'"),
    "refineEdges": (False, True, "turned it off to save CPU"),
    "doMultiTarget": (False, True, "did not know what it did"),
    "cameraBrightness": (5, 40, "dragged the brightness slider down"),
    "cameraAutoExposure": (True, False, "auto sounds safer than manual"),
    "cameraRedGain": (50, 0, "fiddled with white balance"),
    "cameraBlueGain": (50, 0, "fiddled with white balance"),
}

# needs a full tune to repair, not just the baseline
TUNED = {
    "cameraExposureRaw": (25000.0, "exposure at maximum - blind on a moving robot"),
    "cameraGain": (100, "gain pinned at the ceiling"),
}


# ───────────────────── the verdict matrix (no hardware) ─────────────────────
#
# One sample value per registered problem. The test asserts three things:
#   1. every key in photontune.PROBLEMS has a case here - a new problem cannot
#      be added without deciding what it does to the exit code;
#   2. a HARD key makes the real main() exit 1, a WARN key exits 0;
#   3. the text of every problem appears in the output. "Nothing the tool
#      detected may vanish" is the actual requirement, and an exit code alone
#      does not check it.
#
# It runs the REAL main() in a subprocess with run() stubbed out, so what is
# asserted is the process exit status, not a helper's return value. Six failure
# modes used to be recorded and then exit 0.
MATRIX_VALUES = {
    "baseline_failed":        ("camera", [["cameraBrightness", "40", "5"]]),
    "baseline_unconfirmed":   ("camera", "no cameraSettings for 12 s"),
    "setting_rejected":       ("camera", [["cameraGain", "20", "0"]]),
    "unverified":             ("camera", "no cameraSettings for 12 s"),
    "calibration_problem":    ("camera", "no calibration for the active mode"),
    "apply_mismatch":         ("camera", {"wanted": [20, 1500], "got": [0, 1500]}),
    "apply_unconfirmed":      ("camera", "no cameraSettings after the write"),
    "final_state_wrong":      ("camera", [["cameraRedGain", 0, 50]]),
    "gain_scan_error":        ("camera", "ConnectionClosed, 4 candidates untried"),
    "video_mode_switched":    ("camera", [0, 1]),
    "would_switch_video_mode": ("camera", [0, 1]),
    "over_blur_budget":       ("camera", 17.9),
    "fellback":               ("camera", True),
    "would_baseline":         ("camera", ["numIterations", "decisionMargin"]),
}

_CHILD = r'''
import json, sys
sys.path.insert(0, sys.argv[1])
import photontune as pt
key, mode, value = sys.argv[2], sys.argv[3], json.loads(sys.argv[4])
rec = {"camera": "CAM", "uniqueName": "u", "applied": 1500.0, "gain": 0}
if mode == "camera":
    pt.note_problem(rec, key, value)
async def fake_run(args, log=print, progress=None, on_camera=None):
    args._run_problems = {}
    if mode == "run":
        pt.note_run_problem(args, key, value)
    return [rec]
pt.run = fake_run
sys.argv = ["photontune.py", "--host", "127.0.0.1"]
pt.main()
'''


# ───────────────── the gain decision, replayed (no hardware) ─────────────────
#
# The audit's three runs, same room, same light, on OV9281. It reported the two
# candidates the decision came down to:
#
#     gain 100   0.378   0.393   0.510      <- the deciding point, 35% swing
#     gain  60   0.478   0.479   0.478      <- 0.2%, stable
#     chosen      100     100      60       <- not reproducible
#
# The reason the old rule flipped is not that the scan was unlucky. It measured
# its repeat noise at the FIRST PASSING gain - gain 0, where the audit measured
# 0% - so the band floored at 5%, and a 35% swing at the gain actually being
# decided was invisible to it. In run 3 that swing put gain 100 above gain 60
# and the answer changed.
#
# This replays those numbers through the real gain_decision() and through the
# old rule, which is reproduced here from the previous revision:
#     band = max(math.exp(noise) if noise is not None else 1.0, 1.05)
#     contenders = [t for t in usable if t[1] <= best_rp * band]
#     pick = min(contenders, key=lambda t: t[0])
# with `noise` coming from a repeat of the first passing candidate.
AUDIT_RUNS = [
    # (run, {gain: reproj}, repeats of gain 100 as measured across the audit)
    ("run 1", {100: 0.378, 60: 0.478}),
    ("run 2", {100: 0.393, 60: 0.479}),
    ("run 3", {100: 0.510, 60: 0.478}),
]
AUDIT_REPEATS_AT_100 = [0.378, 0.393, 0.510]   # the swing, measured at gain 100
AUDIT_REPEATS_AT_60 = [0.478, 0.479, 0.478]    # stable
AUDIT_NOISE_AT_QUIET_GAIN = 0.0                # what the old rule measured


def _old_rule(good, noise, tolerance=1.5):
    """The previous revision's pick, reproduced verbatim from its three lines."""
    import math
    best_rp = min(rp for _g, rp in good)
    usable = [t for t in good if t[1] <= best_rp * float(tolerance)]
    band = max(math.exp(noise) if noise is not None else 1.0, 1.05)
    contenders = [t for t in usable if t[1] <= best_rp * band]
    return min(contenders, key=lambda t: t[0])


def gain_decision_replay(src_dir):
    sys.path.insert(0, src_dir)
    import photontune as pt

    print("=" * 66)
    print("GAIN DECISION  - the audit's own three runs, replayed")
    print("=" * 66)
    print("%-7s %-22s %-10s %-10s" % ("run", "measured", "old rule", "new rule"))
    print("-" * 66)
    old_picks, new_picks, ties = [], [], []
    for label, table in AUDIT_RUNS:
        good = sorted(table.items())
        old = _old_rule(good, AUDIT_NOISE_AT_QUIET_GAIN)
        # the new rule repeats whichever candidate leads, so the repeats it gets
        # are the ones the audit measured AT that gain
        leader = min(good, key=lambda t: t[1])[0]
        repeats = (AUDIT_REPEATS_AT_100 if leader == 100 else AUDIT_REPEATS_AT_60)
        pick, best, band, noise, _pairs = pt.gain_decision(good, repeats, leader, 1.5)
        old_picks.append(old[0])
        new_picks.append(pick[0])
        ties.append(pick[0] != best[0])
        print("%-7s %-22s gain %-5d gain %-5d  (band %.0f%%%s)"
              % (label, ", ".join("%d:%.3f" % (g, rp) for g, rp in good),
                 old[0], pick[0], 100 * (band - 1),
                 ", reported as a tie" if pick[0] != best[0] else ""))
    print("-" * 66)
    ok = len(set(new_picks)) == 1
    print("old rule chose %s  <- the audit's finding, reproduced"
          % " / ".join(str(g) for g in old_picks))
    print("new rule chose %s  <- %s"
          % (" / ".join(str(g) for g in new_picks),
             "the same gain every run" if ok else "STILL NOT REPRODUCIBLE"))
    if len(set(old_picks)) == 1:
        print("  !! the old rule did NOT flip here - this replay proves nothing")
        ok = False
    print("=" * 66)
    return 0 if ok else 1


def sample_floor(src_dir):
    """2 of 3 frames must not clear a 90% gate. Offline."""
    sys.path.insert(0, src_dir)
    import photontune as pt

    def s(frames, solves):
        x = pt.Sample(1500.0)
        x.frames, x.multitag_solves = frames, solves
        x.reproj = [0.5] * solves
        x.tag_counts = [4] * frames
        x.ambiguities = [0.05] * frames
        return x

    print("=" * 66)
    print("SAMPLE FLOOR  - can a tiny sample clear the 0.90 solve-rate gate?")
    print("=" * 66)
    cases = [
        # frames, solves, must_pass, note
        (3,  2, False, "the audit's case: 67% clearing a 90% gate"),
        (3,  3, False, "3/3 is still only 3 frames"),
        (8,  8, False, "reachable on --no-nt --fast"),
        (19, 19, False, "one frame under the floor"),
        (20, 20, True,  "at the floor, a clean sample passes"),
        (20, 16, True,  "80% at n=20 IS the documented effective floor"),
        (20, 15, False, "75% at the floor does not pass"),
        (60, 56, True,  "93% over 60 frames passes"),
    ]
    bad = 0
    print("%-8s %-8s %-8s %-8s %s" % ("frames", "solves", "want", "got", "why_failed"))
    print("-" * 66)
    for n, k, want, note in cases:
        x = s(n, k)
        got = x.passes(2.0, 0.20)
        ok = got == want
        bad += 0 if ok else 1
        print("%-8d %-8d %-8s %-8s %s%s"
              % (n, k, want, got, (x.why_failed(2.0, 0.20) or "-")[:34],
                 "   <-- WRONG" if not ok else ""))
    print("-" * 66)
    n_min = getattr(pt, "MIN_SAMPLE_FRAMES", None)
    if n_min is None:
        print("MIN_SAMPLE_FRAMES: not present in this revision (the floor was 3)")
        bad += 1
    else:
        print("MIN_SAMPLE_FRAMES = %d, effective floor there = %.0f%%"
              % (n_min, 100 * pt._effective_floor(n_min)))
    print("=" * 66)
    return 1 if bad else 0


def verdict_matrix(src_dir):
    sys.path.insert(0, src_dir)
    import photontune as pt

    print("=" * 66)
    print("VERDICT MATRIX  - one recorded problem at a time, real exit codes")
    print("=" * 66)
    missing = set(pt.PROBLEMS) - set(MATRIX_VALUES)
    extra = set(MATRIX_VALUES) - set(pt.PROBLEMS)
    bad = 0
    if missing:
        print("  !! PROBLEMS has keys with no matrix case: %s" % sorted(missing))
        bad += len(missing)
    if extra:
        print("  !! matrix cases for unregistered keys: %s" % sorted(extra))
        bad += len(extra)

    cases = [(k, pt.PROBLEMS[k][0], MATRIX_VALUES[k][0], MATRIX_VALUES[k][1])
             for k in sorted(set(pt.PROBLEMS) & set(MATRIX_VALUES))]
    cases.append(("(nothing recorded)", "clean", "none", None))
    print("%-26s %-6s %-6s %-6s %s" % ("problem", "sev", "want", "got", "text surfaced"))
    print("-" * 66)
    for key, sev, mode, value in cases:
        want = 1 if sev == pt.HARD else 0
        proc = subprocess.run(
            [sys.executable, "-c", _CHILD, src_dir, key, mode, json.dumps(value)],
            capture_output=True, text=True, timeout=120)
        got = proc.returncode
        out = proc.stdout + proc.stderr
        if sev == "clean":
            surfaced = True
        else:
            text = pt.PROBLEMS[key][1](value)
            # the whole sentence is often wrapped; check a distinctive slice
            surfaced = text.split(".")[0][:40] in out
        ok = (got == want) and surfaced
        bad += 0 if ok else 1
        print("%-26s %-6s %-6d %-6d %s%s"
              % (key, sev, want, got, "yes" if surfaced else "NO",
                 "" if ok else "   <-- WRONG"))
        if not ok and out:
            print("      output: %s" % out.strip().replace("\n", " | ")[:300])

    # note_problem must refuse a key nobody decided the severity of
    try:
        pt.note_problem({}, "a_problem_nobody_registered", True)
        print("  !! note_problem accepted an unregistered key")
        bad += 1
    except KeyError:
        print("  note_problem refuses an unregistered key: ok")

    print("=" * 66)
    print("verdict matrix: %d problem(s)" % bad if bad else
          "verdict matrix: all %d cases correct" % len(cases))
    return 1 if bad else 0


class PV:
    def __init__(self, host, port=5800):
        self.uri = "ws://%s:%d/websocket_data" % (host, port)
        self.ws = None

    async def __aenter__(self):
        self.ws = await websockets.connect(self.uri, open_timeout=15,
                                           ping_interval=None, max_size=80_000_000)
        return self

    async def __aexit__(self, *a):
        if self.ws:
            await self.ws.close()

    async def cameras(self, timeout=15, window=5.0):
        """Read for a WINDOW and keep the LAST value seen, not the first.

        PhotonVision's cameraSettings broadcast lags a second or two behind a
        write, so taking the first message after a change reports the PREVIOUS
        value. That made this harness report a repair as a failure: it read
        exposure 1500 immediately after setting 25000, then read 25000 after the
        repair to 1500 had already happened. Both reads were one step behind.
        """
        found, t0 = {}, time.time()
        while time.time() - t0 < min(timeout, window):
            try:
                raw = await asyncio.wait_for(self.ws.recv(), timeout=6)
            except asyncio.TimeoutError:
                break
            if not isinstance(raw, bytes):
                continue
            m = msgpack.unpackb(raw, raw=False)
            if isinstance(m, dict):
                for c in m.get("cameraSettings", []) or []:
                    found[c["uniqueName"]] = c
        return found

    async def set(self, uid, **kw):
        for k, v in kw.items():
            await self.ws.send(msgpack.packb(
                {"changePipelineSetting": {k: v, "cameraUniqueName": uid}}))
            await asyncio.sleep(0.25)


def same(a, b):
    if isinstance(b, bool):
        return bool(a) == b
    try:
        return abs(float(a) - float(b)) < 0.51
    except (TypeError, ValueError):
        return a == b


async def read_settings(host, port, nick):
    async with PV(host, port) as pv:
        cams = await pv.cameras()
        for uid, c in cams.items():
            if c.get("nickname") == nick:
                return uid, dict(c["currentPipelineSettings"])
    return None, None


async def main_async(a):
    async with PV(a.host, a.port) as pv:
        cams = await pv.cameras()
    if not cams:
        sys.exit("no cameras at %s" % a.host)
    targets = [(u, c) for u, c in cams.items()
               if not a.camera or c.get("nickname") == a.camera]
    if not targets:
        sys.exit("no camera named %r" % a.camera)
    uid, cam = targets[0]
    nick = cam.get("nickname")
    snapshot = dict(cam["currentPipelineSettings"])
    print("target camera: %s" % nick)
    print("snapshotting %d settings" % len(snapshot))
    # Do not restore a snapshot that is itself broken. This harness has been run
    # after an aborted tune left the camera at 25000 us, and faithfully putting
    # that back at the end looked exactly like photontune failing to repair it.
    exp0 = float(snapshot.get("cameraExposureRaw") or 0)
    if not (0 < exp0 < 20000):
        print("  NOTE: snapshot exposure %.0f is not sane; will restore 1500 instead"
              % exp0)
        snapshot["cameraExposureRaw"] = 1500.0
    print("")

    results = []
    try:
        # ---- structural: sabotage everything at once, one baseline run ----
        print("=" * 66)
        print("PHASE 1  structural settings, all broken at once")
        print("=" * 66)
        async with PV(a.host, a.port) as pv:
            for k, (bad, _good, _why) in STRUCTURAL.items():
                await pv.set(uid, **{k: bad})
        await asyncio.sleep(3)
        _, after_sab = await read_settings(a.host, a.port, nick)
        applied = [k for k in STRUCTURAL if not same(after_sab.get(k), STRUCTURAL[k][1])]
        print("  sabotaged %d/%d settings (the rest already differed or were refused)\n"
              % (len(applied), len(STRUCTURAL)))

        t0 = time.time()
        cmd = a.photontune + ["--cameras", nick, "--baseline-only"]
        proc = subprocess.run(cmd, capture_output=True, text=True, timeout=600)
        print("  ran %s -> exit %d in %.0f s"
              % (" ".join(cmd[-3:]), proc.returncode, time.time() - t0))

        _, after = await read_settings(a.host, a.port, nick)
        for k, (bad, good, why) in STRUCTURAL.items():
            if k not in applied:
                results.append((k, "SKIP", after.get(k), good, "sabotage did not take"))
                continue
            ok = same(after.get(k), good)
            results.append((k, "PASS" if ok else "FAIL", after.get(k), good, why))

        # ---- tuned: exposure and gain need a real tune ----
        print("\n" + "=" * 66)
        print("PHASE 2  exposure and gain, broken, repaired by a full tune")
        print("=" * 66)
        async with PV(a.host, a.port) as pv:
            for k, (bad, _why) in TUNED.items():
                await pv.set(uid, **{k: bad})
        await asyncio.sleep(3)
        _, before = await read_settings(a.host, a.port, nick)
        print("  exposure %s, gain %s" % (before.get("cameraExposureRaw"),
                                          before.get("cameraGain")))
        t0 = time.time()
        proc2 = subprocess.run(a.photontune + ["--cameras", nick],
                               capture_output=True, text=True, timeout=1800)
        dur = time.time() - t0
        print("  ran photontune -> exit %d in %.0f s" % (proc2.returncode, dur))
        _, after2 = await read_settings(a.host, a.port, nick)
        exp = float(after2.get("cameraExposureRaw") or 0)
        ok_exp = 0 < exp < 20000
        results.append(("cameraExposureRaw", "PASS" if ok_exp else "FAIL",
                        exp, "< 20000", TUNED["cameraExposureRaw"][1]))
        # gain is tuned, not asserted, so "repaired" means "chosen deliberately",
        # which we can only judge as "the tune ran and set one"
        results.append(("cameraGain", "INFO", after2.get("cameraGain"),
                        "tuned", "gain is searched, not asserted"))
        results.append(("(tune runtime)", "INFO", "%.0f s" % dur, "", ""))
    finally:
        print("\nrestoring the snapshot ...")
        async with PV(a.host, a.port) as pv:
            for k, v in snapshot.items():
                if k in STRUCTURAL or k in TUNED:
                    try:
                        await pv.set(uid, **{k: v})
                    except Exception:
                        pass
        await asyncio.sleep(2)
        _, final = await read_settings(a.host, a.port, nick)
        drift = [k for k in list(STRUCTURAL) + list(TUNED)
                 if not same(final.get(k), snapshot.get(k))]
        print("  restored; %s" % ("clean" if not drift else "STILL DIFFERENT: %s" % drift))

    print("\n" + "=" * 66)
    print("%-24s %-6s %-12s %-10s" % ("setting", "result", "now", "wanted"))
    print("=" * 66)
    npass = nfail = 0
    for k, verdict, now, want, why in results:
        if verdict == "PASS":
            npass += 1
        elif verdict == "FAIL":
            nfail += 1
        print("%-24s %-6s %-12s %-10s %s"
              % (k, verdict, str(now)[:12], str(want)[:10],
                 why if verdict in ("FAIL", "SKIP") else ""))
    print("=" * 66)
    print("%d passed, %d FAILED" % (npass, nfail))
    if nfail:
        print("\nA FAIL means a human can put the camera into that state and "
              "photontune will not fix it.")
    return 1 if nfail else 0


def main():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--host", default="127.0.0.1")
    p.add_argument("--port", type=int, default=5800)
    p.add_argument("--camera", default=None, help="nickname (default: first)")
    p.add_argument("--photontune", default="python3 /opt/photontune/photontune.py",
                   help="command that runs photontune")
    p.add_argument("--sample-floor", action="store_true",
                   help="offline: assert a sample too small to judge is refused.")
    p.add_argument("--gain-decision", action="store_true",
                   help="offline: replay the audit's three runs through the gain "
                        "decision and show it no longer flips. No hardware.")
    p.add_argument("--verdict-matrix", action="store_true",
                   help="offline: assert the exit code and the surfaced text for "
                        "every problem photontune can record. No hardware.")
    p.add_argument("--src", default=os.path.dirname(os.path.abspath(__file__)),
                   help="directory holding photontune.py, for --verdict-matrix")
    a = p.parse_args()
    if a.verdict_matrix:
        sys.exit(verdict_matrix(a.src))
    if a.gain_decision:
        sys.exit(gain_decision_replay(a.src))
    if a.sample_floor:
        sys.exit(sample_floor(a.src))
    a.photontune = a.photontune.split()
    sys.exit(asyncio.run(main_async(a)))


if __name__ == "__main__":
    main()
