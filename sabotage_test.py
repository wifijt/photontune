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
import argparse, asyncio, json, subprocess, sys, time

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
    a = p.parse_args()
    a.photontune = a.photontune.split()
    sys.exit(asyncio.run(main_async(a)))


if __name__ == "__main__":
    main()
