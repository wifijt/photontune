"""Bound the real motion-blur tolerance without anyone waving a tag.

The budget rests on a Gaussian-blur sweep and two conversions, and NEITHER
conversion has been measured:

  1. WHAT UNITS IS THE SETTING IN? PhotonVision's `blur` is the AprilTag
     detector's quad_sigma. If upstream applies it AFTER quad_decimate, a
     sigma of 1.0 at the default decimate=2 is 2.0 FULL-RESOLUTION pixels,
     and every number derived from the sweep is out by a factor of 2.
  2. IS TOLERANCE ABSOLUTE OR RELATIVE? The budget is a constant number of
     pixels. That is only right if a tag twice as big in the image tolerates
     the same absolute blur. If tolerance is instead a fixed fraction of the
     tag, the budget should scale with range and currently does not.

Both are answered by one sweep, because the four combinations predict four
different signatures. With s = tag side in full-res px and d = decimate:

    units      tolerance   sigma50 vs s     sigma50(d=1)/sigma50(d=2)
    decimated  absolute    flat             1.0
    full-res   absolute    flat             0.5
    decimated  relative    proportional     2.0
    full-res   relative    proportional     1.0

The lever for column 1 is that the six tags in view span 45.5 to 87.3 px of
apparent side - a 1.9x spread - measured from PhotonVision's own `area`.

Nothing here moves and nobody has to be present.
"""
import asyncio, sys, time, math, json, collections
import websockets, msgpack

HOST = "127.0.0.1"
W, H = 1280, 800
SIGMAS = [float(x) for x in __import__('os').environ.get('SIGMAS','0,0.25,0.5,0.75,1.0,1.25,1.5,1.75,2.0,2.5,3.0,4.0').split(',')]
DWELL = 7.0
SETTLE = 2.0


async def cams(ws, timeout=12):
    t0 = time.time()
    while time.time() - t0 < timeout:
        raw = await asyncio.wait_for(ws.recv(), timeout=5)
        if not isinstance(raw, bytes):
            continue
        m = msgpack.unpackb(raw, raw=False)
        if isinstance(m, dict) and m.get("cameraSettings"):
            return {c["uniqueName"]: c for c in m["cameraSettings"]}
    return {}


async def setting(ws, uid, **kw):
    p = dict(kw); p["cameraUniqueName"] = uid
    await ws.send(msgpack.packb({"changePipelineSetting": p}))
    await asyncio.sleep(0.3)


async def measure(ws, seconds):
    """Per (camera, fiducialId) detection counts, plus frames and area."""
    frames = collections.Counter()
    hits = collections.defaultdict(int)
    area = collections.defaultdict(float)
    solves = collections.Counter()
    t0 = time.time()
    while time.time() - t0 < seconds:
        try:
            raw = await asyncio.wait_for(ws.recv(), timeout=3)
        except asyncio.TimeoutError:
            break
        if not isinstance(raw, bytes):
            continue
        m = msgpack.unpackb(raw, raw=False)
        r = (m or {}).get("updatePipelineResult") if isinstance(m, dict) else None
        if not r:
            continue
        for u, c in r.items():
            frames[u] += 1
            if c.get("multitagResult"):
                solves[u] += 1
            for t in (c.get("targets") or []):
                k = (u, t.get("fiducialId"))
                hits[k] += 1
                area[k] += t.get("area", 0.0)
    return frames, hits, area, solves


async def main():
    decimates = [int(x) for x in sys.argv[1].split(",")] if len(sys.argv) > 1 else [2, 1]
    async with websockets.connect("ws://%s:5800/websocket_data" % HOST,
                                  max_size=80_000_000, open_timeout=10,
                                  ping_interval=None) as ws:
        cs = await cams(ws)
        uids = list(cs)
        orig = {u: {"blur": cs[u]["currentPipelineSettings"].get("blur"),
                    "decimate": cs[u]["currentPipelineSettings"].get("decimate")}
                for u in uids}
        nick = {u: cs[u].get("nickname") for u in uids}
        print("cameras: %s" % ", ".join("%s(%s)" % (nick[u], u[:8]) for u in uids))
        print("original blur/decimate: %s" % json.dumps(
            {nick[u]: orig[u] for u in uids}))
        rows = []
        try:
            for dec in decimates:
                for u in uids:
                    await setting(ws, u, decimate=int(dec), blur=0.0)
                await asyncio.sleep(SETTLE)
                for sg in SIGMAS:
                    for u in uids:
                        await setting(ws, u, blur=float(sg))
                    await asyncio.sleep(SETTLE)
                    f, h, a, s = await measure(ws, DWELL)
                    for (u, fid), n in sorted(h.items()):
                        rows.append({"dec": dec, "sigma": sg, "cam": nick[u],
                                     "fid": fid, "frames": f[u], "hits": n,
                                     "area": a[(u, fid)] / max(1, n)})
                    for u in uids:
                        rows.append({"dec": dec, "sigma": sg, "cam": nick[u],
                                     "fid": "MULTITAG", "frames": f[u],
                                     "hits": s[u], "area": 0.0})
                    print("  dec=%d sigma=%-5.2f  %s" % (dec, sg, "  ".join(
                        "%s:%d/%d f=%d" % (nick[u], s[u], f[u], f[u]) for u in uids)))
        finally:
            for u in uids:
                await setting(ws, u, blur=float(orig[u]["blur"] or 0.0),
                              decimate=int(orig[u]["decimate"] or 2))
            await asyncio.sleep(2)
            back = await cams(ws)
            print("restored: %s" % json.dumps(
                {nick[u]: {"blur": back[u]["currentPipelineSettings"].get("blur"),
                           "decimate": back[u]["currentPipelineSettings"].get("decimate")}
                 for u in uids if u in back}))
        json.dump(rows, open(__import__("os").environ.get("OUT","/tmp/pt/blurscale.json"), "w"), indent=1)
        print("wrote /tmp/pt/blurscale.json  (%d rows)" % len(rows))

asyncio.run(main())
