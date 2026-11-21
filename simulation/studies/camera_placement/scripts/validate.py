"""Check the recommended layout on other tracks.

1. Make 5 tracks with kart-sw's generator (4 compact kart circuits, 1 flowing), fixed seeds,
   350-700 m long, saved to data/tracks/generated/. Each is driven on 3 lines (centreline,
   +-40 % of the half width), since generated tracks have no racing line.
2. On each track (and Purdue), score the named sets (config.SETS).
3. Give each camera one coarse re-optimisation with the other two at the crossbar layout. The best
   score found is the reference for "% of best", and we check whether each camera's best
   position on that track falls inside the Purdue box.
4. Write results/validation.json and results/validation.md.

The layout "holds" on a track when the crossbar set is >= 95 % of best and the longest unseen
edge stretch 0-8 m ahead is <= 1 m.

  python simulation/studies/camera_placement/scripts/validate.py --workers 6
"""
import argparse
import json
import os
import time

from sim.camplace import track
from sim.camplace.config import BOXES, ROOT, SETS, TRACKS_DIR
from sim.camplace.score import Study
from sim.camplace.search import best_per_position, sweep
from sim.tracks.generator import GenerationError, Params, generate

GEN = [("kart", 101), ("kart", 102), ("kart", 103), ("kart", 104), ("flowing", 201)]


def make_tracks():
    out = []
    d = os.path.join(TRACKS_DIR, "generated")
    os.makedirs(d, exist_ok=True)
    for layout, seed in GEN:
        s = seed
        while True:
            name = f"{layout}_s{s}"
            path = os.path.join(d, name + ".json")
            if os.path.exists(path):
                break
            try:
                t = generate(seed=s, params=Params(length=(350.0, 700.0)), edges="lines", runoff="grass",
                             obstacles=0, layout=layout)
                t.save(path)
                break
            except GenerationError:
                s += 1000
        out.append(name)
    return out


def in_box(cfg, box, tol=1e-6):
    x, y, z = cfg[:3]
    return (box[0] - tol <= x <= box[1] + tol and min(box[2:4]) - tol <= y <= max(box[2:4]) + tol
            and box[4] - tol <= z <= box[5] + tol)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--workers", type=int, default=6)
    ap.add_argument("--tracks", default=None, help="comma list (default: purdue + the 5 generated)")
    ap.add_argument("--no-reopt", action="store_true", help="only score the named sets")
    args = ap.parse_args()
    names = args.tracks.split(",") if args.tracks else ["purdue"] + make_tracks()
    results = {}
    for name in names:
        t0 = time.time()
        c = track.load(name)
        st = Study(c)
        r = dict(length_m=c.length, width_min=float(c.width.min()), width_max=float(c.width.max()),
                 min_radius_m=c.min_radius, poses=int(st.P["nframes"]), lines=[n for n, _ in c.lines], sets={})
        for sname, cfgs in SETS.items():
            r["sets"][sname] = st.evaluate(cfgs)
        print(f"{name}: {c.length:.0f} m, width {r['width_min']:.1f}-{r['width_max']:.1f} m, "
              f"tightest radius {c.min_radius:.1f} m; crossbar {r['sets']['crossbar']['score']:.4f}, "
              f"crossbar65 {r['sets']['crossbar65']['score']:.4f}, nose {r['sets']['nose']['score']:.4f}", flush=True)
        if not args.no_reopt:
            base = dict(SETS["crossbar"])
            r["reopt"] = {}
            for which in ("left", "right", "front"):
                rows, _ = sweep(st, name, which, base, args.workers, log=lambda *_: None)
                top = max(best_per_position(rows).values(), key=lambda q: q["score"])
                cfg = (top["x"], top["y"], top["z"], top["yaw"], top["pitch"])
                r["reopt"][which] = dict(best=cfg, score=top["score"], in_purdue_box=in_box(cfg, BOXES[which]))
                print(f"  {which}: best {cfg} -> {top['score']:.4f}, inside the Purdue box: "
                      f"{r['reopt'][which]['in_purdue_box']}", flush=True)
            ref = max([q["score"] for q in r["reopt"].values()] + [s["score"] for s in r["sets"].values()
                                                                    if s["passes_limits"]])
        else:
            ref = max(s["score"] for s in r["sets"].values() if s["passes_limits"])
        r["best_found"] = ref
        cb = r["sets"]["crossbar"]
        r["holds"] = cb["score"] >= 0.95 * ref and cb["gap8_max"] <= 1.0
        results[name] = r
        print(f"  crossbar = {cb['score'] / ref * 100:.1f}% of best found, worst gap 0-8 m {cb['gap8_max']:.2f} m, "
              f"holds: {r['holds']}  ({time.time() - t0:.0f} s)", flush=True)
    os.makedirs(os.path.join(ROOT, "results"), exist_ok=True)
    with open(os.path.join(ROOT, "results", "validation.json"), "w") as fh:
        json.dump(results, fh, indent=1, default=float)
    write_md(results)


def write_md(results):
    L = ["# Multi-track validation", "",
         "The crossbar layout (all cameras at x 0.05 m, z 0.75 m; sides at y +-0.30 m, 60 deg out, 25 deg down; "
         "front 6 deg down) scored on Purdue and 5 tracks from kart-sw's generator. "
         "\"% of best\" compares with the best set found on that track (the named sets and one coarse "
         "re-optimisation of each camera with the other two at the crossbar). It holds when it is >= 95 % "
         "of best with no unseen edge stretch over 1 m within 8 m ahead.", "",
         "| Track | Length (m) | Width (m) | Tightest radius (m) | Crossbar % of best | 65 deg sides % of best | "
         "Nose % of best | Worst gap 0-8 m (m) | Beside the kart (avg / worst 10 %) | Holds |",
         "| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |"]
    for n, r in results.items():
        ref = r["best_found"]
        cb, c65, nose = r["sets"]["crossbar"], r["sets"]["crossbar65"], r["sets"]["nose"]
        L.append(f"| {n} | {r['length_m']:.0f} | {r['width_min']:.1f}-{r['width_max']:.1f} | {r['min_radius_m']:.1f} | "
                 f"{cb['score'] / ref * 100:.1f} | {c65['score'] / ref * 100:.1f} | {nose['score'] / ref * 100:.1f} | "
                 f"{cb['gap8_max']:.2f} | {cb['mean']['beside'] * 100:.0f} / {cb['p10']['beside'] * 100:.0f} % | "
                 f"{'yes' if r['holds'] else 'NO'} |")
    if any("reopt" in r for r in results.values()):
        L += ["", "Best position per camera when re-optimised on each track (others at the crossbar layout):", "",
              "| Track | Front (x, y, z, yaw, pitch) | In box | Left | In box | Right | In box |",
              "| --- | --- | --- | --- | --- | --- | --- |"]
        for n, r in results.items():
            if "reopt" not in r:
                continue
            cells = []
            for w in ("front", "left", "right"):
                q = r["reopt"][w]
                cells += [", ".join(f"{v:g}" for v in q["best"]), "yes" if q["in_purdue_box"] else "no"]
            L.append(f"| {n} | " + " | ".join(cells) + " |")
    with open(os.path.join(ROOT, "results", "validation.md"), "w") as fh:
        fh.write("\n".join(L) + "\n")
    print("wrote results/validation.md")


if __name__ == "__main__":
    main()
