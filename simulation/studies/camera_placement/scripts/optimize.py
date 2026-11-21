"""Joint optimisation of the three-camera set, then the mounting boxes.

Each camera is swept on a coarse grid with the other two fixed, in turn (left, right, front),
until no camera's best setting moves. Then each camera gets a fine grid around its good region.
A camera's box is the largest block of fine-grid positions that all score within GOOD of the best.

  python simulation/studies/camera_placement/scripts/optimize.py                     # Purdue, about 30 min with 6 workers
  python simulation/studies/camera_placement/scripts/optimize.py --track kart_s101 --out runs/kart_s101.json
"""
import argparse
import json
import os

from sim.camplace import track
from sim.camplace.config import CLEAR_MIN, GOOD, OCC_MAX, ROOT, SETS, Z_MAX
from sim.camplace.score import Study
from sim.camplace.search import best_per_position, largest_box, sweep


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--track", default="purdue")
    ap.add_argument("--workers", type=int, default=6)
    ap.add_argument("--rounds", type=int, default=4)
    ap.add_argument("--start", default="previous", choices=sorted(SETS), help="starting set")
    ap.add_argument("--out", default=os.path.join(ROOT, "runs", "optimize_purdue.json"))
    args = ap.parse_args()
    os.makedirs(os.path.dirname(args.out), exist_ok=True)
    st = Study(track.load(args.track))
    cfgs = dict(SETS[args.start])
    history, final_rows = [], {}
    for rnd in range(1, args.rounds + 1):
        moved = False
        for which in ("left", "right", "front"):
            rows, base = sweep(st, args.track, which, cfgs, args.workers)
            top = max(best_per_position(rows).values(), key=lambda r: r["score"])
            new = (top["x"], top["y"], top["z"], top["yaw"], top["pitch"])
            print("round %d %s: best %s score %.4f (other two alone %.4f), was %s"
                  % (rnd, which, new, top["score"], base, cfgs[which]), flush=True)
            moved |= any(abs(a - b) > 1e-6 for a, b in zip(new, cfgs[which]))
            cfgs[which], final_rows[which] = new, rows
            history.append(dict(round=rnd, cam=which, best=new, score=top["score"], others_alone=base))
        if not moved:
            break
    for which in ("left", "right", "front"):
        best = best_per_position(final_rows[which])
        top = max(r["score"] for r in best.values())
        gd = [r for r in best.values() if r["score"] >= 0.95 * top]
        step = {"x": 0.1, "y": 0.05, "z": 0.1 if which != "front" else 0.05}
        env = {a: (min(r[a] for r in gd) - step[a], max(r[a] for r in gd) + step[a]) for a in ("x", "y", "z")}
        rows, base = sweep(st, args.track, which, cfgs, args.workers, fine=True, env=env)
        t = max(best_per_position(rows).values(), key=lambda r: r["score"])
        cfgs[which] = (t["x"], t["y"], t["z"], t["yaw"], t["pitch"])
        print("fine %s: best %s score %.4f" % (which, cfgs[which], t["score"]), flush=True)
        final_rows[which] = rows
        history.append(dict(round="fine", cam=which, best=cfgs[which], score=t["score"], others_alone=base))
    summary = {"track": args.track, "cfgs": cfgs, "history": history,
               "limits": dict(z_max=Z_MAX, clear_min=CLEAR_MIN, occ_max=OCC_MAX, good=GOOD)}
    for which, rows in final_rows.items():
        best = best_per_position(rows)
        top = max(r["score"] for r in best.values())
        thr = GOOD * top
        gd = [r for r in best.values() if r["score"] >= thr]
        env = {a: (min(r[a] for r in gd), max(r[a] for r in gd)) for a in ("x", "y", "z")}
        box, vol = largest_box(best, thr)
        inbox = [r for r in gd if box[0] <= r["x"] <= box[1] and min(box[2:4]) <= r["y"] <= max(box[2:4])
                 and box[4] <= r["z"] <= box[5]]
        ori = {a: (min(r[a] for r in inbox), max(r[a] for r in inbox)) for a in ("yaw", "pitch")}
        rej = {}
        for r in rows:
            if not r.get("valid"):
                rej[r["why"]] = rej.get(r["why"], 0) + 1
        summary[which] = dict(top=top, thr=thr, n_good=len(gd), n_pos=len(best), envelope=env, box=box,
                              box_volume_l=vol * 1000, box_orient=ori, rejected=rej,
                              cells=[{k: r[k] for k in ("x", "y", "z", "yaw", "pitch", "score", "blocked")}
                                     for r in best.values()])
        print(f"{which}: best {top:.4f}, {len(gd)}/{len(best)} positions within {GOOD:.0%}; envelope {env}; "
              f"largest all-good box {box} ({vol * 1000:.1f} L), angles {ori}; rejected {rej}")
    with open(args.out, "w") as fh:
        json.dump(summary, fh, default=float)
    print("wrote", args.out)


if __name__ == "__main__":
    main()
