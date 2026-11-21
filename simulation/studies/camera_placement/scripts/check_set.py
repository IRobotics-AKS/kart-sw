"""Score a camera set on a course and report gaps and how much of each image the kart fills.

  python simulation/studies/camera_placement/scripts/check_set.py                         # the recommended crossbar set on Purdue
  python simulation/studies/camera_placement/scripts/check_set.py --set nose --track purdue
  python simulation/studies/camera_placement/scripts/check_set.py --custom "front=0.05,0,0.75,0,6;left=0.05,0.3,0.75,60,25;right=0.05,-0.3,0.75,-60,25"
  python simulation/studies/camera_placement/scripts/check_set.py --worst-corners         # all three cameras at the worst corners of their boxes
"""
import argparse
import itertools
import json

from sim.camplace import track
from sim.camplace.config import BOXES, SETS
from sim.camplace.score import Study


def parse_custom(s):
    out = {}
    for part in s.split(";"):
        k, v = part.split("=")
        out[k.strip()] = tuple(float(x) for x in v.split(","))
    return out


def report(name, r):
    b = r["blocked"]
    print(f"{name}: score {r['score']:.4f}, edge seen {r['seen'] * 100:.1f}%, "
          f"beside the kart {r['mean']['beside'] * 100:.0f}% avg / {r['p10']['beside'] * 100:.0f}% worst-10%, "
          f"longest gap 0-8 m ahead {r['gap8_p95']:.2f} m (p95) / {r['gap8_max']:.2f} m (max), "
          f"kart in image front {b['front'] * 100:.0f}% left {b['left'] * 100:.0f}% right {b['right'] * 100:.0f}%"
          f"{'' if r['passes_limits'] else '  [FAILS A LIMIT]'}")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--track", default="purdue")
    ap.add_argument("--set", default="crossbar", choices=sorted(SETS))
    ap.add_argument("--custom", default=None, help="front=x,y,z,yaw,pitch;left=...;right=...")
    ap.add_argument("--worst-corners", action="store_true")
    ap.add_argument("--json", default=None, help="write the result here")
    args = ap.parse_args()
    st = Study(track.load(args.track))
    cfgs = parse_custom(args.custom) if args.custom else SETS[args.set]
    r = st.evaluate(cfgs)
    report(args.custom and "custom" or args.set, r)
    if args.worst_corners:
        # Every camera at a corner of its box (x, y, z), aimed with the set's angles.
        corners = {w: list(itertools.product(BOXES[w][0:2], BOXES[w][2:4], BOXES[w][4:6])) for w in BOXES}
        worst = None
        cache = {}
        for combo in itertools.product(*(corners[w] for w in ("front", "left", "right"))):
            cfg = {w: tuple(p) + tuple(cfgs[w][3:]) for w, p in zip(("front", "left", "right"), combo)}
            key = tuple(sorted(cfg.items()))
            qs = []
            for w, c in cfg.items():
                if (w, c) not in cache:
                    cache[(w, c)] = st.cam_quality(w, c)
                qs.append(cache[(w, c)])
            import numpy as np
            s = st.set_score(np.maximum.reduce(qs))[0]
            if worst is None or s < worst[0]:
                worst = (s, cfg)
        print(f"worst corners: score {worst[0]:.4f} ({worst[0] / r['score'] * 100:.1f}% of this set) at {worst[1]}")
        r["worst_corners"] = dict(score=worst[0], cfgs=worst[1])
    if args.json:
        with open(args.json, "w") as fh:
            json.dump(r, fh, indent=1, default=float)


if __name__ == "__main__":
    main()
