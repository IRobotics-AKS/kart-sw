"""Grid search over camera positions and angles, one camera at a time with the other two fixed."""
import itertools
import time
from multiprocessing import Pool

import numpy as np

from .config import CAM_OF, CLEAR_MIN, OCC_MAX, X_MIN, Z_MAX


def rng(lo, hi, step):
    return np.round(np.arange(lo, hi + 1e-6, step), 4)


def grid(which, fine=False, env=None):
    """(positions, orientations). Coarse grid for the rounds; fine grid (optionally limited to
    env = {axis: (lo, hi)}) for the boxes."""
    if which == "front":
        ax = dict(x=rng(X_MIN, 1.4, 0.1), y=np.array([-0.10, 0.0, 0.10]), z=rng(0.40, Z_MAX, 0.05))
        if fine:
            ax.update(x=rng(X_MIN, 1.45, 0.05), z=rng(0.40, Z_MAX, 0.025))
        ori = [(0.0, p) for p in (rng(0.0, 15.0, 1.25) if fine else rng(0.0, 15.0, 2.5))]
    else:
        sg = 1 if which == "left" else -1
        ax = dict(x=rng(X_MIN, 1.0, 0.1), y=sg * rng(0.30, 0.60, 0.05), z=rng(0.30, Z_MAX, 0.1))
        if fine:
            ax.update(x=rng(X_MIN, 1.0, 0.05), z=rng(0.30, Z_MAX, 0.05))
        yaws = range(45, 81, 5) if fine else range(45, 81, 10)
        pitches = range(10, 36, 5) if fine else (10, 20, 30)
        ori = [(sg * yw, p) for yw in yaws for p in pitches]
    if env:
        for a, (lo, hi) in env.items():
            ax[a] = ax[a][(ax[a] >= min(lo, hi) - 1e-6) & (ax[a] <= max(lo, hi) + 1e-6)]
    pos = itertools.product(ax["x"], ax["y"], ax["z"])
    return [tuple(round(float(v), 4) for v in p) for p in pos], [(float(a), float(b)) for a, b in ori]


# ------------------------------------------------------------------ workers
W = {}


def _init(course_name, fixed):
    from . import track
    from .score import Study
    W["study"] = Study(track.load(course_name))
    W["fixed"] = fixed


def _run_pos(job):
    which, pos, oris = job
    st = W["study"]
    occ, mind = st.kart.occ_map(pos, which)
    if mind < CLEAR_MIN:
        return [dict(x=pos[0], y=pos[1], z=pos[2], valid=False, why="too close to kart", clear_m=float(mind))]
    sel = st.side_sel(which)
    rows = []
    for yaw, pitch in oris:
        blk = st.kart.image_blocked(occ, yaw, pitch, CAM_OF[which])
        r = dict(x=pos[0], y=pos[1], z=pos[2], yaw=yaw, pitch=pitch, clear_m=float(mind), blocked=blk)
        if blk > OCC_MAX:
            r.update(valid=False, why="kart fills image")
        else:
            q = st.quality(occ, pos, yaw, pitch, CAM_OF[which], sel)
            s, rm, rp = st.set_score(np.maximum(q, W["fixed"]))
            r.update(valid=True, score=s)
        rows.append(r)
    return rows


def sweep(study, course_name, which, cfgs, workers, fine=False, env=None, log=print):
    """Score every grid setting of camera `which` with the other two at cfgs. -> (rows, others-alone score)."""
    others = [w for w in cfgs if w != which]
    fixed = np.maximum(study.cam_quality(others[0], cfgs[others[0]]), study.cam_quality(others[1], cfgs[others[1]]))
    positions, oris = grid(which, fine, env)
    jobs = [(which, p, oris) for p in positions]
    rows, t0 = [], time.time()
    with Pool(workers, initializer=_init, initargs=(course_name, fixed)) as pool:
        for i, r in enumerate(pool.imap_unordered(_run_pos, jobs, chunksize=4)):
            rows += r
            if (i + 1) % 200 == 0:
                log(f"    {which}: {i + 1}/{len(jobs)} positions, {time.time() - t0:.0f} s")
    return rows, study.set_score(fixed)[0]


def best_per_position(rows):
    best = {}
    for r in rows:
        if r.get("valid"):
            k = (r["x"], r["y"], r["z"])
            if k not in best or r["score"] > best[k]["score"]:
                best[k] = r
    return best


def largest_box(best, thr):
    """Largest axis-aligned box (by volume) whose grid positions all score >= thr."""
    xs = sorted({k[0] for k in best}); ys = sorted({k[1] for k in best}); zs = sorted({k[2] for k in best})
    good = np.zeros((len(xs), len(ys), len(zs)), int)
    for (x, y, z), r in best.items():
        good[xs.index(x), ys.index(y), zs.index(z)] = r["score"] >= thr
    S = np.zeros(tuple(n + 1 for n in good.shape), int)
    S[1:, 1:, 1:] = good.cumsum(0).cumsum(1).cumsum(2)
    d = [(v[1] - v[0]) if len(v) > 1 else 0.05 for v in (xs, ys, zs)]
    dx, dy, dz = d[0], abs(d[1]), d[2]
    bestv, box = -1, None
    nx, ny, nz = good.shape
    for i0 in range(nx):
        for i1 in range(i0, nx):
            for j0 in range(ny):
                for j1 in range(j0, ny):
                    for k0 in range(nz):
                        for k1 in range(k0, nz):
                            n = (i1 - i0 + 1) * (j1 - j0 + 1) * (k1 - k0 + 1)
                            s = (S[i1 + 1, j1 + 1, k1 + 1] - S[i0, j1 + 1, k1 + 1] - S[i1 + 1, j0, k1 + 1]
                                 - S[i1 + 1, j1 + 1, k0] + S[i0, j0, k1 + 1] + S[i0, j1 + 1, k0]
                                 + S[i1 + 1, j0, k0] - S[i0, j0, k0])
                            if s < n:
                                break
                            vol = (i1 - i0 + 1) * dx * (j1 - j0 + 1) * dy * (k1 - k0 + 1) * dz
                            if vol > bestv:
                                bestv, box = vol, (xs[i0], xs[i1], ys[j0], ys[j1], zs[k0], zs[k1])
    return box, bestv
