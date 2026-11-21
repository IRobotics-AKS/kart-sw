"""Scoring a camera set on a course.

Every edge point (both edges, 1.5 m behind the kart centre to 25 m ahead, at every pose) gets a
quality from each camera: 0 if it is outside the image or the kart blocks it at that pose's
steering angle, else min(1, GSD_THR_CM / cm-per-pixel across the edge). A set's quality per point
is the best of its cameras. Coverage per pose and band is the mean over both edges; the set score
weights the bands (WEIGHTS) and, per band, averages the lap mean and the worst-10 % pose,
each taken as the worst over the pitch/roll perturbations.
"""
import math

import numpy as np

from .config import BANDS, CAM_OF, CAMS, GSD_THR_CM, OCC_MAX, PERTURB, WEIGHTS, WIDTH_PX
from .geometry import cam_rot
from .kart import Kart, lookup_idx
from .lap import edge_points


class Study:
    def __init__(self, course):
        self.course = course
        self.P = edge_points(course)
        self.kart = Kart(bins=np.unique(self.P["steer_bin"]).tolist())

    # ---------------------------------------------------------------- one camera
    def quality(self, occ, pos, yaw, pitch, cam, sel=None):
        """-> (len(PERTURB), N) quality of every edge point for one camera."""
        P = self.P
        hfov, vfov = CAMS[cam]
        tx, ty = math.tan(math.radians(hfov / 2)), math.tan(math.radians(vfov / 2))
        f = (WIDTH_PX / 2) / tx
        R = cam_rot(yaw, pitch)
        pos = np.asarray(pos, float)
        out = np.zeros((len(PERTURB), len(P["q"])))
        idx = np.arange(len(P["q"])) if sel is None else sel
        sb = P["steer_bin"][idx]
        for pi, (q, nrm) in enumerate(P["tilt"]):
            q, nrm = q[idx], nrm[idx]
            c = (q - pos) @ R
            z = c[:, 2]
            zz = np.maximum(z, 1e-9)
            xn, yn = c[:, 0] / zz, c[:, 1] / zz
            cand = np.flatnonzero((z > 0.05) & (np.abs(xn) <= tx) & (np.abs(yn) <= ty))
            vis = np.zeros(len(idx), bool)
            if cand.size:
                ie, ia = lookup_idx(q[cand] - pos)
                vis[cand] = ~occ[sb[cand], ie, ia]
            c2 = (q + 0.05 * nrm - pos) @ R
            z2 = np.maximum(c2[:, 2], 1e-9)
            dpx = f * np.hypot(c2[:, 0] / z2 - xn, c2[:, 1] / z2 - yn)
            out[pi, idx] = np.where(vis, np.minimum(1.0, GSD_THR_CM / (5.0 / np.maximum(dpx, 1e-9))), 0.0)
        return out

    def side_sel(self, which):
        """Edge points a camera is scored on: all for the front, its own edge for a side camera."""
        if which == "front":
            return None
        return np.flatnonzero(self.P["side"] == (0 if which == "left" else 1))

    def cam_quality(self, which, cfg):
        occ, _ = self.kart.occ_map(cfg[:3], which)
        return self.quality(occ, cfg[:3], cfg[3], cfg[4], CAM_OF[which])

    # ---------------------------------------------------------------- the set
    def set_score(self, qu):
        """qu: (len(PERTURB), N) best quality per point over the set -> (score, robust mean, robust p10)."""
        P = self.P
        nf, nb = P["nframes"], len(BANDS)
        means, p10s = [], []
        for pi in range(qu.shape[0]):
            hit = np.bincount(P["key"], weights=qu[pi], minlength=nf * nb * 2)
            frac = np.where(P["tot"] > 0, hit / np.maximum(P["tot"], 1), np.nan).reshape(nf, nb, 2)
            cov = np.nanmean(frac, axis=2)
            means.append(np.nanmean(cov, axis=0))
            p10s.append(np.nanpercentile(cov, 10, axis=0))
        rob_mean, rob_p10 = np.min(means, axis=0), np.min(p10s, axis=0)
        return float(WEIGHTS @ (0.5 * (rob_mean + rob_p10))), rob_mean, rob_p10

    def gaps(self, seen, lo, hi):
        """Longest stretch (m) of unseen edge between lo and hi m ahead, per pose and edge."""
        P = self.P
        out = []
        order = np.lexsort((P["ds"], P["side"], P["frame"]))
        fr, sd, ds, vs = P["frame"][order], P["side"][order], P["ds"][order], seen[order]
        brk = np.flatnonzero((np.diff(fr) != 0) | (np.diff(sd) != 0)) + 1
        for a, b in zip(np.r_[0, brk], np.r_[brk, len(fr)]):
            d, v = ds[a:b], vs[a:b]
            m = (d >= lo) & (d < hi)
            d, v = d[m], v[m]
            best = run = 0.0
            step = np.r_[np.diff(d), 0.25] if len(d) else d
            for dd, vv, st in zip(d, v, step):
                run = 0.0 if vv else run + min(st, 0.5)
                best = max(best, run)
            out.append(best)
        return np.array(out)

    def evaluate(self, cfgs):
        """Full report for a set {front/left/right: (x, y, z, yaw, pitch)}."""
        qs, blocked, clear = {}, {}, {}
        for w, c in cfgs.items():
            occ, mind = self.kart.occ_map(c[:3], w)
            qs[w] = self.quality(occ, c[:3], c[3], c[4], CAM_OF[w])
            blocked[w] = self.kart.image_blocked(occ, c[3], c[4], CAM_OF[w])
            clear[w] = mind
        u = np.maximum.reduce(list(qs.values()))
        s, rm, rp = self.set_score(u)
        seen = u[0] > 0
        g8 = self.gaps(seen, 0.0, 8.0)
        return dict(score=s, seen=float(seen.mean()),
                    mean={b: float(v) for b, v in zip(BANDS, rm)}, p10={b: float(v) for b, v in zip(BANDS, rp)},
                    gap8_p95=float(np.percentile(g8, 95)), gap8_max=float(g8.max()), gap8_gt1=float((g8 > 1.0).mean()),
                    blocked=blocked, clearance=clear,
                    passes_limits=all(blocked[w] <= OCC_MAX for w in cfgs) and all(clear[w] >= 0.05 for w in cfgs))
