"""Kart poses along the driven line(s) and the track-edge points each pose should see."""
import math

import numpy as np

from .config import BANDS, FRAME_STEP_M, MID, PERTURB, SPEED, STEER_BINS, WHEELBASE
from .geometry import tilted


def speed_profile(kappa, ds, vmax, alat, acc, brk):
    """Curvature-limited speed with acceleration and braking limits, on a closed lap."""
    v = np.minimum(vmax, np.sqrt(alat / np.maximum(np.abs(kappa), 1e-6)))
    n = len(v)
    for _ in range(2):
        for i in range(1, 2 * n):
            j, k = i % n, (i - 1) % n
            v[j] = min(v[j], math.sqrt(v[k] ** 2 + 2 * acc * ds))
        for i in range(2 * n, 0, -1):
            j, k = i % n, (i + 1) % n
            v[j] = min(v[j], math.sqrt(v[k] ** 2 + 2 * brk * ds))
    return v


def poses(course, line, step_m=FRAME_STEP_M):
    """Kart poses every step_m along a closed line: rows of
    (rear-axle x, y, yaw, speed, yaw rate, steer angle, arc length s of the line point)."""
    P = np.asarray(line, float)
    seg = np.linalg.norm(np.diff(np.vstack([P, P[:1]]), axis=0), axis=1)
    ds = float(seg.mean())
    t = np.roll(P, -1, 0) - np.roll(P, 1, 0)
    yaw = np.unwrap(np.arctan2(t[:, 1], t[:, 0]))
    dyaw = np.angle(np.exp(1j * (np.roll(yaw, -1) - np.roll(yaw, 1))))
    kappa = dyaw / (2 * ds)
    kappa = np.convolve(np.r_[kappa[-3:], kappa, kappa[:3]], np.ones(7) / 7, "same")[3:-3]
    v = speed_profile(kappa, ds, **SPEED)
    out = []
    for i in np.arange(0, len(P), max(1, int(round(step_m / ds)))):
        c, s = math.cos(yaw[i]), math.sin(yaw[i])
        out.append((P[i, 0] - MID * c, P[i, 1] - MID * s, yaw[i], v[i], v[i] * kappa[i],
                    math.atan(WHEELBASE * kappa[i]), course.project(P[i, 0], P[i, 1])))
    return np.array(out)


def edge_points(course, bands=BANDS, step_m=FRAME_STEP_M):
    """Edge points in the kart frame for every pose on every driven line.

    -> dict: q (N,3) points, nrm (N,3) across-edge directions, frame, band, side (0 left, 1 right),
       ds (along-track offset from the kart), steer_bin, nframes, tilt (per PERTURB: (q, nrm)),
       key / tot (for per-frame, per-band, per-side coverage)."""
    frames = np.vstack([poses(course, ln, step_m) for _, ln in course.lines])
    L = course.length
    out = {k: [] for k in ("q", "nrm", "frame", "band", "side", "ds")}
    for fi, (x0, y0, yaw, v, r, steer, s0) in enumerate(frames):
        c, s = math.cos(yaw), math.sin(yaw)
        dss = (course.s - s0 + L / 2) % L - L / 2
        for si, E in enumerate((course.left_edge, course.right_edge)):
            for bi, (a, b) in enumerate(bands.values()):
                m = (dss >= a) & (dss < b)
                if not m.any():
                    continue
                dx, dy = E[m, 0] - x0, E[m, 1] - y0
                qx, qy = c * dx + s * dy, -s * dx + c * dy
                N = course.normal[m]
                n = len(qx)
                out["q"].append(np.column_stack([qx, qy, np.zeros(n)]))
                out["nrm"].append(np.column_stack([c * N[:, 0] + s * N[:, 1], -s * N[:, 0] + c * N[:, 1], np.zeros(n)]))
                out["frame"].append(np.full(n, fi))
                out["band"].append(np.full(n, bi))
                out["side"].append(np.full(n, si))
                out["ds"].append(dss[m])
    P = {k: np.concatenate(v) for k, v in out.items()}
    P["steer_bin"] = np.abs(frames[P["frame"], 5][:, None] - STEER_BINS[None]).argmin(1)
    P["nframes"] = len(frames)
    P["frames"] = frames
    nb = len(bands)
    P["key"] = (P["frame"] * nb + P["band"]) * 2 + P["side"]
    P["tot"] = np.bincount(P["key"], minlength=len(frames) * nb * 2).astype(float)
    P["tilt"] = [tilted(P["q"], P["nrm"], pp, rr) for pp, rr in PERTURB]
    return P
