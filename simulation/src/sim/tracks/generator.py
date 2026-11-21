"""Random but realistic closed kart tracks.

Two layout styles:
  "kart" (default): like a kart circuit or a parking-lot course (see the AKS tracks).
     A random region of grid cells is grown in narrow arms; its outline is the centre
     line, so arms become hairpins and the lap folds back on itself with one cell
     between neighbouring parts. The grid is jittered and the corners rounded.
  "flowing": random points -> convex hull -> midpoints pushed in or out -> closed
     Catmull-Rom curve -> scaled to a random lap length -> smoothed. Bigger, more open.
Then a varying width, a start line on the longest straight, edge-style sections and
optional obstacles (by default grouped in one obstacle zone, like the AKS time trial).

A layout is kept only if it passes check(); otherwise the next attempt (from the same
random stream, so a seed always gives the same track) is tried.
"""
import math
from dataclasses import dataclass

import numpy as np

from .format import EDGE_STYLES, OBSTACLE_KINDS, RUNOFFS, Obstacle, Section, Track

STEP = 0.5   # m between centre-line samples


@dataclass
class Params:
    length: tuple = (300.0, 1000.0)     # lap length range (m)
    width: tuple = (5.0, 9.0)           # track width range (m)
    min_radius: float = 5.0             # tightest corner, centre line (m)
    edge_room: float = 2.5              # inside edge to corner centre, keeps barriers room (m)
    start_straight: float = 20.0        # straight with the start line in its middle (m)
    gap: float = 5.0                    # grass between separate parts of the track (m)
    corners: tuple = (8, 40)            # number of corners allowed
    straight_share: tuple = (0.15, 0.65)
    max_tries: int = 400
    zone_spacing: tuple = (6.0, 10.0)   # m between obstacles in the obstacle zone


class GenerationError(RuntimeError):
    pass


LAYOUTS = ("kart", "flowing")


def generate(seed=None, params=None, edges="mixed", runoff="mixed", obstacles=0, moving=0,
             layout="kart", zone=True):
    """A random Track.

    edges: one of EDGE_STYLES, or "mixed" for a different style per section and side.
    runoff: "grass", "concrete" or "mixed".
    obstacles: number of obstacles on the driving surface; `moving` of them slide across.
    layout: "kart" (compact, hairpins, like a kart circuit) or "flowing" (open, sweeping).
    zone: put the obstacles together in one obstacle zone (a slalom on a straight part);
          False spreads them round the lap.
    """
    if layout not in LAYOUTS:
        raise ValueError(f"layout must be one of {LAYOUTS}")
    p = params or Params()
    if seed is None:
        seed = int(np.random.SeedSequence().entropy % 2**31)
    rng = np.random.default_rng(seed)
    for _ in range(p.max_tries):
        made = (_grid_layout if layout == "kart" else _layout)(rng, p)
        if made is None:
            continue
        centre, half = made
        track = _with_start(centre, half, p)
        if track is None or check(track, p):
            continue
        track.seed = seed
        track.sections = make_sections(track, rng, edges, runoff)
        if zone and obstacles:
            track.obstacles = place_obstacle_zone(track, rng, obstacles, moving, p)
        else:
            track.obstacles = place_obstacles(track, rng, obstacles, moving)
        return track
    raise GenerationError(f"no valid track after {p.max_tries} attempts (seed {seed})")


# ------------------------------------------------------------------ layout
def _layout(rng, p):
    n = int(rng.integers(10, 26))
    aspect = rng.uniform(1.0, 2.2)
    pts = rng.uniform(-1, 1, (n, 2)) * [aspect, 1.0]
    hull = _convex_hull(pts)
    if len(hull) < 5:
        return None
    ctrl = _displace(hull, rng, amount=rng.uniform(0.3, 0.7))
    ctrl = _push_apart(ctrl, 0.3)
    ctrl = _displace(ctrl, rng, amount=rng.uniform(0.0, 0.25))
    ctrl = _push_apart(ctrl, 0.15)
    centre = _catmull_rom(ctrl, 30)
    if _self_intersects(centre[::3]):
        return None
    target = rng.uniform(*p.length)
    centre = _resample(centre * target / _length(centre), STEP)
    centre = _smooth(centre, 25)
    centre = _resample(centre * target / _length(centre), STEP)
    n = len(centre)
    s = np.arange(n) / n
    w0 = rng.uniform(p.width[0] + 0.5, p.width[1] - 0.5)
    w = w0 + rng.uniform(0, 1.0) * np.sin(2 * math.pi * (int(rng.integers(1, 4)) * s + rng.uniform()))
    w = np.clip(w, *p.width)
    centre = _open_tight_corners(centre, w / 2, p)
    if centre is None or _self_intersects(centre[::4]):
        return None
    w = np.interp(np.arange(len(centre)) / len(centre), s, w, period=1.0)
    return centre, w / 2


def _grid_layout(rng, p):
    """Kart-circuit layout: outline of a random region of grid cells grown in arms."""
    w0 = rng.uniform(p.width[0] + 0.5, p.width[1] - 0.5)
    amp = rng.uniform(0, 1.0)
    cell = w0 + amp + p.gap + rng.uniform(1.0, 4.0)     # neighbouring parts: one cell apart
    target = rng.uniform(*p.length)
    cells = _grow_region(rng, target / cell)
    if cells is None:
        return None
    loop = _outline(cells).astype(float)
    # jitter grid lines (not single vertices) so parallel parts stay parallel-ish
    for ax in (0, 1):
        vals = np.unique(loop[:, ax])
        shift = dict(zip(vals, rng.uniform(-0.08, 0.08, len(vals))))
        loop[:, ax] += [shift[v] for v in loop[:, ax]]
    shear = rng.uniform(-0.2, 0.2)                    # some layouts lean, like Apex
    loop[:, 0] += shear * loop[:, 1]
    need = max(p.min_radius, (w0 + amp) / 2 + p.edge_room)
    radius = need * rng.uniform(1.1, 1.4, len(loop))     # some corners rounder than others
    centre = _resample(_fillet(loop * cell, radius), STEP)
    centre = _smooth(centre, 5)
    n = len(centre)
    s = np.arange(n) / n
    w = w0 + amp * np.sin(2 * math.pi * (int(rng.integers(1, 4)) * s + rng.uniform()))
    w = np.clip(w, *p.width)
    centre = _open_tight_corners(centre, w / 2, p)
    if centre is None or _self_intersects(centre[::4]):
        return None
    w = np.interp(np.arange(len(centre)) / len(centre), s, w, period=1.0)
    theta = rng.uniform(-np.pi, np.pi)
    R = np.array([[np.cos(theta), -np.sin(theta)], [np.sin(theta), np.cos(theta)]])
    return centre @ R.T, w / 2


def _fillet(P, radius):
    """Closed polygon with each corner replaced by a circular arc (radius per corner,
    shrunk where the neighbouring sides are too short to fit it)."""
    n = len(P)
    a_in = P - np.roll(P, 1, axis=0)
    a_out = np.roll(P, -1, axis=0) - P
    L_in, L_out = np.linalg.norm(a_in, axis=1), np.linalg.norm(a_out, axis=1)
    u_in, u_out = a_in / L_in[:, None], a_out / L_out[:, None]
    turn = np.arccos(np.clip((u_in * u_out).sum(1), -1, 1))
    t = np.tan(turn / 2)
    cut = radius * t
    room = np.minimum(L_in, L_out) / 2                  # each side shared by two corners
    cut = np.minimum(cut, room)
    r = cut / np.maximum(t, 1e-9)
    out = []
    for i in range(n):
        p0 = P[i] - u_in[i] * cut[i]
        sgn = np.sign(u_in[i, 0] * u_out[i, 1] - u_in[i, 1] * u_out[i, 0])
        nrm = np.array([-u_in[i, 1], u_in[i, 0]]) * sgn
        c = p0 + nrm * r[i]
        a0 = math.atan2(*(p0 - c)[::-1])
        for f in np.linspace(0, 1, 12):
            a = a0 + sgn * turn[i] * f
            out.append(c + r[i] * np.array([math.cos(a), math.sin(a)]))
    return np.array(out)


def _grow_region(rng, perimeter, max_span=(9, 13)):
    """Random simply connected set of grid cells (no holes, no corner-only touches)
    with about `perimeter` cell edges of outline, grown mostly as 1-cell-wide arms."""
    span = sorted((int(rng.integers(5, max_span[0] + 1)), int(rng.integers(6, max_span[1] + 1))))
    cells = {(i, 0) for i in range(int(rng.integers(2, 5)))}
    dirs = [(1, 0), (-1, 0), (0, 1), (0, -1)]
    for _ in range(4000):
        if _perimeter(cells) >= perimeter:
            return cells
        cx, cy = sorted(cells)[int(rng.integers(len(cells)))]
        dx, dy = dirs[int(rng.integers(4))]
        arm, new = [], set(cells)
        for k in range(1, int(rng.integers(2, 6)) + 1):    # whole arm or nothing
            c = (cx + dx * k, cy + dy * k)
            if c in new:
                arm = []
                break
            # arms stay thin: each new cell touches the region on at most 2 sides
            if sum((c[0] + a, c[1] + b) in new for a, b in dirs) > (1 if k == 1 else 1):
                arm = []
                break
            new.add(c)
            arm.append(c)
        if len(arm) < 2:
            continue
        xs, ys = zip(*new)
        w, h = sorted((max(xs) - min(xs) + 1, max(ys) - min(ys) + 1))
        if w > span[0] or h > span[1] or not all(_simple(new, c) for c in arm):
            continue
        cells = new
    return None


def _perimeter(cells):
    return 4 * len(cells) - sum((x + 1, y) in cells for x, y in cells) * 2 \
        - sum((x, y + 1) in cells for x, y in cells) * 2


def _simple(cells, c):
    """True if adding c left the region without holes or corner-only contacts."""
    x, y = c
    for a in (-1, 0):                       # 2x2 windows containing c: no diagonal pair
        for b in (-1, 0):
            q = [(x + a + i, y + b + j) in cells for i in (0, 1) for j in (0, 1)]
            if q == [True, False, False, True] or q == [False, True, True, False]:
                return False
    xs, ys = zip(*cells)
    x0, x1, y0, y1 = min(xs) - 1, max(xs) + 1, min(ys) - 1, max(ys) + 1
    free = (x1 - x0 + 1) * (y1 - y0 + 1) - len(cells)
    seen, stack = {(x0, y0)}, [(x0, y0)]
    while stack:                            # flood the outside; every empty cell reached?
        px, py = stack.pop()
        for a, b in ((1, 0), (-1, 0), (0, 1), (0, -1)):
            q = (px + a, py + b)
            if x0 <= q[0] <= x1 and y0 <= q[1] <= y1 and q not in cells and q not in seen:
                seen.add(q)
                stack.append(q)
    return len(seen) == free


def _outline(cells):
    """Corner points of the outline of a simply connected cell region, anticlockwise."""
    edges = set()
    for x, y in cells:
        edges |= {((x, y), (x + 1, y)), ((x + 1, y), (x + 1, y + 1)),
                  ((x + 1, y + 1), (x, y + 1)), ((x, y + 1), (x, y))}
    nxt = {a: b for a, b in edges if (b, a) not in edges}
    start = min(nxt)
    pts, q = [start], nxt[start]
    while q != start:
        pts.append(q)
        q = nxt[q]
    pts = np.array(pts)
    d1 = pts - np.roll(pts, 1, axis=0)
    d2 = np.roll(pts, -1, axis=0) - pts
    corner = (d1[:, 0] * d2[:, 1] - d1[:, 1] * d2[:, 0]) != 0
    return pts[corner]


def _open_tight_corners(P, half, p, rounds=60):
    """Smooth only around corners tighter than allowed, until none are left."""
    for _ in range(rounds):
        trk = Track(P, half, half, (0, 0, 0))
        radius = 1 / np.maximum(np.abs(trk.curvature), 1e-9)
        bad = radius < np.maximum(p.min_radius, half + p.edge_room) * 1.05
        if not bad.any():
            return P
        win = np.convolve(np.r_[bad[-30:], bad, bad[:30]].astype(float), np.ones(61), "same")[30:-30] > 0
        P = _resample(P + (_blur(P, 6) - P) * win[:, None], STEP)
        half = np.interp(np.arange(len(P)) / len(P), np.arange(len(half)) / len(half), half, period=1.0)
    return None


def _blur(P, sigma):
    """Closed polyline convolved with a Gaussian of `sigma` samples."""
    k = np.arange(-3 * sigma, 3 * sigma + 1)
    w = np.exp(-0.5 * (k / sigma) ** 2)
    w /= w.sum()
    return sum(wi * np.roll(P, -ki, axis=0) for ki, wi in zip(k, w))


def _with_start(centre, half, p):
    """Roll the centre line so the start line sits on the longest straight."""
    trk = Track(centre, half, half, (0, 0, 0))
    straight = np.abs(trk.curvature) < 1 / 60
    if straight.all():
        return None
    run = _runs(straight)
    if not run:
        return None
    i0, length = max(run, key=lambda r: r[1])
    if length * STEP < p.start_straight:
        return None
    start = (i0 + length // 2) % trk.n
    if np.random.default_rng(start).random() < 0.5:     # deterministic direction per layout
        centre, half = centre[::-1], half[::-1]
        start = trk.n - 1 - start
    centre, half = np.roll(centre, -start, axis=0), np.roll(half, -start)
    trk = Track(centre, half, half, (0, 0, 0))
    back = trk.index_at(-3.0)                     # kart rear axle 3 m behind the line
    x, y = trk.centre[back]
    return Track(centre, half, half, (x, y, float(trk.heading[back])))


def check(track, p=None):
    """Problems with a track (empty list = valid)."""
    p = p or Params()
    bad = []
    k = np.abs(track.curvature)
    radius = 1 / np.maximum(k, 1e-9)
    inside_half = np.where(track.curvature > 0, track.half_left, track.half_right)
    need = np.maximum(p.min_radius, inside_half + p.edge_room)
    if (radius < need).any():
        bad.append(f"corner too tight: {radius.min():.1f} m")
    if not (p.length[0] - 1 <= track.length <= p.length[1] + 1):
        bad.append(f"length {track.length:.0f} m")
    if track.width.min() < p.width[0] - 1e-6 or track.width.max() > p.width[1] + 1e-6:
        bad.append("width out of range")
    corners = _runs(radius < 40)
    if not (p.corners[0] <= len(corners) <= p.corners[1]):
        bad.append(f"{len(corners)} corners")
    share = float(np.mean(radius > 60))
    if not (p.straight_share[0] <= share <= p.straight_share[1]):
        bad.append(f"straight share {share:.2f}")
    for s in (-p.start_straight / 2, p.start_straight / 2):
        i = track.index_at(s)
        lo, hi = sorted((0, i)) if s > 0 else (i, track.n)
        if (k[lo:hi] >= 1 / 60).any():
            bad.append("start not on a straight")
            break
    if _too_close(track, p.gap):
        bad.append("track parts too close")
    return bad


def _too_close(track, gap):
    """Any two parts of the track (far apart along the lap) closer than edges + gap."""
    P = track.centre[::2]
    hw = np.maximum(track.half_left, track.half_right)[::2]
    s = track.s[::2]
    d = np.linalg.norm(P[:, None] - P[None], axis=2)
    ds = np.abs(s[:, None] - s[None])
    ds = np.minimum(ds, track.length - ds)
    need = hw[:, None] + hw[None] + gap
    far = ds > np.pi * (need / 2 + 1.0) + 2.0      # beyond what one corner can bring back
    return bool((far & (d < need)).any())


# ------------------------------------------------------------------ dressing
def make_sections(track, rng, edges="mixed", runoff="mixed"):
    if edges != "mixed" and edges not in EDGE_STYLES:
        raise ValueError(f"edges must be 'mixed' or one of {EDGE_STYLES}")
    if runoff != "mixed" and runoff not in RUNOFFS:
        raise ValueError(f"runoff must be 'mixed' or one of {RUNOFFS}")
    if edges != "mixed":
        r = runoff if runoff != "mixed" else str(rng.choice(RUNOFFS))
        return [Section(0.0, track.length, edges, edges, r)]
    cuts = [0.0]
    while cuts[-1] < track.length:
        cuts.append(cuts[-1] + rng.uniform(40, 120))
    cuts[-1] = track.length
    if len(cuts) > 2 and cuts[-1] - cuts[-2] < 30:
        del cuts[-2]
    styles = [s for s in EDGE_STYLES if s != "none"]
    out = []
    for a, b in zip(cuts[:-1], cuts[1:]):
        left = str(rng.choice(styles))
        right = left if rng.random() < 0.5 else str(rng.choice(styles))
        r = runoff if runoff != "mixed" else str(rng.choice(RUNOFFS, p=[0.6, 0.4]))
        out.append(Section(float(a), float(b), left, right, r))
    return out


def place_obstacles(track, rng, count, moving=0):
    """`count` obstacles on the surface, >= 25 m from the start line and 15 m apart."""
    out, used = [], []
    moving = min(moving, count)
    for i in range(count):
        for _ in range(200):
            s = rng.uniform(25, track.length - 25)
            if all(abs(s - u) > 15 for u in used):
                break
        else:
            break
        used.append(s)
        j = track.index_at(s)
        kind = str(rng.choice(OBSTACLE_KINDS))
        if kind == "cone":
            size, r = [0.45], 0.19
        elif kind == "box":
            h = rng.uniform(0.2, 0.4)
            size, r = [h, h * rng.uniform(0.8, 1.2), h * rng.uniform(0.8, 1.2)], h * 1.42
        else:
            size, r = [0.29, 0.44], 0.29
        lo, hi = -track.half_right[j] + r + 0.2, track.half_left[j] - r - 0.2
        u = rng.uniform(lo, hi)
        x, y = track.centre[j] + track.normal[j] * u
        nx, ny = track.normal[j]
        o = Obstacle(kind, float(x), float(y), float(rng.uniform(-np.pi, np.pi)),
                     [float(v) for v in size])
        if i < moving:
            amp = (hi - lo) / 2
            cx, cy = track.centre[j] + track.normal[j] * (lo + hi) / 2
            o.x, o.y = float(cx), float(cy)
            o.amplitude, o.period = float(amp), float(rng.uniform(3, 8))
            o.phase, o.nx, o.ny = float(rng.uniform(0, 2 * np.pi)), float(nx), float(ny)
        out.append(o)
    return out


def place_obstacle_zone(track, rng, count, moving=0, p=None):
    """`count` obstacles grouped in one obstacle zone: a slalom on the straightest part
    of the lap away from the start, `zone_spacing` m apart on alternating sides, always
    leaving a gap the kart fits through. `moving` of them slide across the track."""
    p = p or Params()
    gaps = rng.uniform(*p.zone_spacing, max(count - 1, 0))
    span = float(gaps.sum())
    radius = 1 / np.maximum(np.abs(track.curvature), 1e-9)
    best, best_s = -1.0, None
    for s0 in np.arange(30.0, track.length - 30.0 - span, 2.0):
        idx = [track.index_at(v) for v in np.arange(s0 - 5, s0 + span + 5, 1.0)]
        score = float(radius[idx].min())
        if score > best:
            best, best_s = score, s0
    if best_s is None:
        return place_obstacles(track, rng, count, moving)
    side = 1 if rng.random() < 0.5 else -1
    is_moving = set(rng.choice(count, size=min(moving, count), replace=False).tolist())
    out, s = [], best_s
    for i in range(count):
        j = track.index_at(s)
        kind = str(rng.choice(OBSTACLE_KINDS))
        size, r = _obstacle_size(kind, rng)
        lo, hi = -track.half_right[j] + r + 0.2, track.half_left[j] - r - 0.2
        u = side * rng.uniform(0.3, 0.75) * (hi if side > 0 else -lo)   # leaves >= 2 m the other side
        x, y = track.centre[j] + track.normal[j] * u
        o = Obstacle(kind, float(x), float(y), float(rng.uniform(-np.pi, np.pi)),
                     [float(v) for v in size])
        if i in is_moving:
            cx, cy = track.centre[j] + track.normal[j] * (lo + hi) / 2
            o.x, o.y = float(cx), float(cy)
            o.amplitude, o.period = float((hi - lo) / 2), float(rng.uniform(3, 8))
            o.phase = float(rng.uniform(0, 2 * np.pi))
            o.nx, o.ny = (float(v) for v in track.normal[j])
        out.append(o)
        side = -side
        if i < count - 1:
            s += gaps[i]
    return out


def _obstacle_size(kind, rng):
    """(size list, footprint radius) for an obstacle kind."""
    if kind == "cone":
        return [0.45], 0.19
    if kind == "box":
        h = rng.uniform(0.2, 0.4)
        return [h, h * rng.uniform(0.8, 1.2), h * rng.uniform(0.8, 1.2)], h * 1.42
    return [0.29, 0.44], 0.29


# ------------------------------------------------------------------ geometry
def _convex_hull(P):
    P = sorted(map(tuple, P))

    def half(pts):
        h = []
        for q in pts:
            while len(h) >= 2 and _cross2(np.subtract(h[-1], h[-2]), np.subtract(q, h[-2])) <= 0:
                h.pop()
            h.append(q)
        return h
    lower, upper = half(P), half(P[::-1])
    return np.array(lower[:-1] + upper[:-1])


def _cross2(a, b):
    return a[0] * b[1] - a[1] * b[0]


def _displace(hull, rng, amount):
    out = []
    for a, b in zip(hull, np.roll(hull, -1, axis=0)):
        out.append(a)
        d = b - a
        L = np.linalg.norm(d)
        nrm = np.array([-d[1], d[0]]) / L
        out.append((a + b) / 2 + nrm * rng.normal(0, amount) * L)
    return np.array(out)


def _push_apart(P, dmin, iters=10):
    P = P.copy()
    for _ in range(iters):
        for i in range(len(P)):
            for j in range(i + 1, len(P)):
                d = P[j] - P[i]
                L = np.linalg.norm(d)
                if 1e-9 < L < dmin:
                    push = d / L * (dmin - L) / 2
                    P[i] -= push
                    P[j] += push
    return P


def _catmull_rom(P, per_seg):
    """Closed centripetal Catmull-Rom curve through the points."""
    out = []
    n = len(P)
    for i in range(n):
        p0, p1, p2, p3 = P[i - 1], P[i], P[(i + 1) % n], P[(i + 2) % n]
        t0 = 0.0
        t1 = t0 + max(np.linalg.norm(p1 - p0), 1e-6) ** 0.5
        t2 = t1 + max(np.linalg.norm(p2 - p1), 1e-6) ** 0.5
        t3 = t2 + max(np.linalg.norm(p3 - p2), 1e-6) ** 0.5
        for t in np.linspace(t1, t2, per_seg, endpoint=False):
            a1 = (t1 - t) / (t1 - t0) * p0 + (t - t0) / (t1 - t0) * p1
            a2 = (t2 - t) / (t2 - t1) * p1 + (t - t1) / (t2 - t1) * p2
            a3 = (t3 - t) / (t3 - t2) * p2 + (t - t2) / (t3 - t2) * p3
            b1 = (t2 - t) / (t2 - t0) * a1 + (t - t0) / (t2 - t0) * a2
            b2 = (t3 - t) / (t3 - t1) * a2 + (t - t1) / (t3 - t1) * a3
            out.append((t2 - t) / (t2 - t1) * b1 + (t - t1) / (t2 - t1) * b2)
    return np.array(out)


def _length(P):
    return float(np.linalg.norm(np.roll(P, -1, axis=0) - P, axis=1).sum())


def _resample(P, step):
    Q = np.vstack([P, P[:1]])
    seg = np.linalg.norm(np.diff(Q, axis=0), axis=1)
    s = np.r_[0.0, np.cumsum(seg)]
    n = max(int(round(s[-1] / step)), 8)
    t = np.linspace(0, s[-1], n, endpoint=False)
    return np.column_stack([np.interp(t, s, Q[:, 0]), np.interp(t, s, Q[:, 1])])


def _smooth(P, iters):
    """Taubin smoothing (removes kinks without shrinking the loop)."""
    P = P.copy()
    for _ in range(iters):
        for f in (0.5, -0.53):
            P = P + f * ((np.roll(P, 1, axis=0) + np.roll(P, -1, axis=0)) / 2 - P)
    return P


def _segments_cross(a, b, c, d):
    def orient(p, q, r):
        return np.sign((q[..., 0] - p[..., 0]) * (r[..., 1] - p[..., 1])
                       - (q[..., 1] - p[..., 1]) * (r[..., 0] - p[..., 0]))
    return (orient(a, b, c) * orient(a, b, d) < 0) & (orient(c, d, a) * orient(c, d, b) < 0)


def _self_intersects(P):
    A, B = P, np.roll(P, -1, axis=0)
    n = len(P)
    cross = _segments_cross(A[:, None], B[:, None], A[None], B[None])
    i, j = np.triu_indices(n, 2)
    keep = ~((i == 0) & (j == n - 1))
    return bool(cross[i[keep], j[keep]].any())


def _runs(mask):
    """(start index, length) of each run of True in a circular mask."""
    n = len(mask)
    if mask.all():
        return [(0, n)]
    if not mask.any():
        return []
    first_false = int(np.argmin(mask))
    m = np.roll(mask, -first_false)
    out, i = [], 0
    while i < n:
        if m[i]:
            j = i
            while j < n and m[j]:
                j += 1
            out.append(((i + first_false) % n, j - i))
            i = j
        else:
            i += 1
    return out
