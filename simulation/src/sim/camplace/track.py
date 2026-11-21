"""Courses the study runs on: the Purdue track (RTK survey) and tracks from kart-sw's generator.

A Course has a closed centreline sampled every EDGE_STEP_M, both edges at those samples,
arc length s (0 at the start, increasing in the race direction), and one or more driven lines.
"""
import json
import math
import os
from dataclasses import dataclass, field

import numpy as np

from .config import EDGE_STEP_M, TRACKS_DIR


@dataclass
class Course:
    name: str
    centre: np.ndarray
    tangent: np.ndarray
    normal: np.ndarray            # points left
    s: np.ndarray
    length: float
    left_edge: np.ndarray
    right_edge: np.ndarray
    width: np.ndarray
    curvature: np.ndarray
    lines: list = field(default_factory=list)   # [(name, (M, 2) closed polyline)]

    def project(self, x, y):
        """Arc length s of the point's projection on the centreline."""
        k = int(np.argmin(np.hypot(self.centre[:, 0] - x, self.centre[:, 1] - y)))
        along = float((x - self.centre[k, 0]) * self.tangent[k, 0] + (y - self.centre[k, 1]) * self.tangent[k, 1])
        return (self.s[k] + along) % self.length

    @property
    def min_radius(self):
        return float(1.0 / max(np.abs(self.curvature).max(), 1e-9))


def _frame(centre):
    nxt, prv = np.roll(centre, -1, axis=0), np.roll(centre, 1, axis=0)
    seg = np.linalg.norm(nxt - centre, axis=1)
    tang = nxt - prv
    tang /= np.linalg.norm(tang, axis=1, keepdims=True)
    normal = np.column_stack([-tang[:, 1], tang[:, 0]])
    s = np.r_[0.0, np.cumsum(seg)[:-1]]
    return tang, normal, s, float(seg.sum())


def _curvature(P, k):
    a, b, c = np.roll(P, k, axis=0), P, np.roll(P, -k, axis=0)
    ab, bc, ac = b - a, c - b, c - a
    cross = ab[:, 0] * bc[:, 1] - ab[:, 1] * bc[:, 0]
    den = np.linalg.norm(ab, axis=1) * np.linalg.norm(bc, axis=1) * np.linalg.norm(ac, axis=1)
    return 2 * cross / np.maximum(den, 1e-12)


def purdue():
    """Purdue kart track from the 2021 RTK edge survey, driven on its min-time racing line."""
    from .trackmap import TrackMap
    d = os.path.join(TRACKS_DIR, "purdue_rtk")
    tm = TrackMap(os.path.join(d, "track_world.xml"))
    line = np.load(os.path.join(d, "mintime_line.npz"))["pts"]
    return Course("purdue", tm.pts, tm.tangent, tm.normal, tm.s, tm.length, tm.left_edge, tm.right_edge,
                  tm.hw_left + tm.hw_right, _curvature(tm.pts, 8), [("min-time line", line)])


def generated(path, offsets=(0.0, 0.4, -0.4)):
    """A kart-sw generator track (JSON). It has no racing line, so the kart drives the centreline
    and lines at +-40 % of the half width, which spans apex and outside lines."""
    with open(path) as fh:
        d = json.load(fh)
    C = np.asarray(d["centre"], float)
    hl, hr = np.asarray(d["half_left"], float), np.asarray(d["half_right"], float)
    # Resample to EDGE_STEP_M so edge points are as dense as on Purdue.
    Cc = np.vstack([C, C[:1]])
    sc = np.r_[0.0, np.cumsum(np.linalg.norm(np.diff(Cc, axis=0), axis=1))]
    n = int(round(sc[-1] / EDGE_STEP_M))
    t = np.linspace(0.0, sc[-1], n, endpoint=False)
    centre = np.column_stack([np.interp(t, sc, Cc[:, 0]), np.interp(t, sc, Cc[:, 1])])
    hl = np.interp(t, sc, np.r_[hl, hl[:1]])
    hr = np.interp(t, sc, np.r_[hr, hr[:1]])
    tang, normal, s, L = _frame(centre)
    lines = []
    for off in offsets:
        lat = np.where(off >= 0, off * hl, off * hr)
        nm = "centreline" if off == 0 else f"{abs(off):.0%} of half width to the {'left' if off > 0 else 'right'}"
        lines.append((nm, centre + normal * lat[:, None]))
    name = os.path.splitext(os.path.basename(path))[0]
    return Course(name, centre, tang, normal, s, L, centre + normal * hl[:, None], centre - normal * hr[:, None],
                  hl + hr, _curvature(centre, 8), lines)


def load(name):
    """'purdue', or a generated track name / path (data/tracks/generated/<name>.json)."""
    if name == "purdue":
        return purdue()
    path = name if name.endswith(".json") else os.path.join(TRACKS_DIR, "generated", name + ".json")
    return generated(path)
