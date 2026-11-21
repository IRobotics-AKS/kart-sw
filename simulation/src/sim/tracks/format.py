"""Track data and JSON load/save.

Local sim frame: x, y in metres on the ground (z up), angles in radians.
A track is a closed centre line sampled about every 0.5 m, a half width on each
side, a start pose, edge-style sections and optional obstacles. The start line is
at arc length s = 0 and the track is driven in the direction of increasing s.
"""
import json
import math
from dataclasses import asdict, dataclass, field

import numpy as np

EDGE_STYLES = ("none", "lines", "kerbs", "cones", "tyres", "concrete", "plastic")
RUNOFFS = ("grass", "concrete")
OBSTACLE_KINDS = ("cone", "box", "barrel")
FORMAT_VERSION = 1


@dataclass
class Section:
    """Edge dressing from arc length s0 to s1 (m). left/right: an EDGE_STYLES entry."""
    s0: float
    s1: float
    left: str
    right: str
    runoff: str = "grass"


@dataclass
class Obstacle:
    """An object on the driving surface.

    size: cone [height], box [half x, half y, half z], barrel [radius, half height].
    A moving obstacle slides across the track along (nx, ny) by +-amplitude with the
    given period (s); amplitude 0 means it stays still.
    """
    kind: str
    x: float
    y: float
    yaw: float
    size: list
    amplitude: float = 0.0
    period: float = 0.0
    phase: float = 0.0
    nx: float = 0.0
    ny: float = 0.0

    @property
    def moving(self):
        return self.amplitude > 0.0


@dataclass
class Track:
    centre: np.ndarray            # (N, 2) closed centre line, centre[0] on the start line
    half_left: np.ndarray         # (N,) m, centre line to the left edge
    half_right: np.ndarray        # (N,) m, centre line to the right edge
    start_pose: tuple             # (x, y, yaw) of the kart's rear axle centre
    sections: list = field(default_factory=list)
    obstacles: list = field(default_factory=list)
    seed: int | None = None

    def __post_init__(self):
        self.centre = np.asarray(self.centre, dtype=float)
        self.half_left = np.asarray(self.half_left, dtype=float)
        self.half_right = np.asarray(self.half_right, dtype=float)
        self.start_pose = tuple(float(v) for v in self.start_pose)
        nxt = np.roll(self.centre, -1, axis=0)
        seg = np.linalg.norm(nxt - self.centre, axis=1)
        self.seg = seg
        self.s = np.r_[0.0, np.cumsum(seg)[:-1]]
        self.length = float(seg.sum())
        tang = nxt - np.roll(self.centre, 1, axis=0)
        self.tangent = tang / np.linalg.norm(tang, axis=1, keepdims=True)
        self.normal = np.column_stack([-self.tangent[:, 1], self.tangent[:, 0]])   # points left
        self.heading = np.arctan2(self.tangent[:, 1], self.tangent[:, 0])
        self.curvature = _curvature(self.centre, k=4)                               # >0 = turning left

    @property
    def n(self):
        return len(self.centre)

    @property
    def left_edge(self):
        return self.centre + self.normal * self.half_left[:, None]

    @property
    def right_edge(self):
        return self.centre - self.normal * self.half_right[:, None]

    @property
    def width(self):
        return self.half_left + self.half_right

    def offset_line(self, side, offset):
        """Points `offset` m outside the edge on side +1 (left) or -1 (right)."""
        half = self.half_left if side > 0 else self.half_right
        return self.centre + side * self.normal * (half + offset)[:, None]

    def index_at(self, s):
        return int(np.searchsorted(self.s, s % self.length, side="right") - 1) % self.n

    def bounds(self, margin=0.0):
        pts = np.vstack([self.left_edge, self.right_edge])
        return pts.min(axis=0) - margin, pts.max(axis=0) + margin

    # ------------------------------------------------------------------ JSON
    def to_dict(self):
        return {
            "format": FORMAT_VERSION,
            "seed": self.seed,
            "length_m": round(self.length, 2),
            "start_pose": list(self.start_pose),
            "centre": np.round(self.centre, 4).tolist(),
            "half_left": np.round(self.half_left, 4).tolist(),
            "half_right": np.round(self.half_right, 4).tolist(),
            "sections": [asdict(s) for s in self.sections],
            "obstacles": [asdict(o) for o in self.obstacles],
        }

    @classmethod
    def from_dict(cls, d):
        if d.get("format") != FORMAT_VERSION:
            raise ValueError(f"unsupported track format {d.get('format')!r}")
        return cls(centre=d["centre"], half_left=d["half_left"], half_right=d["half_right"],
                   start_pose=d["start_pose"], seed=d.get("seed"),
                   sections=[Section(**s) for s in d["sections"]],
                   obstacles=[Obstacle(**o) for o in d["obstacles"]])

    def save(self, path):
        with open(path, "w") as f:
            json.dump(self.to_dict(), f)

    @classmethod
    def load(cls, path):
        with open(path) as f:
            return cls.from_dict(json.load(f))


def _curvature(P, k):
    """Signed curvature of the circle through points i-k, i, i+k (about 2 m apart),
    so sampling noise in a 0.5 m polyline does not read as a tight corner."""
    a, b, c = np.roll(P, k, axis=0), P, np.roll(P, -k, axis=0)
    ab, bc, ac = b - a, c - b, c - a
    cross = ab[:, 0] * bc[:, 1] - ab[:, 1] * bc[:, 0]
    den = np.linalg.norm(ab, axis=1) * np.linalg.norm(bc, axis=1) * np.linalg.norm(ac, axis=1)
    return 2 * cross / np.maximum(den, 1e-12)


def yaw_quat(yaw):
    """MuJoCo quaternion (w, x, y, z) for a rotation of yaw about z."""
    return [math.cos(yaw / 2), 0.0, 0.0, math.sin(yaw / 2)]
