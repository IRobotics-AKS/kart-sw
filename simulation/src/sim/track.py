"""Go-kart track generator.

Turns a closed list of centerline control points into a smooth loop, then writes
visual meshes (asphalt surface and painted edge lines) plus a scene MJCF that
includes the kart model. Run it with:

    python -m sim.track

The default layout is a generic kart-track shape (long straight, hairpin, esses),
NOT a survey of the AKS race track. To match the real track, replace
DEFAULT_CONTROL_POINTS with points traced from a map (meters, x east, y north)
and set TRACK_WIDTH to the measured width.

Collision is handled by the world's ground plane. The track meshes are visual
only, so surface friction is the same everywhere until you add zones.
"""
from __future__ import annotations

import math
from pathlib import Path

import numpy as np

TRACK_WIDTH = 6.0  # meters. PLACEHOLDER, measure the real track.
LINE_WIDTH = 0.15
START_NEAR = (25.0, 0.0)  # start/finish spot, on the main straight

# Closed loop, meters, driven in the listed order.
DEFAULT_CONTROL_POINTS = [
    (0, 0), (40, 0), (80, 0), (112, 10), (126, 36), (110, 62),
    (80, 66), (60, 54), (45, 70), (20, 82), (-10, 70), (-26, 45), (-16, 20),
]


def catmull_rom_closed(points: np.ndarray, samples_per_segment: int = 40) -> np.ndarray:
    """Smooth closed curve through the control points (uniform Catmull-Rom)."""
    n = len(points)
    out = []
    for i in range(n):
        p0, p1, p2, p3 = (points[(i + k) % n] for k in (-1, 0, 1, 2))
        for t in np.linspace(0.0, 1.0, samples_per_segment, endpoint=False):
            out.append(
                0.5 * ((2 * p1) + (-p0 + p2) * t
                       + (2 * p0 - 5 * p1 + 4 * p2 - p3) * t**2
                       + (-p0 + 3 * p1 - 3 * p2 + p3) * t**3)
            )
    return np.array(out)


def resample_closed(curve: np.ndarray, spacing: float = 1.0) -> np.ndarray:
    """Resample a closed polyline at roughly even arc length."""
    closed = np.vstack([curve, curve[:1]])
    seg = np.linalg.norm(np.diff(closed, axis=0), axis=1)
    s = np.concatenate([[0.0], np.cumsum(seg)])
    total = s[-1]
    count = max(int(round(total / spacing)), 8)
    targets = np.linspace(0.0, total, count, endpoint=False)
    x = np.interp(targets, s, closed[:, 0])
    y = np.interp(targets, s, closed[:, 1])
    return np.column_stack([x, y])


def centerline(control_points=DEFAULT_CONTROL_POINTS, spacing: float = 1.0) -> np.ndarray:
    pts = np.asarray(control_points, dtype=float)
    return resample_closed(catmull_rom_closed(pts), spacing)


def tangents_and_normals(line: np.ndarray):
    t = np.roll(line, -1, axis=0) - np.roll(line, 1, axis=0)
    t /= np.linalg.norm(t, axis=1, keepdims=True)
    n = np.column_stack([-t[:, 1], t[:, 0]])  # left of travel
    return t, n


def min_nonadjacent_gap(line: np.ndarray, skip_arc: float = 30.0, spacing: float = 1.0) -> float:
    """Smallest centerline distance between parts of the track far apart along it.

    If this is below TRACK_WIDTH the track overlaps itself.
    """
    skip = int(skip_arc / spacing)
    n = len(line)
    best = np.inf
    for i in range(n):
        d = np.linalg.norm(line - line[i], axis=1)
        idx = np.arange(n)
        sep = np.minimum(np.abs(idx - i), n - np.abs(idx - i))
        d = d[sep > skip]
        if len(d):
            best = min(best, d.min())
    return float(best)


def _strip_obj(left: np.ndarray, right: np.ndarray, z: float, name: str) -> str:
    n = len(left)
    lines = []
    for i in range(n):
        lines.append(f"v {left[i, 0]:.4f} {left[i, 1]:.4f} {z:.4f}")
        lines.append(f"v {right[i, 0]:.4f} {right[i, 1]:.4f} {z:.4f}")
    for i in range(n):
        j = (i + 1) % n
        li, ri, lj, rj = 2 * i + 1, 2 * i + 2, 2 * j + 1, 2 * j + 2  # OBJ is 1-indexed
        lines.append(f"f {ri} {lj} {li}")
        lines.append(f"f {ri} {rj} {lj}")
    return "\n".join(lines) + "\n"


def write_world(out_dir: Path, control_points=DEFAULT_CONTROL_POINTS,
                width: float = TRACK_WIDTH) -> dict:
    out_dir = Path(out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    line = centerline(control_points)
    t, n = tangents_and_normals(line)
    half = width / 2

    surf_l, surf_r = line + n * half, line - n * half
    (out_dir / "track_surface.obj").write_text(_strip_obj(surf_l, surf_r, 0.01, "surface"))

    # Edge lines are written as one mesh: two thin strips, one per edge.
    def strip_lines(offset):
        a = line + n * (offset + LINE_WIDTH / 2)
        b = line + n * (offset - LINE_WIDTH / 2)
        return a, b

    parts = []
    base = 0
    for offset in (half - 0.1, -(half - 0.1)):
        a, b = strip_lines(offset)
        text = _strip_obj(a, b, 0.02, f"edge{base}")
        # Re-index faces for the second strip so both live in one OBJ file.
        if base:
            new = []
            for ln in text.splitlines():
                if ln.startswith("f "):
                    idx = [int(v) + base for v in ln[2:].split()]
                    ln = "f " + " ".join(map(str, idx))
                new.append(ln)
            text = "\n".join(new) + "\n"
        parts.append(text)
        base += 2 * len(line)
    (out_dir / "track_lines.obj").write_text("".join(parts))

    i0 = int(np.argmin(np.linalg.norm(line - np.asarray(START_NEAR), axis=1)))
    start = line[i0]
    heading = math.atan2(t[i0, 1], t[i0, 0])
    (out_dir / "track_demo.xml").write_text(_scene_xml())
    return {"start_xy": (float(start[0]), float(start[1])), "start_yaw": heading,
            "length_m": float(np.sum(np.linalg.norm(np.roll(line, -1, axis=0) - line, axis=1))),
            "min_gap_m": min_nonadjacent_gap(line)}


def _scene_xml() -> str:
    return """<!-- Generated by `python -m sim.track`. Edit sim/track.py, not this file. -->
<mujoco model="kart_track_demo">
  <compiler meshdir="." autolimits="true"/>
  <include file="../models/dreamer/dreamer.xml"/>

  <visual>
    <global offwidth="1280" offheight="960"/>
    <quality shadowsize="4096"/>
  </visual>

  <asset>
    <texture type="skybox" builtin="gradient" rgb1="0.55 0.7 0.9" rgb2="0.9 0.93 0.97"
             width="256" height="256"/>
    <material name="grass" rgba="0.25 0.5 0.22 1"/>
    <material name="asphalt" rgba="0.22 0.22 0.24 1"/>
    <material name="paint" rgba="0.95 0.95 0.95 1"/>
    <mesh name="track_surface" file="track_surface.obj" inertia="shell"/>
    <mesh name="track_lines" file="track_lines.obj" inertia="shell"/>
  </asset>

  <worldbody>
    <light pos="60 40 120" dir="-0.2 -0.2 -1" directional="true" diffuse="0.9 0.9 0.9"/>
    <geom name="ground" type="plane" size="400 400 0.1" material="grass"
          friction="1.0 0.005 0.0001"/>
    <geom name="track_surface" type="mesh" mesh="track_surface" material="asphalt"
          contype="0" conaffinity="0" group="1"/>
    <geom name="track_lines" type="mesh" mesh="track_lines" material="paint"
          contype="0" conaffinity="0" group="1"/>
    <camera name="overview" pos="50 40 190" xyaxes="1 0 0 0 1 0" fovy="55"/>
  </worldbody>
</mujoco>
"""


def main() -> None:
    out = Path(__file__).resolve().parents[2] / "worlds"
    info = write_world(out)
    print(f"wrote {out}")
    print(f"lap length {info['length_m']:.0f} m, min self-gap {info['min_gap_m']:.1f} m "
          f"(track width {TRACK_WIDTH} m)")
    print(f"start xy={info['start_xy']} yaw={info['start_yaw']:.3f} rad")


if __name__ == "__main__":
    main()
