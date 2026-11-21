"""Camera rotations, kart tilt and image-plane helpers."""
import math

import numpy as np

from .config import CG


def cam_rot(yaw_deg, pitch_deg):
    """Columns = the camera's x (image right), y (image down), z (view) axes in the kart frame.
    yaw + = turned to the left, pitch + = tilted down."""
    y, p = math.radians(yaw_deg), math.radians(pitch_deg)
    f = np.array([math.cos(p) * math.cos(y), math.cos(p) * math.sin(y), -math.sin(p)])
    r = np.array([math.sin(y), -math.cos(y), 0.0])
    return np.column_stack([r, np.cross(f, r), f])


def tilt_rot(pitch_deg, roll_deg):
    p, r = math.radians(pitch_deg), math.radians(roll_deg)
    Ry = np.array([[math.cos(p), 0, math.sin(p)], [0, 1, 0], [-math.sin(p), 0, math.cos(p)]])
    Rx = np.array([[1, 0, 0], [0, math.cos(r), -math.sin(r)], [0, math.sin(r), math.cos(r)]])
    return Ry @ Rx


def tilted(q, nrm, pitch_deg, roll_deg):
    """Ground points and edge normals as seen from a kart pitched (nose down) and rolled about its CG."""
    if not (pitch_deg or roll_deg):
        return q, nrm
    Rt = tilt_rot(pitch_deg, roll_deg)
    return CG + (q - CG) @ Rt, nrm @ Rt


def pixel_grid(R, hfov, vfov, nu=48, nv=36):
    """Unit view directions (kart frame) on an nu x nv grid across the image."""
    tx, ty = math.tan(math.radians(hfov / 2)), math.tan(math.radians(vfov / 2))
    u, w = np.meshgrid(np.linspace(-tx, tx, nu), np.linspace(-ty, ty, nv))
    dc = np.stack([u, w, np.ones_like(u)], -1).reshape(-1, 3)
    dk = dc @ R.T
    return dk / np.linalg.norm(dk, axis=1, keepdims=True), u.ravel(), w.ravel()
