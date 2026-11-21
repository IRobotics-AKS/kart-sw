"""The CAD kart in MuJoCo: which view directions it blocks from a camera position, and clearance."""
import numpy as np

from .config import AZ, AZ_WIN, CAMS, EL, KART_XML, RES, STEER_BINS
from .geometry import cam_rot, pixel_grid


def ray_dirs():
    a, e = np.meshgrid(np.radians(AZ), np.radians(EL), indexing="xy")
    return np.stack([np.cos(e) * np.cos(a), np.cos(e) * np.sin(a), np.sin(e)], -1).reshape(-1, 3)


def lookup_idx(d):
    """Map directions (N,3) to (elevation, azimuth) indices of the blocking map."""
    az = np.degrees(np.arctan2(d[:, 1], d[:, 0]))
    el = np.degrees(np.arctan2(d[:, 2], np.hypot(d[:, 0], d[:, 1])))
    ia = np.round((az + 180) / RES).astype(int) % len(AZ)
    ie = np.clip(np.round((el - EL[0]) / RES).astype(int), 0, len(EL) - 1)
    return ie, ia


class Kart:
    """bins: indices into STEER_BINS to cast (the steering angles the lap uses, plus full lock)."""

    def __init__(self, bins=None, xml=KART_XML):
        import mujoco
        self.mj = mujoco
        self.m = mujoco.MjModel.from_xml_path(xml)
        self.d = mujoco.MjData(self.m)
        self.dirs = ray_dirs()
        rs = np.random.default_rng(0).normal(size=(4000, 3))
        self.ball = rs / np.linalg.norm(rs, axis=1, keepdims=True)
        a = np.tile(AZ, len(EL))
        self.win = {w: np.flatnonzero((a >= lo) & (a <= hi)) for w, (lo, hi) in AZ_WIN.items()}
        self.bins = sorted(set(bins if bins is not None else range(len(STEER_BINS))) | {0, len(STEER_BINS) - 1})
        self.jl = self.m.joint("front_left_steer").qposadr[0]
        self.jr = self.m.joint("front_right_steer").qposadr[0]

    def _pose(self, steer=0.0):
        self.d.qpos[:] = self.m.qpos0
        self.d.qpos[self.jl] = self.d.qpos[self.jr] = steer
        self.mj.mj_kinematics(self.m, self.d)

    def occ_map(self, pos, which):
        """-> (blocked[steer_bin, el, az] bool, clearance m). Only directions camera `which` can
        look in are cast. Clearance = nearest kart surface in any direction, wheels straight
        (0 when the point is inside a closed part)."""
        mj, m, d = self.mj, self.m, self.d
        win = self.win[which]
        D = np.ascontiguousarray(self.dirs[win])
        n = len(D)
        occ = np.zeros((len(STEER_BINS), len(EL) * len(AZ)), bool)
        gid, dist = np.zeros(n, np.int32), np.zeros(n)
        p = np.asarray(pos, float)
        for k in self.bins:
            self._pose(STEER_BINS[k])
            mj.mj_multiRay(m, d, p, D.ravel(), None, 1, 0, gid, dist, None, n, 30.0)
            occ[k, win] = gid >= 0
        self._pose(0.0)
        B = self.ball
        gb, db = np.zeros(len(B), np.int32), np.zeros(len(B))
        mj.mj_multiRay(m, d, p, B.ravel(), None, 1, 0, gb, db, None, len(B), 30.0)
        mind = float(db[gb >= 0].min()) if (gb >= 0).any() else np.inf
        if (gb >= 0).mean() > 0.5:
            mind = 0.0
        return occ.reshape(len(STEER_BINS), len(EL), len(AZ)), mind

    def image_blocked(self, occ, yaw, pitch, cam):
        """Share of the image the kart fills, worst steering angle up to full lock."""
        hfov, vfov = CAMS[cam]
        dk, _, _ = pixel_grid(cam_rot(yaw, pitch), hfov, vfov, 64, 48)
        ie, ia = lookup_idx(dk)
        return float(occ[self.bins][:, ie, ia].mean(axis=1).max())
