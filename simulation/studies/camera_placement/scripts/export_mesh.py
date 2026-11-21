"""Export the CAD kart's visual geometry (as MuJoCo places it, wheels straight) to a compact
JSON for the 3D viewer in the camera report: one triangle mesh per colour, kart frame
(rear axle centre on the ground, x forward, y left, z up), base64 float32/uint32.

  python simulation/studies/camera_placement/scripts/export_mesh.py runs/kart.json
"""
import base64
import json
import os
import sys

import mujoco
import numpy as np

from sim.camplace.config import KART_XML


def uv_sphere(nu=20, nv=12):
    th, ph = np.meshgrid(np.linspace(0, 2 * np.pi, nu, endpoint=False), np.linspace(0, np.pi, nv))
    V = np.stack([np.sin(ph) * np.cos(th), np.sin(ph) * np.sin(th), np.cos(ph)], -1).reshape(-1, 3)
    F = []
    for j in range(nv - 1):
        for i in range(nu):
            a, b = j * nu + i, j * nu + (i + 1) % nu
            c, d = a + nu, b + nu
            F += [(a, c, b), (b, c, d)]
    return V, np.array(F)


def cylinder(n=28):
    a = np.linspace(0, 2 * np.pi, n, endpoint=False)
    ring = np.stack([np.cos(a), np.sin(a)], 1)
    V = np.vstack([np.c_[ring, -np.ones(n)], np.c_[ring, np.ones(n)], [[0, 0, -1], [0, 0, 1]]])
    F = []
    for i in range(n):
        j = (i + 1) % n
        F += [(i, j, n + i), (j, n + j, n + i), (2 * n, j, i), (2 * n + 1, n + i, n + j)]
    return V, np.array(F)


def box():
    V = np.array([[x, y, z] for x in (-1, 1) for y in (-1, 1) for z in (-1, 1)], float)
    F = [(0, 1, 3), (0, 3, 2), (4, 6, 7), (4, 7, 5), (0, 4, 5), (0, 5, 1),
         (2, 3, 7), (2, 7, 6), (0, 2, 6), (0, 6, 4), (1, 5, 7), (1, 7, 3)]
    return V, np.array(F)


def main(out):
    m = mujoco.MjModel.from_xml_path(KART_XML)
    d = mujoco.MjData(m)
    mujoco.mj_forward(m, d)
    S, C, B = uv_sphere(), cylinder(), box()
    groups = {}
    G = mujoco.mjtGeom
    for g in range(m.ngeom):
        if m.geom_group[g] not in (0, 1) or m.geom_type[g] == G.mjGEOM_PLANE:
            continue
        t, sz = m.geom_type[g], m.geom_size[g]
        if t == G.mjGEOM_MESH:
            mid = m.geom_dataid[g]
            V = m.mesh_vert[m.mesh_vertadr[mid]: m.mesh_vertadr[mid] + m.mesh_vertnum[mid]].astype(float)
            F = m.mesh_face[m.mesh_faceadr[mid]: m.mesh_faceadr[mid] + m.mesh_facenum[mid]]
        elif t == G.mjGEOM_BOX:
            V, F = B[0] * sz, B[1]
        elif t in (G.mjGEOM_ELLIPSOID, G.mjGEOM_SPHERE):
            V, F = S[0] * (sz if t == G.mjGEOM_ELLIPSOID else sz[0]), S[1]
        elif t == G.mjGEOM_CYLINDER:
            V, F = C[0] * [sz[0], sz[0], sz[1]], C[1]
        elif t == G.mjGEOM_CAPSULE:
            V1, F1 = C[0] * [sz[0], sz[0], sz[1]], C[1]
            caps = [S[0] * sz[0] + [0, 0, s * sz[1]] for s in (-1, 1)]
            V = np.vstack([V1] + caps)
            n1, ns = len(V1), len(S[0])
            F = np.vstack([F1, S[1] + n1, S[1] + n1 + ns])
        else:
            continue
        W = d.geom_xpos[g] + V @ d.geom_xmat[g].reshape(3, 3).T
        rgba = m.mat_rgba[m.geom_matid[g]] if m.geom_matid[g] >= 0 else m.geom_rgba[g]
        key = (tuple(float(v) for v in np.round(rgba[:3], 3)), int(m.geom_group[g]))
        gr = groups.setdefault(key, {"V": [], "F": [], "n": 0})
        gr["V"].append(W)
        gr["F"].append(np.asarray(F) + gr["n"])
        gr["n"] += len(W)
    meshes = []
    for (rgb, grp), gr in groups.items():
        V = np.vstack(gr["V"]).astype(np.float32)
        F = np.vstack(gr["F"]).astype(np.uint32)
        meshes.append({"rgb": [float(v) for v in rgb], "group": grp,
                       "pos": base64.b64encode(V.tobytes()).decode(),
                       "idx": base64.b64encode(F.tobytes()).decode()})
    with open(out, "w") as fh:
        json.dump({"frame": "rear axle centre on the ground, x forward, y left, z up (m)", "meshes": meshes}, fh)
    print(out, os.path.getsize(out) // 1024, "KB,", len(meshes), "meshes,",
          sum(len(g["F"]) and sum(len(f) for f in g["F"]) for g in groups.values()), "triangles")


if __name__ == "__main__":
    main(sys.argv[1] if len(sys.argv) > 1 else "runs/kart.json")
