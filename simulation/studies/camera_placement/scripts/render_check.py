"""Sanity check of the kart-blocking map: render each camera's view of the kart with MuJoCo and
tint red every pixel the ray map says the kart blocks. Red should cover the kart exactly.

  python simulation/studies/camera_placement/scripts/render_check.py --set crossbar --out runs/render_check.png
"""
import argparse
import math
import os

import numpy as np

from sim.camplace.config import CAM_OF, CAMS, KART_XML, SETS
from sim.camplace.geometry import cam_rot, pixel_grid
from sim.camplace.kart import Kart, lookup_idx

W, H = 400, 320


def render_views(cfgs):
    """-> list of (name, rendered image HxWx3, blocked mask HxW) for each camera, wheels straight."""
    import mujoco
    spec = mujoco.MjSpec.from_file(KART_XML)
    for name, c in cfgs.items():
        R = cam_rot(c[3], c[4])
        hf = CAMS[CAM_OF[name]][0]
        fovy = math.degrees(2 * math.atan(math.tan(math.radians(hf / 2)) * H / W))
        q = np.zeros(4)
        mujoco.mju_mat2Quat(q, np.column_stack([R[:, 0], -R[:, 1], -R[:, 2]]).ravel())   # MuJoCo cams look along -z
        spec.worldbody.add_camera(name=f"cam_{name}", pos=list(c[:3]), quat=q, fovy=fovy)
    m = spec.compile()
    d = mujoco.MjData(m)
    mujoco.mj_forward(m, d)
    rend = mujoco.Renderer(m, H, W)
    kart = Kart()
    out = []
    for name, c in cfgs.items():
        rend.update_scene(d, camera=f"cam_{name}")
        img = rend.render().copy()
        occ, _ = kart.occ_map(c[:3], name)
        hf = CAMS[CAM_OF[name]][0]
        vf = math.degrees(2 * math.atan(math.tan(math.radians(hf / 2)) * H / W))
        dk, _, _ = pixel_grid(cam_rot(c[3], c[4]), hf, vf, W, H)
        ie, ia = lookup_idx(dk)
        mask = occ[3, ie, ia].reshape(H, W)            # bin 3 = wheels straight
        out.append((name, img, mask))
    rend.close()
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--set", default="crossbar", choices=sorted(SETS))
    ap.add_argument("--out", default="runs/render_check.png")
    args = ap.parse_args()
    from PIL import Image
    views = render_views(SETS[args.set])
    tiles = []
    for name, img, mask in views:
        over = img.copy()
        over[mask] = (0.5 * over[mask] + [127, 0, 0]).astype(np.uint8)
        tiles.append(np.vstack([img, over]))
        print(f"{name}: blocking map marks {mask.mean() * 100:.1f}% of the image as kart")
    os.makedirs(os.path.dirname(args.out) or ".", exist_ok=True)
    Image.fromarray(np.hstack([np.pad(t, ((0, 0), (0, 8), (0, 0))) for t in tiles])).save(args.out)
    print("wrote", args.out, "(top row: render, bottom row: render with the blocking map in red)")


if __name__ == "__main__":
    main()
