"""Drive the kart forward briefly and save the three camera views plus an overview.

    python simulation/scripts/render_cameras.py [out_dir]

Linux headless: set MUJOCO_GL=egl (or osmesa) first. macOS needs nothing extra
because this renders offscreen and opens no window.
"""
from __future__ import annotations

import sys
from pathlib import Path

import mujoco

from sim import scene
from sim.track import write_world


def main() -> None:
    out = Path(sys.argv[1] if len(sys.argv) > 1 else "render_out")
    out.mkdir(parents=True, exist_ok=True)

    model, data = scene.load()
    info = write_world(Path(scene.asset_path("worlds")))  # also returns the start pose
    scene.reset(model, data, xy=info["start_xy"], yaw=info["start_yaw"])

    scene.set_controls(data, steer_rad=0.0, speed_mps=4.0)
    for _ in range(int(3.0 / model.opt.timestep)):  # 3 simulated seconds
        mujoco.mj_step(model, data)
    print(f"speed after 3 s: {scene.forward_speed(model, data):.2f} m/s")

    renderer = mujoco.Renderer(model, height=480, width=640)
    for cam in (*scene.CAMERAS, "overview"):
        scene.write_png(out / f"{cam}.png", scene.render(renderer, data, cam))
        print("wrote", out / f"{cam}.png")


if __name__ == "__main__":
    main()
