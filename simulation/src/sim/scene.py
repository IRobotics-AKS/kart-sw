"""Load, reset, drive, and render the kart scene. No ROS imports."""
from __future__ import annotations

import math
import struct
import sys
import zlib
from pathlib import Path

import mujoco
import numpy as np

CAMERAS = ("cam_front", "cam_left", "cam_right")


def asset_path(relative: str) -> Path:
    """Find a packaged asset in the source tree, else in the installed share dir."""
    source = Path(__file__).resolve().parents[2] / relative
    if source.exists():
        return source
    installed = Path(sys.prefix) / "share" / "sim" / relative
    if installed.exists():
        return installed
    raise FileNotFoundError(f"asset not found: {relative}")


def load(world: str = "worlds/track_demo.xml") -> tuple[mujoco.MjModel, mujoco.MjData]:
    model = mujoco.MjModel.from_xml_path(str(asset_path(world)))
    data = mujoco.MjData(model)
    reset(model, data)
    return model, data


def reset(model: mujoco.MjModel, data: mujoco.MjData,
          xy=(0.0, 0.0), yaw: float = 0.0) -> None:
    """Put the kart at xy facing yaw (radians), at rest."""
    mujoco.mj_resetData(model, data)
    data.qpos[0:2] = xy
    data.qpos[3:7] = (math.cos(yaw / 2), 0.0, 0.0, math.sin(yaw / 2))  # w x y z
    data.ctrl[:] = 0.0
    mujoco.mj_forward(model, data)


def set_controls(data: mujoco.MjData, steer_rad: float, speed_mps: float,
                 wheel_radius: float = 0.13) -> None:
    """steer_rad: front wheel angle (left positive). speed_mps: target forward speed."""
    wheel_speed = speed_mps / wheel_radius
    data.ctrl[0] = data.ctrl[1] = steer_rad
    data.ctrl[2] = data.ctrl[3] = wheel_speed


def forward_speed(model: mujoco.MjModel, data: mujoco.MjData) -> float:
    """Kart speed along its own +X axis, m/s.

    Uses the free joint's world-frame velocity and the body's rotation. Do not use
    mj_objectVelocity for this: it reports in the inertial frame, whose axes follow
    the principal inertia directions and need not point along the kart's +X.
    """
    body = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "kart")
    world_v = data.qvel[0:3]  # free joint: linear velocity in the world frame
    rot = data.xmat[body].reshape(3, 3)  # body frame axes as columns, in world
    return float((rot.T @ world_v)[0])


def render(renderer: mujoco.Renderer, data: mujoco.MjData, camera: str) -> np.ndarray:
    renderer.update_scene(data, camera=camera)
    return renderer.render().copy()


def write_png(path: Path, rgb: np.ndarray) -> None:
    """Minimal PNG writer so rendering needs no imaging dependency."""
    h, w, _ = rgb.shape
    raw = b"".join(b"\x00" + rgb[y].tobytes() for y in range(h))

    def chunk(tag: bytes, body: bytes) -> bytes:
        crc = zlib.crc32(tag + body) & 0xFFFFFFFF
        return struct.pack(">I", len(body)) + tag + body + struct.pack(">I", crc)

    png = (b"\x89PNG\r\n\x1a\n"
           + chunk(b"IHDR", struct.pack(">IIBBBBB", w, h, 8, 2, 0, 0, 0))
           + chunk(b"IDAT", zlib.compress(raw))
           + chunk(b"IEND", b""))
    Path(path).write_bytes(png)
