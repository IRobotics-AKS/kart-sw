"""Model loading, camera layout, and basic driving. Run: pytest simulation/tests"""
import math

import mujoco
import numpy as np
import pytest

from sim import scene, track


@pytest.fixture(scope="module")
def world():
    info = track.write_world(scene.asset_path("worlds"))
    model, data = scene.load()
    return model, data, info


def test_three_cameras_with_expected_yaw(world):
    model, data, _ = world
    scene.reset(model, data, xy=(0, 0), yaw=0.0)
    expected = {"cam_front": 0.0, "cam_left": 45.0, "cam_right": -45.0}
    for name, yaw_deg in expected.items():
        i = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_CAMERA, name)
        look = -data.cam_xmat[i].reshape(3, 3)[:, 2]
        assert math.degrees(math.atan2(look[1], look[0])) == pytest.approx(yaw_deg, abs=0.1)


def test_track_does_not_overlap_itself(world):
    _, _, info = world
    assert info["min_gap_m"] > track.TRACK_WIDTH * 2


def test_kart_drives_forward_and_stays_on_ground(world):
    model, data, info = world
    scene.reset(model, data, xy=info["start_xy"], yaw=info["start_yaw"])
    scene.set_controls(data, steer_rad=0.0, speed_mps=3.0)
    for _ in range(int(3.0 / model.opt.timestep)):
        mujoco.mj_step(model, data)
    assert scene.forward_speed(model, data) == pytest.approx(3.0, abs=0.3)
    assert 0.10 < data.qpos[2] < 0.25  # not sunk, not flipped or airborne


def test_reset_is_reproducible(world):
    model, data, info = world
    runs = []
    for _ in range(2):
        scene.reset(model, data, xy=info["start_xy"], yaw=info["start_yaw"])
        scene.set_controls(data, steer_rad=0.2, speed_mps=3.0)
        for _ in range(500):
            mujoco.mj_step(model, data)
        runs.append(data.qpos.copy())
    assert np.allclose(runs[0], runs[1])
