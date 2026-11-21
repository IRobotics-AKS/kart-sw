import math

import numpy as np
import pytest

from sim.camplace import track
from sim.camplace.config import BANDS
from sim.camplace.geometry import cam_rot
from sim.camplace.kart import Kart
from sim.camplace.lap import edge_points


@pytest.fixture(scope="module")
def kart():
    return Kart()


@pytest.mark.parametrize("yaw,pitch", [(0, 0), (0, 6.25), (60, 25), (-65, 25), (90, 10)])
def test_view_axis_projects_to_image_centre(yaw, pitch):
    R = cam_rot(yaw, pitch)
    pos = np.array([0.05, 0.3, 0.75])
    c = (pos + 5.0 * R[:, 2] - pos) @ R
    assert abs(c[0] / c[2]) < 1e-12 and abs(c[1] / c[2]) < 1e-12
    # yaw + turns the view to the left (+y), pitch + tilts it down (-z)
    if yaw:
        assert np.sign(R[1, 2]) == np.sign(yaw)
    if pitch:
        assert R[2, 2] < 0


def test_camera_axes_are_right_handed():
    R = cam_rot(37, 12)
    assert np.allclose(R.T @ R, np.eye(3), atol=1e-12)
    assert np.allclose(np.cross(R[:, 0], R[:, 1]), R[:, 2], atol=1e-12)
    assert R[2, 1] < 0          # image "down" points down in the kart frame


def test_left_and_right_cameras_mirror(kart):
    occ_l, cl = kart.occ_map((0.05, 0.30, 0.75), "left")
    occ_r, cr = kart.occ_map((0.05, -0.30, 0.75), "right")
    bl = kart.image_blocked(occ_l, 60, 25, "oak1_lite")
    br = kart.image_blocked(occ_r, -60, 25, "oak1_lite")
    assert abs(bl - br) < 0.03          # the kart is only slightly asymmetric
    assert abs(cl - cr) < 0.05


def test_camera_inside_steering_motor_is_rejected(kart):
    _, clearance = kart.occ_map((0.755, 0.0, 0.415), "front")
    assert clearance == 0.0


def test_purdue_edge_points_match_original_study():
    P = edge_points(track.load("purdue"))
    assert P["nframes"] == 430
    assert len(P["q"]) == 92020         # same as the original aks-gps scripts/cam/refine.py
    assert set(np.unique(P["band"])) == set(range(len(BANDS)))


def test_generated_track_loads():
    c = track.load("kart_s104")
    P = edge_points(c)
    assert len(c.lines) == 3 and P["nframes"] > 0
    assert 5.0 <= c.width.min() and c.width.max() <= 9.0
    assert set(np.unique(P["side"])) == {0, 1}


def test_blocking_map_matches_render():
    """The ray map's kart pixels agree with MuJoCo's segmentation render on >95 % of pixels."""
    mujoco = pytest.importorskip("mujoco")
    from sim.camplace.config import CAM_OF, CAMS, KART_XML, SETS
    from sim.camplace.geometry import pixel_grid
    from sim.camplace.kart import lookup_idx
    W, H = 160, 128
    cfgs = SETS["crossbar"]
    spec = mujoco.MjSpec.from_file(KART_XML)
    for name, c in cfgs.items():
        R = cam_rot(c[3], c[4])
        hf = CAMS[CAM_OF[name]][0]
        fovy = math.degrees(2 * math.atan(math.tan(math.radians(hf / 2)) * H / W))
        q = np.zeros(4)
        mujoco.mju_mat2Quat(q, np.column_stack([R[:, 0], -R[:, 1], -R[:, 2]]).ravel())
        spec.worldbody.add_camera(name=f"cam_{name}", pos=list(c[:3]), quat=q, fovy=fovy)
    m = spec.compile()
    d = mujoco.MjData(m)
    mujoco.mj_forward(m, d)
    try:
        rend = mujoco.Renderer(m, H, W)
    except Exception as e:      # no OpenGL context on this machine
        pytest.skip(f"no GL: {e}")
    rend.enable_segmentation_rendering()
    floor = m.geom("floor").id
    k = Kart()
    for name, c in cfgs.items():
        rend.update_scene(d, camera=f"cam_{name}")
        seg = rend.render()[..., 0]
        kart_px = (seg >= 0) & (seg != floor)
        occ, _ = k.occ_map(c[:3], name)
        hf = CAMS[CAM_OF[name]][0]
        vf = math.degrees(2 * math.atan(math.tan(math.radians(hf / 2)) * H / W))
        dk, _, _ = pixel_grid(cam_rot(c[3], c[4]), hf, vf, W, H)
        ie, ia = lookup_idx(dk)
        mask = occ[3, ie, ia].reshape(H, W)
        assert (mask == kart_px).mean() > 0.95, name
    rend.close()
