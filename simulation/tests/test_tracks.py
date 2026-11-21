"""Track generator and world tests. Run from the repository root:

    .venv/bin/python -m pytest simulation/tests/test_tracks.py
"""
import math

import mujoco
import numpy as np
import pytest

from sim.tracks import EDGE_STYLES, Params, Track, check, generate, world

SEEDS = range(20)


@pytest.fixture(scope="module")
def tracks():
    return [generate(s, obstacles=4, moving=1) for s in SEEDS]


def test_same_seed_same_track():
    a, b = generate(11, obstacles=3, moving=1), generate(11, obstacles=3, moving=1)
    assert np.array_equal(a.centre, b.centre)
    assert a.sections == b.sections and a.obstacles == b.obstacles
    assert not np.array_equal(a.centre[:10], generate(12).centre[:10])


def test_tracks_are_valid_and_realistic(tracks):
    p = Params()
    for t in tracks:
        assert check(t, p) == [], t.seed
        assert p.length[0] - 1 <= t.length <= p.length[1] + 1
        radius = 1 / np.maximum(np.abs(t.curvature), 1e-9)
        assert radius.min() >= p.min_radius
        assert p.width[0] <= t.width.min() and t.width.max() <= p.width[1]


def test_start_pose_on_track_behind_line(tracks):
    for t in tracks:
        x, y, yaw = t.start_pose
        assert np.hypot(*(t.centre[t.index_at(-3.0)] - [x, y])) < 1e-6
        assert abs(math.remainder(yaw - t.heading[0], 2 * math.pi)) < 0.2


def test_sections_cover_the_lap(tracks):
    for t in tracks:
        assert t.sections[0].s0 == 0 and abs(t.sections[-1].s1 - t.length) < 1e-6
        for a, b in zip(t.sections, t.sections[1:]):
            assert a.s1 == b.s0


def test_obstacles_on_the_surface(tracks):
    for t in tracks:
        assert len(t.obstacles) == 4 and sum(o.moving for o in t.obstacles) == 1
        for o in t.obstacles:
            j = int(np.argmin(np.hypot(*(t.centre - [o.x, o.y]).T)))
            u = float(np.dot([o.x, o.y] - t.centre[j], t.normal[j]))
            reach = o.amplitude + 0.2
            assert -t.half_right[j] < u - reach and u + reach < t.half_left[j]
            assert min(t.s[j], t.length - t.s[j]) >= 20


def test_obstacle_zone_is_a_passable_slalom(tracks):
    for t in tracks:
        s = sorted(t.s[int(np.argmin(np.hypot(*(t.centre - [o.x, o.y]).T)))] for o in t.obstacles)
        gaps = np.diff(s)
        assert ((gaps > 5) & (gaps < 11)).all(), (t.seed, gaps)
        for o in t.obstacles:
            if o.moving:
                continue
            j = int(np.argmin(np.hypot(*(t.centre - [o.x, o.y]).T)))
            u = float(np.dot([o.x, o.y] - t.centre[j], t.normal[j]))
            free = max(t.half_left[j] - u, u + t.half_right[j]) - 0.3
            assert free >= 2.0, (t.seed, free)       # the kart (about 1.4 m) fits past


def test_both_layouts_and_spread_obstacles():
    for layout in ("kart", "flowing"):
        t = generate(4, layout=layout, obstacles=3, zone=False)
        assert check(t) == []
        s = sorted(t.s[int(np.argmin(np.hypot(*(t.centre - [o.x, o.y]).T)))] for o in t.obstacles)
        assert (np.diff(s) > 14).all()


def test_json_round_trip(tmp_path):
    t = generate(3, obstacles=2, moving=1)
    t.save(tmp_path / "t.json")
    u = Track.load(tmp_path / "t.json")
    assert np.allclose(t.centre, u.centre, atol=1e-4)
    assert t.sections == u.sections and t.obstacles == u.obstacles
    assert u.start_pose == pytest.approx(t.start_pose)


@pytest.mark.parametrize("edges", ("mixed",) + EDGE_STYLES)
def test_world_compiles_for_every_edge_style(edges):
    model, data = world.compile(generate(5, edges=edges, obstacles=2))
    assert model.body("chassis").id > 0 and model.geom("track_surface").id >= 0


def test_kart_settles_at_start():
    t = generate(8)
    model, data = world.compile(t)
    for _ in range(int(1.0 / model.opt.timestep)):
        mujoco.mj_step(model, data)
    x, y, _ = t.start_pose
    pos = data.body("chassis").xpos
    assert np.hypot(pos[0] - x, pos[1] - y) < 0.05
    assert 0.05 < pos[2] < 0.3
    assert np.linalg.norm(data.qvel[:3]) < 0.05


def test_barriers_and_obstacles_collide_markings_do_not():
    model, _ = world.compile(generate(4, edges="plastic", obstacles=3))
    floor = model.geom("floor")
    for g in range(model.ngeom):
        name = model.geom(g).name
        if name in ("track_surface", "start_line", "grass"):
            assert model.geom_contype[g] == 0 and model.geom_conaffinity[g] == 0
    plastic = [g for g in range(model.ngeom)
               if model.geom_type[g] == mujoco.mjtGeom.mjGEOM_BOX and model.geom_bodyid[g] == 0
               and model.geom_size[g][0] == pytest.approx(0.6)]
    assert plastic and all(model.geom_contype[g] == floor.contype[0] for g in plastic)


def test_moving_obstacle_moves():
    model, data = world.compile(generate(6, obstacles=1, moving=1))
    body = next(b for b in range(model.nbody) if model.body(b).name.startswith("obstacle"))
    m = model.body_mocapid[body]
    world.update(model, data)
    p0 = data.mocap_pos[m].copy()
    data.time = 1.3
    world.update(model, data)
    assert np.linalg.norm(data.mocap_pos[m] - p0) > 0.05
