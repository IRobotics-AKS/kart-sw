"""Geometric tracker, checked against closed forms and against its refusals.

Runs without ROS, which is the point of keeping algorithms/ import-free.
"""

import math

import pytest

from control.algorithms.pure_pursuit import (Pose, PathSample, TrackerConfig, cross_track_error,
                                             nearest_index, track)

# Representative of a kart but not measured from one. Tests assert relationships,
# never a number that depends on these values being right.
CONFIG = TrackerConfig(
    wheelbase_m=1.0,
    max_steer_rad=math.radians(30.0),
    lookahead_min_m=1.0,
    lookahead_max_m=6.0,
    lookahead_gain_s=0.5,
    max_cross_track_m=3.0,
)


def straight(length=20.0, step=0.5, speed=5.0, y=0.0):
    n = int(length / step) + 1
    return [PathSample(i * step, y, speed) for i in range(n)]


def arc(radius, sweep_rad=math.pi / 2, n=60, speed=5.0):
    """Circular arc starting at the origin heading +X, curving left."""
    return [PathSample(radius * math.sin(t), radius * (1.0 - math.cos(t)), speed)
            for t in (sweep_rad * i / (n - 1) for i in range(n))]


# ----------------------------------------------------------------- config

def test_config_rejects_nonsense():
    for bad in ({"wheelbase_m": 0.0}, {"max_steer_rad": -0.1},
                {"lookahead_min_m": 0.0}, {"max_cross_track_m": -1.0}):
        with pytest.raises(ValueError):
            TrackerConfig(**{**CONFIG.__dict__, **bad})


def test_config_rejects_inverted_lookahead_band():
    with pytest.raises(ValueError):
        TrackerConfig(**{**CONFIG.__dict__, "lookahead_min_m": 5.0, "lookahead_max_m": 2.0})


def test_config_has_no_default_geometry():
    """Vehicle geometry must be supplied. A default here would bury a guess."""
    with pytest.raises(TypeError):
        TrackerConfig(max_steer_rad=0.5, lookahead_min_m=1.0, lookahead_max_m=6.0,
                      lookahead_gain_s=0.5, max_cross_track_m=3.0)


# ----------------------------------------------------------------- geometry helpers

def test_nearest_index_finds_the_closest_sample():
    path = straight()
    assert nearest_index(Pose(4.9, 0.3, 0.0), path) == 10      # 10 * 0.5 m = 5.0 m


def test_cross_track_error_is_positive_to_the_left():
    path = straight()
    assert cross_track_error(Pose(2.0, 0.4, 0.0), path, 4) == pytest.approx(0.4)
    assert cross_track_error(Pose(2.0, -0.4, 0.0), path, 4) == pytest.approx(-0.4)


def test_cross_track_error_stays_signed_at_the_final_sample():
    """The last sample has no outgoing segment; the sign must still mean something."""
    path = straight(length=2.0, step=1.0)
    assert cross_track_error(Pose(2.0, 0.5, 0.0), path, len(path) - 1) == pytest.approx(0.5)


# ----------------------------------------------------------------- straight line

def test_centred_on_a_straight_line_commands_no_steering():
    result = track(Pose(2.0, 0.0, 0.0), 5.0, straight(), CONFIG)
    assert result.valid
    assert result.steering_rad == pytest.approx(0.0, abs=1e-9)


def test_offset_right_steers_left_and_vice_versa():
    left = track(Pose(2.0, -0.5, 0.0), 5.0, straight(), CONFIG)
    right = track(Pose(2.0, 0.5, 0.0), 5.0, straight(), CONFIG)
    assert left.steering_rad > 0.0, "offset to the right should steer left"
    assert right.steering_rad < 0.0, "offset to the left should steer right"
    assert left.steering_rad == pytest.approx(-right.steering_rad)


def test_larger_offset_steers_harder():
    small = track(Pose(2.0, -0.2, 0.0), 5.0, straight(), CONFIG)
    large = track(Pose(2.0, -1.0, 0.0), 5.0, straight(), CONFIG)
    assert large.steering_rad > small.steering_rad > 0.0


# ----------------------------------------------------------------- lookahead

def test_lookahead_grows_with_speed_and_clamps():
    path = straight(length=60.0)
    slow = track(Pose(0.0, 0.0, 0.0), 0.0, path, CONFIG)
    # 1.0 + 0.5 * 20.0 = 11.0 m, well past the 6.0 m cap
    fast = track(Pose(0.0, 0.0, 0.0), 20.0, path, CONFIG)
    assert slow.lookahead_m == pytest.approx(CONFIG.lookahead_min_m)
    assert fast.lookahead_m == pytest.approx(CONFIG.lookahead_max_m)


def test_lookahead_follows_the_stated_law_between_the_limits():
    speed = 4.0
    expected = CONFIG.lookahead_min_m + CONFIG.lookahead_gain_s * speed
    result = track(Pose(0.0, 0.0, 0.0), speed, straight(length=60.0), CONFIG)
    assert result.lookahead_m == pytest.approx(expected)


def test_target_sits_about_a_lookahead_ahead():
    path = straight(length=60.0)
    result = track(Pose(0.0, 0.0, 0.0), 4.0, path, CONFIG)
    target = path[result.target_index]
    assert math.hypot(target.x_m, target.y_m) == pytest.approx(result.lookahead_m, abs=0.5)


# ----------------------------------------------------------------- curvature

def test_steering_matches_the_bicycle_model_on_an_arc():
    """On a circle of radius R the tracker should ask for about atan(L/R)."""
    radius = 10.0
    path = arc(radius)
    result = track(Pose(0.0, 0.0, 0.0), 2.0, path, CONFIG)
    assert result.valid
    assert result.steering_rad == pytest.approx(math.atan(CONFIG.wheelbase_m / radius), rel=0.15)


def test_tighter_arcs_need_more_steering():
    wide = track(Pose(0.0, 0.0, 0.0), 2.0, arc(20.0), CONFIG)
    tight = track(Pose(0.0, 0.0, 0.0), 2.0, arc(5.0), CONFIG)
    assert tight.steering_rad > wide.steering_rad > 0.0


def test_steering_saturates_at_the_configured_limit_and_says_so():
    result = track(Pose(0.0, 0.0, 0.0), 0.0, arc(0.6, n=40), CONFIG)
    assert result.valid
    assert result.steering_saturated
    assert abs(result.steering_rad) == pytest.approx(CONFIG.max_steer_rad)


# ----------------------------------------------------------------- speed reference

def test_speed_reference_comes_from_the_path():
    path = straight(speed=7.5)
    assert track(Pose(0.0, 0.0, 0.0), 3.0, path, CONFIG).speed_mps == pytest.approx(7.5)


def test_measured_speed_does_not_feed_back_into_the_reference():
    """There is no speed loop here; the reference must not depend on measured speed."""
    path = straight(length=60.0, speed=6.0)
    at_rest = track(Pose(0.0, 0.0, 0.0), 0.0, path, CONFIG)
    moving = track(Pose(0.0, 0.0, 0.0), 6.0, path, CONFIG)
    assert at_rest.speed_mps == moving.speed_mps == pytest.approx(6.0)


def test_curvature_limit_slows_the_reference_when_configured():
    limited = TrackerConfig(**{**CONFIG.__dict__, "max_lateral_accel_mps2": 2.0})
    radius = 4.0
    result = track(Pose(0.0, 0.0, 0.0), 2.0, arc(radius, n=80, speed=20.0), limited)
    assert result.valid
    assert result.speed_mps < 20.0
    assert result.speed_mps == pytest.approx(math.sqrt(2.0 * radius), rel=0.3)


def test_no_curvature_limit_leaves_the_reference_alone():
    result = track(Pose(0.0, 0.0, 0.0), 2.0, arc(4.0, n=80, speed=20.0), CONFIG)
    assert result.speed_mps == pytest.approx(20.0)


def test_negative_path_speed_is_clamped_rather_than_passed_through():
    """Reverse is not expressed as a negative reference here; that contract is unchosen."""
    path = [PathSample(i * 0.5, 0.0, -3.0) for i in range(40)]
    assert track(Pose(0.0, 0.0, 0.0), 1.0, path, CONFIG).speed_mps == 0.0


# ----------------------------------------------------------------- refusals

def test_a_short_path_is_refused():
    result = track(Pose(0.0, 0.0, 0.0), 1.0, [PathSample(0.0, 0.0, 1.0)], CONFIG)
    assert not result.valid and "two samples" in result.reason


def test_being_too_far_off_the_path_is_refused():
    result = track(Pose(2.0, 10.0, 0.0), 2.0, straight(), CONFIG)
    assert not result.valid and "cross-track" in result.reason


def test_a_path_entirely_behind_the_vehicle_is_refused():
    """Facing away from the path, steering toward it is not geometrically justified."""
    result = track(Pose(25.0, 0.0, 0.0), 2.0, straight(length=20.0), CONFIG)
    assert not result.valid and "behind" in result.reason


def test_non_finite_pose_is_refused():
    assert not track(Pose(math.nan, 0.0, 0.0), 1.0, straight(), CONFIG).valid


def test_negative_measured_speed_is_refused():
    assert not track(Pose(0.0, 0.0, 0.0), -1.0, straight(), CONFIG).valid


def test_an_invalid_result_carries_no_misleading_numbers():
    """Zero steering is a defined answer, not a safe one. Callers must read `valid`."""
    result = track(Pose(2.0, 10.0, 0.0), 2.0, straight(), CONFIG)
    assert not result.valid
    assert result.steering_rad == 0.0 and result.speed_mps == 0.0
    assert result.target_index == -1


# ----------------------------------------------------------------- end of path

def test_running_out_of_path_is_flagged_not_wrapped():
    """The path is open. Wrapping it would invent a lap the planner never declared."""
    path = straight(length=4.0)
    result = track(Pose(3.0, 0.0, 0.0), 5.0, path, CONFIG)
    assert result.valid
    assert result.path_exhausted
    assert result.target_index == len(path) - 1


def test_mid_path_is_not_flagged_as_exhausted():
    assert not track(Pose(0.0, 0.0, 0.0), 2.0, straight(length=60.0), CONFIG).path_exhausted
