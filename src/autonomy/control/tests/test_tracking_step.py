"""The per-step gating, checked without a ROS environment.

These are the decisions the node delegates: whether the frame agrees, whether
the pose is fresh enough to act on, whether the orientation is usable, and what
the tracker then makes of it. Keeping them here rather than in the node is what
lets them run anywhere.
"""

import math

import pytest

from control.algorithms.pure_pursuit import TrackerConfig
from control.algorithms.route import parse_route
from control.algorithms.tracking_step import StepInput, decide

CONFIG = TrackerConfig(
    wheelbase_m=1.0,
    max_steer_rad=math.radians(30.0),
    lookahead_min_m=1.0,
    lookahead_max_m=6.0,
    lookahead_gain_s=0.5,
    max_cross_track_m=3.0,
)
ROUTE = parse_route(["x_m,y_m,speed_mps"] + [f"{i * 0.5},0.0,5.0" for i in range(60)])
FRAME = "map"
TIMEOUT_S = 0.2
NOW_S = 1000.0


def step(x=0.0, y=0.0, yaw=0.0, speed=5.0, frame=FRAME, age_s=0.0, quat=None):
    qx, qy, qz, qw = quat or (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))
    return StepInput(frame_id=frame, stamp_s=NOW_S - age_s, x_m=x, y_m=y,
                     qx=qx, qy=qy, qz=qz, qw=qw, vx_mps=speed, vy_mps=0.0)


def run(**kwargs):
    return decide(step(**kwargs), NOW_S, ROUTE, CONFIG, FRAME, TIMEOUT_S)


# ----------------------------------------------------------------- frames

def test_a_matching_frame_is_tracked():
    decision = run()
    assert decision.tracking and decision.result is not None


def test_a_different_frame_is_refused_by_name():
    decision = run(frame="odom")
    assert not decision.tracking
    assert "odom" in decision.reason and FRAME in decision.reason


def test_a_refusal_carries_no_result():
    """Callers must branch on `tracking`; there is no partial answer to read."""
    assert run(frame="odom").result is None


# ----------------------------------------------------------------- freshness

def test_a_pose_inside_the_budget_is_tracked():
    assert run(age_s=TIMEOUT_S / 2.0).tracking


def test_a_stale_pose_is_refused_with_both_numbers():
    decision = run(age_s=0.5)
    assert not decision.tracking
    assert "0.500" in decision.reason and "0.200" in decision.reason


def test_a_pose_from_the_future_is_refused_rather_than_treated_as_fresh():
    """Two clocks, usually one node left on wall time while the rest follow /clock."""
    decision = run(age_s=-0.5)
    assert not decision.tracking
    assert "use_sim_time" in decision.reason


def test_a_small_clock_disagreement_is_tolerated():
    """Within the budget either way; refusing on a millisecond would be useless."""
    assert run(age_s=-TIMEOUT_S / 2.0).tracking


def test_a_non_finite_timestamp_is_refused():
    bad = StepInput(frame_id=FRAME, stamp_s=float("nan"), x_m=0.0, y_m=0.0,
                    qx=0.0, qy=0.0, qz=0.0, qw=1.0, vx_mps=1.0, vy_mps=0.0)
    assert not decide(bad, NOW_S, ROUTE, CONFIG, FRAME, TIMEOUT_S).tracking


# ----------------------------------------------------------------- orientation

def test_an_unusable_orientation_is_refused_rather_than_assumed():
    decision = run(quat=(0.0, 0.0, 0.0, 0.0))
    assert not decision.tracking and "orientation" in decision.reason


def test_heading_reaches_the_tracker():
    """Facing away from the route is refused; facing along it is not."""
    assert run(x=2.0, yaw=0.0).tracking
    assert not run(x=2.0, yaw=math.pi).tracking


# ----------------------------------------------------------------- velocity

def test_speed_sizes_the_lookahead():
    slow = run(speed=0.0).result
    fast = run(speed=4.0).result
    assert fast.lookahead_m > slow.lookahead_m


def test_a_sign_flipped_velocity_estimate_does_not_refuse_the_step():
    """Magnitude, not the forward component; the tracker rejects negative speed."""
    reversed_estimate = StepInput(frame_id=FRAME, stamp_s=NOW_S, x_m=2.0, y_m=0.0,
                                  qx=0.0, qy=0.0, qz=0.0, qw=1.0,
                                  vx_mps=-3.0, vy_mps=0.0)
    assert decide(reversed_estimate, NOW_S, ROUTE, CONFIG, FRAME, TIMEOUT_S).tracking


def test_sideslip_contributes_to_the_lookahead_speed():
    sideways = StepInput(frame_id=FRAME, stamp_s=NOW_S, x_m=0.0, y_m=0.0,
                         qx=0.0, qy=0.0, qz=0.0, qw=1.0, vx_mps=3.0, vy_mps=4.0)
    decision = decide(sideways, NOW_S, ROUTE, CONFIG, FRAME, TIMEOUT_S)
    # hypot(3, 4) = 5, so lookahead = 1.0 + 0.5 * 5.0 = 3.5 m
    assert decision.result.lookahead_m == pytest.approx(3.5)


def test_a_non_finite_velocity_is_refused():
    bad = StepInput(frame_id=FRAME, stamp_s=NOW_S, x_m=0.0, y_m=0.0,
                    qx=0.0, qy=0.0, qz=0.0, qw=1.0, vx_mps=float("inf"), vy_mps=0.0)
    assert not decide(bad, NOW_S, ROUTE, CONFIG, FRAME, TIMEOUT_S).tracking


# ----------------------------------------------------------------- the tracker

def test_a_tracker_refusal_is_passed_through_verbatim():
    decision = run(x=2.0, y=10.0)
    assert not decision.tracking and "cross-track" in decision.reason


def test_a_tracked_step_reports_the_tracker_result():
    decision = run(x=2.0, y=-0.5)
    assert decision.tracking
    assert decision.reason == "tracking"
    assert decision.result.steering_rad > 0.0, "offset to the right should steer left"
