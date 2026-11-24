"""One control step, from unpacked estimator values to a tracking decision.

This holds the checks that decide whether tracking is justified at all: frame
agreement, pose freshness, and a usable orientation. They live here rather than
in the node so they are testable without a ROS environment, which is the same
rule the rest of `algorithms/` follows. The node above is left with message
unpacking and publication, and no judgement of its own.

A refusal is a result, not an error. The caller is told what was refused and
why, and decides what the vehicle does about it; stopping is a policy question
that `docs/ros-structure.md` leaves to the manager and the MCU.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

from control.algorithms.geometry import yaw_from_quaternion
from control.algorithms.pure_pursuit import (PathSample, Pose, TrackerConfig, TrackerResult,
                                             track)


@dataclass(frozen=True)
class StepInput:
    """Estimator values for one step, already out of their message types.

    Position and orientation are expressed in `frame_id`. Velocity is expressed
    in the estimator's child frame, which for `nav_msgs/Odometry` is the body
    frame; only its magnitude is used, so the distinction does not affect the
    result here.
    """

    frame_id: str
    stamp_s: float
    x_m: float
    y_m: float
    qx: float
    qy: float
    qz: float
    qw: float
    vx_mps: float
    vy_mps: float


@dataclass(frozen=True)
class StepDecision:
    """Whether this step tracked, and what came of it.

    `result` is present only when `tracking` is True. `reason` carries the
    refusal when it is False, phrased for a log line.
    """

    tracking: bool
    reason: str
    result: TrackerResult | None = None


def _refuse(reason: str) -> StepDecision:
    return StepDecision(tracking=False, reason=reason)


def decide(step: StepInput, now_s: float, route: list[PathSample],
           config: TrackerConfig, route_frame: str,
           odometry_timeout_s: float) -> StepDecision:
    """Gate the estimator values, then run one tracking step if they hold up."""
    if step.frame_id != route_frame:
        # Tracking a route expressed somewhere else is not a degraded result, it
        # is a different trajectory. Refuse rather than drive it.
        return _refuse(
            f"odometry frame {step.frame_id!r} does not match route frame {route_frame!r}")

    age_s = now_s - step.stamp_s
    if not math.isfinite(age_s):
        return _refuse("odometry timestamp is not finite")
    if age_s > odometry_timeout_s:
        return _refuse(
            f"odometry is {age_s:.3f} s old, over the {odometry_timeout_s:.3f} s budget")
    if age_s < -odometry_timeout_s:
        # A pose from the future usually means two clocks, often one node left on
        # wall time while the rest followed /clock. Freshness cannot be judged.
        return _refuse(
            f"odometry is {-age_s:.3f} s in the future; check use_sim_time across the graph")

    try:
        yaw_rad = yaw_from_quaternion(step.qx, step.qy, step.qz, step.qw)
    except ValueError as error:
        return _refuse(f"odometry orientation unusable: {error}")

    # Magnitude rather than the forward component, so a sideslipping or
    # sign-flipped estimate cannot produce the negative speed the tracker
    # refuses. It only sizes the lookahead and is not fed back into any loop.
    speed_mps = math.hypot(step.vx_mps, step.vy_mps)
    if not math.isfinite(speed_mps):
        return _refuse("odometry velocity is not finite")

    result = track(Pose(step.x_m, step.y_m, yaw_rad), speed_mps, route, config)
    if not result.valid:
        return _refuse(result.reason)
    return StepDecision(tracking=True, reason="tracking", result=result)
