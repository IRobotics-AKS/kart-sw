"""Geometric path tracking: pose and path in, steering angle and speed reference out.

No ROS, simulation, or driver imports, per `control/README.md` and enforced by
`tools/check_scaffold.py`. Nothing here knows what a topic is.

WHAT THIS DELIBERATELY DOES NOT DO
----------------------------------
It does not emit actuator commands. `docs/ros-structure.md` records that the
`ActuatorCommand` fields depend on whether the real boundary is current, torque,
throttle/brake or speed, and that question is open. This module therefore stops
at quantities that are true under every answer: a road-wheel steering angle in
radians and a speed reference in metres per second. Converting either into
actuator units belongs downstream, once the boundary is chosen.

For the same reason there is no speed loop. `speed_mps` is a reference drawn from
the path, not the output of a controller tracking measured speed. `AGENTS.md`
asks that an unimplemented speed loop never be added silently, so it is named
here instead.

Nor does it supply vehicle geometry or actuator limits. `TrackerConfig` has no
defaults for wheelbase or steering limit; the caller passes measured values.
Kart geometry is unmeasured at the time of writing and inventing a number here
would bury a guess somewhere nobody looks again.

CONVENTIONS
-----------
SI units throughout. Body frame is X forward, Y left, Z up, per REP-103. Yaw is
measured from the +X axis of the reference frame, counter-clockwise positive.
A positive steering angle turns left, and positive cross-track error means the
vehicle sits to the left of the path.

The caller is responsible for the frame agreeing between `pose` and `path`.
Nothing here can detect a frame mismatch; it will happily track a path expressed
in the wrong frame.
"""

from __future__ import annotations

import math
from dataclasses import dataclass


@dataclass(frozen=True)
class Pose:
    """Vehicle pose in the same frame as the path."""

    x_m: float
    y_m: float
    yaw_rad: float


@dataclass(frozen=True)
class PathSample:
    """One ordered sample of the reference path.

    `speed_mps` is the speed the planner intends at this point. It is advisory:
    this module passes it through, optionally reduced for curvature, and never
    treats it as achieved.
    """

    x_m: float
    y_m: float
    speed_mps: float


@dataclass(frozen=True)
class TrackerConfig:
    """Tuning and vehicle geometry. No defaults for anything vehicle-specific.

    lookahead_m = clamp(lookahead_min_m + lookahead_gain_s * speed,
                        lookahead_min_m, lookahead_max_m)

    The gain has units of seconds, so it reads as "look this far ahead in time".
    Short lookahead tracks tightly and oscillates; long lookahead is smooth and
    cuts corners. The right value is measured on the vehicle, not derived.
    """

    wheelbase_m: float
    max_steer_rad: float
    lookahead_min_m: float
    lookahead_max_m: float
    lookahead_gain_s: float
    max_cross_track_m: float
    max_lateral_accel_mps2: float | None = None

    def __post_init__(self) -> None:
        positive = {
            "wheelbase_m": self.wheelbase_m,
            "max_steer_rad": self.max_steer_rad,
            "lookahead_min_m": self.lookahead_min_m,
            "lookahead_max_m": self.lookahead_max_m,
            "max_cross_track_m": self.max_cross_track_m,
        }
        for name, value in positive.items():
            if not (value > 0.0 and math.isfinite(value)):
                raise ValueError(f"{name} must be finite and positive, got {value!r}")
        if self.lookahead_max_m < self.lookahead_min_m:
            raise ValueError("lookahead_max_m must be at least lookahead_min_m")
        if self.lookahead_gain_s < 0.0:
            raise ValueError("lookahead_gain_s must not be negative")
        if self.max_lateral_accel_mps2 is not None and self.max_lateral_accel_mps2 <= 0.0:
            raise ValueError("max_lateral_accel_mps2 must be positive when set")


@dataclass(frozen=True)
class TrackerResult:
    """What the tracker decided, and enough context to see why.

    When `valid` is False every numeric field is zero and `reason` says what went
    wrong. Callers must branch on `valid`; a zero steering angle is not a safe
    default, it is merely a defined one, and deciding what the vehicle does when
    tracking fails is the manager's job rather than this module's.
    """

    valid: bool
    reason: str
    steering_rad: float
    speed_mps: float
    lookahead_m: float
    target_index: int
    cross_track_error_m: float
    steering_saturated: bool
    path_exhausted: bool


_INVALID = {
    "steering_rad": 0.0,
    "speed_mps": 0.0,
    "lookahead_m": 0.0,
    "target_index": -1,
    "cross_track_error_m": 0.0,
    "steering_saturated": False,
    "path_exhausted": False,
}


def _invalid(reason: str) -> TrackerResult:
    return TrackerResult(valid=False, reason=reason, **_INVALID)


def nearest_index(pose: Pose, path: list[PathSample]) -> int:
    """Index of the path sample closest to the vehicle.

    Searches the whole path. A kart lap is short enough that this is cheap, and a
    windowed search around the previous index would carry state this module
    deliberately does not hold. Revisit if profiling says otherwise.
    """
    best, best_d2 = 0, math.inf
    for i, sample in enumerate(path):
        d2 = (sample.x_m - pose.x_m) ** 2 + (sample.y_m - pose.y_m) ** 2
        if d2 < best_d2:
            best, best_d2 = i, d2
    return best


def cross_track_error(pose: Pose, path: list[PathSample], index: int) -> float:
    """Signed lateral offset from the path, positive when the vehicle is left of it.

    Measured against the segment leaving `index`, or the one arriving at it for
    the final sample, so the sign stays meaningful at both ends.
    """
    if index + 1 < len(path):
        a, b = path[index], path[index + 1]
    else:
        a, b = path[index - 1], path[index]

    seg_x, seg_y = b.x_m - a.x_m, b.y_m - a.y_m
    length = math.hypot(seg_x, seg_y)
    if length < 1e-9:
        return math.hypot(pose.x_m - a.x_m, pose.y_m - a.y_m)

    # z component of (segment x vehicle offset), normalised: positive to the left
    to_vehicle_x, to_vehicle_y = pose.x_m - a.x_m, pose.y_m - a.y_m
    return (seg_x * to_vehicle_y - seg_y * to_vehicle_x) / length


def _lookahead_target(path: list[PathSample], start: int, lookahead_m: float) -> tuple[int, bool]:
    """Walk forward from `start` until `lookahead_m` of path has been covered.

    Returns the index and whether the path ran out first. The path is treated as
    open, not as a lap: closing it is the planner's decision to express, and
    wrapping here would invent one.
    """
    travelled = 0.0
    for i in range(start, len(path) - 1):
        travelled += math.hypot(path[i + 1].x_m - path[i].x_m,
                                path[i + 1].y_m - path[i].y_m)
        if travelled >= lookahead_m:
            return i + 1, False
    return len(path) - 1, True


def _curvature_speed_limit(curvature: float, max_lateral_accel_mps2: float | None) -> float:
    """Speed at which this curvature would reach the lateral acceleration limit."""
    if max_lateral_accel_mps2 is None or abs(curvature) < 1e-9:
        return math.inf
    return math.sqrt(max_lateral_accel_mps2 / abs(curvature))


def track(pose: Pose, speed_mps: float, path: list[PathSample],
          config: TrackerConfig) -> TrackerResult:
    """One tracking step.

    `speed_mps` is the vehicle's current measured speed, used only to size the
    lookahead distance. It is not fed back into a speed controller.
    """
    if len(path) < 2:
        return _invalid("path needs at least two samples")
    if not math.isfinite(speed_mps) or speed_mps < 0.0:
        return _invalid(f"measured speed must be finite and non-negative, got {speed_mps!r}")
    if not all(math.isfinite(v) for v in (pose.x_m, pose.y_m, pose.yaw_rad)):
        return _invalid("pose is not finite")

    near = nearest_index(pose, path)
    cte = cross_track_error(pose, path, near)
    if abs(cte) > config.max_cross_track_m:
        return _invalid(f"cross-track error {cte:.2f} m exceeds limit "
                        f"{config.max_cross_track_m:.2f} m")

    lookahead = min(max(config.lookahead_min_m + config.lookahead_gain_s * speed_mps,
                        config.lookahead_min_m), config.lookahead_max_m)
    target_index, exhausted = _lookahead_target(path, near, lookahead)
    target = path[target_index]

    # Target into the body frame: X forward, Y left.
    dx, dy = target.x_m - pose.x_m, target.y_m - pose.y_m
    cos_yaw, sin_yaw = math.cos(pose.yaw_rad), math.sin(pose.yaw_rad)
    forward = dx * cos_yaw + dy * sin_yaw
    left = -dx * sin_yaw + dy * cos_yaw

    distance = math.hypot(forward, left)
    if distance < 1e-6:
        return _invalid("lookahead target coincides with the vehicle")
    if forward <= 0.0:
        # Every remaining sample is behind the vehicle. Steering toward it would
        # command a turn the geometry cannot justify.
        return _invalid("lookahead target is behind the vehicle")

    curvature = 2.0 * left / (distance * distance)
    steering = math.atan(config.wheelbase_m * curvature)

    saturated = abs(steering) > config.max_steer_rad
    if saturated:
        steering = math.copysign(config.max_steer_rad, steering)

    speed_reference = max(target.speed_mps, 0.0)
    speed_reference = min(speed_reference,
                          _curvature_speed_limit(curvature, config.max_lateral_accel_mps2))

    return TrackerResult(
        valid=True,
        reason="tracking",
        steering_rad=steering,
        speed_mps=speed_reference,
        lookahead_m=lookahead,
        target_index=target_index,
        cross_track_error_m=cte,
        steering_saturated=saturated,
        path_exhausted=exhausted,
    )
