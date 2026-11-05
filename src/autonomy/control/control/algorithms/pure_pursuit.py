

from __future__ import annotations

import math
from dataclasses import dataclass


@dataclass(frozen=True)
class Pose:

    x_m: float
    y_m: float
    yaw_rad: float


@dataclass(frozen=True)
class PathSample:
    x_m: float
    y_m: float
    speed_mps: float


@dataclass(frozen=True)
class TrackerConfig:
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
    best, best_d2 = 0, math.inf
    for i, sample in enumerate(path):
        d2 = (sample.x_m - pose.x_m) ** 2 + (sample.y_m - pose.y_m) ** 2
        if d2 < best_d2:
            best, best_d2 = i, d2
    return best


def cross_track_error(pose: Pose, path: list[PathSample], index: int) -> float:
    if index + 1 < len(path):
        a, b = path[index], path[index + 1]
    else:
        a, b = path[index - 1], path[index]

    seg_x, seg_y = b.x_m - a.x_m, b.y_m - a.y_m
    length = math.hypot(seg_x, seg_y)
    if length < 1e-9:
        return math.hypot(pose.x_m - a.x_m, pose.y_m - a.y_m)

    to_vehicle_x, to_vehicle_y = pose.x_m - a.x_m, pose.y_m - a.y_m
    return (seg_x * to_vehicle_y - seg_y * to_vehicle_x) / length


def _lookahead_target(path: list[PathSample], start: int, lookahead_m: float) -> tuple[int, bool]:
    travelled = 0.0
    for i in range(start, len(path) - 1):
        travelled += math.hypot(path[i + 1].x_m - path[i].x_m,
                                path[i + 1].y_m - path[i].y_m)
        if travelled >= lookahead_m:
            return i + 1, False
    return len(path) - 1, True


def _curvature_speed_limit(curvature: float, max_lateral_accel_mps2: float | None) -> float:
    if max_lateral_accel_mps2 is None or abs(curvature) < 1e-9:
        return math.inf
    return math.sqrt(max_lateral_accel_mps2 / abs(curvature))


def track(pose: Pose, speed_mps: float, path: list[PathSample],
          config: TrackerConfig) -> TrackerResult:
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
