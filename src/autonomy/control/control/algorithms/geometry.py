"""Orientation helpers shared by the control algorithms.

ROS-free, like everything else under `algorithms/`: these take and return plain
floats, so they are testable without a ROS environment. Unpacking a message into
these arguments is the node layer's job.

Conventions follow REP-103. Body frame is X forward, Y left, Z up, and yaw is
measured counter-clockwise from the +X axis of the reference frame.
"""

from __future__ import annotations

import math


def yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    """Yaw in radians from a quaternion, in the range (-pi, pi].

    Only the yaw component is recovered, which is the only one a planar tracker
    can act on. Whenever the kart is not level this is therefore an
    approximation rather than the full orientation, and the error grows with
    roll and pitch; on a flat track it is negligible.

    Raises ValueError on a zero-length or non-finite quaternion rather than
    returning a plausible angle, because a silently wrong heading is the kind of
    fault that only shows up as the vehicle driving into a wall.
    """
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if not math.isfinite(norm) or norm < 1e-12:
        raise ValueError(f"quaternion is not usable: ({x!r}, {y!r}, {z!r}, {w!r})")
    x, y, z, w = x / norm, y / norm, z / norm, w / norm
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
