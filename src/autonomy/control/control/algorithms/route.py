"""Read a pre-calculated route from disk into tracker samples.

WHY A FILE AND NOT A TOPIC
--------------------------
`docs/ros-structure.md` specifies `planning/trajectory` as
`interfaces/msg/Trajectory`, and that message does not exist yet; its reference
frame, timing and expiration fields are listed there as contract details still
to be settled. Publishing some other type on that topic in the meantime would
contradict the documented contract and would have to be undone later. Reading
the route from a file keeps the controller runnable now, matches the "explicit
test source" the same document anticipates for planning, and leaves the topic
free for the real message.

FORMAT
------
A header line, then one sample per line::

    x_m,y_m,speed_mps
    0.0,0.0,5.0
    1.0,0.0,5.0

Coordinates are in whichever frame the caller names; nothing here can verify
that and nothing here tries. Speeds are the planner's intent in metres per
second, advisory in the same sense as `PathSample.speed_mps`: the tracker passes
them through and never treats them as achieved.

The route is read as an open polyline. Repeating the first point at the end is
how a caller expresses a closed lap; this module will not close one on its
caller's behalf, because inventing a final segment would invent a path the
planner never declared.
"""

from __future__ import annotations

import math
from pathlib import Path

from control.algorithms.pure_pursuit import PathSample

EXPECTED_HEADER = ("x_m", "y_m", "speed_mps")


class RouteError(ValueError):
    """A route file that cannot be read as a route.

    Separate from ValueError so a node can distinguish bad input data from a
    programming error and report the file path rather than a stack trace.
    """


def parse_route(lines: list[str], source: str = "<route>") -> list[PathSample]:
    """Parse route lines into samples, rejecting anything ambiguous.

    `source` names the origin in error messages and is cosmetic.
    """
    rows = [line.strip() for line in lines]
    rows = [line for line in rows if line and not line.startswith("#")]
    if not rows:
        raise RouteError(f"{source}: no data")

    header = tuple(field.strip().lower() for field in rows[0].split(","))
    if header != EXPECTED_HEADER:
        raise RouteError(
            f"{source}: expected header {','.join(EXPECTED_HEADER)}, got {rows[0]!r}")

    samples: list[PathSample] = []
    for number, row in enumerate(rows[1:], start=2):
        fields = [field.strip() for field in row.split(",")]
        if len(fields) != 3:
            raise RouteError(
                f"{source}: line {number} has {len(fields)} fields, expected 3: {row!r}")
        try:
            x, y, speed = (float(field) for field in fields)
        except ValueError as error:
            raise RouteError(f"{source}: line {number} is not numeric: {row!r}") from error
        if not all(math.isfinite(value) for value in (x, y, speed)):
            raise RouteError(f"{source}: line {number} is not finite: {row!r}")
        if speed < 0.0:
            # Reverse along a route is a contract nobody has chosen, and the
            # tracker refuses negative references anyway. Fail here, where the
            # line number is still known, rather than at the first control step.
            raise RouteError(
                f"{source}: line {number} has negative speed {speed}; reverse is unspecified")
        samples.append(PathSample(x_m=x, y_m=y, speed_mps=speed))

    if len(samples) < 2:
        raise RouteError(f"{source}: a route needs at least two samples, got {len(samples)}")
    return samples


def load_route_csv(path: str | Path) -> list[PathSample]:
    """Read and parse a route file.

    Raises RouteError for a missing or unreadable file as well as for malformed
    contents, so a caller has one exception type to handle.
    """
    route_path = Path(path)
    try:
        text = route_path.read_text(encoding="utf-8")
    except OSError as error:
        raise RouteError(f"cannot read route {route_path}: {error}") from error
    return parse_route(text.splitlines(), source=str(route_path))


def route_length_m(samples: list[PathSample]) -> float:
    """Total polyline length, for logging what was loaded."""
    return sum(math.hypot(b.x_m - a.x_m, b.y_m - a.y_m)
               for a, b in zip(samples, samples[1:]))
