# control

Consume estimated motion and a trajectory; publish requests to the vehicle manager. Keep algorithms under algorithms/ free of ROS callbacks, MuJoCo, SDKs, and the MCU wire protocol. No speed loop or actuator command semantics have been chosen.

## Layout

`algorithms/` holds everything that decides anything, and imports no ROS:

- `pure_pursuit.py` is geometric path tracking: a pose and an ordered path in, a
  road-wheel steering angle and a speed reference out, both in SI units. It
  holds no state.
- `route.py` reads a pre-calculated route from a CSV of `x_m,y_m,speed_mps`.
  A file rather than a topic, because `planning/trajectory` is specified as
  `interfaces/msg/Trajectory` and that message does not exist yet; publishing
  something else on that topic would contradict the documented contract.
- `geometry.py` recovers yaw from a quaternion.
- `tracking_step.py` is one control step: frame agreement, pose freshness and a
  usable orientation, then the tracker.

`pure_pursuit_node.py` is the ROS adapter. It unpacks `nav_msgs/Odometry`,
calls `tracking_step.decide`, and publishes the lookahead point it selected on
`control/debug/lookahead` as a `nav_msgs/Path`. It makes no decisions of its
own, which is why the tests run without a ROS environment.

## What it does not do

It does not publish `control/request`. `docs/ros-structure.md` lists
`interfaces/msg/ActuatorCommand` as proposed rather than defined and records
that its final fields depend on whether the real boundary is current, torque,
throttle/brake or speed; the same question is open item 7 in
`docs/decisions.md`. A steering angle and a speed reference are true under every
answer, so the node computes them, reports them and stops there. When the
boundary is chosen, what remains is a publisher and a unit conversion.

There is no speed loop, and `control/debug/lookahead` is observability rather
than a command.

## Parameters

Every parameter is required and none has a default, because the kart is
unmeasured and a default would be a guess wearing the clothes of a configured
value. The node refuses to start instead. The worksheet is
`src/bringup/autonomy_bringup/config/pure_pursuit.example.yaml`; geometry comes
from the same measurement pass as
`src/common/description/config/geometry.example.yaml`.

The route and the incoming odometry must share a frame. Nothing here transforms
between frames, so a mismatch is refused rather than tracked.

## Tests

    PYTHONPATH=src/autonomy/control python -m pytest src/autonomy/control/tests

They need no ROS environment and no built workspace. CI does not run them yet;
`.github/workflows/scaffold.yml` runs only the structural checks.
