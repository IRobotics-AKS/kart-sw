"""ROS node around the geometric tracker in `algorithms/pure_pursuit.py`.

Subscribes to estimated motion, runs one tracking step per control period
against a pre-calculated route, and publishes the lookahead point it selected.

This file holds message unpacking, parameters and publication, and no judgement
of its own. Whether a step is worth tracking, and what the tracker then makes of
it, is decided in `algorithms/tracking_step.py`, which runs without ROS and
carries the tests.

WHAT THIS DELIBERATELY DOES NOT DO
----------------------------------
It does not publish `control/request`. `docs/ros-structure.md` lists
`interfaces/msg/ActuatorCommand` as proposed rather than defined, and records
that its final fields depend on whether the real boundary is current, torque,
throttle/brake or speed; `docs/decisions.md` carries the same question as open
item 7. `AGENTS.md` forbids implementing ambiguous command modes without an
explicit contract. A road-wheel steering angle and a speed reference are true
under every answer to that question, so this node computes them, reports them
and stops. Once the boundary is chosen, what remains is a publisher and a unit
conversion rather than a rewrite.

For the same reason there is no speed loop, and no actuator model beyond the
steering limit the caller supplies.

IT GUESSES NOTHING
------------------
Every parameter below is required and has no default. `AGENTS.md` forbids
inventing actuator limits, and `src/bringup/autonomy_bringup/config/README.md`
forbids substituting guessed timeouts; `docs/ros-structure.md` adds that rates,
queue depths and deadline budgets follow from measurement rather than from
hardware-independent constants. The kart is unmeasured at the time of writing,
so the node refuses to start rather than running on plausible numbers. The
worksheet is `config/pure_pursuit.example.yaml` in `autonomy_bringup`.

FRAMES
------
The route and the incoming pose must share a frame. Nothing here transforms
between frames, so the node compares the odometry frame against the configured
`route_frame` and refuses a mismatch. Pose is read from `pose.pose`, which
`nav_msgs/Odometry` expresses in `header.frame_id`; velocity is read from
`twist.twist`, which the same message expresses in `child_frame_id`.
"""

from __future__ import annotations

import math

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time

from control.algorithms.pure_pursuit import TrackerConfig
from control.algorithms.route import RouteError, load_route_csv, route_length_m
from control.algorithms.tracking_step import StepInput, decide

#: Doubles the caller must supply. Absent any measurement of the kart, a default
#: here would be a guess wearing the clothes of a configured value.
REQUIRED_DOUBLES = (
    "wheelbase_m",
    "max_steer_rad",
    "lookahead_min_m",
    "lookahead_max_m",
    "lookahead_gain_s",
    "max_cross_track_m",
    "control_rate_hz",
    "odometry_timeout_s",
)

ODOMETRY_TOPIC = "localization/odometry"
LOOKAHEAD_TOPIC = "control/debug/lookahead"

_UNSET = float("nan")
_LOG_PERIOD_S = 2.0


class ParameterError(RuntimeError):
    """A parameter set the node will not start on."""


class PurePursuitNode(Node):
    """Tracks a pre-calculated route and reports the result.

    Construction raises rather than degrading. A controller that comes up
    half-configured is worse than one that does not come up, because the first
    failure then happens with the kart moving.
    """

    def __init__(self, **kwargs) -> None:
        super().__init__("pure_pursuit", **kwargs)

        self.declare_parameter("route_csv", "")
        self.declare_parameter("route_frame", "")
        self.declare_parameter("max_lateral_accel_mps2", 0.0)
        for name in REQUIRED_DOUBLES:
            self.declare_parameter(name, _UNSET)

        values = self._required_doubles()
        self._route_frame = self.get_parameter("route_frame").value
        if not self._route_frame:
            raise ParameterError(
                "route_frame is required; it names the frame the route coordinates "
                "and the incoming odometry must share")

        self._route = self._load_route()
        self._config = self._build_config(values)
        self._timeout_s = values["odometry_timeout_s"]

        # Reliable, volatile and shallow, as docs/ros-structure.md sets out for an
        # initial configuration. Volatile matters here: a pose retained from before
        # a reset must never be tracked as though it were current.
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self._odometry: Odometry | None = None
        self.create_subscription(Odometry, ODOMETRY_TOPIC, self._on_odometry, qos)
        self._lookahead_pub = self.create_publisher(Path, LOOKAHEAD_TOPIC, qos)
        self.create_timer(1.0 / values["control_rate_hz"], self.on_control_step)

        self.get_logger().info(
            f"tracking {len(self._route)} samples spanning "
            f"{route_length_m(self._route):.1f} m in frame {self._route_frame!r} at "
            f"{values['control_rate_hz']:.1f} Hz; publishing no actuator command "
            "(see docs/decisions.md open question 7)")

    # ------------------------------------------------------------------ setup

    def _required_doubles(self) -> dict[str, float]:
        values = {name: float(self.get_parameter(name).value) for name in REQUIRED_DOUBLES}
        missing = sorted(name for name, value in values.items() if math.isnan(value))
        if missing:
            raise ParameterError(
                "these parameters have no default and must be supplied: "
                + ", ".join(missing)
                + ". Vehicle geometry is unmeasured; see "
                "src/common/description/config/geometry.example.yaml")
        for name in ("control_rate_hz", "odometry_timeout_s"):
            if values[name] <= 0.0:
                raise ParameterError(f"{name} must be positive, got {values[name]}")
        return values

    def _load_route(self):
        route_csv = self.get_parameter("route_csv").value
        if not route_csv:
            raise ParameterError("route_csv is required and names the pre-calculated route")
        try:
            return load_route_csv(route_csv)
        except RouteError as error:
            raise ParameterError(str(error)) from error

    def _build_config(self, values: dict[str, float]) -> TrackerConfig:
        lateral = self.get_parameter("max_lateral_accel_mps2").value
        try:
            return TrackerConfig(
                wheelbase_m=values["wheelbase_m"],
                max_steer_rad=values["max_steer_rad"],
                lookahead_min_m=values["lookahead_min_m"],
                lookahead_max_m=values["lookahead_max_m"],
                lookahead_gain_s=values["lookahead_gain_s"],
                max_cross_track_m=values["max_cross_track_m"],
                # The tracker reads None as "no limit". A non-positive parameter is
                # how a caller says that through ROS, which has no null double.
                max_lateral_accel_mps2=lateral if lateral > 0.0 else None,
            )
        except ValueError as error:
            raise ParameterError(str(error)) from error

    # --------------------------------------------------------------- runtime

    def _on_odometry(self, message: Odometry) -> None:
        self._odometry = message

    def on_control_step(self) -> None:
        """One control period. Public so a test can drive it without a timer."""
        message = self._odometry
        if message is None:
            self.get_logger().warning(
                "no odometry received yet", throttle_duration_sec=_LOG_PERIOD_S)
            return

        now_s = self.get_clock().now().nanoseconds / 1e9
        decision = decide(
            self._step_input(message), now_s, self._route, self._config,
            self._route_frame, self._timeout_s)

        if not decision.tracking:
            self.get_logger().warning(
                f"not tracking: {decision.reason}", throttle_duration_sec=_LOG_PERIOD_S)
            self._publish_lookahead(None)
            return

        result = decision.result
        if result.steering_saturated:
            self.get_logger().warning(
                "steering request is at the configured limit",
                throttle_duration_sec=_LOG_PERIOD_S)
        if result.path_exhausted:
            self.get_logger().warning(
                "tracking the final route sample; the route is open and is not wrapped",
                throttle_duration_sec=_LOG_PERIOD_S)

        self._publish_lookahead(result.target_index)
        self.get_logger().debug(
            f"steering {result.steering_rad:+.4f} rad, speed reference "
            f"{result.speed_mps:.2f} m/s, lookahead {result.lookahead_m:.2f} m, "
            f"cross-track {result.cross_track_error_m:+.3f} m")

    @staticmethod
    def _step_input(message: Odometry) -> StepInput:
        position = message.pose.pose.position
        orientation = message.pose.pose.orientation
        linear = message.twist.twist.linear
        return StepInput(
            frame_id=message.header.frame_id,
            stamp_s=Time.from_msg(message.header.stamp).nanoseconds / 1e9,
            x_m=position.x,
            y_m=position.y,
            qx=orientation.x,
            qy=orientation.y,
            qz=orientation.z,
            qw=orientation.w,
            vx_mps=linear.x,
            vy_mps=linear.y,
        )

    def _publish_lookahead(self, target_index: int | None) -> None:
        """Publish the selected lookahead point, or an empty path when not tracking.

        This is observability, not a command. An empty path is how a viewer tells
        that the controller stopped tracking rather than froze on a stale target.
        """
        path = Path()
        path.header.frame_id = self._route_frame
        path.header.stamp = self.get_clock().now().to_msg()
        if target_index is not None:
            sample = self._route[target_index]
            pose = PoseStamped()
            pose.header = path.header
            pose.pose.position.x = sample.x_m
            pose.pose.position.y = sample.y_m
            pose.pose.orientation.w = 1.0
            path.poses.append(pose)
        self._lookahead_pub.publish(path)


def main(args=None) -> int:
    rclpy.init(args=args)
    try:
        node = PurePursuitNode()
    except ParameterError as error:
        rclpy.logging.get_logger("pure_pursuit").fatal(str(error))
        rclpy.shutdown()
        return 1
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
