"""MuJoCo bridge with simulation-only efforts and a steady stepping timer."""
import math
import time

import rclpy
from rcl_interfaces.msg import ParameterDescriptor, SetParametersResult
from rclpy.clock import Clock, ClockType
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data,
)
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock as ClockMessage
from sensor_msgs.msg import Imu
from std_srvs.srv import SetBool, Trigger
from std_msgs.msg import String
from interfaces.msg import SimActuatorCommand

from sim.runtime import Simulation
from sim_ros.messages import clock_message, imu_message, truth_message
from sim_ros.command_gate import CommandGate


class SimBridge(Node):
    def __init__(self, **kwargs):
        super().__init__("sim_bridge", **kwargs)
        self.viewer = None
        try:
            self._configure()
        except Exception:
            self.destroy_node()
            raise

    def _parameter(self, name, default):
        return self.declare_parameter(
            name, default, ParameterDescriptor(read_only=True)).value

    def _configure(self):
        if not self.get_parameter("use_sim_time").value:
            raise ValueError("sim_bridge requires use_sim_time=true")
        self.add_on_set_parameters_callback(self._guard_clock)
        model = self._parameter("model", "")
        body = self._parameter("body", "base_link")
        accel = self._parameter("accelerometer", "imu_accel")
        gyro = self._parameter("gyro", "imu_gyro")
        self.world_frame = self._parameter("world_frame", "sim_world")
        self.body_frame = self._parameter("body_frame", "base_link")
        self.imu_frame = self._parameter("imu_frame", "imu_link")
        for frame in (self.world_frame, self.body_frame, self.imu_frame):
            if not frame or frame.startswith("/") or any(c.isspace() for c in frame):
                raise ValueError("frame IDs must be nonempty, relative, and contain no whitespace")
        if len({self.world_frame, self.body_frame, self.imu_frame}) != 3:
            raise ValueError("world, body, and IMU frame IDs must be distinct")
        accel_std = self._parameter("accel_noise_std", float("nan"))
        gyro_std = self._parameter("gyro_noise_std", float("nan"))
        seed = self._parameter("seed", 0)
        timestep = self._parameter("timestep", 0.0)
        factor = self._parameter("real_time_factor", 1.0)
        if not math.isfinite(factor) or factor <= 0:
            raise ValueError("real_time_factor must be finite and positive")
        self.paused = self._parameter("start_paused", False)
        rendering = self._parameter("rendering", False)
        command_timeout = self._parameter("command_timeout_s", float("nan"))
        wall_timeout = self._parameter("wall_timeout_s", float("nan"))
        self.sim = Simulation(model, body=body, accelerometer=accel, gyro=gyro,
                              accel_noise_std=accel_std, gyro_noise_std=gyro_std,
                              seed=seed, timestep=None if timestep == 0.0 else timestep)
        self.commands = CommandGate(self.sim, command_timeout, wall_timeout)
        period = self.sim.timestep / factor
        if not math.isfinite(period) or period < 1e-9:
            raise ValueError("requested timer period must be at least one nanosecond")
        clock_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                               durability=DurabilityPolicy.VOLATILE)
        self.clock_pub = self.create_publisher(ClockMessage, "/clock", clock_qos)
        self.imu_pub = self.create_publisher(Imu, "sensors/imu/data", qos_profile_sensor_data)
        self.truth_pub = self.create_publisher(
            Odometry, "sim/ground_truth/odometry", qos_profile_sensor_data)
        self.create_subscription(SimActuatorCommand, "sim/actuator_command",
                                 self._command, clock_qos)
        session_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                                 durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.session_pub = self.create_publisher(String, "sim/session", session_qos)
        self.create_service(SetBool, "sim/pause", self._pause)
        self.create_service(Trigger, "sim/reset", self._reset)
        # A ROS-time timer would deadlock waiting for this node's own /clock.
        self.steady_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self.timer = self.create_timer(period, self._tick, clock=self.steady_clock)
        self._restart_metrics()
        self._last_rejection_wall = -math.inf
        if rendering:
            self.viewer = self.sim.open_viewer()
        self._publish()
        self._publish_session()
        self.get_logger().info(
            "MuJoCo bridge: clock, IMU, evaluation truth and simulation-only joint efforts. "
            "No physical commands, GNSS, cameras, TF or estimated odometry.")
        self.get_logger().info(f"Simulation motors (index, min, max, unit): {self.sim.actuators}")
        if model == "demo":
            self.get_logger().warning("Using synthetic falling-box fixture; this is not a kart model.")
        if accel_std == 0 or gyro_std == 0:
            self.get_logger().warning("Zero noise explicitly selected; IMU channels may be ideal.")

    @staticmethod
    def _guard_clock(parameters):
        for parameter in parameters:
            if parameter.name == "use_sim_time" and parameter.value is not True:
                return SetParametersResult(successful=False, reason="simulation time is required")
        return SetParametersResult(successful=True)

    def _restart_metrics(self):
        self._wall_start = time.monotonic()
        self._sim_start = self.sim.snapshot.time_s

    def _publish(self):
        sample = self.sim.snapshot
        self.clock_pub.publish(clock_message(sample))
        self.imu_pub.publish(imu_message(sample, self.imu_frame,
                                         self.sim.accel_noise_std, self.sim.gyro_noise_std))
        self.truth_pub.publish(truth_message(sample, self.world_frame, self.body_frame))

    def _tick(self):
        if self.viewer is not None:
            if not self.viewer.is_running():
                self.context.try_shutdown()
                return
            self.viewer.sync()
        if self.paused:
            self.clock_pub.publish(clock_message(self.sim.snapshot))
            return
        self.commands.expire()
        self.sim.step()
        self._publish()
        elapsed = time.monotonic() - self._wall_start
        if elapsed >= 5.0:
            factor = (self.sim.snapshot.time_s - self._sim_start) / elapsed
            self.get_logger().info(f"Measured real-time factor: {factor:.3f}")
            self._restart_metrics()

    def _pause(self, request, response):
        if request.data and not self.paused:
            self.commands.invalidate()
            self._publish_session()
        self.paused = request.data
        self._restart_metrics()
        response.success = True
        response.message = "paused" if self.paused else "running"
        return response

    def _reset(self, request, response):
        if not self.paused:
            response.success = False
            response.message = "Pause first; stop/reset downstream consumers before resetting simulation time."
            return response
        self.sim.reset()
        self.commands.invalidate()
        self._publish_session()
        self._restart_metrics()
        self._publish()
        response.success = True
        response.message = "Reset to t=0; remains paused. Downstream state is not reset by this service."
        return response

    def _publish_session(self):
        self.session_pub.publish(String(data=self.commands.session))

    def _command(self, message):
        if not (0 <= message.stamp.nanosec < 1_000_000_000
                and 0 <= message.valid_for.nanosec < 1_000_000_000):
            reason = "non-normalized ROS timestamp/duration"
        else:
            reason = self.commands.accept(
                session=message.session_id, sequence=message.sequence,
                stamp_ns=message.stamp.sec * 1_000_000_000 + message.stamp.nanosec,
                validity_ns=message.valid_for.sec * 1_000_000_000 + message.valid_for.nanosec,
                names=message.actuator_names, efforts=message.efforts, paused=self.paused)
        now = time.monotonic()
        if reason is not None and now - self._last_rejection_wall >= 1.0:
            self.get_logger().warning(f"Rejected simulation command: {reason}")
            self._last_rejection_wall = now

    def destroy_node(self):
        if self.viewer is not None:
            self.viewer.close()
            self.viewer = None
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    result = 0
    try:
        node = SimBridge()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except Exception as error:
        rclpy.logging.get_logger("sim_bridge").fatal(str(error))
        result = 1
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return result


if __name__ == "__main__":
    raise SystemExit(main())
