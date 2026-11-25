"""Send a finite burst to sim/actuator_command, using observed clock/session."""
import argparse
import math
import time

import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.utilities import remove_ros_args
from interfaces.msg import SimActuatorCommand
from rosgraph_msgs.msg import Clock as ClockMessage
from std_msgs.msg import String


def main(args=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--effort", action="append", required=True, metavar="MOTOR=VALUE")
    parser.add_argument("--duration", type=float, default=2.0, help="wall seconds")
    parser.add_argument("--valid-for", type=float, default=0.2, help="simulation seconds")
    parser.add_argument("--rate", type=float, default=20.0, help="wall Hz")
    options = parser.parse_args(remove_ros_args(args=args)[1:])
    for value in (options.duration, options.valid_for, options.rate):
        if not math.isfinite(value) or value <= 0:
            parser.error("duration, validity and rate must be finite and positive")
    validity_ns = round(options.valid_for * 1e9)
    if not 1 <= validity_ns < 2**31 * 1_000_000_000 or options.rate > 1e9:
        parser.error("validity or rate exceeds ROS timing range")
    efforts = {}
    try:
        for item in options.effort:
            name, value = item.split("=", 1)
            if not name or name in efforts or not math.isfinite(float(value)):
                raise ValueError()
            efforts[name] = float(value)
    except ValueError:
        parser.error("each effort must have a unique motor name and finite numeric value")

    rclpy.init(args=args)
    node = Node("send_sim_efforts", parameter_overrides=[Parameter("use_sim_time", value=True)])
    qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)
    session_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
    publisher = node.create_publisher(SimActuatorCommand, "sim/actuator_command", qos)
    latest = {"stamp": None, "session": None, "started": None, "sequence": time.monotonic_ns(),
              "done": False, "failed": False}
    created = time.monotonic()

    def on_clock(message):
        latest["stamp"] = message.clock

    def on_session(message):
        if latest["session"] is not None and latest["session"] != message.data:
            # Do not automatically resume a previous intent after reset/pause.
            latest["done"] = True
            node.get_logger().warning("Simulation session changed; stopped sending.")
        latest["session"] = message.data

    node.create_subscription(ClockMessage, "/clock", on_clock, qos)
    node.create_subscription(String, "sim/session", on_session, session_qos)

    def tick():
        now = time.monotonic()
        if latest["done"]:
            return
        if latest["stamp"] is None or latest["session"] is None:
            if now - created > 5.0:
                latest["done"] = latest["failed"] = True
                node.get_logger().error("No simulation clock/session received within 5 seconds.")
            return
        if latest["started"] is None:
            latest["started"] = now
        finished = now - latest["started"] >= options.duration
        message = SimActuatorCommand()
        message.stamp = latest["stamp"]
        message.session_id = latest["session"]
        message.sequence = latest["sequence"]
        message.valid_for.sec, message.valid_for.nanosec = divmod(validity_ns, 1_000_000_000)
        message.actuator_names = list(efforts)
        message.efforts = [0.0] * len(efforts) if finished else list(efforts.values())
        publisher.publish(message)
        latest["sequence"] += 1
        latest["done"] = finished

    steady = Clock(clock_type=ClockType.STEADY_TIME)
    node.create_timer(1.0 / options.rate, tick, clock=steady)
    try:
        while rclpy.ok() and not latest["done"]:
            rclpy.spin_once(node, timeout_sec=0.1)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        # Delivery of a final zero is not guaranteed; bridge expiry is authoritative.
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 1 if latest["failed"] else 0
