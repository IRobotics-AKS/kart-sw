"""Bounded ROS check inside the running workstation simulation container."""
import argparse
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from std_msgs.msg import String


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--health', action='store_true', help='clock heartbeat only; paused sim remains healthy')
    options = parser.parse_args()
    rclpy.init()
    node = Node('workstation_sim_check')
    clocks, imus, truth, sessions = [], [], [], []
    qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)
    node.create_subscription(Clock, '/clock', lambda msg: clocks.append(msg.clock.sec * 10**9 + msg.clock.nanosec), qos)
    if not options.health:
        node.create_subscription(Imu, '/kart/sensors/imu/data', imus.append, qos_profile_sensor_data)
        node.create_subscription(Odometry, '/kart/sim/ground_truth/odometry', truth.append, qos_profile_sensor_data)
        session_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                                durability=DurabilityPolicy.TRANSIENT_LOCAL)
        node.create_subscription(String, '/kart/sim/session', sessions.append, session_qos)
    deadline = time.monotonic() + (5.0 if options.health else 15.0)
    try:
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
            if options.health and clocks:
                return 0
            if len(clocks) >= 2 and clocks[-1] > clocks[0] and imus and truth and sessions:
                if imus[-1].orientation_covariance[0] != -1 or not sessions[-1].data:
                    raise RuntimeError('IMU validity or simulation session is incorrect')
                if truth[-1].header.frame_id != 'sim_world' or truth[-1].child_frame_id != 'base_link':
                    raise RuntimeError('Unexpected demo truth frame IDs')
                names = dict(node.get_topic_names_and_types())
                if '/kart/vehicle/command' in names or '/kart/localization/odometry' in names:
                    raise RuntimeError('Unexpected physical command or estimated-state publisher')
                print('PASS: advancing /clock, IMU, evaluation truth and simulation session received')
                return 0
        raise RuntimeError('Timed out waiting for simulation data; check logs and resume if paused')
    except RuntimeError as error:
        print(f'FAIL: {error}')
        return 1
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
