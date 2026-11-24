"""Translate core samples; never turn simulator truth into estimated state."""
import math

from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Imu


def stamp(seconds):
    if not math.isfinite(seconds) or seconds < 0:
        raise ValueError("simulation time must be finite and nonnegative")
    sec, nanosec = divmod(round(seconds * 1_000_000_000), 1_000_000_000)
    if sec >= 2**31:
        raise ValueError("simulation time exceeds the ROS timestamp range")
    result = Clock().clock
    result.sec, result.nanosec = sec, nanosec
    return result


def vector(target, values):
    target.x, target.y, target.z = values


def diagonal(variance):
    return [variance, 0.0, 0.0, 0.0, variance, 0.0, 0.0, 0.0, variance]


def clock_message(sample):
    message = Clock()
    message.clock = stamp(sample.time_s)
    return message


def imu_message(sample, frame, accel_std, gyro_std):
    message = Imu()
    message.header.stamp = stamp(sample.time_s)
    message.header.frame_id = frame
    # No fused orientation exists; do not substitute truth for a measurement.
    message.orientation.w = 1.0
    message.orientation_covariance[0] = -1.0
    vector(message.linear_acceleration, sample.acceleration_mps2)
    vector(message.angular_velocity, sample.gyro_rps)
    message.linear_acceleration_covariance = diagonal(accel_std**2)
    message.angular_velocity_covariance = diagonal(gyro_std**2)
    return message


def truth_message(sample, world_frame, body_frame):
    message = Odometry()
    message.header.stamp = stamp(sample.time_s)
    message.header.frame_id = world_frame
    message.child_frame_id = body_frame
    vector(message.pose.pose.position, sample.position_m)
    q = message.pose.pose.orientation
    q.x, q.y, q.z, q.w = sample.orientation_xyzw
    vector(message.twist.twist.linear, sample.linear_velocity_mps)
    vector(message.twist.twist.angular, sample.angular_velocity_rps)
    return message
