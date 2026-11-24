"""Real ROS integration checks; skipped explicitly when ROS is unavailable."""
import math
import time
import unittest


class RosBridgeTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        try:
            import rclpy
        except ImportError as error:
            raise unittest.SkipTest(f"ROS 2 unavailable: {error}")
        # Once ROS exists, missing generated interfaces are an actual failure.
        from sim_ros.bridge_node import SimBridge
        cls.rclpy = rclpy
        cls.Bridge = SimBridge

    def setUp(self):
        from rclpy.context import Context
        from rclpy.executors import SingleThreadedExecutor
        from rclpy.parameter import Parameter
        from rclpy.node import Node
        self.context = Context()
        self.rclpy.init(context=self.context)
        values = {"use_sim_time": True, "model": "demo", "accel_noise_std": 0.02,
                  "gyro_noise_std": 0.001, "command_timeout_s": 0.25, "wall_timeout_s": 0.5}
        self.bridge = self.Bridge(context=self.context, namespace="bridge_test",
                                 parameter_overrides=[Parameter(k, value=v) for k, v in values.items()])
        self.observer = Node("observer", context=self.context, namespace="bridge_test")
        self.executor = SingleThreadedExecutor(context=self.context)
        self.executor.add_node(self.bridge)
        self.executor.add_node(self.observer)

    def tearDown(self):
        self.executor.shutdown()
        self.bridge.destroy_node()
        self.observer.destroy_node()
        self.context.try_shutdown()

    def until(self, condition, timeout=5.0):
        deadline = time.monotonic() + timeout
        while not condition() and time.monotonic() < deadline:
            self.executor.spin_once(timeout_sec=0.02)
        self.assertTrue(condition())

    def test_clock_starts_and_sensor_truth_separation(self):
        from rosgraph_msgs.msg import Clock
        from sensor_msgs.msg import Imu
        from nav_msgs.msg import Odometry
        from rclpy.qos import qos_profile_sensor_data
        clocks, imus, truth = [], [], []
        self.observer.create_subscription(Clock, "/clock", clocks.append, 1)
        self.observer.create_subscription(Imu, "sensors/imu/data", imus.append, qos_profile_sensor_data)
        self.observer.create_subscription(Odometry, "sim/ground_truth/odometry", truth.append, qos_profile_sensor_data)
        self.until(lambda: len(clocks) > 3 and bool(imus) and bool(truth))
        self.assertGreater(clocks[-1].clock.sec * 1e9 + clocks[-1].clock.nanosec, 0)
        self.assertEqual(imus[-1].orientation_covariance[0], -1)
        self.assertEqual(imus[-1].header.frame_id, "imu_link")
        self.assertAlmostEqual(imus[-1].linear_acceleration_covariance[0], 0.02**2)
        self.assertEqual(truth[-1].header.frame_id, "sim_world")
        self.assertEqual(truth[-1].child_frame_id, "base_link")
        names = dict(self.bridge.get_topic_names_and_types())
        self.assertNotIn("/bridge_test/localization/odometry", names)
        self.assertNotIn("/bridge_test/vehicle/command", names)

    def test_pause_reset_and_resume_services(self):
        from std_srvs.srv import SetBool, Trigger
        pause = self.observer.create_client(SetBool, "sim/pause")
        reset = self.observer.create_client(Trigger, "sim/reset")
        self.until(lambda: pause.service_is_ready() and reset.service_is_ready())
        initial_session = self.bridge.commands.session
        future = reset.call_async(Trigger.Request())
        self.until(future.done)
        self.assertFalse(future.result().success)
        future = pause.call_async(SetBool.Request(data=True))
        self.until(future.done)
        self.assertTrue(future.result().success)
        self.assertNotEqual(initial_session, self.bridge.commands.session)
        frozen = self.bridge.sim.snapshot.time_s
        self.bridge._tick()
        self.assertEqual(self.bridge.sim.snapshot.time_s, frozen)
        future = reset.call_async(Trigger.Request())
        self.until(future.done)
        self.assertTrue(future.result().success)
        self.assertEqual(self.bridge.sim.snapshot.time_s, 0)
        self.assertTrue(self.bridge.paused)
        future = pause.call_async(SetBool.Request(data=False))
        self.until(future.done)
        self.until(lambda: self.bridge.sim.snapshot.time_s > 0)

    def test_real_command_subscription_and_parameter_guard(self):
        from interfaces.msg import SimActuatorCommand
        from rclpy.parameter import Parameter
        publisher = self.observer.create_publisher(SimActuatorCommand, "sim/actuator_command", 1)
        self.until(lambda: publisher.get_subscription_count() > 0)
        message = SimActuatorCommand()
        # Stop automatic stepping for a deterministic receipt/expiry assertion.
        self.bridge.timer.cancel()
        from sim_ros.messages import stamp
        message.stamp = stamp(self.bridge.sim.snapshot.time_s)
        message.session_id = self.bridge.commands.session
        message.sequence = 1
        message.valid_for.nanosec = 200_000_000
        message.actuator_names = ["x_force", "yaw_torque"]
        message.efforts = [2.0, 0.0]
        publisher.publish(message)
        self.until(lambda: self.bridge.commands.last_sequence == 1)
        self.assertEqual(self.bridge.sim.data.ctrl[0], 2.0)
        results = self.bridge.set_parameters([Parameter("use_sim_time", value=False)])
        self.assertFalse(results[0].successful)

    def test_message_timestamp_rounding(self):
        from sim_ros.messages import stamp
        value = stamp(1.9999999996)
        self.assertEqual((value.sec, value.nanosec), (2, 0))
        for seconds in (-1, math.nan, math.inf, 2**31):
            with self.assertRaises(ValueError):
                stamp(seconds)


if __name__ == "__main__":
    unittest.main()
