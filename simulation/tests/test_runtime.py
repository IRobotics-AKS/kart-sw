import math
from pathlib import Path
import tempfile
import unittest

import mujoco

from sim.runtime import Simulation


class SimulationTests(unittest.TestCase):
    def make(self, model="demo", **kwargs):
        return Simulation(model, **{"accel_noise_std": 0.0, "gyro_noise_std": 0.0,
                                    "seed": 7, **kwargs})

    def test_fall_and_consistent_timestamp(self):
        sim = self.make()
        first = sim.snapshot
        sample = sim.step()
        self.assertAlmostEqual(sample.time_s, sim.timestep)
        self.assertLess(sample.position_m[2], first.position_m[2])
        self.assertAlmostEqual(sample.linear_velocity_mps[2], -9.81 * sim.timestep)
        self.assertEqual(sample.orientation_xyzw, (0.0, 0.0, 0.0, 1.0))
        for value in sample.acceleration_mps2:
            self.assertAlmostEqual(value, 0.0)  # free-fall specific force

    def test_settled_accelerometer_includes_gravity(self):
        sim = self.make()
        for _ in range(1000):
            sample = sim.step()
        self.assertAlmostEqual(sample.acceleration_mps2[2], 9.81, places=3)
        self.assertAlmostEqual(sample.linear_velocity_mps[2], 0.0, places=3)

    def test_body_origin_velocity_and_quaternion_order(self):
        xml = '''<mujoco><option gravity="0 0 0"/><worldbody>
          <body name="base_link" quat="0.707106781 0 0 0.707106781">
            <freejoint/><geom type="sphere" pos="0.3 0 0" size="0.1" mass="1"/>
            <site name="imu"/>
          </body></worldbody><sensor>
          <accelerometer name="imu_accel" site="imu"/>
          <gyro name="imu_gyro" site="imu"/></sensor></mujoco>'''
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "offset.xml"
            path.write_text(xml)
            sim = self.make(str(path))
        sim.data.qvel[:] = [1, 0, 0, 0, 0, 2]
        snapshot = sim._snapshot()
        self.assertAlmostEqual(snapshot.orientation_xyzw[2], math.sqrt(0.5))
        # World +X becomes body -Y. A COM offset must not alter origin twist.
        self.assertAlmostEqual(snapshot.linear_velocity_mps[0], 0.0, places=6)
        self.assertAlmostEqual(snapshot.linear_velocity_mps[1], -1.0, places=6)
        self.assertAlmostEqual(snapshot.angular_velocity_rps[2], 2.0)
        self.assertAlmostEqual(snapshot.gyro_rps[2], 2.0)

    def test_noise_and_reset_are_repeatable(self):
        sim = self.make(accel_noise_std=0.2, gyro_noise_std=0.01)
        initial = sim.snapshot
        samples = [sim.step() for _ in range(10)]
        self.assertEqual(sim.reset(), initial)
        self.assertEqual([sim.step() for _ in range(10)], samples)
        other = self.make(accel_noise_std=0.2, gyro_noise_std=0.01, seed=8)
        self.assertNotEqual(initial.acceleration_mps2, other.snapshot.acceleration_mps2)
        self.assertEqual(initial.position_m, other.snapshot.position_m)

    def test_cached_snapshot_does_not_resample_noise(self):
        sim = self.make(accel_noise_std=1.0)
        self.assertIs(sim.snapshot, sim.snapshot)

    def test_validation(self):
        for kwargs in ({"accel_noise_std": -1.0}, {"gyro_noise_std": math.nan},
                       {"timestep": math.inf}, {"timestep": 0.0}, {"timestep": 1e-10},
                       {"seed": -1}, {"body": "missing"}, {"gyro": "imu_accel"}):
            with self.subTest(kwargs=kwargs), self.assertRaises(ValueError):
                self.make(**kwargs)
        with self.assertRaises(ValueError):
            self.make("/missing/model.xml")

    def test_unsupported_actuators_are_rejected(self):
        from importlib.resources import files
        xml = files("sim").joinpath("assets/bridge_demo.xml").read_text()
        xml = xml.replace('gear="1"', 'gear="2"')
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "actuated.xml"
            path.write_text(xml)
            with self.assertRaisesRegex(ValueError, "unit-gear"):
                self.make(str(path))

    def test_effort_moves_body_and_reset_clears_controls(self):
        sim = self.make()
        sim.set_efforts(["x_force", "yaw_torque"], [2.0, 0.0])
        sample = sim.step()
        self.assertGreater(sample.position_m[0], 0)
        self.assertAlmostEqual(sample.linear_velocity_mps[0], 2.0 * sim.timestep)
        sim.reset()
        self.assertTrue(all(value == 0 for value in sim.data.ctrl))

    def test_invalid_efforts_leave_previous_controls_unchanged(self):
        sim = self.make()
        sim.set_efforts(["x_force", "yaw_torque"], [2.0, 1.0])
        for names, values in [(["x_force"], [1.0]),
                              (["x_force", "x_force"], [1.0, 1.0]),
                              (["x_force", "yaw_torque"], [5.0, 3.0]),
                              (["x_force", "yaw_torque"], [math.nan, 0.0])]:
            with self.subTest(names=names, values=values), self.assertRaises(ValueError):
                sim.set_efforts(names, values)
            self.assertEqual(tuple(sim.data.ctrl), (2.0, 1.0))

    def test_timestep_override(self):
        sim = self.make(timestep=0.01)
        self.assertEqual(sim.step().time_s, 0.01)


if __name__ == "__main__":
    unittest.main()
