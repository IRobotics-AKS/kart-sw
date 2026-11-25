import math
import unittest

from sim.runtime import Simulation
from sim_ros.command_gate import CommandGate


class CommandTests(unittest.TestCase):
    def setUp(self):
        self.wall = 10.0
        self.sim = Simulation("demo", accel_noise_std=0.0, gyro_noise_std=0.0, seed=0)
        self.gate = CommandGate(self.sim, 0.25, 0.5, monotonic=lambda: self.wall)

    def send(self, **overrides):
        return self.gate.accept(**{
            "session": self.gate.session, "sequence": 0, "stamp_ns": 0,
            "validity_ns": 100_000_000, "names": ["x_force", "yaw_torque"],
            "efforts": [2.0, 0.0], "paused": False, **overrides})

    def test_valid_command_and_simulation_expiration(self):
        self.assertIsNone(self.send())
        self.assertEqual(self.sim.data.ctrl[0], 2.0)
        for _ in range(20):
            self.sim.step()
        self.gate.expire()
        self.assertEqual(tuple(self.sim.data.ctrl), (0.0, 0.0))
        self.assertIsNotNone(self.send())  # old sequence cannot refresh watchdog

    def test_wall_timeout_without_simulation_advancement(self):
        self.send()
        self.wall += 0.5
        self.gate.expire()
        self.assertEqual(self.sim.data.ctrl[0], 0.0)

    def test_bad_commands_do_not_replace_effort_or_refresh_deadline(self):
        self.send()
        self.wall += 0.4
        for changes in ({"sequence": 0}, {"sequence": -1}, {"session": "old"},
                        {"stamp_ns": 1}, {"stamp_ns": -1}, {"validity_ns": 0},
                        {"validity_ns": 300_000_000}, {"paused": True},
                        {"names": ["unknown", "yaw_torque"]}, {"efforts": [0.0, math.inf]}):
            with self.subTest(changes=changes):
                args = {"sequence": 1, **changes}
                self.assertIsNotNone(self.send(**args))
                self.assertEqual(self.sim.data.ctrl[0], 2.0)
                self.assertEqual(self.gate.last_sequence, 0)
        self.wall += 0.1
        self.gate.expire()
        self.assertEqual(self.sim.data.ctrl[0], 0.0)

    def test_reset_session_rejects_queued_pre_reset_command(self):
        old_session = self.gate.session
        self.send()
        self.sim.reset()
        self.gate.invalidate()
        self.assertNotEqual(old_session, self.gate.session)
        self.assertIsNotNone(self.send(session=old_session, sequence=100))
        self.assertIsNone(self.send())

    def test_expired_and_reordered_commands(self):
        for _ in range(21):
            self.sim.step()
        self.assertIsNotNone(self.send())
        stamp = round(self.sim.snapshot.time_s * 1e9)
        self.assertIsNone(self.send(stamp_ns=stamp, sequence=2))
        self.assertIsNotNone(self.send(stamp_ns=stamp, sequence=1))

    def test_invalid_timeouts(self):
        for command, wall in [(0, 1), (1, 0), (math.nan, 1), (1, math.inf)]:
            with self.assertRaises(ValueError):
                CommandGate(self.sim, command, wall)

    def test_viewer_controls_cannot_bypass_command_gate(self):
        self.sim.data.ctrl[:] = [9.0, 1.0]
        self.gate.expire()
        self.assertEqual(tuple(self.sim.data.ctrl), (0.0, 0.0))
        self.send()
        self.sim.data.ctrl[:] = [9.0, 1.0]
        self.gate.expire()
        self.assertEqual(tuple(self.sim.data.ctrl), (2.0, 0.0))


if __name__ == "__main__":
    unittest.main()
