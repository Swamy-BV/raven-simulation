# SPDX-License-Identifier: AGPL-3.0-only
"""Check diagnostic trajectories before sending them to the HITL controller."""
import math
import unittest
from yaw_profile import yaw_response_sample


class YawProfileTests(unittest.TestCase):
    def test_bounded_consistent_derivatives(self):
        origin = [0.0, 0.0, -4.5]
        heading = math.pi / 2
        phases = set()
        for index in range(16400):
            elapsed = index / 100
            sample = yaw_response_sample(elapsed, origin, heading)
            if sample is None:
                break
            name, position, velocity, acceleration, yaw, rate = sample
            phases.add(name)
            self.assertTrue(all(math.isfinite(v) for v in position + velocity + acceleration + [yaw, rate]))
            self.assertTrue(-.001 <= position[0] <= 340.001)
            self.assertTrue(-80.001 <= position[1] <= 80.001)
            self.assertEqual(position[2], origin[2])
            self.assertLessEqual(math.hypot(*velocity), 18.000001)
            self.assertLessEqual(math.hypot(*acceleration), 6)
            step = 1e-5
            after = yaw_response_sample(elapsed + step, origin, heading)
            if after is None or after[0] != name:
                continue
            for axis in range(3):
                self.assertAlmostEqual((after[1][axis] - position[axis]) / step, velocity[axis], delta=.0001)
                self.assertAlmostEqual((after[2][axis] - velocity[axis]) / step, acceleration[axis], delta=.0001)
            jerk = [(after[3][axis] - acceleration[axis]) / step for axis in range(3)]
            self.assertLessEqual(math.hypot(*jerk), 4)
        self.assertEqual(len(phases), 12)
        self.assertIsNone(yaw_response_sample(164, origin, heading))


if __name__ == '__main__':
    unittest.main()
