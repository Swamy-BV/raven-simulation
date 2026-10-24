# SPDX-License-Identifier: AGPL-3.0-only
import math
import unittest

from crowd_metrics import classify, ground_point


class CrowdMetricsTests(unittest.TestCase):
    def test_camera_ray_and_yaw(self):
        box = [320, 260, 40, 40]
        self.assertEqual(ground_point(box, [0, 0, -4, 1, 0, 0, 0], 100, 0), [4, 0])
        point = ground_point(box, [0, 0, -4, math.sqrt(.5), 0, 0, math.sqrt(.5)], 100, 0)
        self.assertAlmostEqual(point[0], 0)
        self.assertAlmostEqual(point[1], 4)

    def test_rejects_horizon_and_ground_behind_camera(self):
        self.assertIsNone(ground_point([320, 160, 40, 40], [0, 0, -4, 1, 0, 0, 0], 100, 0))
        self.assertIsNone(ground_point([320, 260, 40, 40], [0, 0, 4, 1, 0, 0, 0], 100, 0))

    def test_distinguishes_selected_other_and_ambiguous(self):
        actors = dict(selected=[0, 0], neighbour=[2.3, 0])
        self.assertEqual(classify([.1, 0], actors, "selected")[0], "correct")
        self.assertEqual(classify([2.2, 0], actors, "selected")[0], "other")
        self.assertEqual(classify([1.15, 0], actors, "selected")[0], "ambiguous")
        self.assertEqual(classify([20, 20], actors, "selected")[0], "ambiguous")


if __name__ == "__main__":
    unittest.main()
