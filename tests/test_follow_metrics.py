# SPDX-License-Identifier: AGPL-3.0-only
import unittest

from follow_metrics import Series, projection_metrics, formation_error


class MetricTests(unittest.TestCase):
    def test_distinguishes_rear_side_and_front_formation(self):
        self.assertEqual(formation_error([-10, 0], [1, 0]), (True, 0, 0))
        self.assertEqual(formation_error([0, 10], [1, 0]), (False, 10, 90))
        self.assertEqual(formation_error([10, 0], [1, 0]), (False, 0, 180))
        self.assertEqual(formation_error([0, -10], [0, 1]), (True, 0, 0))

    def test_interpolates_capture_time(self):
        series = Series([(0, [0, 0, 0]), (100_000_000, [10, 20, 30])])
        self.assertEqual(series.at(25_000_000), [2.5, 5, 7.5])
        self.assertEqual(series.at(100_000_000), [10, 20, 30])
        self.assertEqual(series.nearest_age_ms(25_000_000), 25)

    def test_rejects_missing_or_distant_truth(self):
        series = Series([(0, [0, 0, 0]), (1_000_000_000, [10, 20, 30])])
        self.assertIsNone(series.at(-1))
        self.assertIsNone(series.at(500_000_000))
        self.assertIsNone(series.at(1_000_000_001))

    def test_error_and_latency_use_capture_timestamp(self):
        truth = Series([(1_000_000_000, [8, 12, 0])])
        rows = [{"stamp_ns": 1_000_000_000, "received_ns": 1_120_000_000,
                 "frame": "px4_local_ned", "point": [1, 2, 0]}]
        metrics = projection_metrics(rows, truth, [4, 6, 0], 0, 2_000_000_000)
        self.assertEqual(metrics["matched_samples"], 1)
        self.assertEqual(metrics["error_to_actor_origin_m"]["rms"], 5)
        self.assertEqual(metrics["capture_to_ros_receipt_ms"]["mean"], 120)
        rows[0]["frame"] = "px4_local_enu"
        with self.assertRaises(ValueError):
            projection_metrics(rows, truth, [4, 6, 0], 0, 2_000_000_000)


if __name__ == "__main__":
    unittest.main()
