#!/usr/bin/env python3
# SPDX-License-Identifier: AGPL-3.0-only
"""Check that VIO evaluation does not hide scale drift or timestamp faults."""
import unittest

import numpy as np

from vio_metrics import compare, rotation_xyzw


class DriftMetricsTest(unittest.TestCase):
    @staticmethod
    def samples(scale=1.0):
        yaw = np.pi / 3
        q = [0.0, 0.0, np.sin(yaw / 2), np.cos(yaw / 2)]
        rotation = rotation_xyzw(q)
        rows = []
        for index in range(11):
            p = np.array([index, index * index / 10, 1.0])
            stamp = 1_000_000_000 + index * 100_000_000
            fields = {"stamp_ns": stamp, "receive_monotonic_ns": stamp}
            rows.append({**fields, "kind": "truth", "position": p.tolist(),
                         "orientation_xyzw": [0.0, 0.0, 0.0, 1.0]})
            rows.append({**fields, "kind": "odometry",
                         "position": (scale * rotation @ p + [3, 4, 5]).tolist(),
                         "orientation_xyzw": q})
        return rows

    def test_rigid_origin_and_yaw_are_removed(self):
        result, _ = compare(self.samples())
        self.assertLess(result["position_rmse_m"], 1e-12)

    def test_scale_error_remains_visible(self):
        result, _ = compare(self.samples(scale=2))
        self.assertGreater(result["end_position_error_m"], 10)

    def test_no_extrapolation_outside_truth_interval(self):
        rows = self.samples()
        rows.append({"kind": "odometry", "stamp_ns": 3_000_000_000,
                     "receive_monotonic_ns": 3_000_000_000,
                     "position": [1e9, 0, 0], "orientation_xyzw": [0, 0, 0, 1]})
        result, _ = compare(rows)
        self.assertLess(result["position_rmse_m"], 1e-12)

    def test_duplicate_sensor_stamp_is_rejected(self):
        rows = self.samples()
        rows.append(rows[-1])
        with self.assertRaises(ValueError):
            compare(rows)

    def test_early_estimator_exit_has_incomplete_coverage(self):
        rows = [r for r in self.samples()
                if r["kind"] != "odometry" or r["stamp_ns"] <= 1_200_000_000]
        result, _ = compare(rows)
        self.assertAlmostEqual(result["odometry_time_coverage_fraction"], 0.2)
        self.assertAlmostEqual(result["unobserved_reference_tail_seconds"], 0.8)

    def test_no_odometry_is_not_comparable(self):
        result, _ = compare([r for r in self.samples() if r["kind"] != "odometry"])
        self.assertFalse(result["comparable"])


if __name__ == "__main__":
    unittest.main()
