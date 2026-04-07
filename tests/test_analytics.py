import os
import sys
import unittest

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))

from analytics import compute_channel_metrics


class AnalyticsTests(unittest.TestCase):
    def test_compute_basic_and_error_metrics(self) -> None:
        time_s = np.array([0.0, 0.1, 0.2, 0.3])
        target = np.array([1.0, 1.0, 1.0, 1.0])
        final = np.array([0.5, 0.8, 1.0, 1.0])

        metrics = compute_channel_metrics(time_s, target, final)

        self.assertAlmostEqual(metrics["mean"], 0.825, places=3)
        self.assertAlmostEqual(metrics["max_abs_error"], 0.5, places=3)
        self.assertIsNone(metrics["overshoot_pct"])

    def test_return_none_like_values_when_samples_insufficient(self) -> None:
        time_s = np.array([0.0])
        target = np.array([1.0])
        final = np.array([1.0])

        metrics = compute_channel_metrics(time_s, target, final)

        self.assertIsNone(metrics["rise_time_s"])
        self.assertIsNone(metrics["settling_time_s"])
