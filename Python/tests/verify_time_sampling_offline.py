"""时间采样诊断的纯内存测试；不加载JAKA SDK。"""
import importlib.util
from pathlib import Path
import unittest

spec = importlib.util.spec_from_file_location(
    "time_sampling_offline", Path(__file__).resolve().parents[1] / "分析时间采样与转角_离线.py")
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


class Tests(unittest.TestCase):
    def test_fixed_time_spacing_keeps_latest_observation(self):
        points = [(0, (0., 0., 0.)), (20_000_000, (1., 0., 0.)),
                  (70_000_000, (3., 0., 0.)), (120_000_000, (4., 0., 0.))]
        sampled = module.time_sample(points, 50)
        self.assertEqual([item[0] for item in sampled], [0, 50_000_000, 100_000_000])
        self.assertEqual([item[1][0] for item in sampled], [0., 1., 3.])

    def test_reversal_is_reported_not_silently_smoothed(self):
        points = [(i * 50_000_000, (float(x), 0., 0.))
                  for i, x in enumerate((0, 2, 4, 2, 0))]
        result = module.diagnose(points, 50)
        self.assertEqual(result["reversals_over_120deg"], 1)
        self.assertEqual(result["polyline_length_mm"], 8.)

    def test_period_outside_review_range_rejected(self):
        points = [(i * 50_000_000, (float(i), 0., 0.)) for i in range(3)]
        with self.assertRaises(ValueError):
            module.time_sample(points, 10)


if __name__ == "__main__":
    unittest.main()
