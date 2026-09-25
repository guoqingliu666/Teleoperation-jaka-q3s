import math
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
from vla_lab.lookahead_shadow import ShadowSettings, bounded_candidates, diagnose_windows, time_sample_latest


def timed(*xyz):
    return [(index * 50_000_000, tuple(point)) for index, point in enumerate(xyz)]


class LookaheadShadowTests(unittest.TestCase):
    def test_time_sample_never_uses_future_observation(self):
        points = [(0, (0, 0, 0)), (70_000_000, (7, 0, 0)), (120_000_000, (12, 0, 0))]
        sampled = time_sample_latest(points, 50)
        self.assertEqual(sampled[1][1], (0, 0, 0))
        self.assertLessEqual(sampled[1][2], sampled[1][0])

    def test_large_gap_is_subdivided_to_bound_chord(self):
        settings = ShadowSettings(max_chord_mm=8)
        candidates = bounded_candidates(timed((0,0,0), (20,0,0), (30,0,0)), settings)
        diagnosis = diagnose_windows(candidates, settings)
        self.assertLessEqual(diagnosis["max_chord_mm"], 8)
        self.assertTrue(any(point[3] for point in candidates))

    def test_sharp_turn_requires_exact_stop(self):
        settings = ShadowSettings(stop_turn_deg=60)
        candidates = bounded_candidates(timed((0,0,0), (5,0,0), (5,5,0)), settings)
        diagnosis = diagnose_windows(candidates, settings)
        self.assertEqual(diagnosis["exact_stop_windows"], 1)
        self.assertAlmostEqual(diagnosis["max_turn_deg"], 90)

    def test_gentle_turn_is_only_candidate_not_motion_claim(self):
        settings = ShadowSettings(stop_turn_deg=60)
        candidates = bounded_candidates(timed((0,0,0), (5,0,0), (10,1,0)), settings)
        diagnosis = diagnose_windows(candidates, settings)
        self.assertEqual(diagnosis["blend_candidate_windows"], 1)
        self.assertLess(diagnosis["max_turn_deg"], 60)

    def test_invalid_settings_and_too_few_candidates_rejected(self):
        with self.assertRaises(ValueError):
            ShadowSettings(max_chord_mm=20)
        with self.assertRaises(ValueError):
            bounded_candidates(timed((0,0,0), (.1,0,0), (.2,0,0)))


if __name__ == "__main__":
    unittest.main()
