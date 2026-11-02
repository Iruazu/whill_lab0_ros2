"""基準の閾値が、実測した正常時を通し、わざと壊した場合を落とすことを確かめる。"""
import json
from pathlib import Path
import sys
import unittest

REGRESSION = Path(__file__).resolve().parents[1] / 'regression'
sys.path.insert(0, str(REGRESSION))
from compare import compare  # noqa: E402

BASELINE = json.loads((REGRESSION / 'baseline.json').read_text())

# 2026-11-02 に scripts/regression/replay_check.py で測った値。
FAILSAFE_OK = [
    {'engagements': 42, 'engaged_s': 33.64, 'stops_while_driving': 0},
    {'engagements': 40, 'engaged_s': 32.66, 'stops_while_driving': 0},
]
LOCALIZER_OK = [
    {'poses': n, 'max_gap_s': 1.116, 'xy_err_p50_m': 0.0, 'xy_err_p95_m': p95,
     'xy_err_max_m': 0.6828, 'yaw_err_p95_deg': yaw}
    for n, p95, yaw in ((2353, 0.0803, 0.18), (2350, 0.0528, 0.128), (2354, 0.0633, 0.146))
]


class CompareTests(unittest.TestCase):
    def test_measured_normal_runs_pass(self):
        for failsafe in FAILSAFE_OK:
            self.assertEqual(compare({'failsafe': failsafe}, BASELINE), [])
        for localizer in LOCALIZER_OK:
            self.assertEqual(compare({'localizer': localizer}, BASELINE), [])

    def test_wider_layer_d_sector_fails(self):
        wide = {'engagements': 52, 'engaged_s': 43.3, 'stops_while_driving': 9}
        self.assertEqual(len(compare({'failsafe': wide}, BASELINE)), 3)

    def test_shifted_lidar_tf_fails(self):
        shifted = {'poses': 2353, 'max_gap_s': 1.116, 'xy_err_p50_m': 0.0993,
                   'xy_err_p95_m': 0.1321, 'xy_err_max_m': 0.4765, 'yaw_err_p95_deg': 0.166}
        failures = compare({'localizer': shifted}, BASELINE)
        self.assertTrue(any('xy_err_p50_m' in f for f in failures), failures)
        self.assertTrue(any('xy_err_p95_m' in f for f in failures), failures)

    def test_missing_section_is_not_compared(self):
        self.assertEqual(compare({}, BASELINE), [])


if __name__ == '__main__':
    unittest.main()
