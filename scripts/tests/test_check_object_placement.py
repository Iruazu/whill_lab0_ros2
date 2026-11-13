"""check_object_placement.py の純粋な計算部分を、小さな合成点群で検証する。

ROS・bag・実機は使わない（read_bag と hires_tables 以外は numpy だけで動く）。
TF の既定値が launch と台帳の値に揃っていることもここで確かめる。
"""
import math
from pathlib import Path
import re
import sys
import unittest

import numpy as np

SCRIPTS = Path(__file__).resolve().parents[1]
REPO = SCRIPTS.parent
sys.path.insert(0, str(SCRIPTS))
import check_object_placement as cop  # noqa: E402

# 仰角表（ring 昇順）。値は velodyne_pointcloud の VLP16db.yaml / VLP16_hires_db.yaml と同じ。
DB_V = np.radians(-15.0 + 2.0 * np.arange(16))
DB_O = np.array([0.0112, 0.0097, 0.0081, 0.0066, 0.0051, 0.0037, 0.0022, 0.0007,
                 -0.0007, -0.0022, -0.0037, -0.0051, -0.0066, -0.0081, -0.0097, -0.0112])
HI_V = np.radians(-10.0 + (4.0 / 3.0) * np.arange(16))
HI_O = np.zeros(16)
TABLES = (DB_V, DB_O, HI_V, HI_O, np.arange(16))


def scan(v, o, d=5.0, n_az=36):
    """距離 d の全周スキャン（velodyne 系、rawdata.cpp の式）。列は x, y, z, ring。"""
    pts = []
    for ring in range(16):
        for az in np.linspace(0, 2 * math.pi, n_az, endpoint=False):
            xy = d * math.cos(v[ring]) - o[ring] * math.sin(v[ring])
            z = d * math.sin(v[ring]) + o[ring] * math.cos(v[ring])
            pts.append((xy * math.cos(az), xy * math.sin(az), z, ring))
    return np.array(pts)


class _Tree:
    """cKDTree.query の代わり（最近傍距離だけ）。scipy に依存しないため。"""

    def __init__(self, pts):
        self.pts = pts

    def query(self, q):
        d = np.linalg.norm(q[:, None, :] - self.pts[None, :, :], axis=2)
        return d.min(axis=1), d.argmin(axis=1)


class TfTests(unittest.TestCase):
    def test_default_matches_launch(self):
        # launch の static_tf_velodyne と既定値がずれると、解析だけ旧 TF で行ってしまう。
        src = (REPO / 'src/whill_sensors_bringup/launch/static_tf_launch.py').read_text()
        m = re.search(r"_static_tf\('static_tf_velodyne',(.*?)'base_link', 'velodyne'\)",
                      src, re.S)
        self.assertIsNotNone(m)
        nums = [float(x) for x in re.findall(r'-?\d+\.\d+', m.group(1))]
        self.assertEqual(len(nums), 6)
        for a, b in zip(nums, cop.LEDGER_LIDAR_TF):
            self.assertAlmostEqual(a, b, places=6)

    def test_default_matches_ledger(self):
        # 台帳 §2.1 の表の値（yaw は台帳が 4 桁、launch が 3 桁なので丸めの差まで許す）。
        ledger = (REPO / 'docs/calibration-ledger.md').read_text()
        sec = ledger[ledger.index('### 2.1'):ledger.index('### 2.2')]
        for row, want in zip(('x', 'y', 'z', 'roll', 'pitch', 'yaw'), cop.LEDGER_LIDAR_TF):
            m = re.search(rf'^\| {row} \| ([+−-]?\d+\.\d+)', sec, re.M)
            self.assertIsNotNone(m, row)
            self.assertAlmostEqual(float(m.group(1).replace('−', '-')), want, delta=5e-4, msg=row)

    def test_rpy_matches_quaternion(self):
        # tf2 の RPY（固定軸 X→Y→Z）と、同じ姿勢の四元数から作った行列が一致すること。
        r, p, y = map(math.radians, (-0.05, 0.40, -0.317))
        cr, sr = math.cos(r / 2), math.sin(r / 2)
        cp, sp = math.cos(p / 2), math.sin(p / 2)
        cy, sy = math.cos(y / 2), math.sin(y / 2)
        q = (sr * cp * cy - cr * sp * sy, cr * sp * cy + sr * cp * sy,
             cr * cp * sy - sr * sp * cy, cr * cp * cy + sr * sp * sy)
        np.testing.assert_allclose(cop.rpy_to_R(r, p, y), cop.quat_to_R(*q), atol=1e-12)

    def test_tf_differs(self):
        a = cop.tf_from_args(cop.LEDGER_LIDAR_TF)
        self.assertIsNone(cop.tf_differs(a, a))
        old = cop.tf_from_args((0.42, 0.36, 0.777, 0.24, 0.97, -0.317))
        self.assertIsNotNone(cop.tf_differs(a, old))


class CalibrationTests(unittest.TestCase):
    def test_detect(self):
        self.assertEqual(cop.detect_calibration([scan(DB_V, DB_O)]), 'vlp16db')
        self.assertEqual(cop.detect_calibration([scan(HI_V, HI_O)]), 'hires')

    def test_detect_with_missing_top_rings(self):
        # 屋外では上向きの ring が空に抜けて点が出ない。残った ring から判定できること。
        for v, o, want in ((DB_V, DB_O, 'vlp16db'), (HI_V, HI_O, 'hires')):
            P = scan(v, o)
            self.assertEqual(cop.detect_calibration([P[P[:, 3] <= 10]]), want)

    def test_reproject_exact(self):
        # 同じ距離・方位を VLP16db で計算した点を投影し直すと、Hi-Res で計算した点に一致する。
        np.testing.assert_allclose(cop.reproject_points(scan(DB_V, DB_O), TABLES),
                                   scan(HI_V, HI_O), atol=1e-9)

    def test_lidar_to_base_puts_floor_at_zero(self):
        # 床（base_link z=0）を velodyne 系で表した点が、既定の TF で z=0 に戻ること。
        R, t = cop.tf_from_args(cop.LEDGER_LIDAR_TF)
        g = np.array([[x, y, 0.0] for x in np.arange(2, 6, 0.5) for y in (-1, 0, 1)])
        P = np.c_[(g - t) @ R, np.zeros(len(g))]  # base -> velodyne は Rᵀ(p − t)
        out = cop.lidar_to_base([P], 'hires', (R, t))[0]
        np.testing.assert_allclose(out[:, :3], g, atol=1e-12)


class ObjectTests(unittest.TestCase):
    def setUp(self):
        rng = np.random.default_rng(1)
        # 背景: 2〜6 m の平らな床（base_link z=0）。
        xs, ys = np.meshgrid(np.arange(1.0, 6.0, 0.1), np.arange(-1.0, 1.01, 0.1))
        self.floor = np.c_[xs.ravel(), ys.ravel(), np.zeros(xs.size), np.zeros(xs.size)]
        # 物: x 2.0〜2.2 m、高さ 0.5 m の箱の前面と上面。
        face = [(2.0, y, z) for y in np.arange(-0.1, 0.11, 0.02) for z in np.arange(0.1, 0.51, 0.02)]
        top = [(x, y, 0.5) for x in np.arange(2.0, 2.21, 0.02) for y in np.arange(-0.1, 0.11, 0.02)]
        obj = np.array(face + top)
        self.frames = []
        for _ in range(5):
            noise = rng.normal(0, 0.003, obj.shape)
            o = np.c_[obj + noise, np.zeros(len(obj))]
            self.frames.append(np.vstack([self.floor, o]))
        self.tree = _Tree(self.floor[:, :3])

    def test_metrics(self):
        rec, P = cop.object_metrics(self.frames, self.tree,
                                    {'x_m': '2.0', 'height_m': '0.5', 'y_m': '', 'half_width_m': ''})
        self.assertTrue(rec['detected'])
        self.assertLess(abs(rec['dx']), 0.02)
        self.assertLess(abs(rec['dz']), 0.02)
        self.assertLess(abs(rec['dy']), 0.02)
        self.assertEqual(set(rec['rings']), {0})
        self.assertGreater(len(P), 100)

    def test_not_detected_when_absent(self):
        rec, P = cop.object_metrics([self.floor] * 3, self.tree, {'x_m': '2.0', 'height_m': '0.5'})
        self.assertEqual(rec, {'detected': False})
        self.assertIsNone(P)

    def test_persistent_drops_transient(self):
        # 1 フレームにだけ出る点（雨粒・通行人の一瞬）は残らない。
        frames = [self.floor.copy() for _ in range(10)]
        frames[3] = np.vstack([frames[3], [[3.0, 0.0, 0.3, 0]]])
        kept = cop.persistent(frames)
        self.assertFalse(np.any(np.isclose(kept[:, 2], 0.3)))

    def test_accept_failures(self):
        res = [{'bag': 'a', 'lidar': {'detected': True, 'dx': 0.01, 'dz': 0.06}},
               {'bag': 'b', 'lidar': {'detected': True, 'dx': 0.01, 'dz': 0.01}},
               {'bag': 'c', 'lidar': {'detected': False}}]
        self.assertEqual(cop.accept_failures(res, ['lidar', 'depth'], 0.05), ['a:lidar'])


class FloorTests(unittest.TestCase):
    def test_cross_slope(self):
        # 横断勾配 2°（y が増えると下がる）の床。roll_deg と高さのばらつきが出ること。
        rng = np.random.default_rng(2)
        xs, ys = np.meshgrid(np.arange(0.6, 8.0, 0.05), np.arange(-1.4, 1.41, 0.05))
        x, y = xs.ravel(), ys.ravel()
        z = -math.tan(math.radians(2.0)) * y + rng.normal(0, 0.005, x.size)
        out = cop.floor_profile([np.c_[x, y, z, np.zeros(x.size)]])
        self.assertAlmostEqual(out['plane']['roll_deg'], -2.0, delta=0.1)
        self.assertAlmostEqual(out['plane']['pitch_deg'], 0.0, delta=0.1)
        self.assertTrue(out['bins'])
        self.assertLess(max(b['z_sigma'] for b in out['bins']), 0.06)

    def test_deterministic(self):
        # 乱数を呼び出しごとに作るので、同じ入力なら何度呼んでも同じ結果。
        rng = np.random.default_rng(3)
        P = np.c_[rng.uniform(0.6, 8, 3000), rng.uniform(-1.4, 1.4, 3000),
                  rng.normal(0, 0.01, 3000), np.zeros(3000)]
        self.assertEqual(cop.floor_profile([P]), cop.floor_profile([P]))

    def test_too_few_points(self):
        self.assertEqual(cop.floor_profile([np.zeros((10, 4))]), {'bins': [], 'plane': None})


class ArgsTests(unittest.TestCase):
    def test_defaults(self):
        a = cop.parse_args(['--bag-root', 'r', '--notes', 'n', '--empty', 'e', '--out', 'o'])
        self.assertEqual(tuple(a.lidar_tf), cop.LEDGER_LIDAR_TF)
        self.assertEqual(a.calibration, 'auto')
        self.assertEqual(a.tol, 0.05)


if __name__ == '__main__':
    unittest.main()
