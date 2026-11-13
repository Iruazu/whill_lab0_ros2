"""check_object_placement.py の純粋な計算部分を、小さな合成点群で検証する。

ROS・bag・実機は使わない（read_bag と hires_tables 以外は numpy だけで動く）。
TF の既定値が launch と台帳の値に揃っていることもここで確かめる。
ROS の Python パッケージがあるときだけ、bag の補正ツールとの一致も確かめる。
"""
import datetime
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


def box(x0, h, half=0.1, step=0.02):
    """手前の面が x0、奥行き 0.2 m、高さ h の箱の前面（z 0.1〜h）と上面（base_link）。"""
    ys = np.arange(-half, half + 1e-9, step)
    face = [(x0, y, z) for y in ys for z in np.arange(0.1, h + 1e-9, step)]
    top = [(x, y, h) for x in np.arange(x0, x0 + 0.2 + 1e-9, step) for y in ys]
    return np.array(face + top)


def cone(x0, h, r0, n=4000, seed=0):
    """手前の足元が x0、底の半径 r0、高さ h の円錐の手前半分の表面。"""
    rng = np.random.default_rng(seed)
    z = rng.uniform(0.0, h, n)
    th = rng.uniform(-math.pi / 2, math.pi / 2, n)
    r = r0 * (1 - z / h)
    return np.c_[x0 + r0 - r * np.cos(th), r * np.sin(th), z]


def flat_floor(step=0.05):
    xs, ys = np.meshgrid(np.arange(0.6, 8.0, step), np.arange(-1.4, 1.41, step))
    return xs.ravel(), ys.ravel()


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

    def test_pre_tape_matches_ledger(self):
        # 台帳 §2.1 の「計画書」行（テープ固定前の取付）と一致すること。
        ledger = (REPO / 'docs/calibration-ledger.md').read_text()
        m = re.search(r'^\| 計画書[^|]*\| ([\d.]+) m \| \+([\d.]+)° \| \+([\d.]+)° \|', ledger, re.M)
        self.assertIsNotNone(m)
        self.assertEqual(tuple(map(float, m.groups())), cop.PRE_TAPE_LIDAR_TF[2:5])

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
        self.assertIsNotNone(cop.tf_differs(a, cop.tf_from_args(cop.PRE_TAPE_LIDAR_TF)))

    def test_chain_order(self):
        # T_a_c = T_a_b · T_b_c: b で 90° 回してから c へ 1 m 進むと、a では y に 1 m。
        Rz = cop.rpy_to_R(0, 0, math.pi / 2)
        tf = {('a', 'b'): (Rz, np.zeros(3)), ('b', 'c'): (np.eye(3), np.array([1.0, 0, 0]))}
        R, t = cop.chain(tf, 'a', 'b', 'c')
        np.testing.assert_allclose(t, (0, 1, 0), atol=1e-12)
        np.testing.assert_allclose(R, Rz, atol=1e-12)


class MountAndTfEdgeTests(unittest.TestCase):
    def test_mount_warning_by_date(self):
        d = cop.LEDGER_LIDAR_TF
        self.assertIn('--lidar-tf 0.42 0.36 0.777 0.24 0.97 -0.317',
                      cop.mount_warning(datetime.date(2026, 10, 28), d))
        self.assertIn('混在', cop.mount_warning(datetime.date(2026, 11, 13), d))
        self.assertIsNone(cop.mount_warning(datetime.date(2026, 11, 20), d))
        # TF を明示したら、古い bag でも警告しない（利用者の判断を優先）。
        self.assertIsNone(cop.mount_warning(datetime.date(2026, 10, 28), cop.PRE_TAPE_LIDAR_TF))

    def test_missing_edges(self):
        tf = {('base_link', 'camera_link'): None, ('camera_link', 'camera_depth_frame'): None}
        self.assertEqual(cop.missing_edges(tf, cop.DEPTH_EDGES),
                         [('camera_depth_frame', 'camera_depth_optical_frame')])


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


def _ros_available():
    try:
        import rosbag2_py  # noqa: F401
        import sensor_msgs_py  # noqa: F401
        return True
    except ImportError:
        return False


class ReprojectAgainstToolTests(unittest.TestCase):
    @unittest.skipUnless(_ros_available(), 'ROS の Python パッケージが無い')
    def test_matches_velodyne_reproject_bag(self):
        # 配列版の reproject_points が、bag の補正（#152）の reproject と同じ結果になること。
        from sensor_msgs.msg import PointField
        from sensor_msgs_py import point_cloud2 as pc2
        from std_msgs.msg import Header
        import velodyne_reproject_bag as vrb
        P = scan(DB_V, DB_O, d=4.0, n_az=8)
        fields = [PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
                  PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
                  PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
                  PointField(name='ring', offset=12, datatype=PointField.UINT16, count=1)]
        msg = pc2.create_cloud(Header(frame_id='velodyne'), fields,
                               [(float(x), float(y), float(z), int(r)) for x, y, z, r in P])
        out = vrb.reproject(msg, TABLES, 'ring', {})
        a = pc2.read_points(out, ('x', 'y', 'z', 'ring'))
        got = np.c_[a['x'], a['y'], a['z'], a['ring']].astype(float)
        # ツールへの入力は float32 に丸まるので、こちらも同じ入力から計算する。
        mine = cop.reproject_points(np.c_[P[:, :3].astype(np.float32), P[:, 3]], TABLES)
        np.testing.assert_allclose(got, mine, atol=1e-5)


class DepthTests(unittest.TestCase):
    def test_depth_to_base_wall(self):
        # 光軸方向 2 m の壁。optical（z 前・x 右・y 下）-> camera_link（x 前・y 左・z 上）。
        K = np.array([[100.0, 0, 10], [0, 100.0, 10], [0, 0, 1]])
        img = np.full((21, 21), 2000, np.uint16)
        img[0, 0] = 0  # 無効画素は除かれる
        R_opt = np.array([[0.0, 0, 1], [-1, 0, 0], [0, -1, 0]])
        cam = (np.eye(3), np.array([0.47, 0.378, 0.73]))
        P = cop.depth_to_base([img, img], K, (R_opt, np.zeros(3)), cam)
        self.assertEqual(len(P), 21 * 21 - 1)
        np.testing.assert_allclose(P[:, 0], 2.47, atol=1e-6)
        centre = P[np.argmin(np.hypot(P[:, 1] - 0.378, P[:, 2] - 0.73))]
        np.testing.assert_allclose(centre[:3], (2.47, 0.378, 0.73), atol=1e-6)
        # 画像の右（u 大）は base_link の右（y 小）、下（v 大）は低い（z 小）。
        u20 = P[np.isclose(P[:, 1], 0.378 - 0.2, atol=1e-6)]
        self.assertTrue(len(u20))
        self.assertTrue(np.all(P[:, 3] == -1.0))


class ObjectTests(unittest.TestCase):
    def setUp(self):
        rng = np.random.default_rng(1)
        # 背景: 1〜6 m の平らな床（base_link z=0）。
        xs, ys = np.meshgrid(np.arange(1.0, 6.0, 0.1), np.arange(-1.0, 1.01, 0.1))
        self.floor = np.c_[xs.ravel(), ys.ravel(), np.zeros(xs.size), np.zeros(xs.size)]
        obj = box(2.0, 0.5)
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
        # 手前の面は z 0.1〜0.5 m にあるので、x_front を取った高さもその範囲。
        self.assertTrue(0.1 <= rec['x_front_z'] <= 0.5)
        self.assertNotIn('dz_floor', rec)
        self.assertEqual(set(rec['rings']), {0})
        self.assertGreater(len(P), 100)

    def test_dz_floor_uses_floor_plane(self):
        # 物の置き場の床が base_link の z=0 より 3 cm 高い（路面のうねり）。物はその上に置く。
        up = np.array([0, 0, 0.03])
        frames = [np.c_[f[:, :3] + up, f[:, 3]] for f in self.frames]
        rec, _ = cop.object_metrics(frames, _Tree(self.floor[:, :3] + up),
                                    {'x_m': '2.0', 'height_m': '0.5'}, {'coef': [0.0, 0.0, 0.03]})
        self.assertAlmostEqual(rec['dz'], 0.03, delta=0.02)
        self.assertAlmostEqual(rec['dz_floor'], 0.0, delta=0.02)
        self.assertAlmostEqual(rec['floor_z_at_object'], 0.03, places=6)

    def test_floor_like_points_dropped_for_depth(self):
        # 深度の床のにじみ: 物の手前 10 cm の床の点が 2 cm 浮いて出る。背景を空にして
        # 背景との差では消えない状態にし、床のにじみの除去だけを確かめる。
        smear = np.array([(1.9, y, 0.02, -1.0) for y in np.arange(-0.1, 0.11, 0.01)])
        obj = box(2.0, 0.5)
        frame = [np.vstack([np.c_[obj, np.full(len(obj), -1.0)], smear])]
        tree = _Tree(np.array([[10.0, 10.0, 10.0]]))
        row = {'x_m': '2.0', 'height_m': '0.5'}
        kept, _ = cop.object_metrics(frame, tree, row)
        self.assertLess(kept['dx'], -0.05)  # にじみが前端を手前へ引っぱる
        rec, _ = cop.object_metrics(frame, tree, row, {'coef': [0.0, 0.0, 0.0]},
                                    drop_floor_like=True)
        self.assertEqual(rec['n_floor_like'], len(smear))
        self.assertEqual(rec['floor_ref'], 'empty_floor_plane')
        self.assertLess(abs(rec['dx']), 0.02)

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


class CompareTests(unittest.TestCase):
    def test_same_z_removes_cone_shape(self):
        # 同じ円錐を LiDAR（2 本の ring）と深度（面）で見る。深度だけ +3 cm 奥にずらす。
        c = cone(2.0, 0.7, 0.15)
        lidar = []
        for ring, z in ((3, 0.30), (4, 0.42)):
            band = c[np.abs(c[:, 2] - z) < 0.004]
            lidar.append(np.c_[band[:, :2], np.full(len(band), z), np.full(len(band), ring)])
        L = np.vstack(lidar)
        D = np.c_[c + [0.03, 0, 0], np.full(len(c), -1.0)]
        out = cop.compare_same_z(L, D)
        self.assertEqual([r['ring'] for r in out['rings']], [3, 4])
        self.assertAlmostEqual(out['lidar_minus_depth_median'], -0.03, delta=0.01)
        # 全体の前端どうしでは、深度は足元、LiDAR は 0.3 m 以上の高さを見るので、
        # 形の差（約 +6 cm）が混ざって、本当は −3 cm のずれの符号まで変わる。
        naive = np.percentile(L[:, 0], 3) - np.percentile(D[:, 0], 3)
        self.assertGreater(naive, 0.0)

    def test_accept_failures(self):
        res = [{'bag': 'a', 'lidar': {'detected': True, 'dx': 0.01, 'dz': 0.06}},
               {'bag': 'b', 'lidar': {'detected': True, 'dx': 0.01, 'dz': 0.01}},
               {'bag': 'c', 'lidar': {'detected': False}},
               # cone は dx（形の差を含む）で落とさず、dz だけで判定する。
               {'bag': 'd', 'shape': 'cone', 'lidar': {'detected': True, 'dx': 0.09, 'dz': 0.01}}]
        self.assertEqual(cop.accept_failures(res, ['lidar', 'depth'], 0.05), ['a:lidar'])

    def test_undetected_list(self):
        res = [{'bag': 'a', 'object': 'low', 'expected': ['depth'],
                'lidar': {'detected': False}, 'depth': {'detected': False}},
               {'bag': 'b', 'object': 'cone', 'expected': ['lidar'],
                'lidar': {'detected': True}}]
        self.assertEqual(cop.undetected_list(res), [
            {'bag': 'a', 'object': 'low', 'source': 'lidar', 'expected': False},
            {'bag': 'a', 'object': 'low', 'source': 'depth', 'expected': True}])


class ExpectedTests(unittest.TestCase):
    def setUp(self):
        self.tf = cop.tf_from_args(cop.LEDGER_LIDAR_TF)

    def test_ring_heights_monotonic(self):
        hs = cop.lidar_ring_heights(3.0, 0.0, self.tf)
        self.assertEqual(len(hs), 16)
        self.assertTrue(all(a < b for a, b in zip(hs, hs[1:])))
        # 最下 ring（−10°）は 3 m 先で約 0.771 − 2.58·tan(10°) ≈ 0.32 m。
        self.assertAlmostEqual(hs[0], 0.771 - 2.58 * math.tan(math.radians(10)), delta=0.03)

    def test_low_object_blind_spot(self):
        # 高さ 21 cm の物は 2〜3 m では LiDAR に当たらない（計画書 §1.1）。5 m では当たる。
        low = {'height_m': '0.21'}
        self.assertEqual(cop.expected_sources(dict(low, x_m='2.0'), self.tf, 3.0), {'depth'})
        self.assertEqual(cop.expected_sources(dict(low, x_m='3.0'), self.tf, 3.0), {'depth'})
        self.assertEqual(cop.expected_sources(dict(low, x_m='5.0'), self.tf, 3.0), {'lidar'})
        self.assertEqual(cop.expected_sources({'x_m': '2.0', 'height_m': '0.7'}, self.tf, 3.0),
                         {'lidar', 'depth'})

    def test_expect_column_overrides(self):
        row = {'x_m': '2.0', 'height_m': '0.21'}
        self.assertEqual(cop.expected_sources(dict(row, expect='lidar;depth'), self.tf, 3.0),
                         {'lidar', 'depth'})
        self.assertEqual(cop.expected_sources(dict(row, expect='none'), self.tf, 3.0), set())


class NotesTests(unittest.TestCase):
    def test_empty_rows(self):
        rows = [{'bag': 'e1', 'object': 'empty', 'x_m': '', 'height_m': ''},
                {'bag': 'c2', 'object': 'cone', 'x_m': '2', 'height_m': '0.7', 'shape': 'cone'},
                {'bag': 'b2', 'object': 'box', 'x_m': '2', 'height_m': '0.5'},
                {'bag': 'e2', 'object': 'empty', 'x_m': '', 'height_m': ''}]
        objects, empties = cop.validate_rows(rows, 'e1')
        self.assertEqual([r['bag'] for r in objects], ['c2', 'b2'])
        self.assertEqual([r['shape'] for r in objects], ['cone', 'box'])
        self.assertEqual([r['bag'] for r in empties], ['e2'])

    def test_bad_rows_exit(self):
        for row in ({'bag': 'x', 'object': 'cone', 'x_m': '', 'height_m': '0.7'},
                    {'bag': 'x', 'object': 'cone', 'x_m': '2', 'height_m': '0.7', 'shape': 'ball'},
                    {'bag': 'x', 'object': 'cone', 'x_m': '2', 'height_m': '0.7', 'expect': 'radar'},
                    {'bag': 'e1', 'object': 'cone', 'x_m': '2', 'height_m': '0.7'}):
            with self.subTest(row=row), self.assertRaises(SystemExit):
                cop.validate_rows([row], 'e1')

    def test_empty_drift(self):
        x, y = flat_floor()
        ref = cop.floor_profile([np.c_[x, y, np.zeros(x.size), np.zeros(x.size)]])['plane']
        # 2 本目は床が 2 cm 下がって見える（取付が沈んだ等）。点は背景から 7 cm 以内。
        b = np.c_[x, y, np.full(x.size, -0.02), np.zeros(x.size)][::7]
        tree = _Tree(np.c_[x, y, np.zeros(x.size)][::7])
        out = cop.empty_drift(ref, tree, [b])
        self.assertAlmostEqual(out['d_z0'], -0.02, delta=0.005)
        self.assertAlmostEqual(out['d_slope_x_deg'], 0.0, delta=0.05)
        self.assertEqual(out['changed_frac'], 0.0)


class FloorTests(unittest.TestCase):
    def test_slope(self):
        # base_link で左へ行くほど 2° 下がる床（slope_y_deg = atan(dz/dy) = −2°）。
        rng = np.random.default_rng(2)
        x, y = flat_floor()
        z = -math.tan(math.radians(2.0)) * y + rng.normal(0, 0.005, x.size)
        out = cop.floor_profile([np.c_[x, y, z, np.zeros(x.size)]])
        self.assertAlmostEqual(out['plane']['slope_y_deg'], -2.0, delta=0.1)
        self.assertAlmostEqual(out['plane']['slope_x_deg'], 0.0, delta=0.1)
        self.assertNotIn('roll_deg', out['plane'])
        self.assertAlmostEqual(cop.floor_z(out['plane'], 0.0, 1.0),
                               -math.tan(math.radians(2.0)), delta=0.005)
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
        self.assertIsNone(cop.floor_z(None, 1.0, 0.0))


class ArgsTests(unittest.TestCase):
    def test_defaults(self):
        a = cop.parse_args(['--bag-root', 'r', '--notes', 'n', '--empty', 'e', '--out', 'o'])
        self.assertEqual(tuple(a.lidar_tf), cop.LEDGER_LIDAR_TF)
        self.assertEqual(a.calibration, 'auto')
        self.assertEqual(a.tol, 0.05)
        self.assertEqual(a.require_detected, [])
        self.assertEqual(a.depth_expect_max, 3.0)


if __name__ == '__main__':
    unittest.main()
