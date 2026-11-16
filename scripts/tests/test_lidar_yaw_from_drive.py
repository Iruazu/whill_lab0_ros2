"""lidar_yaw_from_drive.py の計算部分を、合成の走行で検証する。

ROS・bag は使わない（read_bag 以外は numpy だけ）。合成の走行は、後輪の差動駆動
（base_link の原点の速度が常に車体の x を向く）で、真の取り付け T_true の LiDAR を
載せ、別の TF（T_used）で NDT が出した base_link の姿勢を作る。NDT は点群を地図に
合わせるので、出るのは T_map_velodyne で、/pcl_pose はそれに T_used⁻¹ を掛けたもの。
"""
import math
from pathlib import Path
import sys
import unittest

import numpy as np

SCRIPTS = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(SCRIPTS))
import lidar_yaw_from_drive as lyd  # noqa: E402


def drive(segments, dt=0.1, slope_deg=0.0):
    """(v [m/s], ω [rad/s], 秒) の列で走らせ、base_link の真の (t, P, R, v, ω) を返す。

    slope_deg は進行方向の上り坂（地図の傾きの代わり）。base_link の z 軸は路面の法線。
    """
    th, x = 0.0, np.zeros(3)
    pitch = -math.radians(slope_deg)   # 上り坂は車体の前が上がる＝tf2 の pitch は負
    ts, Ps, Rs, vs, ws = [], [], [], [], []
    t = 0.0
    for v, w, dur in segments:
        for _ in range(int(round(dur / dt))):
            R = lyd.rpy_to_R(0.0, pitch, th)
            ts.append(t)
            Ps.append(x.copy())
            Rs.append(R)
            vs.append(v)
            ws.append(w)
            # 弧の弦は中間の向きを向く（一定の v・ω で厳密。前の向きで進めると、
            # 合成のほうに ω·dt/2 の横ずれが入ってしまう）。
            x = x + lyd.rpy_to_R(0.0, pitch, th + 0.5 * w * dt)[:, 0] * v * dt
            th += w * dt
            t += dt
    return np.array(ts), np.array(Ps), np.array(Rs), np.array(vs), np.array(ws)


def ndt_output(P, R, true_tf, used_tf, pos_noise=0.0, yaw_noise_deg=0.0, seed=0):
    """真の base_link と真の取り付けから、used の TF で出る /pcl_pose を作る。"""
    rng = np.random.default_rng(seed)
    tt, Rt = true_tf
    tu, Ru = used_tf
    Rmv = R @ Rt                       # 真の T_map_velodyne
    pmv = P + R @ tt
    pmv = pmv + rng.normal(0, pos_noise, pmv.shape)
    if yaw_noise_deg:
        dz = np.radians(rng.normal(0, yaw_noise_deg, len(R)))
        Rmv = np.array([lyd.rpy_to_R(0, 0, a) @ M for a, M in zip(dz, Rmv)])
    Rest = Rmv @ Ru.T
    Pest = pmv - Rest @ tu
    return Pest, Rest


STRAIGHTS = [(1.0, 0.0, 8.0), (1.0, 0.5, 3.0), (0.8, 0.0, 10.0), (0.5, -0.4, 4.0),
             (1.2, 0.0, 12.0), (0.0, 0.0, 2.0), (-0.5, 0.0, 6.0)]


class YawFromDrive(unittest.TestCase):
    def run_case(self, true_tf, used_tf, eval_tf, **noise):
        t, P, R, v, w = drive(STRAIGHTS, slope_deg=noise.pop('slope_deg', 0.0))
        Pe, Re = ndt_output(P, R, true_tf, used_tf, **noise)
        rows = lyd.estimate(t, Pe, Re, t, v, w, used_tf, eval_tf)
        return rows, lyd.summarize(rows)

    def test_sign_and_value(self):
        # LiDAR が真に +2°（左回り）付いていて、TF は 0° だと思っている。
        true = lyd.tf_from_spec((0.42, 0.36, 0.771, -0.05, 0.40, 2.0))
        used = lyd.tf_from_spec((0.42, 0.36, 0.771, -0.05, 0.40, 0.0))
        rows, s = self.run_case(true, used, used)
        self.assertGreater(s['windows'], 10)
        self.assertAlmostEqual(s['yaw_median_deg'], 2.0, places=6)
        # a（推定の車体から見た進む向き）は −ε。推定の車体は左に回っているので、右へ進んで見える。
        self.assertTrue(all(abs(r['azimuth_deg'] + 2.0) < 1e-6 for r in rows))

    def test_result_does_not_depend_on_eval_yaw(self):
        true = lyd.tf_from_spec((0.42, 0.36, 0.771, -0.05, 0.40, -1.3))
        used = lyd.tf_from_spec((0.42, 0.36, 0.771, -0.05, 0.40, -0.317))
        for eval_yaw in (-0.317, 0.0, 5.0):
            ev = lyd.tf_from_spec((0.42, 0.36, 0.771, -0.05, 0.40, eval_yaw))
            _, s = self.run_case(true, used, ev)
            self.assertAlmostEqual(s['yaw_median_deg'], -1.3, places=6)

    def test_old_tf_with_large_pitch(self):
        # 07-24 の bag の状況：記録時の TF は pitch +9°・並進も違う。台帳の roll・pitch で
        # 作り直せば、二次の混ざり込みも消えて真の yaw が出る。
        true = lyd.tf_from_spec((0.42, 0.36, 0.771, -0.05, 0.40, 0.8))
        old = lyd.tf_from_spec((0.484, 0.3815, 0.7937, -2.025, 8.995, -0.317))
        ev = lyd.tf_from_spec((0.42, 0.36, 0.771, -0.05, 0.40, -0.317))
        _, s = self.run_case(true, old, ev, slope_deg=1.8)
        self.assertAlmostEqual(s['yaw_median_deg'], 0.8, places=6)
        # 作り直さずに旧 TF の base_link のまま測っても、roll・pitch の取り違えが yaw に
        # 混ざるのは二次（この組み合わせで 0.005°）。作り直しは厳密にするためのもので、
        # 旧 TF の bag がこの方法に使えない理由にはならないことを確かめておく。
        _, s_old = self.run_case(true, old, old, slope_deg=1.8)
        diff = abs(s_old['yaw_median_deg'] - 0.8)
        self.assertGreater(diff, 1e-4)
        self.assertLess(diff, 0.05)

    def test_reverse_driving_counts_with_same_sign(self):
        true = lyd.tf_from_spec((0.42, 0.36, 0.771, 0.0, 0.0, 1.5))
        used = lyd.tf_from_spec((0.42, 0.36, 0.771, 0.0, 0.0, 0.0))
        t, P, R, v, w = drive([(-0.8, 0.0, 20.0)])
        Pe, Re = ndt_output(P, R, true, used)
        rows = lyd.estimate(t, Pe, Re, t, v, w, used, used)
        self.assertGreater(len(rows), 3)
        for r in rows:
            self.assertAlmostEqual(r['yaw_deg'], 1.5, places=6)

    def test_noise_is_averaged(self):
        # 位置 2 cm・向き 0.2° の雑音。窓 2 m で 1 窓の方位の雑音は 1° 弱、
        # 中央値は窓の数の平方根で縮む。
        true = lyd.tf_from_spec((0.42, 0.36, 0.771, -0.05, 0.40, 1.0))
        used = lyd.tf_from_spec((0.42, 0.36, 0.771, -0.05, 0.40, -0.317))
        _, s = self.run_case(true, used, used, pos_noise=0.02, yaw_noise_deg=0.2, seed=3)
        self.assertLess(abs(s['yaw_median_deg'] - 1.0), 0.6)
        lo, hi = s['yaw_median_ci95_deg']
        self.assertLess(lo, s['yaw_median_deg'])
        self.assertGreater(hi, s['yaw_median_deg'])

    def test_turns_and_slow_parts_are_excluded(self):
        t, P, R, v, w = drive(STRAIGHTS)
        good, _ = lyd.straight_mask(t, t, v, w)
        self.assertFalse(good[np.abs(w) >= lyd.MAX_RATE].any())
        self.assertFalse(good[np.abs(v) <= lyd.MIN_SPEED].any())
        for i, j in lyd.windows(t, v, good):
            self.assertTrue(good[i:j + 1].all())
            # 道のりは odom の積分（台形）で window_m 以上。
            self.assertGreaterEqual(np.trapz(np.abs(v[i:j + 1]), t[i:j + 1]), lyd.WINDOW_M - 1e-9)

    def test_weaving_and_longitudinal_jitter_do_not_bias(self):
        # 07-24 の記録の様子：Nav2 の蛇行（±0.04 rad/s が数秒ごとに入れ替わる）と、
        # NDT の前後方向の行ったり来たり（±0.25 m）。どちらも yaw を偏らせない。
        segs = []
        for k in range(40):
            segs.append((0.45, 0.04 if k % 2 else -0.04, 2.0))
        t, P, R, v, w = drive(segs)
        true = lyd.tf_from_spec((0.42, 0.36, 0.771, -0.05, 0.40, 1.2))
        used = lyd.tf_from_spec((0.42, 0.36, 0.771, -0.05, 0.40, -0.317))
        Pe, Re = ndt_output(P, R, true, used)
        rng = np.random.default_rng(1)
        Pe = Pe + Re[:, :, 0] * rng.normal(0, 0.25, (len(Pe), 1))
        rows = lyd.estimate(t, Pe, Re, t, v, w, used, used)
        s = lyd.summarize(rows)
        self.assertGreater(s['windows'], 10)
        self.assertLess(abs(s['yaw_median_deg'] - 1.2), 0.05)

    def test_mid_rotation_removes_turn_bias(self):
        # 一定の旋回（0.3 rad/s）でも、中間の回転を使えば各歩の d_k は +x を向く。
        t, P, R, v, w = drive([(0.5, 0.3, 10.0)])
        D = lyd.body_steps(P, R)
        np.testing.assert_allclose(np.degrees(np.arctan2(D[:, 1], D[:, 0])), 0, atol=1e-9)

    def test_stop_and_go(self):
        # 屋外 2 回目の手順：止まる・5 m 進む・止まる・5 m 戻る、を繰り返す。
        segs = [(0.0, 0.0, 5.0)]
        for _ in range(3):
            segs += [(1.0, 0.0, 5.0), (0.0, 0.0, 5.0), (-1.0, 0.0, 5.0), (0.0, 0.0, 5.0)]
        t, P, R, v, w = drive(segs)
        true = lyd.tf_from_spec((0.42, 0.36, 0.771, -0.05, 0.40, 0.6))
        used = lyd.tf_from_spec((0.42, 0.36, 0.771, -0.05, 0.40, -0.317))
        Pe, Re = ndt_output(P, R, true, used, pos_noise=0.01, seed=2)
        rows = lyd.estimate_stops(t, Pe, Re, t, v, used, used)
        self.assertEqual(len(rows), 6)
        for r in rows:
            self.assertLess(abs(r['yaw_deg'] - 0.6), 0.1)
        # 後退の組も同じ符号になる。
        self.assertEqual(sum(r['speed'] < 0 for r in rows), 3)

    def test_rpy_matches_floor_fit_convention(self):
        # lidar_floor_fit.py の rpy_to_R と同じ規約（tf2 の setRPY）であること。
        # そちらは rosbag2_py を import するので、式を直接比べる代わりに性質で確かめる：
        # 小さい yaw は x 軸を +y 側へ回す（反時計回りが正）。
        R = lyd.rpy_to_R(0, 0, math.radians(10))
        self.assertGreater(R[1, 0], 0)
        Rq = lyd.quat_to_R([[0, 0, math.sin(math.radians(5)), math.cos(math.radians(5))]])[0]
        np.testing.assert_allclose(R, Rq, atol=1e-12)
        R2 = lyd.rpy_to_R(0.1, 0.2, 0.3)
        Rz, Ry, Rx = (lyd.rpy_to_R(0, 0, 0.3), lyd.rpy_to_R(0, 0.2, 0), lyd.rpy_to_R(0.1, 0, 0))
        np.testing.assert_allclose(R2, Rz @ Ry @ Rx, atol=1e-12)


if __name__ == '__main__':
    unittest.main()
