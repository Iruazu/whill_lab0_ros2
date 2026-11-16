#!/usr/bin/env python3
# SPDX-License-Identifier: BSD-3-Clause
"""走行 bag の NDT の姿勢から base_link→velodyne の yaw を求める（#149）。

用途: 校正台帳 §2.1 の yaw の測定。床面フィット（lidar_floor_fit.py）では yaw が
観測できないので、走行の運動学を使う。

原理:
  WHILL は後輪の差動駆動で横に滑らない（非ホロノミック）。base_link の原点は後輪の
  ハブ中心の中点（台帳 §1）なので、原点の速度は常に車体の x 軸の向きを向く。
  NDT は LiDAR の点群を地図に合わせ、T_map_velodyne を求める。/pcl_pose は、それに
  使った TF の逆を掛けた T_map_base = T_map_velodyne · T_base_velodyne_used⁻¹ である。
  使った TF の yaw ψ_used が真の ψ_true と ε = ψ_true − ψ_used だけ違うと、
    R_est = R_mv · R_used⁻¹ = R_true · R_t · R_used⁻¹ ≈ R_true · Rz(ε)
  で、推定の向きは真の向きより ε だけ左（反時計回り）に回る。進む向き（推定の軌跡の
  向き）は、回転が一定の区間では T_base_velodyne の並進のずれが定数のずれになるだけ
  なので、真の進行方向＝真の車体 x のまま。したがって、進んだ向きを推定の車体系で
  表した方位角 a = atan2(d_y, d_x)（d = R_est⁻¹ Δp）は −ε になる:
    ψ_true = ψ_used − a     （a が正＝推定の車体から見て左へ進んでいる＝LiDAR が右に回っている）
  符号は tf2 の規約（R = Rz(yaw)·Ry(pitch)·Rx(roll)、velodyne 系の点を base_link 系へ
  移す、yaw は z 軸まわりに反時計回りが正）。LiDAR が +ψ 回っていれば、車体の前方は
  velodyne 系で方位 −ψ に見える、と言い換えても同じ。

  NDT の結果 T_map_velodyne は、使った TF によらない（収束していれば点群を地図に合わせ
  た姿勢そのもの）。そこで、記録の /pcl_pose を記録時の TF（--used-tf、既定は bag の
  /tf_static）で T_map_velodyne に戻し、評価用の TF（--eval-tf。roll・pitch・並進は
  台帳の値）で base_link を作り直してから a を測る。これで旧 TF（pitch +9°）で走った
  bag でも、roll・pitch の取り違えが yaw に二次で混ざる分を除ける。--eval-tf の yaw は
  基準として使うだけで、結果は ψ_eval − a（a は ψ_eval を変えても同じだけ動くので、
  基準の選び方によらない）。

  3 次元のまま車体系で方位角を取るので、地図の傾き（campus の 1.81°）や坂は、進んだ
  向きと車体の x が同じ 3 次元の向きである限り効かない。

区間の選び方と推定量:
  /odometry/filtered（または --rate-topic で /whill/odom）の角速度 |ω| と速さ |v|
  で「曲がりが小さく・十分速い」時刻を選び、連続した区間を odom の道のり --window-m
  ごとの重ならない窓に切る。各窓で、隣り合う姿勢の移動 Δp_k を、その間の中間の姿勢
  （R_k と R_k+1 の中間の回転）の車体系へ移した d_k を足し合わせ、和 D の方位角を a
  とする。非ホロノミックなら各 d_k が +x を向くので、窓の中で向きが変わっても（蛇行
  していても）偏らない。向きが一定なら D は弦（終点 − 始点）と同じで、NDT の前後方向
  の行ったり来たり（07-24 の記録で 0.2〜0.3 m）は打ち消し合う。後退（v < 0）は
  向きを反転して数える。

  TF の並進のずれ・base_link の原点の置き方のずれ δ（巻尺で約 1 cm）は、推定の位置に
  R·δ のずれを足す。窓の始めと終わりの差は (R_j − R_i)·δ で、大きさは |Δθ|·|δ|
  （Δθ は窓の中の正味の向きの変化）。そこで偏りは ω ではなく正味の向きの変化で抑える
  （--max-turn-deg 15°・δ 2 cm・窓 2 m で 0.15°。符号は曲がる向きで変わる）。ω の上限は
  その場旋回など、進まずに回る時刻を外すためだけに置く。

誤差の要因（結果の解釈に使う）:
  - NDT の位置の雑音 σ_p は、窓の長さ L に対して約 √2·σ_p/L の方位の雑音になる。
  - use_odom: true では、次の NDT の初期値は「前の姿勢＋odom の移動量」で、odom の移動
    は推定の車体 x 方向に入る。NDT が毎回その横ずれ（v·dt·ε）を直しきらないと、軌跡が
    推定の向きに引っぱられて a が 0 に寄る（感度が落ちる）。定常の横ずれは定数なので
    偏りにはならないはず。07-24 の bag を再生時の TF の yaw だけ +3° ずらして回した
    比較では、推定の差は窓 72 組の中央値で −0.005° だった（docs/lidar-yaw-measurement.md
    §3.2）。
  - 横断勾配での横滑り、地図の局所的なゆがみ（窓ごとの回転）は窓ごとに違う符号で入る。

例:
  python3 scripts/lidar_yaw_from_drive.py \\
      ~/bags/m7-bench-data/2026-07-24-live-campus/rosbag2_2026_07_24-19_05_26-hires
  # 止まる・まっすぐ進む・止まる、を繰り返した bag（屋外 2 回目の手順）
  python3 scripts/lidar_yaw_from_drive.py <bag> --stops --rate-topic /whill/odom
  # ndt_replay_compare.py の出力（poses_full.txt）を使う。odom は元の bag から読む
  python3 scripts/lidar_yaw_from_drive.py <bag> --poses <out>/poses_full.txt \\
      --used-tf "0.42 0.36 0.771 -0.05 0.40 -0.317"

計算は numpy だけ。bag を読む関数だけが rosbag2_py を中で import する。
"""
import argparse
import json
import math
import sys

import numpy as np

# 評価に使う TF の既定は台帳 §2.1 と static_tf_launch.py の値（yaw は基準として使う
# だけで、結果には効かない）。
LEDGER_TF = (0.42, 0.36, 0.771, -0.05, 0.40, -0.317)

# 曲がりの判定（上の「区間の選び方と推定量」）。07-24 の記録は Nav2 の蛇行（#101）で
# |ω| の中央値が 0.043 rad/s あり、|ω| < 0.02 では 4 秒以上続く区間が 1 つも無かった。
# 推定量は曲がりに偏らないので、ω はその場旋回を外す程度の 0.3 rad/s にした。
MAX_RATE = 0.3
# 低速では位置の雑音に対して進む量が小さく、方位が決まらない。
MIN_SPEED = 0.3
# 窓の道のり（odom）。NDT の横の雑音 2 cm なら 1 窓の方位の雑音は約 0.8°。
WINDOW_M = 2.0
# 窓の中で NDT の向きが正味で変わってよい上限。TF の並進のずれの影響（|Δθ|·|δ|/L）
# の上限を決める。odom と食い違う窓（NDT の向きの飛び）もこれで外れる。
MAX_TURN_DEG = 15.0


def rpy_to_R(r, p, y):
    """tf2 の setRPY と同じ Rz(y)·Ry(p)·Rx(r)。lidar_floor_fit.py と同じ式。"""
    cr, sr, cp, sp, cy, sy = (math.cos(r), math.sin(r), math.cos(p), math.sin(p),
                              math.cos(y), math.sin(y))
    return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                     [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                     [-sp, cp * sr, cp * cr]])


def quat_to_R(q):
    """(x, y, z, w) の配列 (N, 4) → (N, 3, 3)。"""
    q = np.asarray(q, float)
    q = q / np.linalg.norm(q, axis=-1, keepdims=True)
    x, y, z, w = q[..., 0], q[..., 1], q[..., 2], q[..., 3]
    return np.stack([
        np.stack([1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)], -1),
        np.stack([2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)], -1),
        np.stack([2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)], -1),
    ], -2)


def tf_from_spec(spec):
    """(x, y, z, roll°, pitch°, yaw°) → (t, R)。R は velodyne 系の点を base_link 系へ移す。"""
    x, y, z, r, p, yw = map(float, spec)
    return np.array([x, y, z]), rpy_to_R(*map(math.radians, (r, p, yw)))


def rebase(P, R, used, evaluated):
    """used の TF で出た base_link の姿勢 (P, R) を、evaluated の TF の base_link に直す。

    T_map_velodyne = T_map_base_used · T_base_velodyne_used はどちらでも同じなので、
      R_mv = R·R_u、p_mv = P + R·t_u
      R_e = R_mv·R_e_tfᵀ、P_e = p_mv − R_e·t_e
    """
    tu, Ru = used
    te, Re = evaluated
    Rmv = R @ Ru
    pmv = P + R @ tu
    Rb = Rmv @ Re.T
    Pb = pmv - Rb @ te
    return Pb, Rb


def heading(R):
    """map の xy 面での車体 x の方位（ZYX の yaw）。"""
    return np.arctan2(R[..., 1, 0], R[..., 0, 0])


def straight_mask(t, odom_t, odom_v, odom_w, max_rate=MAX_RATE, min_speed=MIN_SPEED):
    """姿勢の時刻ごとに「曲がりが小さく・十分速い」かを odom から決める。

    odom の時刻の間は線形補間する。odom の範囲の外は使わない。
    """
    inside = (t >= odom_t[0]) & (t <= odom_t[-1])
    v = np.interp(t, odom_t, odom_v)
    w = np.interp(t, odom_t, odom_w)
    return inside & (np.abs(w) < max_rate) & (np.abs(v) > min_speed), v


def windows(t, v, good, window_m=WINDOW_M):
    """good が続く区間を、odom の道のり window_m ごとの重ならない窓 [i, j] に切る。

    道のりは NDT ではなく odom の |v| の積分で測る。NDT の前後の行ったり来たりで
    道のりが水増しされ、短すぎる窓ができるのを避けるため。
    """
    out = []
    n = len(t)
    i = 0
    while i < n:
        if not good[i]:
            i += 1
            continue
        k = i
        while k + 1 < n and good[k + 1]:
            k += 1
        # 区間 [i, k] を窓に切る。最後の短い残りは捨てる（雑音が大きい）。
        seg_v = np.abs(v[i:k + 1])
        s = np.concatenate([[0.0], np.cumsum(0.5 * (seg_v[1:] + seg_v[:-1]) * np.diff(t[i:k + 1]))])
        start = 0
        for m in range(1, len(s)):
            if s[m] - s[start] >= window_m:
                out.append((i + start, i + m))
                start = m
        i = k + 1
    return out


def mid_rotations(R):
    """隣り合う回転 R_k, R_k+1 の中間（R_k·exp(½·log(R_kᵀR_k+1))）の列 (N−1, 3, 3)。

    10 Hz・0.3 rad/s で 1 歩に 1.7° 回るので、R_k をそのまま使うと ω·dt/2 ≈ 0.9° の
    偏りになる。中間の回転なら一定の回転では偏りが消える。
    """
    rel = np.einsum('nji,njk->nik', R[:-1], R[1:])
    ang = np.arccos(np.clip((np.trace(rel, axis1=1, axis2=2) - 1) / 2, -1, 1))
    axis = np.stack([rel[:, 2, 1] - rel[:, 1, 2], rel[:, 0, 2] - rel[:, 2, 0],
                     rel[:, 1, 0] - rel[:, 0, 1]], -1)
    norm = np.linalg.norm(axis, axis=1, keepdims=True)
    axis = np.where(norm > 1e-12, axis / np.maximum(norm, 1e-12), [0.0, 0.0, 1.0])
    h = ang / 2
    K = np.zeros((len(h), 3, 3))
    K[:, 0, 1], K[:, 0, 2], K[:, 1, 2] = -axis[:, 2], axis[:, 1], -axis[:, 0]
    K[:, 1, 0], K[:, 2, 0], K[:, 2, 1] = axis[:, 2], -axis[:, 1], axis[:, 0]
    half = (np.eye(3) + np.sin(h)[:, None, None] * K
            + (1 - np.cos(h))[:, None, None] * (K @ K))
    return R[:-1] @ half


def body_steps(P, R):
    """各歩の移動を中間の車体系で表した d_k (N−1, 3)。"""
    Rm = mid_rotations(R)
    return np.einsum('nji,nj->ni', Rm, np.diff(P, axis=0))


def window_azimuth(D, i, j, v_sign=1.0):
    """窓 [i, j] の d_k の和の、車体系での方位角 a と仰角 [rad]。"""
    s = D[i:j].sum(axis=0) * v_sign
    return math.atan2(s[1], s[0]), math.atan2(s[2], math.hypot(s[0], s[1]))


def estimate(t, P, R, odom_t, odom_v, odom_w, used, evaluated,
             max_rate=MAX_RATE, min_speed=MIN_SPEED, window_m=WINDOW_M,
             max_turn_deg=MAX_TURN_DEG):
    """窓ごとの yaw の推定 [deg] と、窓の情報を返す。"""
    Pb, Rb = rebase(P, R, used, evaluated)
    psi_eval = math.degrees(math.atan2(evaluated[1][1, 0], evaluated[1][0, 0]))
    good, v = straight_mask(t, odom_t, odom_v, odom_w, max_rate, min_speed)
    hd = heading(Rb)
    D = body_steps(Pb, Rb)
    rows = []
    for i, j in windows(t, v, good, window_m):
        turn = math.degrees(math.atan2(math.sin(hd[j] - hd[i]), math.cos(hd[j] - hd[i])))
        if abs(turn) > max_turn_deg:
            continue
        sgn = float(np.sign(np.median(v[i:j + 1])))
        a, elev = window_azimuth(D, i, j, sgn)
        rows.append({
            't0': float(t[i]), 't1': float(t[j]), 'n': int(j - i + 1),
            'length_m': float(np.linalg.norm(Pb[j] - Pb[i])),
            'speed': float(np.median(v[i:j + 1])),
            'mean_rate': float(np.interp(0.5 * (t[i] + t[j]), odom_t, odom_w)),
            'turn_deg': turn,
            'x_mid': float(Pb[(i + j) // 2, 0]), 'y_mid': float(Pb[(i + j) // 2, 1]),
            'heading_deg': float(math.degrees(hd[(i + j) // 2])),
            'azimuth_deg': math.degrees(a),
            'elevation_deg': math.degrees(elev),
            'yaw_deg': psi_eval - math.degrees(a),
        })
    return rows


# 止まって・まっすぐ進んで・止まる（stop-and-go）の解析。止まっている間は NDT が
# odom の予測に引っぱられず、走行中の点群のゆがみ（10 Hz の 1 回転の間に 4.5 cm 進む）
# も無いので、走行中の窓より偏りの要因が少ない。屋外 2 回目（#156）の手順用。
STOP_SPEED = 0.02       # これ未満を停止とみなす [m/s]
STOP_MIN_S = 3.0        # 停止として使う最短の長さ [s]
STOP_MIN_MOVE_M = 3.0  # 停止の間の移動の下限。位置 1 cm の誤差で 0.2° 以下にする


def stops(t, v, stop_speed=STOP_SPEED, min_s=STOP_MIN_S):
    """|v| < stop_speed が min_s 以上続く区間 [i, j] の列。始めと終わり 0.5 s は捨てる。"""
    still = np.abs(v) < stop_speed
    out, i, n = [], 0, len(t)
    while i < n:
        if not still[i]:
            i += 1
            continue
        k = i
        while k + 1 < n and still[k + 1]:
            k += 1
        lo = np.searchsorted(t, t[i] + 0.5)
        hi = np.searchsorted(t, t[k] - 0.5) - 1
        if t[k] - t[i] >= min_s and hi > lo:
            out.append((int(lo), int(hi)))
        i = k + 1
    return out


def mean_pose(P, R):
    """位置の平均と、回転の平均（和を SVD で回転に戻す。静止中の小さなばらつき用）。"""
    U, _, Vt = np.linalg.svd(R.sum(axis=0))
    M = U @ Vt
    if np.linalg.det(M) < 0:
        U[:, -1] *= -1
        M = U @ Vt
    return P.mean(axis=0), M


def estimate_stops(t, P, R, odom_t, odom_v, used, evaluated, min_move=STOP_MIN_MOVE_M,
                   max_turn_deg=MAX_TURN_DEG):
    """隣り合う停止の組ごとの yaw の推定 [deg]。

    2 つの停止の平均の姿勢の差 Δp を、2 つの回転の中間の車体系で表し、その方位角を a と
    する。間の走行が弧でも、弦は中間の向きを向くので偏らない（蛇行は窓と同じく、正味
    の向きの変化 --max-turn-deg で抑える）。後退で戻った組は向きを反転して数える。
    """
    Pb, Rb = rebase(P, R, used, evaluated)
    psi_eval = math.degrees(math.atan2(evaluated[1][1, 0], evaluated[1][0, 0]))
    v = np.interp(t, odom_t, odom_v)
    st = [mean_pose(Pb[i:j + 1], Rb[i:j + 1]) + (i, j) for i, j in stops(t, v)]
    rows = []
    for (p0, R0, i0, j0), (p1, R1, i1, j1) in zip(st[:-1], st[1:]):
        d = p1 - p0
        if np.linalg.norm(d) < min_move:
            continue
        Rm = mid_rotations(np.stack([R0, R1]))[0]
        turn = math.degrees(heading(R1) - heading(R0))
        turn = (turn + 180) % 360 - 180
        # 正味の変化だけでなく、途中の向きも出発の向きから外れていないこと（曲がって
        # 戻ってきた経路は、弦が車体の向きを表さない）。
        hd = np.degrees(heading(Rb[j0:i1 + 1]) - heading(R0))
        hd = (hd + 180) % 360 - 180
        if abs(turn) > max_turn_deg or np.abs(hd).max() > max_turn_deg:
            continue
        sgn = float(np.sign(np.median(v[j0:i1 + 1])))
        db = Rm.T @ d * sgn
        a = math.atan2(db[1], db[0])
        rows.append({
            't0': float(t[j0]), 't1': float(t[i1]), 'n': int(j1 - i1 + j0 - i0 + 2),
            'length_m': float(np.linalg.norm(d)), 'speed': float(np.median(v[j0:i1 + 1])),
            'turn_deg': turn, 'x_mid': float((p0[0] + p1[0]) / 2), 'y_mid': float((p0[1] + p1[1]) / 2),
            'heading_deg': float(math.degrees(heading(Rm))),
            'azimuth_deg': math.degrees(a),
            'elevation_deg': math.degrees(math.atan2(db[2], math.hypot(db[0], db[1]))),
            'yaw_deg': psi_eval - math.degrees(a),
        })
    return rows


def summarize(rows, n_boot=2000, seed=0):
    """中央値・IQR・平均と、中央値のブートストラップ 95% 区間。"""
    y = np.array([r['yaw_deg'] for r in rows])
    if len(y) == 0:
        return {'windows': 0}
    rng = np.random.default_rng(seed)
    boot = np.median(rng.choice(y, size=(n_boot, len(y)), replace=True), axis=1)
    q1, q3 = np.percentile(y, [25, 75])
    el = np.array([r['elevation_deg'] for r in rows])
    return {
        'windows': int(len(y)),
        'total_length_m': round(float(sum(r['length_m'] for r in rows)), 1),
        'yaw_median_deg': round(float(np.median(y)), 3),
        'yaw_iqr_deg': [round(float(q1), 3), round(float(q3), 3)],
        'yaw_mean_deg': round(float(y.mean()), 3),
        'yaw_std_deg': round(float(y.std(ddof=1)), 3) if len(y) > 1 else None,
        'yaw_median_ci95_deg': [round(float(np.percentile(boot, 2.5)), 3),
                                round(float(np.percentile(boot, 97.5)), 3)],
        # 進んだ向きの車体系での仰角。pitch の TF のずれ・坂の変化・車体の沈みが入る。
        # yaw の判定には使わないが、桁の確認に出す。
        'elevation_median_deg': round(float(np.median(el)), 3),
    }


# ---- ここから下は ROS を使う（bag の読み出し） ----

def read_bag(bag, pose_topic='/pcl_pose', rate_topic='/odometry/filtered'):
    """bag から姿勢・odom・base_link→velodyne の /tf_static を読む。

    姿勢と odom の時刻は header の stamp（再生でも同じ時刻になる）。
    """
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message

    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(bag), storage_id=''),
                rosbag2_py.ConverterOptions('cdr', 'cdr'))
    types = {m.name: m.type for m in reader.get_all_topics_and_types()}
    topics = [x for x in (pose_topic, rate_topic, '/tf_static') if x and x in types]
    reader.set_filter(rosbag2_py.StorageFilter(topics=topics))
    cls = {x: get_message(types[x]) for x in topics}
    poses, odom, static = [], [], {}
    while reader.has_next():
        name, data, _ = reader.read_next()
        msg = deserialize_message(data, cls[name])
        if name == '/tf_static':
            for tf in msg.transforms:
                static[(tf.header.frame_id, tf.child_frame_id)] = tf.transform
            continue
        st = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if name == pose_topic:
            p, q = msg.pose.pose.position, msg.pose.pose.orientation
            poses.append((st, p.x, p.y, p.z, q.x, q.y, q.z, q.w))
        else:
            odom.append((st, msg.twist.twist.linear.x, msg.twist.twist.angular.z))
    return np.array(poses), np.array(odom), static


def chain_base_velodyne(static):
    """/tf_static の辺をたどって T_base_link_velodyne を作る（親→子の向き）。"""
    children = {}
    for (parent, child), tf in static.items():
        children.setdefault(parent, []).append((child, tf))

    def walk(frame, t, R):
        if frame == 'velodyne':
            return t, R
        for child, tf in children.get(frame, []):
            q = tf.rotation
            Rc = quat_to_R([[q.x, q.y, q.z, q.w]])[0]
            tc = np.array([tf.translation.x, tf.translation.y, tf.translation.z])
            got = walk(child, t + R @ tc, R @ Rc)
            if got is not None:
                return got
        return None

    return walk('base_link', np.zeros(3), np.eye(3))


def describe_tf(tR):
    t, R = tR
    pitch = -math.asin(max(-1.0, min(1.0, R[2, 0])))
    roll = math.atan2(R[2, 1], R[2, 2])
    yaw = math.atan2(R[1, 0], R[0, 0])
    return [round(float(v), 4) for v in t] + [round(math.degrees(v), 4) for v in (roll, pitch, yaw)]


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('bag', help='odom（と、--poses が無ければ /pcl_pose・/tf_static）を読む bag')
    ap.add_argument('--poses', help='ndt_replay_compare.py の poses_full.txt '
                                    '(t x y z qx qy qz qw)。省略時は bag の /pcl_pose')
    ap.add_argument('--used-tf', help='"x y z roll pitch yaw" [m, deg]：姿勢を出したときの '
                                      'base_link→velodyne。省略時は bag の /tf_static')
    ap.add_argument('--eval-tf', default=' '.join(map(str, LEDGER_TF)),
                    help='評価に使う base_link→velodyne（既定は台帳 §2.1。yaw は基準）')
    ap.add_argument('--rate-topic', default='/odometry/filtered',
                    help='曲がり・速さの判定に使う Odometry（/whill/odom なら車輪だけ）')
    ap.add_argument('--max-rate', type=float, default=MAX_RATE, help='|ω| の上限 [rad/s]')
    ap.add_argument('--min-speed', type=float, default=MIN_SPEED, help='|v| の下限 [m/s]')
    ap.add_argument('--window-m', type=float, default=WINDOW_M, help='窓の道のり [m]')
    ap.add_argument('--max-turn-deg', type=float, default=MAX_TURN_DEG,
                    help='窓の中の NDT の向きの変化の上限 [deg]')
    ap.add_argument('--stops', action='store_true',
                    help='走行中の窓ではなく、隣り合う停止（3 秒以上）の組で測る（stop-and-go の手順）')
    ap.add_argument('--min-move', type=float, default=STOP_MIN_MOVE_M,
                    help='--stops で使う停止の間の移動の下限 [m]')
    ap.add_argument('--start', type=float, default=0.0, help='使う区間の始め（bag 内の秒）')
    ap.add_argument('--end', type=float, default=float('inf'), help='使う区間の終わり')
    ap.add_argument('--json', help='窓ごとの結果を書く JSON')
    a = ap.parse_args()

    poses, odom, static = read_bag(a.bag, '' if a.poses else '/pcl_pose', a.rate_topic)
    if a.poses:
        poses = np.loadtxt(a.poses, ndmin=2)
    if len(poses) < 10 or len(odom) < 10:
        sys.exit(f'姿勢 {len(poses)} 個・odom {len(odom)} 個では足りない')
    if a.used_tf:
        used = tf_from_spec(a.used_tf.split())
        used_src = 'argument'
    else:
        used = chain_base_velodyne(static)
        if used is None:
            sys.exit('/tf_static に base_link→velodyne の経路が無い。--used-tf を渡す')
        used_src = 'bag /tf_static'
    evaluated = tf_from_spec(a.eval_tf.split())

    t0 = min(poses[0, 0], odom[0, 0])
    keep = (poses[:, 0] - t0 >= a.start) & (poses[:, 0] - t0 <= a.end)
    poses = poses[keep]
    o = odom[np.argsort(odom[:, 0])]
    if a.stops:
        rows = estimate_stops(poses[:, 0], poses[:, 1:4], quat_to_R(poses[:, 4:8]),
                              o[:, 0], o[:, 1], used, evaluated, a.min_move, a.max_turn_deg)
    else:
        rows = estimate(poses[:, 0], poses[:, 1:4], quat_to_R(poses[:, 4:8]),
                        o[:, 0], o[:, 1], o[:, 2], used, evaluated,
                        a.max_rate, a.min_speed, a.window_m, a.max_turn_deg)
    res = {
        'bag': str(a.bag), 'poses_source': a.poses or '/pcl_pose',
        'used_tf': describe_tf(used), 'used_tf_source': used_src,
        'eval_tf': describe_tf(evaluated), 'rate_topic': a.rate_topic,
        'max_rate': a.max_rate, 'min_speed': a.min_speed, 'window_m': a.window_m,
        'max_turn_deg': a.max_turn_deg, 'mode': 'stops' if a.stops else 'windows',
        'poses': int(len(poses)),
        **summarize(rows),
    }
    print(json.dumps(res, indent=2, ensure_ascii=False))
    if a.json:
        with open(a.json, 'w') as f:
            json.dump({**res, 'per_window': rows}, f, indent=1, ensure_ascii=False)
    if not rows:
        sys.exit(1)


if __name__ == '__main__':
    main()
