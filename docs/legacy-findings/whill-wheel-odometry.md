# Legacy Investigation: whill 車輪オドメトリ計算ロジック

## 調査日
2026-05-28 (legacy-archaeologist)

## TL;DR

odometry は `odom.cpp` に分離。エンコーダ角度 (`Motor.angle`, 単位: rad) の差分から瞬時角速度を自前で計算し、差動2輪キネマティクスで位置を midpoint 積分。`wheel_radius = 0.1325 m`, `tread = 0.496 m` は `WHILL.h` にハードコード (外部パラメータ化なし)。共分散は **全要素 0**。ROS 2 wrapper では `ModelCr2State.right_motor_angle` / `left_motor_angle` を直接使い、dt と角度差分から同じ式を再現できる。`motor_speed` フィールドは旧実装でも使われておらず無視してよい。

---

## A. odometry publisher の場所

- **publish 箇所**: `ros_whill.cpp:366-370`
- **publish trigger**: WHILL からデータパケットを受信するたびに呼ばれる `whill_callback_data1()` 内 (イベント駆動、約 50 Hz)
- **topic 名**: `"odom"` (node namespace `whill` 下なので実効 `/whill/odom`、`ros_whill.cpp:416`)
- **frame_id / child_frame_id**: `"odom"` / `"base_link"` (`ros_whill.cpp:368-369`)
- **データ要求頻度**: `whill->begin(20)` (`ros_whill.cpp:509`) — 20 ms 間隔 (≈50 Hz)

---

## B. 計算式

### B-1. motor angle の単位

`PacketParser.cpp:72-73`:
```
right_motor.angle = (int16_t)(payload[18..19]) * 0.001  // 単位: rad
left_motor.angle  = (int16_t)(payload[20..21]) * 0.001  // 単位: rad
```
1 LSB = 1 mrad。範囲は ±32.767 rad で連続ロールオーバー。

### B-2. motor speed の単位 (不使用)

`PacketParser.cpp:75-76`:
```
right_motor.speed = (int16_t)(payload[22..23]) * 4   // WHILL 内部 unit
left_motor.speed  = (int16_t)(payload[24..25]) * 4
```
**`speed` フィールドは odometry 計算で使われていない**。angle 差分から自前で速度を計算している。

### B-3. JointState.velocity の生成 (ros_whill.cpp:325-343)

```
velocity[0] = rad_diff(past_left,  current_left)  / (interval_ms / 1000.0)   // 左 [rad/s]
velocity[1] = rad_diff(past_right, current_right) / (interval_ms / 1000.0)   // 右 [rad/s]
```

`rad_diff()` (`rotation_tools.cpp:30-41`) は ±π 境界ラップアラウンド補正付き差分 (`past - current`)。

### B-4. 差動2輪キネマティクス (odom.cpp:75-92)

```
angle_vel_r =  jointState.velocity[1]       // 右 [rad/s] (前進=正)
angle_vel_l = -jointState.velocity[0]       // 左 [rad/s] (符号反転)

vr = angle_vel_r * wheel_radius             // [m/s]
vl = angle_vel_l * wheel_radius

delta_L     = (vr + vl) / 2.0              // 前進速度 [m/s]
delta_theta = (vr - vl) / tread            // 角速度 [rad/s] (CCW 正)

// midpoint 法積分
pose.x     += delta_L * dt * cos(pose.theta + delta_theta * dt / 2.0)
pose.y     += delta_L * dt * sin(pose.theta + delta_theta * dt / 2.0)
pose.theta += delta_theta * dt              // ±π に正規化
```

### B-5. 車輪諸元 (WHILL.h:122-123)

```cpp
const float wheel_radius = 0.1325;  // [m]
const float tread        = 0.496;   // [m]
```
WHILL 公式 SDK 値、ハードコード。仕様書引用なし。

---

## C. 共分散

`getROSOdometry()` (`odom.cpp:121-142`) は `nav_msgs::Odometry` をデフォルト構築のみ。**`pose.covariance` も `twist.covariance` も全 0**。Nav2 `robot_localization` は covariance = 0 を invalid と扱う場合があるため、**ROS 2 側では対角成分を必ず設定**。

---

## D. TF broadcast

- `tf::TransformBroadcaster::sendTransform()` 使用 (`ros_whill.cpp:379-382`)
- `odom -> base_link` を毎パケット受信時に broadcast (≈50 Hz)
- `publish_tf: true` (launch デフォルト, `ros_whill.cpp:437`)
- **`z` 座標に `base_link_height = 0.1325` をハードコード** (`odom.cpp:44, 130, 153`) — 移植時は廃棄

---

## E. パラメータ外部化状況

| パラメータ | 外部化 | 場所 |
|-----------|--------|------|
| `wheel_radius = 0.1325` | なし | `WHILL.h:122` ハードコード |
| `tread = 0.496` | なし | `WHILL.h:123` ハードコード |
| `publish_tf` | あり | `ros_whill.launch:28` |
| `send_interval` | あり (デフォルト 10) | `ros_whill.cpp:423-429` |
| `serialport` | あり (env `TTY_WHILL`) | `ros_whill.launch:11` |
| `base_link_height = 0.1325` | なし | `odom.cpp:44` ハードコード |

---

## F. 罠

### F-1. 左輪符号反転は必須

`odom.cpp:76` で `angle_vel_l = -jointState.velocity[0]`。`rad_diff` は `past - current` 順なので前進時左輪 diff が正になり、ROS 規約の前進右回り CCW 規約に合わせるため反転している。**忘れると旋回方向が逆**になる。

### F-2. 旋回符号

`delta_theta = (vr - vl) / tread`。右輪が速い (= 左旋回 = CCW) で正。ROS 標準 (CCW 正) と一致。

### F-3. motor.speed フィールドは無視

`ros_whill.cpp` 全体で `right_motor.speed` / `left_motor.speed` を一度も参照していない。`ModelCr2State` の `right_motor_speed` / `left_motor_speed` フィールドの単位不明問題は、**使わないので影響なし**。

### F-4. ラップアラウンド補正必須

`right_motor_angle` は ±32.767 rad で折り返す。単純な `current - past` では境界をまたぐと巨大な値になる。`rotation_tools.cpp:30-41` の `atan2(sin(d), cos(d))` 等価ロジックを ROS 2 側で再実装する必要あり。

---

## ROS 2 wrapper で踏襲すべき計算式

`whill_msgs::msg::ModelCr2State` のフィールド名で書き直した擬似コード:

```python
# 定数 (WHILL.h:122-123 から)
WHEEL_RADIUS = 0.1325   # [m]
TREAD        = 0.496    # [m]

# ModelCr2State コールバック
def on_state(msg, prev_state, dt):
    # エンコーダ角度差分 [rad] — ±π ラップアラウンド補正必須
    d_right =  angle_diff(prev_state.right_motor_angle, msg.right_motor_angle)  # past - current
    d_left  = -angle_diff(prev_state.left_motor_angle,  msg.left_motor_angle)   # 左輪符号反転

    # 瞬時線速度 [m/s]
    vr = (d_right / dt) * WHEEL_RADIUS
    vl = (d_left  / dt) * WHEEL_RADIUS

    # 差動2輪キネマティクス
    v_linear  = (vr + vl) / 2.0           # [m/s]
    v_angular = (vr - vl) / TREAD         # [rad/s], CCW 正

    # 位置積分 (midpoint 法)
    pose.x     += v_linear * dt * cos(pose.theta + v_angular * dt / 2.0)
    pose.y     += v_linear * dt * sin(pose.theta + v_angular * dt / 2.0)
    pose.theta  = wrap_to_pi(pose.theta + v_angular * dt)

    # Odometry メッセージ
    odom.header.stamp = msg.header.stamp     # ← dt 計算もこの stamp 差分を使う
    odom.header.frame_id = "odom"
    odom.child_frame_id  = "base_link"
    odom.twist.twist.linear.x  = v_linear
    odom.twist.twist.angular.z = v_angular
    odom.pose.pose.position.x  = pose.x
    odom.pose.pose.position.y  = pose.y
    # z = 0 (旧 base_link_height ハックは URDF 任せ)
    odom.pose.covariance[0]   = 1e-3   # x
    odom.pose.covariance[7]   = 1e-3   # y
    odom.pose.covariance[35]  = 1e-3   # yaw
    odom.twist.covariance[0]  = 1e-3   # vx
    odom.twist.covariance[35] = 1e-2   # vyaw
```

`angle_diff(past, current)` は Python なら `math.atan2(math.sin(past - current), math.cos(past - current))` が等価。

dt は `ModelCr2State.header.stamp` 差分を推奨 (旧は固定 `interval_ms/1000.0`)。

---

## 移植時の注意点

1. **左輪符号反転は必須** (`odom.cpp:76`)
2. **共分散を必ずセット**: 旧は全 0
3. **`position.z = 0`** にする (`base_link_height` ハックは URDF/robot_state_publisher に任せる)
4. **`motor_speed` フィールドは無視**
5. **ラップアラウンド補正必須** (`atan2(sin, cos)` 等価)
6. **dt は header.stamp 差分** で計算
7. **wheel_radius / tread を ROS 2 では param 化** すべき (旧はハードコード)

---

## 移植不要 / 廃棄

- `odom.pose.position.z = base_link_height` (`odom.cpp:130, 153`) — URDF へ移管
- 固定 `interval_ms` による dt 計算 (`ros_whill.cpp:353`) — `header.stamp` 差分に置換

---

## 開いている疑問

- `tread = 0.496 m` が Model CR2 仕様値か実測値か未確認 (WHILL 公式ドキュ照合推奨)
- `right_motor.speed` の `* 4` 単位 (mRPM? 内部 unit?) — 使わないので実用上問題なし

---

## 主要 file:line

| ファイル | 内容 | 行 |
|---------|------|----|
| `ros_whill/src/whill/WHILL.h` | `wheel_radius=0.1325`, `tread=0.496` 定義 | 122-123 |
| `ros_whill/src/odom.cpp` | 差動2輪キネマティクス (midpoint 積分) | 70-95 |
| `ros_whill/src/odom.cpp` | 左輪符号反転 | 76 |
| `ros_whill/src/odom.cpp` | `getROSOdometry()` (共分散 0) | 121-142 |
| `ros_whill/src/odom.cpp` | `base_link_height=0.1325` z セット | 44, 130, 153 |
| `ros_whill/src/ros_whill.cpp` | publish topic + frame 設定 | 366-370 |
| `ros_whill/src/ros_whill.cpp` | TF broadcast | 379-382 |
| `ros_whill/src/ros_whill.cpp` | odometry trigger | 283-386 |
| `ros_whill/src/ros_whill.cpp` | `odom.setParameters` 呼び出し | 482 |
| `ros_whill/src/ros_whill.cpp` | 角速度計算 | 329-342 |
| `ros_whill/src/ros_whill.cpp` | データ要求頻度 (50 Hz) | 509 |
| `ros_whill/src/whill/PacketParser.cpp` | angle = int16 * 0.001 rad | 72-76 |
| `ros_whill/src/utils/rotation_tools.cpp` | `rad_diff` (ラップ補正) | 30-41 |
| `ros_whill/launch/ros_whill.launch` | param 設定 | 11, 28 |
