# M4-R 前置: ModelCr2State の単位曖昧性を実機で確定する

Language: [日本語](m4r-whill-units.md) | [English](../en/m4r-whill-units.md)

## 目的

M4-R (odom 基盤再構築) では、`/odometry/filtered` を車輪オドメトリと IMU から
EKF で組み立てる構成に切り替える。その入力となる `/whill/odom`
(`nav_msgs/Odometry`) を、まず上流 `ros2_whill` の `whill_driver` 内側で
publish する案を採る (案 1: Iruazu/ros2_whill fork に publisher を追加)。
案 1 が成立するには、`whill_msgs/ModelCr2State` の数値フィールドを SI 単位
(m, m/s, rad, rad/s) に換算して差動駆動 odometry を組み立てる必要があるが、
msg 定義 (`src/third_party/ros2_whill_interfaces/whill_msgs/msg/ModelCr2State.msg`)
には単位の注釈がない。

```
int32 battery_power
float32 battery_current
float32 right_motor_angle   # 単位: 不明 (deg / rad / encoder count のどれか)
float32 left_motor_angle    # 同上
float32 right_motor_speed   # 単位: 不明 (m/s / km/h / rpm のどれか)
float32 left_motor_speed    # 同上
int32 power_on
int32 speed_mode_indicator
int32 error
```

この前提を踏まないまま fork に publisher を埋め込むと、係数の符号や桁が
1 桁ずれた状態で `/whill/odom` が出続け、後段 EKF と Nav2 の挙動デバッグが
原因切り分け不能になる。M4-R に入る前に実機で単位を確定し、本文書の結果記入欄に
固定値として残すのが本 Issue の目的である。

本 Issue で確定する対象は次の 4 つ。

- `right_motor_angle`、`left_motor_angle` の単位
- `right_motor_speed`、`left_motor_speed` の単位
- 後輪 (駆動輪) のタイヤ直径 — 走行距離換算と odometry 積分に必要
- tread 幅 (後輪中心間距離) — 差動駆動の角速度計算に必要

確定値は M4R-1 (fork パッチ) の `whill_node.cpp::OnStatesModelCr2Timer()`
内に、本文書末尾の C++ コード例の `WHEEL_RADIUS` / `TREAD` /
`ANGLE_TO_RAD` / `SPEED_TO_MPS` という形でハードコードもしくは launch
パラメータ化して埋め込む。本文書は手順書であり、パッチ実装は別 Issue
(M4R-1) で行う。

## 前提環境

- WHILL Model CR2 実機 (本リポの M2 でドライバ動作確認済の個体)
- `whill_driver` ノードが起動可能で、`/whill/states/model_cr2`
  (`whill_msgs/ModelCr2State`) を publish している
  - 起動コマンド: `ros2 launch whill_bringup whill_launch.py`
    (`src/third_party/ros2_whill/whill_bringup/launch/whill_launch.py` を実行)
- ジョイスティック (`/whill/controller/joy`) または速度指令
  (`/whill/controller/cmd_vel`) で椅子を駆動できる状態
- 物理計測用のメジャー (5 m 以上)、マーキング用の養生テープまたはチョーク
- 1 m 以上の直線が取れる平坦な床、両輪をジャッキアップできるスペース
  (台木、レンガ、または安定したジャッキ)
- 計測者 2 名を推奨 (椅子の操作と stopwatch / ログ取得の同時並行)

## 手順 1: 1 回転試験 — motor_angle の単位確定

目的: `right_motor_angle`、`left_motor_angle` の単位が deg / rad /
encoder count のどれであるかを判定する。

ジャッキアップして両輪を浮かせるのは、椅子が前進してしまうと床上で 1 回転を
正確に取れないため。床面で行うと駆動輪のわずかなスリップが角度差分に乗り、
判定式の境界が曖昧になる。

手順:

1. 椅子をジャッキアップし、左右の駆動輪 (後輪) が床から完全に浮いた状態にする。
   フレームの揺れがないことを目視で確認する。
2. 右側駆動輪のタイヤ側面に養生テープで基準マークを 1 本貼る (12 時方向)。
   1 回転を物理的に判定する目印になる。左輪も同様にする。
3. 端末 1 で `whill_driver` を起動する:

   ```bash
   ros2 launch whill_bringup whill_launch.py
   ```

4. 端末 2 でログを取り始める:

   ```bash
   ros2 topic echo /whill/states/model_cr2 | tee /tmp/cr2_rot.log
   ```

5. 端末 3 (またはジョイスティック) で右輪のみ低速で駆動する。両輪を独立に
   動かすコマンドは ros2_whill にはないので、`/whill/controller/cmd_vel` で
   `angular.z` だけを与え、椅子が浮いた状態で旋回させる (片輪が正、もう片輪が
   負に回る)。回転速度は基準マークを目視追従できる程度 (角速度 0.5 rad/s 目安)。
6. 基準マークが 12 時 → 12 時に戻った瞬間 (1 回転完了) で停止する。
7. ログから 1 回転の開始直前の `right_motor_angle` (A_start) と、停止直後の
   `right_motor_angle` (A_end) を読み取る。差分を取る:

   ```
   ΔA = A_end - A_start
   ```

8. 左輪についても同様に計測する (`left_motor_angle` の ΔA)。両輪は上流の
   ros2_whill が対称的に扱っているはずなので、同じ単位スケールが出ることを
   期待する。違う値が出たら本手順を再確認する (基準マークの読み違い、旋回中の
   停止タイミングずれ等)。

判定式:

| ΔA の値 | 判定される単位 | 備考 |
|---------|---------------|------|
| `ΔA ≈ 360` | **deg** | rad→deg 変換は `× π/180` |
| `ΔA ≈ 6.283` (= 2π) | **rad** | そのまま使える |
| `ΔA` が大きい整数値 (例: 4096、500、1000、…) | **encoder count** | 1 回転あたりのパルス数 N を確定値として記録 |

encoder count の場合、N の値が分かるとそのまま rad 換算が `× 2π / N` で
書ける。N が「きりの良い数」(2^n や 100 の倍数) でない場合は上流のギア比が
混入している可能性があり、その場合は本文書の結果欄に「ギア後段か、ホイール
直結か」を備考として残す。

参考: 上流コードの読み方。本 Issue では実機実測が一次情報だが、上流の
`whill_node.cpp::OnStatesModelCr2Timer()` から `ReceiveDataset1()` を辿ると、
バイト列のどこから `motor_angle` を取り出し、どんなスケール係数をかけている
かが読める。実測値と矛盾した場合の crosscheck に使う。

## 手順 2: 1 m 直進試験 — motor_speed の単位確定と motor_angle との整合

目的: `right/left_motor_speed` の単位 (m/s / km/h / rpm) を確定し、同時に
手順 1 で確定した `motor_angle` の単位が距離換算で破綻しないことを
1 m 直進で交差検証する。

手順:

1. 椅子をジャッキから下ろし、床面に置く。タイヤの空気圧 (もしくは硬度) は
   実走行と同じ状態にする (ジャッキアップ時の空転と実効直径が違うため)。
2. 床にスタートとゴールを養生テープで明示する。間隔はメジャーで正確に 1.000 m。
3. 端末 1 でログを取る:

   ```bash
   ros2 topic echo /whill/states/model_cr2 | tee /tmp/cr2_drive.log
   ```

4. 別の計測者がストップウォッチを構え、椅子をスタート位置に置く。ジョイスティック
   または `/whill/controller/cmd_vel` で速度指令を与え、できる限り一定速度で
   ゴールまで直進する。スタート位置を後輪が通過した瞬間から、ゴールを後輪が
   通過する瞬間までの時間 T [s] を読む。
5. ログから、直進中 (加速・減速区間を除いた中間帯) の `right_motor_speed` の
   平均値 V を計算する。awk または手計算で構わない。

速度の理論値: V_theory = 1 [m] / T [s] = 1/T [m/s]

判定式:

| V の値 | 判定される単位 | 備考 |
|--------|---------------|------|
| `V ≈ 1/T` | **m/s** | 換算係数 1.0 |
| `V ≈ 3.6/T` | **km/h** | m/s 換算は `÷ 3.6` |
| `V が大きい` (タイヤ円周から逆算した値) | **rpm** | m/s 換算は `× WHEEL_RADIUS × 2π / 60` |

距離換算の交差検証:

1. 同じ走行区間の `right_motor_angle` の開始値と終了値を取り、差分 ΔA
   (手順 1 と同じ単位) を求める。
2. ΔA を rad に正規化する:

   - deg の場合: `ΔA_rad = ΔA × π / 180`
   - rad の場合: `ΔA_rad = ΔA`
   - encoder count (N pulses/rev) の場合: `ΔA_rad = ΔA × 2π / N`

3. 走行距離を計算: `distance_calc = WHEEL_RADIUS × ΔA_rad`
   (WHEEL_RADIUS = タイヤ半径。手順 2 の実施時点では手順 3 が未完了の
   ため、暫定値として URDF 公称値 `0.1325 m` を使う。手順 3 で実測値を
   確定したら、結果欄の `1 m 直進試験の誤差` は確定値で再計算して書く)
4. 1.000 m との誤差が 5 % 以内で合格。これを超える場合は次のいずれかが
   疑われる:

   - 手順 1 の単位判定が間違っている (deg/rad/encoder の取り違え)
   - WHEEL_RADIUS が公称値どおりではない (空気圧や摩耗で実効直径が違う)
   - 駆動輪の路面スリップが想定より大きい (走行速度が高すぎたケース)

   原因切り分けのため、再試験するときは走行速度を半減して再計測する。

## 手順 3: タイヤ直径と tread 幅の確認

目的: 差動駆動 odometry の計算 (`v_right`、`v_left` から並進・角速度) に必要な
車両寸法を、URDF の公称値と実機実測の両方で取り、ズレを記録する。

公称値の所在:

- `src/third_party/ros2_whill/whill_description/urdf/whill_model_cr2.urdf` の
  `left_wheel` / `right_wheel` リンク (後輪駆動輪) は
  `radius="0.1325"` (= 0.265 m 直径) で書かれている。
- tread 幅の値は URDF の joint origin から計算可能。
  `whill_node.cpp::OnControllerCmdVel()` のコメント (`wheel_tread: 0.496`)
  が一次情報源として最も明示的で、ここから 0.496 m と読み取れる。

実測値:

- タイヤ直径: 後輪をメジャーで床接地状態のまま外径計測。床に荷重がかかった
  実効直径を測るため、椅子を空車のまま 1 名分の荷重で床に置いた状態で
  メジャーを当てる (理由: 走行中の WHEEL_RADIUS は荷重と空気圧の影響を受ける
  ため、無荷重時の値で計算すると 1 m 試験の誤差として表面化する)。
- tread 幅: 後輪のタイヤ中心線同士の距離をメジャーで測る。左右タイヤの内側
  距離と外側距離を測って ((内側 + 外側) / 2) で中心間距離を求めるのが正確。

採用値の決め方:

- 通常は実測を優先する (本機固有の摩耗・空気圧を反映するため)
- URDF と整合させる必要がある場合 (例: rviz と odometry の整合) は公称値を
  使い、URDF と実機の差は別途記録に残す
- 実測と公称が 2% 以上ズレる場合は、その差を結果欄の備考に必ず書く

## 検証結果 (実機検証完了 2026-06-14)

- 計測日: 2026-06-14
- 計測者: Iruazu
- whill_driver の git SHA: `ceebd45` (Iruazu/ros2_whill humble fork)
- 検証ログ: `/tmp/cr2_full.log` (約 2 m 前進、1 回試行、メジャー計測なしの粗い裏取り)

#### 単位 (上流ソース宣言 + 実機裏取り)

| フィールド | 単位 | 1 m 走行での観測値 | 備考 |
|-----------|------|-------------------|------|
| `right_motor_angle` | rad | 累積 -27.6 rad (wrap 補正後、約 2 m 走行) = -4.39 回転 | ±π で wrap、前進時は減少 |
| `left_motor_angle`  | rad | 同上 (走行は前進のみ) | 前進時は増加 (右と符号反対) |
| `right_motor_speed` | km/h | ピーク 1.036 km/h ≈ 0.288 m/s | 前進時は負 |
| `left_motor_speed`  | km/h | ピーク 1.052 km/h ≈ 0.292 m/s | 前進時は正 |

ソース根拠: `src/third_party/ros2_whill/whill_driver/src/model_cr2/whill.cpp:62-69`
- L63: `// The value for converting [0.001rad] to [rad]` (`kMotorAngleFactor = 0.001`)
- L68: `// The value for converting [0.004km/h] to [km/h]` (`kMotorSpeedFactor = 0.004`)

#### 寸法

| 項目 | 公称値 (出典) | 実測値 | 採用値 |
|------|--------|--------|--------|
| タイヤ直径 | 0.265 m (URDF `whill_modelc.urdf`) | (未実測) | 0.265 m |
| タイヤ半径 (= `WHEEL_RADIUS`) | 0.1325 m | (未実測) | 0.1325 m |
| tread 幅 (= `TREAD`) | 0.496 m (`whill_node.cpp:115` コメント) | (未実測) | 0.496 m |

#### 1 m 直進試験の誤差

- 実距離検証はメジャー計測なしの粗い裏取りのため、数値合格判定は **M4R-3 (EKF 導入後) に持ち越し**
- 単位確定の目的 (motor_angle = rad、motor_speed = km/h) は達成
- ピーク motor_speed = 1.036 km/h = 0.288 m/s はジョイスティック前進の値域として妥当 (ソース宣言と整合)

#### M4R-1 で必要な追加注意事項 (実機検証で判明)

1. **motor_angle の ±π wrap**: 約 2 m 走行で 3 回観測。角度ベース odometry を採用する場合、ROS 2 標準 `angles::shortest_angular_distance()` で wrap-aware に処理する (詳細は次節)
2. **左右符号反転**: 前進時に `right_motor_speed` は負、`left_motor_speed` は正、また `right_motor_angle` は減少、`left_motor_angle` は増加。`angles::shortest_angular_distance(prev, curr) = (curr - prev)` の符号慣習下では、**右輪の差分を反転** して「前進が正」に統一する (前進時 d_right_raw < 0、d_left_raw > 0 → 右輪反転で両方 +)
3. **WHILL 公式に odometry 実装はなし**: `whill-labs/ros2_whill`、`whill-labs/ros2_whill_applications`、`whill-labs/whill_visualization` のいずれにも odometry 関連実装はゼロ。M4R-1 で自前実装が必須

## M4R-1 への転記

確定した数値は、M4R-1 (案 1) で fork する `ros2_whill` の
`whill_driver/src/whill_node.cpp` 内で、`OnStatesModelCr2Timer()` を拡張して
`/whill/odom` を publish する処理に埋め込む。コード例を以下に示す
(実際の実装は M4R-1 で行うため、ここでは差動駆動 odometry の輪郭のみ示す):

```cpp
// whill_node.cpp に追加するコード (M4R-1 で fork パッチ実装時)
//
// 設計判断 (2026-06-14 確定):
// - wrap 処理: ROS 2 標準 angles パッケージ (ros-humble-angles)
//   - 自前 WrappedAngleDiff() の再発明を避け、保守は ROS コミュニティ任せ
// - odometry 方式: 角度ベース (motor_angle 差分から速度逆算)
//   - publish 頻度 ~3 Hz の低頻度に対して頑健 (速度ベースは量子化に弱い)
//   - 採用しなかった選択肢: 速度ベース odometry
//     (motor_speed 0.004 km/h 量子化を直接受ける)
// - 左右符号: 右輪を反転して「前進が正」に統一
//   - 実機ログ (2026-06-14) で前進時に right_motor_angle が減少、
//     left_motor_angle が増加することを確認。
//     angles::shortest_angular_distance(prev, curr) は (curr - prev) を
//     返すため、前進で d_right_raw < 0、d_left_raw > 0 となる。
//     右輪の符号を反転すれば両輪とも前進で正となり、
//     v_angular = (v_right - v_left) / TREAD が ROS REP-103 慣習
//     (左旋回 = 正) と一致する。

#include <angles/angles.h>
#include <cmath>

// 単位変換係数 (whill.cpp:62-69 ソースコメント + 2026-06-14 実機裏取り)
constexpr double WHEEL_RADIUS = 0.1325;  // [m] URDF whill_modelc.urdf 公称
constexpr double TREAD        = 0.496;   // [m] whill_node.cpp:115 コメント

// 前回サンプル保持 (角度ベース odometry の差分計算用)
whill_msgs::msg::ModelCr2State::SharedPtr prev_state_;
rclcpp::Time prev_stamp_;

// 累積位置 (map 原点からの相対、初期化は launch 時の reset で 0)
double x_ = 0.0, y_ = 0.0, yaw_ = 0.0;

void WhillNode::OnStatesModelCr2Timer()
{
  auto msg = std::make_shared<whill_msgs::msg::ModelCr2State>();
  if (whill_->ReceiveDataset1(msg) < 1) {return;}

  const auto now = this->now();

  // 初回サンプルは prev に保存して return (差分計算できないため)
  if (!prev_state_) {
    prev_state_ = msg;
    prev_stamp_ = now;
    states_model_cr2_pub_->publish(*msg);
    return;
  }

  const double dt = (now - prev_stamp_).seconds();
  // dt 異常値ガード (publish 抜け / clock 巻き戻し)
  if (dt <= 0.0 || dt > 1.0) {
    prev_state_ = msg;
    prev_stamp_ = now;
    states_model_cr2_pub_->publish(*msg);
    return;
  }

  // wrap-aware な角度差分。右輪を反転して「前進が正」に揃える
  // (実機ログ 2026-06-14: 前進時に right_motor_angle 減少、left 増加)。
  const double d_right = -angles::shortest_angular_distance(
      prev_state_->right_motor_angle, msg->right_motor_angle);
  const double d_left  =  angles::shortest_angular_distance(
      prev_state_->left_motor_angle,  msg->left_motor_angle);

  // 角速度 (rad/s) → タイヤ接地点速度 (m/s)
  const double v_right_mps = (d_right / dt) * WHEEL_RADIUS;
  const double v_left_mps  = (d_left  / dt) * WHEEL_RADIUS;

  // 差動駆動 odometry。
  // v_angular = (右 - 左) / tread とすることで、ROS REP-103 慣習
  // (左旋回 = 正の angular.z) と一致する: 左旋回時は右輪が左輪より速く回り
  // v_right > v_left → 正の ω となる。
  const double v_linear  = 0.5 * (v_right_mps + v_left_mps);
  const double v_angular = (v_right_mps - v_left_mps) / TREAD;

  // 姿勢積分 (中点法、yaw 単独積分)
  yaw_ += v_angular * dt;
  // yaw を [-π, π] に正規化 (累積誤差防止)
  yaw_ = angles::normalize_angle(yaw_);
  x_   += v_linear * std::cos(yaw_) * dt;
  y_   += v_linear * std::sin(yaw_) * dt;

  // nav_msgs/Odometry の組み立てと publish
  // (frame_id: "odom"、child_frame_id: "base_link"、quaternion は yaw_ から構築)
  // 詳細は M4R-1 実装時に。

  states_model_cr2_pub_->publish(*msg);
  prev_state_ = msg;
  prev_stamp_ = now;
}
```

### M4R-1 fork パッチに含める依存追加

`Iruazu/ros2_whill` の fork パッチには以下も含める:

1. `whill_driver/package.xml` に依存追加:
   ```xml
   <depend>angles</depend>
   <depend>nav_msgs</depend>
   <depend>tf2</depend>
   <depend>tf2_geometry_msgs</depend>
   ```

2. `whill_driver/CMakeLists.txt` に追加:
   ```cmake
   find_package(angles REQUIRED)
   find_package(nav_msgs REQUIRED)
   find_package(tf2 REQUIRED)
   find_package(tf2_geometry_msgs REQUIRED)

   target_link_libraries(whill_node
     # ... 既存 ...
     angles::angles
   )
   ament_target_dependencies(whill_node
     # ... 既存 ...
     nav_msgs tf2 tf2_geometry_msgs
   )
   ```

3. `/whill/odom` publisher の追加 (`whill_node.hpp` と `whill_node.cpp::Initialize()`):
   ```cpp
   odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/whill/odom", 10);
   ```

## 関連

- M4R-1 (案 1): Iruazu/ros2_whill fork に `/whill/odom` publisher を追加。
  本文書の確定値を当該 PR の C++ パッチに転記する。
- 開発方針: [`plans/2026-06-11-platform-pivot.md`](plans/2026-06-11-platform-pivot.md)
  §4 (M4-R の位置づけ)、§6 (M4-R 受け入れ基準: `/odometry/filtered` が
  車輪 + IMU 由来で publish され、手押し 10 m 直進で終端誤差が許容内)
- 上流コード: `src/third_party/ros2_whill/whill_driver/src/whill_node.cpp`
  (`OnStatesModelCr2Timer()` と `ReceiveDataset1()` が motor_angle /
  motor_speed のバイト列パース実装。本文書手順での crosscheck 用)
- 上流寸法情報: `src/third_party/ros2_whill/whill_description/urdf/whill_model_cr2.urdf`
  (後輪 `radius="0.1325"`)、`whill_node.cpp::OnControllerCmdVel()` の
  `wheel_tread: 0.496` コメント
- ADR 0001: [`decisions/0001-docs-i18n.md`](decisions/0001-docs-i18n.md) —
  新規 docs は ja/en 並列で生やす
