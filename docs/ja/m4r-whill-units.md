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

## 検証結果 (実機検証後にユーザーが記入)

- 計測日:
- 計測者:
- whill_driver の git SHA (`git -C src/third_party/ros2_whill log -1 --format=%H`):

### 単位

| フィールド | 単位 (deg / rad / encoder / m/s / km/h / rpm) | 1 回転または 1 m に対応する値 | 備考 |
|-----------|---------|--------|------|
| `right_motor_angle` |         |        |      |
| `left_motor_angle`  |         |        |      |
| `right_motor_speed` |         |        |      |
| `left_motor_speed`  |         |        |      |

### 寸法

odometry 計算は半径 (`WHEEL_RADIUS`) を使うため、直径を測ったら必ず 2 で割って半径欄に書く。直径値をそのまま `WHEEL_RADIUS` に代入すると odometry が約 2 倍を報告する。

| 項目 | 公称値 (出典: URDF 等) | 実測値 | 採用値 (odometry 計算用) |
|------|---------|--------|---------------|
| タイヤ直径 |        |       | (採用値は半径で記入) |
| タイヤ半径 (= `WHEEL_RADIUS`) |        |       |                 |
| tread 幅 (= `TREAD`)  |        |       |                 |

### 1 m 直進試験の誤差

- 計算距離: `<計算式と値>`
- 実測 1 m との差: ` % `
- 合格判定 (< 5% で合格):

## M4R-1 への転記

確定した数値は、M4R-1 (案 1) で fork する `ros2_whill` の
`whill_driver/src/whill_node.cpp` 内で、`OnStatesModelCr2Timer()` を拡張して
`/whill/odom` を publish する処理に埋め込む。コード例を以下に示す
(実際の実装は M4R-1 で行うため、ここでは差動駆動 odometry の輪郭のみ示す):

```cpp
// whill_node.cpp::OnStatesModelCr2Timer() に追加する処理の骨子。
// ハードコード値の確定根拠は docs/ja/m4r-whill-units.md の結果記入欄を参照。
//
// 値そのものを launch パラメータ化するか、コンパイル時定数として固定するかは
// M4R-1 で判断する。「両輪のキャリブレーションを実機ごとに差し替える可能性が
// 残る」なら ROS パラメータ、「上流仕様として固定」と確信できるなら定数。

// M_PI を使うため <cmath> をインクルードする。M_PI は C++ 標準ではなく
// POSIX 拡張のため、未定義環境では `constexpr double M_PI = 3.14159265358979323846;`
// を別途定義するか、コンパイラに `-D_USE_MATH_DEFINES` を渡す。
#include <cmath>

// 車両寸法 (m4r-whill-units.md 手順 3 の採用値)
constexpr double WHEEL_RADIUS = ???;   // [m] 後輪有効半径 (荷重時)
constexpr double TREAD        = ???;   // [m] 後輪中心間距離

// motor_angle → rad の換算係数 (手順 1 で確定)
//   deg の場合:    M_PI / 180.0
//   rad の場合:    1.0
//   encoder の場合: 2.0 * M_PI / N  (N は 1 回転あたりのパルス数)
constexpr double ANGLE_TO_RAD = ???;

// motor_speed → m/s の換算係数 (手順 2 で確定)
//   m/s の場合:  1.0
//   km/h の場合: 1.0 / 3.6
//   rpm の場合:  WHEEL_RADIUS * 2.0 * M_PI / 60.0
constexpr double SPEED_TO_MPS = ???;

void WhillNode::OnStatesModelCr2Timer()
{
  auto msg = std::make_shared<whill_msgs::msg::ModelCr2State>();
  if (whill_->ReceiveDataset1(msg) < 1) {return;}
  states_model_cr2_pub_->publish(*msg);

  // 差動駆動 odometry の擬似コード (本文書はあくまで単位確定の手順書なので、
  // 実装の細部 — 時刻管理、初期化、共分散行列、frame_id — は M4R-1 で詰める)
  const double v_right = msg->right_motor_speed * SPEED_TO_MPS;
  const double v_left  = msg->left_motor_speed  * SPEED_TO_MPS;
  const double v_lin   = (v_right + v_left) / 2.0;        // [m/s] 並進速度
  const double w_ang   = (v_right - v_left) / TREAD;      // [rad/s] 角速度
  // dt を取って x, y, yaw を積分 → nav_msgs/Odometry を構築・publish
  // (詳細は M4R-1)
}
```

`???` を残したコードは M4R-1 のレビュー時点で確定値に置き換わっているべき。
レビュアーは本文書の「検証結果」表が埋まっていることを fork パッチの
受け入れ条件として確認する。

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
