# 古典的な障害物回避（比較基準）実装計画

2026-10-28 作成。依頼者承認済み（段階B）。将来の VA/VLA（ViNT、OmniVLA-edge）と比べるための、十分に調整した古典的な回避と走行記録の仕組みを作る。VA/VLA の組み込みは範囲外。

## 1. 前提

### 1.1 段階Aで確定した事実

| 項目 | 値 | 根拠 |
| --- | --- | --- |
| LiDAR | VLP-16 Puck Hi-Res（上下±10°）。ドライバは±15°用の `VLP16db.yaml` のまま | 本体ラベル、床面当てはめ（今の設定で高さ1.15 m、Hi-Resで再計算すると0.777 m） |
| LiDAR取付 | x 0.42、y 0.36、z 0.777、roll 0.24°、pitch 0.97° | x・yは巻尺（2回、旧URDFと一致）、z・姿勢は床面当てはめ |
| 今のTFの誤差 | x +6.4 cm、y +2.2 cm、pitch 約+8° | `static_tf_launch.py` との比較 |
| カメラ | D435（IMUなし）、FW 5.12.11、レンズ中心0.73 m、LiDARの真下、水平 | `rs-enumerate-devices`、巻尺 |
| 深度で使える距離 | 小さな物（直径12 cm）で約3 m。4 mで距離が約+0.26 mずれ、5 mでは捉えられない | 静止物bag |
| LiDARの死角 | 見える一番下の高さが1.6 mで0.47 m、2.1 mで0.37 m、3.1 mで0.24 m。高さ21 cmの物は2〜3 mで見えない | 静止物bag |
| 今の `/scan` | 高さ21 cmの水筒が2〜5 mのどの距離でも出ない | bag再生による再現 |
| 前方の自己遮蔽 | なし | 4本のbagの点の割合 |

### 1.2 依頼者の決定（2026-10-28）

- 試験は構内の屋外アスファルト道路だけで行う。地図のある既知ルートから、見通しのよい直線20〜30 mを1区間選ぶ。交通の少ない時間帯に、見張り1名をつけて行う。屋内試験は行わない。
- カメラは0°（水平）のまま。新しいホルダは作らない。
- 縁石と下りの段差は範囲外（運用でカバーする）。
- MPPIとの比較を範囲に含める。
- apt（STVL、compressed-image-transport）と、★印の変更（F1〜F4、V1、V2、T1）を承認。
- 歩行者役は研究室のメンバー（依頼者が手配）。C3〜C4の搭乗は無人または研究者のみ。

### 1.3 比較基準の前提として明記すること

- **カメラ0°の弱点**：旋回中に、真横の足元へ現れる低い物は見えない。深度の視野は左右±43.5°、床が見えるのは前方1.3 mから。LiDARも近距離の低い物は見えない（§1.1）。
- 縁石と下りの段差は検出しない。
- 回避と停止の対象は「そこに物がある」ことだけで、人の分類はしない。

## 2. 構成

新しい部分は `avoidance:=true` のときだけ起動する。既定は false で、今のデモ構成（`scripts/start_dispatch_demo.sh`、`speed:=normal`）と同じ動作になる。

```
/velodyne_packets ─┬─ 既存 transform（VLP16db）─▶ /velodyne_points ─▶ localizer・Patchwork++#1・/scan・Layer C/D   【変更なし】
                   └─ transform#2（VLP16_hires_db）─▶ Patchwork++#2（sensor_height 0.777、frame_id_override=velodyne_hires）
                                                        └─▶ /perception/lidar_obstacles   [velodyne_hires]
D435 ─▶ 深度点群（2×2間引き、範囲はC1の屋外確認で決める）─▶ /camera/camera/depth/color/points
   いずれも ─▶ STVL（local・global）─▶ NavFn の再計画（1 Hz）＋ RPP または MPPI
           └─▶ Collision Monitor（approach で減速、stop で停止）
twist_mux ─▶ velocity_smoother ─▶ /cmd_vel_smoothed ─▶ Collision Monitor ─▶ /whill/controller/cmd_vel
```

| 引数 | 既定値 | 受け取るlaunch | 内容 |
| --- | --- | --- | --- |
| `avoidance` | false | `m6r_bringup`、`nav_launch` | true で D435 を起動する。C2以降で検知用の点群、STVL、Collision Monitor、failsafe の回避モードを追加 |
| `controller` | rpp | `nav_launch` | `rpp` か `mppi`。`mppi` は `avoidance:=true` のときだけ選べる |

### 2.1 `velodyne_hires` フレーム

- `base_link → velodyne_hires` を (0.42, 0.36, 0.777、roll 0.24°、pitch 0.97°、yaw −0.317°) とし、新しい `whill_perception/launch/perception_3d_launch.py` から配信する。`static_tf_launch.py` の `velodyne` には触らない。
- 使うのは検知（Patchwork++#2、STVL、Collision Monitor）だけ。localizer、地図、`/scan`、Layer C/D はこれまでどおり `velodyne` を使う。
- 補正した点群は、上流の `velodyne_transform_node` に `VLP16_hires_db.yaml` を読ませて作る。上流には出力フレームを変える設定がないので、自作の `patchworkpp_node` に `frame_id_override` を1つ追加する（既定は空で、今と同じ動作）。
- 受け入れ条件：`scripts/check_object_placement.py` で、置いた物の誤差が ±5 cm 以内。屋外の床面で、roll と pitch が道路の勾配の範囲に収まること。
- 根本対応（§8）が済んだら廃止する。

### 2.2 Layer D と Collision Monitor

- `avoidance:=true` のときは、Layer D を長方形（前方1.0〜2.0 m、左右±0.5 m）にする（`layer_d_shape`）。脇を通り抜ける物では止まらず、Collision Monitor とは別のデータ源（`/scan`）で独立に止める仕組みとして残す。参考：2026-07-24 のbag（843 s）では、扇形で43回作動した場面が、長方形では3回だった。
- 検知用の入力（`/perception/lidar_obstacles`、深度点群）の途絶は `extra_perception_topics` で監視する。Humble の Collision Monitor は入力が古くなっても停止しない可能性があるため。
- Collision Monitor は、公式チュートリアルどおり smoother の後ろに置く。`FootprintApproach`（衝突までの時間で減速）と前方の `PolygonStop`（停止）を使う。

## 3. 変更するファイルと影響範囲

| # | 変更 | ファイル | `avoidance:=false` での影響 | 状態 |
| --- | --- | --- | --- | --- |
| F1 | failsafe の閾値を ROS パラメータにする（既定値は今の値と同じ） | `failsafe_node.py` | なし（bag 2本で作動・解除の順序と時刻が一致） | C0済 |
| F2 | `layer_d_shape`（扇形・長方形） | 同上 | なし（既定は扇形） | C0済（引数との接続はC3） |
| F3 | `extra_perception_topics` | 同上 | なし（既定は空） | C0済 |
| F4 | `/failsafe/state` を配信 | 同上 | トピックが1本増えるだけ | C0済 |
| T1 | camera_link を (0.47, 0.378, 0.73) に直す | `static_tf_launch.py` | 使っているノードはない | C0済 |
| S1 | RealSense の設定 yaml、引数の受け渡し | `sensors_launch.py`、`config/realsense.yaml` | なし（realsense は既定で off） | C0済 |
| A1 | `avoidance`・`controller` 引数 | `m6r_bringup_launch.py`、`nav_launch.py` | なし | C0済（骨組み） |
| R1 | 記録スクリプト、置いた物の検証ツール | `scripts/` | なし | C1済 |
| P1 | transform#2、Patchwork++#2、`velodyne_hires` | 新しい launch、`patchworkpp_node.cpp` | なし | C2 |
| N1 | STVL、MPPI、Collision Monitor の設定 | `nav_launch.py`、新しい yaml | なし | C2〜C4 |
| V1 | smoother → Collision Monitor → driver（true のときだけ） | `nav_launch.py` | なし | C3 |
| V2 | speed プリセットを MPPI の `vx_max` にも反映 | `nav_launch.py` | なし | C4 |

twist_mux の優先度とタイムアウト、smoother の加減速、localizer と地図の設定、dispatch には手を入れない。

## 4. 作業表

| 段階 | 内容 | 検証 | 人日 |
| --- | --- | --- | --- |
| C0 | ブランチ、引数の骨組み、F1〜F4、T1、S1 | 回帰（bag 2本）、ユニットテスト、`--show-args` | 1.5 |
| C1 | 版合わせとFW更新（§4.1）、記録スクリプト、検証ツール、屋外での静止確認（§4.2） | 実機で起動とTFを確認、屋外のbagで距離・最低の高さ・横断勾配を求める | 1.5 |
| C2 | P1、STVL（local・global） | 屋外bagの再生で costmap を確認、検証ツールで受け入れ、**global の STVL の CPU 使用率と更新周期を測る** | 2.0 |
| C3 | Collision Monitor、V1、Layer D を長方形にする | 選んだ区間で、slow → normal の順に停止距離を実測（無人／研究者のみ） | 2.0 |
| C4 | 回避 (a) RPP＋global STVL＋再計画の調整 → (b) MPPI の調整、V2 | 選んだ区間（無人／研究者のみ） | 3.0 |
| C5 | §6 の測定、評価、報告 | — | 2.0 |
| 合計 | | | **12** |

### 4.1 RealSense の版合わせとFW更新（C1）

方針：realsense-ros の README が推奨する、ROS の apt バイナリの組み合わせ（`ros-humble-realsense2-camera` 4.58.4 と `ros-humble-librealsense2` 2.58.4）に揃える。今の構成（ソースビルドの 4.55.1 と apt の 2.57.7）は、Intel が組み合わせて出していない版の組み合わせ。

1. 依頼者が apt を実行する（§4.3）。librealsense2 は 2.58.4 に上がる。
2. ワークスペースにあるソースビルドを無効にする：`src/third_party/realsense-ros/COLCON_IGNORE` を置き、`build/` と `install/` の `realsense2_camera*` を削除する。削除するのはビルド成果物だけなので、再ビルドすれば元に戻せる。apt 版より優先されてしまうのを防ぐため。`whill_lab.repos` からも外す（C1で実施済み）。
3. `rs-enumerate-devices` で `Recommended Firmware Version` を確認する（2.57.7 では 5.17.0.10）。
4. Intel の配布ページから、その版の `Signed_Image_UVC_*.bin` を入手し、`rs-fw-update -f <bin>` で書き込む。カメラを使うプロセスがないことを先に確かめる。書き込み中はUSBを抜かない。
5. `rs-enumerate-devices` で版を確認し、`ros2 launch whill_safety m6r_bringup_launch.py avoidance:=true` で、深度・カラー・点群のトピックと TF を確認する。

### 4.2 屋外での静止確認（C1）

- **場所**：試験区間のアスファルト上。椅子は車線の向きに合わせて止める（横断勾配が roll として見えるようにするため）。
- **置く物**：コーン（高さを記録する）を中心線上の 1.5／2／2.5／3／3.5／4／5 m に順に置く。低い物（高さ約20 cm）を 1.5／2／3 m に置く。何も置かない基準を最初と最後に1本ずつ撮る。
- **記録**：`scripts/record_avoidance_bag.sh --profile static --duration 12 <タグ>` を各位置で実行する。テープは `base_link` の原点（後輪車軸の中点）から前方へ伸ばす。
- **解析**：`scripts/check_object_placement.py` を使う。notes.csv の列は `bag,object,x_m,height_m`。
- **決めること**：深度で使える距離（物の位置の誤差が ±5 cm 以内に収まる最大距離）、障害物とみなす最低の高さ（床の高さのばらつきの3σと、TF の誤差の合計を目安にする）、横断勾配（床面の roll）。

### 4.3 依頼者に実行してもらう apt

```bash
sudo apt update
sudo apt install ros-humble-spatio-temporal-voxel-layer ros-humble-compressed-image-transport \
    ros-humble-realsense2-camera ros-humble-realsense2-description ros-humble-librealsense2 \
    python3-pytest
```

`python3-pytest` は、whill_safety のユニットテスト（`colcon test`）を実行するために必要。

## 5. 記録の仕様

`scripts/record_avoidance_bag.sh`（mcap）。プロファイルは `static`（センサだけ）と `drive`（走行一式）。保存先は `~/bags/<日付>/<タグ>_<時刻>/` で、リポジトリには置かない。

- センサ：`/velodyne_packets`、`/velodyne_points`、IMU 2本、カラー（圧縮）15 Hz、深度（424×240）15 Hz、各 `camera_info`、`/tf`、`/tf_static`、`/rosout`
- 走行（drive のみ）：odom（生・EKF後）、`/pcl_pose`、`/alignment_status`、cmd_vel の各段（nav・teleop・safety・mux後・smoother後・driver入力）、`/failsafe/state`、`/scan`、検知用の点群、local costmap、global costmap（全体1回と差分更新）、`/plan`、Nav2 のゴールの状態と feedback、`/dispatch/state`、Collision Monitor の polygon
- 容量の見積もり：約21 MB/s ≈ 1.3 GB/分 ≈ 75 GB/時。空き容量605 GBで約8時間分。C1で実測して更新する。

## 6. 完成の定義

**場面**：S1 静止物が正面／S2 静止物が経路から0.3 mずれている／S1L 低い静止物（約20 cm）が正面／S3 歩行者が斜めに横切る／S4 歩行者と正面ですれ違う／S5 歩行者が前に来て立ち止まる。

**回数**：slow で RPP と MPPI を各場面5回ずつ比べる。normal は採用した制御器だけで各場面5回。

| 指標 | 目標 |
| --- | --- |
| 通過率 | 静止物 ≥ 9/10 相当（5回中5回）、歩行者 ≥ 8/10 相当（5回中4回以上） |
| 停止回数 | 1回以下 |
| 最小離隔（車体外形から最も近い点まで） | 静止物 ≥ 0.30 m、歩行者 ≥ 0.50 m |
| 所要時間 | 障害物なしの 1.5 倍以内 |
| 停止距離 | 実測値 ＋ 0.2 m が、検知を始める距離より短い |
| 不要な停止 | 100 m あたり 1 回以下 |
| 取付の検証 | 置いた物の誤差が ±5 cm 以内（補正したLiDARと深度） |
| `avoidance:=false` | 今と同じ動作 |

## 7. 方式の出典

- RPP は衝突を予測すると止まるだけで、避けない：[Nav2 RPP](https://docs.nav2.org/jazzy/configuration_and_development/configuration_guide/controller_plugins/configuring_regulated_pp/)
- MPPI は動的な障害物に向く：[Nav2: Setting Up Navigation Plugins](https://docs.nav2.org/setup_guides/algorithm/select_algorithm.html)、[nav2_mppi_controller](https://docs.ros.org/en/humble/p/nav2_mppi_controller/)
- Collision Monitor の置き場所と動作モデル：[Using Collision Monitor](https://docs.nav2.org/tutorials/docs/using_collision_monitor.html)、[humble README](https://api.nav2.org/nav2-humble/html/md_nav2_collision_monitor_README.html)
- STVL（時間で消える、深度・3D LiDAR のモデル）：[spatio_temporal_voxel_layer](https://github.com/SteveMacenski/spatio_temporal_voxel_layer)、消え残りの事例 [answers.ros.org #403262](https://answers.ros.org/question/403262/obstacle-and-voxel-layer-not-raytracing-free-space-properly/)
- realsense の apt での導入：[realsense-ros README](https://github.com/IntelRealSense/realsense-ros)

## 8. 別課題：根本対応（今回は実施しない）

1. ドライバの設定を `VLP16_hires_db.yaml` にする。
2. `base_link → velodyne` を (0.42, 0.36, 約0.78、pitch 約1°) に直す。
3. Patchwork++ の `sensor_height`、p2ls の高さの帯（ADR-0009）、Layer D の閾値を見直す。
4. 既存のbagを Hi-Res の角度で補正し直す（`check_object_placement.py` の `to_hires()` と同じ計算で、ノードの出力と 0.00 mm で一致）。そのうえで GLIM → DUFOMap → `static.pcd`・占有格子を作り直し、v2 の手描きマスクを移し替える。補正前後で GLIM の IMU 検証の比率（rot・trans・vel）を比べる。
5. localizer の受け入れ試験（ADR-0006）をやり直し、配車 Web の地図、校正記録、ADR を更新する。

影響範囲：localizer、地図、failsafe の Layer B〜D、p2ls、Patchwork++、GLIM、dispatch の Web 地図。工数は5〜8人日＋実機での受け入れ。
