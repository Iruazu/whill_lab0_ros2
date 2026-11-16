# LiDAR の yaw（base_link → velodyne）の測り方（#149）

校正台帳 §2.1 の yaw は**未測定**（−0.317° は noetic の LiDAR 対 IMU の値の流用）。床面フィット（`scripts/lidar_floor_fit.py`）は z・roll・pitch しか観測できない。この文書は、yaw を測る方法の設計と、既存の bag での試行の結果、屋外 2 回目（#156）の手順をまとめる。

## 1. 原理と符号

- 規約は台帳と同じ tf2 の RPY（R = Rz(yaw)·Ry(pitch)·Rx(roll)、velodyne 系の点を base_link 系へ移す）。yaw は z 軸まわりに反時計回り（上から見て左回り）が正。
- WHILL CR2 は後輪の差動駆動で横に滑らない（非ホロノミック）。base_link の原点は後輪車軸の中点なので、**原点の速度は常に車体の x 軸を向く**（曲がっていても）。
- NDT が求めるのは点群を地図に合わせた T_map_velodyne で、`/pcl_pose` はそれに使った TF の逆を掛けた T_map_base = T_map_velodyne · T_base_velodyne_used⁻¹。
- 使った TF の yaw ψ_used が真の ψ_true と ε = ψ_true − ψ_used だけ違うと、推定の向きは真の向きより ε だけ左に回る（R_est ≈ R_true·Rz(ε)）。進む向きは真の車体 x のまま（回転が一定なら、TF の並進のずれは位置の定数のずれになるだけ）。
- したがって、進んだ向きを**推定の車体系**で表した方位角 a は −ε になり、

  **ψ_true = ψ_used − a**（a > 0：推定の車体から見て左へ進んで見える ＝ LiDAR は実際には TF より右に回っている）

  言い換えると、LiDAR が +ψ 回っていれば、車体の前方は velodyne 系で方位 −ψ に見える。
- 3 次元のまま車体系で測るので、地図の傾き・坂の影響を受けない。roll・pitch の取り違えが混ざるのは二次（記録時の旧 TF（pitch +9°）のままでも 0.005°、`scripts/tests/test_lidar_yaw_from_drive.py`）。それでも厳密にするため、スクリプトは記録の姿勢を記録時の TF で T_map_velodyne に戻し、台帳の roll・pitch・並進で base_link を作り直してから測る。
- 符号は合成の走行（真の取り付け +2° の LiDAR を 0° の TF で使う）で確かめた：結果は +2.000°、a は −2.000°（テスト `test_sign_and_value`）。

## 2. 推定量（`scripts/lidar_yaw_from_drive.py`）

- 窓：odom（`/odometry/filtered`、または車輪だけの `/whill/odom`）で |v| > 0.3 m/s・|ω| < 0.3 rad/s の時刻を選び、odom の道のり 2 m ごとの重ならない窓に切る。窓の中の正味の向きの変化が 15° を超える窓は捨てる。
- 各窓で、隣り合う姿勢の移動 Δp_k を、その間の中間の回転の車体系へ移した d_k を足し合わせ、和の方位角を a とする。各 d_k が +x を向くので、蛇行（07-24 は Nav2 の蛇行 #101 で |ω| の中央値 0.043 rad/s）でも偏らない。中間の回転を使うのは、R_k のままだと 10 Hz・0.3 rad/s で ω·dt/2 ≈ 0.9° の偏りになるため。NDT の前後方向の行ったり来たり（07-24 の記録で 1 歩 0.2〜0.3 m が 4 %）は和で打ち消し合う。
- 停止の組（`--stops`）：3 秒以上の停止の平均の姿勢 2 つ（3 m 以上離れ、途中の向きが出発から 15° 以内）の差を、中間の車体系で測る。停止中は NDT が odom の予測に引っぱられず、走行中の点群のゆがみもない。屋外 2 回目の手順用（§5）。
- 出力：窓ごとの yaw、中央値・IQR・平均・標準偏差・中央値のブートストラップ 95 % 区間。

偏りの要因と大きさ：

| 要因 | 大きさの見積もり | 扱い |
| --- | --- | --- |
| NDT の横の位置の雑音 σ | 1 窓 √2·σ/L（σ 2 cm、L 2 m で 0.8°） | 窓の数で平均。中央値の区間に出る |
| TF の並進・base_link の原点のずれ δ | 窓の正味の向きの変化 Δθ に対して |Δθ|·|δ|/L（15°・2 cm・2 m で 0.15°）。符号は曲がる向きで変わる | `--max-turn-deg` |
| NDT の初期値が odom（`use_odom: true`）で、推定の車体 x 方向に予測される | NDT が毎回横ずれを直しきらないと a が 0 に寄る（感度が落ちる） | 再生で TF の yaw を +3° ずらして確かめた：推定は −0.005° しか動かない（§3.2）。07-24 の bag では無視できる |
| 走行中の点群のゆがみ（1 回転 0.1 s に 4.5 cm 進む） | 前後方向のずれが主で、横はほぼ出ない | 停止の組（`--stops`）なら無い |
| 横断勾配での横滑り・後輪の空気圧の差 | 未評価。往復すると横滑りは向きで符号が変わらない | 往復の両方向で測って比べる（§5） |
| 地図の局所的なゆがみ | 窓ごとに違う向きで入る | 場所・向きで分けて比べる |
| **取り付けがいつのものか** | 07-24 の bag は 2026-11-13 のテープ固定より前 | 今の取り付けは屋外 2 回目で測る |

## 3. 既存の bag での試行（2026-11-16）

bag：`~/bags/m7-bench-data/2026-07-24-live-campus/rosbag2_2026_07_24-19_05_26-hires`（07-24 の Nav2 の走行、841 秒、#152 の方法で点群を Hi-Res に補正したもの）。**この bag は 2026-11-13 のテープ固定より前の取り付けで、記録された `/pcl_pose` は旧 TF（bag の `/tf_static`：(0.484, 0.382, 0.794)・RPY (−2.02°, +8.99°, −0.317°)）・旧 campus 地図・旧校正表の点群で出たもの**。今の取り付けの yaw ではない。

### 3.1 記録された `/pcl_pose`（旧 TF・旧地図）

```bash
cd ~/whill_lab0_ros2   # このブランチの checkout（worktree ならそのディレクトリ）
source /opt/ros/humble/setup.bash
B=~/bags/m7-bench-data/2026-07-24-live-campus/rosbag2_2026_07_24-19_05_26-hires
python3 scripts/lidar_yaw_from_drive.py $B                           # 曲がりの判定は /odometry/filtered
python3 scripts/lidar_yaw_from_drive.py $B --rate-topic /whill/odom  # 車輪だけ
```

| 条件 | 窓 | yaw 中央値 | IQR | 標準偏差 | 中央値の 95 % 区間 |
| --- | --- | --- | --- | --- | --- |
| 既定（`/odometry/filtered`、窓 2 m） | 72 | **+1.02°** | +0.30〜+1.89° | 1.21° | +0.66〜+1.40° |
| `/whill/odom` | 89 | +0.92° | +0.32〜+1.90° | 1.39° | +0.68〜+1.29° |
| 窓 1 m | 169 | +0.96° | — | 2.33° | +0.64〜+1.19° |
| 窓 4 m | 27 | +0.67° | — | 0.73° | +0.37〜+1.29° |
| 正味の向きの変化 5° 以内 | 53 | +1.01° | — | 1.05° | +0.65〜+1.40° |
| |ω| < 0.1 rad/s | 17 | +1.03° | — | 0.85° | +0.74〜+1.79° |
| |v| > 0.4 m/s | 24 | +1.46° | — | 1.05° | +0.87〜+1.72° |
| 作り直さない（旧 TF の base_link のまま） | 73 | +0.92° | — | 1.28° | +0.63〜+1.30° |

- 窓の向き（4 象限）ごとの中央値は −0.04〜+1.34°（窓 9〜25 個ずつ）、時間で 3 つに分けると +0.77〜+1.11°。正味の向きの変化との相関は −0.04（`/odometry/filtered`）。
- 停止の組（`--stops`）は 0 組（Nav2 の走行で、まっすぐの区間を挟む停止がない）。

### 3.2 現在の TF・campus-m8 での NDT の再生

同じ Hi-Res の bag を、今の TF（台帳 §2.1）と campus-m8 で `scripts/regression/ndt_replay_compare.py` により再生した（全区間 741 秒、bag の 73〜814 秒）。odom の予測（`use_odom: true`）で軌跡が引っぱられるかを確かめるため、TF の yaw だけを **+3°** ずらした再生も 1 本回した。`ndt_replay_compare.py` は姿勢を回転ごと `poses_full.txt` に残すようにした（この変更で追加）。

```bash
cd ~/whill_lab0_ros2   # このブランチの checkout（worktree ならそのディレクトリ）
source /opt/ros/humble/setup.bash
source ~/whill_lab0_ros2/install/setup.bash   # main 側のビルドを読むだけ
# 走行中の車載 PC（domain 0）や他の再生と分ける。空でなければスクリプトが止まる。
export ROS_DOMAIN_ID=149 ROS_LOCALHOST_ONLY=1 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
unset CYCLONEDDS_URI
B=~/bags/m7-bench-data/2026-07-24-live-campus/rosbag2_2026_07_24-19_05_26-hires
M=~/yaw_maps; mkdir -p $M/campus-m8   # static.pcd は Git 管理外。正本へのリンクを置く（sha256 d2448658…03b5 を確認）
ln -sf ~/whill_lab0_ros2/docs/maps/campus-m8/static.pcd $M/campus-m8/static.pcd
T=docs/maps/campus-m8/align_from_campus/T_new_old.txt
O=~/yaw_replay_$(date +%Y%m%d_%H%M%S)   # 出力先は毎回新しくする（既にあると止まる）
python3 scripts/regression/ndt_replay_compare.py --bag $B --site campus-m8 --maps-root $M \
  --lidar-tf "0.42 0.36 0.771 -0.05 0.40 -0.317" --map-transform $T --start 73 --duration 741 --out $O/cur
python3 scripts/regression/ndt_replay_compare.py --bag $B --site campus-m8 --maps-root $M \
  --lidar-tf "0.42 0.36 0.771 -0.05 0.40 2.683" --map-transform $T --start 73 --duration 741 --out $O/plus3
python3 scripts/lidar_yaw_from_drive.py $B --poses $O/cur/poses_full.txt   --used-tf "0.42 0.36 0.771 -0.05 0.40 -0.317"
python3 scripts/lidar_yaw_from_drive.py $B --poses $O/plus3/poses_full.txt --used-tf "0.42 0.36 0.771 -0.05 0.40 2.683"
```

各 1 本、約 13 分。2 本とも終了コード 0、閾値超え 0 回・収束しなかった回 0 回、姿勢 7299 個（記録 7298 個）。

| 再生 | NDT の fitness 中央値 / p95 / 最大 | 曲がりの判定 | 窓 | yaw 中央値 | IQR | 標準偏差 | 中央値の 95 % 区間 |
| --- | --- | --- | --- | --- | --- | --- | --- |
| 今の TF（yaw −0.317°） | 0.0154 / 0.065 / 0.62 | `/odometry/filtered` | 72 | **+1.16°** | +0.48〜+1.65° | 1.10° | +0.88〜+1.41° |
| 同上 | 同上 | `/whill/odom` | 89 | +0.99° | +0.39〜+1.75° | 1.34° | +0.75〜+1.32° |
| yaw を +3° ずらした TF（+2.683°） | 0.0154 / 0.065 / 0.34 | `/odometry/filtered` | 72 | +1.10° | +0.51〜+1.80° | 1.09° | +0.89〜+1.34° |
| 同上 | 同上 | `/whill/odom` | 89 | +0.93° | +0.41〜+1.77° | 1.34° | +0.76〜+1.24° |

読み取れること：

- **地図・TF を変えても同じ値が出る**：旧 TF・旧地図の記録（§3.1、+1.02°）と、今の TF・campus-m8 の再生（+1.16°）は 95 % 区間が重なる。窓の向き 4 象限ごとの中央値は +0.42〜+1.41°、時間の 3 分割で +0.94〜+1.25°。
- **odom の予測による引っぱりは見えない**：yaw を +3° ずらしても推定は +1.10°（同じ窓 72 組の差の中央値 −0.005°、IQR −0.11〜+0.09°）。引っぱられていれば推定は +2.68° 側に寄るはずで、感度は 3° に対して 99 % 以上。NDT が出す T_map_velodyne は TF の yaw によらない、という §1 の前提が成り立っている。
- **NDT の fitness は yaw の 3° のずれを区別しない**（中央値・p95 が同じ）。yaw を決めるのは fitness ではなく、この運動学の関係である。
- 1 窓の標準偏差は約 1.1°、窓 72 個で中央値の 95 % 区間の幅は ±0.25° 程度。条件（窓の長さ・速さ・曲がりの判定）を変えた幅は §3.1 で +0.7〜+1.5°。**07-24 の取り付けの yaw は約 +1.0°（−0.317° より約 1.3° 左回り）、不確かさは ±0.5° 程度**と読む。横滑りなど往復で打ち消せない偏りは、この bag（往復の直線がない）では評価できていない。

## 4. 静止のコーンの並びとの比較（2026-11-16 の屋外静止確認 D）

屋外 D（台帳 §2.1）では、テープに沿って 1.5〜5 m にコーン 7 本を置いた（今の取り付け、TF は台帳の値）。コーンの中心の y（`docs/calibration/2026-11-16-outdoor-static/results.json`）を x に対して直線で当てはめると：

| センサ | y の傾き | テープの向き（base_link 系、今の TF） |
| --- | --- | --- |
| LiDAR | +1.19 cm/m | **+0.68° ± 0.07°**（左回り） |
| 深度（D435、yaw 0 の仮の値） | +0.72 cm/m | +0.41° ± 0.13° |

- テープが車体の x と平行なら、LiDAR の yaw は −0.317° − 0.68° ＝ **約 −1.0°** になる。走行の推定（07-24、+1.0°）とは**符号が逆で約 2° 食い違う**。
- テープは後輪のハブ中心の中点から前へ**目分量で**引いたもので、車体の x との平行は確かめていない（5 m で 17 cm 横にずれれば 2°）。LiDAR と深度が同じ向き（左回り）にテープを見ていることは、テープ自体が車体の x から左に回っていた可能性と矛盾しない。両者の差 0.27° は LiDAR と D435 の相対の yaw（同じホルダに共締め）を表す候補だが、1 回だけの値。
- 07-24 から 11-16 までに取り付けが yaw に回った可能性（11-13 のテープ固定で水平を合わせ直した）もある。どちらが原因かは、既存のデータでは分けられない。

## 5. 結論と屋外 2 回目（#156）の手順

結論：

- **原理と符号は合成で確かめた**（§1、テストは `scripts/tests/test_lidar_yaw_from_drive.py`）。
- 07-24 の bag（**テープ固定前の取り付け**）では、記録の `/pcl_pose` から +1.0°（95 % 区間 +0.7〜+1.4°、条件を変えて +0.7〜+1.5°）。今の TF・campus-m8 で再生し直しても +1.16°（+1.0〜+1.2°）で、TF の yaw を +3° ずらした再生でも値が動かない（感度 99 % 以上）。**走行による方法は、odom と組み合わせた NDT でも感度が十分**で、使える。
- 今の取り付けについては、静止のコーンの並び（テープの向き）から −1.0° が出るが、テープの平行を確かめていないので測定にならない。**台帳の yaw は変えない（未測定のまま）**。

屋外 2 回目でやること（今の取り付けで、走行とコーンの両方を同じ日に行い、どちらの前提が崩れているかを分ける）。所要 約 30 分。

1. 起動は runbook（`docs/runbook-live-demo.md`）どおり（m6r_bringup + nav_launch + dispatch_launch、failsafe / twist_mux 有効。センサ・odom を別に起動しない。Nav2 のゴールは出さない）。ジョイスティックで走る。NDT を `campus-m8` で初期化し、`/alignment_status` の fitness が通常の値であることを確かめる。
2. 記録（`ros2 bag record`）：`/velodyne_points /pcl_pose /alignment_status /odometry/filtered /whill/odom /imu/data_rep145 /tf_static /tf`。
3. **停止の組（主）**：地図のある、まっすぐで平らな区間（工農研の前の通り）で、止まる 5 秒 → ジョイスティックをまっすぐ前に倒して約 0.5 m/s で 8 m 進む → 止まる 5 秒 → 同じ線を 8 m 後退 → 止まる 5 秒、を **4 往復**。途中でジョイスティックを左右に動かさない（曲がった組は解析で捨てる）。続けて、車体を 180° 回して**逆向きでも 4 往復**（横断勾配の横滑りと地図の局所的なゆがみは、向きを変えると符号や量が変わるので、向きごとの差で見分ける）。
4. **走行の窓（副）**：同じ区間を、止まらずに約 0.8 m/s で 30 m 以上、両方向に 2 回ずつ。
5. 解析：

   ```bash
   cd ~/whill_lab0_ros2   # このブランチの checkout
   source /opt/ros/humble/setup.bash
   B=<記録した bag>
   python3 scripts/lidar_yaw_from_drive.py $B --stops --rate-topic /whill/odom --json /tmp/yaw_stops.json
   python3 scripts/lidar_yaw_from_drive.py $B --json /tmp/yaw_windows.json
   ```

   判定の目安：停止の組 16 組の中央値の 95 % 区間の幅が 0.5° 以下で、前進・後退・2 つの向きの中央値が 0.5° 以内で揃えば、その値を台帳に採る。揃わなければ、横滑りか地図のゆがみが疑わしいので採らない。
6. **コーンの線で交差確認（地図・NDT を使わない）**：
   - 平らな場所で、車体をまっすぐ 5 m 前に進め、出発と到着で左右の後輪のハブ中心から床へ下げ振り（または水平器で垂直に当てた定規）で 2 点ずつ印を付け、それぞれの中点を結ぶ線を引く。**この線は車体が実際に進んだ向き＝車体の x**（目分量のテープではなく、走りで決める）。中点の印の誤差 1 cm なら、5 m で 0.16°。
   - 到着した位置のまま（動かさない）、その線の延長上 3 m と 7 m にコーンを置き、静止 12 秒を記録（`scripts/record_avoidance_bag.sh --profile static`）。`scripts/check_object_placement.py` の y_center を x に対して当てはめた傾きの角度を θ とすると、yaw ＝ −0.317° − θ（§4 と同じ式）。180° 回して同じことを 1 回。
   - 走行の値とこの値が 0.5° 以内で一致すれば、その平均を台帳に採る。11-16 の −1.0° と +1.0° の食い違いの原因（テープの平行か、取り付けの変化か）もここで分かる。
7. 結果は台帳 §2.1 の yaw の行と §7 の履歴に書き、0.2° 以上変われば `src/whill_sensors_bringup/launch/static_tf_launch.py` の yaw を同じコミットで変える。
