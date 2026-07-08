# ADR 0006: 運用 localizer の選定 (M6-R)

Language: [日本語](0006-localizer-choice.md) | [English](../../en/decisions/0006-localizer-choice.md)

- Status: **proposed** (M6R-1 着手時起案、M6R-5 完了時 accepted 化予定)
- Date: 2026-07-08
- Deciders: Iruazu (承認待ち)

## 背景

親方針 ([`../plans/2026-06-11-platform-pivot.md`](../plans/2026-06-11-platform-pivot.md))
§3.3 は運用 (オンライン車載) 側の scan-to-map localizer の第一候補として
`lidar_localization_ros2` を挙げた。M6-R 実行計画
([`../plans/2026-06-24-m6r-localization.md`](../plans/2026-06-24-m6r-localization.md))
§6 M6R-1 はこれを実バグリプレイで動作確認し、確定した commit SHA と IMU
配線方針を本 ADR に記録することを求める。

本 ADR は M6R-1 着手時点で **proposed** として起案し、M6R-5 の受入テスト
(G1-G3) 通過をもって **accepted** に昇格させる。それまでの間、他の候補
(hdl_localization, Autoware ndt_scan_matcher 等) に切り替える場合は本 ADR を
差し戻して選定をやり直す。

## 決定

1. **採用: `rsasaki0109/lidar_localization_ros2`** (upstream fork)。
   - ライセンス: BSD-2-Clause (親方針 §3.4 permissive 要件を満たす)
   - registration: NDT_OMP (fork の `ndt_omp_ros2` に依存)
   - 提供 topic: `map -> odom` TF, `/pcl_pose` (`geometry_msgs/PoseStamped`),
     `/alignment_status` (`diagnostic_msgs/DiagnosticArray`) を publish
2. **vcs pin (proposed 段階)**: `whill_lab.repos` で `version: main` (2026-07-08
   時点の HEAD)。M6R-1 smoke test 完了時に **確定 commit SHA を pin** する
   (M5R-3 の GLIM ADR で tag pin を選んだ前例と整合)。
3. **依存パッケージ**: `ndt_omp_ros2` (rsasaki0109 fork, humble branch) を
   `whill_lab.repos` に合わせて追加。単一 `vcs import` で完結する運用を維持。
4. **IMU 配線既定 = `use_imu: false`**:
   - v1.1.0 の `lidar_localization_component.cpp` 直読で確認済 (既定 false)
   - `use_imu: true` 時の用途は scan undistortion であり、EKF prediction ではない
   - 本フェーズでは EKF (M4-R) が `/imu/data_rep145` を消費して
     `odom -> base_link` を supply し、localizer は独立に `map -> odom` のみ
     supply する経路分離を維持
   - 将来 scan undistortion が必要 (高速旋回等) になった時は、`use_imu: true`
     に切替 + `imu` topic を `/imu/data_rep145` に remap (**生 `/imu/data_raw`
     ではない** — PR #56 で REP-145 化された corrected な方を渡す)
5. **入力地図パス**: `docs/maps/<site>/static.pcd` (ADR-0005 準拠)。
   `map_path` パラメータで localizer に渡す。
6. **NDT パラメータ初期値** (`param/boreas_ndt_velodyne.yaml` を出発点、
   scripts/m6r_smoke_test.sh 内でクローン):
   - `ndt_resolution: 1.0` m
   - `ndt_step_size: 0.1`
   - `ndt_max_iterations: 25`
   - `transform_epsilon: 0.01`
   - `voxel_leaf_size: 0.5` m (VLP-16 の疎な点群に合わせて Boreas 128 の 1.5 から
     下げる)
   - `score_threshold: 6.0`
   - `scan_max_range: 80.0` / `scan_min_range: 1.0` m
7. **map -> odom TF 出力**: `enable_map_odom_tf: true` を明示。REP-105 の連続
   `odom -> base_link` (M4-R EKF) と接続する運用を前提とする。

## 採用しなかった案

- **`koide3/hdl_localization`** (BSD-2-Clause):
  - 実績あり (M9 選択肢としては残すが第一候補ではない)
  - 採用しなかった理由: NDT_OMP + KDTree の両方を選べる利点はあるが、
    親方針 §3.3 の「つくばチャレンジ 2024 実績 (odometry 拘束併用前提)」の
    根拠は `lidar_localization_ros2` 側にある。棄却理由は「実績側の差」
    のみで、技術的な棄却ではない
- **Autoware `ndt_scan_matcher`** (Apache 2.0):
  - 品質は高い。棄却理由: 依存が大きい (autoware_ 系のスタック全体を
    引き摺る) ため、単体パッケージ運用の M6-R には過剰
- **`FAST_LIO_LOCALIZATION` 系** (GPL):
  - GPL 継承が懸念材料 (親方針 §3.4)。棄却
- **自作 NDT** (from scratch):
  - 実装工数を掛ける価値なし (upstream 完成品で足りる)。棄却
- **IMU 有効化を既定にする** (`use_imu: true`):
  - scan undistortion 効果は WHILL 速度域 (最大 1.7 m/s) で顕著でないと
    推定。加えて `/imu/data_rep145` の REP-145 補正符号が正しいかを
    localizer 側で追加検証するコストがかかる。M6R-1 smoke test で
    `use_imu: false` が pass するなら不要。M6R-5 の余力次第で追加試験
    (下記 Consequences 参照)

## 結果

Positive:
- vcs 一発 import + colcon build で環境再現。build 実測 2 分 20 秒
  (ndt_omp_ros2 25 s + lidar_localization_ros2 115 s、Alienware x15 R2 = i9-12900H)
- `use_imu: false` の経路分離で M4-R EKF と localizer の責務が明快
- `docs/maps/<site>/` 規約 (ADR-0005) との親和性が高い
- BSD-2-Clause で親方針 §3.4 permissive 要件を満たす

Negative / TBD:
- **[M6R-1 で解消] fork 上流の main HEAD 変化リスク**: 現在の pin は
  `version: main` で不安定。M6R-1 完了時に確定 commit SHA へ pin し直す
- **[M6R-3 の入力] `/alignment_status` の実フィールド構成が未確認**:
  上流 README に schema が書かれていない。M6R-1 smoke test で
  `ros2 topic echo /alignment_status --once` を採取し、
  `docs/ja/m6r-localizer-eval.md` に記録
- **[M6R-1 で解消] GLIM 出力 PCD の voxel と NDT resolution の整合**:
  campus-half-v3 の PCD は drift 66 m を含む。`voxel_leaf_size` 0.5 m と
  `ndt_resolution` 1.0 m の組合せで実際に NDT が locking できるかは
  smoke test で実測
- **[demo 後の課題] `use_imu: true` 側の未評価**: 高速旋回時の scan
  undistortion 効果は本 ADR 決定範囲外。demo 後、必要なら別 ADR で追記
- **[M6R-3 依存] 失探定義**: `score_threshold: 6.0` は Boreas プリセット
  そのままの値。WHILL / VLP-16 / 屋外キャンパスでの真の failure threshold
  は M6R-1〜M6R-3 で調整、確定値を本 ADR の「決定 6」に追記

## M6R-5 accepted 化条件

以下 4 点が全て pass した時点で本 ADR を **accepted** へ昇格させる:

- [ ] M6R-1 smoke test で `map -> odom` TF が連続出力 (親方針 §6 M6R-1)
- [ ] M6R-5 の G1-G3 実機試験全て pass (計画書 §5)
- [ ] 確定 commit SHA が `whill_lab.repos` にピン
- [ ] `/alignment_status` schema と NDT failure threshold の実測値が本 ADR
  「決定 6」に記載
