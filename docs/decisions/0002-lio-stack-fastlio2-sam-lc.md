# ADR-0002: LiDAR-Inertial Odometry スタックに FASTLIO2_SAM_LC (liangheming) を採用する

- 日付: 2026-05-28
- 状態: accepted (2026-05-28, ユーザー承認)

## 文脈

現行 `whill_localization` は `hku-mars/FAST_LIO@ROS2` (バニラ FAST-LIO) を
ラップしている。これは M4 で動作実証済みだが以下の問題を抱える。

- **ループクロージャ無し**: 60 秒ドライブで終端誤差 18%。キャンパス 200 m
  以上の走行で `map` フレームが破綻する
- **リローカライゼーション機能無し**: 起動時に常に原点から出発するため、
  事前点群地図 (M5-b で作成済み) に対する位置合わせができない。Nav2 を
  `map` フレームに対して使うには別途リローカライゼーション層が必要
- **`map -> camera_init` の identity 接続**: M5-a で導入した暫定対処
  (`tf_bridge_launch.py`) で誤魔化しており、地図とのアラインメントが取れない

旧 noetic は `slam_localization/` の PCL NDT がリローカライゼーションと
ドリフト補正を担っていたが、本リポはこれを移植せず Nav2 標準スタックに
合う OSS で代替する方針 (research 文書の論点 3)。

## 検討した選択肢

### 選択肢 A: バニラ FAST-LIO のまま + 別ノードで AMCL / NDT localization 追加
- 現状の `whill_localization` を維持し、地図補正は別 stack
- メリット: 既存パッケージへの破壊的変更が小さい
- デメリット:
  - 2 つの localization スタックを並走させる複雑さ
  - AMCL は VLP-16 の LaserScan 変換で屋外高低差が壊れる (research 地雷)
  - NDT 単独だとループクロージャ無しは変わらず、長距離ドリフトに対し
    マッチングコストが嵩む

### 選択肢 B: FASTLIO2_SAM_LC (liangheming) に置換
- GTSAM ベースの pose graph 最適化 (ループクロージャ)
- `/slam_reloc` サービス (ICP ベースのリローカライゼーション)
- ROS 2 humble 対応、2024-2025 active
- メリット:
  - FAST-LIO2 コアを内部に持つためドロップイン置換に近い
  - ループクロージャと relocalization が 1 つの stack で完結
  - 既存 `whill_localization/config/velodyne_whill.yaml` の extrinsic を
    ほぼそのまま流用可能
- デメリット:
  - GTSAM 依存 (apt 提供だがバージョン衝突可能性)
  - ループクロージャは「位置近傍 ICP」のため開放経路 (長い直線) で効きにくい
  - MIT ライセンス (本リポ BSD-3 統一からの差分。MIT は緩いので問題なし)

### 選択肢 C: autoware_ndt_scan_matcher
- Autoware 提供の NDT スキャンマッチャ
- メリット: Autoware エコシステムでの実績
- デメリット: Autoware への依存が深く、本リポへの切り出しコストが高い

### 選択肢 D: lidar_localization_ros2 (ndt_omp 系) 単独
- 軽量 NDT pure localization (rsasaki0109)
- メリット: 計算負荷が軽く、安定走行時の選択肢として優秀
- デメリット:
  - マップ作成機能を持たない。別途 LIO で地図を作ってから流す前提
  - Phase B の段階 B (将来) としては有力だが、初期マップ作成段階で使えない

## 決定

**選択肢 B (FASTLIO2_SAM_LC)** を採用する。ただし二段階運用とし、将来的に
選択肢 D を「段階 B」として安定走行時に切り替える余地を残す。

段階 A (本 ADR 範囲, M5-e / Phase B):
- `FASTLIO2_SAM_LC` で「マップ作成 + ループクロージャ + リローカライゼーション」
  を一括担当
- `map -> odom` をこの stack が直接 publish (ADR-0001 の二段 EKF と整合)

段階 B (将来, M6 安定化後):
- マップが固まったら `lidar_localization_ros2` の ndt_omp pure localization に
  切り替え、計算負荷を削減
- 本 ADR は段階 B 移行時に supersede される可能性あり

理由:
- ループクロージャ + relocalization + LIO を 1 stack で持つのは humble 対応 OSS
  では FASTLIO2_SAM_LC のみ (research 比較表)
- FAST-LIO2 コア継承で既存 extrinsic / config 流用可能、移行コスト最小
- 段階 A → B 切替路があるため「最初から完璧」を要求せずに済む

## 帰結

良い側面:
- 60 秒ドライブ 18% ドリフトを 5% 以下に圧縮できる見込み (ループクロージャ効果)
- 事前点群地図に対する `/slam_reloc` で Nav2 を `map` フレームに正しく
  ローカライズできる
- `tf_bridge_launch.py` の identity 接続を削除でき、TF tree が正常化

悪い側面:
- GTSAM ビルド失敗リスク (Phase B 早期に build smoke test 必須)
- 開放経路でループクロージャが効かないため、キャンパス長い直線ではドリフトが
  残る可能性。relocalization で間欠補正することで凌ぐ
- 上流が活発な分、API breaking change に追随する保守コスト

将来見直すべき条件:
- マップ作成が一旦完了し、毎回新規 SLAM する必要が無くなった時点で段階 B
  (ndt_omp pure localization) に移行
- FASTLIO2_SAM_LC upstream の保守が停止した場合は HDL Localization (ROS2) または
  自作の pose graph 層への移行を検討
