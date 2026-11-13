# デモ回帰チェック

main を常にデモができる状態に保つための、マージ前の確認。変更の内容に応じて、次の段階まで通す。実機の受け入れの代わりにはならない。

| 段階 | コマンド | 時間 | 通す変更 |
| --- | --- | --- | --- |
| 1. 静的 | `scripts/demo_regression.sh` | 1分未満 | すべての PR |
| 2. 再生 | `scripts/demo_regression.sh --level replay` | 約12分 | failsafe、localizer、TF、センサ処理、地図、launch の配線 |
| 3. 屋外 | runbook のとおりデモのルートを1周 | — | 走行に影響する変更（Nav2 のパラメータ、速度、地図、TF など） |

文書だけの変更は段階1で足りる。迷ったら上の段階まで通す。段階3を省くときは、PR に理由を書く。

実機は要らず、走行指令も出さない。ROS は domain 142・localhost に閉じ、domain に他のノードがいれば再生を始めない。出力は `.test_ws/regression/` に置く。

## 確かめていること

**静的**：`scripts/run_tests.sh`、地図と Web の背景の同期（`scripts/check_map_sync.py`。このブランチの `docs/maps` と `src/whill_dispatch/web` を照合）、`start_dispatch_demo.sh --dry-run`、3つの launch の `--show-args`。

**再生**：基準 bag（2026-07-24 の走行。`~/bags/m7-bench-data/` にある。[置き場](bags.md)）を、このブランチのコードに流す。閾値は `scripts/regression/baseline.json`。

| 検査 | 入力 | 見るもの | 使う閾値の理由 |
| --- | --- | --- | --- |
| failsafe | `/scan`、`/pcl_pose`、`/alignment_status`（全行程、2倍速） | 走行中の停止、作動の回数と時間 | 走行中の停止は基準で0回。回数と時間は再生ごとに揺れる（40〜42回） |
| localizer | `/velodyne_points` ほか（走行中の240秒） | bag に記録された `/pcl_pose` との差 | 同じ入力なら結果がほぼ再現し、中央値の差は 0 mm |

わざと壊して検出を確かめた（`scripts/tests/test_regression_compare.py` に実測値がある）。

| 壊し方 | 結果 |
| --- | --- |
| LiDAR の TF の x を +10 cm | xy の差の中央値 0.099 m、p95 0.132 m（正常時は 0.000 m、0.08 m 以下） |
| Layer D の扇形を ±60° | 走行中の停止 9回、作動時間 43.3 秒（正常時は 0回、33 秒） |

## 確かめていないこと

- **Nav2**：経路計画と制御は動かない。段階3で確かめる。
- **Layer C**：bag に `/velodyne_points_no_ground` が無いので、作動しない。
- **入力の途絶の検出**：再生の開始時に failsafe が `DEAD INPUT` を出すが、当てにならない。届いている Layer B と D も挙げるので、時刻が bag の時刻へ飛んだ直後に判定が走っていると見られる（確かめていない）。実機とは条件が違う。
- **速度の指令と停止距離**：車体を動かさないので測れない。
- **NDT の fitness**：TF を 10 cm ずらしても変わらなかったので、閾値にしていない。
- **地図と入力点群そのものの正しさ**：記録された姿勢と同じであることを見ているだけで、記録が正しいとは限らない。

## 今の基準の状態（2026-11-13）

**再生の localizer の項目は、今の main では失敗する見込み。** `baseline.json` は 2026-11-02（main 81f3ec7）に、旧い LiDAR の TF（base_link→velodyne 0.484, 0.382, 0.794 m、roll −2.03°、pitch +9.00°）で測った。#186 で TF は 0.42, 0.36, 0.771 m、roll −0.05°、pitch +0.40° に変わり、localizer の再生はブランチの `static_tf_launch.py` を使う。参考に、旧 campus・新しい TF・Hi-Res に補正した bag を `ndt_replay_compare.py` で回すと、記録との xy の差の中央値は 0.063 m で、閾値（0.02 m）を超えた（[campus-m8 の README](maps/campus-m8/README.md) 4 節の実行 a）。補正していない基準 bag と新しい TF の組み合わせは測っていない。

さらに、デモの既定 site は campus-m8 になったが、再生は旧 campus のまま（基準 bag の `/pcl_pose` が旧地図の座標で、`replay_check.py` は地図間の変換をしない）。campus-m8 の基準は、屋外の受入（#156）の走行の bag で取る。それまで `demo_regression.sh --level replay` は main で失敗する。

failsafe の項目は TF に依存しない。bag に記録した `/scan`・`/pcl_pose`・`/alignment_status` を `failsafe_node` に流すだけで、`failsafe_node.py` は TF を参照しない（`/scan` は記録時に作られたもの）。基準を取り直すまでは、failsafe だけを確かめる：

```bash
cd ~/whill_lab0_ros2
scripts/demo_regression.sh --level replay --only failsafe
```

## 基準を更新するとき

意図して結果が変わる変更では、再生が失敗するのが正常。例えば LiDAR の校正と TF の修正、地図の作り直し（#144、#145、#153）では、記録された姿勢との差が大きくなる。

1. `scripts/demo_regression.sh --level replay --measure` を複数回実行して測る。
2. `baseline.json` の値を変え、PR に変わった理由と測定結果を書く。
3. 屋外で確かめる（段階3）。再生が通るだけでは、新しい値の正しさは保証されない。
