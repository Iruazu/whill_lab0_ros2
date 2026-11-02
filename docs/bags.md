# bag の置き場

走行 bag は `~/bags/` に置く。`docs/` には置かない。方針は [#139](https://github.com/Iruazu/whill_lab0_ros2/issues/139) で決めた。

## 理由

- `docs/` の bag は git 管理外なので、worktree から見えず、スクリプトが main 側を探す必要があった。
- git の整理の操作（`git clean` など）で失う恐れがあった。

## 配置

| 場所 | 中身 |
| --- | --- |
| `~/bags/m*-bench-data/` | `docs/m*-bench-data/` から移した bag。下の階層は移動前と同じ |
| `~/bags/<用途>_<日付>/`、`~/bags/<日付>/` | 2026-10 以降の記録（`sensor_check_20261028` など） |
| `docs/m*-bench-data/` | README、解析結果、GLIM と DUFOMap の出力。ここには bag を置かない |
| `docs/m6r-bench-data/2026-07-12-acceptance-campus/bag/` | 例外。DDS の不具合の証跡として git で管理する 32 KB の bag。移さない |

別の場所を使うときは、`WHILL_BAGS_ROOT` で置き場を指定する（`scripts/move_bags.py` と `scripts/demo_regression.sh` が読む）。

## パスの読み替え

過去の記録、`run.log`、地図の `metadata.yaml` の `source_bag` は、当時の `docs/m*-bench-data/...` のまま残している。今の場所は `docs/` を `~/bags/` に読み替える。

```text
docs/m5r-bench-data/2026-07-10-campus-outer-final/bag
~/bags/m5r-bench-data/2026-07-10-campus-outer-final/bag
```

移した bag の一覧は [`bags-manifest.tsv`](bags-manifest.tsv) にある。列は移動前のパス、`~/bags/` からのパス、大きさ、ファイル数、長さ、メッセージ数、記録の開始時刻、形式。

## 新しく記録するとき

日付の入ったディレクトリを `~/bags/` に作る。

```bash
ros2 bag record -o ~/bags/$(date +%Y-%m-%d)-<run-id> /velodyne_points /imu/data_rep145 /tf_static
```

## 移動の手順

`scripts/move_bags.py` は一覧にある bag だけを移す。削除はしない。

```bash
scripts/move_bags.py          # 確認だけ。何も動かさない
scripts/move_bags.py --apply  # 移す
scripts/move_bags.py          # 全部「移動済み」になり、問題が 0 であることを確かめる
```

- 移動前に全 bag の大きさとファイル数を一覧と照合し、1本でも合わなければ1本も動かさない。
- `rename` で移すのでコピーは発生せず、同じディスク内なら一瞬で終わる。別のディスクへは移さず、エラーで止まる。
- 途中で止まっても、もう一度実行すれば続きから移せる。
