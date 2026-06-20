# `docs/maps/` — マップ成果物 registry

M5-R パイプライン (bag → SLAM → 動的物体除去 → 占有格子変換) の **最終出力**
を、運用 (M6-R 以降) が消費可能な形で格納する registry。中間アーティファクト
(生 bag、SLAM 直出力 PCD、ERASOR 前後の比較等) は別 directory
`docs/m5r-bench-data/` に置く (Issue #51 / M5R-7 で規約確定予定)。

この README は registry の運用文書なので、`docs/m4r-bench-data/README.md` と
同じく日本語単独とする。i18n 方針 (ADR-0001) は「`docs/{ja,en}/` 配下の
narrative docs は二言語化」を定めるが、`docs/maps/` のような運用 registry は
その対象外。

根拠: 親方針 §6 受け入れ基準 (3) 「`docs/maps/<site>/` に pcd + pgm + yaml +
取得日 / 経路 / 天候のメタデータが揃う」、および M5-R 実行計画 §M5R-5。

## ディレクトリ規約

```
docs/maps/
├── README.md                   (本ファイル)
├── _template/                  (新規 site の出発点。cp -r で複製して使う)
│   ├── README.md               (per-site README 雛形)
│   ├── metadata.yaml           (スキーマ通りのサンプル)
│   └── occupancy.yaml          (Nav2 map_server 互換のサンプル)
├── <site-name>/                (1 site = 1 directory。例: lab-loop,
│   │                            utsunomiya-campus-east)
│   ├── static.pcd              (ERASOR 後の静的 PCD。gitignored)
│   ├── occupancy.pgm           (2D 占有格子 = Nav2 map_server 入力)
│   ├── occupancy.yaml          (Nav2 map_server メタデータ)
│   ├── metadata.yaml           (取得経緯・パラメータ・出典)
│   └── README.md               (このサイト固有メモ。任意)
└── lab-legacy-m5b/             (旧 M5-b 試作品。M5-R 完了時に削除候補)
    ├── lab.pcd                 (gitignored)
    ├── lab.pgm
    ├── lab.yaml
    └── global_2026-06-04_10min.pcd  (gitignored)
```

`<site-name>` は英小文字ハイフンつなぎ。site は「同一の地理的範囲を同一の取得
手順で覆ったマップの単位」と定義する。同じ場所でも取得日や経路が異なるなら
別 site とする (例: `lab-loop-2026-06-22` / `lab-loop-2026-07-15-rain`)。
site 名に日付を入れるかは運用判断。再取得で完全に置き換える方針なら `lab-loop`
のまま上書き、世代を残したいなら日付を含める。

## `metadata.yaml` スキーマ

`docs/maps/<site>/metadata.yaml` は次のフィールドを持つ。スキーマは
`_template/metadata.yaml` も同じ形を取るので、新規 site 作成時はそれを
コピーして埋める。

| フィールド | 形式 | 必須 | 説明 |
|-----------|------|:---:|------|
| `acquired_at` | ISO8601 日時文字列 (例: `2026-06-22T14:30:00+09:00`) | yes | bag を取り終えた時刻。SLAM を回した時刻ではない |
| `route_summary` | 自由記述文字列 | yes | 経路の一行要約。例: "lab 50m loop, start/end at NE corner of room 2F" |
| `weather` | 自由記述文字列 | yes | 取得時天候。屋内なら `"indoor"` と書く (空文字を許さないことでメタデータ不備を発見しやすくする) |
| `slam_method` | `glim` または `fast_lio_sam` | yes | 採用 SLAM の識別子。ADR-0003 (M5R-3 で起案) で確定する選択肢のいずれか |
| `slam_params` | path string | optional | 引用元 yaml への repo 相対パス。例: `src/whill_localization/config/velodyne_whill.yaml` |
| `erasor_params` | mapping または path | optional | ERASOR パラメータ。インライン (voxel size, PR/RR 閾値) か yaml ファイルへの相対パスのどちらか |
| `source_bag` | path string | yes | `docs/m5r-bench-data/...` 配下への repo 相対パス。M5R-7 で `docs/m5r-bench-data/` 規約が確定したらそれに揃える |
| `commit` | 40 文字の SHA | yes | このファイル生成時の repo HEAD。後で「どのパイプライン版で作られた地図か」を遡れるようにする |

任意フィールドを足したくなった場合 (例: 屋外で `gnss_used: true`、屋外拡張で
`utm_zone: 54S`、ループクロージャ誤差 `loop_closure_error_m: 0.42`) は、本 README
の表を更新してから足す。スキーマ拡張の追跡可能性を保つため。

## 新規 site の作り方

```bash
# 1. _template を複製
cp -r docs/maps/_template docs/maps/<site-name>

# 2. metadata.yaml の placeholder を全て埋める。空 / "TODO" のまま commit しない
${EDITOR:-vi} docs/maps/<site-name>/metadata.yaml

# 3. M5-R パイプライン (Issue #50 / #51 で確定) の出力を置く
mv path/to/static.pcd docs/maps/<site-name>/static.pcd
mv path/to/occupancy.pgm docs/maps/<site-name>/occupancy.pgm
mv path/to/occupancy.yaml docs/maps/<site-name>/occupancy.yaml

# 4. occupancy.yaml の image: フィールドが "occupancy.pgm" を指していることを確認
#    (Nav2 map_server は相対パスを yaml ファイル基準で解決する)

# 5. 任意: per-site README をユーザー自身が書く (取得時の特記事項、既知の
#    地図品質課題など)
```

## gitignore 規約

PCD は容量が大きい (数十〜数百 MB) ため、`.gitignore` の
`docs/maps/**/*.pcd` で再帰的に除外する。`.pgm` / `.yaml` / `.md` は tracked。
`_template/` には `static.pcd` placeholder は置かない (gitignored で空の
placeholder は意味がないため。README 内で「ここに置く」と注記するに留める)。

site directory ごと丸ごと再生成する運用 (M5R-7 のパイプラインスクリプトが
出力先を毎回作る) なので、`docs/maps/<site>/.gitkeep` のような空 placeholder は
原則作らない。site directory が `.pgm` / `.yaml` / `.md` のいずれかを持っていれば
git tree に残る。

## `lab-legacy-m5b/` の扱い

`docs/maps/lab-legacy-m5b/` は旧 M5-b フェーズ (2026-05 期、親方針で凍結対象に
含まれる M5-d/e より前) で作られた試作マップ。本規約 (M5-R) とはディレクトリ
構造もメタデータ仕様も合っていないが、現時点で `velodyne_whill.yaml` と
`nav_launch.py` が直接参照しているため即時削除はしない。M5R-7 (#51) で新規約
への向け直しが完了した段階で削除候補となる。

リネーム履歴の詳細は `docs/legacy-findings/2026-06-21-m5b-maps-renamed.md` を
参照。

## 後続フェーズが触る場所

- **M5R-6 (Issue #50)**: `static.pcd` → `occupancy.pgm + .yaml` 変換スクリプト。
  本 README の `_template/occupancy.yaml` をスキーマ参照元として用いる
- **M5R-7 (Issue #51)**: bag 取得 → SLAM → ERASOR → 占有格子変換 → 本 registry
  への格納までを E2E 文書化する。`metadata.yaml` 自動生成 (commit SHA 等) も
  ここで整備
- **M6-R**: scan-to-map localizer (`lidar_localization_ros2` 第一候補) が
  `docs/maps/<site>/static.pcd` を入力に `map -> odom` を publish。Nav2
  obstacle layer が `docs/maps/<site>/occupancy.yaml` を map_server に流す
- **M9 / 屋外拡張**: `metadata.yaml` に GNSS 関連フィールド追加が想定される。
  スキーマ拡張は上記の方針 (本 README の表を更新してから足す) に従う

## 関連

- 親方針: [`../ja/plans/2026-06-11-platform-pivot.md`](../ja/plans/2026-06-11-platform-pivot.md)
  §3.1 (二相分離)、§6 (3) (受け入れ基準)
- M5-R 実行計画: [`../ja/plans/2026-06-21-m5r-execution.md`](../ja/plans/2026-06-21-m5r-execution.md)
  §2.1 (6)、§M5R-5
- i18n 方針 (本 registry 単言語化の根拠): [`../decisions/0001-docs-i18n.md`](../decisions/0001-docs-i18n.md)
- 旧 M5-b → `lab-legacy-m5b/` リネーム記録: [`../legacy-findings/2026-06-21-m5b-maps-renamed.md`](../legacy-findings/2026-06-21-m5b-maps-renamed.md)
