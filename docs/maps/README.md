# 地図成果物の管理

地図は `docs/maps/<site>/` に置く。2026-10-14 に実装の参照先とcampus成果物を照合。地図の再生成手順と実機運用手順を区別する。

## runtimeが読むファイル

| ファイル | 参照するもの |
| --- | --- |
| `static.pcd` | m6r bringupのNDT localizer。大容量のためGit対象外 |
| `occupancy.yaml` | Nav2のvariant無指定時（site ごとの既定 variant が無い場合。campus-m8 は `candidate`） |
| `occupancy_<variant>.yaml` | Nav2の `map_variant:=<variant>` 指定時 |
| YAMLの `image` | map_serverが読む画像。YAMLからの相対パス |
| `waypoints.yaml` | dispatchの地点一覧 |
| `metadata.yaml` | 取得・生成の出典記録。runtimeによる必須検証はない |

現用デモの既定 site は `campus-m8`（2026-11-13、#190）で、`occupancy_candidate.yaml` → `v2/candidate_final.pgm`。旧 `campus` は fallback で、`occupancy_v2.yaml` → `v2/final.pgm`。Web背景は別成果物の `src/whill_dispatch/web/map.png` / `map_meta.json`（1組だけ。現在は campus-m8 から作成）であり、地図・site の変更時に再生成する。食い違いは `scripts/check_map_sync.py` で検出する。

既定の site を変えたときに揃えるもの：3 つの launch（`m6r_bringup_launch.py`、`nav_launch.py`、`dispatch_launch.py`）の `_DEFAULT_SITE`、`nav_launch.py` の `_DEFAULT_VARIANT_BY_SITE`、`scripts/start_dispatch_demo.sh` の `SITE` と `demo_variant_for`、`scripts/check_map_sync.py` の `DEFAULT_SITE` と `DEMO_VARIANT_BY_SITE`、`scripts/m7_make_web_map.py` の既定の `--yaml`、Web背景、runbook。回帰チェック（`scripts/regression/`）は基準 bag が旧 campus で記録されているので campus のまま（`baseline.json` の `site_note`）。

launchは既定でこの地図ディレクトリをworkspaceから解決する。配置が異なる場合は `WHILL_MAPS_ROOT` を `site` ディレクトリの親に指定する。

## metadata.yaml スキーマ

既存成果物の記録項目を以下に示す。現在、厳密なスキーマ検証をlaunchが実施するわけではない。

| 項目 | 内容 |
| --- | --- |
| `acquired_at` | bag取得日時とタイムゾーン |
| `route_summary` | 経路・距離・始終点 |
| `weather` | 取得環境 |
| `slam_method` | 使用したSLAM |
| `slam_params` | 使用設定の出典 |
| `erasor_params` | 旧名称の動的除去設定欄。既存campusにも残る |
| `source_bag` | 元bagの保存先 |
| `commit` | 生成版の識別子。既存campusは短縮SHA |

新規記録では実際に使用した動的除去方式・版・引数を明記する。旧フィールド名だけを根拠にERASOR使用と解釈しない。取得時間や品質指標はmanifest・bag・解析結果へ対応付ける。

## 変更時の確認

点群地図、占有格子、地点は同じ座標系・原点を使う。Nav2のvariant、Web画像のmeta、地点を揃える。地図生成には [スクリプト一覧](../scripts.md) を参照し、入力と出力先を確認する。

v2の人手maskは編集成果物として保持する。PCD・bag・再生成キャッシュとGit管理する画像を混同しない。旧地図は根拠なく削除しない。

- [campus地図](campus/README.md)：M5-R の地図。2026-11-13 から fallback（`site:=campus`）
- [campus-m8地図](campus-m8/README.md)：M8-2 で Hi-Res の点群から作り直した地図。2026-11-13 から既定の site（占有格子は手塗りの候補 `candidate`）。屋外の受入（#156）と手塗りの確定（#154）は既定にした後に行う
- [新規siteのテンプレート](_template/README.md)
- [旧試作地図](lab-legacy-m5b/README.md)
- [v2塗り手順](v2-paint-guide.md)
