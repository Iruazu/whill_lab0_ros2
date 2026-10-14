# 地図成果物の管理

地図は `docs/maps/<site>/` に置く。2026-10-14 に実装の参照先とcampus成果物を照合。地図の再生成手順と実機運用手順を区別する。

## runtimeが読むファイル

| ファイル | 参照するもの |
| --- | --- |
| `static.pcd` | m6r bringupのNDT localizer。大容量のためGit対象外 |
| `occupancy.yaml` | Nav2のvariant無指定時 |
| `occupancy_<variant>.yaml` | Nav2の `map_variant:=<variant>` 指定時 |
| YAMLの `image` | map_serverが読む画像。YAMLからの相対パス |
| `waypoints.yaml` | dispatchの地点一覧 |
| `metadata.yaml` | 取得・生成の出典記録。runtimeによる必須検証はない |

現用campusデモは `occupancy_v2.yaml` → `v2/final.pgm`。Web背景は別成果物の `src/whill_dispatch/web/map.png` / `map_meta.json` であり、地図変更時に再生成する。

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

点群地図、占有格子、地点は同じ座標系・原点を使う。Nav2のvariant、Web画像のmeta、地点を揃える。地図生成には [スクリプト一覧](../ja/scripts.md) を参照し、入力と出力先を確認する。

v2の人手maskは編集成果物として保持する。PCD・bag・再生成キャッシュとGit管理する画像を混同しない。旧地図は根拠なく削除しない。

- [campus地図](campus/README.md)
- [新規siteのテンプレート](_template/README.md)
- [旧試作地図](lab-legacy-m5b/README.md)
- [v2塗り手順](v2-paint-guide.md)
