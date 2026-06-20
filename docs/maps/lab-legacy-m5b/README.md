# lab-legacy-m5b — 旧 M5-b 試作品の保管庫

これは M5-R が `docs/maps/<site>/` 規約 (`../README.md`) を確立する前に、
旧 M5-b フェーズで試作された PCD / 占有格子の保管庫であり、**新規約には
準拠していない**。

- 起源: M5-b (`run2` 椅子 bag を `fast_lio_mapping` で replay → `/map_save`
  サービスで取得した `lab.pcd`、それを `scripts/pcd_to_occupancy_grid.py`
  で `lab.pgm` + `lab.yaml` に変換)。経緯は `docs/legacy-findings/2026-06-21-m5b-maps-renamed.md`
- 規約不一致: `metadata.yaml` が存在しない、`static.pcd` ではなく `lab.pcd`
  と命名、`occupancy.{pgm,yaml}` の対も `lab.{pgm,yaml}`
- 現役の理由: `src/whill_localization/config/velodyne_whill.yaml` の
  `map_file_path` と `src/whill_navigation/launch/nav_launch.py` の
  `default_map_yaml` が直接参照しているため、本ディレクトリを今削除すると
  FAST-LIO の `pcd_save` と Nav2 `map_server` の CONFIGURE が壊れる
- 廃止予定: M5R-7 (#51) で active 参照先が M5-R パイプライン出力
  (`docs/maps/<site>/static.pcd` と `occupancy.yaml`) に切り替わった時点
  で、本ディレクトリを別 Issue で削除する
