# 退避したスクリプト

現行の手順では使わない。旧地図の出所として残している。試験は無く、既定値は古い（例：`m5r_pcd_to_occupancy.py` の LiDAR の高さは 0.79 m のまま）。現行の占有格子の作成は `scripts/pcd_to_occupancy_v2.py` と `scripts/compose_occupancy.py`。

| スクリプト | 退避の理由（2026-11-27） |
| --- | --- |
| `pcd_to_occupancy_grid.py` | FAST-LIO が保存した PCD 向け（旧 M5-b）。FAST-LIO の runtime は削除済み。`docs/maps/lab-legacy-m5b/` の出所 |
| `m5r_pcd_to_occupancy.py` | v2 に置き換えられた旧版。2026-07 の旧地図（campus、campus-demo、campus-half-v3、campus-outdoor-corrected）の出所 |
