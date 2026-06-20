# 旧 M5-b マップディレクトリ リネーム記録 (2026-06-21)

Language: [日本語](2026-06-21-m5b-maps-renamed.md) | [English](../../en/legacy-findings/2026-06-21-m5b-maps-renamed.md)

## 何を / なぜ / いつ

2026-06-21、Issue #47 (M5R-5: `docs/maps/<site>/` 成果物規約の確立) で、
旧 `docs/m5-maps/` ディレクトリを `docs/maps/lab-legacy-m5b/` にリネームし、
新規約 `docs/maps/README.md` を確立した。

### 経緯

M5-R (`docs/ja/plans/2026-06-21-m5r-execution.md`) で
「マップパイプライン (bag → SLAM → ERASOR → 占有格子) の最終出力先」として
`docs/maps/<site>/` 規約を導入することが親方針 §6 受け入れ基準 (3) で確定した。
旧 `docs/m5-maps/` は M5-b (2026-05 期、親方針で凍結された M5-d / e より前) の
試作成果物を保持していたが、

- ディレクトリ名規約 (旧: `docs/m5-maps/`、新: `docs/maps/<site>/`) が衝突する
- 旧 M5-b 試作品は新規約のメタデータ要件 (`metadata.yaml`、SLAM 識別、ERASOR
  パラメータ、commit SHA 等) を満たしていない
- かつ新規約の `lab-loop` site を取得・配置する際、名前空間が紛らわしくなる

ため、旧 M5-b 成果物を「凍結前の試作品である」と明示する形に整理した。
削除ではなくリネームにしたのは、`velodyne_whill.yaml` と `nav_launch.py` が
現時点で旧パスを直接参照しており、削除すると active config が壊れるため。
M5R-7 (#51) で新規約のパスに向け直した時点で、`lab-legacy-m5b/` 自体は削除
候補となる。

### 物理操作

```bash
git mv docs/m5-maps/lab.pgm  docs/maps/lab-legacy-m5b/lab.pgm
git mv docs/m5-maps/lab.yaml docs/maps/lab-legacy-m5b/lab.yaml
mv     docs/m5-maps/lab.pcd                    docs/maps/lab-legacy-m5b/
mv     docs/m5-maps/global_2026-06-04_10min.pcd docs/maps/lab-legacy-m5b/
rmdir  docs/m5-maps
```

`lab.pcd` は M5-b 期の commit `6d8b299` で一度 tracked になったが、その後
`.gitignore` への追記 (`docs/m5-maps/*.pcd`) で git tracking 対象外化されていた。
よって本リネームでは作業ツリー上の `mv` で物理移動するのみで、git の履歴には
「`docs/m5-maps/lab.pcd` 削除 + `docs/maps/lab-legacy-m5b/lab.pcd` 新規 (gitignored)」
として現れる。新パス側は `.gitignore` の `docs/maps/**/*.pcd` ルール (本 Issue
で旧 `docs/m5-maps/*.pcd` 行と統合) によって引き続き ignored。

`global_2026-06-04_10min.pcd` は最初から untracked のため、git には何の変化も
発生しない (作業ツリーの物理移動のみ)。

### 影響と追従

active コード / 設定の 3 箇所が旧パスを参照していたため、追従更新を行った:

- `src/whill_localization/config/velodyne_whill.yaml:24` の `map_file_path`
- `src/whill_navigation/launch/nav_launch.py:55` の `default_map_yaml`
- `src/whill_navigation/config/nav2_params.yaml:212` のコメント (`map_server` の
  `yaml_filename` 注記)

旧パスを残している箇所は次の通り、いずれも歴史的 narrative なので意図的に
更新していない:

- `docs/{ja,en}/session-2026-05-08.md` (M5-b 当時の session log)
- `docs/{ja,en}/m5-navigation.md` (M5 マイルストーン記録)
- `docs/{ja,en}/plans/2026-06-21-m5r-execution.md` §3.2 / §M5R-5 (本リネームを
  予告する計画書。「予告 → 実施」の関係を残すため計画書自身の予告部分は
  変更しない)
- `scripts/pcd_to_occupancy_grid.py:22-23` (旧 M5-b 期のスクリプト docstring。
  Issue #50 / M5R-6 で新スクリプトに置き換える計画のため、現スクリプトの
  docstring 例を本 Issue では触らない)

### `lab-legacy-m5b/` 自体の今後

- 短期: `velodyne_whill.yaml` と `nav_launch.py` の legacy path 解決を保つ
- M5R-7 (#51) で新規約 (`docs/maps/<site>/...`) のパスに向け直し完了 → 旧
  legacy 参照消滅 → `lab-legacy-m5b/` 自体を削除候補に
- 削除実施時は別 Issue で「`docs/maps/lab-legacy-m5b/` を物理削除する」と
  単独で実施し、本リネーム記録は履歴として残す

## 関連

- 開発方針: [`../plans/2026-06-11-platform-pivot.md`](../plans/2026-06-11-platform-pivot.md)
  §6 (3) 受け入れ基準
- M5-R 実行計画: [`../plans/2026-06-21-m5r-execution.md`](../plans/2026-06-21-m5r-execution.md)
  §3.2 (旧 M5-b 残骸の扱い)、§M5R-5
- 新規約: [`../../maps/README.md`](../../maps/README.md)
- 関連 Issue: #47 (本 Issue、M5R-5)、#50 (M5R-6)、#51 (M5R-7)
