# `<site-name>` (テンプレート)

> このファイルは `docs/maps/_template/` 配下のテンプレート。新しい site を
> 作るときに `cp -r docs/maps/_template docs/maps/<site-name>` で複製し、
> 以下の placeholder を全て埋めてから commit する。`<...>` が残ったまま
> commit しないこと。

## 概要

- **site 名**: `<site-name>` (英小文字ハイフンつなぎ。例: `lab-loop`,
  `utsunomiya-campus-east`)
- **取得日**: `<YYYY-MM-DD>`
- **経路**: `<route summary in 1 sentence>` (例: 研究室 2F NE 角を始終点とする
  50 m ループ)
- **採用 SLAM**: `<glim | fast_lio_sam>`
- **動的除去**: `<erasor | none>` (none は屋内・無人取得など歩行者が映っていない
  bag の場合のみ)

## 構成ファイル

| ファイル | 役割 | tracked? |
|---------|------|:--------:|
| `static.pcd` | ERASOR 後の静的 PCD | no (gitignored) |
| `occupancy.pgm` | 2D 占有格子 | yes |
| `occupancy.yaml` | Nav2 map_server メタデータ | yes |
| `metadata.yaml` | 取得経緯・パラメータ・出典 | yes |
| `README.md` | 本ファイル (per-site メモ) | yes |

## 特記事項

`<取得時の特殊条件、既知の地図品質課題、後続フェーズが踏みやすい罠などを
書く。例: 「2 階廊下の窓ガラスが LiDAR を抜けて反射が薄い」「中央階段付近に
ループクロージャ後も 30cm 程度の壁残像あり」など>`

## 再生成手順

`<このマップを再生成するときの bag → SLAM → ERASOR → occupancy までの
シェルコマンド。M5R-7 (Issue #51) で `docs/ja/m5r-pipeline.md` が確定したら
そちらを参照する形に置き換える>`

## 関連

- 規約: [`../README.md`](../README.md)
- 元 bag: `docs/m5r-bench-data/<YYYY-MM-DD>-<run-id>/` (M5R-7 で規約確定)
