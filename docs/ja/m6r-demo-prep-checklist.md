# M6-R デモ準備チェックリスト

Language: [日本語](m6r-demo-prep-checklist.md) | [English](../en/m6r-demo-prep-checklist.md)

デモ本番前に**現地で**実施する運用手順。コード側で扱えない
経路依存の問題を、事前踏査と物理的な経路整備で吸収するためのリスト。

デモ形態が変わったら (会場変更、季節変更、コース変更) 再走査すること。
現時点の適用範囲は M6-R 統合デモ (キャンパス外周)。

## 経路整備

### 背の高い雑草の刈り取り / 迂回

- **理由**: `min_height = 0.05 m` (ADR-0009 accepted) と Patchwork++ 地面
  除去 (ADR-0011 accepted) の組み合わせでは、地面から立ち上がる形状は
  すべて obstacle として lethal 化される。人と雑草を分離する情報がない
- **手順**:
  - デモ 1-2 日前にコースを踏査、腰高 (~50 cm) 以上の雑草群を特定
  - 経路 lethal 化を避ける方法: (a) 刈り取り (b) map annotation で
    迂回経路を敷く (c) `raytrace_max_range` 内で回り込ませる
- **記録**: 対応済み雑草 patch の位置と対応方法を `docs/m6r-bench-data/
  <demo-date>-<site>/route-grooming.md` に残す

### 5 cm 級の路面段差

- **仕様上の扱い**: 検出対象外 (ADR-0009 §結果)。車体走破可
- **踏査項目**: WHILL の走破可能限界を超える段差 (~10 cm 以上) が新規に
  現れていないかを目視確認。ある場合は map annotation で迂回路を用意

## デモ当日の起動前チェック

環境前提は [CLAUDE.md §ランタイム環境の前提](../../CLAUDE.md) を参照
(RMW, CPU governor, sysctl, NVIDIA suspend fix)。ここでは M6-R 統合
デモに固有の項目のみを列挙する。

- [ ] `map -> odom -> base_link` の TF chain が 1 本鎖 (`ros2 run
      tf2_tools view_frames`)
- [ ] `/alignment_status.has_converged: true` かつ `fitness < 1.0`
      (静止状態、初期位置合わせ後)
- [ ] `/scan` の publisher count = 1 (velodyne_laserscan の
      `/scan_raw` remap 有効)
- [ ] operator 随伴、ジョイスティック介入可能 (ADR-0007 §Demo-scope
      reduction)

## 関連

- [ADR-0009: p2ls 高さ帯 + QoS bridge](decisions/0009-p2ls-height-band.md)
- [ADR-0011: 地面除去手法選定](decisions/0011-ground-removal-choice.md)
- [ADR-0007: failsafe / twist_mux](decisions/0007-failsafe-design.md)
  §Demo-scope reduction
