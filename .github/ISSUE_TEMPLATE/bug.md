---
name: バグ報告 (動かない / 期待と違う)
about: 観測された現象を debugger が引き継げる形に翻訳する。再現できないバグは修正できない。
labels: 'type:fix'
---

## 観測された現象

(1-3 行。「動かない」ではなく、観測可能な事実として書く。
例: 「/Odometry の publish が 30 秒で停止し、fastlio_mapping プロセスは生存しているが No Effective Points! を吐き続ける」)

## 再現手順

```bash
# 起動コマンドと操作手順を上から順に
```

再現性: 100% / 間欠的 (約 X%) / 単発のみ

## 期待動作

(1-3 行。本来こう動くはず、という基準)

## 環境

- ROS distro: (例: humble)
- ハードウェア関与: (WHILL / Velodyne / RealSense / IMU のうち該当するもの。bag 再生なら bag のパスと録画条件)
- コミット SHA: (`git rev-parse HEAD` の出力)
- launch コマンド: (`ros2 launch ...` の完全な文字列)

## 関連フェーズ

`docs/plans/2026-06-11-platform-pivot.md` のフェーズ (M4-R / M5-R / M6-R / M7 / M8 / M9 / chore) と、
該当する問題 ID (P1〜P5) があれば併記する。

## 仮説 (任意)

報告者が思い当たる原因があれば 1-3 個列挙する。空でもよい (debugger agent が引き継ぐ)。
