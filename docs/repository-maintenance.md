# 未解決事項と制約

## 未解決事項

課題の正本は [GitHub Issues](https://github.com/Iruazu/whill_lab0_ros2/issues)。順番は [M8 ロードマップ](plans/2026-10-30-roadmap-m8.md) を見る。この文書には一覧を持たない（以前ここにあった項目は #139、#168〜#171 に移した。DDS の項目は 2026-10-30 に bashrc を直して解消した）。

## 実装の制約

- failsafe の初回未受信 watchdog は、既定ではエラーを出すだけで、入力が無いときの起動を止めない。`startup_grace_sec` > 0 で停止に切り替えられるが、既定を変えるかは未決（#159、[whill_safety](../src/whill_safety/README.md)）
- Layer D は前方 ±30°・1〜2 m の点を検知する。人の分類・後方・全周の監視はない
- safety のゼロ指令も velocity_smoother を通る。停止距離は実機で確かめる
- dispatch に認証・永続キュー・複数車両管理はない
- 手動救出中に safety が優先して止まることは、実機の証跡がまだない
- 既定では、Layer D の作動中は teleop による後退・回転の救援もできない。Layer D 単独の停止中に後退・回転を failsafe が中継する `layer_d_rescue_enabled` があるが既定は無効で、採否は ADR-0013（提案）で決める（#161）
- 静的 TF には旧校正・概算の値がある。機器を付け直したら現物と校正台帳を照合する
