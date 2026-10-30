# 未解決事項と制約

## 未解決事項

| 項目 | 現状と次の確認 |
| --- | --- |
| preflight | check 4 は `/rosout` を 1 件見るだけで、無期限に待つことがある。有限時間で入力状態を直接確かめる形に直す。runbook は外側の `timeout` を併用 |
| DDS 設定 | 現用は `configs/cyclonedds-runtime-min.xml`。旧 `cyclonedds-runtime.xml`（旧 NIC 名）は `configs/legacy/` に移した。`~/.bashrc` の `CYCLONEDDS_URI` が旧パスを指していれば手で直す（bashrc は自動編集しない） |
| 録画データ | M7 の大容量 bag が未追跡一覧に出る。録画本体と共有する記録の置き場を分ける |
| 配車地点 | 2026-10-14 の報告とログ上の地点名が食い違う。実機で対応を確かめる（[検証記録](reports/2026-10-14-dispatch-demo.md)） |
| 終了・再起動 | 終了時に rosapi と imu_sign_corrector の例外ログが出る。全プロセスの消失と連続再起動を確かめる |
| 地図同期 | Web 背景と Nav2 地図の同期は手動。YAML と生成 meta の出典を検査する |

## 実装の制約

- failsafe の初回未受信 watchdog はエラーを出すだけで、入力が無いときの起動を止めない
- Layer D は前方 ±30°・1〜2 m の点を検知する。人の分類・後方・全周の監視はない
- safety のゼロ指令も velocity_smoother を通る。停止距離は実機で確かめる
- dispatch に認証・永続キュー・複数車両管理はない
- 手動救出中に safety が優先して止まることは、実機の証跡がまだない
- 静的 TF には旧校正・概算の値がある。機器を付け直したら現物と校正台帳を照合する
