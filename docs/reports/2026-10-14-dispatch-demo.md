# 2026-10-14 統合起動・実機配車の確認

## 条件

- 実行PC: systemlab-Alienware-x15-R2
- 起動: `scripts/start_dispatch_demo.sh`（既定: campus、v2、normal、domain 0、TLS）
- 初期位置: RVizで指定後、起動ターミナルでEnter
- ログ: `log/dispatch-demo-20261014-140220-LhNDRE/`（成功した起動）、`log/dispatch-demo-20261014-135842-WRMVhW/`（直前の起動）。`log/` は Git 管理外で、main 側の作業ディレクトリにある

## 結果と根拠

ユーザーから「起動完了し、7号館から工農研究所までの配車成功」との報告があった。

14:02:20 起動のログでもNav2の `Managed nodes are active`、controllerの `Reached the goal!`、bt_navigatorの `Goal succeeded`、dispatchの `job 1 SUCCEEDED` を確認した。統合起動と、実機の配車ジョブ完了を確認できた。

### 報告とログの向きの食い違い（#170、2026-11-12 にログで照合）

報告の向きは「7号館から工農研」だが、成功したジョブは逆向きの「工農研の近くから7号館（`bldg7`）」だった。配車ジョブを出した2回の起動が、報告では1つにまとまっていると見られる（13:44:10 と 13:47:22 の起動もあるが、dispatch.log が無くジョブは出ていない）。

| 起動 | ログ | 開始位置（bt_navigator） | ジョブ | 結果 |
| --- | --- | --- | --- | --- |
| 13:58:42 | `log/dispatch-demo-20261014-135842-WRMVhW/` | (-38.67, 0.54)。`bldg7` から約 1.0 m | `kounoken`（0.00, 0.00） | 14:00:10 に送信。38 秒後に RPP の `collision ahead` が2回出て、復帰動作（wait）中の 14:00:56 に `/dispatch/cancel`（Web の取消ボタンなどが呼ぶ。呼び出し元はログで区別できない）で取り消された（`CANCELED`）。到着していない |
| 14:02:20 | `log/dispatch-demo-20261014-140220-LhNDRE/` | (-3.50, -0.46)。`kounoken` から約 3.5 m | `bldg7`（-38.42, -0.47） | 14:03:19 に送信、14:04:43 に `SUCCEEDED` |

- 報告の「7号館から工農研究所まで」は1回目の向きと一致する。1回目は到着していない。
- 速度の上限は normal の 0.45 m/s なので、送信から取消までの 46 秒で、1回目は最大でも約 20.7 m しか進めない。2回目の開始位置までは約 35 m ある。起動の間に、車体を工農研側へ手で移したと見られる（記録は無い）。
- 地点名、座標、Web の表示は食い違っていない。Web の表示名は `docs/maps/campus/waypoints.yaml` の `label` を dispatch_node が配信したもので、UI に地点名の直書きは無い。`kounoken` の表示名は「工農研横」、`bldg7` は「7号館前」。`waypoints.yaml` は 2026-07-22 から変わっていない。
- 地点名と物理位置の対応の実機での確認は #155 で行う。このログで確かめたのは、2回目の起動で `bldg7` 宛てのジョブが成功したことだけ。

## 終了時と確認範囲

ユーザーはCtrl-Cで終了した。終了時のNav2取消応答はreturn_code 0、取消対象なし。停止ログにはrosapiとimu_sign_correctorの `ExternalShutdownException` が残る。終了後の全プロセス消失・直後の再起動成功は今回未確認。

この記録は1回の配車成功であり、preflight全項目・手動中のLayer D停止・停止距離・他経路の受入完了を意味しない。
