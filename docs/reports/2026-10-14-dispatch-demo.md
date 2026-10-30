# 2026-10-14 統合起動・実機配車の確認

## 条件

- 実行PC: systemlab-Alienware-x15-R2
- 起動: `scripts/start_dispatch_demo.sh`（既定: campus、v2、normal、domain 0、TLS）
- 初期位置: RVizで指定後、起動ターミナルでEnter
- ログ: `log/dispatch-demo-20261014-140220-LhNDRE/`

## 結果と根拠

ユーザーから「起動完了し、7号館から工農研究所までの配車成功」との報告があった。

ログでもNav2の `Managed nodes are active`、controllerの `Reached the goal!`、bt_navigatorの `Goal succeeded`、dispatchの `job 1 SUCCEEDED` を確認した。統合起動と、実機の配車ジョブ完了を確認できた。

ただしこのログの投入先は `bldg7`（-38.42, -0.47）で、ユーザー報告の物理的な行先と地点名の対応は一致していない。物理配置・地図原点・地点名の対応を再確認するまでは、このログだけで `kounoken` 宛ての成功を断定しない。

## 終了時と確認範囲

ユーザーはCtrl-Cで終了した。終了時のNav2取消応答はreturn_code 0、取消対象なし。停止ログにはrosapiとimu_sign_correctorの `ExternalShutdownException` が残る。終了後の全プロセス消失・直後の再起動成功は今回未確認。

この記録は1回の配車成功であり、preflight全項目・手動中のLayer D停止・停止距離・他経路の受入完了を意味しない。
