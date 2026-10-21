# whill_dispatch

タブレットWebとNav2を接続する配車ノード・静的Web UI。起動手順は [runbook](../../docs/ja/runbook-live-demo.md)。

## 起動するもの

`dispatch_launch.py` はdispatch_node、rosbridge/rosapi、静的Web配信を起動する。既定はHTTP 8000 / WS 9090、`use_tls:=true` でHTTPS / WSSになる。TLS証明書の既定パスは `~/.whill_dispatch_tls/dispatch.crt` / `dispatch.key`。

`site:=campus` は `docs/maps/campus/waypoints.yaml` を読む。`WHILL_MAPS_ROOT` で親ディレクトリを変更できる。`use_mock:=true` の場合は模擬NavigateToPose actionを追加する。mockは車体や安全層を再現しない。実機構成ではfalse。

dispatchはTFを出さない。手動救出では `/cmd_vel_teleop` に速度指令を出す。

## Webとのインターフェース

| 名前 | 型 | 用途 |
| --- | --- | --- |
| `/dispatch/submit` | `std_msgs/String` topic | 地点名または座標をJSONで投入 |
| `/dispatch/teleop` | `std_msgs/String` topic | 手動モード切替・速度指令 |
| `/dispatch/cancel` | `std_srvs/Trigger` service | 進行中の配車を取消 |
| `/dispatch/state` | `std_msgs/String` topic | 状態JSON。既定5Hz、状態変化時にも配信 |
| `/dispatch/waypoints` | `std_msgs/String` topic | 地点一覧。既定1Hz |
| `/plan` | `nav_msgs/Path` topic | Webで経路を読む。操作指令には使わない |

Webは `/navigate_to_pose` や `/cmd_vel` に直接指令を送らない。地点JSONは `{"waypoint":"kounoken","type":"goto"}`、座標JSONは `{"point":{"x":0.0,"y":0.0,"yaw":0.0},"type":"goto"}`。例は形式の説明であり、走行先の指定手順ではない。`type` はgoto / recallを受け付けるが、現在の処理は共通。pointがあれば地点名より優先する。

配車状態はIDLE / QUEUED / ACTIVE / SUCCEEDED / ABORTED / CANCELED。キューはメモリ内で保持し、同時に1件をNav2へ渡す。再起動でキューは失われ、dispatch停止だけでは既存Nav2 goalの取消を保証しない。

## 地点と背景地図

campusの地点名は `kounoken`、`bldg7`、`seimon`、`bldg3`。座標の正本は `docs/maps/campus/waypoints.yaml`。

Web背景は `web/map.png` / `web/map_meta.json`。現用metaは `docs/maps/campus/v2/final.pgm` を出典として記録し、縮小率8、解像度0.4m/pixel。再生成は `scripts/m7_make_web_map.py --yaml docs/maps/campus/occupancy_v2.yaml --out-dir src/whill_dispatch/web`。

WebのFREE判定は縮小画像上の補助判定。dispatchは地図の占有状態を再検査しない。最終的な経路計画はNav2が行う。画像と地図の同期は自動ではない。

## 手動救出

`{"active":true}` で有効化、`{"vx":0.2,"wz":0.0}` の形式で速度を送る。既定無効。上限は並進±0.3m/s、回転±0.6rad/s。Webは押下中のみ継続送信し、release / cancel / blur等で停止を送る。

dispatchのwatchdogは指令途絶0.4秒でゼロを送り、無操作20秒でモード解除。twist_muxのteleopタイムアウトは0.5秒、優先度はsafety100 > teleop50 > nav10。

teleop入力が切れるとNav2入力に戻り得る。進行中の配車を取消してから救出する。手動中のLayer D停止は実機で未検証。

## 制約と検証

認証、永続キュー、複数車両管理は未実装。この構成は `whill_console` gatewayのトークンを使用しない。未知の地点名などはwarnで拒否され、Webへの詳細エラー返信は限定的。

`m7_ws_smoke.py` はジョブ投入・取消を含むため、読み取り専用の接続確認ではない。実機の接続診断に無条件で実行せず、模擬actionを使う検証環境で使用する。

launch変更時は `ros2 launch whill_dispatch dispatch_launch.py --show-args`。現在の制約は [整備課題](../../docs/ja/repository-maintenance.md) を参照。
