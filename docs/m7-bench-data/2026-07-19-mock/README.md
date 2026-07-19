# M7 whill_dispatch — mock 検証エビデンス (2026-07-19)

計画: [`docs/ja/plans/2026-07-19-m7-dispatch.md`](../../ja/plans/2026-07-19-m7-dispatch.md)

実機なし。`scripts/mock_navigate_to_pose.py` (理想化された NavigateToPose
action server) を相手に、配車境界 (`whill_dispatch`) の全 AC を実測した。
実 Nav2 / localizer / WHILL に対する最終確認は 2026-07-20 の実機検証
(計画 §引き渡し) に送る。

## 起動構成

```
ros2 launch whill_dispatch dispatch_launch.py use_mock:=true
```

これで rosbridge_websocket (+ rosapi, 9090) + dispatch_node + 静的 UI
サーバ (8000) + mock action server が 1 コマンドで立ち上がる。

`ros2 node list`:

```
/dispatch_node
/mock_navigate_to_pose
/rosapi
/rosapi_params
/rosbridge_websocket
```

ポート: `ss -ltnp` で 9090 (ws) と 8000 (http) が LISTEN。

## AC 実測結果

| AC | 内容 | 結果 | 観測値 |
|----|------|:----:|--------|
| 1 | build + `--show-args` が import 段で落ちない | PASS | build 0 warn、`--show-args` が 6 引数を列挙 |
| 2 | `use_mock:=true` で全ノード起動、node list に dispatch_node/rosbridge_websocket | PASS | 上記 node list |
| 3 | `/dispatch/waypoints --once` が地点リストを JSON で返す | PASS | 4 点 (kounoken/gate/bldg7/library) の name/label/x/y/yaw |
| 4 | submit → phase IDLE→QUEUED→ACTIVE→SUCCEEDED、progress 単調増加 | PASS | 下記タイムライン、progress 0.0→1.0 単調 |
| 5 | ACTIVE 中に `/dispatch/cancel` → phase CANCELED | PASS | service success=true、phase CANCELED 到達 |
| 6 | rosbridge 越し E2E (ブラウザ + ws protocol) | PASS | 下記スクショ + ws smoke |
| 7 | ACTIVE 中に 2 件目 submit → QUEUED で待ち、完了後 auto-ACTIVE | PASS | 下記キュー タイムライン |

### AC4 phase / progress (CLI, `ros2 topic pub /dispatch/submit`)

```
phase: IDLE -> QUEUED -> ACTIVE -> SUCCEEDED
progress: 0.0, 0.0, 0.033, 0.1, ... 0.9, 0.967, 1.0   (単調非減少)
```

QUEUED は submit と goal 受理の間の短命な状態。5 Hz 定期 publish では
取りこぼしうるため、dispatch_node は phase 遷移時に即時 publish する
(`_set_phase`)。これで単発 submit でも QUEUED が観測できる。

### AC7 キュー (2 件連続 submit: gate → library)

`/dispatch/state` の (phase, waypoint, queue_len) を時系列で:

```
QUEUED    gate     0
ACTIVE    gate     0
ACTIVE    gate     1     <- library を submit、gate 実行中に待ち行列 1
SUCCEEDED gate     1
QUEUED    library  0     <- gate 完了で library が自動で ACTIVE 化へ
ACTIVE    library  0
SUCCEEDED library  0
```

`queue_len` は「待機中」のみを数える (実行中の 1 件は job_id/waypoint/phase
側で報告)。

### AC6 rosbridge websocket E2E

**CLI proxy** (`scripts/m7_ws_smoke.py --cancel`、ブラウザと同じ ws
protocol を headless で叩く):

```
PASS ws waypoints: ['kounoken', 'gate', 'bldg7', 'library']
PASS ws goto: phases=['QUEUED', 'ACTIVE', 'SUCCEEDED'] max_progress=1.00
PASS ws cancel: reached CANCELED
ALL WS SMOKE CHECKS PASSED
```

**ブラウザ** (`google-chrome` headless、CDP で実時間待機後にキャプチャ):

- `ui_connected_waypoints.png` — 接続済み、map 背景 (campus 外周ループ) +
  4 地点マーカ + ラベル、目的地ドロップダウンに全地点、選択マーカが黄色
- `ui_active_progress.png` — 図書館前へ配車中。状態 ACTIVE、progress バーが
  部分的に伸長、キャンセルボタンが活性

スクショは 1280x900 を 0.7 倍・最適化した縮小版 (各 ~120 KiB)。

## 注意 (mock の割り切り)

mock feedback は `distance_remaining` を線形に減らす理想化。実 Nav2 の
feedback は非単調・replan で分母が変わりうる。dispatch_node 側は D0 を
「最初の正の distance_remaining」で固定し progress を直近最大で保持する
ので逆行はしないが、分母算出の妥当性は 2026-07-20 の U1 実測で確認する。

waypoints.yaml の x/y/yaw は**プレースホルダ**。現地で各地点に WHILL を
運び `/pcl_pose` を実測して差し替える (計画 §引き渡し U2)。
