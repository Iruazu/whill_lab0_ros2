# whill_dispatch

M7 dispatch (配車) API boundary between the Web/tablet UI and Nav2.

One node (`dispatch_node`) sits between the tablet browser and Nav2 and
owns the four responsibilities the platform-pivot plan §3.5 assigns to
`whill_dispatch`, collapsed into a single process for the demo:

- resolve named waypoints (`docs/maps/<site>/waypoints.yaml`)
- queue dispatch jobs (FIFO, one at a time, cancelable)
- drive Nav2 as a `NavigateToPose` action client
- fold vehicle pose + queue state into `/dispatch/state`

The Web side never touches `/navigate_to_pose` or `/cmd_vel*`
(platform-pivot §5 #4). It speaks only the four `/dispatch/*` interfaces
over rosbridge, all standard-typed (JSON-over-`std_msgs/String` +
`std_srvs/Trigger`, no custom rosidl interface — ADR-0012 choice A for the
demo).

Governing plan: [`../../docs/ja/plans/2026-07-19-m7-dispatch.md`](../../docs/ja/plans/2026-07-19-m7-dispatch.md).
Interface decision: [ADR-0012](../../docs/decisions/0012-dispatch-web-interface.md).

## Web boundary (over rosbridge, port 9090)

| dir | name | type | payload |
|-----|------|------|---------|
| Web→ROS | `/dispatch/submit` (topic) | `std_msgs/String` | JSON `{"waypoint":"<name>","type":"goto"\|"recall"}` |
| Web→ROS | `/dispatch/cancel` (service) | `std_srvs/Trigger` | cancel the active job |
| ROS→Web | `/dispatch/state` (topic, 5 Hz) | `std_msgs/String` | JSON `{job_id,phase,waypoint,progress,queue_len,pose,aligned}` |
| ROS→Web | `/dispatch/waypoints` (topic, 1 Hz) | `std_msgs/String` | JSON `[{name,label,x,y,yaw}]` |

`phase ∈ IDLE / QUEUED / ACTIVE / SUCCEEDED / ABORTED / CANCELED`.

Both ROS→Web topics are re-published on a timer (not latched): roslibjs
subscribes volatile, so periodic resend makes UI attach order irrelevant.
`dispatch_node` also pushes a `/dispatch/state` snapshot the instant the
phase changes, so a short-lived QUEUED is never dropped between two 5 Hz
samples. `type` is carried through but `goto` and `recall` behave
identically today (the branch is for post-demo work).

## What dispatch_launch.py launches

`ros2 launch whill_dispatch dispatch_launch.py` starts:

- `rosbridge_websocket` + `rosapi` (port 9090) — the UI's websocket. Skip
  with `use_rosbridge:=false` if rosbridge already runs elsewhere.
- `dispatch_node` — queue / action-client / state node. Reads
  `docs/<site>/waypoints.yaml` (resolved at launch time from `site:=`).
- `python3 -m http.server 8000` (cwd = installed `web/`) — serves the
  static UI. Open `http://<host>:8000` on the tablet.
- (`use_mock:=true` only) `scripts/mock_navigate_to_pose.py` — a stand-in
  `/navigate_to_pose` so the whole boundary runs with no robot.

Arguments: `site` (default `campus`), `use_mock` (default `false`),
`use_rosbridge` (default `true`), `rosbridge_port` (9090), `http_port`
(8000), `action_name` (`/navigate_to_pose`).

## Demo operation (three terminals)

dispatch adds no TF and no `/cmd_vel*`, so it runs alongside the bringup
launches without touching their mutual-exclusion tree — it only opens
ports 9090 / 8000 and a NavigateToPose action client.

```bash
# A — sensors + odom + localizer + failsafe + twist_mux
ros2 launch whill_safety     m6r_bringup_launch.py site:=campus
# B — Nav2 lifecycle + obstacle layer
ros2 launch whill_navigation nav_launch.py site:=campus map_variant:=cleaned
# C — dispatch boundary + Web UI (real Nav2, no mock)
ros2 launch whill_dispatch   dispatch_launch.py use_mock:=false
```

Then open `http://<host>:8000` on a tablet on the same LAN (page and ws
are both plain http/ws on the same host to avoid mixed-content blocking).

## HTTPS / iPad (use_tls:=true)

iOS Safari's HTTPS-First upgrades `http://<host>:8000` to `https://`, and
the plain `http.server` then 400s the TLS handshake — iPad cannot open the
UI over plain http (confirmed 2026-07-20 field). Serve over HTTPS/WSS:

```bash
# 1) once: self-signed cert with the AP IP in the SAN (default 10.42.0.1)
scripts/m7_make_tls_cert.sh 10.42.0.1

# 2) launch with TLS (UI over https:8000, rosbridge over wss:9090)
ros2 launch whill_dispatch dispatch_launch.py use_mock:=false use_tls:=true
```

On the iPad, connect to the `whill-demo` AP, then **first** trust the cert,
**then** open the UI:

1. Safari → `https://10.42.0.1:8000/dispatch.crt` → 設定 → プロファイルが
   ダウンロードされました → インストール
2. 設定 → 一般 → 情報 → 証明書信頼設定 → whill-dispatch を全面的に信頼
3. Safari → `https://10.42.0.1:8000` — 地図と「接続済み」が出れば成立

`app.js` picks `wss://` automatically when the page is https, so no UI edit
is needed. The `.crt` is staged into the served dir at launch; the private
key never leaves `~/.whill_dispatch_tls`.

## Mock operation (no robot)

```bash
ros2 launch whill_dispatch dispatch_launch.py use_mock:=true
```

CLI checks against the mock:

```bash
ros2 topic echo /dispatch/waypoints --once
ros2 topic pub --once /dispatch/submit std_msgs/String \
  '{data: "{\"waypoint\":\"gate\",\"type\":\"goto\"}"}'
ros2 topic echo /dispatch/state          # phase walk + progress
ros2 service call /dispatch/cancel std_srvs/srv/Trigger
```

Websocket E2E (same protocol the browser uses, headless, dependency-free):

```bash
python3 scripts/m7_ws_smoke.py --cancel
```

The mock's timing is tunable via env (`MOCK_NAV_DURATION_S`,
`MOCK_NAV_START_DIST_M`, `MOCK_NAV_FEEDBACK_HZ`).

## Waypoints

`docs/maps/<site>/waypoints.yaml` — an ADR-0005 (maps spec) extension. map
frame metres, same origin basis as `occupancy_cleaned.yaml`. As of
2026-07-19 the coordinates are **placeholders**; they get replaced with
per-point `/pcl_pose` measurements on 2026-07-20 (plan §引き渡し U2).

## Web map background

`web/map.png` + `web/map_meta.json` are generated from the operative
campus map (6640x6295, ~41 MB — too heavy to serve raw). Current source is
the v2 map (`map_meta.json` の `source_pgm` が正):

```bash
python3 scripts/m7_make_web_map.py \
    --pgm docs/maps/campus/v2/final.pgm \
    --yaml docs/maps/campus/occupancy_v2.yaml \
    --out-dir src/whill_dispatch/web
```

Idempotent; re-run whenever the operative map changes (keep it in sync
with the `map_variant` used at demo time). `app.js` reads
`map_meta.json` (origin + effective resolution) and converts map-frame
metres to png pixels with the occupancy-grid transform (origin at the pgm
bottom-left, y flipped for image coordinates).

## Package layout note

This package is ament_cmake + ament_cmake_python (mirroring whill_safety),
not ament_python as the plan's scaffold section named. Every package in
this repo is ament_cmake; `ament_cmake_python` installs the pure-python
`dispatch_node` identically, and the plan's stated reason for ament_python
("avoid the rosidl build") is already met because there are no custom
interfaces. Rationale is in `CMakeLists.txt`.

## Scope / caveats

- Demo scope: no auth, no multi-vehicle, no added safety. The demo is run
  with an operator beside the chair whose WHILL joystick bypasses this
  path (safety stays with the existing failsafe / twist_mux, unchanged).
- The single-node collapse is deliberate for the demo. The M7 full build
  splits it into gateway / job-manager / state-publisher (responsibilities
  are already separated by function in `dispatch_node.py`).
- mock feedback is idealised (linear `distance_remaining`). Real Nav2
  feedback is non-monotonic; progress math holds a running max so the bar
  cannot run backward, but the denominator (D0) choice is confirmed on the
  chair on 2026-07-20 (plan §引き渡し U1).
- A submit for an unknown waypoint name is dropped with a warn log only;
  nothing appears on `/dispatch/state`. The bundled UI cannot produce one
  (its list comes from `/dispatch/waypoints`), so the boundary favours
  simplicity over error reporting for now.
- Queue state lives only in dispatch_node memory. If the node dies or is
  restarted mid-job, the queue is lost while the Nav2 goal keeps running
  unsupervised — cancel it via the operator (joystick override / Nav2
  cancel) before restarting dispatch.
- `web/vendor/roslib.min.js` is vendored from roslibjs 1.4.1 (BSD
  3-Clause); provenance and license text in `web/vendor/LICENSE.roslibjs`.
