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
(platform-pivot §5 #4). It speaks the four `/dispatch/*` interfaces over
rosbridge, all standard-typed (JSON-over-`std_msgs/String` +
`std_srvs/Trigger`, no custom rosidl interface — ADR-0012 choice A for the
demo). The one exception is a **read-only** subscription to Nav2's `/plan`
(`nav_msgs/Path`) for route visualization: no command flows back, so it is
not an operation boundary. ADR-0012 permits this explicitly (see its `/plan`
addendum).

Governing plan: [`../../docs/ja/plans/2026-07-19-m7-dispatch.md`](../../docs/ja/plans/2026-07-19-m7-dispatch.md).
Interface decision: [ADR-0012](../../docs/decisions/0012-dispatch-web-interface.md).

## Web boundary (over rosbridge, port 9090)

| dir | name | type | payload |
|-----|------|------|---------|
| Web→ROS | `/dispatch/submit` (topic) | `std_msgs/String` | JSON, one of two goal forms (below) |
| Web→ROS | `/dispatch/teleop` (topic) | `std_msgs/String` | JSON `{"active":bool}` (toggle) or `{"vx":<m/s>,"wz":<rad/s>}` (motion) |
| Web→ROS | `/dispatch/cancel` (service) | `std_srvs/Trigger` | cancel the active job |
| ROS→Web | `/dispatch/state` (topic, 5 Hz) | `std_msgs/String` | JSON `{job_id,phase,waypoint,progress,queue_len,pose,aligned,fitness,battery,teleop_active}` |
| ROS→Web | `/dispatch/waypoints` (topic, 1 Hz) | `std_msgs/String` | JSON `[{name,label,x,y,yaw}]` |

`/dispatch/submit` carries exactly one of two goal forms (`type` is
`"goto"\|"recall"` for both, and `point` wins if both are somehow present):

- named waypoint: `{"waypoint":"<name>","type":"goto"}` — resolved via
  `waypoints.yaml`. Unchanged from v1.
- arbitrary map-frame point: `{"point":{"x":<m>,"y":<m>,"yaw":<rad>},"type":"goto"}`
  — the tablet's map-tap goal (v2). `yaw` is optional (defaults 0.0; heading
  control is future work). `x`/`y` are required and validated in
  `dispatch_node` (`_parse_point`): a non-dict, missing/ non-numeric x or y,
  or a non-finite value is dropped with a warn, never crashes the node.

For a `point` job, `/dispatch/state` reports `waypoint` as a coordinate
string (e.g. `"(5.0, 2.0)"`) so the UI has something to display; for a named
job it is the waypoint name as before.

`fitness` is the localizer's raw `fitness_score` from `/alignment_status`
(lower = better; the whill_safety failsafe trips past 1.0 sustained 2 s) and
`battery` is the CR2 gauge % from `/whill/states/model_cr2` — both are
`null` until their first source message (battery stays `null` when the
driver stack / `whill_msgs` is absent; dispatch degrades instead of failing
to import).

`phase ∈ IDLE / QUEUED / ACTIVE / SUCCEEDED / ABORTED / CANCELED`.

FREE-ness of a tapped point is gated twice: the UI reads `map.png` and
refuses to submit a non-drivable cell (first net), and `dispatch_node` does
NOT re-check the map — it relies on Nav2's global planner
(`allow_unknown:false`) to fail-to-plan an UNKNOWN/OCC coordinate and return
`ABORTED` (second, authoritative net). Re-reading the map in dispatch would
duplicate that check against a downscaled png and risk disagreeing with the
planner's grid.

Both ROS→Web topics are re-published on a timer (not latched): roslibjs
subscribes volatile, so periodic resend makes UI attach order irrelevant.
`dispatch_node` also pushes a `/dispatch/state` snapshot the instant the
phase changes, so a short-lived QUEUED is never dropped between two 5 Hz
samples. `type` is carried through but `goto` and `recall` behave
identically today (the branch is for post-demo work).

## Manual-rescue teleop (feat/teleop-rescue)

> ⚠ **未検証の安全ゲート (2026-07-21)**: 本機能の核心的な安全保証
> 「手動操縦中に Layer D (前方歩行者検知) が発火したら safety(100) が
> teleop(50) を上書きして停止する」は **実機未検証**。mock は robot 無しで
> twist_mux の3スロット同時調停を確認できない。**実機で「手動操縦中に前方へ
> 人を立たせ停止する」を確認するまで、手動操縦は監督者同伴でのみ使用すること。**
> 検証 pass 後に本注記と ADR-0007/0012 の Status を更新する。

When the chair stalls in a spot Nav2 cannot plan out of, an operator drives it
free from the iPad, then re-picks a goal — no terminal. To keep the Web side on
`/dispatch/*` only (ADR-0012), the UI never publishes `/cmd_vel*`: it publishes
`/dispatch/teleop` and `dispatch_node` does the String→Twist conversion into
`/cmd_vel_teleop`, which `twist_mux` arbitrates at **priority 50** (ADR-0007:
safety 100 > teleop 50 > nav 10).

`/dispatch/teleop` carries one of two JSON shapes:

- `{"active":true|false}` — manual-rescue mode ON/OFF. Explicit toggle, **OFF
  is default** (誤操作防止). Only literal boolean `true` enables; anything else
  disables (safe bias). The state is echoed back as `teleop_active` in
  `/dispatch/state`, which drives the UI's direction-button enable.
- `{"vx":<m/s>,"wz":<rad/s>}` — a motion command. Honored only while manual
  mode is active, and **clamped** to `|vx| ≤ 0.3` m/s, `|wz| ≤ 0.6` rad/s
  (`TELEOP_*_MAX`). The UI sends a fixed low rescue speed (0.2 / 0.4); the
  clamp bounds untrusted browser input the same way `_parse_point` does — a
  non-dict / non-numeric / non-finite / out-of-range value is dropped or
  clamped, never crashes the node or emits a NaN twist.

`teleop_active` is the toggle state, **not** "the chair is moving": with manual
mode ON but no button held, the dead-man keeps `/cmd_vel_teleop` silent and the
chair stays put. Motion happens only while commands keep arriving.

**dead-man, three layers** (a command stream that stops must brake the chair):

1. UI finger-up — on `pointerup`/`pointercancel` (and page hide/blur) the UI
   stops the ~10 Hz stream and sends one zero twist. Pointer capture makes the
   release fire even if the finger slides off the button or off-screen.
2. dispatch watchdog — if no teleop command arrives for `TELEOP_WATCHDOG_S =
   0.4 s` while a stream is live, `dispatch_node` sends one zero and goes
   silent (covers a UI that failed to send its zero: frozen tab, dropped link).
3. twist_mux timeout — the teleop slot has `timeout: 0.5 s`; once dispatch goes
   silent the mux drops teleop and navigation (or a stopped bus) resumes.

**Safety interaction (unchanged by design):** the Layer-D pedestrian stop, and
every other failsafe layer, publish `/cmd_vel_safety` at priority 100 > teleop
50, so a manual command can never drive the chair through a detected person or
a diverged localizer. Nothing in the teleop path enforces this — it falls out
of the twist_mux priority order. During rescue with an ACTIVE Nav2 job, teleop
50 > nav 10 means the manual command wins without cancelling the job; after
rescue, toggle OFF and re-submit a goal (existing flow) to resume dispatch.

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

# teleop: enable, then a motion command shows up as a Twist on /cmd_vel_teleop;
# stop sending and ~0.4 s later the watchdog emits one zero and goes silent.
ros2 topic echo /cmd_vel_teleop &        # watch the converted Twist
ros2 topic pub --once /dispatch/teleop std_msgs/String '{data: "{\"active\":true}"}'
ros2 topic pub --once /dispatch/teleop std_msgs/String '{data: "{\"vx\":0.2,\"wz\":0.0}"}'
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
bottom-left, y flipped for image coordinates), plus the inverse
(`pixelToMap`) to turn a map tap into a goal.

For the map-tap FREE gate, `app.js` also decodes `map.png` into an offscreen
canvas and reads the tapped cell's greyscale value. The threshold
(`FREE_THRESHOLD = 230`) sits between UNKNOWN (205) and FREE (254): the
current map.png is ~84% value-205 (unmapped campus) and ~4% value-254
(roads), so a lower cutoff would pass the whole grey campus as drivable.
If a future map regeneration changes the greyscale mapping of FREE/UNKNOWN,
re-check this constant against a histogram of the new `map.png`.

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
