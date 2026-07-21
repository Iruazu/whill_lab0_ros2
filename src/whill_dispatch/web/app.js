// M7 whill_dispatch tablet UI logic.
//
// Talks to ROS only through rosbridge (ws) via roslibjs, using the four
// /dispatch/* interfaces. No direct Nav2 / cmd_vel access (platform-pivot
// §5 #4). All map<->pixel maths uses map_meta.json (origin + effective
// resolution written by scripts/m7_make_web_map.py).
//
// QoS: roslibjs subscribes volatile. dispatch_node re-publishes
// /dispatch/waypoints (1 Hz) and /dispatch/state (5 Hz) on a timer rather
// than latching, so attaching late (e.g. reloading the tab) still fills the
// UI within a second — no transient_local needed.

'use strict';

const $ = (id) => document.getElementById(id);

// --- map geometry --------------------------------------------------------

let mapMeta = null;
// Offscreen 2D context holding map.png at natural size. Used only to read a
// tapped cell's greyscale value for the UI-side FREE gate (see isFree).
let mapCtx = null;

// Occupancy-grid convention: origin is the map-frame coordinate of the
// pgm's BOTTOM-left pixel, resolution is metres per pixel, and image y runs
// downward — hence the flip. res_eff already folds in the png downscale
// factor, so this operates directly in the small png's pixel space.
function mapToPixel(mx, my) {
  const px = (mx - mapMeta.origin[0]) / mapMeta.resolution;
  const py = mapMeta.height - (my - mapMeta.origin[1]) / mapMeta.resolution;
  return [px, py];
}

// Exact inverse of mapToPixel: a png pixel back to a map-frame coordinate,
// so a tap can be turned into a NavigateToPose goal. Keep the two in lockstep
// — a drift between them would put the goal somewhere the operator did not
// tap.
function pixelToMap(px, py) {
  const mx = px * mapMeta.resolution + mapMeta.origin[0];
  const my = (mapMeta.height - py) * mapMeta.resolution + mapMeta.origin[1];
  return [mx, my];
}

async function loadMap() {
  // Cache-bust both requests: the filenames never change across map
  // regenerations, so a browser that visited the page before a map update
  // silently keeps showing the stale png (bitten on 2026-07-19 after the
  // v1 -> v2 background swap). The png is ~120 KiB on a LAN; refetching
  // per page load is cheaper than a stale-map incident in the field.
  const bust = '?v=' + Date.now();
  mapMeta = await fetch('map_meta.json' + bust).then((r) => r.json());
  const img = $('map-img');
  img.src = mapMeta.image + bust;
  // Pin the box to the natural png size so absolute marker/vehicle pixel
  // positions line up 1:1 with mapToPixel output.
  const box = $('map-box');
  box.style.width = mapMeta.width + 'px';
  box.style.height = mapMeta.height + 'px';
  img.width = mapMeta.width;
  img.height = mapMeta.height;

  // Match the plan SVG's user space to the png pixel space so the polyline
  // coordinates (from mapToPixel) need no further scaling.
  const plan = $('plan-layer');
  plan.setAttribute('width', mapMeta.width);
  plan.setAttribute('height', mapMeta.height);
  plan.setAttribute('viewBox', `0 0 ${mapMeta.width} ${mapMeta.height}`);

  // Decode the png into an offscreen canvas so the tap handler can read the
  // FREE/OCC value of a cell (getImageData). Same-origin (served by the
  // launch's http.server), so the canvas is not tainted. willReadFrequently
  // hints the 2D backend for the repeated 1x1 reads. img.decode() guarantees
  // the pixels are ready before the first tap can land.
  await img.decode();
  const canvas = document.createElement('canvas');
  canvas.width = mapMeta.width;
  canvas.height = mapMeta.height;
  mapCtx = canvas.getContext('2d', { willReadFrequently: true });
  mapCtx.drawImage(img, 0, 0, mapMeta.width, mapMeta.height);
}

// UI-side FREE gate. map.png is the greyscale, downscaled operative map:
// FREE=254 (white roads), UNKNOWN=205, OCC=0 in the source pgm, blurred by the
// 8x downscale. Only the white roads are drivable, so the threshold MUST sit
// above UNKNOWN(205), not below it: a histogram of the current map.png is 84%
// value-205 (the unmapped campus) and 4% value-254 (roads). A cutoff of 200
// (the figure floated in the v2 plan) would pass all of that 205 grey as FREE
// — i.e. every building/lawn tap clears this net and leans entirely on Nav2.
// 230 sits cleanly between 205 and 254: it admits the solid road core (254)
// and its downscale halo (down to ~230) while rejecting UNKNOWN(205) and the
// 201-210 blur around it. Erring high just forces taps onto solid road, which
// is the safe bias. Nav2 (allow_unknown:false) remains the authoritative
// second net in dispatch_node.
const FREE_THRESHOLD = 230;

function isFree(px, py) {
  if (!mapCtx) return false;
  const x = Math.min(mapMeta.width - 1, Math.max(0, Math.round(px)));
  const y = Math.min(mapMeta.height - 1, Math.max(0, Math.round(py)));
  // Greyscale png, so R==G==B; the red channel is the cell value.
  return mapCtx.getImageData(x, y, 1, 1).data[0] > FREE_THRESHOLD;
}

// --- UI state ------------------------------------------------------------

let waypoints = [];       // [{name,label,x,y,yaw}]
let selectedName = null;  // a named waypoint pick, or null
let goalPoint = null;     // an arbitrary tapped goal {x,y,yaw} in map frame
let connected = false;    // rosbridge link state, gates the submit button

function log(msg) {
  $('log').textContent = msg;
}

// Submit is live only when connected AND a goal exists (a named waypoint or a
// tapped FREE point). Without a goal the button greys out — this is what a
// non-FREE tap falls back to (clearGoalPoint below).
function updateSubmitEnabled() {
  $('submit-btn').disabled = !(connected && (selectedName || goalPoint));
}

function renderMarkers() {
  const layer = $('markers');
  layer.innerHTML = '';
  for (const wp of waypoints) {
    const [px, py] = mapToPixel(wp.x, wp.y);

    const dot = document.createElement('div');
    dot.className = 'marker' + (wp.name === selectedName ? ' selected' : '');
    dot.style.left = px + 'px';
    dot.style.top = py + 'px';
    dot.title = wp.label;
    // stopPropagation so clicking a marker selects it and does NOT also fire
    // the map-box tap handler (which would treat the same click as an
    // arbitrary goal and immediately override the named pick).
    dot.addEventListener('click', (e) => {
      e.stopPropagation();
      selectWaypoint(wp.name);
    });

    const label = document.createElement('div');
    label.className = 'marker-label';
    label.style.left = px + 'px';
    label.style.top = py + 'px';
    label.textContent = wp.label;

    layer.appendChild(dot);
    layer.appendChild(label);
  }
}

function renderSelect() {
  const sel = $('waypoint-select');
  const prev = sel.value;
  sel.innerHTML = '';
  for (const wp of waypoints) {
    const opt = document.createElement('option');
    opt.value = wp.name;
    opt.textContent = wp.label;
    sel.appendChild(opt);
  }
  sel.disabled = waypoints.length === 0;
  if (prev && waypoints.some((w) => w.name === prev)) {
    sel.value = prev;
  }
  // Keep dropdown and marker selection in sync on first population.
  if (!selectedName && waypoints.length) {
    selectWaypoint(sel.value);
  }
}

function selectWaypoint(name) {
  selectedName = name;
  clearGoalPoint();   // a named pick and a tapped point are mutually exclusive
  $('waypoint-select').value = name;
  renderMarkers();
  updateSubmitEnabled();
}

// --- arbitrary (tapped) goal --------------------------------------------

function renderGoalMarker(px, py) {
  const el = $('goal-marker');
  el.style.left = px + 'px';
  el.style.top = py + 'px';
  el.classList.remove('hidden');
}

function clearGoalPoint() {
  goalPoint = null;
  $('goal-marker').classList.add('hidden');
  updateSubmitEnabled();
}

function onMapClick(ev) {
  // Ignore taps until the map (and its offscreen canvas) is ready.
  if (!mapMeta || !mapCtx) return;
  const box = $('map-box');
  const rect = box.getBoundingClientRect();
  // Convert the client-space tap to png pixel space. Scaling by the natural/
  // displayed ratio keeps this correct even if the box is CSS-scaled to fit a
  // small tablet viewport (rect.width != mapMeta.width) — same basis the
  // markers use, so tap and marker positions stay consistent.
  const px = (ev.clientX - rect.left) * (mapMeta.width / rect.width);
  const py = (ev.clientY - rect.top) * (mapMeta.height / rect.height);

  if (!isFree(px, py)) {
    // Non-FREE: reject at the UI. Drop any pending point goal so the submit
    // button greys out rather than sending a stale one.
    clearGoalPoint();
    log('走行できない場所です（走行可能な道路上を指定してください）');
    return;
  }

  const [mx, my] = pixelToMap(px, py);
  goalPoint = { x: mx, y: my, yaw: 0.0 };  // yaw fixed at 0 for now (future UX)
  selectedName = null;                     // arbitrary goal overrides a name
  renderMarkers();                         // drop the named "selected" ring
  renderGoalMarker(px, py);
  updateSubmitEnabled();
  log(`目的地: (${mx.toFixed(1)}, ${my.toFixed(1)}) m — 発行できます`);
}

// --- Nav2 plan overlay ---------------------------------------------------

// Draw /plan (nav_msgs/Path) as a single polyline in png pixel space. Empty
// input clears the overlay. Setting innerHTML on the SVG element parses the
// markup in the SVG namespace (modern browsers), which keeps this
// dependency-free — no manual createElementNS bookkeeping.
function renderPlan(poses) {
  const layer = $('plan-layer');
  if (!poses || !poses.length || !mapMeta) {
    layer.innerHTML = '';
    return;
  }
  const pts = poses.map((ps) => {
    const [px, py] = mapToPixel(ps.pose.position.x, ps.pose.position.y);
    return `${px.toFixed(1)},${py.toFixed(1)}`;
  }).join(' ');
  layer.innerHTML =
    `<polyline points="${pts}" fill="none" stroke="#3a86ff" ` +
    `stroke-width="3" stroke-linejoin="round" stroke-linecap="round" ` +
    `opacity="0.9"/>`;
}

function clearPlan() {
  $('plan-layer').innerHTML = '';
}

// --- ROS wiring ----------------------------------------------------------

let submitTopic = null;
let cancelService = null;

function connect() {
  // ws/wss をページの配信スキームから選ぶ。iPad は HTTPS-First で https で
  // 開くため (dispatch_launch use_tls:=true)、その場合ブラウザは平文 ws:// を
  // mixed content で遮断する → wss:// が必須 (use_tls は rosbridge 側も
  // wss 化する)。平文 http のときは従来どおり ws://。
  const scheme = location.protocol === 'https:' ? 'wss' : 'ws';
  const url = `${scheme}://${location.hostname || 'localhost'}:9090`;
  const ros = new ROSLIB.Ros({ url });

  ros.on('connection', () => {
    $('conn').textContent = '接続済み';
    $('conn').className = 'conn conn-up';
    connected = true;
    updateSubmitEnabled();  // enable only if a goal is already picked
  });
  ros.on('error', () => {
    $('conn').textContent = '接続エラー';
    $('conn').className = 'conn conn-down';
  });
  ros.on('close', () => {
    $('conn').textContent = '切断 — 再接続中…';
    $('conn').className = 'conn conn-down';
    connected = false;
    updateSubmitEnabled();
    // rosbridge or the tab's network can drop; retry so the operator does
    // not have to reload. Fixed 2 s backoff is plenty on a local LAN.
    setTimeout(connect, 2000);
  });

  submitTopic = new ROSLIB.Topic({
    ros, name: '/dispatch/submit', messageType: 'std_msgs/String',
  });
  cancelService = new ROSLIB.Service({
    ros, name: '/dispatch/cancel', serviceType: 'std_srvs/Trigger',
  });

  new ROSLIB.Topic({
    ros, name: '/dispatch/waypoints', messageType: 'std_msgs/String',
  }).subscribe((msg) => {
    const next = JSON.parse(msg.data);
    // Only re-render on an actual change — the 1 Hz resend would otherwise
    // rebuild the DOM every second and fight a click mid-render.
    if (JSON.stringify(next) !== JSON.stringify(waypoints)) {
      waypoints = next;
      renderSelect();
      renderMarkers();
    }
  });

  new ROSLIB.Topic({
    ros, name: '/dispatch/state', messageType: 'std_msgs/String',
  }).subscribe((msg) => updateState(JSON.parse(msg.data)));

  // /plan is a Nav2 topic, not a /dispatch/* one, so this is the single place
  // the UI reaches past the dispatch boundary. It is a read-only visualization
  // subscription (no command flows back), which ADR-0012 permits as an
  // explicit exception (see its /plan addendum). Cleared on a terminal phase
  // in updateState so a finished route does not linger on the map.
  new ROSLIB.Topic({
    ros, name: '/plan', messageType: 'nav_msgs/Path',
  }).subscribe((msg) => renderPlan(msg.poses));
}

function updateState(s) {
  const phase = s.phase || 'IDLE';
  const phaseEl = $('phase');
  phaseEl.textContent = phase;
  phaseEl.className = 'phase phase-' + phase;

  $('active-waypoint').textContent = labelFor(s.waypoint) || '—';
  $('queue-len').textContent = s.queue_len != null ? s.queue_len : 0;
  $('progress-bar').style.width = Math.round((s.progress || 0) * 100) + '%';
  $('aligned').textContent =
    s.aligned == null ? '—' : (s.aligned ? 'OK' : 'NG');

  // Cancel is meaningful only while a job is running.
  // ACTIVE only: during the short QUEUED window the goal handle does not
  // exist yet, so /dispatch/cancel would return success=false and the press
  // would be silently lost right before the job goes ACTIVE anyway.
  $('cancel-btn').disabled = phase !== 'ACTIVE';

  // Keep the plan overlay only while a job is live. On any terminal/idle
  // phase the route is done, so clear it rather than leaving a stale line
  // (Nav2 stops updating /plan once idle, so it would otherwise persist).
  if (phase !== 'ACTIVE' && phase !== 'QUEUED') {
    clearPlan();
  }

  // Vehicle dot follows /pcl_pose folded into state.pose.
  const dot = $('vehicle');
  if (s.pose && mapMeta) {
    const [px, py] = mapToPixel(s.pose.x, s.pose.y);
    dot.style.left = px + 'px';
    dot.style.top = py + 'px';
    dot.classList.remove('hidden');
  } else {
    dot.classList.add('hidden');
  }
}

function labelFor(name) {
  const wp = waypoints.find((w) => w.name === name);
  return wp ? wp.label : name;
}

// --- controls ------------------------------------------------------------

function jobType() {
  const checked = document.querySelector('input[name="jobtype"]:checked');
  return checked ? checked.value : 'goto';
}

function wireControls() {
  $('waypoint-select').addEventListener('change', (e) => {
    selectWaypoint(e.target.value);
  });

  $('map-box').addEventListener('click', onMapClick);

  $('submit-btn').addEventListener('click', () => {
    if (!submitTopic) return;
    // A tapped point takes precedence over a named pick (they are mutually
    // exclusive in the UI, but be explicit at the boundary). Send exactly one
    // of {point} / {waypoint}; dispatch_node applies the same precedence.
    let payload;
    if (goalPoint) {
      payload = { point: goalPoint, type: jobType() };
      log(`配車を送信: 任意地点 (${goalPoint.x.toFixed(1)}, ` +
          `${goalPoint.y.toFixed(1)}) (${jobType()})`);
    } else if (selectedName) {
      payload = { waypoint: selectedName, type: jobType() };
      log(`配車を送信: ${labelFor(selectedName)} (${jobType()})`);
    } else {
      return;
    }
    submitTopic.publish(new ROSLIB.Message({ data: JSON.stringify(payload) }));
  });

  $('cancel-btn').addEventListener('click', () => {
    if (!cancelService) return;
    cancelService.callService(
      new ROSLIB.ServiceRequest({}),
      (res) => log(`キャンセル: ${res.message}`),
      (err) => log(`キャンセル失敗: ${err}`));
  });
}

async function main() {
  // SF-2: a failed map fetch/decode (404, corrupt png, cache mismatch) must
  // surface, not hang silently. README records the 2026-07-19 stale-map
  // incident; here we make the failure visible so an operator sees a cause
  // instead of a frozen "接続待ち…" screen. rosbridge (connect) still starts
  // so job control degrades to a mapless-but-live UI rather than nothing.
  try {
    await loadMap();
  } catch (e) {
    log('地図の読み込みに失敗しました: ' + e.message);
  }
  renderMarkers();
  wireControls();
  connect();
}

main();
