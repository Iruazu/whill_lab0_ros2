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

// Occupancy-grid convention: origin is the map-frame coordinate of the
// pgm's BOTTOM-left pixel, resolution is metres per pixel, and image y runs
// downward — hence the flip. res_eff already folds in the png downscale
// factor, so this operates directly in the small png's pixel space.
function mapToPixel(mx, my) {
  const px = (mx - mapMeta.origin[0]) / mapMeta.resolution;
  const py = mapMeta.height - (my - mapMeta.origin[1]) / mapMeta.resolution;
  return [px, py];
}

async function loadMap() {
  mapMeta = await fetch('map_meta.json').then((r) => r.json());
  const img = $('map-img');
  img.src = mapMeta.image;
  // Pin the box to the natural png size so absolute marker/vehicle pixel
  // positions line up 1:1 with mapToPixel output.
  const box = $('map-box');
  box.style.width = mapMeta.width + 'px';
  box.style.height = mapMeta.height + 'px';
  img.width = mapMeta.width;
  img.height = mapMeta.height;
}

// --- UI state ------------------------------------------------------------

let waypoints = [];       // [{name,label,x,y,yaw}]
let selectedName = null;

function log(msg) {
  $('log').textContent = msg;
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
    dot.addEventListener('click', () => selectWaypoint(wp.name));

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
  $('waypoint-select').value = name;
  renderMarkers();
}

// --- ROS wiring ----------------------------------------------------------

let submitTopic = null;
let cancelService = null;

function connect() {
  const url = `ws://${location.hostname || 'localhost'}:9090`;
  const ros = new ROSLIB.Ros({ url });

  ros.on('connection', () => {
    $('conn').textContent = '接続済み';
    $('conn').className = 'conn conn-up';
    $('submit-btn').disabled = false;
  });
  ros.on('error', () => {
    $('conn').textContent = '接続エラー';
    $('conn').className = 'conn conn-down';
  });
  ros.on('close', () => {
    $('conn').textContent = '切断 — 再接続中…';
    $('conn').className = 'conn conn-down';
    $('submit-btn').disabled = true;
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

  $('submit-btn').addEventListener('click', () => {
    if (!selectedName || !submitTopic) return;
    const payload = { waypoint: selectedName, type: jobType() };
    submitTopic.publish(new ROSLIB.Message({ data: JSON.stringify(payload) }));
    log(`配車を送信: ${labelFor(selectedName)} (${jobType()})`);
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
  await loadMap();
  renderMarkers();
  wireControls();
  connect();
}

main();
