"""M7 dispatch boundary bringup: rosbridge + dispatch_node + static UI server.

Single command that stands up everything between the tablet browser and
Nav2 (platform-pivot §3.5). It composes:

  1. rosbridge_websocket (+ rosapi) on ws://<host>:9090 — the websocket the
     roslibjs UI talks to. All four /dispatch/* interfaces cross here.
  2. dispatch_node — the queue / action-client / state node.
  3. python3 -m http.server on :8000, cwd = the installed web/ dir — serves
     index.html + app.js + vendored roslib. Deliberately the stdlib server:
     the UI is static (no build), so a tablet just opens http://<host>:8000.
  4. (use_mock:=true) scripts/mock_navigate_to_pose.py — a stand-in
     /navigate_to_pose so the whole boundary runs with no robot. Off by
     default; at demo time nav_launch.py supplies the real server.

Expected to run ALONGSIDE (not instead of) the real stack at demo time:

  ros2 launch whill_safety     m6r_bringup_launch.py site:=campus   # A
  ros2 launch whill_navigation nav_launch.py site:=campus map_variant:=cleaned  # B
  ros2 launch whill_dispatch   dispatch_launch.py use_mock:=false   # C

dispatch adds no TF and no /cmd_vel*, so it does not conflict with the
mutual-exclusion tree of the bringup launches; it only opens ports 9090 /
8000 and a NavigateToPose action client.

Site selection: `site:=<name>` resolves docs/maps/<site>/waypoints.yaml at
launch time (same WHILL_MAPS_ROOT / repo-root recovery as nav_launch.py)
and injects it into dispatch_node as the waypoints_path param. Keep the
value equal to what m6r_bringup_launch.py / nav_launch.py get.

Path resolution runs inside an OpaqueFunction: this file may be wrapped by
IncludeLaunchDescription, in which case LaunchConfiguration substitutions
inside Node(parameters=[...]) resolve to empty string, so we perform() the
args here where the LaunchContext is live and hard-code the resolved paths
(same rationale as nav_launch.py's _resolve_map_yaml).
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


_MAPS_ROOT_ENV = 'WHILL_MAPS_ROOT'
_DEFAULT_MAPS_ROOT_REL = os.path.join('docs', 'maps')
# Match m6r_bringup_launch.py / nav_launch.py so one site:= value fans out
# to the pcd (localizer), the pgm (Nav2) and now waypoints.yaml (dispatch).
_DEFAULT_SITE = 'campus'


def _repo_root_from_pkg_share(pkg_share):
    """Recover the repo root from an installed package share dir.

    Mirrors nav_launch.py / m6r_bringup_launch.py: install puts our share at
    <repo>/install/whill_dispatch/share/whill_dispatch, so four levels up is
    the repo root. WHILL_MAPS_ROOT is the escape hatch for non-standard
    layouts; the mock script path has no override and always resolves from
    the repo root (it is a dev-only fixture, never shipped).
    """
    return os.path.abspath(os.path.join(pkg_share, '..', '..', '..', '..'))


def _resolve_waypoints_yaml(site, pkg_share):
    maps_root = os.environ.get(_MAPS_ROOT_ENV)
    if not maps_root:
        maps_root = os.path.join(
            _repo_root_from_pkg_share(pkg_share), _DEFAULT_MAPS_ROOT_REL)
    wp_yaml = os.path.abspath(os.path.join(maps_root, site, 'waypoints.yaml'))
    if not os.path.isfile(wp_yaml):
        raise RuntimeError(
            f'dispatch_launch: waypoints.yaml not found for site={site!r}.\n'
            f'  Looked at: {wp_yaml}\n'
            f'  Set {_MAPS_ROOT_ENV}=<path> to override the maps registry '
            f'root, or run:\n'
            f'    ls {os.path.dirname(wp_yaml)}')
    return wp_yaml


def _setup(context):
    site = LaunchConfiguration('site').perform(context)
    use_mock = LaunchConfiguration('use_mock').perform(context) == 'true'
    use_rosbridge = (
        LaunchConfiguration('use_rosbridge').perform(context) == 'true')
    ws_port = LaunchConfiguration('rosbridge_port').perform(context)
    http_port = LaunchConfiguration('http_port').perform(context)
    action_name = LaunchConfiguration('action_name').perform(context)
    use_tls = LaunchConfiguration('use_tls').perform(context) == 'true'
    tls_cert = os.path.expanduser(
        LaunchConfiguration('tls_cert').perform(context))
    tls_key = os.path.expanduser(
        LaunchConfiguration('tls_key').perform(context))

    pkg_share = get_package_share_directory('whill_dispatch')
    web_dir = os.path.join(pkg_share, 'web')
    waypoints_yaml = _resolve_waypoints_yaml(site, pkg_share)

    # TLS: iOS Safari HTTPS-First で iPad が http を https に格上げしてしまう
    # (2026-07-20 field)。use_tls:=true で UI を HTTPS、rosbridge を WSS にする。
    # cert/key は scripts/m7_make_tls_cert.sh 生成物 (既定 ~/.whill_dispatch_tls)。
    if use_tls:
        for p in (tls_cert, tls_key):
            if not os.path.isfile(p):
                raise RuntimeError(
                    f'dispatch_launch: use_tls:=true だが証明書が無い: {p}\n'
                    f'  先に生成する: scripts/m7_make_tls_cert.sh <AP の IP>')

    actions = []

    if use_rosbridge:
        # Include the upstream xml launch (starts rosbridge_websocket AND
        # rosapi_node). rosapi is what roslibjs uses for topic/service
        # introspection; skipping it works for pub/sub but breaks any UI
        # feature that lists topics. use_rosbridge:=false is for when
        # rosbridge is already running from another terminal — starting a
        # second websocket on the same port would fail to bind.
        rosbridge_launch = os.path.join(
            get_package_share_directory('rosbridge_server'),
            'launch', 'rosbridge_websocket_launch.xml')
        # AnyLaunchDescriptionSource dispatches on the .xml extension to the
        # xml frontend loader (humble has no importable XMLLaunchDescription
        # Source class; this is the supported way to include a frontend
        # launch from a python launch file).
        rosbridge_args = {'port': ws_port}
        if use_tls:
            # rosbridge_websocket_launch.xml は ssl / certfile / keyfile を
            # 受ける。ssl:=true で ws:// が wss:// になる。UI (app.js) は
            # ページの protocol から ws/wss を自動選択するので UI 側は不変。
            rosbridge_args.update(
                {'ssl': 'true', 'certfile': tls_cert, 'keyfile': tls_key})
        actions.append(IncludeLaunchDescription(
            AnyLaunchDescriptionSource(rosbridge_launch),
            launch_arguments=rosbridge_args.items()))

    actions.append(Node(
        package='whill_dispatch',
        executable='dispatch_node',
        name='dispatch_node',
        output='screen',
        parameters=[{
            'waypoints_path': waypoints_yaml,
            'action_name': action_name,
        }],
    ))

    # Static UI server. Plain stdlib http.server by default; with use_tls the
    # HTTPS wrapper (scripts/m7_https_server.py) serves the same web/ dir over
    # TLS so iOS Safari's HTTPS-First stops 400-ing the connection.
    if use_tls:
        https_script = os.path.join(
            _repo_root_from_pkg_share(pkg_share),
            'scripts', 'm7_https_server.py')
        actions.append(ExecuteProcess(
            cmd=['python3', https_script,
                 '--port', http_port, '--dir', web_dir,
                 '--cert', tls_cert, '--key', tls_key],
            output='screen',
        ))
    else:
        actions.append(ExecuteProcess(
            cmd=['python3', '-m', 'http.server', http_port],
            cwd=web_dir,
            output='screen',
        ))

    if use_mock:
        # Dev-only fixture — resolve it from the repo scripts/ dir (never
        # installed). At demo time use_mock:=false and nav_launch.py's
        # bt_navigator is the real /navigate_to_pose server.
        mock_script = os.path.join(
            _repo_root_from_pkg_share(pkg_share),
            'scripts', 'mock_navigate_to_pose.py')
        if not os.path.isfile(mock_script):
            raise RuntimeError(
                f'dispatch_launch: use_mock:=true but mock not found at '
                f'{mock_script}')
        actions.append(ExecuteProcess(
            cmd=['python3', mock_script],
            output='screen',
        ))

    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'site',
            default_value=_DEFAULT_SITE,
            description='Map directory under docs/maps/ to load '
                        'waypoints.yaml from. Resolves to '
                        '<maps_root>/<site>/waypoints.yaml at launch time. '
                        'Override the maps root with WHILL_MAPS_ROOT. Keep '
                        'equal to the value passed to m6r_bringup_launch.py '
                        'and nav_launch.py.'),
        DeclareLaunchArgument(
            'use_mock',
            default_value='false',
            description='Start scripts/mock_navigate_to_pose.py as a '
                        'stand-in /navigate_to_pose so the dispatch '
                        'boundary runs with no robot. Keep false at demo '
                        'time — nav_launch.py provides the real server.'),
        DeclareLaunchArgument(
            'use_rosbridge',
            default_value='true',
            description='Start rosbridge_websocket + rosapi here. Set false '
                        'if rosbridge is already running elsewhere (a second '
                        'websocket on the same port fails to bind).'),
        DeclareLaunchArgument(
            'rosbridge_port',
            default_value='9090',
            description='Websocket port for rosbridge. The UI (app.js) '
                        'assumes 9090; change both together.'),
        DeclareLaunchArgument(
            'http_port',
            default_value='8000',
            description='Port for the static UI http.server.'),
        DeclareLaunchArgument(
            'action_name',
            default_value='/navigate_to_pose',
            description='NavigateToPose action dispatch_node drives. '
                        'Default matches Nav2 bt_navigator and the mock.'),
        DeclareLaunchArgument(
            'use_tls',
            default_value='false',
            description='Serve the UI over HTTPS and rosbridge over WSS. '
                        'Needed for iPad/iOS Safari (HTTPS-First upgrades '
                        'http to https). Generate the cert first with '
                        'scripts/m7_make_tls_cert.sh <AP IP>.'),
        DeclareLaunchArgument(
            'tls_cert',
            default_value='~/.whill_dispatch_tls/dispatch.crt',
            description='TLS cert path (used when use_tls:=true).'),
        DeclareLaunchArgument(
            'tls_key',
            default_value='~/.whill_dispatch_tls/dispatch.key',
            description='TLS key path (used when use_tls:=true).'),

        OpaqueFunction(function=_setup),
    ])
