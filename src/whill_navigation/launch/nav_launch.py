"""Nav2 lifecycle bringup for the WHILL chair (M6R4-1 + M6R4-2 + M6R4-c).

Composes map_server + planner_server + controller_server + behavior_server
+ bt_navigator + velocity_smoother behind lifecycle_manager, plus a
Patchwork++ ground-removal preprocessor and a pointcloud_to_laserscan
bridge that feeds the costmap obstacle layer.

Pipeline (M6R4-c onwards):

    /velodyne_points  ─▶ patchworkpp_node (whill_perception)
                                  │
                                  └─▶ /velodyne_points_no_ground
                                             │
                                             └─▶ pointcloud_to_laserscan_node
                                                    │
                                                    └─▶ /scan
                                                             │
                                                             └─▶ obstacle_layer

The upstream `m6r_bringup_launch.py` (whill_safety, M6R-2 + M6R-3 lite)
is expected to be running in parallel and supplies the
`map -> odom -> base_link` TF chain, `failsafe_node`, and `twist_mux`;
this launch adds nothing to those.

  ros2 launch whill_safety   m6r_bringup_launch.py site:=campus     # terminal A
  ros2 launch whill_navigation nav_launch.py       site:=campus     # terminal B

Site selection: pass `site:=<name>`. Resolves to `docs/maps/<site>/
occupancy.yaml` at launch time and injects it into map_server. Matches the
M6R-2 convention where `site:=campus` loads `docs/maps/campus/static.pcd`,
so the same value picks the pgm/yaml for Nav2 and the pcd for the
localizer.

cmd_vel routing (matches ADR-0007 M6R-3 lite twist_mux, PR #79):

    controller_server  ─┐
                        ├─> /cmd_vel_nav ─┐
    behavior_server    ─┘                 │
                                          ├─> twist_mux (priority: safety > nav)
    failsafe_node ─> /cmd_vel_safety ─────┘
                                          │
                                          └─> /cmd_vel ─> velocity_smoother
                                                              │
                                                              └─> /whill/controller/cmd_vel

Nav2 nodes publish to `/cmd_vel_nav`, twist_mux picks between it and
`/cmd_vel_safety`, and the mux output `/cmd_vel` feeds velocity_smoother.
The M6R4-1 scope is the two remaps below; twist_mux itself lives in the
whill_safety package (M6R-3 lite).

Obstacle observation path (M6R4-c update to M6R4-2):

    /velodyne_points  (SensorDataQoS, best-effort)
        │
        └─> patchworkpp_node (whill_perception, ADR-0011)
                │
                └─> /velodyne_points_no_ground  (best-effort, xyz only)
                        │
                        └─> pointcloud_to_laserscan_node
                                │
                                └─> /scan  (reliable)
                                        │
                                        └─> obstacle_layer of
                                            local + global costmaps

nav2_costmap_2d's ObstacleLayer expects reliable QoS on observation
sources; pointcloud_to_laserscan republishes at reliable, so
obstacle_layer subscribes without an explicit QoS override.
Ground removal (M6R4-c) is inserted between the driver and the slice
so p2ls's height band can relax back toward capturing curbs without
resurrecting the sloped-ground false-lethals from Phase B 2026-07-14.
`use_collision_detection` stays false through M6R4-c and flips to true
in M6R4-3 once V1-V6 confirm behaviour on the chair.

Deliberately not in this launch:

  - **Localization.** `m6r_bringup_launch.py` (M6R-2) publishes
    `map -> odom`. Do not include a second localizer here.
  - **failsafe_node / twist_mux.** Live on the whill_safety side
    (safety_launch.py, included from m6r_bringup_launch.py). This launch
    only needs the topic contract (`/cmd_vel_nav` in, `/cmd_vel` out).

Historical: at M4-R close (2026-06-20) this file was intentionally left
in a broken state (default map pointed at `lab-legacy-m5b`, no localizer
authoring `map -> odom`). M6R-2 landed the localizer, this M6R4-1 change
restores map_server + planner_server bringup on top of it.
"""

import os
import tempfile

import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


_MAPS_ROOT_ENV = 'WHILL_MAPS_ROOT'
_DEFAULT_MAPS_ROOT_REL = os.path.join('docs', 'maps')
# Match m6r_bringup_launch.py (whill_safety) which defaults site to
# 'campus' for the M5-R production map. Consistent site names across
# launches let a single `site:=<name>` argument fan out to both.
_DEFAULT_SITE = 'campus'


def _repo_root_from_pkg_share(pkg_share):
    """Recover the repo root from an installed package share dir.

    Mirrors the escape hatch in whill_safety/m6r_bringup_launch.py so the
    two launches resolve `docs/maps/<site>/` identically.
    """
    return os.path.abspath(os.path.join(pkg_share, '..', '..', '..', '..'))


def _resolve_map_yaml(context):
    """Resolve `docs/maps/<site>/occupancy.yaml` from the site arg.

    Runs at launch time (OpaqueFunction) so the site LaunchConfiguration
    is available and the file is validated before map_server tries to
    load it. map_server's own error path on a missing yaml is a lifecycle
    CONFIGURE failure with a stale-looking log line; failing early here
    gives a direct message with the resolved path.
    """
    site = LaunchConfiguration('site').perform(context)

    pkg_share = get_package_share_directory('whill_navigation')
    maps_root = os.environ.get(_MAPS_ROOT_ENV)
    if not maps_root:
        maps_root = os.path.join(_repo_root_from_pkg_share(pkg_share), _DEFAULT_MAPS_ROOT_REL)
    map_yaml = os.path.abspath(os.path.join(maps_root, site, 'occupancy.yaml'))

    if not os.path.isfile(map_yaml):
        raise RuntimeError(
            f'nav_launch: occupancy.yaml not found for site={site!r}.\n'
            f'  Looked at: {map_yaml}\n'
            f'  Set {_MAPS_ROOT_ENV}=<path> to override the maps registry '
            f'root, or run:\n'
            f'    ls {maps_root}\n'
            f'  to see the available sites.'
        )

    nav_share = get_package_share_directory('whill_navigation')
    # Substitutions into Node(parameters=[...]) resolve to empty string
    # when this file is wrapped by IncludeLaunchDescription, so hard-code
    # the paths here inside OpaqueFunction where the LaunchContext is
    # already active. See docs/session-2026-05-08.md.
    nav2_params_src = os.path.join(nav_share, 'config', 'nav2_params.yaml')
    p2ls_params = os.path.join(nav_share, 'config',
                               'pointcloud_to_laserscan.yaml')

    # Bake the resolved map path directly into a per-run copy of
    # nav2_params.yaml, then hand every Nav2 node that yaml as its only
    # parameters entry. The alternative (parameters=[<yaml>, {dict}]) was
    # unreliable in humble: with the yaml's `yaml_filename: ""` load
    # winning over the dict override at least on this host, map_server
    # ended up serving whatever default the yaml carried (the M5-c
    # hardcode of `docs/maps/lab-legacy-m5b/lab.yaml`) instead of the
    # site-resolved path. Mirrors m6r_bringup_launch.py's template →
    # temp yaml → include pattern for the localizer.
    with open(nav2_params_src) as f:
        params = yaml.safe_load(f)
    params['map_server']['ros__parameters']['yaml_filename'] = map_yaml

    tmp = tempfile.NamedTemporaryFile(
        prefix='whill_nav2_params_',
        suffix='.yaml',
        delete=False,
        mode='w',
    )
    yaml.safe_dump(params, tmp)
    tmp.close()
    nav2_params = tmp.name

    lifecycle_nodes = [
        'map_server',
        'planner_server',
        'controller_server',
        'behavior_server',
        'bt_navigator',
        'velocity_smoother',
    ]

    ground_removal_launch = os.path.join(
        get_package_share_directory('whill_perception'),
        'launch', 'ground_removal_launch.py')

    return [
        # Patchwork++ ground removal (ADR-0011). Sits between the LiDAR
        # driver and p2ls so the slice below sees terrain-relative
        # obstacles instead of a raw base_link-flat cut. Publishes
        # /velodyne_points_no_ground on SensorDataQoS (best-effort);
        # p2ls's own subscription is also best-effort.
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(ground_removal_launch),
        ),
        # QoS bridge from best-effort /velodyne_points_no_ground to
        # reliable /scan so nav2_costmap_2d's obstacle_layer
        # (reliable-by-default in humble) can subscribe. Not a lifecycle
        # node — starts at process launch and streams once patchworkpp_node
        # is producing non-ground output.
        Node(
            package='pointcloud_to_laserscan',
            executable='pointcloud_to_laserscan_node',
            name='pointcloud_to_laserscan',
            output='screen',
            parameters=[p2ls_params],
            remappings=[('cloud_in', '/velodyne_points_no_ground'),
                        ('scan', '/scan')],
        ),
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[nav2_params],
        ),
        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            parameters=[nav2_params],
        ),
        Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            output='screen',
            parameters=[nav2_params],
            # Remap the raw output to /cmd_vel_nav so twist_mux (M6R-3
            # lite) can prioritize failsafe /cmd_vel_safety over it.
            remappings=[('/cmd_vel', '/cmd_vel_nav')],
        ),
        Node(
            package='nav2_behaviors',
            executable='behavior_server',
            name='behavior_server',
            output='screen',
            parameters=[nav2_params],
            # Recovery behaviours (spin/backup/wait) also publish via
            # /cmd_vel by default; funnel them through twist_mux the same
            # way so a recovery cannot bypass the safety mux.
            remappings=[('/cmd_vel', '/cmd_vel_nav')],
        ),
        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            parameters=[nav2_params],
        ),
        Node(
            package='nav2_velocity_smoother',
            executable='velocity_smoother',
            name='velocity_smoother',
            output='screen',
            parameters=[nav2_params],
            # Smoother subscribes /cmd_vel (twist_mux output) and
            # publishes /cmd_vel_smoothed. Remap the output to the WHILL
            # driver's input topic so we don't need a separate relay.
            # RPP itself does not ramp, so without the smoother the chair
            # sees a 0 -> desired_linear_vel step that a seated rider
            # feels as a lurch (first noticed on live M5-d).
            remappings=[('/cmd_vel_smoothed', '/whill/controller/cmd_vel')],
        ),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            output='screen',
            parameters=[{
                'use_sim_time': False,
                'autostart': True,
                'node_names': lifecycle_nodes,
                'bond_timeout': 4.0,
            }],
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'site',
            default_value=_DEFAULT_SITE,
            description='Name of the map directory under docs/maps/ to '
                        'load. Resolves to <maps_root>/<site>/'
                        'occupancy.yaml at launch time. Override the '
                        'maps root with WHILL_MAPS_ROOT if not launching '
                        'from a colcon workspace that mirrors this repo '
                        'layout. Should match the value passed to '
                        'm6r_bringup_launch.py.'),

        # No localization include. m6r_bringup_launch.py (whill_safety,
        # M6R-2) is expected to be running in parallel and publishes
        # map -> odom. Adding a second localizer here would race for the
        # same TF edge.

        OpaqueFunction(function=_resolve_map_yaml),
    ])
