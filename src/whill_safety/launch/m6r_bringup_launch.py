"""M6-R unified bringup (Issue #66 / M6R-2).

Single command that stacks the M4-R odom layer under the M6-R
scan-to-map localizer:

  1. `whill_localization/odom_bringup_launch.py` — sensors + WHILL driver
     + M4-R EKF. Publishes `odom -> base_link` and the
     `base_link -> {imu_link, velodyne, camera_link}` static chain.
  2. `lidar_localization_ros2/lidar_localization.launch.py` (upstream
     LifecycleNode with configure -> active transitions built in).
     Publishes `map -> odom` when a scan matches the loaded PCD.

Resulting TF chain after both settle (REP-105):

    map (lidar_localization)
    └── odom (ekf_filter_node, M4-R)
        └── base_link (EKF-integrated, smooth)
            ├── imu_link, velodyne, camera_link (static, PR #74 pitch=-8)

Map selection: pass `site:=<name>`. The launch resolves
`docs/maps/<site>/static.pcd` at launch time and injects it as the
localizer's `map_path` param via a generated per-run yaml under
`/tmp/`. The rest of the NDT tuning (score_threshold, ndt_resolution,
...) comes from `config/m6r_lidar_localization.yaml` verbatim.

Mutual exclusion:

  - Do NOT run `whill_localization/odom_bringup_launch.py` in parallel
    with this launch. This launch INCLUDES it — starting both would
    have `robot_localization` publish `odom -> base_link` twice on
    the same TF edge and both `map_server` (from nav_launch.py, when
    that lands at M6R-4) and this localizer would collide on the map
    frame if unsupervised.
  - Do NOT run `whill_localization/localization_launch.py` (FAST-LIO)
    in parallel either. FAST-LIO was frozen as a runtime localizer by
    the 2026-06-11 platform pivot; it is retained only for offline
    map-making.

To operate:

    ros2 launch whill_safety m6r_bringup_launch.py \\
        site:=campus-outdoor-corrected

Then in RViz, use "2D Pose Estimate" to publish `/initialpose`. The
localizer converges in a few seconds, `map -> odom` starts flowing,
and `/pcl_pose` tracks the chair through the map.

For bag replay use `scripts/m6r_smoke_test.sh` instead of this launch;
that script sets `use_sim_time` and publishes an initial pose from a
script, both of which live-operation does not need.
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


_MAPS_ROOT_ENV = 'WHILL_MAPS_ROOT'
_DEFAULT_MAPS_ROOT_REL = os.path.join('docs', 'maps')
_DEFAULT_SITE = 'campus-outdoor-corrected'   # M6R-1-verified map, PR #74


def _repo_root_from_pkg_share(pkg_share):
    """Recover the repo root from an installed package share dir.

    A colcon install puts our share at `<repo>/install/whill_safety/share/
    whill_safety`. Walking up four levels gets us to the repo root reliably.
    This lookup runs at launch time so it does not have to be robust to
    every possible install layout — but it does need to handle the
    developer's `install/` layout without breaking. `WHILL_MAPS_ROOT` is
    the escape hatch.
    """
    return os.path.abspath(os.path.join(pkg_share, '..', '..', '..', '..'))


def _generate_localizer_launch(context):
    """Compute map_path from the site arg and hand the upstream launch a
    per-run yaml with map_path already filled in.

    Runs at launch time (OpaqueFunction) so the site arg is resolved from
    LaunchConfiguration and the yaml is materialised before
    IncludeLaunchDescription loads the upstream launch.
    """
    site = LaunchConfiguration('site').perform(context)
    use_sim_time = LaunchConfiguration('use_sim_time').perform(context)

    pkg_share = get_package_share_directory('whill_safety')
    template_path = os.path.join(pkg_share, 'config', 'm6r_lidar_localization.yaml')

    # Locate the map. Prefer WHILL_MAPS_ROOT (used by the smoke test wrappers)
    # so a reviewer can point at an alternate maps registry without editing
    # this file. Fall back to the docs/maps/ layout under the repo root.
    maps_root = os.environ.get(_MAPS_ROOT_ENV)
    if not maps_root:
        maps_root = os.path.join(_repo_root_from_pkg_share(pkg_share), _DEFAULT_MAPS_ROOT_REL)
    map_pcd = os.path.abspath(os.path.join(maps_root, site, 'static.pcd'))

    if not os.path.isfile(map_pcd):
        raise RuntimeError(
            f'm6r_bringup: static.pcd not found for site={site!r}.\n'
            f'  Looked at: {map_pcd}\n'
            f'  Set {_MAPS_ROOT_ENV}=<path> to override the maps registry '
            f'root, or run:\n'
            f'    ls {maps_root}\n'
            f'  to see the available sites.'
        )

    with open(template_path) as f:
        params = yaml.safe_load(f)
    # Fill map_path in the wildcard node namespace ("/**") so the localizer
    # picks it up regardless of the actual node name at runtime.
    params['/**']['ros__parameters']['map_path'] = map_pcd

    # Write to a per-launch tempfile. NamedTemporaryFile keeps the file
    # around after this function returns because delete=False; launch
    # subprocesses need the file to still exist when they load it.
    tmp = tempfile.NamedTemporaryFile(
        prefix='m6r_lidar_localization_',
        suffix='.yaml',
        delete=False,
        mode='w',
    )
    yaml.safe_dump(params, tmp)
    tmp.close()

    localizer_launch = os.path.join(
        get_package_share_directory('lidar_localization_ros2'),
        'launch',
        'lidar_localization.launch.py',
    )

    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(localizer_launch),
            launch_arguments={
                'localization_param_dir': tmp.name,
                'cloud_topic': '/velodyne_points',
                # The M4-R static TF chain already publishes base_link ->
                # velodyne (PR #61 / PR #74 pitch fix). Suppressing the
                # upstream's own base -> lidar identity avoids two
                # publishers on the same TF edge.
                'publish_lidar_tf': 'false',
                'use_sim_time': use_sim_time,
            }.items(),
        ),
    ]


def generate_launch_description():
    odom_bringup_launch = os.path.join(
        get_package_share_directory('whill_localization'),
        'launch',
        'odom_bringup_launch.py',
    )

    use_sim_time = LaunchConfiguration('use_sim_time')

    return LaunchDescription([
        DeclareLaunchArgument(
            'site',
            default_value=_DEFAULT_SITE,
            description='Name of the map directory under docs/maps/ to '
                        'load. Resolves to <maps_root>/<site>/static.pcd '
                        'at launch time. Override the maps root itself '
                        'with the WHILL_MAPS_ROOT env var if you are not '
                        'launching from a colcon workspace that mirrors '
                        'this repo layout.'),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Forwarded to the M4-R EKF and to the localizer. '
                        'Keep false for live sensor operation. Flip true '
                        'if you are launching this against a bag played '
                        'with --clock (rare — scripts/m6r_smoke_test.sh '
                        'is the usual bag-replay entry point).'),

        # M4-R odom layer: sensors + driver + EKF.
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(odom_bringup_launch),
            launch_arguments={'use_sim_time': use_sim_time}.items()),

        # M6-R scan-to-map localizer. Wrapped in an OpaqueFunction so the
        # `site` arg is resolved (and validated to an actual file on disk)
        # before the upstream launch loads.
        OpaqueFunction(function=_generate_localizer_launch),
    ])
