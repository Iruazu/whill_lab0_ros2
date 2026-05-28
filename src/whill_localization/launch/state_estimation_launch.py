"""Two-stage robot_localization EKF for Phase A (ADR-0001).

Spawns two `robot_localization/ekf_node` instances and the
`whill_odometry` wrapper that produces `/whill/odom` from the upstream
`ros2_whill/whill_driver`'s `/whill/states/model_cr2` (the upstream driver
does not emit a nav_msgs/Odometry of its own — see whill_odometry/README).

  whill_odometry — converts ModelCr2State -> nav_msgs/Odometry on
             /whill/odom. Must come before ekf_odom because ekf_odom
             subscribes /whill/odom; robot_localization's queue_size=10
             absorbs the startup race in practice, but starting the
             producer first is the explicit guarantee.
  ekf_odom — fuses /whill/odom + /imu/data_raw, publishes odom -> base_link
             and /odometry/filtered. World frame = `odom`. This is the
             continuous local estimator; LIO jumps never reach it.
  ekf_map  — fuses ekf_odom output + FAST-LIO /Odometry, publishes
             map -> odom and /odometry/filtered/global. World frame = `map`.
             This is where drift correction lives.

After this launch + `whill_localization/localization_launch.py` (FAST-LIO +
sensors), the Nav2-standard TF chain is:

  map -> odom         (ekf_map)
  odom -> base_link   (ekf_odom)
  base_link -> ...    (robot_state_publisher / URDF)

Note that FAST-LIO's native `camera_init -> body` chain still exists as a
parallel branch — Nav2 does not look at it. Phase B will retire FAST-LIO in
favor of FASTLIO2_SAM_LC which publishes directly in `map`/`base_link`.

Phase A hack — map -> camera_init identity static TF:
  FAST-LIO publishes /Odometry with `header.frame_id=camera_init`, while
  ekf_map.world_frame=map. robot_localization rejects measurements whose
  header frame differs from world_frame unless it can TF-lookup the
  difference. Without this static TF the lookup fails and ekf_map silently
  drops every odom1 sample, regressing to wheel-odom-only state. We publish
  an identity here as a deliberate Phase A bridge; the assumption is that
  FAST-LIO's `camera_init` was initialised at the chair's start pose, which
  also happens to be the `map` origin in our Phase A workflows.

  This static TF MUST be removed in Phase B once FASTLIO2_SAM_LC publishes
  natively in `map`. Tracked in docs/plans/2026-05-28-campus-autonomous-
  navigation.md under Phase B output.

YAML paths are resolved at launch-description build time rather than via
LaunchConfiguration: when this file is wrapped by IncludeLaunchDescription,
LaunchConfiguration would silently resolve to an empty string in the
`parameters` list and the EKF would fall back to its internal defaults
(publish_tf=true, world_frame=odom, no sensor topics) — producing two
filters that fight over odom -> base_link. Same lesson as
`fast_lio_launch.py`. To override covariances or topic remaps, edit the
installed YAML and re-run `colcon build --symlink-install` (the
symlink-install means a YAML edit takes effect without a full rebuild).

Bag-replay entry point:
  `whill_navigation/nav_launch.py` hardcodes `use_sim_time=false` on its
  include of this file, because nav_launch is the live-chair top-level.
  For offline replay against `ros2 bag play --clock`, invoke
  `state_estimation_launch.py` directly with `use_sim_time:=true`; do not
  route through nav_launch.py (that would mix wall clock and /clock,
  which robot_localization silently tolerates by dropping samples).
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    whill_loc_share = get_package_share_directory('whill_localization')
    whill_odom_share = get_package_share_directory('whill_odometry')

    ekf_odom_yaml = os.path.join(whill_loc_share, 'config', 'ekf_odom.yaml')
    ekf_map_yaml = os.path.join(whill_loc_share, 'config', 'ekf_map.yaml')

    use_sim_time = LaunchConfiguration('use_sim_time')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Set true when consuming /clock from a `ros2 bag play '
                        '--clock` replay. Propagated to whill_odometry and '
                        'both EKFs.'),

        # whill_odometry first — see module docstring for ordering. The
        # include explicitly forwards use_sim_time so a bag replay does not
        # silently fall back to wall-clock stamps on /whill/odom while the
        # EKFs run on /clock.
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(whill_odom_share, 'launch', 'whill_odometry_launch.py')),
            launch_arguments={'use_sim_time': use_sim_time}.items()),

        # Phase A bridge — see module docstring. Identity TF lets ekf_map
        # accept /Odometry samples whose header frame is `camera_init`.
        # The Node executable name is the canonical
        # `static_transform_publisher` from tf2_ros; ros-humble accepts the
        # `--frame-id` / `--child-frame-id` style invocation (the older
        # positional `x y z yaw pitch roll frame child` form is deprecated
        # and emits a warning at startup).
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='map_to_camera_init_phase_a_bridge',
            arguments=[
                '--frame-id', 'map',
                '--child-frame-id', 'camera_init',
                '--x', '0', '--y', '0', '--z', '0',
                '--roll', '0', '--pitch', '0', '--yaw', '0',
            ],
            output='screen',
        ),

        # ekf_odom: odom -> base_link. ekf_node's publisher topic is hardcoded
        # as `odometry/filtered`; we remap it to `/odometry/filtered/local` so
        # that ekf_map's separate remap (which retargets its OWN published
        # topic to /odometry/filtered/global) does not collide here. Why both
        # nodes need distinct remap target names: ROS 2 remap rules match by
        # topic name, not by direction. A remap `('odometry/filtered', X)` on
        # ekf_map would also retarget any subscription it makes to
        # `/odometry/filtered`, redirecting ekf_map's odom0 to its own output
        # (an actual self-loop bug observed before this fix). Renaming
        # ekf_odom's output keeps the names disjoint.
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_odom',
            output='screen',
            parameters=[
                ekf_odom_yaml,
                {'use_sim_time': use_sim_time},
            ],
            remappings=[
                ('odometry/filtered', 'odometry/filtered/local'),
            ],
        ),

        # ekf_map: map -> odom. Its YAML's odom0 reads ekf_odom's renamed
        # output (/odometry/filtered/local), and this remap retargets only
        # ekf_map's own publisher (the hardcoded `odometry/filtered`) to
        # /odometry/filtered/global. With the distinct names there is no
        # remap collision between input and output.
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_map',
            output='screen',
            parameters=[
                ekf_map_yaml,
                {'use_sim_time': use_sim_time},
            ],
            remappings=[
                ('odometry/filtered', 'odometry/filtered/global'),
            ],
        ),
    ])
