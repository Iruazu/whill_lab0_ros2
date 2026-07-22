"""M4-R unified odom-layer bringup (Issue #38).

Single command that wires all the M4-R pieces together:

  1. `whill_sensors_bringup/sensors_launch.py` — VLP-16 + RealSense D435
     + RT 9-axis IMU drivers, plus the M4R-2 measurement-based static TF
     chain `base_link -> {imu_link, velodyne, camera_link}`.
  2. `whill_bringup/whill_launch.py` (upstream `ros2_whill`, Iruazu fork
     pinned to `humble-with-odom-2026-06-18`) — `whill_driver` with the
     M4R-1 `/whill/odom` publisher. Port `/dev/ttyUSB0` resolves to the
     same physical device as `/dev/whill` via the udev rule in
     `udev/99-whill-stack.rules`, so we accept the upstream
     `params.yaml` default rather than carrying a fork-local override.
  3. `whill_localization/ekf_odom_launch.py` — `robot_localization`
     `ekf_node` fusing `/whill/odom` + `/imu/data_raw` into
     `/odometry/filtered` and the `odom -> base_link` TF edge at 30 Hz.

After this launch settles the expected TF tree is:

    odom (ekf_filter_node, M4R-3)
    └── base_link
        ├── imu_link        (static, M4R-2)
        ├── velodyne        (static, M4R-2)
        └── camera_link     (static, M4R-2)
            └── (realsense2_camera subtree)

There is no `map` frame in M4-R — M6-R is responsible for adding the
scan-to-map localizer that publishes `map -> odom`.

Mutual exclusion:

  - Do NOT run `whill_sensors_bringup/sensors_launch.py` in parallel.
    This launch INCLUDES it — starting both duplicates the velodyne
    driver, IMU driver, and static TF publisher. Measured
    `/velodyne_points` at 39.4 Hz on 2026-07-16 field with a doubled
    bringup.
  - Do NOT run `whill_safety/m6r_bringup_launch.py` in parallel either.
    m6r_bringup includes THIS launch, so the parallel run triples up
    the entire M4-R odom layer.

Mutual exclusion is documented in `src/whill_localization/README.md`
as the authoritative source.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # Resolve include paths at launch-description build time. When this
    # file is itself wrapped by an outer `IncludeLaunchDescription` (e.g.
    # by a future M6-R/M7 bringup), a `LaunchConfiguration` used inside
    # `PythonLaunchDescriptionSource(...)` silently expands to an empty
    # string and the include fails at runtime. Same quirk as
    # `ekf_odom_launch.py` — see that header.
    sensors_share = get_package_share_directory('whill_sensors_bringup')
    whill_share = get_package_share_directory('whill_bringup')
    loc_share = get_package_share_directory('whill_localization')

    sensors_launch = os.path.join(sensors_share, 'launch', 'sensors_launch.py')
    whill_launch = os.path.join(whill_share, 'launch', 'whill_launch.py')
    ekf_launch = os.path.join(loc_share, 'launch', 'ekf_odom_launch.py')

    # `use_sim_time` is forwarded only to the EKF include. The upstream
    # `whill_launch.py` does not declare a `use_sim_time` argument and
    # the sensor drivers are live-only (no `--clock` replay path in
    # M4-R), so propagating to those two would be a no-op at best and a
    # silent failure at worst.
    use_sim_time = LaunchConfiguration('use_sim_time')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Forwarded to the EKF include only. Default false '
                        'matches live chair operation; flip to true if you '
                        'replay a sensor + /whill/odom bag with --clock and '
                        'want the EKF to consume bag timestamps.'),
        DeclareLaunchArgument(
            'realsense',
            default_value='false',
            description='Forwarded to sensors_launch.py to start the D435 '
                        'driver. Off by default — see sensors_launch.py '
                        'docstring for rationale.'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(sensors_launch),
            launch_arguments={
                'realsense': LaunchConfiguration('realsense'),
            }.items()),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(whill_launch)),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(ekf_launch),
            launch_arguments={'use_sim_time': use_sim_time}.items()),
    ])
