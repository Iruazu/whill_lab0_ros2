"""TF bridge between FAST-LIO and Nav2.

FAST-LIO publishes `camera_init -> body` (with `body` being the IMU
body frame). Nav2 expects a `map -> odom -> base_link` chain. The
M3 `whill_sensors_bringup` already publishes `base_link -> imu_link`
(identity) and the other `base_link -> sensor` static transforms.

Two extra static identities glue the two trees together:

  map  -> camera_init        (this package)
  camera_init -> body        (FAST-LIO at runtime)
  body -> base_link          (this package)
  base_link -> imu_link      (whill_sensors_bringup, identity)
  base_link -> velodyne      (whill_sensors_bringup, identity)
  base_link -> camera_link   (whill_sensors_bringup, identity)

Net effect: Nav2 sees `map` as the world-fixed frame and resolves
`map -> base_link` through FAST-LIO's pose.

This is the simplest M5-a baseline. A future iteration can replace
the `map -> camera_init` identity with `map -> odom` from a global
re-localizer (AMCL on a saved map, or a loop-closure version of
FAST-LIO) and add a `odom -> base_link` source from WHILL wheel
odometry; for now FAST-LIO's drift over a typical lab-scale drive
is small enough that the identity bridge is acceptable.
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def _static_tf(name, parent, child):
    return Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name=name,
        arguments=[
            '--x', '0.0', '--y', '0.0', '--z', '0.0',
            '--roll', '0.0', '--pitch', '0.0', '--yaw', '0.0',
            '--frame-id', parent,
            '--child-frame-id', child,
        ],
    )


def generate_launch_description():
    return LaunchDescription([
        _static_tf('static_tf_map_to_camera_init', 'map', 'camera_init'),
        _static_tf('static_tf_body_to_base_link', 'body', 'base_link'),
    ])
