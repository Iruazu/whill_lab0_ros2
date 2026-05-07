"""Static TF chain rooted at base_link.

All three sensor frames are wired as direct children of `base_link`:

    base_link
    ├── imu_link
    ├── velodyne
    └── camera_link
        ├── camera_depth_frame -> camera_depth_optical_frame
        └── camera_color_frame -> camera_color_optical_frame

The RealSense subtree below `camera_link` is published by the
`realsense2_camera` driver itself; only the three top-level
`base_link -> sensor_*` transforms are emitted here.

The transforms below are placeholder identities. The LiDAR <-> IMU
extrinsic inherited from the noetic stack is captured in
`docs/m3-extrinsics-from-noetic.md`; the camera is rigidly mounted to
the LiDAR via the support frame and needs a one-shot extrinsic
measurement. Replace the identity values before relying on this TF
tree for FAST-LIO (M4) or Nav2 (M5).
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def _static_tf(name, x, y, z, roll, pitch, yaw, parent, child):
    return Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name=name,
        arguments=[
            '--x', str(x), '--y', str(y), '--z', str(z),
            '--roll', str(roll), '--pitch', str(pitch), '--yaw', str(yaw),
            '--frame-id', parent,
            '--child-frame-id', child,
        ],
    )


def generate_launch_description():
    return LaunchDescription([
        # base_link -> imu_link
        # Identity placeholder. The IMU is the inertial reference for FAST-LIO,
        # so collapsing base_link onto imu_link is a reasonable default until
        # a chassis-anchored base_link is defined.
        _static_tf('static_tf_imu',
                   0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                   'base_link', 'imu_link'),

        # base_link -> velodyne
        # Identity placeholder. TODO: replace with the inherited LiDAR <-> IMU
        # extrinsic from docs/m3-extrinsics-from-noetic.md when wiring M4.
        _static_tf('static_tf_velodyne',
                   0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                   'base_link', 'velodyne'),

        # base_link -> camera_link
        # Identity placeholder. The RealSense is rigidly mounted to the
        # LiDAR via the support frame, so this transform is fixed once
        # measured. The realsense2_camera driver publishes the rest of the
        # camera subtree (camera_depth_frame, camera_color_frame, optical
        # variants) on its own.
        _static_tf('static_tf_camera',
                   0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                   'base_link', 'camera_link'),
    ])
