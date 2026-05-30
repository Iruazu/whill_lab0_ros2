"""robot_state_publisher for the WHILL chair + sensor stack.

Replaces `static_tf_launch.py`. Loads
`urdf/whill_with_sensors.urdf.xacro` (which itself includes the upstream
`whill_description/urdf/whill_model_cr2.urdf`) and feeds the expanded URDF
to `robot_state_publisher`.

Result: every fixed transform in the chair — wheels, seat, sensor arms,
imu_link, velodyne, camera_link, base_footprint — is published from one
declarative source instead of a hand-curated list of static
transform_publisher nodes. Re-measuring an offset means editing one
`<xacro:property>` value, not chasing duplicate identity transforms.
"""

import os
import xacro

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('whill_sensors_bringup')
    xacro_path = os.path.join(pkg_share, 'urdf', 'whill_with_sensors.urdf.xacro')
    # Expand the xacro at launch description build time. Doing this here
    # (rather than passing a `xacro <path>` command to
    # robot_state_publisher) keeps the URDF text in robot_description as a
    # plain string, which downstream tools (Nav2 footprint, RViz robot
    # model) can introspect without spawning a subprocess.
    robot_description_xml = xacro.process_file(xacro_path).toxml()

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': robot_description_xml,
                # The chair has no movable joints we care about for
                # navigation (wheel joints exist in URDF but no encoders
                # feed joint_states), so publish_frequency is just for
                # the static-equivalent transforms.
                'publish_frequency': 30.0,
            }],
        ),
    ])
