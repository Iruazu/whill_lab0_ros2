"""ublox_gps_node bring-up for the NEO-M8P on /dev/ttyACM0.

Loads `config/ublox_m8p_rover.yaml` and starts a single ublox_gps_node.
After this launch, the following topics are expected:

  /fix                           sensor_msgs/NavSatFix       1 Hz
  /fix_velocity                  geometry_msgs/TwistWithCovarianceStamped
  /navpvt                        ublox_msgs/NavPVT            (raw UBX)
  /navsat                        ublox_msgs/NavSAT
  /navstatus                     ublox_msgs/NavSTATUS
  ... and the rest of the publish.all set

frame_id stamped on the messages is `gnss_link`. The matching URDF
link is added in Phase C-3; without it, navsat_transform_node will
warn but ros2 topic echo still works.

The launch is intentionally minimal — Phase C-4 will compose this
with navsat_transform_node and the state_estimation launch via
IncludeLaunchDescription so the runtime stack stays a single
top-level launch.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import RegisterEventHandler, EmitEvent
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('whill_localization')
    params = os.path.join(pkg_share, 'config', 'ublox_m8p_rover.yaml')

    ublox_node = Node(
        package='ublox_gps',
        executable='ublox_gps_node',
        # output='both' mirrors stdout+stderr to console and log file,
        # matching the upstream ublox_gps launch template. The first
        # bring-up benefits from seeing the INF startup banner on
        # console so we can confirm the receiver answered.
        output='both',
        parameters=[params],
    )

    return LaunchDescription([
        ublox_node,
        # If ublox_gps_node dies (e.g. /dev/ttyACM0 was unplugged),
        # take the rest of the launch down too instead of letting
        # the launcher hang waiting for a topic that will never
        # publish again. Mirrors the upstream template behaviour.
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=ublox_node,
                on_exit=[EmitEvent(event=Shutdown())],
            ),
        ),
    ])
