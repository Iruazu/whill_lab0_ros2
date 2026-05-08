"""Top-level Nav2 bringup for the WHILL chair.

Composition order:
  1. M4 localization (whill_localization/localization_launch.py) —
     sensors + FAST-LIO + RViz, producing /Odometry and the
     camera_init -> body TF.
  2. M5-a TF bridge (this package) — map -> camera_init and
     body -> base_link static transforms so Nav2 sees a Nav2-shaped
     TF chain.
  3. Nav2 lifecycle bringup — added in a follow-up commit once the
     map_server, planner_server, controller_server, bt_navigator,
     and lifecycle_manager_navigation pieces are configured for the
     chair (M5-c).

For offline replay (no chair, no live FAST-LIO), use
`whill_localization/fast_lio_launch.py` separately and include only
this package's tf_bridge_launch.py.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    loc_share = get_package_share_directory('whill_localization')
    nav_share = get_package_share_directory('whill_navigation')

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(loc_share, 'launch', 'localization_launch.py'))),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav_share, 'launch', 'tf_bridge_launch.py'))),
        # Nav2 lifecycle bringup will be added here in M5-c.
    ])
