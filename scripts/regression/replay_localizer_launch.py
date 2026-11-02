# SPDX-License-Identifier: BSD-3-Clause
"""Start only the demo's localizer and static TFs, for bag replay (#142).

m6r_bringup also starts the sensor drivers, so it cannot run here. The localizer
wiring is imported from it rather than copied, so a change to the demo's
localizer parameters or remaps is what this check actually exercises.
"""
import importlib.util
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            OpaqueFunction)
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    path = os.path.join(get_package_share_directory('whill_safety'),
                        'launch', 'm6r_bringup_launch.py')
    spec = importlib.util.spec_from_file_location('m6r_bringup_launch', path)
    bringup = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(bringup)
    static_tf = os.path.join(get_package_share_directory('whill_sensors_bringup'),
                             'launch', 'static_tf_launch.py')
    return LaunchDescription([
        DeclareLaunchArgument('site', default_value='campus'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        # From the branch under test, not the bag's /tf_static, so a TF change shows up.
        IncludeLaunchDescription(PythonLaunchDescriptionSource(static_tf),
                                 launch_arguments={'use_sim_time': 'true'}.items()),
        OpaqueFunction(function=bringup._generate_localizer_launch),
    ])
