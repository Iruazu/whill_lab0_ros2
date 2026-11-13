# SPDX-License-Identifier: BSD-3-Clause
"""Start only the demo's localizer and static TFs, for bag replay (#142).

m6r_bringup also starts the sensor drivers, so it cannot run here. The localizer
wiring is imported from it rather than copied, so a change to the demo's
localizer parameters or remaps is what this check actually exercises.

`lidar_tf:="x y z roll pitch yaw"` (metres, degrees) replaces the branch's TF
chain with that single base_link -> velodyne edge. It exists for comparing a
candidate TF (#145) against the current one on the same bag (#155) before the
launch file is changed; the localizer only needs that edge. Empty (default)
keeps the branch's static_tf_launch.py, which is what the regression check uses.
"""
import importlib.util
import math
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            OpaqueFunction)
from launch.conditions import UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def _lidar_tf_override(context):
    spec = LaunchConfiguration('lidar_tf').perform(context).split()
    if not spec:
        return []
    if len(spec) != 6:
        raise RuntimeError(f'lidar_tf needs "x y z roll pitch yaw", got {spec}')
    x, y, z, r, p, yw = map(float, spec)
    args = [str(v) for v in (x, y, z)] + [str(math.radians(v)) for v in (yw, p, r)]
    # static_transform_publisher's positional order is x y z yaw pitch roll.
    return [Node(package='tf2_ros', executable='static_transform_publisher',
                 name='static_tf_velodyne_override',
                 arguments=args + ['base_link', 'velodyne'],
                 parameters=[{'use_sim_time': True}])]


def generate_launch_description():
    path = os.path.join(get_package_share_directory('whill_safety'),
                        'launch', 'm6r_bringup_launch.py')
    spec = importlib.util.spec_from_file_location('m6r_bringup_launch', path)
    bringup = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(bringup)
    static_tf = os.path.join(get_package_share_directory('whill_sensors_bringup'),
                             'launch', 'static_tf_launch.py')
    override = PythonExpression(["'", LaunchConfiguration('lidar_tf'), "'.strip() != ''"])
    return LaunchDescription([
        # デモの既定（m6r_bringup の campus-m8、2026-11-13）とは揃えず、campus のまま。
        # 回帰チェック（replay_check.py）の基準 bag は旧 campus で走った記録で、比べる
        # /pcl_pose も旧 campus の座標にある。campus-m8 で比べるには座標変換と Hi-Res に
        # 補正した bag が要り、それは ndt_replay_compare.py（--site 必須）が扱う。
        # なお基準は #186 の前の TF で測ったので、今の static_tf_launch.py では campus でも
        # 基準を超える見込み（baseline.json の site_note）。
        DeclareLaunchArgument('site', default_value='campus'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument(
            'lidar_tf', default_value='',
            description='"x y z roll pitch yaw" [m, deg]: publish only this base_link->velodyne '
                        'instead of static_tf_launch.py (candidate TF comparison, #155). '
                        'Empty = the branch TF chain (regression check)'),
        # From the branch under test, not the bag's /tf_static, so a TF change shows up.
        IncludeLaunchDescription(PythonLaunchDescriptionSource(static_tf),
                                 launch_arguments={'use_sim_time': 'true'}.items(),
                                 condition=UnlessCondition(override)),
        OpaqueFunction(function=_lidar_tf_override),
        OpaqueFunction(function=bringup._generate_localizer_launch),
    ])
