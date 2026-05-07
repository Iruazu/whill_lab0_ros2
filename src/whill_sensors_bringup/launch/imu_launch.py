"""Bring up the RT 9-axis USB IMU as a lifecycle node and auto-activate it.

The upstream `rt_usb_9axisimu_driver` is a `LifecycleNode`, so a plain
`ros2 run` leaves it in the `unconfigured` state with no topics. This
launch drives `unconfigured -> inactive -> active` via lifecycle events
so subscribers see `/imu/data_raw`, `/imu/mag`, `/imu/temperature`
without any manual `ros2 lifecycle set` calls.
"""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    LogInfo,
    RegisterEventHandler,
    TimerAction,
)
from launch.events import matches_action
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from lifecycle_msgs.msg import Transition


def generate_launch_description():
    port_arg = DeclareLaunchArgument(
        'port',
        default_value='/dev/imu',
        description='Serial path for the RT 9-axis USB IMU. The repo udev '
                    'rule (udev/99-whill-stack.rules) maps the device VID:PID '
                    '2b72:0003 to this stable symlink.',
    )
    frame_arg = DeclareLaunchArgument(
        'frame_id',
        default_value='imu_link',
        description='TF frame populated in the IMU messages.',
    )

    imu_node = LifecycleNode(
        name='rt_usb_9axisimu_driver',
        namespace='',
        package='rt_usb_9axisimu_driver',
        executable='rt_usb_9axisimu_driver',
        output='screen',
        parameters=[{
            'port': LaunchConfiguration('port'),
            'frame_id': LaunchConfiguration('frame_id'),
        }],
    )

    configure = EmitEvent(event=ChangeState(
        lifecycle_node_matcher=matches_action(imu_node),
        transition_id=Transition.TRANSITION_CONFIGURE,
    ))

    # Wait briefly after `configure` before issuing `activate`. The driver's
    # first readSensorData() call happens during the `activate` callback,
    # and if the serial port has not yet buffered a full ASCII frame from
    # the device the read returns FAILURE and the node bounces to errorprocessing.
    # 1.5 s is enough margin at 100 Hz frame rate (~10 ms per frame) plus
    # the device's initial sync window.
    activate_after_settle = TimerAction(
        period=1.5,
        actions=[
            EmitEvent(event=ChangeState(
                lifecycle_node_matcher=matches_action(imu_node),
                transition_id=Transition.TRANSITION_ACTIVATE,
            )),
        ],
    )

    activate_on_inactive = RegisterEventHandler(OnStateTransition(
        target_lifecycle_node=imu_node,
        goal_state='inactive',
        entities=[
            LogInfo(msg="rt_usb_9axisimu_driver reached 'inactive', "
                        'waiting 1.5 s before activate to let the serial '
                        'port buffer a frame.'),
            activate_after_settle,
        ],
    ))

    return LaunchDescription([
        port_arg,
        frame_arg,
        imu_node,
        activate_on_inactive,
        configure,
    ])
