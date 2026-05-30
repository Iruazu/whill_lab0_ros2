#!/usr/bin/env python3
"""Republish /imu/data_raw with all 3-axis acceleration and angular-
velocity components sign-flipped.

The RT PCMK-G3X 9-axis IMU on this chair reports static-state
linear_acceleration as the gravity vector itself, NOT as REP-145's
"force needed to hold the IMU stationary" (= -gravity). With the IMU
mounted +Y-forward / +Z-up on the chair, a level static reading is
therefore (0, 0, -9.81 m/s²) instead of (0, 0, +9.81). FAST-LIO and
FAST-LIO-SAM both assume the REP-145 sign, so without this flip the
grav_align step decides "+Z is down" and everything is buried under
the floor on startup (observed 2026-05-30).

We flip both acceleration and angular_velocity because the device's
sign convention is presumably consistent across the channels; if the
gyro symptom later contradicts that, narrow the flip to acceleration
only. The orientation field is preserved untouched — FAST-LIO does not
use it (orientation_covariance[0] = -1 already marks it as
"unavailable" upstream).
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from sensor_msgs.msg import Imu


def imu_qos() -> QoSProfile:
    # rt_usb_9axisimu_driver publishes /imu/data_raw with RELIABILITY=RELIABLE
    # (rclcpp default), and FAST-LIO-SAM subscribes with RELIABLE as well.
    # Matching both ends avoids "incompatible QoS" warnings that silently
    # drop every message (observed 2026-05-30).
    return QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=10,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.VOLATILE,
    )


class ImuSignFlip(Node):
    def __init__(self) -> None:
        super().__init__('imu_sign_flip')
        self.declare_parameter('input_topic', '/imu/data_raw')
        self.declare_parameter('output_topic', '/imu/data_corrected')
        # flip_accel and flip_gyro let us isolate which channel actually
        # needs inversion if a later device returns mixed conventions.
        self.declare_parameter('flip_accel', True)
        self.declare_parameter('flip_gyro', True)

        in_topic = self.get_parameter('input_topic').value
        out_topic = self.get_parameter('output_topic').value
        self.flip_accel = bool(self.get_parameter('flip_accel').value)
        self.flip_gyro = bool(self.get_parameter('flip_gyro').value)

        qos = imu_qos()
        self.pub = self.create_publisher(Imu, out_topic, qos)
        self.sub = self.create_subscription(
            Imu, in_topic, self.on_imu, qos)

        self.get_logger().info(
            f'imu_sign_flip: {in_topic} -> {out_topic}; '
            f'flip_accel={self.flip_accel}, flip_gyro={self.flip_gyro}')

    def on_imu(self, msg: Imu) -> None:
        out = Imu()
        out.header = msg.header
        out.orientation = msg.orientation
        out.orientation_covariance = msg.orientation_covariance
        if self.flip_gyro:
            out.angular_velocity.x = -msg.angular_velocity.x
            out.angular_velocity.y = -msg.angular_velocity.y
            out.angular_velocity.z = -msg.angular_velocity.z
        else:
            out.angular_velocity = msg.angular_velocity
        out.angular_velocity_covariance = msg.angular_velocity_covariance
        if self.flip_accel:
            out.linear_acceleration.x = -msg.linear_acceleration.x
            out.linear_acceleration.y = -msg.linear_acceleration.y
            out.linear_acceleration.z = -msg.linear_acceleration.z
        else:
            out.linear_acceleration = msg.linear_acceleration
        out.linear_acceleration_covariance = msg.linear_acceleration_covariance
        self.pub.publish(out)


def main() -> None:
    rclpy.init()
    node = ImuSignFlip()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
