#!/usr/bin/env python3
# SPDX-License-Identifier: BSD-3-Clause
"""Rebuild the D435 point cloud from a recorded depth image (bag replay only).

The stage-A static bags (~/bags/sensor_check_20261028/) and the
record_avoidance_bag.sh `static` profile store the depth IMAGE, not
/camera/camera/depth/color/points, to save space. depth_image_proc is not
installed on this PC, so this node does the pinhole back-projection the
realsense driver's pointcloud filter does and republishes it on the live
topic, letting perception_3d_launch.py / STVL run unmodified on old bags.

Matches config/realsense.yaml (whill_sensors_bringup) closely enough for
costmap validation, not bit-exactly:
  - 2x2 decimation like `decimation_filter.filter_magnitude: 2`, using the
    median of the non-zero pixels in each block (the RS filter's rule for
    magnitude 2-3); skipped when the recorded image is already decimated
    (width <= 480).
  - zero depth (no return) is dropped, as the RS pointcloud does.
  - no RS spatial/temporal filters (they are off in realsense.yaml too).

  scripts/tools/replay_depth_to_points.py --ros-args -p use_sim_time:=true
"""

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image, PointCloud2, PointField


class DepthToPoints(Node):

    def __init__(self):
        super().__init__('replay_depth_to_points')
        self.declare_parameter('depth_topic', '/camera/camera/depth/image_rect_raw')
        self.declare_parameter('info_topic', '/camera/camera/depth/camera_info')
        self.declare_parameter('points_topic', '/camera/camera/depth/color/points')
        self.declare_parameter('decimate', True)
        self.K = None
        self.rays = None
        self.create_subscription(
            CameraInfo, self.get_parameter('info_topic').value, self.on_info,
            qos_profile_sensor_data)
        self.create_subscription(
            Image, self.get_parameter('depth_topic').value, self.on_depth,
            qos_profile_sensor_data)
        self.pub = self.create_publisher(
            PointCloud2, self.get_parameter('points_topic').value, qos_profile_sensor_data)

    def on_info(self, msg):
        if self.K is None:
            self.K = np.array(msg.k).reshape(3, 3)

    def on_depth(self, msg):
        if self.K is None or msg.encoding not in ('16UC1', 'mono16'):
            return
        d = np.frombuffer(msg.data, np.uint16).reshape(msg.height, msg.width)
        fx, fy, cx, cy = self.K[0, 0], self.K[1, 1], self.K[0, 2], self.K[1, 2]
        if self.get_parameter('decimate').value and msg.width > 480:
            h, w = (msg.height // 2) * 2, (msg.width // 2) * 2
            blk = d[:h, :w].reshape(h // 2, 2, w // 2, 2).transpose(0, 2, 1, 3)
            blk = blk.reshape(h // 2, w // 2, 4).astype(np.float32)
            blk[blk == 0] = np.nan
            with np.errstate(all='ignore'):
                d = np.nan_to_num(np.nanmedian(blk, axis=2), nan=0.0)
            # Pixel-centre convention for a 2x downscale.
            fx, fy, cx, cy = fx / 2, fy / 2, (cx + 0.5) / 2 - 0.5, (cy + 0.5) / 2 - 0.5
        if self.rays is None or self.rays[0].shape != d.shape:
            v, u = np.indices(d.shape, dtype=np.float32)
            self.rays = ((u - cx) / fx, (v - cy) / fy)
        z = d.astype(np.float32) / 1000.0
        ok = z > 0
        pts = np.stack([self.rays[0][ok] * z[ok], self.rays[1][ok] * z[ok], z[ok]], 1)

        out = PointCloud2()
        out.header = msg.header
        out.height, out.width = 1, len(pts)
        out.fields = [PointField(name=n, offset=4 * i, datatype=PointField.FLOAT32, count=1)
                      for i, n in enumerate('xyz')]
        out.point_step, out.row_step = 12, 12 * len(pts)
        out.is_dense = True
        out.data = pts.astype(np.float32).tobytes()
        self.pub.publish(out)


def main():
    rclpy.init()
    rclpy.spin(DepthToPoints())


if __name__ == '__main__':
    main()
