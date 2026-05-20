#!/usr/bin/env python3
"""Strip LiDAR mount / chair-body self-returns from /velodyne_points.

VLP-16 mounted on the WHILL chair sees its own support frame and the
chair body in the lower (negative-elevation) beams. Those points trail
behind the chair as it moves and FAST-LIO bakes them into the world
map as phantom walls — visible in RViz as fake corridors that aren't
in the real lab.

The radial `blind: 0.5` filter inside FAST-LIO doesn't catch this
because the mount frame typically extends beyond 0.5 m radial; the
`fov_degree: 180` filter doesn't catch it either because the mount is
in front of the LiDAR.

This node drops points outside [z_min, z_max] in the velodyne frame
before FAST-LIO subscribes. With z_min around -0.10 m the mount and
the close-range floor get culled while the bulk of the LiDAR sweep
(which sees walls / shelves at the LiDAR's own height) survives. Tune
z_min downward to keep more floor structure, upward if phantom walls
persist.

Performance: ~28k points × 10 Hz with a numpy-vectorised mask is
sub-millisecond per scan on a laptop CPU; the per-message cost is
dominated by the bytes copy on publish, not the math.
"""

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from sensor_msgs.msg import PointCloud2


def sensor_data_qos() -> QoSProfile:
    """Match the QoS the velodyne driver publishes with.

    velodyne_pointcloud uses best-effort, volatile, depth 10. Subscribing
    with anything stricter (e.g. RELIABLE) silently drops every message.
    """
    return QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=10,
        reliability=ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.VOLATILE,
    )


class VelodyneSelfFilter(Node):

    def __init__(self) -> None:
        super().__init__('velodyne_self_filter')
        self.declare_parameter('input_topic', '/velodyne_points')
        self.declare_parameter('output_topic', '/velodyne_points_filtered')
        self.declare_parameter('z_min', -0.10)
        self.declare_parameter('z_max', 100.0)

        in_topic = self.get_parameter('input_topic').value
        out_topic = self.get_parameter('output_topic').value
        self.z_min = float(self.get_parameter('z_min').value)
        self.z_max = float(self.get_parameter('z_max').value)

        qos = sensor_data_qos()
        self.pub = self.create_publisher(PointCloud2, out_topic, qos)
        self.sub = self.create_subscription(
            PointCloud2, in_topic, self.on_cloud, qos)

        self.get_logger().info(
            f'velodyne_self_filter: {in_topic} -> {out_topic}, '
            f'keep z in [{self.z_min:.2f}, {self.z_max:.2f}] m (velodyne frame)')

    def on_cloud(self, msg: PointCloud2) -> None:
        offsets = {f.name: f.offset for f in msg.fields}
        z_off = offsets.get('z')
        if z_off is None:
            return

        n_points = msg.width * msg.height
        if n_points == 0:
            return

        # Treat raw buffer as (N, point_step) of uint8. Slice out the 4 bytes
        # of z per point, force-contiguous so view(float32) is safe (the slice
        # is not contiguous along the inner axis in general).
        buf = np.frombuffer(msg.data, dtype=np.uint8).reshape(n_points, msg.point_step)
        z_bytes = np.ascontiguousarray(buf[:, z_off:z_off + 4])
        z = z_bytes.view(np.float32).ravel()

        keep = (z >= self.z_min) & (z <= self.z_max)
        kept = buf[keep]

        out = PointCloud2()
        out.header = msg.header
        out.height = 1
        out.width = int(kept.shape[0])
        out.fields = msg.fields
        out.is_bigendian = msg.is_bigendian
        out.point_step = msg.point_step
        out.row_step = msg.point_step * out.width
        out.is_dense = msg.is_dense
        out.data = kept.tobytes()

        self.pub.publish(out)


def main() -> None:
    rclpy.init()
    node = VelodyneSelfFilter()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
