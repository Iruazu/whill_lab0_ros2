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
    """Drop points inside a self-exclusion cylinder around the LiDAR.

    A point at (x, y, z) in the velodyne frame is dropped if it sits
    inside the cylinder:

        sqrt(x² + y²) < self_radius   AND   self_z_min < z < self_z_max

    This carves out a small region around the LiDAR mount / chair body
    while leaving the rest of the scan (floor at distance, walls, etc.)
    intact. An earlier z-only filter was tempting but wrong: stripping
    everything with z < -0.10 m killed legitimate floor returns and
    starved FAST-LIO's scan registration.
    """

    def __init__(self) -> None:
        super().__init__('velodyne_self_filter')
        self.declare_parameter('input_topic', '/velodyne_points')
        self.declare_parameter('output_topic', '/velodyne_points_filtered')
        # Defaults are a conservative "no-op" cylinder so the filter cannot
        # break FAST-LIO until you opt in to a specific geometry. Tune via
        # the launch parameters once you've eyeballed the raw cloud in RViz.
        self.declare_parameter('self_radius', 0.0)
        self.declare_parameter('self_z_min', -0.5)
        self.declare_parameter('self_z_max', 0.1)
        self.declare_parameter('stats_every_n', 100)

        in_topic = self.get_parameter('input_topic').value
        out_topic = self.get_parameter('output_topic').value
        self.self_radius = float(self.get_parameter('self_radius').value)
        self.self_z_min = float(self.get_parameter('self_z_min').value)
        self.self_z_max = float(self.get_parameter('self_z_max').value)
        self.stats_every_n = int(self.get_parameter('stats_every_n').value)

        self._msg_count = 0
        self._kept_total = 0
        self._total = 0

        qos = sensor_data_qos()
        self.pub = self.create_publisher(PointCloud2, out_topic, qos)
        self.sub = self.create_subscription(
            PointCloud2, in_topic, self.on_cloud, qos)

        self.get_logger().info(
            f'velodyne_self_filter: {in_topic} -> {out_topic}; '
            f'exclude cylinder r<{self.self_radius:.2f} m, '
            f'z in [{self.self_z_min:.2f}, {self.self_z_max:.2f}] m '
            f'(self_radius=0 means filter is a pass-through)')

    def on_cloud(self, msg: PointCloud2) -> None:
        offsets = {f.name: f.offset for f in msg.fields}
        if 'x' not in offsets or 'y' not in offsets or 'z' not in offsets:
            return

        n_points = msg.width * msg.height
        if n_points == 0:
            return

        buf = np.frombuffer(msg.data, dtype=np.uint8).reshape(n_points, msg.point_step)

        def field(off: int) -> np.ndarray:
            chunk = np.ascontiguousarray(buf[:, off:off + 4])
            return chunk.view(np.float32).ravel()

        if self.self_radius > 0.0:
            x = field(offsets['x'])
            y = field(offsets['y'])
            z = field(offsets['z'])
            in_self = (
                (x * x + y * y < self.self_radius * self.self_radius)
                & (z > self.self_z_min)
                & (z < self.self_z_max)
            )
            keep = ~in_self
            kept = buf[keep]
        else:
            # self_radius == 0 → no-op pass-through, but the data still flows
            # through the topic so downstream comparisons work.
            kept = buf

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

        # Periodic diagnostic — every stats_every_n messages, log the
        # cumulative kept ratio so a misconfigured filter is obvious in the
        # launch log without dumping a line per scan.
        self._msg_count += 1
        self._kept_total += int(kept.shape[0])
        self._total += int(n_points)
        if self._msg_count >= self.stats_every_n:
            ratio = self._kept_total / max(self._total, 1)
            self.get_logger().info(
                f'velodyne_self_filter: last {self._msg_count} msgs '
                f'kept {self._kept_total}/{self._total} points '
                f'({100*ratio:.1f}%)')
            self._msg_count = 0
            self._kept_total = 0
            self._total = 0


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
