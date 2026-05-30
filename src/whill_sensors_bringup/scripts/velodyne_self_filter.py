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

Two stacked exclusion layers — points dropped if they fall into either:

  1. The legacy near-LiDAR cylinder (self_radius / self_z_min /
     self_z_max). Catches mount strut + cables sitting inside the
     `r < self_radius` cylinder.
  2. Optional forward-arc sectors (forward_arc_enabled +
     forward_arc_sectors). Each sector is an (azimuth, radius, z)
     box in the velodyne frame; intended for chair-body clusters that
     sit outside the cylinder at fixed bearings (LiDAR-frame arms,
     wheels, sensor-arm tubes). Derived from
     analyze_velodyne_arc.py's azimuth histogram and only enabled
     when chair geometry creates persistent off-axis clusters.

Performance: ~28k points × 10 Hz with a numpy-vectorised mask is
sub-millisecond per scan on a laptop CPU; the per-message cost is
dominated by the bytes copy on publish, not the math. Adding 2-3
sectors costs one extra arctan2 + a tight per-sector AND-mask each
scan — well under a millisecond.
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
    """Drop points inside a self-exclusion cylinder + optional sectors.

    Two layers, OR-combined (a point is dropped if it falls into either):

      Cylinder (always-on, governed by self_radius / self_z_min /
                self_z_max):
          sqrt(x² + y²) < self_radius  AND  self_z_min < z < self_z_max

      Forward-arc sectors (opt-in via forward_arc_enabled, list defined
                in forward_arc_sectors). Each sector is a CSV string
                "az_min, az_max, r_min, r_max, z_min, z_max" with az in
                degrees (velodyne-frame azimuth, +x forward = 0°) and
                r/z in metres:
          az_min < atan2(y,x) < az_max  AND
          r_min < sqrt(x²+y²) < r_max   AND
          z_min < z < z_max

    Sectors exist because chair-body clusters can sit OUTSIDE the cylinder
    at fixed bearings — e.g. the left/right armrests project a `>1.2 m`
    arc at +30°/-90° even though they are physically rigid to the LiDAR.
    Expanding the cylinder to swallow them would also eat legitimate
    forward floor / wall returns and starve FAST-LIO. Surgical sector
    cuts preserve the forward sweep that scan registration needs.

    An earlier z-only filter was tempting but wrong: stripping everything
    with z < -0.10 m killed legitimate floor returns and produced
    "No Effective Points!" within a second.
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
        # Forward-arc sectors are opt-in. Disabled by default so a stock
        # install behaves identically to the cylinder-only release. ROS 2
        # humble's declare_parameter has no stable list-of-dict support, so
        # each sector is a CSV string and parsing happens in _parse_sectors.
        self.declare_parameter('forward_arc_enabled', False)
        self.declare_parameter('forward_arc_sectors', [''])

        in_topic = self.get_parameter('input_topic').value
        out_topic = self.get_parameter('output_topic').value
        self.self_radius = float(self.get_parameter('self_radius').value)
        self.self_z_min = float(self.get_parameter('self_z_min').value)
        self.self_z_max = float(self.get_parameter('self_z_max').value)
        self.stats_every_n = int(self.get_parameter('stats_every_n').value)
        arc_enabled = bool(self.get_parameter('forward_arc_enabled').value)
        arc_sectors_raw = list(self.get_parameter('forward_arc_sectors').value)
        self.sectors = self._parse_sectors(arc_enabled, arc_sectors_raw)

        self._msg_count = 0
        self._kept_total = 0
        self._total = 0

        qos = sensor_data_qos()
        self.pub = self.create_publisher(PointCloud2, out_topic, qos)
        self.sub = self.create_subscription(
            PointCloud2, in_topic, self.on_cloud, qos)

        sector_summary = (
            f'{self.sectors.shape[0]} forward-arc sector(s)'
            if self.sectors is not None
            else 'no forward-arc sectors'
        )
        self.get_logger().info(
            f'velodyne_self_filter: {in_topic} -> {out_topic}; '
            f'exclude cylinder r<{self.self_radius:.2f} m, '
            f'z in [{self.self_z_min:.2f}, {self.self_z_max:.2f}] m '
            f'(self_radius=0 means filter is a pass-through); '
            f'{sector_summary}')

    @staticmethod
    def _parse_sectors(enabled, raw_list):
        # Fail loud rather than silently skip: a typo in a sector CSV that
        # produced a no-op filter is exactly the failure mode that masked
        # the M5-d "FAST-LIO ignoring /velodyne_points_filtered" regression.
        # Better the launch dies at startup than mysteriously emits raw
        # data downstream.
        if not enabled:
            return None
        # The default declare value [''] becomes a list with a single empty
        # string when the launch passes no sectors — treat that as "off".
        cleaned = [s.strip() for s in raw_list if s and s.strip()]
        if not cleaned:
            return None
        rows = []
        for spec in cleaned:
            parts = [p.strip() for p in spec.split(',')]
            if len(parts) != 6:
                raise ValueError(
                    f'forward_arc_sectors entry {spec!r} must have 6 comma-'
                    f'separated values (az_min, az_max, r_min, r_max, '
                    f'z_min, z_max); got {len(parts)}')
            az_min_deg, az_max_deg, r_min, r_max, z_min, z_max = map(float, parts)
            if az_min_deg >= az_max_deg:
                raise ValueError(
                    f'forward_arc_sectors entry {spec!r}: az_min '
                    f'({az_min_deg}) must be < az_max ({az_max_deg}); '
                    f'wrap-around sectors are not supported')
            if r_min >= r_max or r_min < 0.0:
                raise ValueError(
                    f'forward_arc_sectors entry {spec!r}: require '
                    f'0 <= r_min < r_max; got r_min={r_min}, r_max={r_max}')
            if z_min >= z_max:
                raise ValueError(
                    f'forward_arc_sectors entry {spec!r}: require '
                    f'z_min < z_max; got z_min={z_min}, z_max={z_max}')
            # Pre-square r and pre-convert az to radians so the per-scan
            # mask in on_cloud avoids sqrt and degrees-to-radians work.
            rows.append((
                np.radians(az_min_deg),
                np.radians(az_max_deg),
                r_min * r_min,
                r_max * r_max,
                z_min,
                z_max,
            ))
        return np.asarray(rows, dtype=np.float32)

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

        # Fast path: cylinder disabled (self_radius=0) AND no sectors → straight
        # passthrough. Keeps the topic alive for downstream comparisons without
        # paying for any per-point math.
        if self.self_radius <= 0.0 and self.sectors is None:
            kept = buf
        else:
            x = field(offsets['x'])
            y = field(offsets['y'])
            z = field(offsets['z'])
            # Compute r² once and reuse for both the cylinder and the sectors;
            # arctan2 is computed only if at least one sector is active.
            r2 = x * x + y * y
            in_self = np.zeros(n_points, dtype=bool)
            if self.self_radius > 0.0:
                in_self |= (
                    (r2 < self.self_radius * self.self_radius)
                    & (z > self.self_z_min)
                    & (z < self.self_z_max)
                )
            if self.sectors is not None:
                az = np.arctan2(y, x)
                for sector in self.sectors:
                    az_min, az_max, r2_min, r2_max, z_min, z_max = sector
                    in_self |= (
                        (az >= az_min) & (az < az_max)
                        & (r2 >= r2_min) & (r2 < r2_max)
                        & (z > z_min) & (z < z_max)
                    )
            kept = buf[~in_self]

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
