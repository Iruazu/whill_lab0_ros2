"""TF bridge between FAST-LIO and Nav2 — deprecated as of Phase A (2026-05-28).

History (kept here so future readers understand the chain in old bags):

  M5-a baseline    : map -> camera_init (identity)
                     camera_init -> body (FAST-LIO at runtime)
                     body -> base_link (identity, when base_link == IMU)
  Phase A-2        : body -> base_link became a real (-0.20, 0, -0.42) static
                     offset once the URDF gained chair geometry.
  Phase A (Nav2-std): both static transforms in this file are removed. The
                     standard Nav2 chain (map -> odom -> base_link) is now
                     supplied by the robot_localization two-stage EKF in
                     `whill_localization/launch/state_estimation_launch.py`
                     (see ADR-0001). FAST-LIO's native `camera_init -> body`
                     branch still exists at runtime but is no longer attached
                     to `map` — it is a dangling subtree that Nav2 does not
                     consult; FAST-LIO's pose reaches Nav2 only via the
                     /Odometry topic which ekf_map subscribes to.

Why this file still exists (instead of being deleted outright):
  `whill_navigation/launch/nav_launch.py` includes it via
  IncludeLaunchDescription. Rather than churn the top-level launch graph in
  the same Phase A change that introduces the EKF, this file is left as an
  empty LaunchDescription. nav_launch.py's include of it becomes a no-op.
  A follow-up change (Phase B or earlier cleanup) should remove both the
  include and this file.

If you need to revive a static `map -> odom` (e.g. you are debugging without
the EKF and want Nav2 to see _some_ map-frame chain), do it in a private
overlay rather than re-editing this file — the whole point of Phase A is
that Nav2 sees the EKF output, not a hardcoded identity.
"""

from launch import LaunchDescription


def generate_launch_description():
    return LaunchDescription([])
