# whill_odometry

Wheel-odometry wrapper for `ros2_whill/whill_driver`.

The upstream driver only publishes `/whill/states/model_cr2`
(`whill_msgs/msg/ModelCr2State`). Phase A's `ekf_odom`
(`whill_localization/config/ekf_odom.yaml`) wants
`/whill/odom` (`nav_msgs/msg/Odometry`) for `vx` and `vyaw`. This package
bridges the gap by reproducing the differential-drive integration that
the noetic `ros_whill` driver did internally. See
`docs/legacy-findings/whill-wheel-odometry.md` for the per-line
cross-reference.

## Topics

| direction | topic | type |
|-----------|-------|------|
| sub | `/whill/states/model_cr2` | `whill_msgs/msg/ModelCr2State` |
| pub | `/whill/odom` | `nav_msgs/msg/Odometry` |

`header.frame_id = odom`, `child_frame_id = base_link`.

## Parameters

See `config/whill_odometry.yaml`. The geometry defaults (`wheel_radius`,
`tread`) are from the legacy `WHILL.h`. Covariance stddevs are starting
guesses; retune once you have a drive log.

`publish_tf` defaults to `false` in Phase A because `ekf_odom` owns the
`odom -> base_link` TF. Flip to `true` only if you are running without
the EKF (sanity-check workflows).

## Usage

Standalone:

```
ros2 launch whill_odometry whill_odometry_launch.py
```

The driver itself (`ros2_whill`) must already be publishing
`/whill/states/model_cr2`. In normal operation this launch is included
from `whill_localization/state_estimation_launch.py`, which is in turn
brought up by `whill_navigation/nav_launch.py`.

### `use_sim_time` and bag replay

`nav_launch.py` hardcodes `use_sim_time=false` when including
`state_estimation_launch.py`. That is intentional — `nav_launch.py` is
the live-chair entry point and any outer wrapper passing
`use_sim_time=true` through it would create a mixed-clock setup where
the EKFs trust `/clock` but `whill_odometry` still stamps with wall
`now()`. For offline replay against a `ros2 bag play --clock` source,
invoke `state_estimation_launch.py` directly with
`use_sim_time:=true` and skip `nav_launch.py`.

## Known limitations

- `ModelCr2State` has no `Header` field, so the output `header.stamp` is
  the node's `now()` at callback entry rather than the sensor timestamp.
  Acceptable while the driver publishes immediately after decoding the
  serial packet. The right long-term fix is to add a `Header` upstream
  in `ros2_whill_interfaces`.
- Pose drift over long runs is expected (encoder integration). Phase A's
  `ekf_map` corrects this via FAST-LIO; this node only needs to be
  trustworthy for velocity.
