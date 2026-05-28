// Copyright (c) 2026, WHILL lab0.
// Licensed under the BSD-3-Clause License (see LICENSE).
//
// Differential-drive wheel odometry for the WHILL Model CR2, derived from
// the noetic ros_whill driver's odom.cpp + ros_whill.cpp. The legacy
// findings document (docs/legacy-findings/whill-wheel-odometry.md) carries
// the file:line cross-references and the rationale for every magic number;
// this implementation tries not to repeat that information here.

#include "whill_odometry/odometry_node.hpp"

#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <utility>

#include "geometry_msgs/msg/quaternion.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace whill_odometry
{

namespace
{
// Large covariance for axes we explicitly do not estimate. Matches what
// robot_localization uses internally to mean "ignore". Going much higher
// (>1e9) sometimes destabilises r_l's matrix inversion in our experience.
constexpr double kLargeVariance = 1e6;

// Throttle period for the dt warnings. ModelCr2State arrives at ~50 Hz, so
// a 1 Hz throttle keeps a backed-up driver from spamming the log.
constexpr int kWarnThrottleMs = 1000;

// Hard upper bound on dt before we refuse to integrate position. Picked at
// 1.0 s because a hot-pluggable WHILL serial reconnect can lose ~hundreds
// of milliseconds of packets; beyond a second the encoder rollover (±32.767
// rad) can wrap silently and the angle diff becomes meaningless.
constexpr double kMaxDtSec = 1.0;
}  // namespace

OdometryNode::OdometryNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("whill_odometry", options)
{
  // Wheel geometry defaults pulled verbatim from the legacy WHILL.h
  // (wheel_radius=0.1325 m, tread=0.496 m). Exposed as parameters because
  // the noetic build hardcoded them and we want to be able to retune
  // without a rebuild once we have live data.
  wheel_radius_ = declare_parameter<double>("wheel_radius", 0.1325);
  tread_ = declare_parameter<double>("tread", 0.496);

  // Phase A: ekf_odom is the canonical publisher of odom -> base_link, so
  // this node defaults to *not* broadcasting TF. Flip to true if you want
  // to drive the chair on wheel odom alone (sanity-check workflows, or any
  // configuration where ekf_odom is disabled).
  publish_tf_ = declare_parameter<bool>("publish_tf", false);

  // Defaults of √0.001 mirror the diagonal pose/twist covariance the legacy
  // findings doc suggests as a starting point. Yaw rate is given a looser
  // bound (0.1 ≈ √0.01) because angle-diff differentiation amplifies
  // encoder quantisation noise.
  pose_stddev_xy_ = declare_parameter<double>("pose_stddev_xy", 0.0316);
  pose_stddev_yaw_ = declare_parameter<double>("pose_stddev_yaw", 0.0316);
  twist_stddev_vx_ = declare_parameter<double>("twist_stddev_vx", 0.0316);
  twist_stddev_vyaw_ = declare_parameter<double>("twist_stddev_vyaw", 0.1);

  frame_id_ = declare_parameter<std::string>("frame_id", "odom");
  child_frame_id_ = declare_parameter<std::string>("child_frame_id", "base_link");

  // ModelCr2State arrives at ~50 Hz. Upstream `whill_driver` publishes
  // with the default rmw QoS (reliable, history-10), so we explicitly
  // match `reliable` here. A SensorDataQoS / best-effort subscription
  // would mismatch and rmw silently drops the connection — the node
  // would start cleanly, print no error, and simply never receive a
  // ModelCr2State sample. Keep this in lockstep with the upstream's
  // QoS profile rather than tracking sample-rate intuition.
  state_sub_ = create_subscription<whill_msgs::msg::ModelCr2State>(
    "/whill/states/model_cr2",
    rclcpp::QoS(10).reliable(),
    std::bind(&OdometryNode::OnModelCr2State, this, std::placeholders::_1));

  // Latched-ish QoS for odom is not standard — r_l + Nav2 expect plain
  // reliable history-10. Keep it that way.
  odom_pub_ = create_publisher<nav_msgs::msg::Odometry>(
    "/whill/odom", rclcpp::QoS(10).reliable());

  if (publish_tf_) {
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
  }

  RCLCPP_INFO(
    get_logger(),
    "whill_odometry up: wheel_radius=%.4f m, tread=%.4f m, publish_tf=%s",
    wheel_radius_, tread_, publish_tf_ ? "true" : "false");
}

double OdometryNode::WrappedAngleDiff(double past, double current)
{
  // Equivalent to the legacy rad_diff(past, current) in
  // utils/rotation_tools.cpp. WHILL emits motor angle as int16 * 0.001 rad,
  // so the value rolls over at ±32.767 rad — atan2(sin(d), cos(d)) folds
  // the rollover discontinuity back into ±π.
  //
  // Order matters: WHILL's motor angle *decreases* on forward roll (the
  // upstream firmware sign convention, see legacy `rotation_tools.cpp:30-41`
  // and `odom.cpp:75-76`). With `past - current`, a forward roll therefore
  // yields a positive d, which then maps to a positive vr = d * r / dt and
  // a positive vx in OnModelCr2State — i.e. ROS-standard "+x is forward".
  // Reversing to `current - past` would silently flip every velocity sign;
  // the EKF would still converge but the chair would drive backwards.
  const double d = past - current;
  return std::atan2(std::sin(d), std::cos(d));
}

double OdometryNode::WrapToPi(double angle)
{
  return std::atan2(std::sin(angle), std::cos(angle));
}

void OdometryNode::OnModelCr2State(
  const whill_msgs::msg::ModelCr2State::SharedPtr msg)
{
  // whill_msgs/ModelCr2State has no header (verified against the .msg in
  // src/third_party/ros2_whill_interfaces). The original task spec asked
  // for `header.stamp` propagation, but that field does not exist. The
  // best we can do is the node's notion of `now()` at callback entry —
  // close enough because the upstream whill_driver publishes on a wall
  // timer that fires immediately after decoding the serial packet, so the
  // delta between sensor sample time and our callback is dominated by
  // the same ~20 ms serial polling interval on both sides. If higher
  // accuracy is ever needed, the right fix is to patch ros2_whill to add
  // a Header field upstream — tracked as a known limitation.
  const rclcpp::Time now = this->now();

  // First sample: nothing to diff against. Cache and return without
  // emitting an Odometry, otherwise dt would be undefined.
  if (!prev_.has_value()) {
    prev_ = PrevSample{
      static_cast<double>(msg->right_motor_angle),
      static_cast<double>(msg->left_motor_angle),
      now,
    };
    return;
  }

  const double dt = (now - prev_->stamp).seconds();

  // dt <= 0: time went backwards. This can happen on a clock rewind
  // (e.g. simulated time jump during bag replay) or two ModelCr2State
  // messages with identical receive timestamps. Skip integration but
  // refresh the cached angle so the *next* diff is taken across the
  // correct interval — otherwise the next valid dt would carry the
  // accumulated angle change of two packets and overshoot the velocity.
  if (dt <= 0.0) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), kWarnThrottleMs,
      "Non-positive dt=%f s; skipping integration", dt);
    prev_ = PrevSample{
      static_cast<double>(msg->right_motor_angle),
      static_cast<double>(msg->left_motor_angle),
      now,
    };
    return;
  }

  // dt too large: the angle diff cannot reliably tell us whether the
  // encoder rolled over an integer number of times. Update prev_ so we
  // resync on the next packet, but do not integrate pose. We do NOT
  // publish anything in this branch: emitting a fabricated twist=0
  // sample would feed bogus zero-velocity evidence into ekf_odom right
  // at the moment we are *least* sure of the chair's motion. Instead we
  // stay silent and rely on r_l's per-sensor `sensor_timeout` (0.1 s,
  // see ekf_odom.yaml) — once that elapses, the EKF gracefully drops
  // /whill/odom from the fusion and continues on IMU + LIO alone until
  // the next valid wheel sample arrives. Skipping integration here
  // prevents a single bad sample from corrupting the long-term pose.
  if (dt > kMaxDtSec) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), kWarnThrottleMs,
      "Large dt=%f s (> %.2f); resyncing without integrating pose",
      dt, kMaxDtSec);
    prev_ = PrevSample{
      static_cast<double>(msg->right_motor_angle),
      static_cast<double>(msg->left_motor_angle),
      now,
    };
    return;
  }

  // Per legacy odom.cpp:75-92. The left wheel sign flip is the trap from
  // legacy-findings F-1: rad_diff is past-minus-current, which makes a
  // forward roll yield a positive diff on both wheels; the left wheel must
  // be negated so that (vr - vl) / tread yields ROS-standard CCW-positive
  // angular velocity.
  const double d_right = WrappedAngleDiff(
    prev_->right_motor_angle, static_cast<double>(msg->right_motor_angle));
  const double d_left = -WrappedAngleDiff(
    prev_->left_motor_angle, static_cast<double>(msg->left_motor_angle));

  const double vr = (d_right / dt) * wheel_radius_;
  const double vl = (d_left / dt) * wheel_radius_;

  const double v_linear = 0.5 * (vr + vl);
  const double v_angular = (vr - vl) / tread_;

  // Midpoint integration (matches legacy odom.cpp:88-90).
  x_ += v_linear * dt * std::cos(theta_ + 0.5 * v_angular * dt);
  y_ += v_linear * dt * std::sin(theta_ + 0.5 * v_angular * dt);
  theta_ = WrapToPi(theta_ + v_angular * dt);

  // Save for next iteration before we touch the message any further.
  prev_ = PrevSample{
    static_cast<double>(msg->right_motor_angle),
    static_cast<double>(msg->left_motor_angle),
    now,
  };

  // ---- Build Odometry ----
  nav_msgs::msg::Odometry odom;
  odom.header.stamp = now;
  odom.header.frame_id = frame_id_;
  odom.child_frame_id = child_frame_id_;

  odom.pose.pose.position.x = x_;
  odom.pose.pose.position.y = y_;
  odom.pose.pose.position.z = 0.0;  // see legacy F-3: drop the base_link_height hack.

  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, theta_);
  odom.pose.pose.orientation.x = q.x();
  odom.pose.pose.orientation.y = q.y();
  odom.pose.pose.orientation.z = q.z();
  odom.pose.pose.orientation.w = q.w();

  // 6x6 row-major. Indices [0]=xx, [7]=yy, [14]=zz, [21]=rr, [28]=pp, [35]=yy.
  const double pose_var_xy = pose_stddev_xy_ * pose_stddev_xy_;
  const double pose_var_yaw = pose_stddev_yaw_ * pose_stddev_yaw_;
  odom.pose.covariance[0] = pose_var_xy;
  odom.pose.covariance[7] = pose_var_xy;
  odom.pose.covariance[14] = kLargeVariance;
  odom.pose.covariance[21] = kLargeVariance;
  odom.pose.covariance[28] = kLargeVariance;
  odom.pose.covariance[35] = pose_var_yaw;

  odom.twist.twist.linear.x = v_linear;
  odom.twist.twist.angular.z = v_angular;

  const double twist_var_vx = twist_stddev_vx_ * twist_stddev_vx_;
  const double twist_var_vyaw = twist_stddev_vyaw_ * twist_stddev_vyaw_;
  odom.twist.covariance[0] = twist_var_vx;
  odom.twist.covariance[7] = kLargeVariance;
  odom.twist.covariance[14] = kLargeVariance;
  odom.twist.covariance[21] = kLargeVariance;
  odom.twist.covariance[28] = kLargeVariance;
  odom.twist.covariance[35] = twist_var_vyaw;

  odom_pub_->publish(odom);

  if (publish_tf_) {
    geometry_msgs::msg::TransformStamped tf_msg;
    tf_msg.header.stamp = now;
    tf_msg.header.frame_id = frame_id_;
    tf_msg.child_frame_id = child_frame_id_;
    tf_msg.transform.translation.x = x_;
    tf_msg.transform.translation.y = y_;
    tf_msg.transform.translation.z = 0.0;
    tf_msg.transform.rotation = odom.pose.pose.orientation;
    tf_broadcaster_->sendTransform(tf_msg);
  }
}

}  // namespace whill_odometry

RCLCPP_COMPONENTS_REGISTER_NODE(whill_odometry::OdometryNode)
