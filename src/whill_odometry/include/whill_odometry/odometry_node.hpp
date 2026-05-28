// Copyright (c) 2026, WHILL lab0.
// Licensed under the BSD-3-Clause License (see LICENSE).

#ifndef WHILL_ODOMETRY__ODOMETRY_NODE_HPP_
#define WHILL_ODOMETRY__ODOMETRY_NODE_HPP_

#include <memory>
#include <optional>
#include <string>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "whill_msgs/msg/model_cr2_state.hpp"

namespace whill_odometry
{

// Wraps ros2_whill/whill_driver's /whill/states/model_cr2 (which carries
// motor encoder angles but no Odometry) and reproduces the differential
// drive integration from the legacy ros_whill (noetic) odom.cpp on the
// ROS 2 side. Output topic is /whill/odom, matching what ekf_odom expects.
//
// Why a separate package rather than living in whill_localization: the
// localization package is responsible for LIO / EKF fusion. Encoder
// integration is a driver-adjacent concern that belongs upstream of the
// fusion stage. Keeping them split also means whill_odometry can be reused
// without dragging in fast_lio / robot_localization.
class OdometryNode : public rclcpp::Node
{
public:
  explicit OdometryNode(const rclcpp::NodeOptions & options);

private:
  // Persisted between callbacks. wrapped in optional so we can detect the
  // very first sample (when there is no previous angle / stamp to diff
  // against) and skip output without writing a special "first sample" flag.
  struct PrevSample
  {
    double right_motor_angle;  // [rad], from ModelCr2State.right_motor_angle
    double left_motor_angle;   // [rad], from ModelCr2State.left_motor_angle
    rclcpp::Time stamp;        // receive time (see ctor comment about why
                               // we cannot use msg.header.stamp).
  };

  void OnModelCr2State(const whill_msgs::msg::ModelCr2State::SharedPtr msg);

  // ±π wrap-aware difference: atan2(sin(past - current), cos(past - current))
  // is equivalent and handles the int16-rad rollover at ±32.767 rad that the
  // WHILL serial protocol emits. Used for both wheels.
  static double WrappedAngleDiff(double past, double current);

  // Wrap [-π, π]. yaw is integrated unbounded otherwise.
  static double WrapToPi(double angle);

  // Subscribers / publishers / TF.
  rclcpp::Subscription<whill_msgs::msg::ModelCr2State>::SharedPtr state_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  // Parameters cached at ctor.
  double wheel_radius_;       // [m]
  double tread_;              // [m]
  bool publish_tf_;
  double pose_stddev_xy_;
  double pose_stddev_yaw_;
  double twist_stddev_vx_;
  double twist_stddev_vyaw_;
  std::string frame_id_;
  std::string child_frame_id_;

  // Integrated pose (in `frame_id_`). Reset is not yet wired — Phase A only
  // needs vx/vyaw out of /whill/odom, so absolute pose drift is acceptable.
  double x_{0.0};
  double y_{0.0};
  double theta_{0.0};

  std::optional<PrevSample> prev_;
};

}  // namespace whill_odometry

#endif  // WHILL_ODOMETRY__ODOMETRY_NODE_HPP_
