// SPDX-License-Identifier: BSD-3-Clause
//
// cloud_height_filter_node: transform a PointCloud2 into a chassis frame
// (default base_link) and keep only points inside a height band and a range
// window. Output is xyz-only, in target_frame, with the input stamp.
//
// Why this exists (avoidance baseline, docs/ja/plans/2026-10-28-avoidance-
// baseline.md, C2): STVL and nav2's obstacle sources filter heights in the
// costmap's global frame (`map`). On this site the map frame is not level
// with the road: in the 2026-07-24 campus bag the chair's /pcl_pose z spans
// -1.2 .. +3.8 m and its pitch sits around -8 deg (the known `velodyne` TF
// pitch error baked into the map). A fixed map-frame band would either mark
// the floor or drop the 21 cm objects the depth camera is added for. The
// floor right in front of the chair is, however, close to the base_link
// z = 0 plane, so the band is applied here, in base_link, before STVL.
//
// The sensor -> target transform is looked up at the latest time
// (tf2::TimePointZero): every input this node is used for (velodyne_hires,
// camera_depth_optical_frame) is rigidly mounted and published as static TF,
// so using the message stamp would only add a failure mode (extrapolation
// when the static TF arrives late) without changing the result.
//
// Contract:
//   sub  cloud_in   sensor_msgs/PointCloud2 (SensorDataQoS; accepts reliable)
//   pub  cloud_out  sensor_msgs/PointCloud2 (SensorDataQoS, xyz, target_frame)
// Parameters:
//   target_frame  (string, "base_link")
//   min_z, max_z  (double, m, in target_frame)
//   min_range, max_range (double, m, Euclidean distance from the sensor
//                 origin; max_range <= 0 disables the far limit)

#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include <Eigen/Geometry>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <tf2/exceptions.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

namespace whill_perception {

class CloudHeightFilterNode : public rclcpp::Node
{
public:
  CloudHeightFilterNode()
  : Node("cloud_height_filter_node")
  {
    target_frame_ = declare_parameter<std::string>("target_frame", "base_link");
    min_z_        = declare_parameter<double>("min_z", 0.10);
    max_z_        = declare_parameter<double>("max_z", 1.60);
    min_range_    = declare_parameter<double>("min_range", 0.0);
    max_range_    = declare_parameter<double>("max_range", 0.0);

    tf_buffer_   = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);

    sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      "cloud_in", rclcpp::SensorDataQoS(),
      std::bind(&CloudHeightFilterNode::OnCloud, this, std::placeholders::_1));
    // Best-effort like every other cloud in this stack; STVL and the
    // Collision Monitor subscribe with SensorDataQoS.
    pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      "cloud_out", rclcpp::SensorDataQoS());

    RCLCPP_INFO(get_logger(),
      "cloud_height_filter_node ready — target_frame=%s, z [%.3f, %.3f] m, "
      "range [%.2f, %.2f] m",
      target_frame_.c_str(), min_z_, max_z_, min_range_, max_range_);
  }

private:
  void OnCloud(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg)
  {
    geometry_msgs::msg::TransformStamped tf;
    try {
      tf = tf_buffer_->lookupTransform(target_frame_, msg->header.frame_id,
                                       tf2::TimePointZero);
    } catch (const tf2::TransformException & e) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
        "no TF %s <- %s yet: %s", target_frame_.c_str(),
        msg->header.frame_id.c_str(), e.what());
      return;
    }
    const auto & q = tf.transform.rotation;
    const auto & t = tf.transform.translation;
    const Eigen::Matrix3f R =
      Eigen::Quaternionf(q.w, q.x, q.y, q.z).normalized().toRotationMatrix();
    const Eigen::Vector3f T(t.x, t.y, t.z);

    const float min_r2 = static_cast<float>(min_range_ * min_range_);
    const float max_r2 = max_range_ > 0.0 ?
      static_cast<float>(max_range_ * max_range_) : INFINITY;
    const float zlo = static_cast<float>(min_z_);
    const float zhi = static_cast<float>(max_z_);

    const size_t n = static_cast<size_t>(msg->height) * msg->width;
    kept_.clear();
    kept_.reserve(n / 4);
    sensor_msgs::PointCloud2ConstIterator<float> it_x(*msg, "x");
    sensor_msgs::PointCloud2ConstIterator<float> it_y(*msg, "y");
    sensor_msgs::PointCloud2ConstIterator<float> it_z(*msg, "z");
    for (size_t i = 0; i < n; ++i, ++it_x, ++it_y, ++it_z) {
      const Eigen::Vector3f p(*it_x, *it_y, *it_z);
      // Organized clouds (velodyne organize_cloud, realsense) carry NaN or
      // zero placeholders for missing returns; drop both.
      if (!p.allFinite()) {
        continue;
      }
      const float r2 = p.squaredNorm();
      if (r2 <= 0.0f || r2 < min_r2 || r2 > max_r2) {
        continue;
      }
      const Eigen::Vector3f b = R * p + T;
      if (b.z() < zlo || b.z() > zhi) {
        continue;
      }
      kept_.push_back(b);
    }

    sensor_msgs::msg::PointCloud2 out;
    out.header.stamp = msg->header.stamp;
    out.header.frame_id = target_frame_;
    out.height = 1;
    sensor_msgs::PointCloud2Modifier mod(out);
    mod.setPointCloud2FieldsByString(1, "xyz");
    mod.resize(kept_.size());
    sensor_msgs::PointCloud2Iterator<float> o_x(out, "x");
    sensor_msgs::PointCloud2Iterator<float> o_y(out, "y");
    sensor_msgs::PointCloud2Iterator<float> o_z(out, "z");
    for (const auto & b : kept_) {
      *o_x = b.x();
      *o_y = b.y();
      *o_z = b.z();
      ++o_x; ++o_y; ++o_z;
    }
    // An empty cloud is still published: STVL's clearing frustum and the
    // staleness checks downstream need "seen, nothing there" to differ
    // from "no data".
    pub_->publish(std::move(out));
  }

  std::string target_frame_;
  double min_z_, max_z_, min_range_, max_range_;
  std::vector<Eigen::Vector3f> kept_;

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;
};

}  // namespace whill_perception

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<whill_perception::CloudHeightFilterNode>());
  rclcpp::shutdown();
  return 0;
}
