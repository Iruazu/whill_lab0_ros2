// SPDX-License-Identifier: BSD-3-Clause
//
// ROS 2 wrapper around the Patchwork++ ground segmentation C++ core
// (src/third_party/patchwork_plusplus/cpp/, BSD-2-Clause). This wrapper
// is BSD-3-Clause and does not include upstream's ros/ subtree, whose
// license story is inconsistent (ros/package.xml GPL-3.0 vs ros/LICENSE
// MIT). Only the BSD core is linked.
//
// Contract:
//   sub  cloud_in         sensor_msgs/PointCloud2  (VLP-16, ~10 Hz)
//   pub  cloud_no_ground  sensor_msgs/PointCloud2  (~10 Hz, xyz only,
//                                                    same header frame_id)
//
// The point cloud is copied into an Eigen matrix (Nx3), handed to
// PatchWorkpp::estimateGround, and the getNonground() Nx3 output is
// converted back to a PointCloud2. Ground return / intensity fields
// downstream of xyz are dropped by design — obstacle_layer only needs
// the geometry.

#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <patchwork/patchworkpp.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

namespace whill_perception {

namespace {

Eigen::MatrixXf PointCloud2ToEigenXYZ(const sensor_msgs::msg::PointCloud2 & msg)
{
  const size_t n = static_cast<size_t>(msg.height) * static_cast<size_t>(msg.width);
  Eigen::MatrixXf out(n, 3);
  sensor_msgs::PointCloud2ConstIterator<float> it_x(msg, "x");
  sensor_msgs::PointCloud2ConstIterator<float> it_y(msg, "y");
  sensor_msgs::PointCloud2ConstIterator<float> it_z(msg, "z");
  for (size_t i = 0; i < n; ++i, ++it_x, ++it_y, ++it_z) {
    out(i, 0) = *it_x;
    out(i, 1) = *it_y;
    out(i, 2) = *it_z;
  }
  return out;
}

sensor_msgs::msg::PointCloud2 EigenXYZToPointCloud2(
  const Eigen::MatrixX3f & pts, const std_msgs::msg::Header & header)
{
  sensor_msgs::msg::PointCloud2 msg;
  msg.header = header;
  msg.height = 1;
  msg.width  = static_cast<uint32_t>(pts.rows());
  sensor_msgs::PointCloud2Modifier mod(msg);
  mod.setPointCloud2FieldsByString(1, "xyz");
  mod.resize(pts.rows());

  sensor_msgs::PointCloud2Iterator<float> it_x(msg, "x");
  sensor_msgs::PointCloud2Iterator<float> it_y(msg, "y");
  sensor_msgs::PointCloud2Iterator<float> it_z(msg, "z");
  for (Eigen::Index i = 0; i < pts.rows(); ++i, ++it_x, ++it_y, ++it_z) {
    *it_x = pts(i, 0);
    *it_y = pts(i, 1);
    *it_z = pts(i, 2);
  }
  return msg;
}

}  // namespace

class PatchworkPpNode : public rclcpp::Node
{
public:
  PatchworkPpNode()
  : Node("patchworkpp_node"), frames_since_log_(0)
  {
    const auto params = LoadParams();
    patchwork_        = std::make_unique<patchwork::PatchWorkpp>(params);

    stats_log_period_ = declare_parameter<int>("stats_log_period_frames", 100);

    sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      "cloud_in", rclcpp::SensorDataQoS(),
      std::bind(&PatchworkPpNode::OnCloud, this, std::placeholders::_1));

    // Publish on SensorDataQoS so downstream (pointcloud_to_laserscan)
    // sees the same best-effort characteristics as /velodyne_points.
    pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      "cloud_no_ground", rclcpp::SensorDataQoS());

    RCLCPP_INFO(get_logger(),
      "patchworkpp_node ready — sensor_height=%.2f, min_range=%.2f, max_range=%.2f, "
      "RNR=%d RVPF=%d TGR=%d, stats every %d frames",
      params.sensor_height, params.min_range, params.max_range,
      static_cast<int>(params.enable_RNR),
      static_cast<int>(params.enable_RVPF),
      static_cast<int>(params.enable_TGR),
      stats_log_period_);
  }

private:
  patchwork::Params LoadParams()
  {
    // Struct-default constructor gives Patchwork++'s recommended values;
    // declare_parameter with the struct default lets a yaml override any
    // individual field without redeclaring the full set here.
    patchwork::Params p;
    p.verbose               = declare_parameter<bool>("verbose", p.verbose);
    p.enable_RNR            = declare_parameter<bool>("enable_RNR", p.enable_RNR);
    p.enable_RVPF           = declare_parameter<bool>("enable_RVPF", p.enable_RVPF);
    p.enable_TGR            = declare_parameter<bool>("enable_TGR", p.enable_TGR);
    p.num_iter              = declare_parameter<int>("num_iter", p.num_iter);
    p.num_lpr               = declare_parameter<int>("num_lpr", p.num_lpr);
    p.num_min_pts           = declare_parameter<int>("num_min_pts", p.num_min_pts);
    p.num_zones             = declare_parameter<int>("num_zones", p.num_zones);
    p.num_rings_of_interest = declare_parameter<int>("num_rings_of_interest", p.num_rings_of_interest);
    p.RNR_ver_angle_thr     = declare_parameter<double>("RNR_ver_angle_thr", p.RNR_ver_angle_thr);
    p.RNR_intensity_thr     = declare_parameter<double>("RNR_intensity_thr", p.RNR_intensity_thr);
    p.sensor_height         = declare_parameter<double>("sensor_height", p.sensor_height);
    p.th_seeds              = declare_parameter<double>("th_seeds", p.th_seeds);
    p.th_dist               = declare_parameter<double>("th_dist", p.th_dist);
    p.th_seeds_v            = declare_parameter<double>("th_seeds_v", p.th_seeds_v);
    p.th_dist_v             = declare_parameter<double>("th_dist_v", p.th_dist_v);
    p.max_range             = declare_parameter<double>("max_range", p.max_range);
    p.min_range             = declare_parameter<double>("min_range", p.min_range);
    p.uprightness_thr       = declare_parameter<double>("uprightness_thr", p.uprightness_thr);
    p.adaptive_seed_selection_margin = declare_parameter<double>(
      "adaptive_seed_selection_margin", p.adaptive_seed_selection_margin);
    p.intensity_thr         = declare_parameter<double>("intensity_thr", p.intensity_thr);
    // num_sectors_each_zone / num_rings_each_zone are vector<int> — leave
    // them at struct defaults for the first iteration. ROS 2 humble's
    // declare_parameter for vectors uses int64_t and the conversion path
    // adds surface area we do not need until CZM re-tuning is a real ask.
    return p;
  }

  void OnCloud(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg)
  {
    Eigen::MatrixXf cloud = PointCloud2ToEigenXYZ(*msg);
    if (cloud.rows() == 0) {
      RCLCPP_WARN(get_logger(), "empty PointCloud2 — passing through as empty non-ground");
      pub_->publish(*msg);
      return;
    }

    patchwork_->estimateGround(cloud);
    const Eigen::MatrixX3f nonground = patchwork_->getNonground();
    const double time_taken_ms       = static_cast<double>(patchwork_->getTimeTaken()) / 1000.0;

    auto out = EigenXYZToPointCloud2(nonground, msg->header);
    pub_->publish(std::move(out));

    if (++frames_since_log_ >= stats_log_period_) {
      RCLCPP_INFO(get_logger(),
        "in %ld pts / non-ground %ld pts / %.1f ms (avg over last %d frames)",
        static_cast<long>(cloud.rows()),
        static_cast<long>(nonground.rows()),
        time_taken_ms,
        stats_log_period_);
      frames_since_log_ = 0;
    }
  }

  std::unique_ptr<patchwork::PatchWorkpp> patchwork_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr    pub_;

  int stats_log_period_;
  int frames_since_log_;
};

}  // namespace whill_perception

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<whill_perception::PatchworkPpNode>());
  rclcpp::shutdown();
  return 0;
}
