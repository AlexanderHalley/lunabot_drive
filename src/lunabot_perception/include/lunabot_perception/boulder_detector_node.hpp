// Copyright 2027 Lunabot
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef LUNABOT_PERCEPTION__BOULDER_DETECTOR_NODE_HPP_
#define LUNABOT_PERCEPTION__BOULDER_DETECTOR_NODE_HPP_

#include <memory>
#include <string>

#include "lunabot_perception/cloud_segmentation.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "vision_msgs/msg/detection3_d_array.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

namespace lunabot_perception
{

/// Detects boulders in the OAK-D's point cloud.
///
///   in:  /oak_d/points          sensor_msgs/PointCloud2, optical frame
///   out: /perception/boulders   vision_msgs/Detection3DArray, base_link
///
/// ==================== ON THE MESSAGE TYPE ====================
/// vision_msgs/Detection3DArray, not a custom lunabot_msgs/BoulderArray.
///
/// The decisive argument is craters. ObjectHypothesis::class_id is a STRING,
/// so "boulder" and "crater" are values of an existing field -- adding crater
/// detection later needs no message change and no downstream rebuild. On top
/// of that it has RViz plugins and it is what Isaac ROS and the Nav2
/// ecosystem already speak.
/// =============================================================
///
/// ==================== ON THE INPUT TOPIC ====================
/// The point cloud, not the depth image.
///
/// The depth image encoding DIFFERS between hardware and sim: 16UC1
/// millimetres from the OAK-D, 32FC1 metres from Isaac. Point clouds are
/// metres in both cases, so this node sees byte-comparable input either way
/// and there is no 1000x trap waiting in it.
/// See docs/TOPIC_FRAME_CONTRACT.md.
/// ============================================================
class BoulderDetectorNode : public rclcpp::Node
{
public:
  explicit BoulderDetectorNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void on_cloud(const sensor_msgs::msg::PointCloud2::ConstSharedPtr & msg);

  void publish_debug(
    const GroundSplit & split, const std::vector<Cluster> & clusters, const rclcpp::Time & stamp);

  /// Reloads parameters that are cheap to change while running.
  void read_parameters();

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Publisher<vision_msgs::msg::Detection3DArray>::SharedPtr detections_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr ground_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr obstacles_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr markers_pub_;

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  std::string output_frame_;
  double voxel_leaf_size_ = 0.03;
  Eigen::Vector3f roi_min_;
  Eigen::Vector3f roi_max_;
  GroundParameters ground_;
  ClusterParameters clustering_;
  DimensionFilter dimensions_;

  /// Constant, and deliberately not called a confidence. Nothing here
  /// estimates one; reporting a varying number would imply a classifier that
  /// does not exist.
  double detection_score_ = 0.5;

  bool publish_debug_clouds_ = true;
};

}  // namespace lunabot_perception

#endif  // LUNABOT_PERCEPTION__BOULDER_DETECTOR_NODE_HPP_
