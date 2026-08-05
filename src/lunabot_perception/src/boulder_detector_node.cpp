// Copyright 2027 Lunabot. Licensed under the MIT License.

#include "lunabot_perception/boulder_detector_node.hpp"

#include <memory>
#include <string>
#include <vector>

#include <pcl_conversions/pcl_conversions.h>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "pcl_ros/transforms.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "vision_msgs/msg/object_hypothesis_with_pose.hpp"

namespace lunabot_perception
{

BoulderDetectorNode::BoulderDetectorNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("boulder_detector", options)
{
  declare_parameter("output_frame", "base_link");
  declare_parameter("voxel_leaf_size", 0.03);

  // Region of interest in base_link. Defaults are shaped by the camera:
  // tilted down, clipped at 2 m, so anything beyond that is not measured and
  // anything behind the front bumper is the rover looking at itself.
  declare_parameter("roi_min", std::vector<double>{0.2, -1.5, -0.5});
  declare_parameter("roi_max", std::vector<double>{2.5, 1.5, 1.0});

  declare_parameter("ground_z", -0.10);
  declare_parameter("ground_plane_distance_threshold", 0.05);

  declare_parameter("cluster_tolerance", 0.10);
  declare_parameter("min_cluster_size", 20);
  declare_parameter("max_cluster_size", 25000);

  declare_parameter("min_boulder_dimension", 0.05);
  declare_parameter("max_boulder_dimension", 1.50);
  declare_parameter("min_boulder_height", 0.03);

  declare_parameter("detection_score", 0.5);
  declare_parameter("publish_debug_clouds", true);

  read_parameters();

  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  detections_pub_ =
    create_publisher<vision_msgs::msg::Detection3DArray>("/perception/boulders", 10);

  if (publish_debug_clouds_) {
    ground_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("/perception/debug/ground", 1);
    obstacles_pub_ =
      create_publisher<sensor_msgs::msg::PointCloud2>("/perception/debug/obstacles", 1);
    markers_pub_ =
      create_publisher<visualization_msgs::msg::MarkerArray>("/perception/debug/markers", 1);
  }

  // Best-effort with a depth of 1. Point clouds are large and this pipeline
  // is stateless per frame, so a stale cloud is worth less than the bandwidth
  // spent redelivering it. Also matches how sensor data is published on both
  // the real driver and Isaac.
  cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
    "/oak_d/points", rclcpp::SensorDataQoS(),
    std::bind(&BoulderDetectorNode::on_cloud, this, std::placeholders::_1));

  RCLCPP_INFO(
    get_logger(),
    "boulder_detector started. THIS IS A GEOMETRIC PLACEHOLDER: it reports lumps above a "
    "flat plane, with a constant score and no classifier. See src/cloud_segmentation.cpp.");
}

void BoulderDetectorNode::read_parameters()
{
  output_frame_ = get_parameter("output_frame").as_string();
  voxel_leaf_size_ = get_parameter("voxel_leaf_size").as_double();

  const auto roi_min = get_parameter("roi_min").as_double_array();
  const auto roi_max = get_parameter("roi_max").as_double_array();
  roi_min_ = Eigen::Vector3f(roi_min[0], roi_min[1], roi_min[2]);
  roi_max_ = Eigen::Vector3f(roi_max[0], roi_max[1], roi_max[2]);

  ground_.ground_z = get_parameter("ground_z").as_double();
  ground_.plane_distance_threshold = get_parameter("ground_plane_distance_threshold").as_double();

  clustering_.tolerance = get_parameter("cluster_tolerance").as_double();
  clustering_.min_points = get_parameter("min_cluster_size").as_int();
  clustering_.max_points = get_parameter("max_cluster_size").as_int();

  dimensions_.min_dimension = get_parameter("min_boulder_dimension").as_double();
  dimensions_.max_dimension = get_parameter("max_boulder_dimension").as_double();
  dimensions_.min_height = get_parameter("min_boulder_height").as_double();

  detection_score_ = get_parameter("detection_score").as_double();
  publish_debug_clouds_ = get_parameter("publish_debug_clouds").as_bool();
}

void BoulderDetectorNode::on_cloud(const sensor_msgs::msg::PointCloud2::ConstSharedPtr & msg)
{
  // The cloud arrives in the camera's OPTICAL frame (Z forward, X right,
  // Y down). Everything downstream reasons in base_link. Doing this first
  // means the ROI and the ground threshold are expressed in robot terms,
  // which is the only way they are comprehensible.
  sensor_msgs::msg::PointCloud2 transformed;
  try {
    if (!pcl_ros::transformPointCloud(output_frame_, *msg, transformed, *tf_buffer_)) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000, "no transform from '%s' to '%s' yet",
        msg->header.frame_id.c_str(), output_frame_.c_str());
      return;
    }
  } catch (const tf2::TransformException & e) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "transform failed: %s", e.what());
    return;
  }

  auto cloud = std::make_shared<Cloud>();
  pcl::fromROSMsg(transformed, *cloud);
  if (cloud->points.empty()) {
    return;
  }

  const auto downsampled = downsample(cloud, voxel_leaf_size_);
  const auto cropped = crop(downsampled, roi_min_, roi_max_);
  const auto split = split_by_ground(cropped, ground_);
  const auto clusters = extract_clusters(split.above_ground, clustering_);
  const auto boulders = filter_by_dimensions(clusters, dimensions_);

  vision_msgs::msg::Detection3DArray detections;
  detections.header.stamp = msg->header.stamp;
  detections.header.frame_id = output_frame_;
  detections.detections.reserve(boulders.size());

  for (const auto & boulder : boulders) {
    vision_msgs::msg::Detection3D detection;
    detection.header = detections.header;

    detection.bbox.center.position.x = boulder.centroid.x();
    detection.bbox.center.position.y = boulder.centroid.y();
    detection.bbox.center.position.z = boulder.centroid.z();
    // Identity orientation: the box is axis-aligned in base_link. An
    // oriented box needs PCA and a convention for which axis is "long".
    detection.bbox.center.orientation.w = 1.0;

    detection.bbox.size.x = boulder.dimensions.x();
    detection.bbox.size.y = boulder.dimensions.y();
    detection.bbox.size.z = boulder.dimensions.z();

    vision_msgs::msg::ObjectHypothesisWithPose hypothesis;
    // A string. When crater detection lands it publishes "crater" here, on
    // this same topic, with no message change and no downstream rebuild.
    hypothesis.hypothesis.class_id = "boulder";
    hypothesis.hypothesis.score = detection_score_;
    hypothesis.pose.pose = detection.bbox.center;
    detection.results.push_back(hypothesis);

    detections.detections.push_back(detection);
  }

  detections_pub_->publish(detections);

  if (publish_debug_clouds_) {
    publish_debug(split, boulders, msg->header.stamp);
  }
}

void BoulderDetectorNode::publish_debug(
  const GroundSplit & split, const std::vector<Cluster> & clusters, const rclcpp::Time & stamp)
{
  auto to_msg = [&](const Cloud::Ptr & cloud) {
    sensor_msgs::msg::PointCloud2 msg;
    pcl::toROSMsg(*cloud, msg);
    msg.header.stamp = stamp;
    msg.header.frame_id = output_frame_;
    return msg;
  };

  if (ground_pub_->get_subscription_count() > 0) {
    ground_pub_->publish(to_msg(split.ground));
  }
  if (obstacles_pub_->get_subscription_count() > 0) {
    obstacles_pub_->publish(to_msg(split.above_ground));
  }
  if (markers_pub_->get_subscription_count() == 0) {
    return;
  }

  visualization_msgs::msg::MarkerArray markers;

  // DELETEALL first. Without it, markers from a frame with more detections
  // than this one linger until their lifetime expires, and the display shows
  // boulders that are no longer detected.
  visualization_msgs::msg::Marker clear;
  clear.action = visualization_msgs::msg::Marker::DELETEALL;
  markers.markers.push_back(clear);

  int id = 0;
  for (const auto & cluster : clusters) {
    visualization_msgs::msg::Marker marker;
    marker.header.stamp = stamp;
    marker.header.frame_id = output_frame_;
    marker.ns = "boulders";
    marker.id = id++;
    marker.type = visualization_msgs::msg::Marker::CUBE;
    marker.action = visualization_msgs::msg::Marker::ADD;

    marker.pose.position.x = cluster.centroid.x();
    marker.pose.position.y = cluster.centroid.y();
    marker.pose.position.z = cluster.centroid.z();
    marker.pose.orientation.w = 1.0;

    marker.scale.x = cluster.dimensions.x();
    marker.scale.y = cluster.dimensions.y();
    marker.scale.z = cluster.dimensions.z();

    marker.color.r = 0.9f;
    marker.color.g = 0.4f;
    marker.color.b = 0.1f;
    marker.color.a = 0.6f;

    markers.markers.push_back(marker);
  }

  markers_pub_->publish(markers);
}

}  // namespace lunabot_perception

RCLCPP_COMPONENTS_REGISTER_NODE(lunabot_perception::BoulderDetectorNode)
