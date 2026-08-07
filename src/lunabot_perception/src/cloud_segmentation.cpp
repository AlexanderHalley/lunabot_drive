// Copyright 2027 Lunabot
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.
//
// ============================ READ THIS ============================
// This is a GEOMETRIC PLACEHOLDER, not a boulder detector.
//
// What it does: finds contiguous lumps of points sticking up out of a flat
// plane, and reports their bounding boxes.
//
// What it cannot do:
//   - tell a boulder from a berm, a wall, a person's leg, or the rover's own
//     excavation tailings. Anything above the plane is a "boulder".
//   - assign a real confidence. The score is a constant.
//   - track anything over time. Every frame is independent, so detections
//     flicker and IDs mean nothing.
//   - cope with a sloped or uneven ground plane. The split is a flat
//     z-threshold.
//
// It exists so that the topic, the message type and the frame conventions are
// fixed and exercised now. Replacing it with an RGB-D network or an Isaac ROS
// model changes NOTHING downstream, which is the entire point of writing the
// contract before the algorithm.
// ===================================================================

#include "lunabot_perception/cloud_segmentation.hpp"

// PCL before the standard library: cpplint reads an angle-bracket `.h` as a C
// system header and wants those first. See cloud_segmentation.hpp.
#include <pcl/common/common.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>

#include <algorithm>
#include <limits>
#include <vector>

namespace lunabot_perception
{

Cloud::Ptr downsample(const Cloud::ConstPtr & input, double leaf_size)
{
  auto output = std::make_shared<Cloud>();

  // A non-positive leaf would make VoxelGrid allocate an unbounded index.
  // Passing the cloud through unchanged is the sane reading of "do not
  // downsample".
  if (leaf_size <= 0.0) {
    *output = *input;
    return output;
  }

  pcl::VoxelGrid<PointT> filter;
  filter.setInputCloud(input);
  filter.setLeafSize(
    static_cast<float>(leaf_size), static_cast<float>(leaf_size), static_cast<float>(leaf_size));
  filter.filter(*output);
  return output;
}

Cloud::Ptr crop(
  const Cloud::ConstPtr & input, const Eigen::Vector3f & min, const Eigen::Vector3f & max)
{
  auto output = std::make_shared<Cloud>();

  pcl::CropBox<PointT> filter;
  filter.setInputCloud(input);
  filter.setMin(Eigen::Vector4f(min.x(), min.y(), min.z(), 1.0f));
  filter.setMax(Eigen::Vector4f(max.x(), max.y(), max.z(), 1.0f));
  filter.filter(*output);
  return output;
}

GroundSplit split_by_ground(const Cloud::ConstPtr & input, const GroundParameters & params)
{
  GroundSplit split{
    std::make_shared<Cloud>(), std::make_shared<Cloud>(), std::make_shared<Cloud>()};

  const float ground_z = static_cast<float>(params.ground_z);
  const float threshold = static_cast<float>(params.plane_distance_threshold);

  for (const auto & point : input->points) {
    const float height = point.z - ground_z;

    if (std::abs(height) <= threshold) {
      split.ground->points.push_back(point);
    } else if (height > threshold) {
      split.above_ground->points.push_back(point);
    } else {
      // Below the plane. Nothing consumes this cloud yet -- craters are not
      // simulated and not detected. It is populated anyway because the cost
      // is one branch, and because a pipeline that discards this information
      // has to be rewritten to add craters rather than extended.
      split.below_ground->points.push_back(point);
    }
  }

  for (auto * cloud : {&split.ground, &split.above_ground, &split.below_ground}) {
    (*cloud)->width = (*cloud)->points.size();
    (*cloud)->height = 1;
    // Height 1 and dense false: these are unordered clouds, and claiming
    // dense would let downstream PCL skip its NaN checks.
    (*cloud)->is_dense = false;
  }

  return split;
}

std::vector<Cluster> extract_clusters(
  const Cloud::ConstPtr & input, const ClusterParameters & params)
{
  std::vector<Cluster> clusters;
  if (input->points.empty()) {
    return clusters;
  }

  auto tree = std::make_shared<pcl::search::KdTree<PointT>>();
  tree->setInputCloud(input);

  std::vector<pcl::PointIndices> indices;
  pcl::EuclideanClusterExtraction<PointT> extractor;
  extractor.setClusterTolerance(params.tolerance);
  extractor.setMinClusterSize(static_cast<int>(params.min_points));
  extractor.setMaxClusterSize(static_cast<int>(params.max_points));
  extractor.setSearchMethod(tree);
  extractor.setInputCloud(input);
  extractor.extract(indices);

  clusters.reserve(indices.size());
  for (const auto & cluster_indices : indices) {
    Eigen::Vector3f min = Eigen::Vector3f::Constant(std::numeric_limits<float>::max());
    Eigen::Vector3f max = Eigen::Vector3f::Constant(std::numeric_limits<float>::lowest());

    for (const auto index : cluster_indices.indices) {
      const auto & point = input->points[index];
      const Eigen::Vector3f p(point.x, point.y, point.z);
      min = min.cwiseMin(p);
      max = max.cwiseMax(p);
    }

    Cluster cluster;
    // Box centre, not the point centroid. For a boulder seen from one side
    // the visible points cluster on the near face, so a point centroid sits
    // on the surface rather than in the middle -- and every downstream
    // consumer wants the middle.
    //
    // This is axis-aligned in base_link. A PCA-derived oriented box would be
    // better for elongated rocks and is the obvious next step; it needs a
    // real orientation convention to be worth the complexity.
    cluster.centroid = (min + max) * 0.5f;
    cluster.dimensions = max - min;
    cluster.point_count = cluster_indices.indices.size();
    clusters.push_back(cluster);
  }

  return clusters;
}

std::vector<Cluster> filter_by_dimensions(
  const std::vector<Cluster> & clusters, const DimensionFilter & filter)
{
  std::vector<Cluster> kept;
  kept.reserve(clusters.size());

  for (const auto & cluster : clusters) {
    // Footprint, not full extent: height is checked separately because a
    // wide flat patch and a tall thin post fail for different reasons.
    const float footprint = std::max(cluster.dimensions.x(), cluster.dimensions.y());
    const float height = cluster.dimensions.z();

    if (footprint < filter.min_dimension || footprint > filter.max_dimension) {
      continue;
    }
    if (height < filter.min_height) {
      continue;
    }
    kept.push_back(cluster);
  }

  return kept;
}

}  // namespace lunabot_perception
