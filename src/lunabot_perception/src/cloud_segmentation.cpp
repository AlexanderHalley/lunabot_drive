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
//   - cope with ground that is not PLANAR. The plane is now fitted rather
//     than assumed, so a slope is fine and so is a wrong `ground_z`, but a
//     crest or a dip still splits badly -- one plane is one plane.
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
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/sac_segmentation.h>

#include <algorithm>
#include <cmath>
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

float GroundPlane::slope() const
{
  // The normal is unit length and oriented upwards by the time anything can
  // observe it, so its z component is the cosine of the tilt directly. The
  // clamp is for the float rounding that puts it at 1.0000001 on an exactly
  // level plane, where acos returns NaN.
  return std::acos(std::clamp(coefficients.z(), -1.0f, 1.0f));
}

float GroundPlane::height_below_origin() const
{
  // Solve ax + by + cz + d = 0 at x = y = 0. c cannot be zero for a plane
  // that passed the slope check, but this is also called on planes that have
  // not passed anything yet.
  const float c = coefficients.z();
  if (std::abs(c) < 1e-6f) {
    return std::numeric_limits<float>::quiet_NaN();
  }
  return -coefficients.w() / c;
}

GroundPlane fit_ground_plane(const Cloud::ConstPtr & input, const GroundParameters & params)
{
  GroundPlane plane;
  // The fallback, and the answer whenever a fit is rejected below: the level
  // plane z = ground_z, written as 0x + 0y + 1z - ground_z = 0.
  plane.coefficients = Eigen::Vector4f(0.0f, 0.0f, 1.0f, -static_cast<float>(params.ground_z));

  if (!params.fit_plane || input->points.size() < 3) {
    return plane;
  }

  pcl::SACSegmentation<PointT> segmentation;
  // PERPENDICULAR_PLANE with the Z axis means "plane perpendicular to Z",
  // i.e. normal within setEpsAngle of vertical. The naming reads backwards;
  // the constraint is the one we want, and having RANSAC enforce it beats
  // fitting freely and discarding, because a tilted candidate can otherwise
  // out-score the ground and leave nothing to fall back from.
  segmentation.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);
  segmentation.setMethodType(pcl::SAC_RANSAC);
  segmentation.setAxis(Eigen::Vector3f::UnitZ());
  segmentation.setEpsAngle(params.max_slope);
  segmentation.setDistanceThreshold(params.plane_distance_threshold);
  segmentation.setMaxIterations(params.max_iterations);
  // Refit the winner over its inliers. Worth it: the raw three-point plane is
  // as noisy as the three points, and this is the plane every distance in the
  // frame is measured against.
  segmentation.setOptimizeCoefficients(true);
  segmentation.setInputCloud(input);

  pcl::PointIndices inliers;
  pcl::ModelCoefficients coefficients;
  segmentation.segment(inliers, coefficients);

  if (coefficients.values.size() != 4 || inliers.indices.empty()) {
    return plane;
  }

  Eigen::Vector4f candidate(
    coefficients.values[0], coefficients.values[1], coefficients.values[2], coefficients.values[3]);

  const float norm = candidate.head<3>().norm();
  if (norm < 1e-6f) {
    return plane;
  }
  candidate /= norm;

  // Orient upwards. RANSAC's normal points whichever way its sample implied,
  // so without this the sign of every height in the frame is a coin flip.
  if (candidate.z() < 0.0f) {
    candidate = -candidate;
  }

  GroundPlane fitted;
  fitted.coefficients = candidate;
  fitted.inlier_count = inliers.indices.size();

  // Three sanity checks, each rejecting to the level fallback. They are
  // separate rather than one score because they fail for different reasons
  // and the fix for each is a different parameter.

  // Too little support: whatever this is, it is not the arena floor.
  const double inlier_fraction =
    static_cast<double>(inliers.indices.size()) / static_cast<double>(input->points.size());
  if (inlier_fraction < params.min_inlier_fraction) {
    return plane;
  }

  // Too steep. setEpsAngle already refused tilted candidates, but
  // setOptimizeCoefficients re-fits afterwards and is not bound by it.
  if (fitted.slope() > static_cast<float>(params.max_slope)) {
    return plane;
  }

  // In the wrong place. This is what catches a plane that locked onto the top
  // of a large rock: correctly horizontal, well supported, and half a metre
  // too high.
  // NaN first: a NaN deviation compares false against everything, so testing
  // the deviation alone would let a degenerate plane through.
  const float height = fitted.height_below_origin();
  const float deviation = std::abs(height - static_cast<float>(params.ground_z));
  if (!std::isfinite(height) || deviation > static_cast<float>(params.max_height_deviation)) {
    return plane;
  }

  fitted.fitted = true;
  return fitted;
}

GroundSplit split_by_ground(const Cloud::ConstPtr & input, const GroundParameters & params)
{
  GroundSplit split;
  split.ground = std::make_shared<Cloud>();
  split.above_ground = std::make_shared<Cloud>();
  split.below_ground = std::make_shared<Cloud>();
  split.plane = fit_ground_plane(input, params);

  const Eigen::Vector3f normal = split.plane.coefficients.head<3>();
  const float offset = split.plane.coefficients.w();
  const float threshold = static_cast<float>(params.plane_distance_threshold);

  for (const auto & point : input->points) {
    // Signed perpendicular distance, positive above the plane because the
    // normal points up. On the fallback plane this is exactly the old
    // `point.z - ground_z`.
    const float height = normal.dot(Eigen::Vector3f(point.x, point.y, point.z)) + offset;

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
