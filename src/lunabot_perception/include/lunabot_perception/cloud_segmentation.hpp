// Copyright 2027 Lunabot
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef LUNABOT_PERCEPTION__CLOUD_SEGMENTATION_HPP_
#define LUNABOT_PERCEPTION__CLOUD_SEGMENTATION_HPP_

// PCL's headers end in .h, so cpplint files them as C system headers and
// wants them before the C++ ones. Not a stylistic preference: with the blocks
// the other way round, ament_cpplint fails with "Found C system header after
// C++ system header" on every pcl include. clang-format keeps the blocks in
// this order (IncludeBlocks: Preserve) and sorts within them.
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <vector>

namespace lunabot_perception
{

using PointT = pcl::PointXYZ;
using Cloud = pcl::PointCloud<PointT>;

/// The segmentation pipeline as free functions on PCL types, with NO ROS
/// dependency. That is what makes them unit-testable against hand-built
/// clouds in about a millisecond, with no graph, no nodes and no bag files.
///
/// All of these operate in `base_link`: X forward, Y left, Z up, origin at
/// axle height. The caller transforms the cloud before calling in.

/// Result of splitting a cloud by height. THREE clouds, not two.
///
/// This is the single most important shape decision in the package.
/// Boulders are the above-ground clusters; craters are the below-ground ones.
/// Splitting two ways -- "ground" and "obstacles" -- would work perfectly
/// well today, when only boulders matter, and would have to be torn apart to
/// add craters. The third cloud costs nothing now.
struct GroundSplit
{
  Cloud::Ptr ground;
  Cloud::Ptr above_ground;
  Cloud::Ptr below_ground;
};

/// An axis-aligned box around one cluster, in base_link.
struct Cluster
{
  Eigen::Vector3f centroid;
  Eigen::Vector3f dimensions;  ///< full extent, not half-extent
  std::size_t point_count = 0;
};

struct GroundParameters
{
  /// Points within this distance of the ground plane count as ground.
  double plane_distance_threshold = 0.05;

  /// Expected height of the ground plane in base_link. The wheels put
  /// base_link one wheel radius above the ground, so this is negative.
  double ground_z = -0.10;
};

struct ClusterParameters
{
  double tolerance = 0.10;         ///< max gap within one cluster, metres
  std::size_t min_points = 20;     ///< below this, it is noise
  std::size_t max_points = 25000;  ///< above this, it is probably a wall
};

struct DimensionFilter
{
  double min_dimension = 0.05;  ///< smaller than this is stereo noise
  double max_dimension = 1.50;  ///< larger than this is not a boulder
  double min_height = 0.03;     ///< a flat patch is not an obstacle
};

/// Voxel-grid downsample. Cheap, and everything downstream is O(n).
Cloud::Ptr downsample(const Cloud::ConstPtr & input, double leaf_size);

/// Keep only points inside an axis-aligned box.
///
/// The region of interest matters more than it looks: the camera is tilted
/// down and clipped at 2 m, so without a ROI the cloud includes a lot of
/// near-field chassis and far-field noise.
Cloud::Ptr crop(
  const Cloud::ConstPtr & input, const Eigen::Vector3f & min, const Eigen::Vector3f & max);

/// Split into ground / above / below by height about a known plane.
///
/// This is the STUB implementation: a flat z-threshold about `ground_z`. It
/// assumes the ground is level and that base_link's height above it is known,
/// neither of which survives a slope or a suspension. A RANSAC plane fit is
/// the obvious replacement and is why the parameters are shaped this way
/// rather than being a bare float.
GroundSplit split_by_ground(const Cloud::ConstPtr & input, const GroundParameters & params);

/// Euclidean clustering, then an axis-aligned box per cluster.
std::vector<Cluster> extract_clusters(
  const Cloud::ConstPtr & input, const ClusterParameters & params);

/// Drop clusters that are the wrong size to be a boulder.
std::vector<Cluster> filter_by_dimensions(
  const std::vector<Cluster> & clusters, const DimensionFilter & filter);

}  // namespace lunabot_perception

#endif  // LUNABOT_PERCEPTION__CLOUD_SEGMENTATION_HPP_
