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

/// The ground plane that a split was classified against.
///
/// Returned rather than kept internal because this is the first thing worth
/// looking at when the detector misbehaves. A detector that suddenly reports
/// the whole arena as one boulder has almost always fitted its plane to
/// something that is not the ground, and no amount of staring at the
/// detections themselves shows that.
struct GroundPlane
{
  /// ax + by + cz + d = 0, with (a, b, c) a UNIT normal that points UP.
  ///
  /// The sign convention is not cosmetic. RANSAC hands back a normal pointing
  /// whichever way the three points it happened to draw imply, so without
  /// orienting it here "above ground" and "below ground" would swap between
  /// one frame and the next.
  Eigen::Vector4f coefficients = Eigen::Vector4f(0.0f, 0.0f, 1.0f, 0.0f);

  /// False when no fit was attempted, or one was attempted and rejected. The
  /// split then ran against the level plane at `ground_z` instead, which is
  /// the old stub behaviour and still a reasonable answer on flat ground.
  bool fitted = false;

  /// Points supporting the fit. Zero when `fitted` is false.
  std::size_t inlier_count = 0;

  /// Angle between the plane normal and vertical, radians.
  float slope() const;

  /// Height of the plane directly below base_link's origin, metres. Negative
  /// on a rover whose origin is at axle height.
  float height_below_origin() const;
};

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

  /// What the three clouds above were classified against.
  GroundPlane plane;
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
  ///
  /// Perpendicular distance to the plane, not a difference in z. On level
  /// ground the two are the same number; on a slope the perpendicular
  /// distance is the one that means "sitting on the ground".
  double plane_distance_threshold = 0.05;

  /// Expected height of the ground plane in base_link. The wheels put
  /// base_link one wheel radius above the ground, so this is negative.
  ///
  /// With `fit_plane` on this is no longer the classification threshold. It
  /// demotes to a prior: the plane used when a fit is rejected, and the value
  /// `max_height_deviation` judges a candidate fit against.
  double ground_z = -0.10;

  /// Fit the plane to the cloud rather than assuming it is level at
  /// `ground_z`.
  ///
  /// Off is the old stub behaviour, kept because it is the only thing that
  /// works when the ground is barely in view -- a cloud that is mostly
  /// boulder has nothing to fit to, and a wrong plane is worse than a
  /// stale one.
  bool fit_plane = true;

  /// Reject a fit tilted more than this from horizontal, radians.
  ///
  /// The constraint is applied twice: once inside RANSAC, so that tilted
  /// candidates never win, and once on the refined coefficients afterwards,
  /// because the least-squares refit is free to tilt the winner back out of
  /// bounds.
  double max_slope = 0.26;  // ~15 degrees

  /// RANSAC iterations. The ground is normally most of the cloud, so the
  /// default finds it with room to spare; this is a cost ceiling rather than
  /// a tuning knob.
  int max_iterations = 100;

  /// Reject a fit supported by less than this fraction of the input.
  ///
  /// The guard against fitting the top of a big rock. A boulder face large
  /// enough to beat this fraction is large enough that calling it the ground
  /// is arguably right.
  double min_inlier_fraction = 0.25;

  /// Reject a fit whose height below base_link's origin differs from
  /// `ground_z` by more than this, metres.
  ///
  /// Deliberately loose. It is a sanity bound that catches a plane locked
  /// onto a boulder top or a berm, NOT a second classification threshold --
  /// tighten it and the fit stops being able to correct `ground_z`, which is
  /// most of the point of fitting at all.
  double max_height_deviation = 0.30;
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

/// Fit the ground plane with RANSAC, constrained to be roughly horizontal.
///
/// Returns the level plane at `ground_z` with `fitted == false` when fitting
/// is off, when there is too little cloud to fit to, or when the fit fails
/// any of the three sanity checks in `GroundParameters`. Never returns a
/// plane the caller has to check for validity before using -- the fallback is
/// always a usable plane, which is what keeps `split_by_ground` branch-free.
///
/// Deterministic: PCL seeds its sample consensus RNG with a fixed value
/// unless asked not to, so the same cloud gives the same plane every run.
/// Tests depend on that, and so does reproducing a bad frame from a bag.
GroundPlane fit_ground_plane(const Cloud::ConstPtr & input, const GroundParameters & params);

/// Split into ground / above / below about the fitted plane.
///
/// Classification is by SIGNED PERPENDICULAR DISTANCE to the plane returned
/// by `fit_ground_plane`, positive upwards. With `fit_plane` off that plane
/// is level at `ground_z` and the distance collapses to `z - ground_z`, so
/// the flat-threshold behaviour this started as is still in here, reached by
/// the same arithmetic rather than by a separate branch.
///
/// What this still does not do: the boxes `extract_clusters` puts around the
/// resulting clusters stay axis-aligned in base_link, so on a slope they are
/// larger than the rock inside them. Fixing that needs an oriented box, not a
/// better plane.
GroundSplit split_by_ground(const Cloud::ConstPtr & input, const GroundParameters & params);

/// Euclidean clustering, then an axis-aligned box per cluster.
std::vector<Cluster> extract_clusters(
  const Cloud::ConstPtr & input, const ClusterParameters & params);

/// Drop clusters that are the wrong size to be a boulder.
std::vector<Cluster> filter_by_dimensions(
  const std::vector<Cluster> & clusters, const DimensionFilter & filter);

}  // namespace lunabot_perception

#endif  // LUNABOT_PERCEPTION__CLOUD_SEGMENTATION_HPP_
