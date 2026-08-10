// Copyright 2027 Lunabot
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.
//
// Unit tests for the segmentation pipeline, against hand-built clouds.
//
// These run in milliseconds with no ROS graph, no camera and no bag, which is
// the whole reason cloud_segmentation.cpp has no ROS dependency. Every
// interesting property of the pipeline is checkable here.

#include <gtest/gtest.h>

#include <cmath>
#include <memory>

#include "lunabot_perception/cloud_segmentation.hpp"

namespace lunabot_perception
{
namespace
{

constexpr float kGroundZ = -0.10f;

/// A flat square of ground at kGroundZ.
void add_ground(const Cloud::Ptr & cloud, float extent = 2.0f, float spacing = 0.05f)
{
  for (float x = 0.0f; x <= extent; x += spacing) {
    for (float y = -extent / 2; y <= extent / 2; y += spacing) {
      cloud->points.emplace_back(x, y, kGroundZ);
    }
  }
}

/// A solid cube sitting ON the ground, centred at (cx, cy).
void add_boulder(const Cloud::Ptr & cloud, float cx, float cy, float size, float spacing = 0.02f)
{
  const float half = size / 2;
  for (float x = cx - half; x <= cx + half; x += spacing) {
    for (float y = cy - half; y <= cy + half; y += spacing) {
      for (float z = kGroundZ; z <= kGroundZ + size; z += spacing) {
        cloud->points.emplace_back(x, y, z);
      }
    }
  }
}

/// Ground height at x on a slope that rises with x and passes through
/// kGroundZ directly under base_link -- the rover is ON the slope, which is
/// the case that matters. A slope the rover is looking at from level ground
/// is an easier problem, not a harder one.
float sloped_ground_z(float x, float slope)
{
  return kGroundZ + std::tan(slope) * x;
}

/// A flat square of ground, tilted `slope` radians about the Y axis.
///
/// Spacing is coarser than add_ground's so that the ground still outnumbers
/// a solid cube of boulder points. Point counts decide RANSAC votes, and a
/// scene where the rock outvotes the floor is testing something else.
void add_sloped_ground(
  const Cloud::Ptr & cloud, float slope, float extent = 2.5f, float spacing = 0.03f)
{
  for (float x = 0.0f; x <= extent; x += spacing) {
    for (float y = -extent / 2; y <= extent / 2; y += spacing) {
      cloud->points.emplace_back(x, y, sloped_ground_z(x, slope));
    }
  }
}

/// A cube whose base follows the slope, so it sits on the ground rather than
/// half-buried at one edge and floating at the other.
void add_boulder_on_slope(
  const Cloud::Ptr & cloud, float cx, float cy, float size, float slope, float spacing = 0.03f)
{
  const float half = size / 2;
  for (float x = cx - half; x <= cx + half; x += spacing) {
    for (float y = cy - half; y <= cy + half; y += spacing) {
      const float base = sloped_ground_z(x, slope);
      for (float z = base; z <= base + size; z += spacing) {
        cloud->points.emplace_back(x, y, z);
      }
    }
  }
}

Cloud::Ptr finish(const Cloud::Ptr & cloud)
{
  cloud->width = cloud->points.size();
  cloud->height = 1;
  cloud->is_dense = false;
  return cloud;
}

GroundParameters ground_params()
{
  GroundParameters params;
  params.ground_z = kGroundZ;
  params.plane_distance_threshold = 0.05;
  return params;
}

ClusterParameters cluster_params()
{
  ClusterParameters params;
  params.tolerance = 0.10;
  params.min_points = 20;
  params.max_points = 25000;
  return params;
}

}  // namespace

TEST(Downsample, reduces_point_count_without_moving_the_cloud)
{
  auto cloud = std::make_shared<Cloud>();
  add_ground(cloud, 2.0f, 0.01f);
  finish(cloud);

  const auto reduced = downsample(cloud, 0.05);

  EXPECT_LT(reduced->points.size(), cloud->points.size());
  EXPECT_GT(reduced->points.size(), 0u);
  for (const auto & point : reduced->points) {
    EXPECT_NEAR(point.z, kGroundZ, 0.05f);
  }
}

TEST(Downsample, a_non_positive_leaf_passes_the_cloud_through)
{
  // Guards against VoxelGrid allocating an unbounded index, which is a crash
  // rather than an error.
  auto cloud = std::make_shared<Cloud>();
  add_ground(cloud);
  finish(cloud);

  EXPECT_EQ(downsample(cloud, 0.0)->points.size(), cloud->points.size());
  EXPECT_EQ(downsample(cloud, -1.0)->points.size(), cloud->points.size());
}

TEST(Crop, discards_points_outside_the_region_of_interest)
{
  auto cloud = std::make_shared<Cloud>();
  cloud->points.emplace_back(1.0f, 0.0f, 0.0f);  // inside
  cloud->points.emplace_back(5.0f, 0.0f, 0.0f);  // too far
  cloud->points.emplace_back(0.0f, 0.0f, 0.0f);  // too near
  cloud->points.emplace_back(1.0f, 3.0f, 0.0f);  // too far left
  finish(cloud);

  const auto cropped =
    crop(cloud, Eigen::Vector3f(0.2f, -1.5f, -0.5f), Eigen::Vector3f(2.5f, 1.5f, 1.0f));

  ASSERT_EQ(cropped->points.size(), 1u);
  EXPECT_FLOAT_EQ(cropped->points[0].x, 1.0f);
}

TEST(SplitByGround, separates_ground_from_what_sits_on_it)
{
  auto cloud = std::make_shared<Cloud>();
  add_ground(cloud);
  add_boulder(cloud, 1.0f, 0.0f, 0.20f);
  finish(cloud);

  const auto split = split_by_ground(cloud, ground_params());

  EXPECT_GT(split.ground->points.size(), 0u);
  EXPECT_GT(split.above_ground->points.size(), 0u);
  for (const auto & point : split.above_ground->points) {
    EXPECT_GT(point.z, kGroundZ + 0.05f);
  }
}

TEST(SplitByGround, populates_the_below_ground_cloud)
{
  // The three-way split is the design decision that keeps craters cheap to
  // add later. A two-way split would pass every other test in this file and
  // would have to be torn apart to support them.
  auto cloud = std::make_shared<Cloud>();
  add_ground(cloud);
  // A depression: points well below the plane.
  for (float x = 0.9f; x <= 1.1f; x += 0.02f) {
    for (float y = -0.1f; y <= 0.1f; y += 0.02f) {
      cloud->points.emplace_back(x, y, kGroundZ - 0.20f);
    }
  }
  finish(cloud);

  const auto split = split_by_ground(cloud, ground_params());

  EXPECT_GT(split.below_ground->points.size(), 0u)
    << "below_ground must be populated, or crater detection needs a rewrite rather "
       "than an extension";
  for (const auto & point : split.below_ground->points) {
    EXPECT_LT(point.z, kGroundZ - 0.05f);
  }
}

TEST(SplitByGround, loses_no_points)
{
  auto cloud = std::make_shared<Cloud>();
  add_ground(cloud);
  add_boulder(cloud, 1.0f, 0.0f, 0.20f);
  finish(cloud);

  const auto split = split_by_ground(cloud, ground_params());

  EXPECT_EQ(
    split.ground->points.size() + split.above_ground->points.size() +
      split.below_ground->points.size(),
    cloud->points.size());
}

// ============================ Ground plane fitting ============================
//
// The flat z-threshold this package started with assumed level ground at a
// known height. These cover the fit that replaced it, and -- in
// detects_a_boulder_on_a_slope and its companion -- the failure that motivated
// the whole thing.

TEST(FitGroundPlane, recovers_a_level_plane_and_reports_it_as_fitted)
{
  auto cloud = std::make_shared<Cloud>();
  add_ground(cloud);
  finish(cloud);

  const auto plane = fit_ground_plane(cloud, ground_params());

  EXPECT_TRUE(plane.fitted);
  EXPECT_NEAR(plane.slope(), 0.0f, 0.01f);
  EXPECT_NEAR(plane.height_below_origin(), kGroundZ, 0.01f);
  EXPECT_GT(plane.inlier_count, 0u);
  // Unit normal, pointing up. Everything downstream reads a sign off this.
  EXPECT_NEAR(plane.coefficients.head<3>().norm(), 1.0f, 1e-4f);
  EXPECT_GT(plane.coefficients.z(), 0.0f);
}

TEST(FitGroundPlane, recovers_the_tilt_of_a_sloped_plane)
{
  constexpr float kSlope = 0.175f;  // 10 degrees

  auto cloud = std::make_shared<Cloud>();
  add_sloped_ground(cloud, kSlope);
  finish(cloud);

  const auto plane = fit_ground_plane(cloud, ground_params());

  ASSERT_TRUE(plane.fitted);
  EXPECT_NEAR(plane.slope(), kSlope, 0.02f);
  // The slope was built through kGroundZ under the origin, so the fit should
  // put it back there even though it is nowhere near level.
  EXPECT_NEAR(plane.height_below_origin(), kGroundZ, 0.02f);
  EXPECT_GT(plane.coefficients.z(), 0.0f);
}

TEST(FitGroundPlane, corrects_a_ground_z_that_is_simply_wrong)
{
  // The prior says -0.10; the ground is really at -0.25. That is a
  // suspension deflection, or a wheel radius nobody updated after a tyre
  // change. The flat threshold would have called the entire floor a crater.
  constexpr float kRealGroundZ = -0.25f;

  auto cloud = std::make_shared<Cloud>();
  for (float x = 0.0f; x <= 2.0f; x += 0.03f) {
    for (float y = -1.0f; y <= 1.0f; y += 0.03f) {
      cloud->points.emplace_back(x, y, kRealGroundZ);
    }
  }
  finish(cloud);

  const auto plane = fit_ground_plane(cloud, ground_params());

  ASSERT_TRUE(plane.fitted);
  EXPECT_NEAR(plane.height_below_origin(), kRealGroundZ, 0.01f);
}

TEST(FitGroundPlane, falls_back_to_the_level_plane_when_fitting_is_off)
{
  auto cloud = std::make_shared<Cloud>();
  add_sloped_ground(cloud, 0.175f);
  finish(cloud);

  auto params = ground_params();
  params.fit_plane = false;

  const auto plane = fit_ground_plane(cloud, params);

  EXPECT_FALSE(plane.fitted);
  EXPECT_EQ(plane.inlier_count, 0u);
  // Exactly the plane the old flat threshold used: z = ground_z.
  EXPECT_NEAR(plane.coefficients.x(), 0.0f, 1e-6f);
  EXPECT_NEAR(plane.coefficients.y(), 0.0f, 1e-6f);
  EXPECT_NEAR(plane.coefficients.z(), 1.0f, 1e-6f);
  EXPECT_NEAR(plane.height_below_origin(), kGroundZ, 1e-6f);
}

TEST(FitGroundPlane, falls_back_when_there_is_almost_no_cloud)
{
  // Fewer than three points cannot define a plane, and three points that do
  // are not a measurement of anything.
  auto cloud = std::make_shared<Cloud>();
  cloud->points.emplace_back(1.0f, 0.0f, kGroundZ);
  cloud->points.emplace_back(1.1f, 0.0f, kGroundZ);
  finish(cloud);

  const auto plane = fit_ground_plane(cloud, ground_params());

  EXPECT_FALSE(plane.fitted);
  EXPECT_NEAR(plane.height_below_origin(), kGroundZ, 1e-6f);
}

TEST(FitGroundPlane, rejects_a_plane_steeper_than_max_slope)
{
  // A 30 degree face with the limit at 15. Better to fall back to a plane
  // that is wrong in a known way than to accept one this tilted -- at 30
  // degrees the thing being fitted is far more likely to be a berm face than
  // the arena floor.
  auto cloud = std::make_shared<Cloud>();
  add_sloped_ground(cloud, 0.524f);
  finish(cloud);

  auto params = ground_params();
  params.max_slope = 0.26;

  const auto plane = fit_ground_plane(cloud, params);

  EXPECT_FALSE(plane.fitted);
  EXPECT_NEAR(plane.slope(), 0.0f, 1e-6f) << "the fallback plane is the level one";
}

TEST(FitGroundPlane, rejects_a_plane_too_far_from_the_expected_height)
{
  // The guard against locking onto the top of a large rock: a perfectly
  // horizontal, perfectly well supported plane, most of a metre too high.
  auto cloud = std::make_shared<Cloud>();
  for (float x = 0.0f; x <= 2.0f; x += 0.03f) {
    for (float y = -1.0f; y <= 1.0f; y += 0.03f) {
      cloud->points.emplace_back(x, y, kGroundZ + 0.80f);
    }
  }
  finish(cloud);

  auto params = ground_params();
  params.max_height_deviation = 0.30;

  const auto plane = fit_ground_plane(cloud, params);

  EXPECT_FALSE(plane.fitted);
  EXPECT_NEAR(plane.height_below_origin(), kGroundZ, 1e-6f);
}

TEST(FitGroundPlane, rejects_a_plane_without_enough_support)
{
  // Ground plus a rock, with the support bar set absurdly high. Stands in
  // for the real case -- a cloud that is mostly obstacle and barely floor --
  // without needing a scene contrived enough to be its own puzzle.
  auto cloud = std::make_shared<Cloud>();
  add_ground(cloud);
  add_boulder(cloud, 1.0f, 0.0f, 0.20f);
  finish(cloud);

  auto params = ground_params();
  params.min_inlier_fraction = 0.99;

  EXPECT_FALSE(fit_ground_plane(cloud, params).fitted);

  // Same cloud, believable bar: the floor is most of it, so it fits.
  params.min_inlier_fraction = 0.25;
  EXPECT_TRUE(fit_ground_plane(cloud, params).fitted);
}

TEST(SplitByGround, classifies_by_distance_to_the_sloped_plane_not_by_height)
{
  constexpr float kSlope = 0.175f;

  auto cloud = std::make_shared<Cloud>();
  add_sloped_ground(cloud, kSlope);
  finish(cloud);

  const auto split = split_by_ground(cloud, ground_params());

  ASSERT_TRUE(split.plane.fitted);
  // Every point is ground. Under the old flat threshold the far half of this
  // same cloud came back as obstacles, because at 10 degrees the floor climbs
  // through a 5 cm band within the first 30 cm.
  EXPECT_EQ(split.ground->points.size(), cloud->points.size());
  EXPECT_EQ(split.above_ground->points.size(), 0u);
  EXPECT_EQ(split.below_ground->points.size(), 0u);
}

TEST(SplitByGround, loses_no_points_on_a_slope)
{
  auto cloud = std::make_shared<Cloud>();
  add_sloped_ground(cloud, 0.175f);
  add_boulder_on_slope(cloud, 1.2f, 0.0f, 0.25f, 0.175f);
  finish(cloud);

  const auto split = split_by_ground(cloud, ground_params());

  EXPECT_EQ(
    split.ground->points.size() + split.above_ground->points.size() +
      split.below_ground->points.size(),
    cloud->points.size());
}

TEST(Pipeline, detects_a_boulder_on_a_slope)
{
  // The payoff. One rock on a 10 degree slope, one detection out, near where
  // it was put -- with no adjustment to ground_z, which is still the level
  // value and still wrong for most of this scene.
  constexpr float kSlope = 0.175f;

  auto cloud = std::make_shared<Cloud>();
  add_sloped_ground(cloud, kSlope);
  add_boulder_on_slope(cloud, 1.2f, 0.0f, 0.25f, kSlope);
  finish(cloud);

  const auto split = split_by_ground(cloud, ground_params());
  ASSERT_TRUE(split.plane.fitted);

  const auto clusters = extract_clusters(split.above_ground, cluster_params());

  DimensionFilter filter;
  filter.min_dimension = 0.05;
  filter.max_dimension = 1.50;
  filter.min_height = 0.03;

  const auto boulders = filter_by_dimensions(clusters, filter);

  ASSERT_EQ(boulders.size(), 1u);
  EXPECT_NEAR(boulders[0].centroid.x(), 1.2f, 0.10f);
  EXPECT_NEAR(boulders[0].centroid.y(), 0.0f, 0.10f);
}

TEST(Pipeline, the_flat_threshold_goes_blind_on_the_same_slope)
{
  // The bug the fit exists to remove, kept as a test so that turning fitting
  // off is a visibly worse answer rather than a quiet one. Same scene as
  // detects_a_boulder_on_a_slope, fit_plane off.
  //
  // The failure is worse than the false positives it looks like it should be.
  // The whole floor comes back as one contiguous above-ground cluster, the
  // rock is joined to it, and the size filter then throws the lot away as a
  // wall -- so the detector does not over-report, it reports NOTHING. A rover
  // driving on that sees a clear path into a boulder.
  constexpr float kSlope = 0.175f;

  auto cloud = std::make_shared<Cloud>();
  add_sloped_ground(cloud, kSlope);
  add_boulder_on_slope(cloud, 1.2f, 0.0f, 0.25f, kSlope);
  finish(cloud);

  auto params = ground_params();
  params.fit_plane = false;

  const auto split = split_by_ground(cloud, params);

  ASSERT_FALSE(split.plane.fitted);
  // Most of the floor, not just the rock, is now "above ground".
  EXPECT_GT(split.above_ground->points.size(), cloud->points.size() / 2)
    << "if this ever stops holding, the flat threshold got better and this "
       "test is the one to re-derive";

  const auto clusters = extract_clusters(split.above_ground, cluster_params());

  DimensionFilter filter;
  filter.min_dimension = 0.05;
  filter.max_dimension = 1.50;
  filter.min_height = 0.03;

  EXPECT_EQ(filter_by_dimensions(clusters, filter).size(), 0u)
    << "the rock is swallowed by the floor-sized cluster and filtered out with it";
}

TEST(ExtractClusters, finds_two_separated_boulders_at_the_right_places)
{
  // The headline test: two rocks in, two detections out, in the right spots.
  auto cloud = std::make_shared<Cloud>();
  add_ground(cloud);
  add_boulder(cloud, 1.0f, -0.5f, 0.20f);
  add_boulder(cloud, 1.5f, 0.6f, 0.30f);
  finish(cloud);

  const auto split = split_by_ground(cloud, ground_params());
  const auto clusters = extract_clusters(split.above_ground, cluster_params());

  ASSERT_EQ(clusters.size(), 2u);

  // Order is not guaranteed, so match by proximity.
  bool found_small = false;
  bool found_large = false;
  for (const auto & cluster : clusters) {
    if (
      std::abs(cluster.centroid.x() - 1.0f) < 0.15f &&
      std::abs(cluster.centroid.y() + 0.5f) < 0.15f) {
      found_small = true;
      EXPECT_NEAR(cluster.dimensions.x(), 0.20f, 0.06f);
    }
    if (
      std::abs(cluster.centroid.x() - 1.5f) < 0.15f &&
      std::abs(cluster.centroid.y() - 0.6f) < 0.15f) {
      found_large = true;
      EXPECT_NEAR(cluster.dimensions.x(), 0.30f, 0.06f);
    }
  }
  EXPECT_TRUE(found_small);
  EXPECT_TRUE(found_large);
}

TEST(ExtractClusters, merges_boulders_closer_than_the_tolerance)
{
  // Not a bug -- a documented limit. Two rocks 5 cm apart with a 10 cm
  // tolerance are one cluster, and no amount of downstream filtering
  // recovers them. This is the parameter to reach for when the detector
  // under-counts a rock field.
  auto cloud = std::make_shared<Cloud>();
  add_ground(cloud);
  add_boulder(cloud, 1.00f, 0.0f, 0.20f);
  add_boulder(cloud, 1.25f, 0.0f, 0.20f);
  finish(cloud);

  const auto split = split_by_ground(cloud, ground_params());
  const auto clusters = extract_clusters(split.above_ground, cluster_params());

  EXPECT_EQ(clusters.size(), 1u);
}

TEST(ExtractClusters, an_empty_cloud_yields_no_clusters)
{
  auto cloud = std::make_shared<Cloud>();
  finish(cloud);
  EXPECT_TRUE(extract_clusters(cloud, cluster_params()).empty());
}

TEST(ExtractClusters, reports_the_box_centre_not_the_visible_surface)
{
  // A boulder seen from one side gives up most of its points on the near
  // face, a few off the sides at a glancing angle, and almost none off the
  // back. A POINT centroid therefore lands near the front surface; the box
  // centre is 0.1 m further away, and the box centre is what every consumer
  // wants.
  //
  // The side rails are not decoration. Euclidean clustering joins points
  // within tolerance (0.10 m) of one another, and the two faces are 0.20 m
  // apart -- faces alone are two clusters, not one shell, which is what this
  // test asserted against for its first life.
  auto cloud = std::make_shared<Cloud>();
  constexpr float kNearFace = 1.0f;
  constexpr float kFarFace = 1.2f;

  for (float y = -0.1f; y <= 0.1f; y += 0.02f) {
    for (float z = 0.0f; z <= 0.2f; z += 0.02f) {
      cloud->points.emplace_back(kNearFace, y, z);
    }
  }
  for (float y = -0.1f; y <= 0.1f; y += 0.05f) {
    for (float z = 0.0f; z <= 0.2f; z += 0.05f) {
      cloud->points.emplace_back(kFarFace, y, z);
    }
  }
  for (float x = kNearFace; x <= kFarFace; x += 0.02f) {
    for (float z = 0.0f; z <= 0.2f; z += 0.10f) {
      cloud->points.emplace_back(x, -0.1f, z);
      cloud->points.emplace_back(x, 0.1f, z);
    }
  }
  finish(cloud);

  const auto clusters = extract_clusters(cloud, cluster_params());
  ASSERT_EQ(clusters.size(), 1u);
  EXPECT_NEAR(clusters[0].centroid.x(), 1.1f, 0.01f);

  // The property under test, stated as the comparison it is: the mean of the
  // points sits well in front of the reported centre.
  float mean_x = 0.0f;
  for (const auto & point : cloud->points) {
    mean_x += point.x;
  }
  mean_x /= static_cast<float>(cloud->points.size());
  EXPECT_LT(mean_x, clusters[0].centroid.x() - 0.02f);
}

TEST(FilterByDimensions, drops_specks_and_walls_and_keeps_boulders)
{
  std::vector<Cluster> clusters;
  clusters.push_back({{1.0f, 0.0f, 0.0f}, {0.02f, 0.02f, 0.02f}, 30});   // speck
  clusters.push_back({{1.0f, 0.0f, 0.0f}, {0.30f, 0.30f, 0.25f}, 500});  // boulder
  clusters.push_back({{1.0f, 0.0f, 0.0f}, {3.00f, 0.10f, 1.00f}, 900});  // wall

  DimensionFilter filter;
  filter.min_dimension = 0.05;
  filter.max_dimension = 1.50;
  filter.min_height = 0.03;

  const auto kept = filter_by_dimensions(clusters, filter);

  ASSERT_EQ(kept.size(), 1u);
  EXPECT_NEAR(kept[0].dimensions.x(), 0.30f, 1e-5f);
}

TEST(FilterByDimensions, drops_flat_patches)
{
  // A wide, flat area is a texture change or a shallow rise, not an obstacle
  // the rover has to avoid.
  std::vector<Cluster> clusters;
  clusters.push_back({{1.0f, 0.0f, 0.0f}, {0.40f, 0.40f, 0.01f}, 400});

  DimensionFilter filter;
  filter.min_dimension = 0.05;
  filter.max_dimension = 1.50;
  filter.min_height = 0.03;

  EXPECT_TRUE(filter_by_dimensions(clusters, filter).empty());
}

TEST(Pipeline, end_to_end_on_a_synthetic_scene)
{
  auto cloud = std::make_shared<Cloud>();
  add_ground(cloud, 3.0f, 0.02f);
  add_boulder(cloud, 1.0f, -0.4f, 0.25f);
  add_boulder(cloud, 2.0f, 0.5f, 0.35f);
  finish(cloud);

  const auto downsampled = downsample(cloud, 0.03);
  const auto cropped =
    crop(downsampled, Eigen::Vector3f(0.2f, -1.5f, -0.5f), Eigen::Vector3f(2.5f, 1.5f, 1.0f));
  const auto split = split_by_ground(cropped, ground_params());
  const auto clusters = extract_clusters(split.above_ground, cluster_params());

  DimensionFilter filter;
  filter.min_dimension = 0.05;
  filter.max_dimension = 1.50;
  filter.min_height = 0.03;

  EXPECT_EQ(filter_by_dimensions(clusters, filter).size(), 2u);
}

}  // namespace lunabot_perception

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
