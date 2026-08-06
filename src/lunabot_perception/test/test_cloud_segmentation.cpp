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
  // A boulder seen from one side has all its points on the near face. The
  // point centroid would sit on the surface; every consumer wants the middle
  // of the box.
  auto cloud = std::make_shared<Cloud>();
  for (float y = -0.1f; y <= 0.1f; y += 0.02f) {
    for (float z = 0.0f; z <= 0.2f; z += 0.02f) {
      cloud->points.emplace_back(1.0f, y, z);  // dense near face
      cloud->points.emplace_back(1.2f, y, z);  // sparse far face
    }
  }
  finish(cloud);

  const auto clusters = extract_clusters(cloud, cluster_params());
  ASSERT_EQ(clusters.size(), 1u);
  EXPECT_NEAR(clusters[0].centroid.x(), 1.1f, 0.01f);
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
