// Copyright 2026 Chair of Embedded Software (Computer Science 11) - RWTH Aachen University
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
// THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.

#include <gtest/gtest.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "ufil_object_tracking/detection/clustering.hpp"
#include "ufil_object_tracking/detection/shape_fitter.hpp"

namespace
{

// Adds a dense axis-aligned cube of points centered at (cx, cy) with the given
// half-extent and point spacing.
void addCube(
  const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud, float cx, float cy,
  float half_extent, float spacing)
{
  for (float dx = -half_extent; dx <= half_extent; dx += spacing) {
    for (float dy = -half_extent; dy <= half_extent; dy += spacing) {
      for (float dz = 0.0f; dz <= half_extent; dz += spacing) {
        cloud->push_back(pcl::PointXYZ{cx + dx, cy + dy, dz});
      }
    }
  }
}

}  // namespace

TEST(ClusteringTest, SeparatesTwoFarApartBlobs)
{
  auto cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  addCube(cloud, 0.0f, 0.0f, 0.5f, 0.1f);
  addCube(cloud, 10.0f, 10.0f, 0.5f, 0.1f);

  // Small minimum so the modest synthetic blobs qualify as clusters.
  ufil::detection::Clustering clustering(0.5, 5, 50000);
  std::vector<ufil::detection::Cluster> clusters = clustering.cluster(cloud);

  EXPECT_EQ(clusters.size(), 2u);
  for (const auto & cluster : clusters) {
    EXPECT_NE(cluster.cloud, nullptr);
    EXPECT_GT(cluster.cloud->size(), 0u);
  }
}

TEST(ShapeFitterTest, CylinderFitRecoversExtentAndCenter)
{
  auto cluster = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  addCube(cluster, 3.0f, -2.0f, 0.5f, 0.1f);

  ufil::detection::ShapeFitter fitter(0.1f, 0.1f, 0.1f);
  ufil::detection::BoundingBox box;
  fitter.fitCylinder(cluster, box);

  EXPECT_FALSE(box.oriented);
  EXPECT_NEAR(box.position_x, 3.0f, 0.1f);
  EXPECT_NEAR(box.position_y, -2.0f, 0.1f);
  EXPECT_NEAR(box.dimension_x, 1.0f, 0.15f);
  EXPECT_NEAR(box.dimension_y, 1.0f, 0.15f);
}

TEST(ShapeFitterTest, LShapeFitIsOriented)
{
  auto cluster = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  addCube(cluster, 0.0f, 0.0f, 0.5f, 0.1f);

  ufil::detection::ShapeFitter fitter(0.1f, 0.1f, 0.1f);
  ufil::detection::BoundingBox box;
  fitter.fitLShape(cluster, box);

  EXPECT_TRUE(box.oriented);
  EXPECT_GT(box.dimension_x, 0.0f);
  EXPECT_GT(box.dimension_y, 0.0f);
}
