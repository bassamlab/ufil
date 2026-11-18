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

#include "ufil_pcl_preprocessor/depth_buffer.hpp"

using ufil_pcl_preprocessor::DepthBuffer;
using ufil_pcl_preprocessor::OcclusionGridConfig;

namespace
{

OcclusionGridConfig smallGrid()
{
  OcclusionGridConfig config;
  config.min_range = 1.0;
  config.max_range = 50.0;
  config.resolution = 1.0;
  config.size_x = 20.0;
  config.size_y = 10.0;
  config.decay_rate = 0.5;
  return config;
}

}  // namespace

TEST(DepthBuffer, GridGeometryFollowsConfig)
{
  DepthBuffer buffer(smallGrid());
  const auto grid = buffer.getOccupancyGrid();
  EXPECT_EQ(grid.width(), 20u);
  EXPECT_EQ(grid.height(), 10u);
  EXPECT_DOUBLE_EQ(grid.resolution(), 1.0);
  EXPECT_DOUBLE_EQ(grid.originX(), -10.0);
  EXPECT_DOUBLE_EQ(grid.originY(), -5.0);
  EXPECT_EQ(grid.data().size(), 200u);
}

TEST(DepthBuffer, ObservedCellsGrowAndDecay)
{
  DepthBuffer buffer(smallGrid());
  buffer.setSensorOrigin(Eigen::Vector3d(0.0, 0.0, 4.0));

  // Point at half the sensor height: its ray hits the ground at x = 4.
  pcl::PointCloud<pcl::PointXYZ> cloud;
  cloud.push_back(pcl::PointXYZ(2.0f, 0.0f, 2.0f));

  // Cell of x = 4, y = 0 in a grid starting at (-10, -5).
  const std::size_t cell = 5 * 20 + 14;
  for (int i = 0; i < 10; ++i) {
    buffer.update(cloud);
  }
  const int occupied = buffer.getOccupancyGrid().data()[cell];
  EXPECT_GT(occupied, 50);

  buffer.update(pcl::PointCloud<pcl::PointXYZ>());
  const int decayed = buffer.getOccupancyGrid().data()[cell];
  EXPECT_LT(decayed, occupied);
  EXPECT_GT(decayed, 0);

  buffer.reset();
  EXPECT_EQ(buffer.getOccupancyGrid().data()[cell], 0);
}

TEST(DepthBuffer, IgnoresPointsOutsideRange)
{
  DepthBuffer buffer(smallGrid());
  buffer.setSensorOrigin(Eigen::Vector3d(0.0, 0.0, 4.0));
  pcl::PointCloud<pcl::PointXYZ> cloud;
  cloud.push_back(pcl::PointXYZ(0.5f, 0.0f, 0.0f));  // closer than min_range
  buffer.update(cloud);
  const auto grid = buffer.getOccupancyGrid();
  for (const auto value : grid.data()) {
    ASSERT_EQ(value, 0);
  }
}

TEST(DepthBuffer, ReconfigureResizesOnlyOnGeometryChange)
{
  DepthBuffer buffer(smallGrid());
  auto config = smallGrid();
  config.decay_rate = 0.9;
  buffer.configure(config);
  EXPECT_EQ(buffer.getOccupancyGrid().width(), 20u);
  config.resolution = 0.5;
  buffer.configure(config);
  EXPECT_EQ(buffer.getOccupancyGrid().width(), 40u);
}
