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

#include <pcl/conversions.h>

#include <stdexcept>

#include "ufil_pcl_preprocessor/ufil_pcl_preprocessor.hpp"

using ufil_pcl_preprocessor::ClippedArea;
using ufil_pcl_preprocessor::PclPreprocessor;
using ufil_pcl_preprocessor::PreprocessorConfig;

namespace
{

using Cloud = pcl::PointCloud<pcl::PointXYZ>;

/// Dense cube of (n+1)^3 points with 0.1 m spacing centred at (x, y, z).
void addBlock(Cloud & cloud, float x, float y, float z, int n = 4)
{
  for (int i = 0; i <= n; ++i) {
    for (int j = 0; j <= n; ++j) {
      for (int k = 0; k <= n; ++k) {
        cloud.push_back(
          pcl::PointXYZ(
            x + 0.1f * (i - n / 2), y + 0.1f * (j - n / 2), z + 0.1f * (k - n / 2)));
      }
    }
  }
}

PreprocessorConfig testConfig()
{
  PreprocessorConfig config;
  config.filter.voxel_size = 0.01;  // keep every point
  config.filter.outlier_radius = 0.25;
  config.filter.min_neighbors = 3;
  config.filter.z_min = 0.5;
  config.filter.z_max = 5.0;
  return config;
}

Cloud run(
  PclPreprocessor & preprocessor, const Cloud & input,
  ufil_pcl_preprocessor::FilterStatistics * stats = nullptr)
{
  pcl::PCLPointCloud2 input_msg;
  pcl::toPCLPointCloud2(input, input_msg);
  pcl::PCLPointCloud2 output_msg;
  const auto result = preprocessor.filter(input_msg, output_msg);
  if (stats) {
    *stats = result;
  }
  Cloud output;
  pcl::fromPCLPointCloud2(output_msg, output);
  return output;
}

}  // namespace

TEST(PclPreprocessor, InvalidConfigurationIsRejected)
{
  PreprocessorConfig config;
  config.filter.voxel_size = -1.0;
  EXPECT_THROW(PclPreprocessor{config}, std::invalid_argument);

  PclPreprocessor preprocessor;
  EXPECT_THROW(preprocessor.configure(config), std::invalid_argument);
  EXPECT_DOUBLE_EQ(preprocessor.config().filter.voxel_size, PreprocessorConfig{}.filter.voxel_size);
}

TEST(PclPreprocessor, EveryPointIsAccountedFor)
{
  PclPreprocessor preprocessor(testConfig());
  preprocessor.setClippedAreas({ClippedArea{5.0f, 0.0f, 2.0f, 0.0f, 2.0f, 2.0f, 2.0f}});

  Cloud input;
  addBlock(input, 0.0f, 0.0f, 2.0f);  // dynamic object, 125 points
  addBlock(input, 5.0f, 0.0f, 2.0f);  // inside clipped area
  addBlock(input, 0.0f, 3.0f, 0.0f, 2);  // below z_min
  input.push_back(pcl::PointXYZ(-5.0f, -5.0f, 2.0f));  // isolated outlier

  ufil_pcl_preprocessor::FilterStatistics stats;
  const Cloud output = run(preprocessor, input, &stats);

  EXPECT_EQ(stats.input_points, input.size());
  EXPECT_EQ(stats.static_points, 125u);
  EXPECT_EQ(stats.out_of_height_points, 27u);
  EXPECT_EQ(stats.outlier_points, 1u);
  EXPECT_EQ(stats.dynamic_points, 125u);
  EXPECT_EQ(
    stats.input_points,
    stats.out_of_height_points + stats.static_points + stats.outlier_points +
    stats.dynamic_points);
  EXPECT_EQ(output.size(), stats.output_points);
  for (const auto & pt : output) {
    EXPECT_NEAR(pt.x, 0.0f, 0.25f);
    EXPECT_NEAR(pt.y, 0.0f, 0.25f);
  }

  const auto status = preprocessor.status();
  EXPECT_EQ(status.processed_clouds, 1u);
  EXPECT_EQ(status.clipped_area_count, 1u);
  EXPECT_EQ(status.last_cloud.output_points, stats.output_points);
}

TEST(PclPreprocessor, OutlierFilterRemovesTheActualOutliers)
{
  // Regression: removed points used to be determined by index position instead of identity.
  PclPreprocessor preprocessor(testConfig());
  Cloud input;
  input.push_back(pcl::PointXYZ(-5.0f, -5.0f, 2.0f));  // outlier first in the cloud
  addBlock(input, 0.0f, 0.0f, 2.0f);

  const Cloud output = run(preprocessor, input);
  ASSERT_EQ(output.size(), 125u);
  for (const auto & pt : output) {
    EXPECT_GT(pt.x, -1.0f);
  }
}

TEST(PclPreprocessor, VoxelGridDownsamples)
{
  auto config = testConfig();
  config.filter.voxel_size = 1.0;
  PclPreprocessor preprocessor(config);
  Cloud input;
  addBlock(input, 0.5f, 0.5f, 2.5f);
  EXPECT_EQ(run(preprocessor, input).size(), 1u);
}

TEST(PclPreprocessor, TransformsIntoTargetFrame)
{
  PclPreprocessor preprocessor(testConfig());
  Eigen::Isometry3d sensor_to_target = Eigen::Isometry3d::Identity();
  sensor_to_target.translation() = Eigen::Vector3d(10.0, 0.0, 2.0);
  preprocessor.setTransforms(Eigen::Isometry3d::Identity(), sensor_to_target);

  Cloud input;
  addBlock(input, 0.0f, 0.0f, 0.0f);  // at the sensor, 2 m above ground in the target frame
  const Cloud output = run(preprocessor, input);
  ASSERT_FALSE(output.empty());
  EXPECT_NEAR(output.front().x, 10.0f, 0.3f);
  EXPECT_NEAR(output.front().z, 2.0f, 0.3f);
}

TEST(PclPreprocessor, EmptyCloud)
{
  PclPreprocessor preprocessor(testConfig());
  ufil_pcl_preprocessor::FilterStatistics stats;
  EXPECT_TRUE(run(preprocessor, Cloud(), &stats).empty());
  EXPECT_EQ(stats.input_points, 0u);
  EXPECT_FALSE(preprocessor.status().fov_valid);
}
