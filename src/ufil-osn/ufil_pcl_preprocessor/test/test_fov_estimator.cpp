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

#include <stdexcept>

#include "ufil_pcl_preprocessor/fov_estimator.hpp"

using ufil_pcl_preprocessor::FOVEstimator;

namespace
{

/// Dense wall of points in front of the sensor, between y = -width/2 and y = width/2.
pcl::PointCloud<pcl::PointXYZ> wall(float distance, float width, int points_per_column)
{
  pcl::PointCloud<pcl::PointXYZ> cloud;
  for (float y = -width / 2.0f; y <= width / 2.0f; y += 0.05f) {
    for (int i = 0; i < points_per_column; ++i) {
      cloud.push_back(pcl::PointXYZ(distance, y, -1.0f + 0.01f * i));
    }
  }
  return cloud;
}

}  // namespace

TEST(FOVEstimator, InvalidArgumentsThrow)
{
  FOVEstimator estimator;
  EXPECT_THROW(estimator.setNumberOfBins(0), std::invalid_argument);
  EXPECT_THROW(estimator.setHistoryLength(0), std::invalid_argument);
  EXPECT_THROW(estimator.setMaxRange(0.0f), std::invalid_argument);
  EXPECT_THROW(FOVEstimator(-1), std::invalid_argument);
}

TEST(FOVEstimator, EstimatesExtentOfDenseCloud)
{
  FOVEstimator estimator(4);
  estimator.setHistoryLength(1);
  estimator.setMaxRange(100.0f);

  estimator.update(wall(10.0f, 10.0f, 200));
  ASSERT_TRUE(estimator.lastEstimateValid());
  const auto fov = estimator.getFOV();
  EXPECT_NEAR(fov.rMin(), 10.0f, 0.01f);
  EXPECT_GT(fov.rMax(), 10.0f);
  EXPECT_LT(fov.phiMin(), 0.0f);
  EXPECT_GT(fov.phiMax(), 0.0f);
  EXPECT_LT(fov.thetaMin(), fov.thetaMax());
}

TEST(FOVEstimator, SparseCloudIsInvalid)
{
  FOVEstimator estimator(4);
  estimator.setMaxRange(100.0f);
  estimator.update(wall(10.0f, 1.0f, 1));
  EXPECT_FALSE(estimator.lastEstimateValid());
}

TEST(FOVEstimator, AveragesOverHistory)
{
  FOVEstimator estimator(4);
  estimator.setHistoryLength(2);
  estimator.setMaxRange(100.0f);

  estimator.update(wall(10.0f, 10.0f, 200));
  estimator.update(wall(20.0f, 20.0f, 200));
  const float averaged = estimator.getFOV().rMin();
  EXPECT_NEAR(averaged, 15.0f, 0.01f);

  // The first estimate drops out of the history.
  estimator.update(wall(20.0f, 20.0f, 200));
  EXPECT_NEAR(estimator.getFOV().rMin(), 20.0f, 0.01f);
}

TEST(FOVEstimator, RangeIsClampedToMaxRange)
{
  FOVEstimator estimator(4);
  estimator.setHistoryLength(1);
  estimator.setMaxRange(12.0f);
  estimator.update(wall(10.0f, 20.0f, 200));
  EXPECT_FLOAT_EQ(estimator.getFOV().rMax(), 12.0f);
}
