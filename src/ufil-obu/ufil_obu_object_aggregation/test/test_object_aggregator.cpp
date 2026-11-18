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

#include <numbers>  // listed with the C headers, cpplint does not know this C++20 header

#include <cmath>
#include <limits>
#include <stdexcept>

#include "ufil_obu_object_aggregation/object_aggregator.hpp"

using ufil_obu_object_aggregation::Acceleration;
using ufil_obu_object_aggregation::AggregationEvent;
using ufil_obu_object_aggregation::AggregatorConfig;
using ufil_obu_object_aggregation::GnssFix;
using ufil_obu_object_aggregation::ObjectAggregator;
using ufil_obu_object_aggregation::Odometry;

namespace
{

// Aachen, Germany: standard zone 32N.
const GnssFix kAachen{50.7779, 6.0778, true};
// Edmonton, Canada: standard zone 12N.
const GnssFix kEdmonton{53.5265, -113.5293, true};
// Sydney, Australia: southern hemisphere.
const GnssFix kSydney{-33.8688, 151.2093, true};

}  // namespace

TEST(ObjectAggregator, InvalidConfigurationIsRejected)
{
  AggregatorConfig config;
  config.object_length = 0.0;
  EXPECT_THROW(ObjectAggregator{config}, std::invalid_argument);

  ObjectAggregator aggregator;
  EXPECT_THROW(aggregator.configure(config), std::invalid_argument);
  EXPECT_DOUBLE_EQ(aggregator.config().object_length, AggregatorConfig{}.object_length);
}

TEST(ObjectAggregator, ConvertsToStandardUtmZone)
{
  ObjectAggregator aggregator;
  EXPECT_EQ(
    aggregator.aggregate(kAachen, Odometry{}, Acceleration{}),
    AggregationEvent::kUtmZoneChanged);
  const auto & position = aggregator.lastObject()->position;
  EXPECT_EQ(position.zone, 32);
  EXPECT_TRUE(position.northern_hemisphere);
  EXPECT_EQ(position.frameId(), "utm_32n");
  // Reference values computed with the transverse Mercator series of Snyder (1987).
  EXPECT_NEAR(position.easting, 293990.85, 0.5);
  EXPECT_NEAR(position.northing, 5629198.44, 0.5);

  EXPECT_EQ(
    aggregator.aggregate(kAachen, Odometry{}, Acceleration{}), AggregationEvent::kAggregated);
}

TEST(ObjectAggregator, ReportsZoneChangesAndHemisphere)
{
  ObjectAggregator aggregator;
  aggregator.aggregate(kAachen, Odometry{}, Acceleration{});
  EXPECT_EQ(
    aggregator.aggregate(kEdmonton, Odometry{}, Acceleration{}),
    AggregationEvent::kUtmZoneChanged);
  EXPECT_EQ(aggregator.lastObject()->position.frameId(), "utm_12n");
  EXPECT_NEAR(aggregator.lastObject()->position.easting, 332345.76, 0.5);
  EXPECT_NEAR(aggregator.lastObject()->position.northing, 5933818.45, 0.5);

  aggregator.aggregate(kSydney, Odometry{}, Acceleration{});
  EXPECT_EQ(aggregator.lastObject()->position.frameId(), "utm_56s");
  EXPECT_NEAR(aggregator.lastObject()->position.easting, 334368.63, 0.5);
  EXPECT_NEAR(aggregator.lastObject()->position.northing, 6250948.35, 0.5);
}

TEST(ObjectAggregator, ForcedZone)
{
  AggregatorConfig config;
  config.utm_zone = 31;
  ObjectAggregator aggregator(config);
  aggregator.aggregate(kAachen, Odometry{}, Acceleration{});
  EXPECT_EQ(aggregator.lastObject()->position.frameId(), "utm_31n");

  // Far outside the forced zone.
  EXPECT_EQ(
    aggregator.aggregate(kEdmonton, Odometry{}, Acceleration{}),
    AggregationEvent::kRejectedInvalidFix);
}

TEST(ObjectAggregator, RejectsInvalidFix)
{
  ObjectAggregator aggregator;
  GnssFix no_fix = kAachen;
  no_fix.has_fix = false;
  EXPECT_EQ(
    aggregator.aggregate(no_fix, Odometry{}, Acceleration{}),
    AggregationEvent::kRejectedInvalidFix);

  GnssFix nan_fix = kAachen;
  nan_fix.latitude = std::numeric_limits<double>::quiet_NaN();
  EXPECT_EQ(
    aggregator.aggregate(nan_fix, Odometry{}, Acceleration{}),
    AggregationEvent::kRejectedInvalidFix);

  EXPECT_FALSE(aggregator.lastObject().has_value());
  auto status = aggregator.status();
  EXPECT_EQ(status.rejected_invalid_fix, 2u);
  EXPECT_EQ(status.aggregated, 0u);
  EXPECT_TRUE(status.last_rejected);

  aggregator.aggregate(kAachen, Odometry{}, Acceleration{});
  status = aggregator.status();
  EXPECT_FALSE(status.last_rejected);
  EXPECT_EQ(status.aggregated, 1u);
  ASSERT_TRUE(status.last_position.has_value());
}

TEST(ObjectAggregator, FillsStateDimensionAndClass)
{
  AggregatorConfig config;
  config.object_length = 4.524;
  config.object_width = 1.838;
  config.object_class = 1;
  ObjectAggregator aggregator(config);

  // Heading east: vehicle and global frame coincide.
  Odometry odometry;
  odometry.yaw = 0.0;
  odometry.v_x = 3.0;
  odometry.v_y = 0.1;
  odometry.yaw_rate = 0.2;
  odometry.pose_covariance[0] = 1.0;    // x
  odometry.pose_covariance[7] = 2.0;    // y
  odometry.pose_covariance[35] = 3.0;   // yaw
  odometry.twist_covariance[0] = 4.0;   // v_x
  odometry.twist_covariance[35] = 5.0;  // yaw_rate
  Acceleration acceleration;
  acceleration.a_x = 0.7;
  acceleration.covariance[7] = 6.0;     // a_y

  aggregator.aggregate(kAachen, odometry, acceleration);
  const auto & object = *aggregator.lastObject();

  EXPECT_EQ(object.id, aggregator.status().object_id);
  EXPECT_DOUBLE_EQ(object.yaw, 0.0);
  EXPECT_DOUBLE_EQ(object.v_x, 3.0);
  EXPECT_DOUBLE_EQ(object.yaw_rate, 0.2);
  EXPECT_DOUBLE_EQ(object.a_x, 0.7);

  // Diagonal of the 8x8 covariance over (x, y, v_x, v_y, a_x, a_y, yaw, yaw_rate).
  EXPECT_DOUBLE_EQ(object.state_covariance[0 * 8 + 0], 1.0);
  EXPECT_DOUBLE_EQ(object.state_covariance[1 * 8 + 1], 2.0);
  // Plus the yaw uncertainty rotated into the global frame: d(v_x)/d(yaw) = -v_y and
  // d(a_y)/d(yaw) = a_x at yaw = 0.
  EXPECT_DOUBLE_EQ(object.state_covariance[2 * 8 + 2], 4.0 + 0.1 * 0.1 * 3.0);
  EXPECT_DOUBLE_EQ(object.state_covariance[5 * 8 + 5], 6.0 + 0.7 * 0.7 * 3.0);
  EXPECT_DOUBLE_EQ(object.state_covariance[6 * 8 + 6], 3.0);
  EXPECT_DOUBLE_EQ(object.state_covariance[7 * 8 + 7], 5.0);

  EXPECT_DOUBLE_EQ(object.length, 4.524);
  EXPECT_DOUBLE_EQ(object.width, 1.838);
  // Variances on the diagonal of the 3x3 dimension covariance, no cross terms.
  EXPECT_GT(object.dimension_covariance[0], 0.0);
  EXPECT_GT(object.dimension_covariance[4], 0.0);
  EXPECT_DOUBLE_EQ(object.dimension_covariance[3], 0.0);

  EXPECT_FLOAT_EQ(object.classification[1], 1.0f);
  EXPECT_FLOAT_EQ(object.classification[0], 0.0f);
}

TEST(ObjectAggregator, VelocityAndAccelerationAreGlobal)
{
  ObjectAggregator aggregator;

  // Heading north, driving forward and accelerating forward and to the left.
  Odometry odometry;
  odometry.yaw = std::numbers::pi / 2.0;
  odometry.v_x = 3.0;
  odometry.twist_covariance[0] = 4.0;   // Var(v_forward)
  odometry.twist_covariance[7] = 1.0;   // Var(v_left)
  Acceleration acceleration;
  acceleration.a_x = 1.0;
  acceleration.a_y = 0.5;
  acceleration.covariance[0] = 0.2;     // Var(a_forward)
  acceleration.covariance[7] = 0.1;     // Var(a_left)

  aggregator.aggregate(kAachen, odometry, acceleration);
  const auto & object = *aggregator.lastObject();

  // Forward is north (+y), left is west (-x).
  EXPECT_NEAR(object.v_x, 0.0, 1e-12);
  EXPECT_NEAR(object.v_y, 3.0, 1e-12);
  EXPECT_NEAR(object.a_x, -0.5, 1e-12);
  EXPECT_NEAR(object.a_y, 1.0, 1e-12);
  EXPECT_NEAR(std::hypot(object.v_x, object.v_y), 3.0, 1e-12);

  const auto & cov = object.state_covariance;
  EXPECT_NEAR(cov[2 * 8 + 2], 1.0, 1e-12);  // Var(v_x) = Var(v_left)
  EXPECT_NEAR(cov[3 * 8 + 3], 4.0, 1e-12);  // Var(v_y) = Var(v_forward)
  EXPECT_NEAR(cov[2 * 8 + 3], 0.0, 1e-12);
  EXPECT_NEAR(cov[4 * 8 + 4], 0.1, 1e-12);  // Var(a_x) = Var(a_left)
  EXPECT_NEAR(cov[5 * 8 + 5], 0.2, 1e-12);  // Var(a_y) = Var(a_forward)
}

TEST(ObjectAggregator, YawUncertaintySpreadsIntoGlobalVelocity)
{
  ObjectAggregator aggregator;
  Odometry odometry;
  odometry.yaw = 0.0;
  odometry.v_x = 3.0;
  odometry.pose_covariance[35] = 0.01;  // Var(yaw)

  aggregator.aggregate(kAachen, odometry, Acceleration{});
  const auto & cov = aggregator.lastObject()->state_covariance;

  // d(v_y)/d(yaw) = v = 3 m/s: Var(v_y) = 3^2 * Var(yaw), Cov(v_y, yaw) = 3 * Var(yaw).
  EXPECT_NEAR(cov[3 * 8 + 3], 0.09, 1e-12);
  EXPECT_NEAR(cov[3 * 8 + 6], 0.03, 1e-12);
  EXPECT_NEAR(cov[6 * 8 + 3], 0.03, 1e-12);
  EXPECT_NEAR(cov[2 * 8 + 2], 0.0, 1e-12);
  EXPECT_NEAR(cov[6 * 8 + 6], 0.01, 1e-12);
}

TEST(ObjectAggregator, IdIsStable)
{
  ObjectAggregator aggregator;
  aggregator.aggregate(kAachen, Odometry{}, Acceleration{});
  const auto id = aggregator.lastObject()->id;
  aggregator.aggregate(kEdmonton, Odometry{}, Acceleration{});
  EXPECT_EQ(aggregator.lastObject()->id, id);
}
