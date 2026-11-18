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

#include <Eigen/Geometry>
#include <numbers>  // listed with the C headers, cpplint does not know this C++20 header

#include <cstdint>
#include <stdexcept>

#include "ufil_examples_cpm_lab_adapter/sensor_simulator.hpp"

using ufil_examples_cpm_lab_adapter::kLabScale;
using ufil_examples_cpm_lab_adapter::SensorSimulator;
using ufil_examples_cpm_lab_adapter::SensorSimulatorConfig;
using ufil_examples_cpm_lab_adapter::StateCheck;
using ufil_examples_cpm_lab_adapter::VehicleState;

namespace
{

constexpr std::int64_t kSecond = 1000000000;

SensorSimulatorConfig exactConfig()
{
  SensorSimulatorConfig config;
  config.artificial_gnss_errors = false;
  return config;
}

VehicleState stateAt(std::int64_t stamp_nanoseconds)
{
  VehicleState state;
  state.stamp_nanoseconds = stamp_nanoseconds;
  state.frame_id = "lab";
  return state;
}

}  // namespace

TEST(SensorSimulatorTest, DiscardsExpiredStates)
{
  SensorSimulator simulator(exactConfig());
  EXPECT_EQ(simulator.checkState(0), StateCheck::kNoState);

  simulator.setState(stateAt(10 * kSecond));
  EXPECT_EQ(simulator.checkState(10 * kSecond + kSecond / 2), StateCheck::kValid);
  EXPECT_TRUE(simulator.status().has_state);
  EXPECT_EQ(simulator.checkState(11 * kSecond), StateCheck::kExpired);
  EXPECT_EQ(simulator.checkState(11 * kSecond), StateCheck::kNoState);
  EXPECT_FALSE(simulator.state());

  const auto status = simulator.status();
  EXPECT_EQ(status.states_received, 1u);
  EXPECT_EQ(status.states_expired, 1u);
  EXPECT_FALSE(status.has_state);
  EXPECT_EQ(status.last_state_stamp_nanoseconds, 10 * kSecond);
}

TEST(SensorSimulatorTest, MeasurementsRequireAState)
{
  SensorSimulator simulator(exactConfig());
  EXPECT_THROW(simulator.gnssFix(Eigen::Isometry3d::Identity()), std::logic_error);
  EXPECT_THROW(simulator.imu(Eigen::Isometry3d::Identity()), std::logic_error);
  EXPECT_THROW(simulator.wheelOdometry(), std::logic_error);
}

TEST(SensorSimulatorTest, GnssFixIsOffsetFromTheOrigin)
{
  const auto config = exactConfig();
  SensorSimulator simulator(config);
  simulator.setState(stateAt(0));
  const auto origin = simulator.gnssFix(Eigen::Isometry3d::Identity());
  EXPECT_NEAR(origin.latitude, config.origin_latitude, 1e-9);
  EXPECT_NEAR(origin.longitude, config.origin_longitude, 1e-9);
  EXPECT_DOUBLE_EQ(origin.position_variance, 0.1);

  // 1 m north in the lab is 18 m north in the world: about 1.6e-4 deg of latitude. The transform
  // into the target frame is applied before scaling.
  auto state = stateAt(0);
  state.position = {0.0, 0.5, 0.0};
  simulator.setState(state);
  const auto north = simulator.gnssFix(
    Eigen::Isometry3d(Eigen::Translation3d(0.0, 0.5, 0.0)));
  EXPECT_NEAR(north.latitude - config.origin_latitude, kLabScale / 111200.0, 2e-6);
  EXPECT_NEAR(north.longitude, config.origin_longitude, 1e-5);
  EXPECT_EQ(simulator.status().gnss_fixes, 2u);
}

TEST(SensorSimulatorTest, ArtificialErrorsIncreaseTheVariance)
{
  SensorSimulator simulator({}, 3);
  simulator.setState(stateAt(0));
  const auto fix = simulator.gnssFix(Eigen::Isometry3d::Identity());
  EXPECT_DOUBLE_EQ(fix.position_variance, 1.0);
  EXPECT_NEAR(fix.latitude, simulator.config().origin_latitude, 1e-3);
}

TEST(SensorSimulatorTest, ImuAndOdometry)
{
  SensorSimulator simulator(exactConfig());
  auto state = stateAt(0);
  state.yaw = 0.25;
  state.linear_velocity = {1.0, 0.5, 3.0};
  state.angular_velocity = {0.0, 0.0, 0.1};
  state.linear_acceleration = {0.2, 0.0, 0.0};
  simulator.setState(state);

  const Eigen::Isometry3d transform(
    Eigen::AngleAxisd(std::numbers::pi / 2.0, Eigen::Vector3d::UnitZ()));
  const auto imu = simulator.imu(transform);
  EXPECT_NEAR(imu.yaw, 0.25 + std::numbers::pi / 2.0, 1e-12);
  EXPECT_EQ(imu.angular_velocity, state.angular_velocity);
  EXPECT_EQ(imu.linear_acceleration, state.linear_acceleration);

  const auto odometry = simulator.wheelOdometry();
  EXPECT_EQ(odometry.linear_velocity, Eigen::Vector3d(kLabScale, 0.5 * kLabScale, 0.0));
}

TEST(SensorSimulatorTest, RejectsInvalidConfiguration)
{
  SensorSimulatorConfig config;
  config.origin_latitude = 91.0;
  EXPECT_THROW(SensorSimulator{config}, std::invalid_argument);
  config = {};
  config.origin_longitude = -181.0;
  EXPECT_THROW(SensorSimulator{config}, std::invalid_argument);
  config = {};
  config.max_state_age = 0.0;
  EXPECT_THROW(SensorSimulator{config}, std::invalid_argument);
  config = {};
  config.gnss_errors.number_of_frequency_bands = 0;
  EXPECT_THROW(SensorSimulator{config}, std::invalid_argument);
}
