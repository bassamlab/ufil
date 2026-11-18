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

#include <cstdint>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include "CpmLabAdapterParameterRos.hpp"
#include "CpmLabSensorParameterRos.hpp"
#include "OscLaneControlLightParameterRos.hpp"

using ufil_examples_cpm_lab_adapter::CpmLabAdapterParameterRos;
using ufil_examples_cpm_lab_adapter::CpmLabSensorParameterRos;
using ufil_examples_cpm_lab_adapter::OscLaneControlLightParameterRos;

class ParameterRosTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite() {rclcpp::init(0, nullptr);}
  static void TearDownTestSuite() {rclcpp::shutdown();}

  template<typename ParameterRos>
  static ParameterRos load(const std::vector<rclcpp::Parameter> & overrides)
  {
    auto node = std::make_shared<rclcpp::Node>(
      "parameters", rclcpp::NodeOptions().parameter_overrides(overrides));
    return ParameterRos(node->get_node_parameters_interface());
  }
};

TEST_F(ParameterRosTest, AdapterDefaults)
{
  const auto parameters = load<CpmLabAdapterParameterRos>({});
  EXPECT_EQ(parameters.vehicleIds(), (std::vector<std::uint8_t>{1}));
  EXPECT_EQ(parameters.vehicleStateTopic(1), "/lab/vehicle_1/vehicleState");
  EXPECT_EQ(parameters.sourceFrame(), "lab");
  EXPECT_EQ(parameters.targetFrame(), "map");
  EXPECT_DOUBLE_EQ(parameters.publishRate(), 20.0);
  EXPECT_DOUBLE_EQ(parameters.stateTimeout(), 0.5);
}

TEST_F(ParameterRosTest, AdapterReadsValues)
{
  const auto parameters = load<CpmLabAdapterParameterRos>(
  {
    rclcpp::Parameter("vehicle_ids", std::vector<std::int64_t>{19, 0, 255}),
    rclcpp::Parameter("vehicle_state_topic", "vehicle_{id}/state_{id}"),
    rclcpp::Parameter("source_frame", "ips"),
    rclcpp::Parameter("target_frame", "utm_32n"),
    rclcpp::Parameter("publish_rate", 50.0),
    rclcpp::Parameter("state_timeout", 2.0),
  });
  EXPECT_EQ(parameters.vehicleIds(), (std::vector<std::uint8_t>{19, 0, 255}));
  EXPECT_EQ(parameters.vehicleStateTopic(19), "vehicle_19/state_19");
  EXPECT_EQ(parameters.sourceFrame(), "ips");
  EXPECT_EQ(parameters.targetFrame(), "utm_32n");
  EXPECT_DOUBLE_EQ(parameters.publishRate(), 50.0);
  EXPECT_DOUBLE_EQ(parameters.stateTimeout(), 2.0);
}

TEST_F(ParameterRosTest, AdapterRejectsInvalidValues)
{
  using Parameters = CpmLabAdapterParameterRos;
  const auto ids = [](std::vector<std::int64_t> values) {
      return rclcpp::Parameter("vehicle_ids", values);
    };
  EXPECT_THROW(load<Parameters>({ids({})}), std::invalid_argument);
  EXPECT_THROW(load<Parameters>({ids({1, -1})}), std::invalid_argument);
  EXPECT_THROW(load<Parameters>({ids({256})}), std::invalid_argument);
  EXPECT_THROW(load<Parameters>({ids({3, 4, 3})}), std::invalid_argument);
  EXPECT_THROW(
    load<Parameters>({rclcpp::Parameter("vehicle_state_topic", "/lab/vehicle/state")}),
    std::invalid_argument);
  EXPECT_THROW(load<Parameters>({rclcpp::Parameter("source_frame", "")}), std::invalid_argument);
  EXPECT_THROW(load<Parameters>({rclcpp::Parameter("target_frame", "")}), std::invalid_argument);
  EXPECT_THROW(load<Parameters>({rclcpp::Parameter("publish_rate", 0.0)}), std::invalid_argument);
  EXPECT_THROW(load<Parameters>({rclcpp::Parameter("state_timeout", 0.0)}), std::invalid_argument);
  EXPECT_THROW(
    load<Parameters>({rclcpp::Parameter("vehicle_ids", 1)}),
    rclcpp::exceptions::InvalidParameterTypeException);
}

TEST_F(ParameterRosTest, SensorDefaults)
{
  const auto parameters = load<CpmLabSensorParameterRos>({});
  EXPECT_EQ(parameters.vehicleId(), 1);
  EXPECT_EQ(parameters.targetFrame(), "map");
  EXPECT_DOUBLE_EQ(parameters.gnssFrequency(), 5.0);
  EXPECT_DOUBLE_EQ(parameters.odometryFrequency(), 50.0);
  EXPECT_TRUE(parameters.config().artificial_gnss_errors);
  EXPECT_DOUBLE_EQ(parameters.config().origin_latitude, 48.079772);
  EXPECT_DOUBLE_EQ(parameters.config().origin_longitude, 11.635734);
}

TEST_F(ParameterRosTest, SensorReadsValues)
{
  const auto parameters = load<CpmLabSensorParameterRos>(
  {
    rclcpp::Parameter("vehicle_id", 4),
    rclcpp::Parameter("target_frame", "map_utm"),
    rclcpp::Parameter("gnss.frequency", 10.0),
    rclcpp::Parameter("gnss.artificial_errors", false),
    rclcpp::Parameter("gnss.origin.latitude", 50.0),
    rclcpp::Parameter("gnss.origin.longitude", 6.0),
    rclcpp::Parameter("odometry.frequency", 100.0),
  });
  EXPECT_EQ(parameters.vehicleId(), 4);
  EXPECT_EQ(parameters.targetFrame(), "map_utm");
  EXPECT_DOUBLE_EQ(parameters.gnssFrequency(), 10.0);
  EXPECT_FALSE(parameters.config().artificial_gnss_errors);
  EXPECT_DOUBLE_EQ(parameters.config().origin_latitude, 50.0);
  EXPECT_DOUBLE_EQ(parameters.config().origin_longitude, 6.0);
  EXPECT_DOUBLE_EQ(parameters.odometryFrequency(), 100.0);
}

TEST_F(ParameterRosTest, SensorRejectsInvalidValues)
{
  using Parameters = CpmLabSensorParameterRos;
  EXPECT_THROW(load<Parameters>({rclcpp::Parameter("vehicle_id", 300)}), std::invalid_argument);
  EXPECT_THROW(load<Parameters>({rclcpp::Parameter("target_frame", "")}), std::invalid_argument);
  EXPECT_THROW(load<Parameters>({rclcpp::Parameter("gnss.frequency", 0.0)}), std::invalid_argument);
  EXPECT_THROW(
    load<Parameters>({rclcpp::Parameter("odometry.frequency", -5.0)}), std::invalid_argument);
  EXPECT_THROW(
    load<Parameters>({rclcpp::Parameter("gnss.origin.latitude", 95.0)}), std::invalid_argument);
  EXPECT_THROW(
    load<Parameters>({rclcpp::Parameter("gnss.origin.longitude", 200.0)}), std::invalid_argument);
  EXPECT_THROW(
    load<Parameters>({rclcpp::Parameter("gnss.frequency", 5)}),
    rclcpp::exceptions::InvalidParameterTypeException);
}

TEST_F(ParameterRosTest, OscDefaults)
{
  const auto parameters = load<OscLaneControlLightParameterRos>({});
  EXPECT_EQ(parameters.config().host, "127.0.0.1");
  EXPECT_EQ(parameters.config().port, 9000);
  EXPECT_EQ(parameters.config().address, "/lane_control_light");
}

TEST_F(ParameterRosTest, OscReadsValues)
{
  const auto parameters = load<OscLaneControlLightParameterRos>(
  {
    rclcpp::Parameter("receiver.host", "192.168.1.77"),
    rclcpp::Parameter("receiver.port", 8200),
    rclcpp::Parameter("osc_address", "/lights"),
  });
  EXPECT_EQ(parameters.config().host, "192.168.1.77");
  EXPECT_EQ(parameters.config().port, 8200);
  EXPECT_EQ(parameters.config().address, "/lights");
}

TEST_F(ParameterRosTest, OscRejectsInvalidValues)
{
  using Parameters = OscLaneControlLightParameterRos;
  EXPECT_THROW(load<Parameters>({rclcpp::Parameter("receiver.host", "")}), std::invalid_argument);
  EXPECT_THROW(load<Parameters>({rclcpp::Parameter("receiver.port", 0)}), std::invalid_argument);
  EXPECT_THROW(
    load<Parameters>({rclcpp::Parameter("receiver.port", 70000)}), std::invalid_argument);
  EXPECT_THROW(
    load<Parameters>({rclcpp::Parameter("osc_address", "lights")}), std::invalid_argument);
}
