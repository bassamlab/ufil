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

#include <chrono>
#include <cstdint>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include "RsuCamGeneratorParameterRos.hpp"
#include "VehicleCamGeneratorParameterRos.hpp"
#include "VehicleCamReceiverParameterRos.hpp"

using ufil_etsi::RsuCamGeneratorParameterRos;
using ufil_etsi::VehicleCamGeneratorParameterRos;
using ufil_etsi::VehicleCamReceiverParameterRos;

class ParameterRosTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite() {rclcpp::init(0, nullptr);}
  static void TearDownTestSuite()
  {
    nodes_.clear();
    rclcpp::shutdown();
  }

  static rclcpp::node_interfaces::NodeParametersInterface::SharedPtr parameters(
    const std::vector<rclcpp::Parameter> & overrides = {})
  {
    nodes_.push_back(
      std::make_shared<rclcpp::Node>(
        "parameter_test_" + std::to_string(nodes_.size()),
        rclcpp::NodeOptions().parameter_overrides(overrides)));
    return nodes_.back()->get_node_parameters_interface();
  }

  static inline std::vector<std::shared_ptr<rclcpp::Node>> nodes_;
};

TEST_F(ParameterRosTest, VehicleCamGeneratorDefaults)
{
  auto node_parameters = parameters();
  VehicleCamGeneratorParameterRos parameters(node_parameters);
  EXPECT_EQ(parameters.settings().target_frame, "utm_32n");
  EXPECT_EQ(parameters.settings().object_array_index, 0u);
  EXPECT_EQ(parameters.config().utm_zone, (ufil_etsi::UtmZone{32, true}));
  EXPECT_EQ(parameters.config().vehicle_role, 0);
  EXPECT_EQ(parameters.config().time_shift, ufil_etsi::Duration(0));
  rcl_interfaces::msg::ParameterDescriptor descriptor =
    node_parameters->describe_parameters({"vehicle_role"}).front();
  EXPECT_TRUE(descriptor.read_only);
  EXPECT_FALSE(descriptor.description.empty());
}

TEST_F(ParameterRosTest, VehicleCamGeneratorOverrides)
{
  VehicleCamGeneratorParameterRos parameters(
    ParameterRosTest::parameters(
  {
    rclcpp::Parameter("target_frame", "utm_33s"),
    rclcpp::Parameter("object_array_index", 2),
    rclcpp::Parameter("vehicle_role", 6),
    rclcpp::Parameter("time_shift_nanosec", static_cast<int64_t>(500000000000)),
  }));
  EXPECT_EQ(parameters.config().utm_zone, (ufil_etsi::UtmZone{33, false}));
  EXPECT_EQ(parameters.settings().object_array_index, 2u);
  EXPECT_EQ(parameters.config().vehicle_role, 6);
  EXPECT_EQ(parameters.config().time_shift, std::chrono::seconds(500));
}

TEST_F(ParameterRosTest, VehicleCamGeneratorRejectsInvalidValues)
{
  EXPECT_THROW(
    VehicleCamGeneratorParameterRos(parameters({rclcpp::Parameter("target_frame", "map")})),
    std::invalid_argument);
  EXPECT_THROW(
    VehicleCamGeneratorParameterRos(parameters({rclcpp::Parameter("vehicle_role", 16)})),
    std::invalid_argument);
  EXPECT_THROW(
    VehicleCamGeneratorParameterRos(parameters({rclcpp::Parameter("vehicle_role", -1)})),
    std::invalid_argument);
  EXPECT_THROW(
    VehicleCamGeneratorParameterRos(parameters({rclcpp::Parameter("object_array_index", -1)})),
    std::invalid_argument);
  // Wrong type.
  EXPECT_ANY_THROW(
    VehicleCamGeneratorParameterRos(parameters({rclcpp::Parameter("vehicle_role", "car")})));
}

TEST_F(ParameterRosTest, VehicleCamReceiver)
{
  VehicleCamReceiverParameterRos defaults(parameters());
  EXPECT_EQ(defaults.settings().target_frame, "map");
  EXPECT_EQ(defaults.config().time_shift, ufil_etsi::Duration(0));

  VehicleCamReceiverParameterRos shifted(
    parameters({rclcpp::Parameter("time_shift_nanosec", static_cast<int64_t>(-1000))}));
  EXPECT_EQ(shifted.config().time_shift, ufil_etsi::Duration(-1000));

  EXPECT_THROW(
    VehicleCamReceiverParameterRos(parameters({rclcpp::Parameter("target_frame", "")})),
    std::invalid_argument);
}

TEST_F(ParameterRosTest, RsuCamGenerator)
{
  RsuCamGeneratorParameterRos defaults(parameters());
  EXPECT_TRUE(defaults.randomStationIdUsed());
  EXPECT_DOUBLE_EQ(
    defaults.config().position.latitude, RsuCamGeneratorParameterRos::kDefaultLatitude);
  EXPECT_DOUBLE_EQ(defaults.config().altitude, 0.0);

  RsuCamGeneratorParameterRos configured(
    parameters(
  {
    rclcpp::Parameter("station_id", static_cast<int64_t>(4294967295)),
    rclcpp::Parameter("position.latitude", 50.0),
    rclcpp::Parameter("position.longitude", 6.0),
    rclcpp::Parameter("position.altitude", 200.0),
  }));
  EXPECT_FALSE(configured.randomStationIdUsed());
  EXPECT_EQ(configured.config().station_id, 4294967295u);
  EXPECT_DOUBLE_EQ(configured.config().position.longitude, 6.0);

  EXPECT_THROW(
    RsuCamGeneratorParameterRos(
      parameters({rclcpp::Parameter("station_id", static_cast<int64_t>(4294967296))})),
    std::invalid_argument);
  EXPECT_THROW(
    RsuCamGeneratorParameterRos(parameters({rclcpp::Parameter("position.latitude", 91.0)})),
    std::invalid_argument);
}
