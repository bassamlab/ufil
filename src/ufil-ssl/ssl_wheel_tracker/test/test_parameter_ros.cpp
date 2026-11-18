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

#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include "SslWheelTrackerParameterRos.hpp"

using ssl_wheel_tracker::DetectorConfig;
using ssl_wheel_tracker::SslWheelTrackerParameterRos;
using ssl_wheel_tracker::TrackerConfig;

class ParameterRosTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite() {rclcpp::init(0, nullptr);}
  static void TearDownTestSuite() {rclcpp::shutdown();}

  static std::shared_ptr<rclcpp::Node> makeNode(const std::vector<rclcpp::Parameter> & overrides)
  {
    return std::make_shared<rclcpp::Node>(
      "parameter_test", rclcpp::NodeOptions().parameter_overrides(overrides));
  }
};

TEST_F(ParameterRosTest, Defaults)
{
  auto node = makeNode({});
  SslWheelTrackerParameterRos parameters(node->get_node_parameters_interface());

  EXPECT_EQ(parameters.settings().input_topics, std::vector<std::string>{"ssl_1/map"});
  EXPECT_EQ(parameters.settings().tracking_frame, "map");
  EXPECT_DOUBLE_EQ(parameters.settings().update_period, 0.01);

  const DetectorConfig detector;
  EXPECT_EQ(parameters.detector().threshold, detector.threshold);
  EXPECT_EQ(parameters.detector().min_peak_distance, detector.min_peak_distance);
  EXPECT_EQ(parameters.detector().contact_radius, detector.contact_radius);
  EXPECT_EQ(parameters.detector().exclude_border, detector.exclude_border);

  const TrackerConfig tracker;
  EXPECT_DOUBLE_EQ(parameters.tracker().process_noise, tracker.process_noise);
  EXPECT_DOUBLE_EQ(parameters.tracker().measurement_noise, tracker.measurement_noise);
  EXPECT_DOUBLE_EQ(parameters.tracker().gate_distance, tracker.gate_distance);
  EXPECT_EQ(parameters.tracker().confirmation_hits, tracker.confirmation_hits);
  EXPECT_DOUBLE_EQ(parameters.tracker().deletion_timeout, tracker.deletion_timeout);
  EXPECT_EQ(parameters.tracker().history_length, tracker.history_length);

  EXPECT_TRUE(node->describe_parameter("tracking.gate_distance").read_only);
  EXPECT_FALSE(node->describe_parameter("detection.threshold").description.empty());
}

TEST_F(ParameterRosTest, ReadsOverrides)
{
  auto node = makeNode(
  {
    rclcpp::Parameter("input_topics", std::vector<std::string>{"ssl_1/map", "ssl_2/map"}),
    rclcpp::Parameter("tracking_frame", "odom"),
    rclcpp::Parameter("update_period", 0.05),
    rclcpp::Parameter("detection.threshold", 20),
    rclcpp::Parameter("detection.min_peak_distance", 3),
    rclcpp::Parameter("detection.contact_radius", 2),
    rclcpp::Parameter("detection.exclude_border", 0),
    rclcpp::Parameter("tracking.process_noise", 0.1),
    rclcpp::Parameter("tracking.measurement_noise", 0.02),
    rclcpp::Parameter("tracking.gate_distance", 1.0),
    rclcpp::Parameter("tracking.confirmation_hits", 2),
    rclcpp::Parameter("tracking.deletion_timeout", 0.5),
    rclcpp::Parameter("tracking.history_length", 20),
  });
  SslWheelTrackerParameterRos parameters(node->get_node_parameters_interface());

  EXPECT_EQ(parameters.settings().input_topics.size(), 2u);
  EXPECT_EQ(parameters.settings().tracking_frame, "odom");
  EXPECT_DOUBLE_EQ(parameters.settings().update_period, 0.05);
  EXPECT_EQ(parameters.detector().threshold, 20);
  EXPECT_EQ(parameters.detector().min_peak_distance, 3u);
  EXPECT_EQ(parameters.detector().contact_radius, 2u);
  EXPECT_EQ(parameters.detector().exclude_border, 0u);
  EXPECT_DOUBLE_EQ(parameters.tracker().process_noise, 0.1);
  EXPECT_DOUBLE_EQ(parameters.tracker().measurement_noise, 0.02);
  EXPECT_DOUBLE_EQ(parameters.tracker().gate_distance, 1.0);
  EXPECT_EQ(parameters.tracker().confirmation_hits, 2u);
  EXPECT_DOUBLE_EQ(parameters.tracker().deletion_timeout, 0.5);
  EXPECT_EQ(parameters.tracker().history_length, 20u);
}

TEST_F(ParameterRosTest, InvalidValuesThrow)
{
  const std::vector<rclcpp::Parameter> invalid{
    rclcpp::Parameter("input_topics", std::vector<std::string>{}),
    rclcpp::Parameter("input_topics", std::vector<std::string>{"a", "a"}),
    rclcpp::Parameter("input_topics", std::vector<std::string>{""}),
    rclcpp::Parameter("tracking_frame", ""),
    rclcpp::Parameter("update_period", 0.0),
    rclcpp::Parameter("detection.threshold", -1),
    rclcpp::Parameter("detection.threshold", 255),
    rclcpp::Parameter("detection.min_peak_distance", 0),
    rclcpp::Parameter("detection.contact_radius", 0),
    rclcpp::Parameter("detection.exclude_border", -1),
    rclcpp::Parameter("tracking.process_noise", 0.0),
    rclcpp::Parameter("tracking.measurement_noise", -0.01),
    rclcpp::Parameter("tracking.gate_distance", 0.0),
    rclcpp::Parameter("tracking.confirmation_hits", 0),
    rclcpp::Parameter("tracking.deletion_timeout", 0.0),
    rclcpp::Parameter("tracking.history_length", 0),
  };
  for (const auto & parameter : invalid) {
    auto node = makeNode({parameter});
    EXPECT_THROW(
      SslWheelTrackerParameterRos(node->get_node_parameters_interface()),
      std::invalid_argument) << parameter.get_name() << " = " << parameter.value_to_string();
  }
}

TEST_F(ParameterRosTest, WrongTypeThrows)
{
  auto node = makeNode({rclcpp::Parameter("detection.threshold", 10.0)});
  EXPECT_THROW(
    SslWheelTrackerParameterRos(node->get_node_parameters_interface()),
    rclcpp::exceptions::InvalidParameterTypeException);
}
