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

#include <cstddef>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include "LaneletMapMarkerArrayParameterRos.hpp"
#include "ObjectListMarkerArrayParameterRos.hpp"

using ufil_visualization::LaneletMapMarkerArrayParameterRos;
using ufil_visualization::LaneletMapStyle;
using ufil_visualization::ObjectListMarkerArrayParameterRos;
using ufil_visualization::ObjectMarkerConfig;

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

TEST_F(ParameterRosTest, ObjectListDefaults)
{
  auto node = makeNode({});
  ObjectListMarkerArrayParameterRos parameters(node->get_node_parameters_interface());
  const ObjectMarkerConfig defaults;
  EXPECT_DOUBLE_EQ(parameters.config().marker_lifetime, defaults.marker_lifetime);
  EXPECT_FLOAT_EQ(parameters.config().color.b, defaults.color.b);
  EXPECT_DOUBLE_EQ(parameters.config().body_alpha, defaults.body_alpha);
  EXPECT_EQ(parameters.config().ellipse_segments, defaults.ellipse_segments);
  EXPECT_EQ(node->get_parameter("covariance.ellipse_segments").as_int(), 64);

  const auto descriptor = node->describe_parameter("velocity.head_length");
  EXPECT_TRUE(descriptor.read_only);
  EXPECT_FALSE(descriptor.description.empty());
}

TEST_F(ParameterRosTest, ObjectListOverrides)
{
  auto node = makeNode(
  {
    rclcpp::Parameter("marker_lifetime", 0.0),
    rclcpp::Parameter("color", std::vector<double>{0.1, 0.2, 0.3, 0.4}),
    rclcpp::Parameter("body.alpha", 0.25),
    rclcpp::Parameter("covariance.ellipse_segments", 16),
    rclcpp::Parameter("label.icons", false),
  });
  ObjectListMarkerArrayParameterRos parameters(node->get_node_parameters_interface());
  EXPECT_FALSE(parameters.config().label_icons);
  EXPECT_DOUBLE_EQ(parameters.config().marker_lifetime, 0.0);
  EXPECT_FLOAT_EQ(parameters.config().color.r, 0.1F);
  EXPECT_FLOAT_EQ(parameters.config().color.a, 0.4F);
  EXPECT_DOUBLE_EQ(parameters.config().body_alpha, 0.25);
  EXPECT_EQ(parameters.config().ellipse_segments, 16);
}

TEST_F(ParameterRosTest, ObjectListInvalidValuesThrow)
{
  const std::vector<rclcpp::Parameter> invalid{
    rclcpp::Parameter("marker_lifetime", -1.0),
    rclcpp::Parameter("color", std::vector<double>{0.1, 0.2, 0.3}),
    rclcpp::Parameter("color", std::vector<double>{0.1, 0.2, 0.3, 2.0}),
    rclcpp::Parameter("body.alpha", 1.5),
    rclcpp::Parameter("covariance.line_width", 0.0),
    rclcpp::Parameter("covariance.ellipse_segments", 2),
  };
  for (const auto & parameter : invalid) {
    auto node = makeNode({parameter});
    EXPECT_THROW(
      ObjectListMarkerArrayParameterRos(node->get_node_parameters_interface()),
      std::invalid_argument) << parameter.get_name();
  }
}

TEST_F(ParameterRosTest, ObjectListWithoutSourcesVisualizesObjectList)
{
  auto node = makeNode({rclcpp::Parameter("color", std::vector<double>{0.1, 0.2, 0.3, 0.4})});
  ObjectListMarkerArrayParameterRos parameters(node->get_node_parameters_interface());
  ASSERT_EQ(parameters.sources().size(), 1u);
  EXPECT_EQ(parameters.sources()[0].input_topic, "object_list");
  EXPECT_EQ(parameters.sources()[0].output_topic, "object_markers");
  EXPECT_FLOAT_EQ(parameters.sources()[0].config.color.a, 0.4F);
}

TEST_F(ParameterRosTest, ObjectListSources)
{
  auto node = makeNode(
  {
    rclcpp::Parameter("sources", std::vector<std::string>{"osn", "rsu_cam"}),
    rclcpp::Parameter("color", std::vector<double>{0.1, 0.2, 0.3, 0.4}),
    rclcpp::Parameter("body.alpha", 0.25),
    rclcpp::Parameter("rsu_cam.input_topic", "/rsu/cam/object_list"),
    rclcpp::Parameter("rsu_cam.output_topic", "/rsu/cam/object_markers"),
    rclcpp::Parameter("rsu_cam.color", std::vector<double>{0.5, 0.6, 0.7, 0.8}),
  });
  ObjectListMarkerArrayParameterRos parameters(node->get_node_parameters_interface());
  const auto & sources = parameters.sources();
  ASSERT_EQ(sources.size(), 2u);

  EXPECT_EQ(sources[0].name, "osn");
  EXPECT_EQ(sources[0].input_topic, "osn/object_list");
  EXPECT_EQ(sources[0].output_topic, "osn/object_markers");
  EXPECT_FLOAT_EQ(sources[0].config.color.r, 0.1F);
  EXPECT_DOUBLE_EQ(sources[0].config.body_alpha, 0.25);

  EXPECT_EQ(sources[1].input_topic, "/rsu/cam/object_list");
  EXPECT_EQ(sources[1].output_topic, "/rsu/cam/object_markers");
  EXPECT_FLOAT_EQ(sources[1].config.color.r, 0.5F);
  EXPECT_DOUBLE_EQ(sources[1].config.body_alpha, 0.25);
  EXPECT_TRUE(node->describe_parameter("rsu_cam.color").read_only);
}

TEST_F(ParameterRosTest, ObjectListInvalidSourcesThrow)
{
  using Parameters = std::vector<rclcpp::Parameter>;
  using Names = std::vector<std::string>;
  const std::vector<Parameters> invalid{
    {rclcpp::Parameter("sources", Names{""})},
    {rclcpp::Parameter("sources", Names{"1osn"})},
    {rclcpp::Parameter("sources", Names{"rsu/cam"})},
    {rclcpp::Parameter("sources", Names{"body"})},
    {rclcpp::Parameter("sources", Names{"osn", "osn"})},
    {rclcpp::Parameter("sources", Names{"osn", "obu"}),
      rclcpp::Parameter("obu.output_topic", "osn/object_markers")},
    {rclcpp::Parameter("sources", Names{"osn"}), rclcpp::Parameter("osn.input_topic", "")},
    {rclcpp::Parameter("sources", Names{"osn"}),
      rclcpp::Parameter("osn.color", std::vector<double>{0.1, 0.2, 0.3, 2.0})},
  };
  for (std::size_t i = 0; i < invalid.size(); ++i) {
    auto node = makeNode(invalid[i]);
    EXPECT_THROW(
      ObjectListMarkerArrayParameterRos(node->get_node_parameters_interface()),
      std::invalid_argument) << "case " << i;
  }
}

TEST_F(ParameterRosTest, LaneletMapDefaults)
{
  auto node = makeNode({});
  LaneletMapMarkerArrayParameterRos parameters(node->get_node_parameters_interface());
  const LaneletMapStyle defaults;
  EXPECT_EQ(parameters.settings().map_path, "");
  EXPECT_EQ(parameters.settings().frame_id, "map");
  EXPECT_DOUBLE_EQ(parameters.style().line_width, defaults.line_width);
  EXPECT_TRUE(parameters.style().show_lanelet_ids);
  EXPECT_FLOAT_EQ(
    parameters.style().area_keepout_color.r, defaults.area_keepout_color.r);
  EXPECT_TRUE(node->describe_parameter("area.keepout_color").read_only);
}

TEST_F(ParameterRosTest, LaneletMapOverrides)
{
  auto node = makeNode(
  {
    rclcpp::Parameter("map_path", "/maps/test.osm"),
    rclcpp::Parameter("frame_id", "map_utm"),
    rclcpp::Parameter("lanelet_ids.show", false),
    rclcpp::Parameter("crosswalk.stripe_gap", 0.0),
  });
  LaneletMapMarkerArrayParameterRos parameters(node->get_node_parameters_interface());
  EXPECT_EQ(parameters.settings().map_path, "/maps/test.osm");
  EXPECT_EQ(parameters.settings().frame_id, "map_utm");
  EXPECT_FALSE(parameters.style().show_lanelet_ids);
  EXPECT_DOUBLE_EQ(parameters.style().crosswalk_stripe_gap, 0.0);
}

TEST_F(ParameterRosTest, LaneletMapInvalidValuesThrow)
{
  const std::vector<rclcpp::Parameter> invalid{
    rclcpp::Parameter("frame_id", ""),
    rclcpp::Parameter("line_width", -0.1),
    rclcpp::Parameter("crosswalk.stripe_gap", -0.5),
    rclcpp::Parameter("lanelet.fill_color", std::vector<double>{0.1, 0.2, 0.3, 0.4, 0.5}),
    rclcpp::Parameter("area.z", std::string("low")),
  };
  for (const auto & parameter : invalid) {
    auto node = makeNode({parameter});
    EXPECT_ANY_THROW(
      LaneletMapMarkerArrayParameterRos(node->get_node_parameters_interface())) <<
      parameter.get_name();
  }
}
