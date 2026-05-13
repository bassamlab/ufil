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

#ifndef UFIL_VISUALIZATION__OBJECT_LIST_MARKER_ARRAY_PARAMETERS_HPP_
#define UFIL_VISUALIZATION__OBJECT_LIST_MARKER_ARRAY_PARAMETERS_HPP_

#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/color_rgba.hpp>

namespace ufil_visualization
{

inline std_msgs::msg::ColorRGBA makeColor(const std::vector<double> & values)
{
  auto get = [&values](std::size_t index, double fallback) {
      return index < values.size() ? values[index] : fallback;
    };

  std_msgs::msg::ColorRGBA color;
  color.r = static_cast<float>(get(0, 0.5));
  color.g = static_cast<float>(get(1, 0.5));
  color.b = static_cast<float>(get(2, 0.5));
  color.a = static_cast<float>(get(3, 1.0));
  return color;
}

struct ObjectListMarkerArrayParameters
{
  explicit ObjectListMarkerArrayParameters(rclcpp::Node & node)
  {
    input_topic = node.declare_parameter<std::string>("input_topic", "objects");
    output_topic = node.declare_parameter<std::string>("output_topic", "object_markers");
    object_timeout = node.declare_parameter<double>("object_timeout", 1.0);

    body_color = makeColor(node.declare_parameter<std::vector<double>>(
      "body_color", {26.0 / 255.0, 95.0 / 255.0, 180.0 / 255.0, 1.0}));
    velocity_color = makeColor(node.declare_parameter<std::vector<double>>(
      "velocity_color", {165.0 / 255.0, 29.0 / 255.0, 45.0 / 255.0, 1.0}));
    text_color = makeColor(node.declare_parameter<std::vector<double>>(
      "text_color", {229.0 / 255.0, 165.0 / 255.0, 10.0 / 255.0, 1.0}));
    covariance_position_color = makeColor(node.declare_parameter<std::vector<double>>(
      "covariance_position_color", {204.0 / 255.0, 51.0 / 255.0, 204.0 / 255.0, 0.3}));
    covariance_orientation_color = makeColor(node.declare_parameter<std::vector<double>>(
      "covariance_orientation_color", {1.0, 1.0, 127.0 / 255.0, 0.5}));
    axis_x_color = makeColor(node.declare_parameter<std::vector<double>>(
      "axis_x_color", {1.0, 0.0, 0.0, 1.0}));
    axis_y_color = makeColor(node.declare_parameter<std::vector<double>>(
      "axis_y_color", {0.0, 1.0, 0.0, 1.0}));
    axis_z_color = makeColor(node.declare_parameter<std::vector<double>>(
      "axis_z_color", {0.0, 0.0, 1.0, 1.0}));

    velocity_arrow_shaft_diameter = node.declare_parameter<double>("velocity_arrow_shaft_diameter",
        0.1);
    velocity_arrow_head_diameter = node.declare_parameter<double>("velocity_arrow_head_diameter",
        0.15);
    velocity_arrow_head_length = node.declare_parameter<double>("velocity_arrow_head_length", 0.2);
    body_axis_arrow_shaft_diameter =
      node.declare_parameter<double>("body_axis_arrow_shaft_diameter", 0.1);
    body_axis_arrow_head_diameter = node.declare_parameter<double>("body_axis_arrow_head_diameter",
        0.15);
    body_axis_arrow_head_length = node.declare_parameter<double>("body_axis_arrow_head_length",
        0.2);
    covariance_line_width = node.declare_parameter<double>("covariance_line_width", 0.05);
    text_size = node.declare_parameter<double>("text_size", 1.0);
    ellipse_segments = node.declare_parameter<int>("ellipse_segments", 64);
  }

  std::string input_topic;
  std::string output_topic;
  double object_timeout{};

  std_msgs::msg::ColorRGBA body_color;
  std_msgs::msg::ColorRGBA velocity_color;
  std_msgs::msg::ColorRGBA text_color;
  std_msgs::msg::ColorRGBA covariance_position_color;
  std_msgs::msg::ColorRGBA covariance_orientation_color;
  std_msgs::msg::ColorRGBA axis_x_color;
  std_msgs::msg::ColorRGBA axis_y_color;
  std_msgs::msg::ColorRGBA axis_z_color;

  double velocity_arrow_shaft_diameter{};
  double velocity_arrow_head_diameter{};
  double velocity_arrow_head_length{};
  double body_axis_arrow_shaft_diameter{};
  double body_axis_arrow_head_diameter{};
  double body_axis_arrow_head_length{};
  double covariance_line_width{};
  double text_size{};
  int ellipse_segments{};
};

}  // namespace ufil_visualization

#endif  // UFIL_VISUALIZATION__OBJECT_LIST_MARKER_ARRAY_PARAMETERS_HPP_
