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


#ifndef UFIL_VISUALIZATION__LANELET_MAP_MARKER_ARRAY_PARAMETERS_HPP_
#define UFIL_VISUALIZATION__LANELET_MAP_MARKER_ARRAY_PARAMETERS_HPP_

#include <string>
#include <vector>

#include <geometry_msgs/msg/point.hpp>
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

struct LaneletMapMarkerArrayParameters
{
  explicit LaneletMapMarkerArrayParameters(rclcpp::Node & node)
  {
    map_path = node.declare_parameter<std::string>("map_path", "");
    frame_id = node.declare_parameter<std::string>("frame_id", "map");
    lanelet_fill_color = makeColor(node.declare_parameter<std::vector<double>>(
      "lanelet_fill_color", {0.369, 0.361, 0.392, 0.8}));
    lanelet_border_color = makeColor(node.declare_parameter<std::vector<double>>(
      "lanelet_border_color", {0.239, 0.220, 0.275, 1.0}));
    road_marking_color = makeColor(node.declare_parameter<std::vector<double>>(
      "road_marking_color", {1.0, 1.0, 1.0, 1.0}));
    area_default_color = makeColor(node.declare_parameter<std::vector<double>>(
      "area_default_color", {0.380, 0.208, 0.514, 0.8}));
    area_vegetation_color = makeColor(node.declare_parameter<std::vector<double>>(
      "area_vegetation_color", {0.149, 0.635, 0.412, 0.8}));
    area_parking_color = makeColor(node.declare_parameter<std::vector<double>>(
      "area_parking_color", {0.369, 0.361, 0.392, 0.8}));
    area_keepout_color = makeColor(node.declare_parameter<std::vector<double>>(
      "area_keepout_color", {0.647, 0.114, 0.176, 0.8}));
    area_building_color = makeColor(node.declare_parameter<std::vector<double>>(
      "area_building_color", {0.239, 0.220, 0.275, 1.0}));
    area_traffic_island_color = makeColor(node.declare_parameter<std::vector<double>>(
      "area_traffic_island_color", {0.239, 0.220, 0.275, 0.8}));
    lanelet_fill_z = node.declare_parameter<double>("lanelet_fill_z", -0.10);
    lanelet_border_z = node.declare_parameter<double>("lanelet_border_z", -0.05);
    road_marking_z = node.declare_parameter<double>("road_marking_z", 0.05);
    area_z = node.declare_parameter<double>("area_z", -0.15);
    line_width = node.declare_parameter<double>("line_width", 0.1);
    area_scale = node.declare_parameter<double>("area_scale", 1.0);
    use_transient_local = node.declare_parameter<bool>("use_transient_local", true);
  }

  std::string map_path;
  std::string frame_id;
  std_msgs::msg::ColorRGBA lanelet_fill_color;
  std_msgs::msg::ColorRGBA lanelet_border_color;
  std_msgs::msg::ColorRGBA road_marking_color;
  std_msgs::msg::ColorRGBA area_default_color;
  std_msgs::msg::ColorRGBA area_vegetation_color;
  std_msgs::msg::ColorRGBA area_parking_color;
  std_msgs::msg::ColorRGBA area_keepout_color;
  std_msgs::msg::ColorRGBA area_building_color;
  std_msgs::msg::ColorRGBA area_traffic_island_color;
  double lanelet_fill_z{};
  double lanelet_border_z{};
  double road_marking_z{};
  double area_z{};
  double line_width{};
  double area_scale{};
  bool use_transient_local{};
};

}  // namespace ufil_visualization

#endif  // UFIL_VISUALIZATION__LANELET_MAP_MARKER_ARRAY_PARAMETERS_HPP_
