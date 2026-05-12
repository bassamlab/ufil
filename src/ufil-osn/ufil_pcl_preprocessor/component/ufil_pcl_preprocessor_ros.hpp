// Copyright 2025 Chair of Embedded Software (Computer Science 11) - RWHT Aachen University
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

#ifndef UFIL_PCL_PREPROCESSOR__COMPONENT__UFIL_PCL_PREPROCESSOR_ROS_HPP_
#define UFIL_PCL_PREPROCESSOR__COMPONENT__UFIL_PCL_PREPROCESSOR_ROS_HPP_

#include <cmath>
#include <algorithm>
#include <string>

#include "ufil_msgs/msg/sensor_fov.hpp"

#include <geometry_msgs/msg/point.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <rclcpp/time.hpp>

namespace ufil_pcl_ros
{

inline visualization_msgs::msg::MarkerArray createFovMarkerArray(
  const ufil_msgs::msg::SensorFOV & fov_msg)
{
  visualization_msgs::msg::MarkerArray markers;

  // --- Horizontal FOV marker ---
  visualization_msgs::msg::Marker horizontal_marker;
  horizontal_marker.header = fov_msg.header;
  horizontal_marker.id = 0;
  horizontal_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
  horizontal_marker.action = visualization_msgs::msg::Marker::ADD;
  horizontal_marker.scale.x = 0.1;  // line width
  horizontal_marker.color.a = 1.0;
  horizontal_marker.color.r = 1.0;
  horizontal_marker.color.g = 0.0;
  horizontal_marker.color.b = 0.0;
  horizontal_marker.lifetime = rclcpp::Duration(0, 0);  // infinite lifetime

  geometry_msgs::msg::Point origin;
  origin.x = fov_msg.origin.x;
  origin.y = fov_msg.origin.y;
  origin.z = fov_msg.origin.z;

  horizontal_marker.points.push_back(origin);
  // horizontal arc from phi_min to phi_max
  const double phi_step = 0.05;
  for (double phi = fov_msg.phi_min; phi <= fov_msg.phi_max; phi += phi_step) {
    geometry_msgs::msg::Point pt;
    double r = fov_msg.r_max;
    pt.x = origin.x + r * std::cos(phi);
    pt.y = origin.y + r * std::sin(phi);
    pt.z = origin.z;
    horizontal_marker.points.push_back(pt);
  }
  horizontal_marker.points.push_back(origin);
  markers.markers.push_back(horizontal_marker);

  // --- Vertical FOV marker ---
  visualization_msgs::msg::Marker vertical_marker = horizontal_marker;
  vertical_marker.id = 1;
  vertical_marker.color.r = 0.0;
  vertical_marker.color.g = 1.0;
  vertical_marker.color.b = 0.0;
  vertical_marker.points.clear();

  vertical_marker.points.push_back(origin);
  // vertical strip along phi midpoint
  double phi_mid = 0.5 * (fov_msg.phi_min + fov_msg.phi_max);
  const double theta_step = 0.05;
  for (double theta = fov_msg.theta_min; theta <= fov_msg.theta_max; theta += theta_step) {
    geometry_msgs::msg::Point pt;
    double r = fov_msg.r_max;
    pt.x = origin.x + r * std::cos(theta) * std::cos(phi_mid);
    pt.y = origin.y + r * std::cos(theta) * std::sin(phi_mid);
    pt.z = origin.z + r * std::sin(theta);
    vertical_marker.points.push_back(pt);
  }
  vertical_marker.points.push_back(origin);
  markers.markers.push_back(vertical_marker);

  return markers;
}

}  // namespace ufil_pcl_ros

#endif  // UFIL_PCL_PREPROCESSOR__COMPONENT__UFIL_PCL_PREPROCESSOR_ROS_HPP_
