// Copyright 2025 Chair of Embedded Software (Computer Science 11) - RWTH Aachen University
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

// Conversions between ROS 2 messages and the ROS-independent ufil_pcl_preprocessor types.

#include <Eigen/Geometry>
#include <pcl/PCLPointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>

#include <cmath>
#include <vector>

#include <tf2_eigen/tf2_eigen.hpp>

#include <builtin_interfaces/msg/duration.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/header.hpp>
#include <ufil_msgs/msg/sensor_fov.hpp>
#include <ufil_ros/ufil_ros.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "ufil_pcl_preprocessor/clipped_area.hpp"

namespace ufil_pcl_preprocessor::conversions
{

inline pcl::PCLPointCloud2 cloudFromMsg(const sensor_msgs::msg::PointCloud2 & msg)
{
  pcl::PCLPointCloud2 cloud;
  pcl_conversions::toPCL(msg, cloud);
  return cloud;
}

inline sensor_msgs::msg::PointCloud2 cloudToMsg(
  const pcl::PCLPointCloud2 & cloud, const std_msgs::msg::Header & header)
{
  sensor_msgs::msg::PointCloud2 msg;
  pcl_conversions::fromPCL(cloud, msg);
  msg.header = header;
  return msg;
}

inline Eigen::Isometry3d transformFromMsg(const geometry_msgs::msg::TransformStamped & msg)
{
  return tf2::transformToEigen(msg.transform);
}

inline nav_msgs::msg::OccupancyGrid occlusionGridToMsg(
  const ufil::type::OccupancyGrid & grid, const std_msgs::msg::Header & header)
{
  nav_msgs::msg::OccupancyGrid msg = ufil_ros::occupancyGridToMsg(grid);
  msg.header = header;
  msg.info.map_load_time = header.stamp;
  return msg;
}

inline ufil_msgs::msg::SensorFOV fovToMsg(
  const ufil::type::SensorFOV & fov, const std_msgs::msg::Header & header)
{
  ufil_msgs::msg::SensorFOV msg = ufil_ros::sensorFovToMsg(fov);
  msg.header = header;
  return msg;
}

/// Outline of the field of view: a horizontal arc (red) and a vertical arc (green).
inline visualization_msgs::msg::MarkerArray fovToMarkers(const ufil_msgs::msg::SensorFOV & fov)
{
  constexpr double kAngleStep = 0.05;

  visualization_msgs::msg::MarkerArray markers;

  geometry_msgs::msg::Point origin;
  origin.x = fov.origin.x;
  origin.y = fov.origin.y;
  origin.z = fov.origin.z;

  visualization_msgs::msg::Marker horizontal;
  horizontal.header = fov.header;
  horizontal.ns = "fov";
  horizontal.id = 0;
  horizontal.type = visualization_msgs::msg::Marker::LINE_STRIP;
  horizontal.action = visualization_msgs::msg::Marker::ADD;
  horizontal.pose.orientation.w = 1.0;
  horizontal.scale.x = 0.1;
  horizontal.color.a = 1.0;
  horizontal.color.r = 1.0;

  horizontal.points.push_back(origin);
  for (double phi = fov.phi_min; phi <= fov.phi_max; phi += kAngleStep) {
    geometry_msgs::msg::Point pt;
    pt.x = origin.x + fov.r_max * std::cos(phi);
    pt.y = origin.y + fov.r_max * std::sin(phi);
    pt.z = origin.z;
    horizontal.points.push_back(pt);
  }
  horizontal.points.push_back(origin);
  markers.markers.push_back(horizontal);

  visualization_msgs::msg::Marker vertical = horizontal;
  vertical.id = 1;
  vertical.color.r = 0.0;
  vertical.color.g = 1.0;
  vertical.points.clear();

  const double phi_mid = 0.5 * (fov.phi_min + fov.phi_max);
  vertical.points.push_back(origin);
  for (double theta = fov.theta_min; theta <= fov.theta_max; theta += kAngleStep) {
    geometry_msgs::msg::Point pt;
    pt.x = origin.x + fov.r_max * std::cos(theta) * std::cos(phi_mid);
    pt.y = origin.y + fov.r_max * std::cos(theta) * std::sin(phi_mid);
    pt.z = origin.z + fov.r_max * std::sin(theta);
    vertical.points.push_back(pt);
  }
  vertical.points.push_back(origin);
  markers.markers.push_back(vertical);

  return markers;
}

/// Semi-transparent boxes for the clipped areas, valid for 0.5 s.
inline visualization_msgs::msg::MarkerArray clippedAreasToMarkers(
  const std::vector<ClippedArea> & areas, const std_msgs::msg::Header & header)
{
  visualization_msgs::msg::MarkerArray markers;
  int id = 0;
  for (const auto & area : areas) {
    const Eigen::Quaterniond orientation(
      Eigen::AngleAxisd(area.yaw, Eigen::Vector3d::UnitZ()));

    visualization_msgs::msg::Marker marker;
    marker.header = header;
    marker.ns = "clipped_areas";
    marker.id = id++;
    marker.type = visualization_msgs::msg::Marker::CUBE;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.position.x = area.x;
    marker.pose.position.y = area.y;
    marker.pose.position.z = area.z;
    marker.pose.orientation.x = orientation.x();
    marker.pose.orientation.y = orientation.y();
    marker.pose.orientation.z = orientation.z();
    marker.pose.orientation.w = orientation.w();
    marker.scale.x = area.length;
    marker.scale.y = area.width;
    marker.scale.z = area.height;
    marker.color.a = 0.5;
    marker.lifetime.sec = 0;
    marker.lifetime.nanosec = 500000000u;
    markers.markers.push_back(marker);
  }
  return markers;
}

}  // namespace ufil_pcl_preprocessor::conversions

#endif  // UFIL_PCL_PREPROCESSOR__COMPONENT__UFIL_PCL_PREPROCESSOR_ROS_HPP_
