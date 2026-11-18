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

#ifndef UFIL_OSN_TRACKER__COMPONENT__UFIL_OSN_TRACKER_ROS_HPP_
#define UFIL_OSN_TRACKER__COMPONENT__UFIL_OSN_TRACKER_ROS_HPP_

// Conversions between ROS 2 messages and the ROS-independent ufil_osn_tracker types.

#include <map>
#include <set>

#include <builtin_interfaces/msg/time.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <rclcpp/time.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/header.hpp>
#include <ufil_msgs/msg/object.hpp>
#include <ufil_msgs/msg/object_list.hpp>
#include <ufil_msgs/msg/sensor_fov.hpp>
#include <ufil_object_tracking/types/time.hpp>
#include <ufil_ros/ufil_ros.hpp>

#include "ufil_osn_tracker/definitions.hpp"

namespace ufil_osn_tracker::conversions
{

inline std::set<Detection> detectionsFromMsg(const sensor_msgs::msg::PointCloud2 & msg)
{
  return ufil_ros::pointCloudDetectionsFromMsg(msg);
}

inline ufil::type::Timestamp timestampFromMsg(const builtin_interfaces::msg::Time & stamp)
{
  return ufil::from_nanoseconds<ufil::type::Timestamp>(rclcpp::Time(stamp).nanoseconds());
}

inline ufil::type::SensorFOV fovFromMsg(const ufil_msgs::msg::SensorFOV & msg)
{
  return ufil_ros::sensorFovFromMsg(msg);
}

inline ufil::type::OccupancyGrid occlusionGridFromMsg(const nav_msgs::msg::OccupancyGrid & msg)
{
  return ufil_ros::occupancyGridFromMsg(msg);
}

/// Valid tracks with an existence probability of at least min_existence_probability.
inline ufil_msgs::msg::ObjectList tracksToMsg(
  const std::map<ufil::type::Id, Track> & tracks, double min_existence_probability,
  const std_msgs::msg::Header & header)
{
  ufil_msgs::msg::ObjectList msg = ufil_ros::tracksToMsg(tracks, true, min_existence_probability);
  msg.header = header;
  return msg;
}

inline ufil_msgs::msg::ObjectList measurementsToMsg(
  const std::set<Measurement> & measurements, const std_msgs::msg::Header & header)
{
  auto msg = ufil_ros::measurementsToMsg<ufil_msgs::msg::ObjectList>(measurements);
  msg.header = header;
  return msg;
}

inline ufil_msgs::msg::ObjectList associationsToMsg(
  const std::map<ufil::type::Id, Measurement> & associations,
  const std_msgs::msg::Header & header)
{
  ufil_msgs::msg::ObjectList msg;
  msg.header = header;
  for (const auto & [id, measurement] : associations) {
    msg.objects.push_back(ufil_ros::measurementToMsg<ufil_msgs::msg::Object>(measurement));
  }
  return msg;
}

}  // namespace ufil_osn_tracker::conversions

#endif  // UFIL_OSN_TRACKER__COMPONENT__UFIL_OSN_TRACKER_ROS_HPP_
