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

#ifndef PEDESTRIAN__UFIL_EXAMPLES_PEDESTRIAN_TRACKER__COMPONENT__TRACKER_ROS_HPP_
#define PEDESTRIAN__UFIL_EXAMPLES_PEDESTRIAN_TRACKER__COMPONENT__TRACKER_ROS_HPP_

// Conversions between ROS 2 messages and the ROS-independent pedestrian tracker types.

#include <map>
#include <set>

#include <builtin_interfaces/msg/time.hpp>
#include <rclcpp/time.hpp>
#include <std_msgs/msg/header.hpp>
#include <ufil_msgs/msg/object_list.hpp>
#include <ufil_object_tracking/types/time.hpp>
#include <ufil_ros/ufil_ros.hpp>

#include "ufil_examples_pedestrian_tracker/pedestrian_tracker.hpp"

namespace ufil_examples_pedestrian_tracker::conversions
{

inline ufil::type::Timestamp timestampFromMsg(const builtin_interfaces::msg::Time & stamp)
{
  return ufil::from_nanoseconds<ufil::type::Timestamp>(rclcpp::Time(stamp).nanoseconds());
}

inline ufil::type::Timestamp timestampFromRos(const rclcpp::Time & time)
{
  return ufil::from_nanoseconds<ufil::type::Timestamp>(time.nanoseconds());
}

/// Detected objects; entries with a negative variance are unknown.
inline std::set<Detection> detectionsFromMsg(const ufil_msgs::msg::ObjectList & msg)
{
  return ufil_ros::measurementsFromMsg<Detection>(msg);
}

/// The valid tracks with the given header.
inline ufil_msgs::msg::ObjectList tracksToMsg(
  const std::map<ufil::type::Id, Track> & tracks, const std_msgs::msg::Header & header)
{
  ufil_msgs::msg::ObjectList msg = ufil_ros::tracksToMsg(tracks);
  msg.header = header;
  return msg;
}

}  // namespace ufil_examples_pedestrian_tracker::conversions

#endif  // PEDESTRIAN__UFIL_EXAMPLES_PEDESTRIAN_TRACKER__COMPONENT__TRACKER_ROS_HPP_
