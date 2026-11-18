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

#ifndef UFIL_MQTT__COMPONENT__UFIL_MQTT_ROS_HPP_
#define UFIL_MQTT__COMPONENT__UFIL_MQTT_ROS_HPP_

// Conversions from ROS 2 messages to the MQTT payload of the ufil_mqtt core library.

#include <string>

#include <nav_msgs/msg/occupancy_grid.hpp>
#include <ufil_msgs/msg/object_list.hpp>
#include <ufil_ros/json_ros.hpp>

namespace ufil_mqtt::conversions
{

/// JSON encoded object list, see ufil_ros::to_json.
inline std::string payloadFromMsg(const ufil_msgs::msg::ObjectList & msg)
{
  return ufil_ros::to_json(msg);
}

/// JSON encoded occupancy grid, see ufil_ros::to_json.
inline std::string payloadFromMsg(const nav_msgs::msg::OccupancyGrid & msg)
{
  return ufil_ros::to_json(msg);
}

}  // namespace ufil_mqtt::conversions

#endif  // UFIL_MQTT__COMPONENT__UFIL_MQTT_ROS_HPP_
