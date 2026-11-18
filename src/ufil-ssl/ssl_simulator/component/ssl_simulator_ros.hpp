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

#ifndef SSL_SIMULATOR__COMPONENT__SSL_SIMULATOR_ROS_HPP_
#define SSL_SIMULATOR__COMPONENT__SSL_SIMULATOR_ROS_HPP_

// Conversions between ROS 2 messages and the ROS-independent ssl_simulator types.

#include <cmath>
#include <cstdint>
#include <string>
#include <vector>

#include <builtin_interfaces/msg/time.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <rclcpp/time.hpp>
#include <ufil_msgs/msg/object.hpp>
#include <ufil_msgs/msg/object_list.hpp>
#include <ufil_object_tracking/types/time.hpp>
#include <ufil_ros/ufil_ros.hpp>

#include "ssl_simulator/config.hpp"
#include "ssl_simulator/types.hpp"

namespace ssl_simulator::conversions
{

/// Frame of the published pressure grid.
inline constexpr char kMapFrame[] = "map";

inline std::string matFrameId(std::int64_t node_id)
{
  return "ssl_frame_" + std::to_string(node_id);
}

inline std::string mapTopic(std::int64_t node_id)
{
  return "ssl_" + std::to_string(node_id) + "/map";
}

inline Timestamp timestampFromRos(const rclcpp::Time & time)
{
  return ufil::from_nanoseconds<Timestamp>(time.nanoseconds());
}

inline Axle axleFromMsg(const ufil_msgs::msg::Axle & msg)
{
  return {msg.single_track, msg.track_width, msg.center_to_axle};
}

/// v_x and v_y of the state are absolute velocity components in the map frame.
inline Vehicle vehicleFromMsg(const ufil_msgs::msg::Object & msg, const Timestamp & stamp)
{
  Vehicle vehicle;
  vehicle.id = msg.id;
  vehicle.state = ufil_ros::stateFromMsg<VehicleState>(msg.state.state);
  vehicle.state.timestamp() = stamp;
  vehicle.axles.reserve(msg.axles.size());
  for (const auto & axle : msg.axles) {
    vehicle.axles.push_back(axleFromMsg(axle));
  }
  return vehicle;
}

/**
 * @brief Converts the objects of an object list in the map frame.
 * @param received reception time, used as state time if the header has no stamp
 */
inline std::vector<Vehicle> vehiclesFromMsg(
  const ufil_msgs::msg::ObjectList & msg, const Timestamp & received)
{
  const rclcpp::Time stamp(msg.header.stamp, RCL_ROS_TIME);
  const Timestamp state_time = stamp.nanoseconds() == 0 ? received : timestampFromRos(stamp);
  std::vector<Vehicle> vehicles;
  vehicles.reserve(msg.objects.size());
  for (const auto & object : msg.objects) {
    vehicles.push_back(vehicleFromMsg(object, state_time));
  }
  return vehicles;
}

/// Pose of the mat in the map frame (rotation about z by the heading).
inline geometry_msgs::msg::Pose matPoseToMsg(const MatPose & pose)
{
  geometry_msgs::msg::Pose msg;
  msg.position.x = pose.x;
  msg.position.y = pose.y;
  msg.position.z = pose.z;
  msg.orientation.z = std::sin(pose.heading / 2.0);
  msg.orientation.w = std::cos(pose.heading / 2.0);
  return msg;
}

/// Static transform from the map frame to the mat frame ssl_frame_<node_id>.
inline geometry_msgs::msg::TransformStamped matTransformToMsg(
  const MatPose & pose, std::int64_t node_id, const builtin_interfaces::msg::Time & stamp)
{
  const auto pose_msg = matPoseToMsg(pose);
  geometry_msgs::msg::TransformStamped msg;
  msg.header.stamp = stamp;
  msg.header.frame_id = kMapFrame;
  msg.child_frame_id = matFrameId(node_id);
  msg.transform.translation.x = pose_msg.position.x;
  msg.transform.translation.y = pose_msg.position.y;
  msg.transform.translation.z = pose_msg.position.z;
  msg.transform.rotation = pose_msg.orientation;
  return msg;
}

/// Occupancy grid in the map frame whose cells hold the pressure values.
inline nav_msgs::msg::OccupancyGrid pressureMapToMsg(
  const PressureMap & pressure, const SimulatorConfig & config,
  const builtin_interfaces::msg::Time & stamp)
{
  nav_msgs::msg::OccupancyGrid msg;
  msg.header.stamp = stamp;
  msg.header.frame_id = kMapFrame;
  msg.info.map_load_time = stamp;
  msg.info.resolution = static_cast<float>(config.resolution);
  msg.info.width = static_cast<std::uint32_t>(pressure.columns);
  msg.info.height = static_cast<std::uint32_t>(pressure.rows);
  msg.info.origin = matPoseToMsg(config.pose);
  msg.data.assign(pressure.values.begin(), pressure.values.end());
  return msg;
}

}  // namespace ssl_simulator::conversions

#endif  // SSL_SIMULATOR__COMPONENT__SSL_SIMULATOR_ROS_HPP_
