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

#ifndef UFIL_VISUALIZATION__COMPONENT__UFIL_VISUALIZATION_ROS_HPP_
#define UFIL_VISUALIZATION__COMPONENT__UFIL_VISUALIZATION_ROS_HPP_

// Conversions between ROS 2 messages and the ROS-independent ufil_visualization types.

#include <cstddef>
#include <cstdint>
#include <vector>

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <rclcpp/duration.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <std_msgs/msg/header.hpp>
#include <ufil_msgs/msg/classification.hpp>
#include <ufil_msgs/msg/object.hpp>
#include <ufil_msgs/msg/object_list.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "ufil_visualization/marker.hpp"
#include "ufil_visualization/object_markers.hpp"

namespace ufil_visualization::conversions
{

inline std_msgs::msg::ColorRGBA colorToMsg(const Color & color)
{
  std_msgs::msg::ColorRGBA msg;
  msg.r = color.r;
  msg.g = color.g;
  msg.b = color.b;
  msg.a = color.a;
  return msg;
}

inline geometry_msgs::msg::Point pointToMsg(const Point & point)
{
  geometry_msgs::msg::Point msg;
  msg.x = point.x;
  msg.y = point.y;
  msg.z = point.z;
  return msg;
}

inline geometry_msgs::msg::Quaternion quaternionToMsg(const Quaternion & quaternion)
{
  geometry_msgs::msg::Quaternion msg;
  msg.x = quaternion.x;
  msg.y = quaternion.y;
  msg.z = quaternion.z;
  msg.w = quaternion.w;
  return msg;
}

inline std::int32_t markerTypeToMsg(MarkerType type)
{
  using visualization_msgs::msg::Marker;
  switch (type) {
    case MarkerType::kArrow:
      return Marker::ARROW;
    case MarkerType::kCube:
      return Marker::CUBE;
    case MarkerType::kLineStrip:
      return Marker::LINE_STRIP;
    case MarkerType::kLineList:
      return Marker::LINE_LIST;
    case MarkerType::kTriangleList:
      return Marker::TRIANGLE_LIST;
    case MarkerType::kTextViewFacing:
      break;
  }
  return Marker::TEXT_VIEW_FACING;
}

inline visualization_msgs::msg::Marker markerToMsg(
  const Marker & marker, const std_msgs::msg::Header & header)
{
  visualization_msgs::msg::Marker msg;
  msg.header = header;
  msg.ns = marker.ns;
  msg.id = marker.id;
  msg.type = markerTypeToMsg(marker.type);
  msg.action = visualization_msgs::msg::Marker::ADD;
  msg.pose.position = pointToMsg(marker.position);
  msg.pose.orientation = quaternionToMsg(marker.orientation);
  msg.scale.x = marker.scale.x;
  msg.scale.y = marker.scale.y;
  msg.scale.z = marker.scale.z;
  msg.color = colorToMsg(marker.color);
  msg.lifetime = rclcpp::Duration::from_seconds(marker.lifetime);
  msg.frame_locked = marker.frame_locked;
  msg.points.reserve(marker.points.size());
  for (const auto & point : marker.points) {
    msg.points.push_back(pointToMsg(point));
  }
  msg.text = marker.text;
  return msg;
}

/**
 * @brief Marker array of the markers, all with the given header.
 *
 * With replace_previous, the array starts with a DELETEALL marker so the viewer removes the
 * markers of the previous array before it adds the new ones.
 */
inline visualization_msgs::msg::MarkerArray markersToMsg(
  const std::vector<Marker> & markers, const std_msgs::msg::Header & header,
  bool replace_previous)
{
  visualization_msgs::msg::MarkerArray msg;
  msg.markers.reserve(markers.size() + 1);
  if (replace_previous) {
    visualization_msgs::msg::Marker delete_all;
    delete_all.header = header;
    delete_all.action = visualization_msgs::msg::Marker::DELETEALL;
    msg.markers.push_back(delete_all);
  }
  for (const auto & marker : markers) {
    msg.markers.push_back(markerToMsg(marker, header));
  }
  return msg;
}

inline Object objectFromMsg(const ufil_msgs::msg::Object & msg)
{
  // Row-major 8x8 state covariance in the order (x, y, v_x, v_y, a_x, a_y, yaw, yaw_rate).
  constexpr std::size_t kStateSize = 8;
  constexpr std::size_t kX = 0;
  constexpr std::size_t kY = 1;
  constexpr std::size_t kYaw = 6;
  const auto & covariance = msg.state.covariance;

  Object object;
  object.id = msg.id;
  object.x = msg.state.state.x;
  object.y = msg.state.state.y;
  object.yaw = msg.state.state.yaw;
  object.vx = msg.state.state.v_x;
  object.vy = msg.state.state.v_y;
  object.length = msg.dimension.dimension.length;
  object.width = msg.dimension.dimension.width;
  object.height = msg.dimension.dimension.height;
  object.position_covariance = {
    covariance[kX * kStateSize + kX], covariance[kX * kStateSize + kY],
    covariance[kY * kStateSize + kX], covariance[kY * kStateSize + kY]};
  object.yaw_variance = covariance[kYaw * kStateSize + kYaw];
  object.existence_probability = msg.existence_probability;

  using ufil_msgs::msg::Classification;
  const auto & classes = msg.classification.classification;
  object.class_probabilities[static_cast<std::size_t>(ObjectClass::kCar)] =
    classes[Classification::CAR];
  object.class_probabilities[static_cast<std::size_t>(ObjectClass::kTruck)] =
    classes[Classification::TRUCK];
  object.class_probabilities[static_cast<std::size_t>(ObjectClass::kMotorcycle)] =
    classes[Classification::MOTORCYCLE];
  object.class_probabilities[static_cast<std::size_t>(ObjectClass::kBicycle)] =
    classes[Classification::BICYCLE];
  object.class_probabilities[static_cast<std::size_t>(ObjectClass::kPedestrian)] =
    classes[Classification::PEDESTRIAN];
  object.class_probabilities[static_cast<std::size_t>(ObjectClass::kStationary)] =
    classes[Classification::STATIONARY];
  object.class_probabilities[static_cast<std::size_t>(ObjectClass::kOther)] =
    classes[Classification::OTHER];
  return object;
}

inline std::vector<Object> objectsFromMsg(const ufil_msgs::msg::ObjectList & msg)
{
  std::vector<Object> objects;
  objects.reserve(msg.objects.size());
  for (const auto & object : msg.objects) {
    objects.push_back(objectFromMsg(object));
  }
  return objects;
}

}  // namespace ufil_visualization::conversions

#endif  // UFIL_VISUALIZATION__COMPONENT__UFIL_VISUALIZATION_ROS_HPP_
