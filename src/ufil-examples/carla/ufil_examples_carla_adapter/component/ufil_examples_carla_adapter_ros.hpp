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

#ifndef CARLA__UFIL_EXAMPLES_CARLA_ADAPTER__COMPONENT__UFIL_EXAMPLES_CARLA_ADAPTER_ROS_HPP_
#define CARLA__UFIL_EXAMPLES_CARLA_ADAPTER__COMPONENT__UFIL_EXAMPLES_CARLA_ADAPTER_ROS_HPP_

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

#include <carla_msgs/msg/carla_actor_list.hpp>
#include <derived_object_msgs/msg/object_array.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <std_msgs/msg/header.hpp>
#include <ufil_msgs/msg/classification.hpp>
#include <ufil_msgs/msg/object_list.hpp>

#include "ufil_examples_carla_adapter/carla_adapter.hpp"

/// Conversions between the ROS messages and the types of the core library.
namespace ufil_examples_carla_adapter::conversions
{

/// @return the rotation about the z axis [rad].
inline double yawFromMsg(const geometry_msgs::msg::Quaternion & q)
{
  const double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
  const double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
  return std::atan2(siny_cosp, cosy_cosp);
}

inline std::vector<ActorInfo> actorsFromMsg(const carla_msgs::msg::CarlaActorList & msg)
{
  std::vector<ActorInfo> actors;
  actors.reserve(msg.actors.size());
  for (const auto & actor : msg.actors) {
    actors.push_back(ActorInfo{actor.id, actor.type, actor.rolename});
  }
  return actors;
}

/// Planar state, size and ID of the objects; height, roll and pitch are dropped.
inline std::vector<SimulatorObject> objectsFromMsg(
  const derived_object_msgs::msg::ObjectArray & msg)
{
  std::vector<SimulatorObject> objects;
  objects.reserve(msg.objects.size());
  for (const auto & in : msg.objects) {
    SimulatorObject object;
    object.id = in.id;
    object.state.x = in.pose.position.x;
    object.state.y = in.pose.position.y;
    object.state.v_x = in.twist.linear.x;
    object.state.v_y = in.twist.linear.y;
    object.state.a_x = in.accel.linear.x;
    object.state.a_y = in.accel.linear.y;
    object.state.yaw = yawFromMsg(in.pose.orientation);
    object.state.yaw_rate = in.twist.angular.z;
    // CARLA reports every object as box: [length, width, height].
    const auto & size = in.shape.dimensions;
    object.dimensions.length = size.size() > 0 ? size[0] : 0.0;
    object.dimensions.width = size.size() > 1 ? size[1] : 0.0;
    object.dimensions.height = size.size() > 2 ? size[2] : 0.0;
    objects.push_back(object);
  }
  return objects;
}

/// @return the pose of the transform's child frame in its parent frame, projected onto the plane.
inline Pose2D poseFromMsg(const geometry_msgs::msg::TransformStamped & msg)
{
  return Pose2D{
    msg.transform.translation.x, msg.transform.translation.y,
    yawFromMsg(msg.transform.rotation)};
}

inline std::uint8_t classificationIndex(ObjectClass object_class)
{
  switch (object_class) {
    case ObjectClass::kCar:
      return ufil_msgs::msg::Classification::CAR;
    case ObjectClass::kTruck:
      return ufil_msgs::msg::Classification::TRUCK;
    case ObjectClass::kMotorcycle:
      return ufil_msgs::msg::Classification::MOTORCYCLE;
    case ObjectClass::kOther:
      break;
  }
  return ufil_msgs::msg::Classification::OTHER;
}

inline ufil_msgs::msg::Axle toMsg(const Axle & axle)
{
  ufil_msgs::msg::Axle msg;
  msg.single_track = axle.single_track;
  msg.track_width = static_cast<float>(axle.track_width);
  msg.track_width_variance = static_cast<float>(axle.track_width_variance);
  msg.center_to_axle = static_cast<float>(axle.center_to_axle);
  msg.center_to_axle_variance = static_cast<float>(axle.center_to_axle_variance);
  msg.wheel_detected = axle.wheel_detected;
  return msg;
}

/// Ground-truth object: all reference points are visible, the classification is certain.
inline ufil_msgs::msg::Object toMsg(const Object & object)
{
  ufil_msgs::msg::Object msg;
  msg.id = object.id;

  auto & state = msg.state.state;
  state.x = object.state.x;
  state.y = object.state.y;
  state.v_x = object.state.v_x;
  state.v_y = object.state.v_y;
  state.a_x = object.state.a_x;
  state.a_y = object.state.a_y;
  state.yaw = object.state.yaw;
  state.yaw_rate = object.state.yaw_rate;
  const std::size_t state_size = object.state_variances.size();
  for (std::size_t i = 0; i < state_size; ++i) {
    msg.state.covariance[i * state_size + i] = object.state_variances[i];
  }

  msg.dimension.dimension.length = object.dimensions.length;
  msg.dimension.dimension.width = object.dimensions.width;
  msg.dimension.dimension.height = object.dimensions.height;
  const std::size_t dimension_size = object.dimension_variances.size();
  for (std::size_t i = 0; i < dimension_size; ++i) {
    msg.dimension.covariance[i * dimension_size + i] = object.dimension_variances[i];
  }

  msg.existence_probability = object.existence_probability;
  msg.classification.classification[classificationIndex(object.object_class)] = 1.0F;

  msg.features.fl = true;
  msg.features.fr = true;
  msg.features.bl = true;
  msg.features.br = true;
  msg.features.f = true;
  msg.features.b = true;
  msg.features.l = true;
  msg.features.r = true;
  msg.features.c = true;

  for (const auto & axle : object.axles) {
    msg.axles.push_back(toMsg(axle));
  }
  return msg;
}

inline ufil_msgs::msg::ObjectList toMsg(
  const std::vector<Object> & objects, const std_msgs::msg::Header & header)
{
  ufil_msgs::msg::ObjectList msg;
  msg.header = header;
  msg.objects.reserve(objects.size());
  for (const auto & object : objects) {
    msg.objects.push_back(toMsg(object));
  }
  return msg;
}

}  // namespace ufil_examples_carla_adapter::conversions

#endif  // CARLA__UFIL_EXAMPLES_CARLA_ADAPTER__COMPONENT__UFIL_EXAMPLES_CARLA_ADAPTER_ROS_HPP_
