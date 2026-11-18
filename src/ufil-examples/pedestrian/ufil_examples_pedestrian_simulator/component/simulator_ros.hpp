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

#ifndef PEDESTRIAN__UFIL_EXAMPLES_PEDESTRIAN_SIMULATOR__COMPONENT__SIMULATOR_ROS_HPP_
#define PEDESTRIAN__UFIL_EXAMPLES_PEDESTRIAN_SIMULATOR__COMPONENT__SIMULATOR_ROS_HPP_

// Conversions from the ROS-independent simulator types to ROS 2 messages.

#include <cstddef>
#include <cstdint>

#include <std_msgs/msg/header.hpp>
#include <ufil_msgs/msg/classification.hpp>
#include <ufil_msgs/msg/object.hpp>
#include <ufil_msgs/msg/object_list.hpp>

#include "ufil_examples_pedestrian_simulator/pedestrian_simulator.hpp"

namespace ufil_examples_pedestrian_simulator::conversions
{

/// Indices of the diagonal of the 8x8 state covariance (x, y, vx, vy, ax, ay, yaw, yaw rate).
namespace state_index
{
constexpr std::size_t kX = 0;
constexpr std::size_t kY = 9;
constexpr std::size_t kVx = 18;
constexpr std::size_t kVy = 27;
constexpr std::size_t kAx = 36;
constexpr std::size_t kAy = 45;
constexpr std::size_t kYaw = 54;
constexpr std::size_t kYawRate = 63;
}  // namespace state_index

/// Variances reported for a measured object; a negative variance marks an unmeasured entry.
constexpr double kMeasuredPositionVariance = 1.0;
constexpr double kMeasuredYawVariance = 0.1;
constexpr double kMeasuredDimensionVariance = 1.0;
constexpr double kNotMeasured = -1.0;

inline ufil_msgs::msg::Object objectToMsg(std::int64_t id, const AgentState & state)
{
  ufil_msgs::msg::Object object;
  object.id = static_cast<std::uint32_t>(id);
  object.state.state.x = state.position.x();
  object.state.state.y = state.position.y();
  object.state.state.yaw = state.orientation;
  object.dimension.dimension.length = state.dimension.x();
  object.dimension.dimension.width = state.dimension.y();
  object.dimension.dimension.height = state.dimension.z();
  object.classification.classification[ufil_msgs::msg::Classification::PEDESTRIAN] = 1.0;
  return object;
}

/// Noisy measurement of an object: position, yaw and dimension are measured, the rest is not.
inline ufil_msgs::msg::Object measuredObjectToMsg(const SimulatedObject & object)
{
  auto msg = objectToMsg(object.id, object.measured);
  auto & state_covariance = msg.state.covariance;
  state_covariance[state_index::kX] = kMeasuredPositionVariance;
  state_covariance[state_index::kY] = kMeasuredPositionVariance;
  state_covariance[state_index::kYaw] = kMeasuredYawVariance;
  for (const auto index : {state_index::kVx, state_index::kVy, state_index::kAx, state_index::kAy,
      state_index::kYawRate})
  {
    state_covariance[index] = kNotMeasured;
  }
  msg.dimension.covariance[0] = kMeasuredDimensionVariance;
  msg.dimension.covariance[4] = kMeasuredDimensionVariance;
  msg.dimension.covariance[8] = kMeasuredDimensionVariance;
  return msg;
}

/// Ground truth of an object, without covariance.
inline ufil_msgs::msg::Object referenceObjectToMsg(const SimulatedObject & object)
{
  return objectToMsg(object.id, object.ground_truth);
}

/// The detected objects (agents and clutter) of a step.
inline ufil_msgs::msg::ObjectList measuredObjectsToMsg(
  const SimulationStep & step, const std_msgs::msg::Header & header)
{
  ufil_msgs::msg::ObjectList msg;
  msg.header = header;
  for (const auto & object : step.objects) {
    if (object.detected) {
      msg.objects.push_back(measuredObjectToMsg(object));
    }
  }
  return msg;
}

/// The ground truth of all agents in the scene of a step, without clutter.
inline ufil_msgs::msg::ObjectList referenceObjectsToMsg(
  const SimulationStep & step, const std_msgs::msg::Header & header)
{
  ufil_msgs::msg::ObjectList msg;
  msg.header = header;
  for (const auto & object : step.objects) {
    if (object.has_ground_truth) {
      msg.objects.push_back(referenceObjectToMsg(object));
    }
  }
  return msg;
}

}  // namespace ufil_examples_pedestrian_simulator::conversions

#endif  // PEDESTRIAN__UFIL_EXAMPLES_PEDESTRIAN_SIMULATOR__COMPONENT__SIMULATOR_ROS_HPP_
