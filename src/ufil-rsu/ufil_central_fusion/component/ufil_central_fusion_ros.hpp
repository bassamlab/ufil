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

#ifndef UFIL_CENTRAL_FUSION__COMPONENT__UFIL_CENTRAL_FUSION_ROS_HPP_
#define UFIL_CENTRAL_FUSION__COMPONENT__UFIL_CENTRAL_FUSION_ROS_HPP_

// Conversions between ROS 2 messages and the ROS-independent ufil_central_fusion types.

#include <map>
#include <set>

#include <builtin_interfaces/msg/time.hpp>
#include <rclcpp/time.hpp>
#include <std_msgs/msg/header.hpp>
#include <ufil_msgs/msg/classification.hpp>
#include <ufil_msgs/msg/object.hpp>
#include <ufil_msgs/msg/object_list.hpp>
#include <ufil_msgs/msg/sensor_fov.hpp>
#include <ufil_object_tracking/types/time.hpp>
#include <ufil_ros/ufil_ros.hpp>

#include "ufil_central_fusion/definitions.hpp"

namespace ufil_ros
{

/// Used by ufil_ros::tracksToMsg for the tracks of the central fusion.
template<>
inline ufil_msgs::msg::Classification classificationToMsg(
  const ufil_central_fusion::Classification & value)
{
  ufil_msgs::msg::Classification result;
  result.classification[ufil_msgs::msg::Classification::CAR] = value.car();
  result.classification[ufil_msgs::msg::Classification::TRUCK] = value.truck();
  result.classification[ufil_msgs::msg::Classification::MOTORCYCLE] = value.motorcycle();
  result.classification[ufil_msgs::msg::Classification::BICYCLE] = value.bicycle();
  result.classification[ufil_msgs::msg::Classification::PEDESTRIAN] = value.pedestrian();
  result.classification[ufil_msgs::msg::Classification::STATIONARY] = value.stationary();
  result.classification[ufil_msgs::msg::Classification::OTHER] = value.other();
  return result;
}

}  // namespace ufil_ros

namespace ufil_central_fusion::conversions
{

inline ufil::type::Timestamp timestampFromMsg(const builtin_interfaces::msg::Time & stamp)
{
  return ufil::from_nanoseconds<ufil::type::Timestamp>(rclcpp::Time(stamp).nanoseconds());
}

inline ufil::type::Timestamp timestampFromRos(const rclcpp::Time & time)
{
  return ufil::from_nanoseconds<ufil::type::Timestamp>(time.nanoseconds());
}

/**
 * @brief Converts an object into a measurement.
 *
 * State and dimension entries with a negative variance are unknown and are not assigned. The
 * yaw rate is never used.
 */
inline Measurement measurementFromMsg(const ufil_msgs::msg::Object & object)
{
  Measurement measurement;
  measurement.classification() =
    ufil_ros::classificationFromMsg<ufil::type::classification::ObjectClassification>(
    object.classification);
  measurement.existenceProbability() = object.existence_probability;

  auto state = ufil_ros::stateFromMsg<ufil::type::state::PoseVelocity2D>(object.state);
  const auto & cov = state.covariance();
  if (cov(Measurement::X, Measurement::X) >= 0.0f) {
    measurement.x() = state.x();
  }
  if (cov(Measurement::Y, Measurement::Y) >= 0.0f) {
    measurement.y() = state.y();
  }
  if (cov(Measurement::VX, Measurement::VX) >= 0.0f) {
    measurement.vx() = state.vx();
  }
  if (cov(Measurement::VY, Measurement::VY) >= 0.0f) {
    measurement.vy() = state.vy();
  }
  if (cov(Measurement::YAW, Measurement::YAW) >= 0.0f) {
    measurement.yaw() = state.yaw();
  }
  state.covariance()(Measurement::YAW_RATE, Measurement::YAW_RATE) = -1;
  measurement.covariance() = state.covariance();

  auto dimension =
    ufil_ros::dimensionFromMsg<ufil::type::dimension::Dimension3D>(object.dimension);
  if (dimension.covariance()(0, 0) >= 0.0f) {
    measurement.dimension().length() = dimension.length();
  }
  if (dimension.covariance()(1, 1) >= 0.0f) {
    measurement.dimension().width() = dimension.width();
  }
  if (dimension.covariance()(2, 2) >= 0.0f) {
    measurement.dimension().height() = dimension.height();
  }
  measurement.dimension().covariance() = dimension.covariance();
  return measurement;
}

inline std::set<Measurement> measurementsFromMsg(const ufil_msgs::msg::ObjectList & msg)
{
  std::set<Measurement> measurements;
  for (const auto & object : msg.objects) {
    measurements.insert(measurementFromMsg(object));
  }
  return measurements;
}

inline ufil::type::SensorFOV fovFromMsg(const ufil_msgs::msg::SensorFOV & msg)
{
  return ufil_ros::sensorFovFromMsg(msg);
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

}  // namespace ufil_central_fusion::conversions

#endif  // UFIL_CENTRAL_FUSION__COMPONENT__UFIL_CENTRAL_FUSION_ROS_HPP_
