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

#ifndef UFIL_CENTRAL_FUSION__COMPONENT__CENTRAL_FUSION_NODE_ROS_HPP_
#define UFIL_CENTRAL_FUSION__COMPONENT__CENTRAL_FUSION_NODE_ROS_HPP_

#include <cmath>
#include <algorithm>
#include <string>

#include <ufil_central_fusion/definitions.hpp>

namespace ufil_central_fusion_node
{

inline bool string_to_bool(const std::string & str)
{
  std::string lower = str;
  std::transform(lower.begin(), lower.end(), lower.begin(), ::tolower);
  return  lower == "true" || lower == "1";
}

}  // namespace ufil_central_fusion_node

namespace ufil_ros
{
template<>
ufil_msgs::msg::Classification classificationToMsg(
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

template<>
ufil_central_fusion::Measurement measurementFromMsg(const ufil_msgs::msg::Object & object)
{
  ufil_central_fusion::Measurement measurement;
  measurement.classification() =
    ufil_ros::classificationFromMsg<ufil::type::classification::ObjectClassification>(
    object.classification
    );
  measurement.existenceProbability() = object.existence_probability;

  ufil::type::state::PoseVelocity2D ufil_state =
    ufil_ros::stateFromMsg<ufil::type::state::PoseVelocity2D>(object.state);

  std::array<bool, 5> stateIndices{};

  stateIndices.fill(false);
  // X
  if (ufil_state.covariance()(0, 0) >= 0.0f) {
    measurement.x() = ufil_state.x();
  }
  // Y
  if (ufil_state.covariance()(1, 1) >= 0.0f) {
    measurement.y() = ufil_state.y();
  }
  // VX
  if (ufil_state.covariance()(2, 2) >= 0.0f) {
    measurement.vx() = ufil_state.vx();
  }
  // VY
  if (ufil_state.covariance()(3, 3) >= 0.0f) {
    measurement.vy() = ufil_state.vy();
  }
  // YAW
  if (ufil_state.covariance()(4, 4) >= 0.0f) {
    measurement.yaw() = ufil_state.yaw();
  }

  // Prevent yaw rate from beeing used

  // YAW RATE
  // if (ufil_state.covariance()(5, 5) >= 0.0f) {
  //   measurement.yawRate() = ufil_state.yawRate();
  // }
  ufil_state.covariance()(ufil_central_fusion::Measurement::YAW_RATE,
    ufil_central_fusion::Measurement::YAW_RATE) = -1;

  measurement.covariance() = ufil_state.covariance();

  ufil::type::dimension::Dimension3D ufil_dimension =
    ufil_ros::dimensionFromMsg<ufil::type::dimension::Dimension3D>(object.dimension);

  if (ufil_dimension.covariance()(0, 0) >= 0.0f) {
    measurement.dimension().length() = ufil_dimension.length();
  }
  if (ufil_dimension.covariance()(1, 1) >= 0.0f) {
    measurement.dimension().width() = ufil_dimension.width();
  }
  if (ufil_dimension.covariance()(2, 2) >= 0.0f) {
    measurement.dimension().height() = ufil_dimension.height();
  }

  measurement.dimension().covariance() = ufil_dimension.covariance();

  return measurement;
}

}  // namespace ufil_ros

#endif  // UFIL_CENTRAL_FUSION__COMPONENT__CENTRAL_FUSION_NODE_ROS_HPP_
