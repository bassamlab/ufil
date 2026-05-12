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


#include "orientation_checker.hpp"
#include <iostream>

namespace ufil_central_fusion
{

void invalidate_measurement(Track & track)
{
  auto & current_measurement = track.currentHistoryEntry().associatedMeasurement();

  current_measurement->covariance().col(State::YAW).setZero();
  current_measurement->covariance().row(State::YAW).setZero();
  current_measurement->covariance()(State::YAW, State::YAW) = 100;

  current_measurement->covariance().col(State::YAW_RATE).setZero();
  current_measurement->covariance().row(State::YAW_RATE).setZero();
  current_measurement->covariance()(State::YAW_RATE, State::YAW_RATE) = -1;
}

void perform_normalization(Track & track)
{
  auto & current_state = track.currentState();
  ufil::type::Scalar yaw = current_state.yaw();
  yaw = ufil::normalize_angle(yaw);
  current_state.yaw() = yaw;
}

void perform_rotation(Track & track)
{
  auto & current_state = track.currentState();
  current_state.yaw() = current_state.yaw() + M_PI_2;
  current_state.yawRate() = 0;
  current_state.covariance().col(State::YAW_RATE).setZero();
  current_state.covariance().row(State::YAW_RATE).setZero();
  current_state.covariance()(State::YAW_RATE, State::YAW_RATE) = 100;

  auto & current_dimension = track.currentDimension();
  ufil::type::Vector3 dimension = current_dimension.dimensionVector();
  dimension(0) = current_dimension.dimensionVector()(1);
  dimension(1) = current_dimension.dimensionVector()(0);
  current_dimension.dimensionVector() = dimension;

  auto covariance = current_dimension.covariance();
  covariance(0, 0) = current_dimension.covariance()(1, 1);
  covariance(1, 1) = current_dimension.covariance()(0, 0);
  current_dimension.covariance() = covariance;

  auto buffers = current_dimension.dimensionGridmapBuffer();
  buffers.at(0) = current_dimension.dimensionGridmapBuffer().at(1);
  buffers.at(1) = current_dimension.dimensionGridmapBuffer().at(0);
  current_dimension.dimensionGridmapBuffer() = buffers;
}

void OrientationChecker::update(
  Track & track,
  const std::optional<Measurement> & measurement,
  const ufil::type::Timestamp & /*timestamp*/)
{
  if(!measurement) {
    return;
  }
  State & current_state = track.currentState();

  if (track.currentClassification().car() > 0.8) {
    ufil::type::Scalar yaw = current_state.yaw();
    MeasurementVectorType z = measurement->measurementVector();

    ufil::type::Scalar angular_diff = ufil::angular_difference(z[State::YAW], yaw);
    if(std::abs(angular_diff) > ufil::deg_to_rad(30.0f)) {
      invalidate_measurement(track);
    }
  }
  ufil::type::Scalar yaw = current_state.yaw();
  ufil::type::Vector2 velocity = current_state.velocity();
  ufil::type::Scalar speed = velocity.norm();
  if (speed < 1.0) {
    return;
  }
  ufil::type::Scalar velocity_angle = std::atan2(velocity.y(), velocity.x());
  ufil::type::Scalar angle_diff = ufil::angular_difference(velocity_angle, yaw);
  if (angle_diff < M_PI_4) {
    current_state.yawRate() = angle_diff;
  }
}

}  // namespace ufil_central_fusion
