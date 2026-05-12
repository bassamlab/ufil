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

#include "measurement_model.hpp"

namespace ufil_central_fusion
{

void DynamicMeasurementModel::step(
  const State & prior_state, const std::optional<Measurement> & measurement,
  const ufil::type::Timestamp & /*timestamp*/, State & resulting_state)
{
  if(!measurement) {
    return;
  }

  ufil::type::SquareMatrix<State::Size> H = ufil::type::SquareMatrix<State::Size>::Identity();

  if(!measurement->hasX()) {
    H(Measurement::X, State::X) = 0;
  }
  if(!measurement->hasX()) {
    H(Measurement::Y, State::Y) = 0;
  }
  if(!measurement->hasVx()) {
    H(Measurement::VX, State::VX) = 0;
  }
  if(!measurement->hasVy()) {
    H(Measurement::VY, State::VY) = 0;
  }
  if(!measurement->hasYaw()) {
    H(Measurement::YAW, State::YAW) = 0;
  }
  if(!measurement->hasYawRate()) {
    H(Measurement::YAW_RATE, State::YAW_RATE) = 0;
  }

  StateVectorType x = prior_state.stateVector();
  MeasurementVectorType z = measurement->measurementVector();

  // // if(!measurement->isCam()) {
  //   ufil::type::Scalar angular_diff = ufil::angular_difference(z[State::YAW], x[State::YAW]);
  //   if(std::abs(angular_diff) > ufil::deg_to_rad(75.0f)) {
  //     float sign = std::signbit(angular_diff) ? -1.0f : 1.0f;
  //     int rotations = sign * std::floor(std::abs(angular_diff) / ufil::deg_to_rad(75.0f));
  //     z(Measurement::YAW) -= rotations * ufil::deg_to_rad(90.0f);
  //   }
  // // }


  const StateCovarianceMatrixType P = prior_state.covariance();
  const StateCovarianceMatrixType R = measurement->covariance();

  const ufil::type::Matrix<Measurement::Size, Measurement::Size> innovation =
    H * P * H.transpose() + R;
  const ufil::type::Matrix<State::Size, Measurement::Size> K =
    P * H.transpose() * innovation.inverse();


  MeasurementVectorType diff = z - H * x;
  if(measurement->hasYaw()) {
    diff[State::YAW] = ufil::angular_difference(z[Measurement::YAW], x[State::YAW]);
  }
  MeasurementVectorType delta_x = K * diff;
  // if(measurement->isCam()) {
  //    delta_x[State::YAW] = K(State::YAW, Measurement::YAW) * diff(State::YAW);
  // }
  const StateVectorType corrected_x = x + delta_x;
  const StateCovarianceMatrixType corrected_covariance =
    (State::CovarianceMatrixType::Identity() - K * H) * P;
  resulting_state.stateVector() = corrected_x;
  resulting_state.covariance() = corrected_covariance;
}

}  // namespace ufil_central_fusion
