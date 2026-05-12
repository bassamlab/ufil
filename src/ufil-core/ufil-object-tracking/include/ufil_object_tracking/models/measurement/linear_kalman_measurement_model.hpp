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

#ifndef UFIL_OBJECT_TRACKING__MODELS__MEASUREMENT__LINEAR_KALMAN_MEASUREMENT_MODEL_HPP_
#define UFIL_OBJECT_TRACKING__MODELS__MEASUREMENT__LINEAR_KALMAN_MEASUREMENT_MODEL_HPP_

#include <algorithm>
#include <memory>
#include <optional>
#include <limits>

#include "ufil_object_tracking/models/measurement/measurement_model.hpp"
#include "ufil_object_tracking/types/matrix.hpp"
#include "ufil_object_tracking/types/time.hpp"
#include "ufil_object_tracking/utility_functions.hpp"

namespace ufil
{
namespace model
{

template<typename S, typename M>
class LinearKalmanMeasurementModel : public ufil::model::MeasurementModel<S, M>
{
public:
  using MeasurementMatrixType = ufil::type::Matrix<M::Size, S::Size>;
  using StateCovarianceMatrixType = S::CovarianceMatrixType;
  using MeasurementCovarianceMatrixType = M::CovarianceMatrixType;
  using MeasurementVectorType = M::MeasurementVectorType;
  using StateVectorType = S::StateVectorType;

protected:
  MeasurementMatrixType H = MeasurementMatrixType::Zero();

  virtual MeasurementVectorType difference(StateVectorType x, MeasurementVectorType z)
  {
    return z - H * x;
  }

  type::Scalar last_nis_{std::numeric_limits<type::Scalar>::quiet_NaN()};

public:
  LinearKalmanMeasurementModel(
    MeasurementMatrixType input_H = ufil::generateMeasurmentMatrix<M, S>())
  : H(input_H)
  {
  }

  type::Scalar nis() const {return this->last_nis_;}

  void step(
    const S & prior_state, const std::optional<M> & measurement,
    const ufil::type::Timestamp & timestamp,
    S & resulting_state) override
  {
    if (!measurement) {
      return;
    }

    const StateVectorType x = prior_state.stateVector();
    const StateCovarianceMatrixType P = prior_state.covariance();
    const MeasurementCovarianceMatrixType R = measurement->covariance();
    const ufil::type::Matrix<M::Size, M::Size> innovation = H * P * H.transpose() + R;
    const ufil::type::Matrix<S::Size, M::Size> K = P * H.transpose() * innovation.inverse();

    const MeasurementVectorType z = measurement->measurementVector();

    MeasurementVectorType y = difference(x, z);

    this->last_nis_ = (y.transpose() * innovation.inverse() * y)(0, 0);

    StateVectorType corrected_x = x + K * y;
    const StateCovarianceMatrixType corrected_covariance =
      (S::CovarianceMatrixType::Identity() - K * H) * P;

    resulting_state.stateVector() = corrected_x;
    resulting_state.covariance() = corrected_covariance;
  }
};

template<>
inline ufil::type::measurement::Pose2D::MeasurementVectorType
LinearKalmanMeasurementModel<ufil::type::state::Pose2D,
  ufil::type::measurement::Pose2D>::difference(
  ufil::type::state::Pose2D::StateVectorType x,
  ufil::type::measurement::Pose2D::MeasurementVectorType z)
{
  ufil::type::measurement::Pose2D::MeasurementVectorType diff = z - H * x;
  diff[ufil::type::measurement::Pose2D::YAW] =
    ufil::angular_difference(z[ufil::type::measurement::Pose2D::YAW],
        x[ufil::type::state::Pose2D::YAW]);
  return diff;
}

template<>
inline ufil::type::measurement::Pose2DWithDimension2D::MeasurementVectorType
LinearKalmanMeasurementModel<ufil::type::state::Pose2D,
  ufil::type::measurement::Pose2DWithDimension2D>::difference(
  ufil::type::state::Pose2D::StateVectorType x,
  ufil::type::measurement::Pose2DWithDimension2D::MeasurementVectorType z)
{
  ufil::type::measurement::Pose2D::MeasurementVectorType diff = z - H * x;
  diff[ufil::type::measurement::Pose2DWithDimension2D::YAW] = ufil::angular_difference(
      z[ufil::type::measurement::Pose2DWithDimension2D::YAW], x[ufil::type::state::Pose2D::YAW]);
  return diff;
}

template<>
inline ufil::type::measurement::Pose2DWithDimension3D::MeasurementVectorType
LinearKalmanMeasurementModel<ufil::type::state::Pose2D,
  ufil::type::measurement::Pose2DWithDimension3D>::difference(
  ufil::type::state::Pose2D::StateVectorType x,
  ufil::type::measurement::Pose2DWithDimension3D::MeasurementVectorType z)
{
  ufil::type::measurement::Pose2DWithDimension3D::MeasurementVectorType diff = z - H * x;
  diff[ufil::type::measurement::Pose2DWithDimension3D::YAW] = ufil::angular_difference(
      z[ufil::type::measurement::Pose2DWithDimension3D::YAW], x[ufil::type::state::Pose2D::YAW]);
  return diff;
}

template<>
inline ufil::type::measurement::Pose2DWithDimension2D::MeasurementVectorType
LinearKalmanMeasurementModel<ufil::type::state::PoseVelocity2D,
  ufil::type::measurement::Pose2DWithDimension2D>::difference(
  ufil::type::state::PoseVelocity2D::StateVectorType x,
  ufil::type::measurement::Pose2DWithDimension2D::MeasurementVectorType z)
{
  ufil::type::measurement::Pose2DWithDimension2D::MeasurementVectorType diff = z - H * x;
  diff[ufil::type::measurement::Pose2DWithDimension2D::YAW] = ufil::angular_difference(
      z[ufil::type::measurement::Pose2DWithDimension2D::YAW],
        x[ufil::type::state::PoseVelocity2D::YAW]);
  return diff;
}

template<>
inline ufil::type::measurement::Pose2DWithDimension3D::MeasurementVectorType
LinearKalmanMeasurementModel<ufil::type::state::PoseVelocity2D,
  ufil::type::measurement::Pose2DWithDimension3D>::difference(
  ufil::type::state::PoseVelocity2D::StateVectorType x,
  ufil::type::measurement::Pose2DWithDimension3D::MeasurementVectorType z)
{
  ufil::type::measurement::Pose2DWithDimension3D::MeasurementVectorType diff = z - H * x;
  diff[ufil::type::measurement::Pose2DWithDimension3D::YAW] = ufil::angular_difference(
      z[ufil::type::measurement::Pose2DWithDimension3D::YAW],
        x[ufil::type::state::PoseVelocity2D::YAW]);
  return diff;
}

}  // namespace model
}  // namespace ufil

#endif  // UFIL_OBJECT_TRACKING__MODELS__MEASUREMENT__LINEAR_KALMAN_MEASUREMENT_MODEL_HPP_
