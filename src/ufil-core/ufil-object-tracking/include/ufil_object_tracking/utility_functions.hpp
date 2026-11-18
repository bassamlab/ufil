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

#ifndef UFIL_OBJECT_TRACKING__UTILITY_FUNCTIONS_HPP_
#define UFIL_OBJECT_TRACKING__UTILITY_FUNCTIONS_HPP_

#include <algorithm>
#include <cmath>   // max, min, atan2
#include <limits>  // numeric_limits


#include "ufil_object_tracking/types/scalar.hpp"
#include "ufil_object_tracking/types/matrix.hpp"
#include "ufil_object_tracking/types/measurement.hpp"
#include "ufil_object_tracking/types/state.hpp"
#include "ufil_object_tracking/types/time.hpp"


namespace ufil
{


inline ufil::type::Scalar angular_difference(
  const ufil::type::Scalar & x,
  const ufil::type::Scalar & y)
{
  if (ufil::scalar::is_invalid(x)) {
    throw std::invalid_argument("x is invalid");
  }
  if (ufil::scalar::is_invalid(y)) {
    throw std::invalid_argument("x is invalid");
  }
  return std::atan2(std::sin(x - y), std::cos(x - y));
}

inline ufil::type::Scalar normalize_angle(ufil::type::Scalar angle)
{
  if (ufil::scalar::is_invalid(angle)) {
    throw std::invalid_argument("angle is invalid");
  }
  const ufil::type::Scalar result = std::fmod(angle + std::numbers::pi, 2.0 * std::numbers::pi);
  if (result <= 0.0) {
    return result + std::numbers::pi;
  }
  return result - std::numbers::pi;
}

inline ufil::type::Scalar rad_to_deg(const ufil::type::Scalar & rad)
{
  if (ufil::scalar::is_invalid(rad)) {
    throw std::invalid_argument("rad is invalid");
  }
  return rad * (180.0 / M_PI);
}

inline ufil::type::Scalar deg_to_rad(const ufil::type::Scalar & deg)
{
  if (ufil::scalar::is_invalid(deg)) {
    throw std::invalid_argument("deg is invalid");
  }
  return deg * (M_PI / 180.0);
}

template<typename M, typename S>
ufil::type::Matrix<M::Size, S::Size> generateMeasurmentMatrix()
{
  using MeasurementMatrixType = typename ufil::type::Matrix<M::Size, S::Size>;
  MeasurementMatrixType result = MeasurementMatrixType::Identity();
  return result;
}

template<>
inline ufil::type::Matrix<ufil::type::measurement::Pose2DWithDimension3D::Size,
  ufil::type::state::PoseVelocity2D::Size>
generateMeasurmentMatrix<ufil::type::measurement::Pose2DWithDimension3D,
  ufil::type::state::PoseVelocity2D>()
{
  using MeasurementMatrixType = typename ufil::type::Matrix<
    ufil::type::measurement::Pose2DWithDimension3D::Size,
    ufil::type::state::PoseVelocity2D::Size>;
  MeasurementMatrixType result = MeasurementMatrixType::Zero();
  result(ufil::type::measurement::Pose2DWithDimension3D::X,
      ufil::type::state::PoseVelocity2D::X) = 1.0;
  result(ufil::type::measurement::Pose2DWithDimension3D::Y,
      ufil::type::state::PoseVelocity2D::Y) = 1.0;
  result(ufil::type::measurement::Pose2DWithDimension3D::YAW,
      ufil::type::state::PoseVelocity2D::YAW) = 1.0;
  return result;
}

template<>
inline ufil::type::Matrix<ufil::type::measurement::Pose2DWithDimension3D::Size,
  ufil::type::state::Pose2D::Size>
generateMeasurmentMatrix<ufil::type::measurement::Pose2DWithDimension3D,
  ufil::type::state::Pose2D>()
{
  using MeasurementMatrixType = typename ufil::type::Matrix<
    ufil::type::measurement::Pose2DWithDimension3D::Size,
    ufil::type::state::Pose2D::Size>;
  MeasurementMatrixType result = MeasurementMatrixType::Zero();
  result(ufil::type::measurement::Pose2DWithDimension3D::X, ufil::type::state::Pose2D::X) = 1.0;
  result(ufil::type::measurement::Pose2DWithDimension3D::Y, ufil::type::state::Pose2D::Y) = 1.0;
  result(ufil::type::measurement::Pose2DWithDimension3D::YAW, ufil::type::state::Pose2D::YAW) = 1.0;
  return result;
}

template<typename M, typename S>
ufil::type::Matrix<M::Size, 1> __difference(M measurement, S state)
{
  const auto & state_vector = state.stateVector();
  const auto & measurment_vector = measurement.measurementVector();
  const ufil::type::Matrix<M::Size, S::Size> H = ufil::generateMeasurmentMatrix<M, S>();

  auto diff = measurment_vector - H * state_vector;
  return diff;
}

template<typename M, typename S>
ufil::type::Vector<M::Size> difference(M measurement, S state)
{
  return __difference<M, S>(measurement, state);
}

template<>
inline ufil::type::Vector<ufil::type::measurement::Pose2D::Size>
difference<ufil::type::measurement::Pose2D, ufil::type::state::Pose2D>(
  ufil::type::measurement::Pose2D measurement,
  ufil::type::state::Pose2D state)
{
  const auto & state_vector = state.stateVector();
  const auto & measurment_vector = measurement.measurementVector();
  auto diff = __difference<ufil::type::measurement::Pose2D, ufil::type::state::Pose2D>(measurement,
      state);
  diff(ufil::type::measurement::Pose2D::YAW) = ufil::angular_difference(
      measurment_vector(ufil::type::measurement::Pose2D::YAW),
      state_vector(ufil::type::state::Pose2D::YAW));
  return diff;
}

template<>
inline ufil::type::Vector<ufil::type::measurement::Pose2DWithDimension2D::Size>
difference<ufil::type::measurement::Pose2DWithDimension2D, ufil::type::state::Pose2D>(
  ufil::type::measurement::Pose2DWithDimension2D measurement, ufil::type::state::Pose2D state)
{
  const auto & state_vector = state.stateVector();
  const auto & measurment_vector = measurement.measurementVector();
  auto diff =
    __difference<ufil::type::measurement::Pose2DWithDimension2D,
      ufil::type::state::Pose2D>(measurement, state);
  diff(ufil::type::measurement::Pose2DWithDimension2D::YAW) =
    ufil::angular_difference(measurment_vector(ufil::type::measurement::Pose2DWithDimension2D::YAW),
                               state_vector(ufil::type::state::Pose2D::YAW));
  return diff;
}

template<>
inline ufil::type::Vector<ufil::type::measurement::Pose2DWithDimension3D::Size>
difference<ufil::type::measurement::Pose2DWithDimension3D, ufil::type::state::Pose2D>(
  ufil::type::measurement::Pose2DWithDimension3D measurement, ufil::type::state::Pose2D state)
{
  const auto & state_vector = state.stateVector();
  const auto & measurment_vector = measurement.measurementVector();
  auto diff =
    __difference<ufil::type::measurement::Pose2DWithDimension3D,
      ufil::type::state::Pose2D>(measurement, state);
  diff(ufil::type::measurement::Pose2DWithDimension3D::YAW) =
    ufil::angular_difference(measurment_vector(ufil::type::measurement::Pose2DWithDimension3D::YAW),
                               state_vector(ufil::type::state::Pose2D::YAW));
  return diff;
}

template<>
inline ufil::type::Vector<ufil::type::measurement::Pose2D::Size>
difference<ufil::type::measurement::Pose2D, ufil::type::state::PoseVelocity2D>(
  ufil::type::measurement::Pose2D measurement, ufil::type::state::PoseVelocity2D state)
{
  const auto & state_vector = state.stateVector();
  const auto & measurment_vector = measurement.measurementVector();
  auto diff = __difference<ufil::type::measurement::Pose2D,
      ufil::type::state::PoseVelocity2D>(measurement, state);
  diff(ufil::type::measurement::Pose2D::YAW) = ufil::angular_difference(
      measurment_vector(ufil::type::measurement::Pose2D::YAW),
      state_vector(ufil::type::state::PoseVelocity2D::YAW));
  return diff;
}

template<>
inline ufil::type::Vector<ufil::type::measurement::Pose2DWithDimension2D::Size>
difference<ufil::type::measurement::Pose2DWithDimension2D, ufil::type::state::PoseVelocity2D>(
  ufil::type::measurement::Pose2DWithDimension2D measurement,
  ufil::type::state::PoseVelocity2D state)
{
  const auto & state_vector = state.stateVector();
  const auto & measurment_vector = measurement.measurementVector();
  auto diff = __difference<ufil::type::measurement::Pose2DWithDimension2D,
      ufil::type::state::PoseVelocity2D>(
      measurement, state);
  diff(ufil::type::measurement::Pose2DWithDimension2D::YAW) =
    ufil::angular_difference(measurment_vector(ufil::type::measurement::Pose2DWithDimension2D::YAW),
                               state_vector(ufil::type::state::PoseVelocity2D::YAW));
  return diff;
}

template<>
inline ufil::type::Vector<ufil::type::measurement::Pose2DWithDimension3D::Size>
difference<ufil::type::measurement::Pose2DWithDimension3D, ufil::type::state::PoseVelocity2D>(
  ufil::type::measurement::Pose2DWithDimension3D measurement,
  ufil::type::state::PoseVelocity2D state)
{
  const auto & state_vector = state.stateVector();
  const auto & measurment_vector = measurement.measurementVector();
  auto diff = __difference<ufil::type::measurement::Pose2DWithDimension3D,
      ufil::type::state::PoseVelocity2D>(
      measurement, state);
  diff(ufil::type::measurement::Pose2DWithDimension3D::YAW) =
    ufil::angular_difference(measurment_vector(ufil::type::measurement::Pose2DWithDimension3D::YAW),
                               state_vector(ufil::type::state::PoseVelocity2D::YAW));
  return diff;
}

inline ufil::type::DynamicMatrix matrix_sqrt(const ufil::type::DynamicMatrix & A)
{
  Eigen::SelfAdjointEigenSolver<ufil::type::DynamicMatrix> solver(A);
  if (solver.info() != Eigen::Success) {
    throw std::runtime_error("Eigen decomposition failed");
  }
  ufil::type::DynamicVector sqrt_evals = solver.eigenvalues().cwiseSqrt();
  return solver.eigenvectors() * sqrt_evals.asDiagonal() * solver.eigenvectors().transpose();
}


}  // namespace ufil
#endif  // UFIL_OBJECT_TRACKING__UTILITY_FUNCTIONS_HPP_
