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

#ifndef UFIL_OBJECT_TRACKING__ASSOCIATION__HYPOTHESIZE_FUNCTIONS_HPP_
#define UFIL_OBJECT_TRACKING__ASSOCIATION__HYPOTHESIZE_FUNCTIONS_HPP_

#include <vector>

#include "ufil_object_tracking/types/matrix.hpp"
#include "ufil_object_tracking/types/state.hpp"
#include "ufil_object_tracking/types/measurement.hpp"
#include "ufil_object_tracking/types/scalar.hpp"
#include "ufil_object_tracking/utility_functions.hpp"
#include "ufil_object_tracking/association/impl/intersection_over_union_functions.hpp"

namespace bg = boost::geometry;


namespace ufil
{
namespace association
{
template<typename TrackType>
void euclidean(
  const TrackType & track, const typename TrackType::MeasurementType & measurement,
  ufil::type::Scalar & distance)
{
  const auto & state = track.currentState();
  const auto position_detection = measurement.position();
  const auto position_object = state.position();
  distance = (position_detection - position_object).norm();
}


template<typename TrackType>
void intersection_over_union_2D(
  const TrackType & track, const typename TrackType::MeasurementType & measurement,
  ufil::type::Scalar & distance)
{
  const ufil::type::Scalar x_detection = measurement.x();
  const ufil::type::Scalar y_detection = measurement.y();
  const ufil::type::Scalar length_detection = measurement.dimension().length();
  const ufil::type::Scalar width_detection = measurement.dimension().width();
  if(length_detection <= 0.0f || width_detection <= 0.0f) {
    throw std::runtime_error("IoU: Dimension can't be <= 0");
  }
  const ufil::type::Scalar yaw_detection = measurement.yaw();


  bg::model::polygon<bg::model::d2::point_xy<ufil::type::Scalar>> detection_polygon =
    create_rotated_rectangle(x_detection, y_detection, length_detection,
                                           width_detection, yaw_detection);

  const auto & state = track.currentState();
  const auto & dimension = track.currentDimension();
  const ufil::type::Scalar x_track = state.x();
  const ufil::type::Scalar y_track = state.y();
  const ufil::type::Scalar length_track = dimension.length();
  const ufil::type::Scalar width_track = dimension.width();
  if(length_track <= 0.0f || width_track <= 0.0f) {
    throw std::runtime_error("IoU: Dimension can't be <= 0");
  }
  const ufil::type::Scalar yaw_track = state.yaw();
  bg::model::polygon<bg::model::d2::point_xy<ufil::type::Scalar>> track_polygon =
    create_rotated_rectangle(x_track, y_track, length_track,
                                           width_track, yaw_track);
  std::vector<bg::model::polygon<bg::model::d2::point_xy<ufil::type::Scalar>>> buffer;
  bg::intersection(detection_polygon, track_polygon, buffer);

  if(buffer.size() > 1) {
    throw std::runtime_error("IoU: intersection with more than one polygon is not possible");
  }
  ufil::type::Scalar intersection_area = 0.0f;
  if(!buffer.empty()) {
    intersection_area = bg::area(buffer[0]);
  }


  buffer.clear();
  bg::union_(detection_polygon, track_polygon, buffer);

  ufil::type::Scalar union_area = 0.0f;
  for(const auto & p : buffer) {
    union_area += bg::area(p);
  }

  if(union_area > 0.0f) {
    distance = (1.0f - (intersection_area / union_area)) * 100;  // intersectionOU 1 great, 0 bad
  } else {
    throw std::runtime_error("IoU calculation failed: Union can't be 0.0");
  }
}


template<typename TrackType>
void mahalanobis(
  const TrackType & track, const typename TrackType::MeasurementType & measurement,
  ufil::type::Scalar & distance)
{
  const auto & state = track.currentState();
  const auto H = ufil::generateMeasurmentMatrix<typename TrackType::MeasurementType,
      typename TrackType::StateType>();

  // Compute the average covariance
  const ufil::type::SquareMatrix<TrackType::MeasurementType::Size> covariance =
    0.5 * (H * state.covariance() * H.transpose() + measurement.covariance());

  // Check if the covariance matrix is invertible using a small threshold
  if (covariance.determinant() < 1e-6) {
    distance = ufil::scalar::ScalarMax;  // use a max distance value to indicate a problem
    return;
  }

  // Compute the difference vector
  const auto difference =
    ufil::difference<typename TrackType::MeasurementType,
      typename TrackType::StateType>(measurement, state);

  // Compute Mahalanobis distance
  // Invert covariance using Eigenvalue decomposition for better stability
  Eigen::SelfAdjointEigenSolver<ufil::type::DynamicMatrix> solver(covariance);
  if (solver.info() != Eigen::Success) {
    // If decomposition fails, return a max distance
    distance = ufil::scalar::ScalarMax;
    return;
  }

  // Calculate the Mahalanobis distance: sqrt(difference^T * inv(covariance) * difference)
  const ufil::type::DynamicMatrix inv_covariance =
    solver.eigenvectors() * solver.eigenvalues().cwiseInverse().asDiagonal() *
    solver.eigenvectors().transpose();
  distance = std::sqrt(difference.transpose() * inv_covariance * difference);
}

template<typename TrackType>
void wasserstein(
  const TrackType & track, const typename TrackType::MeasurementType & measurement,
  ufil::type::Scalar & distance)
{
  const typename TrackType::StateType & state = track.currentState();

  // Compute the difference term
  // The difference term is the squared norm of the difference between the measurement and state.
  // The helper function ufil::difference takes care of periodic variables.
  const ufil::type::Vector<TrackType::MeasurementType::Size> difference =
    ufil::difference<typename TrackType::MeasurementType,
      typename TrackType::StateType>(measurement, state);
  ufil::type::Scalar diff_term = difference.squaredNorm();

  const auto H = ufil::generateMeasurmentMatrix<typename TrackType::MeasurementType,
      typename TrackType::StateType>();

  // Compute the covariance matrices
  const ufil::type::SquareMatrix<TrackType::MeasurementType::Size> C1 = H * state.covariance() *
    H.transpose();
  const ufil::type::SquareMatrix<TrackType::MeasurementType::Size> C2 = measurement.covariance();

  // Calculate square roots
  const ufil::type::DynamicMatrix sqrt_C1 = ufil::matrix_sqrt(C1);
  const ufil::type::SquareMatrix<TrackType::MeasurementType::Size> M = sqrt_C1 * C2 * sqrt_C1;
  const ufil::type::DynamicMatrix sqrt_M = ufil::matrix_sqrt(M);

  // Compute the trace term
  const ufil::type::Scalar trace_term = (C1 + C2 - 2.0f * sqrt_M).trace();

  // Final computation of the distance
  distance = std::sqrt(trace_term + diff_term);
}

}  // namespace association
}  // namespace ufil
#endif  // UFIL_OBJECT_TRACKING__ASSOCIATION__HYPOTHESIZE_FUNCTIONS_HPP_
