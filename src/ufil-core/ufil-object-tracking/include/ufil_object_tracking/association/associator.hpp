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

#ifndef UFIL_OBJECT_TRACKING__ASSOCIATION__ASSOCIATOR_HPP_
#define UFIL_OBJECT_TRACKING__ASSOCIATION__ASSOCIATOR_HPP_

#include <map>
#include <memory>
#include <set>
#include <utility>

#include "ufil_object_tracking/association/hypothesizer.hpp"
#include "ufil_object_tracking/types/id.hpp"
#include "ufil_object_tracking/types/matrix.hpp"

namespace ufil
{
namespace association
{

template<class T>
class Associator
{
protected:
  using MeasurementType = T::MeasurementType;
  using StateType = T::StateType;
  using HypothesizerType = Hypothesizer<T>;

  std::shared_ptr<HypothesizerType> hypothesizer_{};

public:
  explicit Associator(std::shared_ptr<HypothesizerType> hypothesizer)
  : hypothesizer_(std::move(hypothesizer))
  {
  }

  virtual ~Associator() = default;

  virtual void associate(
    const std::map<type::Id, T> & tracks, std::map<type::Id, MeasurementType> && measurements,
    std::map<type::Id, MeasurementType> & associations, std::set<type::Id> & unassociated_tracks,
    std::set<MeasurementType> & unassociated_measurements) = 0;
};

template<class T>
class FunctionAssociator : public Associator<T>
{
protected:
  using typename Associator<T>::MeasurementType;
  using typename Associator<T>::StateType;
  using typename Associator<T>::HypothesizerType;

  std::function<void(const type::CostMatrix &, type::AssignmentMatrix &)> assignment_function_;

  type::Scalar threshhold_ = ufil::scalar::ScalarInf;

  type::CostMatrix cost_matrix_;
  type::AssignmentMatrix assignment_matrix_;

public:
  using UniquePtr = std::unique_ptr<FunctionAssociator<T>>;
  using SharedPtr = std::shared_ptr<FunctionAssociator<T>>;

  explicit FunctionAssociator(
    std::shared_ptr<HypothesizerType> hypothesizer,
    std::function<void(const type::CostMatrix &, type::AssignmentMatrix &)> assignment_function,
    type::Scalar threshhold = ufil::scalar::ScalarInf)
  : Associator<T>(std::move(hypothesizer))
    , assignment_function_(std::move(assignment_function))
    , threshhold_(threshhold)
  {
  }

  explicit FunctionAssociator(
    std::function<void(const T &, const MeasurementType &, type::Scalar &)> distance_function,
    std::function<void(const type::CostMatrix &, type::AssignmentMatrix &)> assignment_function,
    type::Scalar threshhold = ufil::scalar::ScalarInf)
  : Associator<T>(std::make_shared<FunctionHypothesizer<T>>(std::move(distance_function)))
    , assignment_function_(std::move(assignment_function))
    , threshhold_(threshhold)
  {
  }

  void associate(
    const std::map<type::Id, T> & tracks, std::map<type::Id, MeasurementType> && measurements,
    std::map<type::Id, MeasurementType> & associations, std::set<type::Id> & unassociated_tracks,
    std::set<MeasurementType> & unassociated_measurements) override
  {
    // Create and resize cost matrix
    this->cost_matrix_ = type::CostMatrix::Zero(tracks.size(), measurements.size());

    // Get cost matrix from hypothesizer
    this->hypothesizer_->hypothesize(tracks, measurements, this->cost_matrix_);

    // Create and initialize assignment matrix
    this->assignment_matrix_ = type::AssignmentMatrix::Zero(tracks.size(), measurements.size());
    this->assignment_function_(this->cost_matrix_, this->assignment_matrix_);

    // Check threshold
    for (int row = 0; row < this->cost_matrix_.rows(); row++) {
      for (int col = 0; col < this->cost_matrix_.cols(); col++) {
        if (this->cost_matrix_(row, col) > this->threshhold_) {
          this->assignment_matrix_(row, col) = 0;
        }
      }
    }

    // Use iterators to keep track of indices
    auto track_it = tracks.begin();
    auto measurement_it = measurements.begin();

    // Temporary sets to keep track of associations
    std::set<type::Id> associated_tracks;
    std::set<type::Id> associated_measurements;

    // Iterate over assignment matrix to populate associations
    for (int track_idx = 0; track_idx < this->assignment_matrix_.rows(); ++track_idx) {
      for (int measurement_idx = 0; measurement_idx < this->assignment_matrix_.cols();
        ++measurement_idx)
      {
        if (this->assignment_matrix_(track_idx, measurement_idx) != 0) {
          // Associate the track with the measurement
          associations.emplace(track_it->first, std::move(measurement_it->second));

          // Record the associations
          associated_tracks.insert(track_it->first);
          associated_measurements.insert(measurement_it->first);
        }
        ++measurement_it;  // Move to next measurement
      }
      ++track_it;                             // Move to next track
      measurement_it = measurements.begin();  // Reset measurement iterator
    }

    // Identify unassociated tracks
    for (const auto & [id, track] : tracks) {
      if (!associated_tracks.contains(id)) {
        unassociated_tracks.insert(id);
      }
    }

    // Identify unassociated measurements
    for (const auto & [id, measurement] : measurements) {
      if (!associated_measurements.contains(id)) {
        unassociated_measurements.insert(std::move(measurement));
      }
    }
  }

  void setAssociationThreshold(const type::Scalar threshold)
  {
    this->threshhold_ = threshold;
  }

  type::CostMatrix & costMatrix()
  {
    return this->cost_matrix_;
  }

  type::AssignmentMatrix & assignmentMatrix()
  {
    return this->assignment_matrix_;
  }
};

}  // namespace association
}  // namespace ufil

#endif  // UFIL_OBJECT_TRACKING__ASSOCIATION__ASSOCIATOR_HPP_
