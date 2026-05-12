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

#ifndef UFIL_OBJECT_TRACKING__UPDATE__COMPOSITE_UPDATER_HPP_
#define UFIL_OBJECT_TRACKING__UPDATE__COMPOSITE_UPDATER_HPP_

#include <algorithm>
#include <execution>
#include <memory>
#include <utility>
#include <vector>
#include <map>
#include <set>

#include "ufil_object_tracking/types/measurement.hpp"
#include "ufil_object_tracking/update/updater.hpp"


namespace ufil
{
namespace update
{

template<typename ExecutionPolicy>
concept ExecutionPolicyConcept = requires {std::is_execution_policy_v<ExecutionPolicy>;};

template<ExecutionPolicyConcept ExecutionPolicy, typename T>
class CompositeUpdater : public Updater<T>
{
public:
  using MeasurementType = ufil::update::Updater<T>::MeasurementType;

private:
  std::vector<std::shared_ptr<Updater<T>>> sub_updaters_{};

public:
  CompositeUpdater() = default;

  virtual ~CompositeUpdater() = default;

  void addUpdater(std::shared_ptr<Updater<T>> updater)
  {
    this->sub_updaters_.push_back(std::move(updater));
  }

  void update_parameter(
    const std::map<type::Id, T> & tracks, const std::map<type::Id, MeasurementType> & associations,
    const std::set<MeasurementType> & unassociated_measurements) override
  {
    update::ExecutionPolicyConcept auto policy = ExecutionPolicy{};
    // Using the execution policy to update each Updater in the vector
    std::for_each(
      policy, sub_updaters_.begin(), sub_updaters_.end(),
      [&tracks, &associations,
      &unassociated_measurements](const std::shared_ptr<Updater<T>> & updater) {
        updater->update_parameter(tracks, associations, unassociated_measurements);
      });
  }

  void update(
    T & track, const std::optional<MeasurementType> & measurement,
    const type::Timestamp & timestamp) override
  {
    update::ExecutionPolicyConcept auto policy = ExecutionPolicy{};
    // Using the execution policy to update each Updater in the vector
    std::for_each(policy, sub_updaters_.begin(), sub_updaters_.end(),
      [&track, &measurement, &timestamp](const std::shared_ptr<Updater<T>> & updater) {
        updater->update(track, measurement, timestamp);
                  });
  }
};

template<class T>
using ParallelUpdater = CompositeUpdater<std::execution::parallel_policy, T>;

template<class T>
using ParallelUnsequencedUpdater = CompositeUpdater<std::execution::parallel_unsequenced_policy, T>;

template<class T>
using SequentialUpdater = CompositeUpdater<std::execution::sequenced_policy, T>;

template<class T>
using UnsequencedUpdater = CompositeUpdater<std::execution::unsequenced_policy, T>;

}  // namespace update
}  // namespace ufil

#endif  // UFIL_OBJECT_TRACKING__UPDATE__COMPOSITE_UPDATER_HPP_
