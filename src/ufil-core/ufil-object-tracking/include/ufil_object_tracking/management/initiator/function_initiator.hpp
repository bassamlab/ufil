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

#ifndef UFIL_OBJECT_TRACKING__MANAGEMENT__INITIATOR__FUNCTION_INITIATOR_HPP_
#define UFIL_OBJECT_TRACKING__MANAGEMENT__INITIATOR__FUNCTION_INITIATOR_HPP_

#include <map>
#include <set>
#include <utility>

#include "ufil_object_tracking/management/initiator/initiator.hpp"
#include "ufil_object_tracking/types/id.hpp"
#include "ufil_object_tracking/types/time.hpp"

namespace ufil
{
namespace management
{

template<typename T>
class FunctionInitiator : public Initiator<T>
{
public:
  using TrackType = T;
  using MeasurementType = T::MeasurementType;
  using StateType = T::StateType;
  using DimensionType = T::DimensionType;
  using ClassificationType = T::ClassificationType;
  using ExistenceProbabilityType = T::ExistenceProbabilityType;

  using HistoryEntryType = T::HistoryEntryType;

protected:
  std::function<void(const MeasurementType &, StateType &, DimensionType &,
    ClassificationType &, ExistenceProbabilityType &)> initiation_function_;

public:
  explicit FunctionInitiator(
    std::function<void(const MeasurementType &, StateType &, DimensionType &,
    ClassificationType &, ExistenceProbabilityType &)> initiation_function)
  : initiation_function_(std::move(initiation_function))
  {
  }

  void initiate(
    const std::set<MeasurementType> & measurements, const type::Timestamp & timestamp,
    std::map<type::Id, TrackType> & new_tracks) override
  {
    for (const auto & measurement : measurements) {
      // Create state
      StateType state(timestamp);
      DimensionType dimension;
      ClassificationType classification;
      ExistenceProbabilityType existence;
      this->initiation_function_(measurement, state, dimension, classification, existence);

      // Create track
      TrackType track;
      track.insert(
          HistoryEntryType(state, dimension, existence, classification, std::nullopt,
            std::make_optional(measurement)));
      new_tracks.emplace(track.uuid(), std::move(track));
    }
  }
};

}  // namespace management
}  // namespace ufil

#endif  // UFIL_OBJECT_TRACKING__MANAGEMENT__INITIATOR__FUNCTION_INITIATOR_HPP_
