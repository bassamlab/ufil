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

#ifndef UFIL_OBJECT_TRACKING__PREDICT__PREDICTOR_HPP_
#define UFIL_OBJECT_TRACKING__PREDICT__PREDICTOR_HPP_

#include <optional>

#include "ufil_object_tracking/models/transition/transition_model.hpp"
#include "ufil_object_tracking/types/history_entry.hpp"
#include "ufil_object_tracking/types/time.hpp"

namespace ufil
{
namespace predict
{

template<typename T>
class Predictor
{
public:
  using TrackType = T;
  using ControlType = typename TrackType::ControlType;
  using StateType = typename TrackType::StateType;
  using DimensionType = typename TrackType::DimensionType;
  using ClassificationType = typename TrackType::ClassificationType;
  using ExistenceProbabilityType = typename TrackType::ExistenceProbabilityType;
  using MeasurementType = typename TrackType::MeasurementType;
  using HistoryEntryType = typename TrackType::HistoryEntryType;
  using TransitionModelType = model::TransitionModel<StateType, ControlType>;

  virtual ~Predictor() = default;

  virtual void predict(
    TrackType & track, const std::optional<ControlType> & control,
    const ufil::type::Timestamp & timestamp) = 0;

protected:
  void storeControlInput(
    TrackType & track, const std::optional<ControlType> & control) const
  {
    track.currentHistoryEntry().controlInput() = control;
  }
};

}  // namespace predict
}  // namespace ufil

#endif  // UFIL_OBJECT_TRACKING__PREDICT__PREDICTOR_HPP_
