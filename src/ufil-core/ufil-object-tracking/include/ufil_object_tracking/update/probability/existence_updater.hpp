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

#ifndef UFIL_OBJECT_TRACKING__UPDATE__PROBABILITY__EXISTENCE_UPDATER_HPP_
#define UFIL_OBJECT_TRACKING__UPDATE__PROBABILITY__EXISTENCE_UPDATER_HPP_

#include <memory>
#include <optional>

#include "ufil_object_tracking/models/measurement/linearized_measurement_model.hpp"
#include "ufil_object_tracking/update/updater.hpp"

namespace ufil
{
namespace update
{
namespace probability
{

template<typename T>
class ExistenceUpdater : public Updater<T>
{
protected:
  using TrackType = typename Updater<T>::TrackType;
  using MeasurementType = typename Updater<T>::MeasurementType;
  using ExistenceProbabilityType = T::ExistenceProbabilityType;

public:
  void update(
    TrackType & track, const std::optional<MeasurementType> & measurement,
    const ufil::type::Timestamp & timestamp) override
  {
    this->update(track, measurement, timestamp, track.currentExistenceProbability());
  }

  virtual void update(
    const TrackType & track, const std::optional<MeasurementType> & measurement,
    const ufil::type::Timestamp & timestamp, ExistenceProbabilityType & resulting_existence) = 0;
};
}  // namespace probability
}  // namespace update
}  // namespace ufil

#endif  // UFIL_OBJECT_TRACKING__UPDATE__PROBABILITY__EXISTENCE_UPDATER_HPP_
