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

#ifndef UFIL_OBJECT_TRACKING__DETECTION__SIMPLE_DETECTOR_HPP_
#define UFIL_OBJECT_TRACKING__DETECTION__SIMPLE_DETECTOR_HPP_

#include <map>
#include <set>

#include "ufil_object_tracking/detection/detector.hpp"
#include "ufil_object_tracking/types/id.hpp"

namespace ufil
{
namespace detection
{

template<typename T, typename D>
class SimpleDetector : public Detector<T, D>
{
public:
  using TrackType = T;
  using DetectionType = D;
  using StateType = T::StateType;
  using MeasurementType = T::MeasurementType;
  using HistoryEntryType = T::HistoryEntryType;

public:
  void convert(
    const std::map<type::Id, T> & tracks, std::set<DetectionType> && detections,
    [[maybe_unused]] std::map<type::Id, MeasurementType> & associations,
    std::set<type::Id> & unassociated_tracks,
    std::map<type::Id, MeasurementType> & unassociated_measurements) override
  {
    // No associations
    for (const auto & [track_uuid, track] : tracks) {
      unassociated_tracks.insert(track_uuid);
    }
    // Move other
    for (const auto & detection : detections) {
      unassociated_measurements.insert({detection.uuid(), detection});
    }
  }
};

}  // namespace detection
}  // namespace ufil

#endif  // UFIL_OBJECT_TRACKING__DETECTION__SIMPLE_DETECTOR_HPP_
