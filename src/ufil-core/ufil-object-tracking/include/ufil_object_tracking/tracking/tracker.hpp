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

#ifndef UFIL_OBJECT_TRACKING__TRACKING__TRACKER_HPP_
#define UFIL_OBJECT_TRACKING__TRACKING__TRACKER_HPP_

#include <map>
#include <memory>
#include <set>
#include <utility>

#include "ufil_object_tracking/types/id.hpp"
#include "ufil_object_tracking/types/time.hpp"

namespace ufil
{
namespace tracking
{

template<class T, class D>
class Tracker
{
public:
  using TrackType = T;
  using DetectionType = D;

  using StateType = T::StateType;
  using DimensionType = T::DimensionType;
  using ClassificationType = T::ClassificationType;
  using ExistenceProbabilityType = T::ExistenceProbabilityType;

  using MeasurementType = T::MeasurementType;
  using HistoryEntryType = T::HistoryEntryType;

  using UniquePtr = std::unique_ptr<Tracker<TrackType, DetectionType>>;
  using SharedPtr = std::shared_ptr<Tracker<TrackType, DetectionType>>;

public:
  virtual ~Tracker() = default;

  virtual void update(
    std::set<DetectionType> && detections, const type::Timestamp timestamp)
  {
    std::for_each(this->tracks_.begin(), this->tracks_.end(), [&timestamp](auto & track_pair) {
        auto & [uuid, track] = track_pair;

        StateType new_state = track_pair.second.currentState();
        new_state.timestamp() = timestamp;
        DimensionType new_dimension = track_pair.second.currentDimension();
        ClassificationType new_classification = track_pair.second.currentClassification();
        ExistenceProbabilityType new_existence_probability =
        track_pair.second.currentExistenceProbability();

        HistoryEntryType entry(new_state, new_dimension, new_existence_probability,
            new_classification);
        track.insert(std::move(entry));
    });
    this->updateTracks(std::move(detections), timestamp, this->tracks_);
  }

  void rollbackHistory(const type::Timestamp timestamp)
  {
      // First, prune histories of tracks that were created before the timestamp
    for (auto & [id, track] : tracks_) {
      if (timestamp > track.creationTime()) {
        track.pruneAfter(timestamp);
      }
    }

      // Then, remove tracks that were created at/after the timestamp
    std::erase_if(tracks_, [&](const auto & kv) {
        const auto & track = kv.second;
        return timestamp <= track.creationTime();
      });
  }


  [[nodiscard]] const auto & tracks() const
  {
    return this->tracks_;
  }

  [[nodiscard]] auto & tracks()
  {
    return this->tracks_;
  }

protected:
  virtual void updateTracks(
    std::set<DetectionType> && detections, type::Timestamp timestamp,
    std::map<type::Id, TrackType> & tracks) = 0;

private:
  std::map<type::Id, TrackType> tracks_;
};

}  // namespace tracking
}  // namespace ufil

#endif  // UFIL_OBJECT_TRACKING__TRACKING__TRACKER_HPP_
