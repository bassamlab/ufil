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

#ifndef UFIL_OBJECT_TRACKING__TRACKING__MULTI_TARGET_TRACKER_HPP_
#define UFIL_OBJECT_TRACKING__TRACKING__MULTI_TARGET_TRACKER_HPP_

#include <algorithm>
#include <map>
#include <memory>
#include <set>
#include <utility>

#include "ufil_object_tracking/association/associator.hpp"
#include "ufil_object_tracking/detection/detector.hpp"
#include "ufil_object_tracking/management/deleter/deleter.hpp"
#include "ufil_object_tracking/management/initiator/initiator.hpp"
#include "ufil_object_tracking/management/pruner/pruner.hpp"
#include "ufil_object_tracking/predict/predictor.hpp"
#include "ufil_object_tracking/tracking/tracker.hpp"
#include "ufil_object_tracking/types/detection.hpp"
#include "ufil_object_tracking/types/existence_probability.hpp"
#include "ufil_object_tracking/types/id.hpp"
#include "ufil_object_tracking/types/time.hpp"
#include "ufil_object_tracking/update/updater.hpp"
#include "ufil_object_tracking/types/scalar.hpp"

namespace ufil
{
namespace tracking
{

template<typename T, typename D>
class MultiTargetTracker : public Tracker<T, D>
{
public:
  using TrackType = T;
  using DetectionType = D;
  using StateType = T::StateType;
  using MeasurementType = T::MeasurementType;
  using HistoryEntryType = T::HistoryEntryType;

  using UniquePtr = std::unique_ptr<MultiTargetTracker<T, D>>;
  using SharedPtr = std::shared_ptr<MultiTargetTracker<T, D>>;

  std::map<type::Id, MeasurementType> associations_{};
  std::set<MeasurementType> unassociated_measurements_{};

protected:
  type::Scalar existence_threshold_{};

  std::shared_ptr<predict::Predictor<TrackType>> predictor_{};
  std::shared_ptr<detection::Detector<TrackType, DetectionType>> detector_{};
  std::shared_ptr<association::Associator<TrackType>> data_associator_{};
  std::shared_ptr<update::Updater<TrackType>> updater_{};
  std::shared_ptr<management::Initiator<TrackType>> initiator_{};
  std::shared_ptr<management::Deleter<TrackType>> deleter_{};
  std::shared_ptr<management::Pruner<TrackType>> pruner_{};

  std::size_t detector_association_count_{0};
  std::size_t associator_association_count_{0};
  std::size_t total_detector_association_count_{0};
  std::size_t total_associator_association_count_{0};

  void updateTracks(
    std::set<DetectionType> && detections, const type::Timestamp timestamp,
    std::map<type::Id, TrackType> & tracks) override
  {
    // ALIGNMENT
    std::for_each(tracks.begin(), tracks.end(), [this, &timestamp](auto & track_pair) {
        auto & [uuid, track] = track_pair;
      // if (timestamp <= track.lastUpdated())
      //   return;  // This tracker only considers in-order measurements
      // StateType prediction{ timestamp };
        this->predictor_->predict(track, std::nullopt, timestamp);

      // track.insert(HistoryEntryType{ prediction });
    });

    // ASSOCIATION
    std::set<type::Id> unassociated_track_ids{};
    std::map<ufil::type::Id, MeasurementType> unassociated_measurements_detector{};

    // Detect measurements based on detections
    this->associations_.clear();
    this->detector_->convert(tracks, std::move(detections), this->associations_,
          unassociated_track_ids,
                             unassociated_measurements_detector);

    detector_association_count_ = this->associations_.size();

    this->unassociated_measurements_.clear();
    // Associate measurements with tracks
    this->data_associator_->associate(tracks, std::move(unassociated_measurements_detector),
          this->associations_,
                                      unassociated_track_ids, this->unassociated_measurements_);

    associator_association_count_ = this->associations_.size() - detector_association_count_;
    total_detector_association_count_ += detector_association_count_;
    total_associator_association_count_ += associator_association_count_;

    this->updater_->update_parameter(tracks, this->associations_, this->unassociated_measurements_);
    // UPDATE
    std::for_each(tracks.begin(), tracks.end(),
      [this, &timestamp](auto & track_pair) {
        auto & [uuid, track] = track_pair;
        std::optional<MeasurementType> measurement = std::nullopt;
        // Track was matched, update prediction with measurement
        if (this->associations_.contains(uuid)) {
          measurement = this->associations_.at(uuid);
        }
        track.currentHistoryEntry().associatedMeasurement() = measurement;
        this->updater_->update(track, track.currentHistoryEntry().associatedMeasurement(),
            timestamp);
    });

    // MANAGEMENT

    // Initiate new tracks (in-place)
    this->initiator_->initiate(this->unassociated_measurements_, timestamp, tracks);

    // Delete old tracks (in-place)
    this->deleter_->deleteTracks(timestamp, tracks);

    // Prune history of existing tracks (in-place)
    this->pruner_->pruneTrackHistory(timestamp, tracks);
  }

public:
  MultiTargetTracker() = default;

  MultiTargetTracker(
    type::Scalar existence_threshold,
    std::shared_ptr<ufil::predict::Predictor<TrackType>> predictor,
    std::shared_ptr<detection::Detector<TrackType, DetectionType>> detector,
    std::shared_ptr<association::Associator<TrackType>> data_associator,
    std::shared_ptr<update::Updater<TrackType>> updater,
    std::shared_ptr<management::Initiator<TrackType>> initiator,
    std::shared_ptr<management::Deleter<TrackType>> deleter,
    std::shared_ptr<management::Pruner<TrackType>> pruner)
  : existence_threshold_(existence_threshold)
    , predictor_(std::move(predictor))
    , detector_(std::move(detector))
    , data_associator_(std::move(data_associator))
    , updater_(std::move(updater))
    , initiator_(std::move(initiator))
    , deleter_(std::move(deleter))
    , pruner_(std::move(pruner))
  {
  }

  const std::map<type::Id, MeasurementType> & associations()
  {
    return this->associations_;
  }

  const std::set<MeasurementType> & unassociatedMeasurements()
  {
    return this->unassociated_measurements_;
  }

  std::size_t detectorAssociationCount() const
  {
    return detector_association_count_;
  }

  std::size_t associatorAssociationCount() const
  {
    return associator_association_count_;
  }

  std::size_t totalDetectorAssociationCount() const
  {
    return total_detector_association_count_;
  }

  std::size_t totalAssociatorAssociationCount() const
  {
    return total_associator_association_count_;
  }
};
}  // namespace tracking
}  // namespace ufil

#endif  // UFIL_OBJECT_TRACKING__TRACKING__MULTI_TARGET_TRACKER_HPP_
