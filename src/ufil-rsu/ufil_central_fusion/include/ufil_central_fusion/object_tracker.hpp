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


#ifndef UFIL_CENTRAL_FUSION__OBJECT_TRACKER_HPP_
#define UFIL_CENTRAL_FUSION__OBJECT_TRACKER_HPP_

#include <map>
#include <set>
#include <string>
#include <memory>

#include <ufil_object_tracking/predict/composite_predictor.hpp>

#include "ufil_central_fusion/visibility_control.h"
#include "ufil_central_fusion/definitions.hpp"
#include "ufil_central_fusion/classification_updater.hpp"
#include "ufil_central_fusion/existence_probability_updater.hpp"

namespace ufil
{
namespace tracking
{
template<typename TrackT, typename MeasurementT>
class MultiTargetTracker;
}  // namespace tracking
}  // namespace ufil

namespace ufil_central_fusion
{


class ObjectTracker
{
public:
  using SharedPtr = std::shared_ptr<ObjectTracker>;
  using UniquePtr = std::unique_ptr<ObjectTracker>;

public:
  ~ObjectTracker();

  ObjectTracker(
    ufil::type::Scalar min_existence_weight, ufil::type::Scalar max_existence_weight,
    ufil::type::Scalar decay_factor, ufil::type::Scalar delta_d,
    ufil::type::Scalar alpha, ufil::type::Scalar p_min,
    ufil::type::Scalar p_ref, ufil::type::Scalar p_max,
    ufil::type::Scalar association_threshold);

  const std::map<ufil::type::Id, Track> & tracks() const;

  void update(std::set<DynamicMeasurement> && detections, ufil::type::Timestamp timestamp);

  void predict(const ufil::type::Timestamp timestamp);

  void rollbackHistory(const ufil::type::Timestamp timestamp);

  void setCurrentSensorData(const SensorData & sensor_data);

private:
  std::shared_ptr<ClassificationDempsterShaferUpdater> classification_updater_;
  std::shared_ptr<ProbDempsterShaferUpdater> existence_probability_updater_;

  std::unique_ptr<ufil::tracking::MultiTargetTracker<Track, DynamicMeasurement>> tracker_;

  ufil::type::Scalar min_existence_weight_ = 0.2f;
  ufil::type::Scalar max_existence_weight_ = 0.8f;
  ufil::type::Scalar decay_factor_ = 0.005f;

  ufil::type::Scalar delta_d_ = 0.1f;
  ufil::type::Scalar alpha_ = 0.95f;
  ufil::type::Scalar p_min_ = 0.1f;
  ufil::type::Scalar p_ref_ = 0.5f;
  ufil::type::Scalar p_max_ = 0.9f;

  ufil::type::Scalar association_threshold_ = 4.0f;

  std::shared_ptr<ufil::predict::ParallelPredictor<Track>> predictor_;
};

}  // namespace ufil_central_fusion

#endif  // UFIL_CENTRAL_FUSION__OBJECT_TRACKER_HPP_
