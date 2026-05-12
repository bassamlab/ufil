// Copyright 2025 Chair of Embedded Software (Computer Science 11) - RWHT Aachen University
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

#ifndef UFIL_OBJECT_TRACKING__PREDICT__SELECTIVE_PREDICTOR_HPP_
#define UFIL_OBJECT_TRACKING__PREDICT__SELECTIVE_PREDICTOR_HPP_

#include <algorithm>
#include <set>
#include <memory>
#include <utility>
#include <vector>

#include "ufil_object_tracking/predict/state/state_predictor.hpp"
#include "ufil_object_tracking/predict/state/model_state_predictor.hpp"
#include "ufil_object_tracking/types/scalar.hpp"
#include "ufil_object_tracking/types/classification.hpp"

namespace ufil
{
namespace predict
{

/**
 * @class SelectivePredictor
 * @brief Allows prediction only for certain classes
 *
 *
 *
 * @tparam T Type of tracks being predicted.
 */
template<typename T>
class SelectivePredictor : public StatePredictor<T>
{
public:
  using BaseClass = StatePredictor<T>;
  using TrackType = typename StatePredictor<T>::TrackType;
  using TransitionModelType = typename StatePredictor<T>::TransitionModelType;
  using ControlType = typename StatePredictor<T>::ControlType;
  using StateType = typename StatePredictor<T>::StateType;

  using StatePredictor<T>::predict;

private:
  std::set<int> possible_classifications;
  ModelStatePredictor<T> predictor_;

public:
  explicit SelectivePredictor(std::shared_ptr<TransitionModelType> transition_model)
  : predictor_(ModelStatePredictor<T>(transition_model))
  {
  }

  /**
   * Adds a new classification to the predictor.
   */
  void addClassification(int index)
  {
    possible_classifications.insert(index);
  }
  void removeClassification(int index)
  {
    if(possible_classifications.count(index)) {
      possible_classifications.erase(index);
    } else {
      throw std::runtime_error("Element in predictor not found!");
    }
  }

  void predict(
    const TrackType & track,
    const std::optional<ControlType> & control,
    const ufil::type::Timestamp & timestamp,
    StateType & prediction) override
  {
    auto classification = track.currentClassification().classificationVector();
    ufil::type::Scalar maximum = 0.0f;
    int maximum_index = 0;
    for(int i = 0; i < classification.size(); i++) {
      if(classification[i] > maximum) {
        maximum = classification[i];
        maximum_index = i;
      }
    }
    if(possible_classifications.count(maximum_index)) {
      predictor_.predict(track, control, timestamp, prediction);
    }
  }
};


}  // namespace predict
}  // namespace ufil

#endif  // UFIL_OBJECT_TRACKING__PREDICT__SELECTIVE_PREDICTOR_HPP_
