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

#ifndef UFIL_OBJECT_TRACKING__PREDICT__STATE__MODEL_STATE_PREDICTOR_HPP_
#define UFIL_OBJECT_TRACKING__PREDICT__STATE__MODEL_STATE_PREDICTOR_HPP_

#include <memory>
#include <optional>
#include <utility>

#include "ufil_object_tracking/models/transition/linearized_transition_model.hpp"
#include "ufil_object_tracking/predict/state/state_predictor.hpp"
#include "ufil_object_tracking/types/time.hpp"

namespace ufil
{
namespace predict
{

/**
 * @class ModelStatePredictor
 * @brief Predicts the future state of a track using a transition model.
 *
 * @tparam T The type of track being predicted.
 */
template<typename T>
class ModelStatePredictor : public StatePredictor<T>
{
public:
  using TrackType = typename StatePredictor<T>::TrackType;
  using ControlType = typename StatePredictor<T>::ControlType;
  using StateType = typename StatePredictor<T>::StateType;
  using TransitionModelType = typename StatePredictor<T>::TransitionModelType;
  using LinearTransitionModelType = model::LinearizedTransitionModel<StateType, ControlType>;

  using StatePredictor<T>::predict;

protected:
  std::shared_ptr<TransitionModelType> transition_model_;

public:
  /**
   * @brief Constructs a ModelStatePredictor with the specified transition model.
   * @param transition_model The transition model used for state prediction.
   */
  explicit ModelStatePredictor(std::shared_ptr<TransitionModelType> transition_model)
  : transition_model_(std::move(transition_model))
  {
    transition_model_->onModelInitialization();
  }

  /**
   * @brief Predicts the future state of a track.
   * @param track The track whose state is to be predicted.
   * @param control Optional control input.
   * @param timestamp The timestamp for the prediction.
   * @param prediction The predicted state output.
   */
  void predict(
    const TrackType & track, const std::optional<ControlType> & control,
    const type::Timestamp & timestamp,
    StateType & prediction) override
  {
    const auto & prior_state = track.previousState();
    const auto delta_t = (timestamp - prior_state.timestamp());
    // const double dt = ufil::to_seconds(delta_t);
    // std::cout << "dt:" << dt << std::endl;
    transition_model_->onEveryTimestep(prior_state, timestamp, delta_t);

    // Perform the state prediction step
    transition_model_->step(prior_state, control, timestamp, prediction);
  }
};

}  // namespace predict
}  // namespace ufil

#endif  // UFIL_OBJECT_TRACKING__PREDICT__STATE__MODEL_STATE_PREDICTOR_HPP_
