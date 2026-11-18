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

#ifndef UFIL_OBJECT_TRACKING__UPDATE__STATE__MODEL_STATE_UPDATER_HPP_
#define UFIL_OBJECT_TRACKING__UPDATE__STATE__MODEL_STATE_UPDATER_HPP_

#include <memory>
#include <optional>
#include <utility>

#include "ufil_object_tracking/models/measurement/linearized_measurement_model.hpp"
#include "ufil_object_tracking/update/state/state_updater.hpp"

namespace ufil
{
namespace update
{
namespace state
{

/**
 * @class ModelStateUpdater
 * @brief Updates the state of a track using a measurement model.
 *
 * @tparam T The type of track being updated.
 */
template<typename T>
class ModelStateUpdater : public StateUpdater<T>
{
protected:
  using TrackType = typename StateUpdater<T>::TrackType;
  using MeasurementType = typename StateUpdater<T>::MeasurementType;
  using StateType = typename StateUpdater<T>::StateType;
  using MeasurementModelType = ufil::model::MeasurementModel<StateType, MeasurementType>;
  using LinearMeasurementModelType = ufil::model::LinearizedMeasurementModel<StateType,
      MeasurementType>;

public:
  /**
   * @brief Constructs a ModelStateUpdater with the specified measurement model.
   *
   * @param measurement_model The measurement model used for state updates.
   */
  explicit ModelStateUpdater(std::shared_ptr<MeasurementModelType> measurement_model)
  : measurement_model_(std::move(measurement_model))
  {
  }

  /**
   * @brief Updates the state of a track based on measurements.
   *
   * This method uses the provided measurement to update the track's current state,
   * optionally linearizing the measurement model if applicable.
   *
   * @param track The track whose state is to be updated.
   * @param measurement Optional new measurement input for updating the state.
   * @param timestamp The timestamp at which to perform the update.
   * @param resulting_state The output parameter where the updated state will be stored.
   */
  void update(
    const T & track, const std::optional<MeasurementType> & measurement,
    const type::Timestamp & timestamp,
    StateType & resulting_state) override
  {
    // Get current state from track history
    const auto & current_state = track.currentState();

    // Attempt to linearize the measurement model if applicable
    if (auto linearized_measurement_model =
      std::dynamic_pointer_cast<LinearMeasurementModelType>(measurement_model_))
    {
      linearized_measurement_model->linearize(current_state);
    }

    // Perform the state update step
    measurement_model_->step(current_state, measurement, timestamp, resulting_state);

    if constexpr (requires(const MeasurementModelType & m) {m.nis();}) {
      if (
        auto linearized_measurement_model =
        std::dynamic_pointer_cast<LinearMeasurementModelType>(measurement_model_))
      {
        type::Scalar nis = linearized_measurement_model->nis();

        if (std::isfinite(nis)) {
          auto & hist_entry = track.currentHistoryEntry();

          hist_entry.nis() = nis;
        }
      }
    }
  }

private:
  std::shared_ptr<MeasurementModelType> measurement_model_;
};

}  // namespace state
}  // namespace update
}  // namespace ufil

#endif  // UFIL_OBJECT_TRACKING__UPDATE__STATE__MODEL_STATE_UPDATER_HPP_
