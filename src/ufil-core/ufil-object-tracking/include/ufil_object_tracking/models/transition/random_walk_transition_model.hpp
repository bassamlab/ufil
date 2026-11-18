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

#ifndef UFIL_OBJECT_TRACKING__MODELS__TRANSITION__RANDOM_WALK_TRANSITION_MODEL_HPP_
#define UFIL_OBJECT_TRACKING__MODELS__TRANSITION__RANDOM_WALK_TRANSITION_MODEL_HPP_

#include <memory>
#include <optional>

#include "ufil_object_tracking/types/control.hpp"
#include "ufil_object_tracking/types/state.hpp"

#include "ufil_object_tracking/models/transition/linear_kalman_transition_model.hpp"
#include "ufil_object_tracking/types/matrix.hpp"
#include "ufil_object_tracking/types/time.hpp"

namespace ufil
{
namespace model
{
namespace transition
{

/**
 * @brief Random Walk Transition Model
 *
 * This class implements a random walk model as a specific case of the linear Kalman transition model.
 * In a random walk model, the state is assumed to remain the same from one time step to the next,
 * with the addition of process noise. The model formulation is based on [1]. The state transition
 * equation simplifies to:
 *
 *   x' = x + w
 *
 * where:
 *   x  - Previous state vector
 *   x' - Predicted state vector
 *   w  - Process noise (assumed to be zero-mean Gaussian with covariance Q)
 *
 * References:
 *
 * [1] R. Schubert, E. Richter, and G. Wanielik, "Comparison and evaluation of advanced motion
 * models for vehicle tracking," International Conference on Information Fusion, Cologne, Germany, 2008
 *
 * @tparam S State type, must provide Size, StateVectorType, and CovarianceMatrixType
 * @tparam C Control type, must provide Size and ControlVectorType (unused in this model)
 */
template<typename S, typename C>
class RandomWalkTransitionModel : public LinearKalmanTransitionModel<S, C>
{
  // Check that the state and control types are compatible with the random walk model
  static_assert(std::is_base_of<ufil::type::control::None, C>::value,
          "Control type must be None as no control inputs are used in Random Walk model.");
  static_assert(!std::is_base_of<ufil::type::state::NoState, S>::value,
          "State type None is not supported by the Random Walk model as it has no position"
          "components.");

public:
  // Define matrix and vector types based on state and control types
  using StateType = typename LinearKalmanTransitionModel<S, C>::StateType;
  using ControlType = typename LinearKalmanTransitionModel<S, C>::ControlType;
  using NoiseMatrixType = ufil::type::Matrix<2, 2>;
  using NoiseMappingMatrixType = ufil::type::Matrix<StateType::Size, 2>;
  using CovarianceMatrixType = typename LinearKalmanTransitionModel<S, C>::CovarianceMatrixType;
  using TransitionMatrixType = typename LinearKalmanTransitionModel<S, C>::TransitionMatrixType;
  using InputMatrixType = typename LinearKalmanTransitionModel<S, C>::InputMatrixType;

protected:
  /// Noise matrix associated with the control inputs (W)
  NoiseMatrixType W_;
  NoiseMappingMatrixType G_;

public:
  /**
   * @brief Constructor
   *
   * Initializes the random walk model with the provided process noise covariance matrix.
   * The state transition matrix (F) is set to the identity matrix, reflecting that the state
   * remains the same in the absence of noise.
   *
   * @param process_noise_covariance Process noise covariance matrix (Q)
   */
  explicit RandomWalkTransitionModel(const NoiseMatrixType & W = NoiseMatrixType::Identity())
  : LinearKalmanTransitionModel<S, C>(), W_(W), G_(NoiseMappingMatrixType::Zero())
  {
  }

  /**
   * @brief Model initialization method
   *
   * This method sets the state transition matrix (F) to the identity matrix.
   * It can be extended or overridden in derived classes if necessary.
   */
  void onModelInitialization() override
  {
    // Set the state transition matrix F to identity
    this->F_ = TransitionMatrixType::Identity();

    // The control input matrix B as this model does not support control inputs
    this->B_ = InputMatrixType::Zero();
  }

    /**
   * @brief Updates the model matrices at every time step
   *
   * This method updates the state transition matrix (F), control input matrix (B),
   * and process noise covariance matrix (Q) based on the time delta.
   *
   * @param state The current state (unused in this implementation)
   * @param timestamp The current timestamp (unused in this implementation)
   * @param delta_t The time difference between the current and previous time steps
   */
  void onEveryTimestep(
    const StateType & /*state*/, const type::Timestamp & /*timestamp*/,
    const type::Duration & delta_t) override
  {
    // Convert delta_t to seconds
    const double dt = ufil::to_seconds(delta_t);

    // Update the process noice matrix G
    G_ = NoiseMappingMatrixType::Zero();
    G_(StateType::X, 0) = dt;
    G_(StateType::Y, 1) = dt;

    // Update the process noise covariance matrix Q
    this->Q_ = G_ * W_ * G_.transpose();
  }
};
}  // namespace transition
}  // namespace model
}  // namespace ufil

#endif  // UFIL_OBJECT_TRACKING__MODELS__TRANSITION__RANDOM_WALK_TRANSITION_MODEL_HPP_
