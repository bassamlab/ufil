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

#ifndef UFIL_OBJECT_TRACKING__MODELS__TRANSITION__EXTENDED_CV_TRANSITION_MODEL_HPP_
#define UFIL_OBJECT_TRACKING__MODELS__TRANSITION__EXTENDED_CV_TRANSITION_MODEL_HPP_

#include <cmath>
#include <memory>
#include <optional>
#include <stdexcept>

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
 * @brief Extended Constant Velocity Transition Model with Yaw Rate
 *
 * This class extends the constant velocity model to include yaw and yaw rate.
 * It assumes that an object moves with a constant velocity in Cartesian coordinates,
 * while its yaw angle is updated using a constant yaw rate. The model formulation is based on [1].
 *
 * The state transition equation is given by:
 *
 *   x' = F * x + B * u + G * w
 *
 * where:
 *   x  - Previous state vector
 *   x' - Predicted state vector
 *   F  - State transition matrix (depends on time delta)
 *   B  - Control input matrix (depends on time delta)
 *   u  - Control vector (e.g., acceleration)
 *   G  - Noise mapping matrix
 *   w  - Process noise (zero-mean Gaussian with covariance Q)
 *
 * The process noise covariance Q is computed as:
 *
 *   Q = G * W * G^T
 *
 * where W is the noise covariance associated with acceleration and yaw rate noise.
 *
 * References:
 *
 * [1] R. Schubert, E. Richter, and G. Wanielik, "Comparison and evaluation of advanced motion
 * models for vehicle tracking," International Conference on Information Fusion, Cologne, Germany, 2008
 *
 * @tparam S State type, must provide StateVectorType, CovarianceMatrixType, and static indices for state components
 * @tparam C Control type, must provide ControlVectorType and static indices for control components
 */
template<typename S, typename C>
class ExtendedConstantVelocityTransitionModel : public LinearKalmanTransitionModel<S, C>
{
  static_assert(std::is_base_of<ufil::type::control::None, C>::value,
          "Control type must be None as no control inputs are used in Extended Constant Velocity"
          "model.");
  static_assert(!std::is_base_of<ufil::type::state::NoState, S>::value,
          "State type None is not supported by the Extended Constant Velocity model as it has no"
          "position components.");
  static_assert(!std::is_base_of<ufil::type::state::Position2D, S>::value,
          "State type Position2D is not supported by the Extended Constant Velocity model as it"
          "has no velocity components.");
  static_assert(!std::is_base_of<ufil::type::state::Pose2D, S>::value,
          "State type Pose2D is not supported by the Extended Constant Velocity model as it has"
          "no velocity components.");
  static_assert(!std::is_base_of<ufil::type::state::PositionVelocity2D, S>::value,
          "State type PositionVelocity2D is not supported by the Extended Constant Velocity model"
          "as it has no orientation components.");
  static_assert(!std::is_base_of<ufil::type::state::PositionVelocityAcceleration2D, S>::value,
          "State type PositionVelocityAcceleration2D is not supported by the Extended Constant"
          "Velocity model as it has no orientation components.");

public:
  using StateType = typename TransitionModel<S, C>::StateType;
  using ControlType = typename TransitionModel<S, C>::ControlType;
  // Process noise covariance for (ax, ay, yaw rate noise)
  using NoiseMatrixType = ufil::type::Matrix<3, 3>;
  using NoiseMappingMatrixType = ufil::type::Matrix<StateType::Size, 3>;  // Maps noise to state

  using CovarianceMatrixType = typename LinearKalmanTransitionModel<S, C>::CovarianceMatrixType;
  using TransitionMatrixType = typename LinearKalmanTransitionModel<S, C>::TransitionMatrixType;
  using InputMatrixType = typename LinearKalmanTransitionModel<S, C>::InputMatrixType;

protected:
  NoiseMatrixType W_;         // Process noise covariance
  NoiseMappingMatrixType G_;  // Noise mapping matrix

public:
  /**
   * @brief Constructor
   *
   * Initializes the extended constant velocity model with the provided noise covariance matrix.
   *
   * @param W Process noise covariance matrix (defaults to identity matrix)
   */
  explicit ExtendedConstantVelocityTransitionModel(
    const NoiseMatrixType & W = NoiseMatrixType::Identity())
  : LinearKalmanTransitionModel<S, C>(), W_(W), G_(NoiseMappingMatrixType::Zero())
  {
  }

  /**
   * @brief Model initialization method
   *
   * Initializes the state transition matrix to identity.
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
   * @param state The current state
   * @param timestamp The current timestamp
   * @param delta_t The time difference between the current and previous time steps
   */
  void onEveryTimestep(
    const StateType & state, const type::Timestamp & timestamp,
    const type::Duration & delta_t) override
  {
    const double dt = ufil::to_seconds(delta_t);
    const double dt2 = 0.5 * std::pow(dt, 2);

    // Update the state transition matrix F
    this->F_ = TransitionMatrixType::Identity();
    this->F_(StateType::X, StateType::VX) = dt;
    this->F_(StateType::Y, StateType::VY) = dt;
    this->F_(StateType::YAW, StateType::YAW_RATE) = dt;

    // Update the process noise mapping matrix G
    G_ = NoiseMappingMatrixType::Zero();
    G_(StateType::X, 0) = dt2;        // Noise affecting x position
    G_(StateType::Y, 1) = dt2;        // Noise affecting y position
    G_(StateType::VX, 0) = dt;        // Noise affecting x velocity
    G_(StateType::VY, 1) = dt;        // Noise affecting y velocity
    G_(StateType::YAW, 2) = dt2;      // Noise affecting yaw
    G_(StateType::YAW_RATE, 2) = dt;  // Noise affecting yaw
    // Update the process noise covariance matrix Q
    this->Q_ = G_ * W_ * G_.transpose();
  }
};

}  // namespace transition
}  // namespace model
}  // namespace ufil

#endif  // UFIL_OBJECT_TRACKING__MODELS__TRANSITION__EXTENDED_CV_TRANSITION_MODEL_HPP_
