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

#ifndef UFIL_OBJECT_TRACKING__MODELS__TRANSITION__LINEAR_KALMAN_TRANSITION_MODEL_HPP_
#define UFIL_OBJECT_TRACKING__MODELS__TRANSITION__LINEAR_KALMAN_TRANSITION_MODEL_HPP_

#include <memory>
#include <optional>

#include "ufil_object_tracking/models/transition/transition_model.hpp"
#include "ufil_object_tracking/types/control.hpp"
#include "ufil_object_tracking/types/matrix.hpp"
#include "ufil_object_tracking/types/time.hpp"

namespace ufil
{
namespace model
{

/**
 * @brief Linear Kalman Transition Model
 *
 * This class implements a linear transition model for use in Kalman filters.
 * It models the evolution of the system state over time using linear equations.
 *
 * The state transition is given by:
 *   x' = F * x + B * u + w
 *
 * where:
 *   x  - Previous state vector
 *   x' - Predicted state vector
 *   F  - State transition matrix
 *   B  - Control input matrix
 *   u  - Control vector
 *   w  - Process noise (assumed to be zero-mean Gaussian with covariance Q)
 *
 * @tparam S State type, must provide Size, StateVectorType, and CovarianceMatrixType
 * @tparam C Control type, must provide Size and ControlVectorType
 */
template<typename S, typename C>
class LinearKalmanTransitionModel : public TransitionModel<S, C>
{
public:
  // Define matrix and vector types based on state and control types
  using StateType = typename TransitionModel<S, C>::StateType;
  using ControlType = typename TransitionModel<S, C>::ControlType;

  using TransitionMatrixType = ufil::type::Matrix<StateType::Size, StateType::Size>;
  using InputMatrixType = ufil::type::Matrix<StateType::Size, ControlType::Size>;
  using CovarianceMatrixType = typename StateType::CovarianceMatrixType;
  using ControlVectorType = typename ControlType::ControlVectorType;
  using StateVectorType = typename StateType::StateVectorType;

protected:
  /// Process noise covariance matrix (Q)
  CovarianceMatrixType Q_;

  /// State transition matrix (F)
  TransitionMatrixType F_;

  /// Control input matrix (B)
  InputMatrixType B_;

public:
  /**
   * @brief Constructor
   *
   * Initializes the transition model with the provided matrices.
   *
   * @param Q Process noise covariance matrix
   * @param F State transition matrix
   * @param B Control input matrix
   */
  explicit LinearKalmanTransitionModel(
    const CovarianceMatrixType & Q = CovarianceMatrixType::Zero(),
    const TransitionMatrixType & F = TransitionMatrixType::Zero(),
    const InputMatrixType & B = InputMatrixType::Zero())
  : Q_(Q), F_(F), B_(B)
  {
  }

  /**
   * @brief Performs a state transition step
   *
   * Predicts the next state and covariance based on the prior state,
   * optional control input, and the process noise.
   *
   * @param prior_state The previous state estimate
   * @param control Optional control input
   * @param timestamp The current timestamp (unused in this model)
   * @param resulting_state The predicted state estimate
   */
  void step(
    const StateType & prior_state, const std::optional<ControlType> & control,
    const type::Timestamp & /*timestamp*/,
    StateType & resulting_state) override
  {
    // Initialize control vector (u) to zero
    ControlVectorType u = ControlVectorType::Zero();

    // If control input is provided, use its control vector
    if (control) {
      u = control.value().controlVector();
    }

    // Retrieve the prior state vector (x)
    const StateVectorType & x = prior_state.stateVector();

    // Predict the next state vector: x' = F * x + B * u
    StateVectorType predicted_x = F_ * x + B_ * u;

    // Retrieve the prior covariance matrix (P)
    const CovarianceMatrixType & P = prior_state.covariance();

    // Predict the next covariance matrix: P' = F * P * F^T + Q
    CovarianceMatrixType predicted_P = F_ * P * F_.transpose() + Q_;

    // Update the resulting state with the predicted values
    resulting_state.stateVector() = predicted_x;
    resulting_state.covariance() = predicted_P;
  }

  /**
   * @brief Sets the process noise covariance matrix (Q)
   *
   * @param Q New process noise covariance matrix
   */
  void setProcessNoiseCovariance(const CovarianceMatrixType & Q)
  {
    Q_ = Q;
  }

  /**
   * @brief Gets the process noise covariance matrix (Q)
   *
   * @return The current process noise covariance matrix
   */
  const CovarianceMatrixType & getProcessNoiseCovariance() const
  {
    return Q_;
  }

  /**
   * @brief Sets the state transition matrix (F)
   *
   * @param F New state transition matrix
   */
  void setTransitionMatrix(const TransitionMatrixType & F)
  {
    F_ = F;
  }

  /**
   * @brief Gets the state transition matrix (F)
   *
   * @return The current state transition matrix
   */
  const TransitionMatrixType & getTransitionMatrix() const
  {
    return F_;
  }

  /**
   * @brief Sets the control input matrix (B)
   *
   * @param B New control input matrix
   */
  void setInputMatrix(const InputMatrixType & B)
  {
    B_ = B;
  }

  /**
   * @brief Gets the control input matrix (B)
   *
   * @return The current control input matrix
   */
  const InputMatrixType & getInputMatrix() const
  {
    return B_;
  }
};

}  // namespace model
}  // namespace ufil

#endif  // UFIL_OBJECT_TRACKING__MODELS__TRANSITION__LINEAR_KALMAN_TRANSITION_MODEL_HPP_
