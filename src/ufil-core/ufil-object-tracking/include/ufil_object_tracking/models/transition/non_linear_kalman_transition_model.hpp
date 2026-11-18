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

#ifndef UFIL_OBJECT_TRACKING__MODELS__TRANSITION__NON_LINEAR_KALMAN_TRANSITION_MODEL_HPP_
#define UFIL_OBJECT_TRACKING__MODELS__TRANSITION__NON_LINEAR_KALMAN_TRANSITION_MODEL_HPP_

#include <iostream>
#include <memory>
#include <optional>

#include "ufil_object_tracking/models/transition/linearized_transition_model.hpp"
#include "ufil_object_tracking/types/matrix.hpp"
#include "ufil_object_tracking/types/time.hpp"

namespace ufil
{
namespace model
{

template<typename S, typename C>
class NonLinearKalmanTransitionModel : public LinearizedTransitionModel<S, C>
{
public:
  using JacobianMatrixType = ufil::type::Matrix<S::Size, S::Size>;
  using CovarianceMatrixType = S::CovarianceMatrixType;
  using ControlVectorType = C::ControlVectorType;
  using StateVectorType = S::StateVectorType;

protected:
  std::function<void(const S &, const std::optional<C> &)> f_;

  JacobianMatrixType F;
  CovarianceMatrixType Q;

public:
  NonLinearKalmanTransitionModel(
    JacobianMatrixType input_F = JacobianMatrixType::Zero(),
    CovarianceMatrixType input_Q = CovarianceMatrixType::Zero())
  : F(input_F), Q(input_Q)
  {
  }

  void step(
    const S & prior_state, const std::optional<C> & control, const type::Timestamp & timestamp,
    S & resulting_state) override
  {
    ControlVectorType u = ControlVectorType::Zero();
    if (control) {
      u = control.value().controlVector();
    }

    const StateVectorType x = prior_state.stateVector();
    StateVectorType predicted_x = StateVectorType::Zero();
    this->predict(x, u, this->delta_t_, predicted_x);

    const CovarianceMatrixType P = prior_state.covariance();
    const CovarianceMatrixType predicted_P = this->F * P * this->F.transpose() + this->Q;

    resulting_state.stateVector() = predicted_x;
    resulting_state.covariance() = predicted_P;
    // std::cout << "P" << P << std::endl;
    std::cout << "predicted_P: \n" << predicted_P << std::endl;
    std::cout << "F: \n" << F << std::endl;
    std::cout << "Q: \n" << Q << std::endl;
  }

  virtual void linearize(
    const S & state, const type::Timestamp & timestamp,
    const type::Duration & delta_t)
  {
  }
  virtual void predict(
    const StateVectorType prior_x, const ControlVectorType u, const type::Duration & delta_t,
    StateVectorType & resulting_x)
  {
  }

protected:
};

}  // namespace model
}  // namespace ufil

#endif  // UFIL_OBJECT_TRACKING__MODELS__TRANSITION__NON_LINEAR_KALMAN_TRANSITION_MODEL_HPP_
