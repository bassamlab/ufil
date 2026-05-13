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

#ifndef UFIL_OBJECT_TRACKING__PREDICT__COMPOSITE_PREDICTOR_HPP_
#define UFIL_OBJECT_TRACKING__PREDICT__COMPOSITE_PREDICTOR_HPP_

#include <algorithm>
#include <execution>
#include <memory>
#include <utility>
#include <vector>

#include "ufil_object_tracking/predict/predictor.hpp"
#include "ufil_object_tracking/types/time.hpp"

namespace ufil
{
namespace predict
{

template<typename ExecutionPolicy>
concept ExecutionPolicyConcept = requires {std::is_execution_policy_v<ExecutionPolicy>;};

/**
 * @class CompositePredictor
 * @brief Combines multiple predictors into a composite predictor that applies them concurrently or sequentially based
 * on an execution policy.
 *
 * @tparam ExecutionPolicy Policy defining how predictions are executed (e.g., parallel).
 * @tparam T Type of tracks being predicted.
 */
template<ExecutionPolicyConcept ExecutionPolicy, typename T>
class CompositePredictor : public Predictor<T>
{
public:
  using BaseClass = Predictor<T>;

private:
  std::vector<std::shared_ptr<BaseClass>> sub_predictors_;

public:
  CompositePredictor()
  : BaseClass() {}

  virtual ~CompositePredictor() = default;

  /**
   * Adds a new predictor to the composite list of sub-predictors.
   */
  void addPredictor(std::shared_ptr<BaseClass> predictor)
  {
    sub_predictors_.push_back(std::move(predictor));
  }

  /**
   * Applies all sub-predictors on a given track according to an execution policy (e.g., parallel).
   */
  void predict(
    typename BaseClass::TrackType & track,
    const std::optional<typename BaseClass::ControlType> & control,
    const type::Timestamp & timestamp) override
  {
    this->storeControlInput(track, control);
    auto policy_instance = ExecutionPolicy{};

    auto run_predictor_lambda = [&track, &control,
        &timestamp](const std::shared_ptr<BaseClass> & predictor) {
        predictor->predict(track, control, timestamp);
      };

    std::for_each(policy_instance, sub_predictors_.begin(), sub_predictors_.end(),
          run_predictor_lambda);
  }
};

// Convenience aliases for different execution policies
template<class T>
using ParallelPredictor = CompositePredictor<std::execution::parallel_policy, T>;

template<class T>
using ParallelUnsequencedPredictor = CompositePredictor<std::execution::parallel_unsequenced_policy,
    T>;

template<class T>
using SequentialPredictor = CompositePredictor<std::execution::sequenced_policy, T>;

template<class T>
using UnsequencedPredictor = CompositePredictor<std::execution::unsequenced_policy, T>;

}  // namespace predict
}  // namespace ufil

#endif  // UFIL_OBJECT_TRACKING__PREDICT__COMPOSITE_PREDICTOR_HPP_
