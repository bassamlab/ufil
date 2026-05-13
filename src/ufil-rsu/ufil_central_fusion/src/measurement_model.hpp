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

#ifndef MEASUREMENT_MODEL_HPP_
#define MEASUREMENT_MODEL_HPP_


#include <memory>
#include <optional>
#include <algorithm>

#include <ufil_object_tracking/models/measurement/measurement_model.hpp>
#include <ufil_object_tracking/utility_functions.hpp>

#include "ufil_central_fusion/definitions.hpp"

namespace ufil_central_fusion
{

class DynamicMeasurementModel : public ufil::model::MeasurementModel<State, Measurement>
{
protected:
  using StateCovarianceMatrixType = State::CovarianceMatrixType;
  using StateVectorType = State::StateVectorType;
  using MeasurementCovarianceMatrixType = Measurement::CovarianceMatrixType;
  using MeasurementVectorType = Measurement::MeasurementVectorType;

public:
  void step(
    const State & prior_state, const std::optional<Measurement> & measurement,
    const ufil::type::Timestamp & timestamp, State & resulting_state) override;
};

}  // namespace ufil_central_fusion

#endif  // MEASUREMENT_MODEL_HPP_
