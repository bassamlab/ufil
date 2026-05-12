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

#ifndef UFIL_CENTRAL_FUSION__DEFINITIONS_HPP_
#define UFIL_CENTRAL_FUSION__DEFINITIONS_HPP_

#include <map>

#include <ufil_object_tracking/types/state.hpp>
#include <ufil_object_tracking/models/transition/extended_cv_transition_model.hpp>
#include <ufil_object_tracking/models/transition/random_walk_transition_model.hpp>
#include <ufil_object_tracking/types/control.hpp>
#include <ufil_object_tracking/types/dimension.hpp>
#include <ufil_object_tracking/types/measurement.hpp>
#include <ufil_object_tracking/types/track.hpp>
#include "ufil_central_fusion/classification.hpp"
#include "ufil_central_fusion/existence_probability.hpp"
#include "ufil_central_fusion/measurement.hpp"


namespace ufil_central_fusion
{

using State = ufil::type::state::PoseVelocity2D;
using Dimension = ufil::type::dimension::Dimension3D;
using Classification = ufil_central_fusion::Classification;
using ExistenceProbability = ufil_central_fusion::ExistenceProbability;
using Control = ufil::type::control::None;
using Measurement = ufil_central_fusion::DynamicMeasurement;

using Track =
  ufil::type::Track<State, Dimension, Classification, ExistenceProbability, Control, Measurement>;

using OtherTransitionModel =
  typename ufil::model::transition::ExtendedConstantVelocityTransitionModel<State, Control>;

using PedestrianTransitionModel =
  typename ufil::model::transition::RandomWalkTransitionModel<State, Control>;

}  // namespace ufil_central_fusion
#endif  // UFIL_CENTRAL_FUSION__DEFINITIONS_HPP_
