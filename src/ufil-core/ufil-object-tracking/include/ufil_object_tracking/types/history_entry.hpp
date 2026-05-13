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

#ifndef UFIL_OBJECT_TRACKING__TYPES__HISTORY_ENTRY_HPP_
#define UFIL_OBJECT_TRACKING__TYPES__HISTORY_ENTRY_HPP_

#include <optional>

#include "ufil_object_tracking/types/existence_probability.hpp"

namespace ufil
{
namespace type
{

template<typename S, typename D, typename C, typename E, typename I, typename M>
class HistoryEntry
{
private:
  S state_;
  D dimension_;
  C classification_vector_;
  E existence_probability_;

  std::optional<I> control_input_;
  std::optional<M> associated_measurement_;

  std::optional<type::Scalar> nis_;
  bool occluded_;

public:
  HistoryEntry(S state, D dimension)
  : state_(state)
    , dimension_(dimension)
    , classification_vector_({})
    , existence_probability_()
    , control_input_(std::nullopt)
    , associated_measurement_(std::nullopt)
  {
  }

  HistoryEntry(
    S state, D dimension, E existence_probability, C classification_vector = {},
    std::optional<I> control_input = std::nullopt,
    std::optional<M> associated_measurement = std::nullopt)
  : state_(state)
    , dimension_(dimension)
    , classification_vector_(classification_vector)
    , existence_probability_(existence_probability)
    , control_input_(control_input)
    , associated_measurement_(associated_measurement)
    , nis_(std::nullopt)
    , occluded_(false)
  {
  }

  [[nodiscard]] const S & state() const
  {
    return this->state_;
  }

  [[nodiscard]] S & state()
  {
    return this->state_;
  }

  [[nodiscard]] const D & dimension() const
  {
    return this->dimension_;
  }

  [[nodiscard]] D & dimension()
  {
    return this->dimension_;
  }

  [[nodiscard]] const E & existenceProbability() const
  {
    return this->existence_probability_;
  }

  [[nodiscard]] E & existenceProbability()
  {
    return this->existence_probability_;
  }

  [[nodiscard]] const C & classification() const
  {
    return this->classification_vector_;
  }

  [[nodiscard]] C & classification()
  {
    return this->classification_vector_;
  }

  [[nodiscard]] bool hasControlInput() const
  {
    return this->control_input_.has_value();
  }

  [[nodiscard]] const std::optional<I> & controlInput() const
  {
    return this->control_input_;
  }

  [[nodiscard]] std::optional<I> & controlInput()
  {
    return this->control_input_;
  }

  [[nodiscard]] bool hasAssociatedMeasurement() const
  {
    return this->associated_measurement_.has_value();
  }

  [[nodiscard]] const std::optional<M> & associatedMeasurement() const
  {
    return this->associated_measurement_;
  }

  [[nodiscard]] std::optional<M> & associatedMeasurement()
  {
    return this->associated_measurement_;
  }

  [[nodiscard]] const std::optional<type::Scalar> & nis() const
  {
    return this->nis_;
  }

  [[nodiscard]] std::optional<type::Scalar> & nis()
  {
    return this->nis_;
  }

  [[nodiscard]] bool hasNIS() const
  {
    return nis_.has_value() && std::isfinite(*nis_);
  }

  [[nodiscard]] bool & occluded()
  {
    return this->occluded_;
  }

  [[nodiscard]] const bool & occluded() const
  {
    return this->occluded_;
  }
};

}  // namespace type
}  // namespace ufil

#endif  // UFIL_OBJECT_TRACKING__TYPES__HISTORY_ENTRY_HPP_
