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

#ifndef UFIL_OBJECT_TRACKING__TYPES__EXISTENCE_PROBABILITY_HPP_
#define UFIL_OBJECT_TRACKING__TYPES__EXISTENCE_PROBABILITY_HPP_

#include "ufil_object_tracking/types/scalar.hpp"
#include "ufil_object_tracking/types/basic_belief_assignments.hpp"

namespace ufil::type
{

class ExistenceProbability : public ufil::type::BasicBeliefAssignments<1>
{
public:
  using VectorType = ufil::type::BasicBeliefAssignments<1>::VectorType;

  static constexpr int EXISTENCE = 0;

public:
  ExistenceProbability()
  : ufil::type::BasicBeliefAssignments<1>()
  {
    existence() = 1.0f;
  }

  [[nodiscard]] ufil::type::Scalar existence() const
  {
    return vector()[EXISTENCE];
  }

  [[nodiscard]] ufil::type::Scalar & existence()
  {
    return vector()[EXISTENCE];
  }
};

}  // namespace ufil::type

namespace ufil::type::probability
{

class ExistenceProbabilityBasicBelief : public ufil::type::BasicBeliefAssignments<3>
{
public:
  using VectorType = ufil::type::BasicBeliefAssignments<3>::VectorType;

  static constexpr int EXISTENCE = 0;
  static constexpr int NON_EXISTENCE = 1;
  static constexpr int UNCERTAINTY = 2;

  ExistenceProbabilityBasicBelief()
  : ufil::type::BasicBeliefAssignments<3>()
  {
    existence() = 1.0f / 3.0f;
    nonExistence() = 1.0f / 3.0f;
    uncertainty() = 1.0f / 3.0f;
  }

  ufil::type::Scalar existence() const
  {
    return vector()[EXISTENCE];
  }

  ufil::type::Scalar nonExistence() const
  {
    return vector()[NON_EXISTENCE];
  }

  ufil::type::Scalar uncertainty() const
  {
    return vector()[UNCERTAINTY];
  }


  ufil::type::Scalar & existence()
  {
    return vector()[EXISTENCE];
  }

  ufil::type::Scalar & nonExistence()
  {
    return vector()[NON_EXISTENCE];
  }

  ufil::type::Scalar & uncertainty()
  {
    return vector()[UNCERTAINTY];
  }
};

}  // namespace ufil::type::probability

#endif  // UFIL_OBJECT_TRACKING__TYPES__EXISTENCE_PROBABILITY_HPP_
