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

#ifndef UFIL_OBJECT_TRACKING__TYPES__CONTROL_HPP_
#define UFIL_OBJECT_TRACKING__TYPES__CONTROL_HPP_

#include "ufil_object_tracking/types/matrix.hpp"
#include "ufil_object_tracking/types/vector.hpp"

namespace ufil
{
namespace type
{

template<int L>
class Control : public VectorInterface<L>
{
public:
  using ControlVectorType = Vector<L>;

public:
  constexpr static auto Size = L;

  Control()
  : VectorInterface<L>()
  {
  }

  [[nodiscard]] const ControlVectorType & controlVector() const
  {
    return this->vector_;
  }

  [[nodiscard]] ControlVectorType & controlVector()
  {
    return this->vector_;
  }
};

namespace control
{
class None : public ufil::type::Control<0>
{
};

class Acceleration2D : public ufil::type::Control<2>
{
public:
  using ControlVectorType = ufil::type::Control<2>::ControlVectorType;

  static constexpr int AX = 0;
  static constexpr int AY = 1;

  Acceleration2D()
  : ufil::type::Control<2>()
  {
  }

  ufil::type::Scalar ax() const
  {
    return this->controlVector()[AX];
  }

  ufil::type::Scalar ay() const
  {
    return this->controlVector()[AY];
  }

  ufil::type::Scalar & ax()
  {
    return this->controlVector()[AX];
  }

  ufil::type::Scalar & ay()
  {
    return this->controlVector()[AY];
  }
};
}  // namespace control

}  // namespace type
}  // namespace ufil

#endif  // UFIL_OBJECT_TRACKING__TYPES__CONTROL_HPP_
