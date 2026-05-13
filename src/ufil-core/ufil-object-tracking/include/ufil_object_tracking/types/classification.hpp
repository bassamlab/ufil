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

#ifndef UFIL_OBJECT_TRACKING__TYPES__CLASSIFICATION_HPP_
#define UFIL_OBJECT_TRACKING__TYPES__CLASSIFICATION_HPP_

#include "ufil_object_tracking/types/matrix.hpp"
#include "ufil_object_tracking/types/scalar.hpp"
#include "ufil_object_tracking/types/vector.hpp"

namespace ufil
{
namespace type
{

template<int N>
class Classification : public VectorInterface<N>
{
public:
  using VectorType = Vector<N>;

public:
  Classification()
  : VectorInterface<N>()
  {
  }
  explicit Classification(const VectorType & vector)
  : VectorInterface<N>(vector)
  {
  }

  [[nodiscard]] const auto & classificationVector() const
  {
    return VectorInterface<N>::vector();
  }

  [[nodiscard]] auto & classificationVector()
  {
    return VectorInterface<N>::vector();
  }

  void normalize()
  {
    VectorInterface<N>::vector().normalize();
  }
};

namespace classification
{

class ObjectClassification : public ufil::type::Classification<7>
{
public:
  static constexpr int CAR = 0;
  static constexpr int TRUCK = 1;
  static constexpr int MOTORCYCLE = 2;
  static constexpr int BICYCLE = 3;
  static constexpr int PEDESTRIAN = 4;
  static constexpr int STATIONARY = 5;
  static constexpr int OTHER = 6;

  using VectorType = ufil::type::Classification<7>::VectorType;

  ufil::type::Scalar car() const
  {
    return this->classificationVector()[CAR];
  }
  ufil::type::Scalar truck() const
  {
    return this->classificationVector()[TRUCK];
  }
  ufil::type::Scalar pedestrian() const
  {
    return this->classificationVector()[PEDESTRIAN];
  }
  ufil::type::Scalar bicycle() const
  {
    return this->classificationVector()[BICYCLE];
  }
  ufil::type::Scalar motorcycle() const
  {
    return this->classificationVector()[MOTORCYCLE];
  }
  ufil::type::Scalar stationary() const
  {
    return this->classificationVector()[STATIONARY];
  }
  ufil::type::Scalar other() const
  {
    return this->classificationVector()[OTHER];
  }

  ufil::type::Scalar & car()
  {
    return this->classificationVector()[CAR];
  }
  ufil::type::Scalar & truck()
  {
    return this->classificationVector()[TRUCK];
  }
  ufil::type::Scalar & pedestrian()
  {
    return this->classificationVector()[PEDESTRIAN];
  }
  ufil::type::Scalar & bicycle()
  {
    return this->classificationVector()[BICYCLE];
  }
  ufil::type::Scalar & motorcycle()
  {
    return this->classificationVector()[MOTORCYCLE];
  }
  ufil::type::Scalar & stationary()
  {
    return this->classificationVector()[STATIONARY];
  }
  ufil::type::Scalar & other()
  {
    return this->classificationVector()[OTHER];
  }
};

}  // namespace classification

}  // namespace type
}  // namespace ufil
#endif  // UFIL_OBJECT_TRACKING__TYPES__CLASSIFICATION_HPP_
