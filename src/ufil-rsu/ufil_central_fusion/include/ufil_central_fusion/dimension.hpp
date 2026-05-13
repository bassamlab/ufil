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

#ifndef UFIL_CENTRAL_FUSION__DIMENSION_HPP_
#define UFIL_CENTRAL_FUSION__DIMENSION_HPP_

#include <map>

#include <ufil_object_tracking/types/dimension.hpp>

namespace ufil_central_fusion
{

class DynamicDimension : public ufil::type::Dimension<3>
{
public:
  using CovarianceMatrixType = Dimension<3>::CovarianceMatrixType;
  using DimensionVectorType = Dimension<3>::DimensionVectorType;

  bool length_assigned_ = false;
  bool width_assigned_ = false;
  bool height_assigned_ = false;

public:
  static constexpr int LENGTH = 0;
  static constexpr int WIDTH = 1;
  static constexpr int HEIGHT = 2;

  DynamicDimension();

  ufil::type::Scalar length() const;
  ufil::type::Scalar width() const;
  ufil::type::Scalar height() const;

  bool hasLength() const;
  bool hasWidth() const;
  bool hasHeight() const;

  ufil::type::Scalar & length();
  ufil::type::Scalar & width();
  ufil::type::Scalar & height();

  [[nodiscard]] size_t numberOfElements() const override;
};

}  // namespace ufil_central_fusion
#endif  // UFIL_CENTRAL_FUSION__DIMENSION_HPP_
