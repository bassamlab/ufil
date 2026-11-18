// Copyright 2025 Chair of Embedded Software (Computer Science 11) - RWTH Aachen
// University
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

#ifndef UFIL_OBJECT_TRACKING__TYPES__DIMENSION_HPP_
#define UFIL_OBJECT_TRACKING__TYPES__DIMENSION_HPP_

#include <vector>

#include "ufil_object_tracking/types/matrix.hpp"
#include "ufil_object_tracking/types/vector.hpp"

namespace ufil
{
namespace type
{

struct DimensionGridmapCell
{
  type::Scalar position = 0.0;  // Dimension that this cell represents
  type::Scalar log_odds = 0.0;  // Commulated probability of this cell in log odds notions
  type::Scalar probability = 0.0;  // Probability of this cell
};

template<int M>
class Dimension : public VectorInterface<M>
{
public:
  using CovarianceMatrixType = Covariance<M>;
  using DimensionVectorType = Vector<M>;

protected:
  CovarianceMatrixType covariance_ = CovarianceMatrixType::Zero();
  // bool gridmap_initialized_ = false;
  std::vector<std::vector<DimensionGridmapCell>> gridmap_buffer_;

public:
  constexpr static auto Size = M;
  Dimension()
  : VectorInterface<M>(DimensionVectorType::Zero())
  {
  }

  [[nodiscard]] const CovarianceMatrixType & covariance() const
  {
    return this->covariance_;
  }

  [[nodiscard]] CovarianceMatrixType & covariance()
  {
    return this->covariance_;
  }

  [[nodiscard]] const DimensionVectorType & dimensionVector() const
  {
    return this->vector_;
  }

  [[nodiscard]] DimensionVectorType & dimensionVector()
  {
    return this->vector_;
  }

  [[nodiscard]] const std::vector<std::vector<DimensionGridmapCell>> & dimensionGridmapBuffer()
  const
  {
    return this->gridmap_buffer_;
  }

  [[nodiscard]] std::vector<std::vector<DimensionGridmapCell>> & dimensionGridmapBuffer()
  {
    return this->gridmap_buffer_;
  }
};

namespace dimension
{

class NoDimension : public Dimension<0>
{
public:
  using CovarianceMatrixType = Dimension<0>::CovarianceMatrixType;
  using DimensionVectorType = Dimension<0>::DimensionVectorType;

public:
  NoDimension()
  : Dimension<0>()
  {
  }
};

class Dimension2D : public Dimension<2>
{
public:
  using CovarianceMatrixType = Dimension<2>::CovarianceMatrixType;
  using DimensionVectorType = Dimension<2>::DimensionVectorType;

public:
  static constexpr int LENGTH = 0;
  static constexpr int WIDTH = 1;

  Dimension2D()
  : Dimension<2>()
  {
    this->gridmap_buffer_.push_back(std::vector<DimensionGridmapCell>());
    this->gridmap_buffer_.push_back(std::vector<DimensionGridmapCell>());
  }

  ufil::type::Scalar length() const
  {
    return this->dimensionVector()[LENGTH];
  }
  ufil::type::Scalar width() const
  {
    return this->dimensionVector()[WIDTH];
  }

  ufil::type::Scalar & length()
  {
    return this->dimensionVector()[LENGTH];
  }
  ufil::type::Scalar & width()
  {
    return this->dimensionVector()[WIDTH];
  }
};

class Dimension3D : public Dimension<3>
{
public:
  using CovarianceMatrixType = Dimension<3>::CovarianceMatrixType;
  using DimensionVectorType = Dimension<3>::DimensionVectorType;

public:
  static constexpr int LENGTH = 0;
  static constexpr int WIDTH = 1;
  static constexpr int HEIGHT = 2;

  Dimension3D()
  : Dimension<3>()
  {
    this->gridmap_buffer_.push_back(std::vector<DimensionGridmapCell>());
    this->gridmap_buffer_.push_back(std::vector<DimensionGridmapCell>());
    this->gridmap_buffer_.push_back(std::vector<DimensionGridmapCell>());
  }

  ufil::type::Scalar length() const
  {
    return this->dimensionVector()[LENGTH];
  }
  ufil::type::Scalar width() const
  {
    return this->dimensionVector()[WIDTH];
  }
  ufil::type::Scalar height() const
  {
    return this->dimensionVector()[HEIGHT];
  }

  ufil::type::Scalar & length()
  {
    return this->dimensionVector()[LENGTH];
  }
  ufil::type::Scalar & width()
  {
    return this->dimensionVector()[WIDTH];
  }
  ufil::type::Scalar & height()
  {
    return this->dimensionVector()[HEIGHT];
  }
};
}  // namespace dimension

}  // namespace type
}  // namespace ufil

#endif  // UFIL_OBJECT_TRACKING__TYPES__DIMENSION_HPP_
