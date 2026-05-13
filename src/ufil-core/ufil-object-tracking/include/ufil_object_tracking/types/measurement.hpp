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

#ifndef UFIL_OBJECT_TRACKING__TYPES__MEASUREMENT_HPP_
#define UFIL_OBJECT_TRACKING__TYPES__MEASUREMENT_HPP_

#include "ufil_object_tracking/types/covariance.hpp"
#include "ufil_object_tracking/types/dimension.hpp"
#include "ufil_object_tracking/types/id.hpp"
#include "ufil_object_tracking/types/matrix.hpp"
#include "ufil_object_tracking/types/classification.hpp"
#include "ufil_object_tracking/types/existence_probability.hpp"
#include "ufil_object_tracking/types/vector.hpp"

namespace ufil
{
namespace type
{

template<int M>
class Measurement : public IdInterface, public VectorInterface<M>, public CovarianceInterface<M>
{
public:
  constexpr static auto Size = M;
  using CovarianceMatrixType = CovarianceInterface<M>::Type;
  using MeasurementVectorType = VectorInterface<M>::Type;

protected:
  ufil::type::classification::ObjectClassification classification_;
  ufil::type::Scalar existence_probability_;

public:
  Measurement()
  : IdInterface(), VectorInterface<M>(), CovarianceInterface<M>(), existence_probability_(0.0f)
  {
  }

  [[nodiscard]] const MeasurementVectorType & measurementVector() const
  {
    return VectorInterface<M>::vector();
  }

  [[nodiscard]] MeasurementVectorType & measurementVector()
  {
    return VectorInterface<M>::vector();
  }

  [[nodiscard]] const auto & classification() const
  {
    return classification_;
  }

  [[nodiscard]] auto & classification()
  {
    return classification_;
  }

  [[nodiscard]] const ufil::type::Scalar & existenceProbability() const
  {
    return existence_probability_;
  }

  [[nodiscard]] ufil::type::Scalar & existenceProbability()
  {
    return existence_probability_;
  }
};

namespace measurement
{
class Position2D : public ufil::type::Measurement<2>
{
public:
  static constexpr int X = 0;
  static constexpr int Y = 1;

  Position2D()
  : ufil::type::Measurement<2>()
  {
  }

  ufil::type::Vector2 position() const
  {
    return {this->measurementVector()[X], this->measurementVector()[Y]};
  }

  ufil::type::Scalar x() const
  {
    return this->measurementVector()[X];
  }

  ufil::type::Scalar y() const
  {
    return this->measurementVector()[Y];
  }

  ufil::type::Scalar & x()
  {
    return this->measurementVector()[X];
  }

  ufil::type::Scalar & y()
  {
    return this->measurementVector()[Y];
  }
};

class Pose2D : public ufil::type::Measurement<3>
{
public:
  using MeasurementVectorType = typename ufil::type::Measurement<3>::MeasurementVectorType;

public:
  static constexpr int X = 0;
  static constexpr int Y = 1;
  static constexpr int YAW = 2;

  Pose2D()
  : ufil::type::Measurement<3>()
  {
  }

  ufil::type::Vector2 position() const
  {
    return {this->measurementVector()[X], this->measurementVector()[Y]};
  }

  ufil::type::Scalar x() const
  {
    return this->measurementVector()[X];
  }

  ufil::type::Scalar y() const
  {
    return this->measurementVector()[Y];
  }

  ufil::type::Scalar yaw() const
  {
    return this->measurementVector()[YAW];
  }

  ufil::type::Scalar & x()
  {
    return this->measurementVector()[X];
  }

  ufil::type::Scalar & y()
  {
    return this->measurementVector()[Y];
  }

  ufil::type::Scalar & yaw()
  {
    return this->measurementVector()[YAW];
  }
};

class Pose2DWithDimension3D : public Pose2D
{
public:
  using DimensionType = ufil::type::dimension::Dimension3D;
  using MeasurementVectorType = typename Pose2D::MeasurementVectorType;

protected:
  DimensionType dimension_;

public:
  const DimensionType & dimension() const
  {
    return this->dimension_;
  }
  DimensionType & dimension()
  {
    return this->dimension_;
  }
};

class Pose2DWithDimension2D : public Pose2D
{
public:
  using DimensionType = ufil::type::dimension::Dimension2D;

protected:
  DimensionType dimension_;

public:
  const DimensionType & dimension() const
  {
    return this->dimension_;
  }
  DimensionType & dimension()
  {
    return this->dimension_;
  }
};

class Position3D : public ufil::type::Measurement<3>
{
public:
  static constexpr int X = 0;
  static constexpr int Y = 1;
  static constexpr int Z = 2;

  Position3D()
  : ufil::type::Measurement<3>()
  {
  }

  ufil::type::Vector3 position() const
  {
    return {this->measurementVector()[X], this->measurementVector()[Y],
      this->measurementVector()[Z]};
  }

  ufil::type::Scalar x() const
  {
    return this->measurementVector()[X];
  }

  ufil::type::Scalar y() const
  {
    return this->measurementVector()[Y];
  }

  ufil::type::Scalar z() const
  {
    return this->measurementVector()[Z];
  }

  ufil::type::Scalar & x()
  {
    return this->measurementVector()[X];
  }

  ufil::type::Scalar & y()
  {
    return this->measurementVector()[Y];
  }

  ufil::type::Scalar & z()
  {
    return this->measurementVector()[Z];
  }
};
}  // namespace measurement

}  // namespace type
}  // namespace ufil

#endif  // UFIL_OBJECT_TRACKING__TYPES__MEASUREMENT_HPP_
