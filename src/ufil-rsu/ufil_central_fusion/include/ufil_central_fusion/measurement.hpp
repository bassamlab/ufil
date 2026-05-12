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

#ifndef UFIL_CENTRAL_FUSION__MEASUREMENT_HPP_
#define UFIL_CENTRAL_FUSION__MEASUREMENT_HPP_

#include <string>
#include <array>

#include <ufil_object_tracking/types/measurement.hpp>
#include <ufil_object_tracking/types/scalar.hpp>
#include <ufil_object_tracking/types/vector.hpp>

#include "ufil_central_fusion/sensor_data.hpp"
#include "ufil_central_fusion/dimension.hpp"

namespace ufil_central_fusion
{

class DynamicMeasurement : public ufil::type::Measurement<6>
{
public:
  using DimensionType = typename ufil_central_fusion::DynamicDimension;

private:
  bool cam_ = false;

  DimensionType dimension_{};

  bool x_assigned_ = false;
  bool y_assigned_ = false;
  bool vx_assigned_ = false;
  bool vy_assigned_ = false;
  bool yaw_assigned_ = false;
  bool yaw_rate_assigned_ = false;

public:
  static constexpr int X = 0;
  static constexpr int Y = 1;
  static constexpr int VX = 2;
  static constexpr int VY = 3;
  static constexpr int YAW = 4;
  static constexpr int YAW_RATE = 5;

  DynamicMeasurement();

public:
  const DynamicMeasurement::DimensionType & dimension() const;

  DynamicMeasurement::DimensionType & dimension();

  ufil::type::Vector2 position() const;

  ufil::type::Vector2 velocity() const;

  ufil::type::Scalar x() const;

  ufil::type::Scalar y() const;

  ufil::type::Scalar vx() const;

  ufil::type::Scalar vy() const;

  ufil::type::Scalar yaw() const;

  ufil::type::Scalar yawRate() const;

  bool isCam() const;

  bool hasX() const;

  bool hasY() const;

  bool hasVx() const;

  bool hasVy() const;

  bool hasYaw() const;

  bool hasYawRate() const;

  ufil::type::Scalar & x();

  ufil::type::Scalar & y();

  ufil::type::Scalar & vx();

  ufil::type::Scalar & vy();

  ufil::type::Scalar & yaw();

  ufil::type::Scalar & yawRate();

  bool & isCam();
};

}  // namespace ufil_central_fusion

#endif  // UFIL_CENTRAL_FUSION__MEASUREMENT_HPP_
