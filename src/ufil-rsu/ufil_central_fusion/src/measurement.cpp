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

#include "ufil_central_fusion/measurement.hpp"

namespace ufil_central_fusion
{

DynamicMeasurement::DynamicMeasurement()
: Measurement<6>()
{
}

const DynamicMeasurement::DimensionType & DynamicMeasurement::dimension() const
{
  return this->dimension_;
}

DynamicMeasurement::DimensionType & DynamicMeasurement::dimension()
{
  return this->dimension_;
}

ufil::type::Vector2 DynamicMeasurement::position() const
{
  return {measurementVector()[X], measurementVector()[Y]};
}

ufil::type::Vector2 DynamicMeasurement::velocity() const
{
  return {measurementVector()[VX], measurementVector()[VY]};
}

ufil::type::Scalar DynamicMeasurement::x() const
{
  return measurementVector()[X];
}

ufil::type::Scalar DynamicMeasurement::y() const
{
  return measurementVector()[Y];
}

ufil::type::Scalar DynamicMeasurement::vx() const
{
  return measurementVector()[VX];
}

ufil::type::Scalar DynamicMeasurement::vy() const
{
  return measurementVector()[VY];
}

ufil::type::Scalar DynamicMeasurement::yaw() const
{
  return measurementVector()[YAW];
}

ufil::type::Scalar DynamicMeasurement::yawRate() const
{
  return measurementVector()[YAW_RATE];
}

bool DynamicMeasurement::isCam() const
{
  return cam_;
}

bool DynamicMeasurement::hasX() const
{
  return x_assigned_;
}

bool DynamicMeasurement::hasY() const
{
  return y_assigned_;
}
bool DynamicMeasurement::hasVx() const
{
  return vx_assigned_;
}
bool DynamicMeasurement::hasVy() const
{
  return vy_assigned_;
}
bool DynamicMeasurement::hasYaw() const
{
  return yaw_assigned_;
}
bool DynamicMeasurement::hasYawRate() const
{
  return yaw_rate_assigned_;
}

ufil::type::Scalar & DynamicMeasurement::x()
{
  x_assigned_ = true;
  return measurementVector()[X];
}

ufil::type::Scalar & DynamicMeasurement::y()
{
  y_assigned_ = true;
  return measurementVector()[Y];
}

ufil::type::Scalar & DynamicMeasurement::vx()
{
  vx_assigned_ = true;
  return measurementVector()[VX];
}

ufil::type::Scalar & DynamicMeasurement::vy()
{
  vy_assigned_ = true;
  return measurementVector()[VY];
}

ufil::type::Scalar & DynamicMeasurement::yaw()
{
  yaw_assigned_ = true;
  return measurementVector()[YAW];
}

ufil::type::Scalar & DynamicMeasurement::yawRate()
{
  yaw_rate_assigned_ = true;
  return measurementVector()[YAW_RATE];
}

bool & DynamicMeasurement::isCam()
{
  return cam_;
}

}   // namespace ufil_central_fusion
