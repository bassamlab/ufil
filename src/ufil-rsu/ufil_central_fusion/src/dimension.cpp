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

#include "ufil_central_fusion/dimension.hpp"

namespace ufil_central_fusion
{

DynamicDimension::DynamicDimension()
: Dimension<3>()
{
  this->gridmap_buffer_.push_back(std::vector<ufil::type::DimensionGridmapCell>());
  this->gridmap_buffer_.push_back(std::vector<ufil::type::DimensionGridmapCell>());
  this->gridmap_buffer_.push_back(std::vector<ufil::type::DimensionGridmapCell>());
}

ufil::type::Scalar DynamicDimension::length() const
{
  return this->dimensionVector()[LENGTH];
}

ufil::type::Scalar DynamicDimension::width() const
{
  return this->dimensionVector()[WIDTH];
}

ufil::type::Scalar DynamicDimension::height() const
{
  return this->dimensionVector()[HEIGHT];
}

bool DynamicDimension::hasLength() const
{
  return length_assigned_;
}

bool DynamicDimension::hasWidth() const
{
  return width_assigned_;
}

bool DynamicDimension::hasHeight() const
{
  return height_assigned_;
}

ufil::type::Scalar & DynamicDimension::length()
{
  length_assigned_ = true;
  return this->dimensionVector()[LENGTH];
}

ufil::type::Scalar & DynamicDimension::width()
{
  width_assigned_ = true;
  return this->dimensionVector()[WIDTH];
}

ufil::type::Scalar & DynamicDimension::height()
{
  height_assigned_ = true;
  return this->dimensionVector()[HEIGHT];
}

size_t DynamicDimension::numberOfElements() const
{
  if(hasHeight()) {
    return 3;
  }
  if(hasWidth()) {
    return 2;
  }
  if(hasLength()) {
    return 1;
  }
  return 0;
}

}  // namespace ufil_central_fusion
