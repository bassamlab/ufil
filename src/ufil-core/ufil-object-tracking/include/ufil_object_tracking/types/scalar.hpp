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

#ifndef UFIL_OBJECT_TRACKING__TYPES__SCALAR_HPP_
#define UFIL_OBJECT_TRACKING__TYPES__SCALAR_HPP_

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace ufil
{
namespace type
{

using Scalar = double;

}  // namespace type
namespace scalar
{

constexpr ufil::type::Scalar ScalarInf = std::numeric_limits<ufil::type::Scalar>::infinity();
constexpr ufil::type::Scalar ScalarMax = std::numeric_limits<ufil::type::Scalar>::max();
constexpr ufil::type::Scalar ScalarMin = std::numeric_limits<ufil::type::Scalar>::min();

inline bool is_infinite(const ufil::type::Scalar & value)
{
  return std::isinf(value);
}

inline bool is_nan(const ufil::type::Scalar & value)
{
  return std::isnan(value);
}

inline bool is_valid(const ufil::type::Scalar & value)
{
  return !is_infinite(value) && !is_nan(value);
}

inline bool is_invalid(const ufil::type::Scalar & value)
{
  return !is_valid(value);
}

inline ufil::type::Scalar clamp(
  const ufil::type::Scalar & value, const ufil::type::Scalar & min_value,
  const ufil::type::Scalar & max_value)
{
  if (is_invalid(value)) {
    throw std::invalid_argument("value is invalid");
  }
  if (is_invalid(min_value)) {
    throw std::invalid_argument("min_value is invalid");
  }
  if (is_invalid(max_value)) {
    throw std::invalid_argument("max_value is invalid");
  }
  return std::max(std::min(value, max_value), min_value);
}

}  // namespace scalar

}  // namespace ufil

#endif  // UFIL_OBJECT_TRACKING__TYPES__SCALAR_HPP_
