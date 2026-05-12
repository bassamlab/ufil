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

#ifndef UFIL_OBJECT_TRACKING__TYPES__OCCUPANCY_GRID_HPP_
#define UFIL_OBJECT_TRACKING__TYPES__OCCUPANCY_GRID_HPP_

#include <cstdint>
#include <vector>

#include "ufil_object_tracking/types/scalar.hpp"

namespace ufil
{
namespace type
{

class OccupancyGrid
{
public:
  [[nodiscard]] Scalar resolution() const {return resolution_;}
  [[nodiscard]] Scalar & resolution() {return resolution_;}

  [[nodiscard]] std::uint32_t width() const {return width_;}
  [[nodiscard]] std::uint32_t & width() {return width_;}

  [[nodiscard]] std::uint32_t height() const {return height_;}
  [[nodiscard]] std::uint32_t & height() {return height_;}

  [[nodiscard]] Scalar originX() const {return origin_x_;}
  [[nodiscard]] Scalar & originX() {return origin_x_;}

  [[nodiscard]] Scalar originY() const {return origin_y_;}
  [[nodiscard]] Scalar & originY() {return origin_y_;}

  [[nodiscard]] Scalar originZ() const {return origin_z_;}
  [[nodiscard]] Scalar & originZ() {return origin_z_;}

  [[nodiscard]] const std::vector<std::int8_t> & data() const {return data_;}
  [[nodiscard]] std::vector<std::int8_t> & data() {return data_;}

private:
  Scalar resolution_ = 0.0;
  std::uint32_t width_ = 0;
  std::uint32_t height_ = 0;
  Scalar origin_x_ = 0.0;
  Scalar origin_y_ = 0.0;
  Scalar origin_z_ = 0.0;
  std::vector<std::int8_t> data_;
};

}  // namespace type
}  // namespace ufil

#endif  // UFIL_OBJECT_TRACKING__TYPES__OCCUPANCY_GRID_HPP_
