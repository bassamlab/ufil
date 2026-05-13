// Copyright 2026 Chair of Embedded Software (Computer Science 11) - RWTH Aachen University
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


#ifndef CONFIGURATION_HPP_
#define CONFIGURATION_HPP_

#include <string>
#include <iostream>

#include "geom.hpp"

namespace cppdriver::impl::configuration
{
struct Configuration
{
  float X{};
  float Y{};
  float Z{};
  float HEADING{};
  static constexpr float WHEEL_CONTACT_CROSSTALK = 1.5f;
  float GRID_RESOLUTION{};
  int CELLS_WIDTH{};
  int CELLS_HEIGHT{};
  float SIMULATION_SAFETY_MARGIN{};
  float WHEEL_CONTACT_BASE_WIDTH{};
  float WHEEL_CROSSTALK_AREA{};
  ring_xy WHEEL_CONTACT_CROSSTALK_OFFSETS{};

public:
  Configuration(
    float x, float y, float z, float heading, float grid_resolution, int cells_width,
    int cells_height,
    bool microscale)
  : X(x), Y(y), Z(z), HEADING(heading),
    GRID_RESOLUTION(grid_resolution), CELLS_WIDTH(cells_width),
    CELLS_HEIGHT(cells_height)
  {
    SIMULATION_SAFETY_MARGIN = 0.167f * (microscale ? 1.0f : 18.0f);
    WHEEL_CONTACT_BASE_WIDTH = 0.015f * (microscale ? 1.0f : 18.0f);
    WHEEL_CROSSTALK_AREA = WHEEL_CONTACT_BASE_WIDTH * WHEEL_CONTACT_CROSSTALK;

    WHEEL_CONTACT_CROSSTALK_OFFSETS = {
      p_xy{-WHEEL_CROSSTALK_AREA / 2, -WHEEL_CROSSTALK_AREA / 2},
      p_xy{WHEEL_CROSSTALK_AREA / 2, -WHEEL_CROSSTALK_AREA / 2},
      p_xy{WHEEL_CROSSTALK_AREA / 2, WHEEL_CROSSTALK_AREA / 2},
      p_xy{-WHEEL_CROSSTALK_AREA / 2, WHEEL_CROSSTALK_AREA / 2}
    };
  }

  Configuration(Configuration & other) = default;

  [[nodiscard]] float getX() const
  {
    return X;
  }

  [[nodiscard]] float getY() const
  {
    return Y;
  }

  [[nodiscard]] float getZ() const
  {
    return Z;
  }

  [[nodiscard]] float getHeading() const
  {
    return HEADING;
  }

  [[nodiscard]] float getGridResolution() const
  {
    return GRID_RESOLUTION;
  }

  [[nodiscard]] int getCellsWidth() const
  {
    return CELLS_WIDTH;
  }

  [[nodiscard]] int getCellsHeight() const
  {
    return CELLS_HEIGHT;
  }

  [[nodiscard]] float getSimulationSafetyMargin() const
  {
    return SIMULATION_SAFETY_MARGIN;
  }

  [[nodiscard]] const ring_xy & getWheelContactCrosstalkOffsets() const
  {
    return WHEEL_CONTACT_CROSSTALK_OFFSETS;
  }
};
}  // namespace cppdriver::impl::configuration

#endif  // CONFIGURATION_HPP_
