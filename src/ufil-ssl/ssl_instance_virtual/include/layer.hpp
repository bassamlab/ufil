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


/**
 * @file layer.hpp
 * @author Marius Molz (marius.molz@rwth-aachen.de)
 * @brief Hardware sensor layer simulation
 * @version 1.0
 * @date 2024-01-12
 *
 */
#pragma once

#include <map>
#include <memory>
#include <ufil_msgs/msg/object_list.hpp>

#include "configuration.hpp"
#include "vehicle_model_interface.hpp"

namespace layer
{
/// @brief generate line pressure crosstalk based on the pressure already generated
[[nodiscard, gnu::pure]] auto
  emulateLineCrosstalk(cppdriver::dimension_t const & size_x,
  __attribute((unused)) cppdriver::dimension_t const & size_y,
  cppdriver::pressure_map const & points)->cppdriver::pressure_map;

/// @brief generate random sensor noise
[[nodiscard, gnu::pure]] auto
emulateRandomNoise(
  cppdriver::dimension_t const & size_x,
  cppdriver::dimension_t const & size_y,
  cppdriver::pressure_map const & points) -> cppdriver::pressure_map;

/// @brief rasterize the continous wheel-to-surface contact areas
[[nodiscard, gnu::pure]] auto
rasterizeWheel(
  cppdriver::dimension_t const & size_x,
  cppdriver::dimension_t const & size_y,
  ring_xy const & wheel_geometry,
  float const & grid_resolution) -> cppdriver::pressure_map;

/// @brief compute the wheel positions of the vehicle given its current pose
[[nodiscard, gnu::pure]] auto
calculateWheelPoints(
  cppdriver::impl::configuration::Configuration configuration,
  rclcpp::Time const & current_time,
  std::map<ufil_msgs::msg::Object::_id_type,
  std::shared_ptr<cppdriver::vehicle::IVehicleModel>> & object_map)
-> cppdriver::pressure_map;

}  // namespace layer
