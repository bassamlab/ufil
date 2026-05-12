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
#include "layer.hpp"

#include <random>

#include "vehicle_model_interface.hpp"
#include "driver_interface.hpp"
#include "gcem.hpp"
#include <geom.hpp>

namespace layer
{
[[nodiscard, gnu::pure]] auto
emulateLineCrosstalk(
  cppdriver::dimension_t const & size_x,
  __attribute((unused)) cppdriver::dimension_t const & size_y,
  cppdriver::pressure_map const & points) -> cppdriver::pressure_map
{
  cppdriver::pressure_map p{};
  for (auto const &[point, pressure] : points) {
    int32_t pressure_decline = floor(pressure * 0.7f);
    int32_t x_offset = 1;
    while (pressure_decline > 0) {
      auto const x_low = point.first - x_offset;
      auto const x_high = point.first + x_offset;
      if (x_low >= 0) {
        p[{static_cast<cppdriver::dimension_t>(x_low), point.second}] =
          std::max(p[{static_cast<cppdriver::dimension_t>(x_low), point.second}],
                                     static_cast<uint8_t>(pressure_decline));
      }
      if (x_high < size_x) {
        p[{static_cast<cppdriver::dimension_t>(x_high), point.second}] =
          std::max(p[{static_cast<cppdriver::dimension_t>(x_high), point.second}],
                                     static_cast<uint8_t>(pressure_decline));
      }
      pressure_decline = floor(pressure_decline * 0.7f);
      x_offset++;
      if (x_low < 0 || x_high >= size_x) {
        break;
      }
    }
  }

  for (auto const &[k, v] : points) {
    p[k] = v;
  }

  return p;
}

[[nodiscard, gnu::pure]] auto
emulateRandomNoise(
  cppdriver::dimension_t const & size_x,
  cppdriver::dimension_t const & size_y,
  cppdriver::pressure_map const & points) -> cppdriver::pressure_map
{
  static std::uniform_int_distribution<> distribution_x(0U, size_x - 1);
  static std::uniform_int_distribution<> distribution_y(0U, size_y - 1);
        // Gaussian distribution from 1 to 3
  static std::normal_distribution<> distribution_pressure(2.f, 0.35f);
  static uint32_t const random_points = gcem::floor((size_x * size_y) / 64U);
  static std::default_random_engine generator;
  cppdriver::pressure_map p{};

  for (uint32_t i(0); i < random_points; i++) {
    p[{distribution_x(generator), distribution_y(generator)}] =
      round(distribution_pressure(generator));
  }

  for (auto const &[k, v] : points) {
    p[k] = v;
  }
  return p;
}

[[nodiscard, gnu::pure]] auto
rasterizeWheel(
  cppdriver::dimension_t const & size_x,
  cppdriver::dimension_t const & size_y,
  ring_xy const & wheel_geometry,
  float const & grid_resolution) -> cppdriver::pressure_map
{
  cppdriver::pressure_map points{};

  auto triggered_points = rasterizeGeometry(size_x, size_y, wheel_geometry, grid_resolution);

  uint8_t pressure = 30U;
  static std::default_random_engine generator;
  std::shuffle(triggered_points.begin(), triggered_points.end(), generator);

  for (auto const & point : triggered_points) {
    points[point] = pressure -= 3U;
  }

  return points;
}

std::vector<ring_xy> createWheelBoundaries(
  cppdriver::impl::configuration::Configuration configuration, const vector_xy & wheel_positions,
  float heading)
{
  std::vector<ring_xy> wheel_boundaries{};
  for (const auto & wheel_position : wheel_positions) {
    wheel_boundaries.emplace_back((TRANSLATION_MATRIX(wheel_position) * ROTATION_MATRIX(heading)) *
        configuration.getWheelContactCrosstalkOffsets());
  }

  return wheel_boundaries;
}

[[nodiscard, gnu::pure]] auto
calculateWheelPoints(
  cppdriver::impl::configuration::Configuration configuration,
  rclcpp::Time const & current_time,
  std::map<ufil_msgs::msg::Object::_id_type,
  std::shared_ptr<cppdriver::vehicle::IVehicleModel>> & object_map)
-> cppdriver::pressure_map
{
  cppdriver::pressure_map points;
  for (const auto &[id, object] : object_map) {
    if (!object->isInitalized()) {
      continue;
    }
    auto const prediction = object->predict(current_time);

    p_xy const pose{static_cast<float>(prediction.x()), static_cast<float>(prediction.y())};

            // Create transformation matrices
            // Rotate to target origin orientation
    auto const R = ROTATION_MATRIX(-configuration.getHeading());
    auto const Rotate_angle_to_mat_cs = ROTATION_MATRIX(-M_PI_2);
            // Translate back to origin
    auto const Tb = TRANSLATION_MATRIX({-configuration.getX(), -configuration.getY()});

            // Rotate vehicle heading angle to new (mat) coordinate system
    double x_angle_component, y_angle_component;
    sincos(prediction.yaw(), &y_angle_component, &x_angle_component);
    p_xy heading_vec{x_angle_component, y_angle_component};
            // Rotate angle vector
    auto const heading_vec_translated = R * Rotate_angle_to_mat_cs * heading_vec;
            // Create angle
    auto const heading_translated =
      std::atan2(std::get<1>(heading_vec_translated), std::get<0>(heading_vec_translated));
            // Transform pose
    auto const pose_translated = R * (Tb * pose);
    float x = std::get<0>(pose_translated);
    float y = std::get<1>(pose_translated);

    auto simulation_safety_margin = configuration.getSimulationSafetyMargin();
    auto grid_resolution = configuration.getGridResolution();
    auto size_x = configuration.getCellsWidth();
    auto size_y = configuration.getCellsHeight();

    if (-simulation_safety_margin <= x &&
      x <= simulation_safety_margin + size_x * grid_resolution &&
      -simulation_safety_margin <= y &&
      y <= simulation_safety_margin + size_y * grid_resolution)
    {
      const auto & wheel_positions = object->getWheelPositions(x, y, heading_translated);
      const auto & wheel_boundaries = createWheelBoundaries(configuration, wheel_positions,
          heading_translated);

      for (const auto & wheel_boundary : wheel_boundaries) {
        const auto rasterized_wheel = rasterizeWheel(size_x, size_y, wheel_boundary,
            grid_resolution);
        points.insert(rasterized_wheel.begin(), rasterized_wheel.end());
      }
    }
  }
  return points;
}
}  // namespace layer
