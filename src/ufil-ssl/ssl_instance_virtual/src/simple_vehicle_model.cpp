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
 * @file cpm_modified_vehicle.cpp
 * @author Marius Molz (marius.molz@rwth-aachen.de)
 * @brief simple vehicle model based on a constant velocity model
 * @version 1.0
 * @date 2024-01-12
 *
 */
#include "simple_vehicle_model.hpp"
#include "vehicle_model_interface.hpp"
#include "geom.hpp"

namespace cppdriver::vehicle
{

[[nodiscard]] SimpleVehicleModel::SimpleVehicleModel(const Axles & axles)
{
  for (const auto & axle : axles) {
    if (axle.single_track) {
      this->wheel_offsets_.emplace_back(0, axle.center_to_axle);
    } else {
      this->wheel_offsets_.emplace_back(-axle.track_width / 2, axle.center_to_axle);
      this->wheel_offsets_.emplace_back(axle.track_width / 2, axle.center_to_axle);
    }
  }
}

void
SimpleVehicleModel::update(MeasurementModel & measurement_model, rclcpp::Time const & update_time)
{
  if (measurement_model.isValid()) {
    Measurement measurement = measurement_model.getMeasurement();
    if (!initialized_) {
                // Init self
      x_.setZero();
      x_.x() = measurement.x();
      x_.y() = measurement.y();
      x_.speed() = measurement.speed();
      x_.yaw() = measurement.yaw();
      x_.yawRate() = measurement.yawRate();
      ekf_.init(x_);
      last_update_ = update_time;
      initialized_ = true;
    } else {
      std::lock_guard<std::mutex> lock(model_mutex_);
      Control u{};
      u.timeStep() = (update_time - last_update_).seconds();
      auto predicted = ekf_.predict(system_model, u);
      measurement.yaw() = predicted.yaw() + model::angleDiff(measurement.yaw(), predicted.yaw());
      x_ = ekf_.update(measurement_model, measurement);
      last_update_ = update_time;
    }
  }
}

[[nodiscard]] inline auto
phi(float const theta) -> float
{
  float theta_wrapped = fmod(theta + M_PI, 2.0f * M_PI);
  if (theta_wrapped < 0) {
    theta_wrapped += 2.0f * M_PI;
  }
  return theta_wrapped - M_PI;
}

[[nodiscard]] auto
SimpleVehicleModel::predict(rclcpp::Time const & current_time) -> State
{
  std::lock_guard<std::mutex> lock(model_mutex_);
  Control u{};
  u.timeStep() = (current_time - last_update_).seconds();
  return ekf_.predict(system_model, u);
}

auto SimpleVehicleModel::getWheelPositions(
  const float & x, const float & y,
  const float & yaw) const -> vector_xy
{
  return (TRANSLATION_MATRIX(p_xy{x, y}) * ROTATION_MATRIX(yaw)) * this->wheel_offsets_;
}

}  // namespace cppdriver::vehicle
