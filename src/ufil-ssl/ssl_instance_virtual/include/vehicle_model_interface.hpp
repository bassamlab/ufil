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
 * @file vehicle_model_interface.hpp
 * @author Marius Molz (marius.molz@rwth-aachen.de)
 * @brief vehicle model interface. Every vehicle model must implement this interface
 * @version 1.0
 * @date 2024-01-12
 *
 */
#pragma once

#include <mutex>

#include <geom.hpp>
#include <kalman_cpp/ExtendedKalmanFilter.hpp>
#include <rclcpp/time.hpp>

#include "MeasurementModel.hpp"
#include "SystemModel.hpp"

namespace cppdriver::vehicle
{

using T = float;
using State = ::model::ConstantVelocityState<T>;
using SystemModel = ::model::ConstantVelocitySystemModel<T>;
using Control = ::model::TimeStepControl<T>;
using MeasurementModel = ::model::MeasurementModel<T>;
using Measurement = ::model::Measurement<T>;

class IVehicleModel {
protected:
        /// @brief current state
  State x_{};
        /// @brief vehicle kalman filter
  kalman_cpp::ExtendedKalmanFilter<State> ekf_{};
        /// @brief system model
  SystemModel system_model{};
        /// @brief mutex lock protecting model data
  rclcpp::Time last_update_{0};
  std::mutex model_mutex_{};
  bool initialized_{false};

public:
  virtual ~IVehicleModel() = default;

        /// @brief Update internal vehicle state of the vehicle model
  virtual void update(MeasurementModel & measurement_model, rclcpp::Time const & update_time) = 0;

        /// @brief Predict the internal model
        /// @param current_time Current time, needed to calculate time delta
        /// @return Predicted State
  [[nodiscard]] virtual auto predict(rclcpp::Time const & current_time) -> State = 0;

  [[nodiscard]] auto isInitalized() const -> bool {return initialized_;}

  [[nodiscard]] virtual auto getWheelPositions(
    float const & x, float const & y,
    float const & yaw) const -> vector_xy = 0;
};
}  // namespace cppdriver::vehicle
