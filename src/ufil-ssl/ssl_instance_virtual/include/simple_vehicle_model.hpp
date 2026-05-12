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
 * @file simple_vehicle_model.hpp
 * @author Marius Molz (marius.molz@rwth-aachen.de)
 * @brief simple vehicle model based on a constant velocity model
 * @version 1.0
 * @date 2024-01-12
 *
 */
#pragma once

#include "vehicle_model_interface.hpp"
#include <geom.hpp>

namespace cppdriver::vehicle
{
class SimpleVehicleModel : public IVehicleModel {
private:
  vector_xy wheel_offsets_;

public:
  [[nodiscard]] explicit SimpleVehicleModel(const Axles & axles);

  ~SimpleVehicleModel() override = default;

  [[nodiscard]] auto predict(rclcpp::Time const & current_time) -> State override;

  void update(MeasurementModel & measurement_model, rclcpp::Time const & update_time) override;

  [[nodiscard]] auto getWheelPositions(
    float const & x, float const & y,
    float const & yaw) const -> vector_xy override;
};

}  // namespace cppdriver::vehicle
