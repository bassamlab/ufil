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

#include "ssl_simulator/mat_simulator.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <iterator>
#include <optional>
#include <random>
#include <stdexcept>
#include <utility>
#include <vector>

#include <ufil_object_tracking/types/control.hpp>
#include <ufil_object_tracking/types/state.hpp>
#include <ufil_object_tracking/models/transition/extended_cv_transition_model.hpp>
#include <ufil_object_tracking/utility_functions.hpp>

namespace ssl_simulator
{

namespace
{

/// Pressure of the first cell of a wheel contact; every further cell has kContactPressureStep
/// less. The cells of a contact are shuffled, so the pressure distribution is random.
constexpr int kContactPressure = 30;
constexpr int kContactPressureStep = 3;
/// Ratio of the pressure a cell passes on to its neighbour in the same row.
constexpr double kCrosstalkDecay = 0.7;
/// One noise sample per kNoiseCellsPerSample cells and frame.
constexpr std::size_t kNoiseCellsPerSample = 64;
constexpr double kNoiseMean = 2.0;
constexpr double kNoiseStandardDeviation = 0.35;

using TransitionModel = ufil::model::transition::ExtendedConstantVelocityTransitionModel<
  VehicleState, ufil::type::control::None>;

bool isFinite(const VehicleState & state)
{
  return state.stateVector().allFinite();
}

std::mt19937 makeGenerator(const std::optional<std::uint32_t> & seed)
{
  if (seed) {
    return std::mt19937(*seed);
  }
  std::random_device device;
  return std::mt19937(device());
}

}  // namespace

VehicleState predict(const VehicleState & state, const Timestamp & time)
{
  TransitionModel model;
  model.onModelInitialization();
  model.onEveryTimestep(state, time, time - state.timestamp());
  VehicleState prediction(time);
  model.step(state, std::nullopt, time, prediction);
  prediction.yaw() = ufil::normalize_angle(prediction.yaw());
  return prediction;
}

Point2D mapToMat(const MatPose & pose, const Point2D & point)
{
  const double dx = point.x - pose.x;
  const double dy = point.y - pose.y;
  const double c = std::cos(pose.heading);
  const double s = std::sin(pose.heading);
  return {c * dx + s * dy, -s * dx + c * dy};
}

std::vector<Point2D> wheelPositions(
  const Point2D & center, double yaw, const std::vector<Axle> & axles)
{
  const double c = std::cos(yaw);
  const double s = std::sin(yaw);
  const auto toFrame = [&](double forward, double left) {
      return Point2D{center.x + c * forward - s * left, center.y + s * forward + c * left};
    };

  std::vector<Point2D> wheels;
  for (const auto & axle : axles) {
    if (axle.single_track) {
      wheels.push_back(toFrame(axle.center_to_axle, 0.0));
    } else {
      wheels.push_back(toFrame(axle.center_to_axle, axle.track_width / 2.0));
      wheels.push_back(toFrame(axle.center_to_axle, -axle.track_width / 2.0));
    }
  }
  return wheels;
}

std::vector<Cell> contactCells(
  const Point2D & wheel, double yaw, double size, double resolution, std::size_t columns,
  std::size_t rows)
{
  const double half = size / 2.0;
  const double c = std::cos(yaw);
  const double s = std::sin(yaw);
  // Half extent of the axis-aligned bounding box of the rotated square.
  const double extent = half * (std::abs(c) + std::abs(s));

  const auto range = [&](double center, std::size_t count) -> std::optional<std::pair<std::size_t,
      std::size_t>> {
      const double first = std::floor((center - extent) / resolution);
      const double last = std::floor((center + extent) / resolution);
      if (last < 0.0 || first >= static_cast<double>(count)) {
        return std::nullopt;
      }
      return std::make_pair(
        static_cast<std::size_t>(std::max(first, 0.0)),
        static_cast<std::size_t>(std::min(last, static_cast<double>(count - 1))));
    };
  const auto column_range = range(wheel.x, columns);
  const auto row_range = range(wheel.y, rows);
  if (!column_range || !row_range) {
    return {};
  }

  std::vector<Cell> cells;
  for (std::size_t row = row_range->first; row <= row_range->second; ++row) {
    const double dy = (static_cast<double>(row) + 0.5) * resolution - wheel.y;
    for (std::size_t column = column_range->first; column <= column_range->second; ++column) {
      const double dx = (static_cast<double>(column) + 0.5) * resolution - wheel.x;
      // Cell center in the frame of the contact patch.
      const double along = c * dx + s * dy;
      const double across = -s * dx + c * dy;
      if (std::abs(along) <= half && std::abs(across) <= half) {
        cells.push_back({column, row});
      }
    }
  }
  return cells;
}

PressureMap addLineCrosstalk(const PressureMap & pressure)
{
  PressureMap result = pressure;
  for (std::size_t row = 0; row < pressure.rows; ++row) {
    for (std::size_t column = 0; column < pressure.columns; ++column) {
      int decline = static_cast<int>(std::floor(pressure.at(column, row) * kCrosstalkDecay));
      for (std::size_t offset = 1; decline > 0; ++offset) {
        const auto value = static_cast<std::uint8_t>(decline);
        const bool has_left = column >= offset;
        const bool has_right = column + offset < pressure.columns;
        if (!has_left && !has_right) {
          break;
        }
        if (has_left) {
          auto & cell = result.at(column - offset, row);
          cell = std::max(cell, value);
        }
        if (has_right) {
          auto & cell = result.at(column + offset, row);
          cell = std::max(cell, value);
        }
        decline = static_cast<int>(std::floor(decline * kCrosstalkDecay));
      }
    }
  }
  return result;
}

void applyThreshold(PressureMap & pressure, int threshold)
{
  for (auto & value : pressure.values) {
    if (value < threshold) {
      value = 0U;
    }
  }
}

MatSimulator::MatSimulator(const SimulatorConfig & config)
: config_(config), columns_(config.columns()), rows_(config.rows()),
  generator_(makeGenerator(config.seed))
{
  if (const auto error = validate(config_)) {
    throw std::invalid_argument("invalid simulator configuration: " + *error);
  }
}

std::size_t MatSimulator::setVehicles(std::vector<Vehicle> vehicles, const Timestamp & received)
{
  const auto end = std::remove_if(
    vehicles.begin(), vehicles.end(), [](const Vehicle & vehicle) {
      return !isFinite(vehicle.state);
    });
  const auto rejected = static_cast<std::size_t>(std::distance(end, vehicles.end()));
  vehicles.erase(end, vehicles.end());

  vehicles_ = std::move(vehicles);
  received_ = received;
  ++status_.object_lists;
  status_.rejected_vehicles += rejected;
  status_.last_object_list = received;
  return rejected;
}

Frame MatSimulator::step(const Timestamp & now)
{
  Frame frame;
  if (status_.last_frame && now < *status_.last_frame) {
    ++status_.time_jumps;
    frame.event = FrameEvent::kTimeJump;
  }
  if (received_ && *received_ > now) {
    // Received before a backward time jump: the object list lies in the future.
    vehicles_.clear();
    received_.reset();
    status_.last_object_list.reset();
  } else if (!vehicles_.empty() && received_ && isStale(now)) {
    vehicles_.clear();
    ++status_.expirations;
    if (frame.event == FrameEvent::kNone) {
      frame.event = FrameEvent::kVehiclesExpired;
    }
  }

  PressureMap contact(columns_, rows_);
  for (const auto & vehicle : vehicles_) {
    addVehicle(vehicle, now, contact, frame.vehicles_on_mat);
  }
  frame.pressure = addLineCrosstalk(contact);
  addNoise(frame.pressure);
  applyThreshold(frame.pressure, config_.threshold);

  ++status_.frames;
  status_.last_frame = now;
  status_.vehicles_on_mat = frame.vehicles_on_mat;
  status_.active_cells = frame.pressure.activeCells();
  return frame;
}

bool MatSimulator::isStale(const Timestamp & now) const
{
  return ufil::to_seconds(now - *received_) > config_.stale_duration;
}

SimulatorStatus MatSimulator::status() const
{
  SimulatorStatus status = status_;
  status.vehicles = vehicles_.size();
  return status;
}

void MatSimulator::addVehicle(
  const Vehicle & vehicle, const Timestamp & now, PressureMap & contact,
  std::size_t & vehicles_on_mat)
{
  const VehicleState prediction = predict(vehicle.state, now);
  const Point2D center = mapToMat(config_.pose, {prediction.x(), prediction.y()});
  const double yaw = prediction.yaw() - config_.pose.heading;

  bool on_mat = false;
  for (const auto & wheel : wheelPositions(center, yaw, vehicle.axles)) {
    auto cells = contactCells(
      wheel, yaw, config_.wheel_contact_size, config_.resolution, columns_, rows_);
    std::shuffle(cells.begin(), cells.end(), generator_);
    int pressure = kContactPressure;
    for (const auto & cell : cells) {
      pressure -= kContactPressureStep;
      if (pressure <= 0) {
        break;
      }
      auto & value = contact.at(cell.column, cell.row);
      value = std::max(value, static_cast<std::uint8_t>(pressure));
      on_mat = true;
    }
  }
  if (on_mat) {
    ++vehicles_on_mat;
  }
}

void MatSimulator::addNoise(PressureMap & pressure)
{
  std::uniform_int_distribution<std::size_t> column(0, columns_ - 1);
  std::uniform_int_distribution<std::size_t> row(0, rows_ - 1);
  std::normal_distribution<double> level(kNoiseMean, kNoiseStandardDeviation);

  const std::size_t samples = columns_ * rows_ / kNoiseCellsPerSample;
  for (std::size_t i = 0; i < samples; ++i) {
    const std::size_t c = column(generator_);
    const std::size_t r = row(generator_);
    const double value = std::clamp(std::round(level(generator_)), 0.0, 255.0);
    auto & cell = pressure.at(c, r);
    if (cell == 0U) {
      cell = static_cast<std::uint8_t>(value);
    }
  }
}

}  // namespace ssl_simulator
