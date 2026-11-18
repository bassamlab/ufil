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

#include "ufil_examples_carla_adapter/carla_adapter.hpp"

#include <algorithm>
#include <charconv>
#include <cmath>
#include <set>
#include <stdexcept>
#include <string>
#include <system_error>
#include <utility>
#include <vector>

namespace ufil_examples_carla_adapter
{
namespace
{

constexpr char kCooperativeVehiclePrefix[] = "vehicle_";

/// Simulator objects are ground truth; these variances keep downstream filters well-conditioned.
constexpr std::array<double, 8> kStateVariances{0.9, 0.9, 1.4, 1.4, 2.0, 2.0, 0.05, 0.05};
constexpr std::array<double, 3> kDimensionVariances{0.01, 0.01, 0.01};
constexpr double kAxleVariance = 1e-9;

Axle makeAxle(double track_width, double center_to_axle, std::vector<bool> wheel_detected)
{
  Axle axle;
  axle.track_width = track_width;
  axle.track_width_variance = track_width > 0.0 ? kAxleVariance : 0.0;
  axle.center_to_axle = center_to_axle;
  axle.center_to_axle_variance = kAxleVariance;
  axle.wheel_detected = std::move(wheel_detected);
  return axle;
}

}  // namespace

std::optional<std::string> validate(const CarlaAdapterConfig & config)
{
  if (!config.filter_cooperative_vehicles) {
    return std::nullopt;
  }
  if (config.cooperative_vehicle_ids.empty()) {
    return "the cooperative vehicle filter needs at least one vehicle ID";
  }
  std::set<std::int64_t> unique_ids;
  for (const auto id : config.cooperative_vehicle_ids) {
    if (id < 0) {
      return "cooperative vehicle IDs must not be negative, got " + std::to_string(id);
    }
    if (!unique_ids.insert(id).second) {
      return "duplicate cooperative vehicle ID " + std::to_string(id);
    }
  }
  return std::nullopt;
}

std::optional<std::int64_t> cooperativeVehicleId(const std::string & role_name)
{
  const std::string prefix = kCooperativeVehiclePrefix;
  if (!role_name.starts_with(prefix) || role_name.size() == prefix.size()) {
    return std::nullopt;
  }
  const char * begin = role_name.data() + prefix.size();
  const char * end = role_name.data() + role_name.size();
  if (!std::all_of(begin, end, [](char c) {return c >= '0' && c <= '9';})) {
    return std::nullopt;
  }
  std::int64_t id = 0;
  const auto [ptr, error] = std::from_chars(begin, end, id);
  if (error != std::errc() || ptr != end) {
    return std::nullopt;
  }
  return id;
}

ObjectClass classify(const std::string & type)
{
  if (type == "vehicle.tesla.model3") {
    return ObjectClass::kCar;
  }
  if (type == "vehicle.kawasaki.ninja") {
    return ObjectClass::kMotorcycle;
  }
  if (type == "vehicle.carlamotors.european_hgv") {
    return ObjectClass::kTruck;
  }
  return ObjectClass::kOther;
}

std::vector<Axle> axleGeometry(const std::string & type)
{
  // Wheel base and track width of the CARLA blueprints.
  if (type == "vehicle.tesla.model3") {
    constexpr double kTrackWidth = 2.163450002670288;
    constexpr double kHalfWheelBase = 4.791779518127441 / 2.0;
    return {
      makeAxle(kTrackWidth, kHalfWheelBase, {true, true}),
      makeAxle(kTrackWidth, -kHalfWheelBase, {true, true}),
    };
  }
  if (type == "vehicle.kawasaki.ninja") {
    constexpr double kHalfWheelBase = 2.043684244155884 / 2.0;
    return {
      makeAxle(0.0, kHalfWheelBase, {true}),
      makeAxle(0.0, -kHalfWheelBase, {true}),
    };
  }
  if (type == "vehicle.carlamotors.european_hgv") {
    constexpr double kTrackWidth = 2.8910882472991943;
    constexpr double kHalfWheelBase = 7.935710430145264 / 2.0;
    return {
      makeAxle(kTrackWidth, kHalfWheelBase, {true, true}),
      makeAxle(kTrackWidth, -kHalfWheelBase, {true, true, true, true}),
      makeAxle(kTrackWidth, -kHalfWheelBase * 0.7, {true, true, true, true}),
    };
  }
  return {};
}

std::vector<Object> transformObjects(const std::vector<Object> & objects, const Pose2D & pose)
{
  const double c = std::cos(pose.yaw);
  const double s = std::sin(pose.yaw);
  const auto rotate = [c, s](double & x, double & y) {
      const double rotated_x = c * x - s * y;
      y = s * x + c * y;
      x = rotated_x;
    };

  std::vector<Object> result = objects;
  for (auto & object : result) {
    auto & state = object.state;
    rotate(state.x, state.y);
    state.x += pose.x;
    state.y += pose.y;
    rotate(state.v_x, state.v_y);
    rotate(state.a_x, state.a_y);
    state.yaw += pose.yaw;
  }
  return result;
}

CarlaAdapter::CarlaAdapter(CarlaAdapterConfig config)
: config_(std::move(config))
{
  if (const auto error = validate(config_)) {
    throw std::invalid_argument(*error);
  }
}

void CarlaAdapter::updateActors(const std::vector<ActorInfo> & actors)
{
  actors_.clear();
  for (const auto & actor : actors) {
    actors_[actor.id] = actor;
  }
  has_actors_ = true;
  ++statistics_.actor_lists;
}

bool CarlaAdapter::hasActors() const
{
  return has_actors_;
}

std::vector<Object> CarlaAdapter::convert(const std::vector<SimulatorObject> & objects)
{
  ++statistics_.object_arrays;
  statistics_.objects += objects.size();

  std::vector<Object> result;
  result.reserve(objects.size());
  for (const auto & simulator_object : objects) {
    const auto actor_it = actors_.find(simulator_object.id);
    const ActorInfo * actor = actor_it == actors_.end() ? nullptr : &actor_it->second;

    if (config_.filter_cooperative_vehicles) {
      const auto vehicle_id =
        actor == nullptr ? std::nullopt : cooperativeVehicleId(actor->role_name);
      if (!vehicle_id || std::find(
          config_.cooperative_vehicle_ids.begin(), config_.cooperative_vehicle_ids.end(),
          *vehicle_id) == config_.cooperative_vehicle_ids.end())
      {
        ++statistics_.skipped_objects;
        continue;
      }
    }
    if (actor == nullptr) {
      ++statistics_.unknown_actors;
    }

    Object object;
    object.id = simulator_object.id;
    object.state = simulator_object.state;
    object.state_variances = kStateVariances;
    object.dimensions = simulator_object.dimensions;
    object.dimension_variances = kDimensionVariances;
    object.existence_probability = 1.0;
    if (actor != nullptr) {
      object.object_class = classify(actor->type);
      object.axles = axleGeometry(actor->type);
    }
    result.push_back(std::move(object));
  }
  statistics_.converted_objects += result.size();
  return result;
}

const CarlaAdapterConfig & CarlaAdapter::config() const
{
  return config_;
}

const AdapterStatistics & CarlaAdapter::statistics() const
{
  return statistics_;
}

}  // namespace ufil_examples_carla_adapter
