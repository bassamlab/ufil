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

#ifndef UFIL_EXAMPLES_CARLA_ADAPTER__CARLA_ADAPTER_HPP_
#define UFIL_EXAMPLES_CARLA_ADAPTER__CARLA_ADAPTER_HPP_

#include <array>
#include <cstdint>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

#include "ufil_examples_carla_adapter/visibility_control.hpp"

namespace ufil_examples_carla_adapter
{

/// Planar kinematic state of an object. Positions in [m], angles in [rad], SI units.
struct ObjectState
{
  double x = 0.0;
  double y = 0.0;
  double v_x = 0.0;
  double v_y = 0.0;
  double a_x = 0.0;
  double a_y = 0.0;
  double yaw = 0.0;
  double yaw_rate = 0.0;
};

/// Bounding box size [m].
struct Dimensions
{
  double length = 0.0;
  double width = 0.0;
  double height = 0.0;
};

/// Ground-truth object reported by the simulator.
struct SimulatorObject
{
  /// Actor ID of the simulator.
  std::uint32_t id = 0;
  ObjectState state;
  Dimensions dimensions;
};

/// Description of a simulator actor.
struct ActorInfo
{
  std::uint32_t id = 0;
  /// Blueprint, e.g. "vehicle.tesla.model3".
  std::string type;
  /// Role name; cooperative vehicles are named "vehicle_<id>".
  std::string role_name;
};

enum class ObjectClass
{
  kCar,
  kTruck,
  kMotorcycle,
  kOther,
};

/// Wheel axle of a vehicle.
struct Axle
{
  bool single_track = false;
  double track_width = 0.0;
  double track_width_variance = 0.0;
  /// Signed longitudinal distance from the object center to the axle [m].
  double center_to_axle = 0.0;
  double center_to_axle_variance = 0.0;
  std::vector<bool> wheel_detected;
};

/// UFIL object derived from a simulator object.
struct Object
{
  std::uint32_t id = 0;
  ObjectState state;
  /// Variances of the state in the order x, y, v_x, v_y, a_x, a_y, yaw, yaw_rate.
  std::array<double, 8> state_variances{};
  Dimensions dimensions;
  /// Variances of length, width and height.
  std::array<double, 3> dimension_variances{};
  double existence_probability = 0.0;
  ObjectClass object_class = ObjectClass::kOther;
  std::vector<Axle> axles;
};

/// Rigid 2D transformation: rotation by yaw, then translation.
struct Pose2D
{
  double x = 0.0;
  double y = 0.0;
  double yaw = 0.0;
};

struct CarlaAdapterConfig
{
  /// Convert only the cooperative vehicles listed in cooperative_vehicle_ids.
  bool filter_cooperative_vehicles = false;
  /// IDs of the cooperative vehicles, i.e. actors with the role name "vehicle_<id>".
  std::vector<std::int64_t> cooperative_vehicle_ids;
};

/// @return an error message if the configuration is invalid.
UFIL_EXAMPLES_CARLA_ADAPTER_PUBLIC
std::optional<std::string> validate(const CarlaAdapterConfig & config);

/// @return the ID of a cooperative vehicle with the role name "vehicle_<id>", if it is one.
UFIL_EXAMPLES_CARLA_ADAPTER_PUBLIC
std::optional<std::int64_t> cooperativeVehicleId(const std::string & role_name);

/// @return the class of a simulator blueprint.
UFIL_EXAMPLES_CARLA_ADAPTER_PUBLIC
ObjectClass classify(const std::string & type);

/// @return the axles of a known simulator blueprint, empty for unknown blueprints.
UFIL_EXAMPLES_CARLA_ADAPTER_PUBLIC
std::vector<Axle> axleGeometry(const std::string & type);

/// @return the objects transformed from their frame by the pose of that frame in the target frame.
UFIL_EXAMPLES_CARLA_ADAPTER_PUBLIC
std::vector<Object> transformObjects(const std::vector<Object> & objects, const Pose2D & pose);

struct AdapterStatistics
{
  /// Received actor lists.
  std::uint64_t actor_lists = 0;
  /// Converted object arrays.
  std::uint64_t object_arrays = 0;
  /// Simulator objects of all converted arrays.
  std::uint64_t objects = 0;
  /// Converted objects.
  std::uint64_t converted_objects = 0;
  /// Objects skipped because they are no cooperative vehicle.
  std::uint64_t skipped_objects = 0;
  /// Objects without an entry in the actor list, classified as other.
  std::uint64_t unknown_actors = 0;
};

/**
 * @brief Converts ground-truth simulator objects into UFIL objects.
 *
 * Classification and axles are derived from the actor list. Simulator objects are ground
 * truth, so the converted objects get a fixed small uncertainty and existence probability 1.
 */
class UFIL_EXAMPLES_CARLA_ADAPTER_PUBLIC CarlaAdapter
{
public:
  /// @throws std::invalid_argument if the configuration is invalid, see validate().
  explicit CarlaAdapter(CarlaAdapterConfig config);

  /// Replaces the known actors.
  void updateActors(const std::vector<ActorInfo> & actors);

  /// @return true once an actor list was received.
  bool hasActors() const;

  /// Converts the objects of one simulator update and updates the statistics.
  std::vector<Object> convert(const std::vector<SimulatorObject> & objects);

  const CarlaAdapterConfig & config() const;
  const AdapterStatistics & statistics() const;

private:
  CarlaAdapterConfig config_;
  std::unordered_map<std::uint32_t, ActorInfo> actors_;
  bool has_actors_ = false;
  AdapterStatistics statistics_;
};

}  // namespace ufil_examples_carla_adapter

#endif  // UFIL_EXAMPLES_CARLA_ADAPTER__CARLA_ADAPTER_HPP_
