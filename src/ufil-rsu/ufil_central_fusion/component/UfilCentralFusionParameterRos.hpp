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

#ifndef UFIL_CENTRAL_FUSION__COMPONENT__UFILCENTRALFUSIONPARAMETERROS_HPP_
#define UFIL_CENTRAL_FUSION__COMPONENT__UFILCENTRALFUSIONPARAMETERROS_HPP_

#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <rclcpp/node_interfaces/node_parameters_interface.hpp>
#include <rclcpp/parameter_value.hpp>

#include "ufil_central_fusion/config.hpp"

namespace ufil_central_fusion
{

/// Settings only used by the ROS 2 node.
struct NodeSettings
{
  /// Frame the inputs are transformed into and the fused objects are published in.
  std::string target_frame = "map";
  /// Period of the fusion and publication [s].
  double period = 0.02;
  /// Fused tracks with a lower existence probability are not published.
  double min_existence_probability = 0.2;
};

/// Input topics of a sensor.
struct SensorTopics
{
  std::string objects;
  /// Optional, empty if the sensor does not publish a field of view.
  std::string fov;
  /// Optional, empty if the sensor does not publish an occlusion grid.
  std::string occlusion;
};

/**
 * @brief Declares, validates and reads the parameters of the central fusion node.
 *
 * The sensors are listed in sensor_names; each sensor <name> has the parameters <name>.topic,
 * <name>.fov_topic, <name>.occlusion_topic, <name>.is_cam and <name>.trust.*. All parameters are
 * read-only: changing the fusion set-up requires restarting the node.
 */
class UfilCentralFusionParameterRos
{
public:
  /// @throws std::invalid_argument if the parameters are invalid.
  explicit UfilCentralFusionParameterRos(
    rclcpp::node_interfaces::NodeParametersInterface::SharedPtr parameters)
  : parameters_(std::move(parameters))
  {
    declareSettings();
    declareFusion();
    declareSensors();

    if (const auto error = validate(config_)) {
      throw std::invalid_argument("invalid parameters: " + *error);
    }
    if (const auto error = validate(sensors_)) {
      throw std::invalid_argument("invalid parameters: " + *error);
    }
  }

  const NodeSettings & settings() const {return settings_;}
  const FusionConfig & config() const {return config_;}
  const std::vector<SensorConfig> & sensors() const {return sensors_;}
  /// In the order of sensors().
  const std::vector<SensorTopics> & topics() const {return topics_;}

private:
  template<typename T>
  T declare(const std::string & name, const T & default_value, const std::string & description)
  {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.description = description;
    descriptor.read_only = true;
    return parameters_->declare_parameter(
      name, rclcpp::ParameterValue(default_value), descriptor).get<T>();
  }

  void declareSettings()
  {
    const NodeSettings defaults;
    settings_.target_frame = declare(
      "target_frame", defaults.target_frame,
      "Frame the inputs are transformed into and the fused objects are published in.");
    settings_.period = declare(
      "fusion.period", defaults.period, "Period of the fusion and publication [s].");
    settings_.min_existence_probability = declare(
      "output.min_existence_probability", defaults.min_existence_probability,
      "Fused tracks with a lower existence probability are not published.");

    if (settings_.target_frame.empty()) {
      throw std::invalid_argument("invalid parameters: target_frame must be set");
    }
    if (!(settings_.period > 0.0)) {
      throw std::invalid_argument("invalid parameters: fusion.period must be > 0");
    }
    if (!(settings_.min_existence_probability >= 0.0 &&
      settings_.min_existence_probability <= 1.0))
    {
      throw std::invalid_argument(
              "invalid parameters: output.min_existence_probability must be in [0, 1]");
    }
  }

  void declareFusion()
  {
    const FusionConfig defaults;
    config_.buffer_duration = declare(
      "fusion.buffer_duration", defaults.buffer_duration,
      "Measurements are buffered this long to fuse late arriving data in order [s].");
    config_.association_threshold = declare(
      "association.threshold", defaults.association_threshold,
      "Maximum distance between a track and an associated measurement [m].");
    config_.existence_min_weight = declare(
      "existence.min_weight", defaults.existence_min_weight,
      "Minimum weight of a measurement in the existence fusion.");
    config_.existence_max_weight = declare(
      "existence.max_weight", defaults.existence_max_weight,
      "Maximum weight of a measurement in the existence fusion.");
    config_.existence_decay_factor = declare(
      "existence.decay_factor", defaults.existence_decay_factor,
      "Decay of the existence probability of unobserved tracks per update.");
    config_.dimension_grid_resolution = declare(
      "dimension.grid_resolution", defaults.dimension_grid_resolution,
      "Cell size of the dimension grid map [m].");
    config_.dimension_alpha = declare(
      "dimension.alpha", defaults.dimension_alpha, "Update factor of the dimension grid map.");
    config_.dimension_p_min = declare(
      "dimension.p_min", defaults.dimension_p_min, "Minimum cell probability of the grid map.");
    config_.dimension_p_ref = declare(
      "dimension.p_ref", defaults.dimension_p_ref,
      "Cell probability above which a cell belongs to the object.");
    config_.dimension_p_max = declare(
      "dimension.p_max", defaults.dimension_p_max, "Maximum cell probability of the grid map.");
  }

  void declareSensors()
  {
    const auto names = declare(
      "sensor_names", std::vector<std::string>{},
      "Names of the fused sensors; each one is configured by <name>.* parameters.");

    const SensorTrust trust;
    for (const auto & name : names) {
      SensorConfig sensor;
      SensorTopics topics;
      sensor.name = name;
      topics.objects = declare(name + ".topic", std::string(), "Object list of the sensor.");
      topics.fov = declare(
        name + ".fov_topic", std::string(), "Field of view of the sensor, empty if none.");
      topics.occlusion = declare(
        name + ".occlusion_topic", std::string(),
        "Occlusion grid of the sensor, empty if none.");
      sensor.is_cam = declare(
        name + ".is_cam", false, "The objects stem from received CAMs.");

      const std::string prefix = name + ".trust.";
      const std::string description = "Trust in the sensor's estimate, in [0, 1].";
      sensor.trust.existence = declare(prefix + "existence", trust.existence, description);
      sensor.trust.car = declare(prefix + "car", trust.car, description);
      sensor.trust.truck = declare(prefix + "truck", trust.truck, description);
      sensor.trust.motorcycle = declare(prefix + "motorcycle", trust.motorcycle, description);
      sensor.trust.bicycle = declare(prefix + "bicycle", trust.bicycle, description);
      sensor.trust.pedestrian = declare(prefix + "pedestrian", trust.pedestrian, description);
      sensor.trust.stationary = declare(prefix + "stationary", trust.stationary, description);
      sensor.trust.other = declare(prefix + "other", trust.other, description);

      if (topics.objects.empty()) {
        throw std::invalid_argument("invalid parameters: " + name + ".topic must be set");
      }
      sensors_.push_back(sensor);
      topics_.push_back(topics);
    }
  }

  rclcpp::node_interfaces::NodeParametersInterface::SharedPtr parameters_;
  NodeSettings settings_;
  FusionConfig config_;
  std::vector<SensorConfig> sensors_;
  std::vector<SensorTopics> topics_;
};

}  // namespace ufil_central_fusion

#endif  // UFIL_CENTRAL_FUSION__COMPONENT__UFILCENTRALFUSIONPARAMETERROS_HPP_
