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

#ifndef UFIL_OSN_TRACKER__COMPONENT__UFILOSNTRACKERPARAMETERROS_HPP_
#define UFIL_OSN_TRACKER__COMPONENT__UFILOSNTRACKERPARAMETERROS_HPP_

#include <functional>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <rclcpp/node_interfaces/node_parameters_interface.hpp>
#include <rclcpp/parameter.hpp>
#include <rclcpp/parameter_value.hpp>

#include "ufil_osn_tracker/config.hpp"

namespace ufil_osn_tracker
{

/// Settings only used by the ROS 2 node, fixed for the lifetime of the node.
struct NodeSettings
{
  /// Frame the tracks are published in.
  std::string target_frame = "map";
  /// Tracks with a lower existence probability are not published.
  double min_existence_probability = 0.2;
};

/**
 * @brief Declares, validates and reads the parameters of the OSN tracker node.
 *
 * Defaults are taken from TrackerConfig and NodeSettings. The TrackerConfig parameters
 * can be changed at runtime; every change is validated as a whole against the
 * current configuration before it is accepted. NodeSettings parameters are read-only.
 */
class UfilOsnTrackerParameterRos
{
public:
  using ConfigCallback = std::function<void (const TrackerConfig &)>;

  /// @throws std::invalid_argument if the initial parameter values are invalid.
  explicit UfilOsnTrackerParameterRos(
    rclcpp::node_interfaces::NodeParametersInterface::SharedPtr parameters)
  : parameters_(std::move(parameters)),
    config_parameters_(configParameters())
  {
    declareSettings();

    const TrackerConfig defaults;
    for (const auto & parameter : config_parameters_) {
      rcl_interfaces::msg::ParameterDescriptor descriptor;
      descriptor.description = parameter.description;
      const auto value =
        parameters_->declare_parameter(parameter.name, parameter.get(defaults), descriptor);
      parameter.set(value, config_);
    }
    if (const auto error = validate(config_)) {
      throw std::invalid_argument("invalid parameters: " + *error);
    }

    validate_handle_ = parameters_->add_on_set_parameters_callback(
      [this](const std::vector<rclcpp::Parameter> & parameters) {
        return validateChange(parameters);
      });
    apply_handle_ = parameters_->add_post_set_parameters_callback(
      [this](const std::vector<rclcpp::Parameter> & parameters) {
        applyChange(parameters);
      });
  }

  UfilOsnTrackerParameterRos(const UfilOsnTrackerParameterRos &) = delete;
  UfilOsnTrackerParameterRos & operator=(const UfilOsnTrackerParameterRos &) = delete;

  ~UfilOsnTrackerParameterRos()
  {
    parameters_->remove_on_set_parameters_callback(validate_handle_.get());
    parameters_->remove_post_set_parameters_callback(apply_handle_.get());
  }

  const TrackerConfig & config() const {return config_;}
  const NodeSettings & settings() const {return settings_;}

  /// Called after a runtime change of the configuration was accepted.
  void onConfigChanged(ConfigCallback callback) {config_changed_ = std::move(callback);}

private:
  struct ConfigParameter
  {
    std::string name;
    std::string description;
    std::function<rclcpp::ParameterValue(const TrackerConfig &)> get;
    std::function<void(const rclcpp::ParameterValue &, TrackerConfig &)> set;
  };

  template<typename T>
  static ConfigParameter makeParameter(
    std::string name, std::string description, T TrackerConfig::* field)
  {
    return ConfigParameter{
      std::move(name), std::move(description),
      [field](const TrackerConfig & config) {
        return rclcpp::ParameterValue(config.*field);
      },
      [field](const rclcpp::ParameterValue & value, TrackerConfig & config) {
        config.*field = static_cast<T>(value.get<T>());
      }};
  }

  static std::vector<ConfigParameter> configParameters()
  {
    using C = TrackerConfig;
    return {
      makeParameter(
        "association.threshold",
        "Maximum distance between a track and an associated measurement [m].",
        &C::association_threshold),
      makeParameter(
        "track.history_duration", "Time span of the history kept per track [s].",
        &C::history_duration),
      makeParameter(
        "clustering.min_points", "Minimum number of points of a cluster.",
        &C::cluster_min_points),
      makeParameter(
        "clustering.max_points", "Maximum number of points of a cluster.",
        &C::cluster_max_points),
      makeParameter(
        "clustering.tolerance", "Maximum distance between neighbouring cluster points [m].",
        &C::cluster_tolerance),
      makeParameter(
        "detector.dimension_covariance_multiplier",
        "Scaling of the dimension covariance of the measurements.",
        &C::dimension_covariance_multiplier),
      makeParameter(
        "detector.position_covariance_multiplier",
        "Scaling of the position covariance of the measurements.",
        &C::position_covariance_multiplier),
      makeParameter(
        "detector.orientation_covariance_multiplier",
        "Scaling of the orientation covariance of the measurements.",
        &C::orientation_covariance_multiplier),
    };
  }

  void declareSettings()
  {
    const NodeSettings defaults;
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.read_only = true;

    descriptor.description = "Frame the tracks are published in.";
    settings_.target_frame = parameters_->declare_parameter(
      "target_frame", rclcpp::ParameterValue(defaults.target_frame), descriptor).get<std::string>();

    descriptor.description = "Tracks with a lower existence probability are not published.";
    settings_.min_existence_probability = parameters_->declare_parameter(
      "min_existence_probability", rclcpp::ParameterValue(defaults.min_existence_probability),
      descriptor).get<double>();

    if (settings_.target_frame.empty()) {
      throw std::invalid_argument("invalid parameters: target_frame must be set");
    }
    if (!(settings_.min_existence_probability >= 0.0 &&
      settings_.min_existence_probability <= 1.0))
    {
      throw std::invalid_argument(
          "invalid parameters: min_existence_probability must be in [0, 1]");
    }
  }

  const ConfigParameter * find(const std::string & name) const
  {
    for (const auto & parameter : config_parameters_) {
      if (parameter.name == name) {
        return &parameter;
      }
    }
    return nullptr;
  }

  rcl_interfaces::msg::SetParametersResult validateChange(
    const std::vector<rclcpp::Parameter> & parameters) const
  {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;

    TrackerConfig candidate = config_;
    for (const auto & parameter : parameters) {
      if (const auto * binding = find(parameter.get_name())) {
        binding->set(parameter.get_parameter_value(), candidate);
      }
    }
    if (const auto error = validate(candidate)) {
      result.successful = false;
      result.reason = *error;
    }
    return result;
  }

  void applyChange(const std::vector<rclcpp::Parameter> & parameters)
  {
    bool changed = false;
    for (const auto & parameter : parameters) {
      if (const auto * binding = find(parameter.get_name())) {
        binding->set(parameter.get_parameter_value(), config_);
        changed = true;
      }
    }
    if (changed && config_changed_) {
      config_changed_(config_);
    }
  }

  rclcpp::node_interfaces::NodeParametersInterface::SharedPtr parameters_;
  const std::vector<ConfigParameter> config_parameters_;
  TrackerConfig config_;
  NodeSettings settings_;
  ConfigCallback config_changed_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr validate_handle_;
  rclcpp::node_interfaces::PostSetParametersCallbackHandle::SharedPtr apply_handle_;
};

}  // namespace ufil_osn_tracker

#endif  // UFIL_OSN_TRACKER__COMPONENT__UFILOSNTRACKERPARAMETERROS_HPP_
