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

#ifndef UFIL_PCL_PREPROCESSOR__COMPONENT__UFILPCLPREPROCESSORPARAMETERROS_HPP_
#define UFIL_PCL_PREPROCESSOR__COMPONENT__UFILPCLPREPROCESSORPARAMETERROS_HPP_

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

#include "ufil_pcl_preprocessor/config.hpp"

namespace ufil_pcl_preprocessor
{

/// Settings only used by the ROS 2 node, fixed for the lifetime of the node.
struct NodeSettings
{
  /// Frame the output cloud, occlusion grid and field of view are expressed in.
  std::string target_frame = "sensor_node_base";
  /// Frame in which the field of view is estimated.
  std::string sensor_frame = "sensor_node";
  /// Stamp outputs with the node clock instead of the input stamp (for sensors without
  /// synchronised clocks).
  bool restamp_output = false;
  /// CSV file with the clipped (static) areas; empty disables clipping.
  std::string static_object_file;
};

/**
 * @brief Declares, validates and reads the parameters of the PCL preprocessor node.
 *
 * Defaults are taken from PreprocessorConfig and NodeSettings. The PreprocessorConfig
 * parameters can be changed at runtime; every change is validated as a whole against the
 * current configuration before it is accepted. NodeSettings parameters are read-only.
 */
class UfilPclPreprocessorParameterRos
{
public:
  using ConfigCallback = std::function<void (const PreprocessorConfig &)>;

  /// @throws std::invalid_argument if the initial parameter values are invalid.
  explicit UfilPclPreprocessorParameterRos(
    rclcpp::node_interfaces::NodeParametersInterface::SharedPtr parameters)
  : parameters_(std::move(parameters)),
    config_parameters_(configParameters())
  {
    declareSettings();

    const PreprocessorConfig defaults;
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

  UfilPclPreprocessorParameterRos(const UfilPclPreprocessorParameterRos &) = delete;
  UfilPclPreprocessorParameterRos & operator=(const UfilPclPreprocessorParameterRos &) = delete;

  ~UfilPclPreprocessorParameterRos()
  {
    parameters_->remove_on_set_parameters_callback(validate_handle_.get());
    parameters_->remove_post_set_parameters_callback(apply_handle_.get());
  }

  const PreprocessorConfig & config() const {return config_;}
  const NodeSettings & settings() const {return settings_;}

  /// Called after a runtime change of the configuration was accepted.
  void onConfigChanged(ConfigCallback callback) {config_changed_ = std::move(callback);}

private:
  struct ConfigParameter
  {
    std::string name;
    std::string description;
    std::function<rclcpp::ParameterValue(const PreprocessorConfig &)> get;
    std::function<void(const rclcpp::ParameterValue &, PreprocessorConfig &)> set;
  };

  template<typename Group, typename T>
  static ConfigParameter makeParameter(
    std::string name, std::string description, Group PreprocessorConfig::* group,
    T Group::* field)
  {
    return ConfigParameter{
      std::move(name), std::move(description),
      [group, field](const PreprocessorConfig & config) {
        return rclcpp::ParameterValue((config.*group).*field);
      },
      [group, field](const rclcpp::ParameterValue & value, PreprocessorConfig & config) {
        (config.*group).*field = static_cast<T>(value.get<T>());
      }};
  }

  static std::vector<ConfigParameter> configParameters()
  {
    using C = PreprocessorConfig;
    return {
      makeParameter(
        "filter.voxel_size", "Edge length of the voxel grid applied to the dynamic points [m].",
        &C::filter, &FilterConfig::voxel_size),
      makeParameter(
        "filter.outlier_radius", "Search radius of the radius outlier removal [m].",
        &C::filter, &FilterConfig::outlier_radius),
      makeParameter(
        "filter.min_neighbors",
        "Minimum number of neighbours within filter.outlier_radius to keep a point.",
        &C::filter, &FilterConfig::min_neighbors),
      makeParameter(
        "filter.z_min", "Points below this height in the target frame are removed [m].",
        &C::filter, &FilterConfig::z_min),
      makeParameter(
        "filter.z_max", "Points above this height in the target frame are removed [m].",
        &C::filter, &FilterConfig::z_max),
      makeParameter(
        "fov.num_bins", "Number of azimuth bins used to estimate the field of view.",
        &C::fov, &FovConfig::num_bins),
      makeParameter(
        "fov.history_length", "Number of per-cloud estimates averaged into the field of view.",
        &C::fov, &FovConfig::history_length),
      makeParameter(
        "fov.max_range", "Upper bound of the estimated field of view range [m].",
        &C::fov, &FovConfig::max_range),
      makeParameter(
        "occlusion_grid.min_range", "Static points closer than this are ignored [m].",
        &C::occlusion_grid, &OcclusionGridConfig::min_range),
      makeParameter(
        "occlusion_grid.max_range", "Static points farther than this are ignored [m].",
        &C::occlusion_grid, &OcclusionGridConfig::max_range),
      makeParameter(
        "occlusion_grid.resolution", "Cell size of the occlusion grid [m].",
        &C::occlusion_grid, &OcclusionGridConfig::resolution),
      makeParameter(
        "occlusion_grid.size_x", "Extent of the occlusion grid along x [m].",
        &C::occlusion_grid, &OcclusionGridConfig::size_x),
      makeParameter(
        "occlusion_grid.size_y", "Extent of the occlusion grid along y [m].",
        &C::occlusion_grid, &OcclusionGridConfig::size_y),
      makeParameter(
        "occlusion_grid.decay_rate",
        "Decay of unobserved cells per update in [0, 1]; 1 keeps cells, 0 clears them.",
        &C::occlusion_grid, &OcclusionGridConfig::decay_rate),
    };
  }

  void declareSettings()
  {
    const NodeSettings defaults;
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.read_only = true;

    descriptor.description = "Frame of the published cloud, occlusion grid and field of view.";
    settings_.target_frame = parameters_->declare_parameter(
      "target_frame", rclcpp::ParameterValue(defaults.target_frame), descriptor).get<std::string>();

    descriptor.description = "Sensor frame in which the field of view is estimated.";
    settings_.sensor_frame = parameters_->declare_parameter(
      "sensor_frame", rclcpp::ParameterValue(defaults.sensor_frame), descriptor).get<std::string>();

    descriptor.description =
      "Stamp outputs with the node clock instead of the input cloud stamp.";
    settings_.restamp_output = parameters_->declare_parameter(
      "restamp_output", rclcpp::ParameterValue(defaults.restamp_output), descriptor).get<bool>();

    descriptor.description =
      "CSV file (x, y, z, yaw, length, width, height) with static areas in the target frame. "
      "Empty disables clipping.";
    settings_.static_object_file = parameters_->declare_parameter(
      "static_object_file", rclcpp::ParameterValue(defaults.static_object_file),
      descriptor).get<std::string>();

    if (settings_.target_frame.empty() || settings_.sensor_frame.empty()) {
      throw std::invalid_argument("invalid parameters: target_frame and sensor_frame must be set");
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

    PreprocessorConfig candidate = config_;
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
  PreprocessorConfig config_;
  NodeSettings settings_;
  ConfigCallback config_changed_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr validate_handle_;
  rclcpp::node_interfaces::PostSetParametersCallbackHandle::SharedPtr apply_handle_;
};

}  // namespace ufil_pcl_preprocessor

#endif  // UFIL_PCL_PREPROCESSOR__COMPONENT__UFILPCLPREPROCESSORPARAMETERROS_HPP_
