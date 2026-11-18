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

#ifndef UFIL_OBU_OBJECT_AGGREGATION__COMPONENT__UFILOBUOBJECTAGGREGATIONPARAMETERROS_HPP_
#define UFIL_OBU_OBJECT_AGGREGATION__COMPONENT__UFILOBUOBJECTAGGREGATIONPARAMETERROS_HPP_

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

#include "ufil_obu_object_aggregation/config.hpp"

namespace ufil_obu_object_aggregation
{

/**
 * @brief Declares, validates and reads the parameters of the object aggregation node.
 *
 * Defaults are taken from AggregatorConfig. The parameters can be changed at runtime; every
 * change is validated as a whole against the current configuration before it is accepted.
 */
class UfilObuObjectAggregationParameterRos
{
public:
  using ConfigCallback = std::function<void (const AggregatorConfig &)>;

  /// @throws std::invalid_argument if the initial parameter values are invalid.
  explicit UfilObuObjectAggregationParameterRos(
    rclcpp::node_interfaces::NodeParametersInterface::SharedPtr parameters)
  : parameters_(std::move(parameters)),
    config_parameters_(configParameters())
  {
    const AggregatorConfig defaults;
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

  UfilObuObjectAggregationParameterRos(const UfilObuObjectAggregationParameterRos &) = delete;
  UfilObuObjectAggregationParameterRos & operator=(
    const UfilObuObjectAggregationParameterRos &) = delete;

  ~UfilObuObjectAggregationParameterRos()
  {
    parameters_->remove_on_set_parameters_callback(validate_handle_.get());
    parameters_->remove_post_set_parameters_callback(apply_handle_.get());
  }

  const AggregatorConfig & config() const {return config_;}

  /// Called after a runtime change of the configuration was accepted.
  void onConfigChanged(ConfigCallback callback) {config_changed_ = std::move(callback);}

private:
  struct ConfigParameter
  {
    std::string name;
    std::string description;
    std::function<rclcpp::ParameterValue(const AggregatorConfig &)> get;
    std::function<void(const rclcpp::ParameterValue &, AggregatorConfig &)> set;
  };

  template<typename T>
  static ConfigParameter makeParameter(
    std::string name, std::string description, T AggregatorConfig::* field)
  {
    return ConfigParameter{
      std::move(name), std::move(description),
      [field](const AggregatorConfig & config) {
        return rclcpp::ParameterValue(config.*field);
      },
      [field](const rclcpp::ParameterValue & value, AggregatorConfig & config) {
        config.*field = static_cast<T>(value.get<T>());
      }};
  }

  static std::vector<ConfigParameter> configParameters()
  {
    using C = AggregatorConfig;
    return {
      makeParameter("object.length", "Length of the own vehicle [m].", &C::object_length),
      makeParameter("object.width", "Width of the own vehicle [m].", &C::object_width),
      makeParameter(
        "object.classification",
        "Class of the own vehicle as index of ufil_msgs/Classification (0 = CAR ... 6 = OTHER).",
        &C::object_class),
      makeParameter(
        "utm_zone",
        "UTM zone of the published position (1 to 60), -1 selects the zone from the position.",
        &C::utm_zone),
    };
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

    AggregatorConfig candidate = config_;
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
  AggregatorConfig config_;
  ConfigCallback config_changed_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr validate_handle_;
  rclcpp::node_interfaces::PostSetParametersCallbackHandle::SharedPtr apply_handle_;
};

}  // namespace ufil_obu_object_aggregation

#endif  // UFIL_OBU_OBJECT_AGGREGATION__COMPONENT__UFILOBUOBJECTAGGREGATIONPARAMETERROS_HPP_
