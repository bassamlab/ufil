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

#ifndef SSL_VEHICLE_TRACKING__COMPONENT__SSLVEHICLETRACKINGPARAMETERROS_HPP_
#define SSL_VEHICLE_TRACKING__COMPONENT__SSLVEHICLETRACKINGPARAMETERROS_HPP_

#include <cmath>
#include <cstdint>
#include <filesystem>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <rclcpp/node_interfaces/node_parameters_interface.hpp>
#include <rclcpp/parameter_value.hpp>

#include "ssl_vehicle_tracking/vehicle_template.hpp"
#include "ssl_vehicle_tracking/vehicle_tracker.hpp"

namespace ssl_vehicle_tracking
{

/// Settings only used by the ROS 2 node.
struct NodeSettings
{
  /// Only vehicles with a higher existence probability are published [0, 1].
  double publishing_threshold = 0.0;
};

/**
 * @brief Declares, validates and reads the parameters of the vehicle tracking node and loads
 *        the vehicle templates.
 *
 * All parameters are read-only: changing them requires restarting the node.
 */
class SslVehicleTrackingParameterRos
{
public:
  /// @throws std::invalid_argument if the parameters are invalid or a template cannot be loaded.
  explicit SslVehicleTrackingParameterRos(
    rclcpp::node_interfaces::NodeParametersInterface::SharedPtr parameters)
  : parameters_(std::move(parameters))
  {
    declareTemplates();
    declareTracking();
  }

  const NodeSettings & settings() const {return settings_;}
  const TrackerConfig & tracker() const {return tracker_;}
  const std::vector<VehicleTemplate> & templates() const {return templates_;}
  /// Resolved paths of the loaded templates.
  const std::vector<std::string> & templatePaths() const {return template_paths_;}

  /// A template given by file name only is taken from the templates directory of the package.
  static std::string resolveTemplatePath(const std::string & name)
  {
    if (name.find('/') != std::string::npos) {
      return name;
    }
    return ament_index_cpp::get_package_share_directory("ssl_vehicle_tracking") +
           "/templates/" + name;
  }

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

  void declareTemplates()
  {
    const auto names = declare(
      "vehicle_templates", std::vector<std::string>{"muCar_scaled_lab.json"},
      "Vehicle template files (JSON); a file name without directory is taken from the templates "
      "directory of ssl_vehicle_tracking.");
    if (names.empty()) {
      throw std::invalid_argument("invalid parameters: vehicle_templates must not be empty");
    }
    for (const auto & name : names) {
      const std::string path = resolveTemplatePath(name);
      try {
        templates_.push_back(loadTemplate(path, std::filesystem::path(path).stem().string()));
      } catch (const std::runtime_error & error) {
        throw std::invalid_argument("invalid parameters: vehicle_templates: " +
                std::string(error.what()));
      }
      template_paths_.push_back(path);
    }
  }

  void declareTracking()
  {
    const TrackerConfig defaults;
    tracker_.search_width = declare(
      "search_width", defaults.search_width,
      "Maximum deviation of a wheel track from its expected position in the vehicle [m].");
    const auto pairs = declare(
      "considered_rotation_pairs", static_cast<std::int64_t>(defaults.considered_rotation_pairs),
      "Number of wheel pairs (longest first) used to estimate the vehicle rotation.");
    if (pairs < 1) {
      throw std::invalid_argument("invalid parameters: considered_rotation_pairs must be >= 1");
    }
    tracker_.considered_rotation_pairs = static_cast<std::size_t>(pairs);
    tracker_.dropping_threshold = declare(
      "dropping_threshold", defaults.dropping_threshold,
      "Wheels and vehicles whose probability falls below this value are dropped [0, 1].");
    settings_.publishing_threshold = declare(
      "publishing_threshold", settings_.publishing_threshold,
      "Only vehicles with a higher existence probability are published [0, 1].");

    if (const auto error = validate(tracker_)) {
      throw std::invalid_argument("invalid parameters: " + *error);
    }
    if (!(settings_.publishing_threshold >= 0.0 && settings_.publishing_threshold <= 1.0)) {
      throw std::invalid_argument("invalid parameters: publishing_threshold must be in [0, 1]");
    }
  }

  rclcpp::node_interfaces::NodeParametersInterface::SharedPtr parameters_;
  NodeSettings settings_;
  TrackerConfig tracker_;
  std::vector<VehicleTemplate> templates_;
  std::vector<std::string> template_paths_;
};

}  // namespace ssl_vehicle_tracking

#endif  // SSL_VEHICLE_TRACKING__COMPONENT__SSLVEHICLETRACKINGPARAMETERROS_HPP_
