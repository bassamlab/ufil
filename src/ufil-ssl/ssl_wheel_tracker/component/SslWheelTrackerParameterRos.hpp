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

#ifndef SSL_WHEEL_TRACKER__COMPONENT__SSLWHEELTRACKERPARAMETERROS_HPP_
#define SSL_WHEEL_TRACKER__COMPONENT__SSLWHEELTRACKERPARAMETERROS_HPP_

#include <cmath>
#include <cstdint>
#include <set>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <rclcpp/node_interfaces/node_parameters_interface.hpp>
#include <rclcpp/parameter_value.hpp>

#include "ssl_wheel_tracker/config.hpp"

namespace ssl_wheel_tracker
{

/// Settings only used by the ROS 2 node.
struct NodeSettings
{
  /// Pressure grids (nav_msgs/OccupancyGrid) of the mats.
  std::vector<std::string> input_topics{"ssl_1/map"};
  /// Frame in which the wheels are tracked and published.
  std::string tracking_frame = "map";
  /// Period in which buffered frames are tracked and the tracks are published [s].
  double update_period = 0.01;
};

/**
 * @brief Declares, validates and reads the parameters of the wheel tracker node.
 *
 * All parameters are read-only: changing them requires restarting the node.
 */
class SslWheelTrackerParameterRos
{
public:
  /// @throws std::invalid_argument if the parameters are invalid.
  explicit SslWheelTrackerParameterRos(
    rclcpp::node_interfaces::NodeParametersInterface::SharedPtr parameters)
  : parameters_(std::move(parameters))
  {
    declareSettings();
    declareDetection();
    declareTracking();

    if (const auto error = validate(detector_)) {
      throw std::invalid_argument("invalid parameters: detection." + *error);
    }
    if (const auto error = validate(tracker_)) {
      throw std::invalid_argument("invalid parameters: tracking." + *error);
    }
  }

  const NodeSettings & settings() const {return settings_;}
  const DetectorConfig & detector() const {return detector_;}
  const TrackerConfig & tracker() const {return tracker_;}

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

  /// Declares an integer parameter that must not be negative.
  std::size_t declareCount(
    const std::string & name, std::size_t default_value, const std::string & description)
  {
    const auto value = declare(name, static_cast<std::int64_t>(default_value), description);
    if (value < 0) {
      throw std::invalid_argument("invalid parameters: " + name + " must be >= 0");
    }
    return static_cast<std::size_t>(value);
  }

  void declareSettings()
  {
    const NodeSettings defaults;
    settings_.input_topics = declare(
      "input_topics", defaults.input_topics,
      "Pressure grids (nav_msgs/OccupancyGrid) of the mats whose wheels are tracked together.");
    settings_.tracking_frame = declare(
      "tracking_frame", defaults.tracking_frame,
      "Frame in which the wheels are tracked and published.");
    settings_.update_period = declare(
      "update_period", defaults.update_period,
      "Period in which received frames are tracked in time order and published [s].");

    if (settings_.input_topics.empty()) {
      throw std::invalid_argument("invalid parameters: input_topics must not be empty");
    }
    const std::set<std::string> unique(
      settings_.input_topics.begin(), settings_.input_topics.end());
    if (unique.size() != settings_.input_topics.size() || unique.count("")) {
      throw std::invalid_argument(
              "invalid parameters: input_topics must be unique and not empty");
    }
    if (settings_.tracking_frame.empty()) {
      throw std::invalid_argument("invalid parameters: tracking_frame must not be empty");
    }
    if (!(std::isfinite(settings_.update_period) && settings_.update_period > 0.0)) {
      throw std::invalid_argument("invalid parameters: update_period must be > 0");
    }
  }

  void declareDetection()
  {
    const DetectorConfig defaults;
    const auto threshold = declare(
      "detection.threshold", static_cast<std::int64_t>(defaults.threshold),
      "Cells must exceed this pressure to be a local maximum [0, 254].");
    if (threshold < 0 || threshold > 254) {
      throw std::invalid_argument("invalid parameters: detection.threshold must be in [0, 254]");
    }
    detector_.threshold = static_cast<int>(threshold);
    detector_.min_peak_distance = declareCount(
      "detection.min_peak_distance", defaults.min_peak_distance,
      "Radius of the square window in which a local maximum is the largest value [cells].");
    detector_.contact_radius = declareCount(
      "detection.contact_radius", defaults.contact_radius,
      "Cells within this L1 distance of a maximum belong to its wheel contact [cells].");
    detector_.exclude_border = declareCount(
      "detection.exclude_border", defaults.exclude_border,
      "Local maxima within this distance of the grid border are ignored [cells].");
  }

  void declareTracking()
  {
    const TrackerConfig defaults;
    tracker_.process_noise = declare(
      "tracking.process_noise", defaults.process_noise,
      "Spectral density of the acceleration noise of the constant velocity model [m^2/s^3].");
    tracker_.measurement_noise = declare(
      "tracking.measurement_noise", defaults.measurement_noise,
      "Variance of the detected wheel position per axis [m^2].");
    tracker_.gate_distance = declare(
      "tracking.gate_distance", defaults.gate_distance,
      "Maximum distance of a detection to the predicted wheel position [m].");
    tracker_.confirmation_hits = declareCount(
      "tracking.confirmation_hits", defaults.confirmation_hits,
      "Associated detections, including the first, before a track is published.");
    tracker_.deletion_timeout = declare(
      "tracking.deletion_timeout", defaults.deletion_timeout,
      "Tracks are deleted when no detection was associated for longer than this [s].");
    tracker_.history_length = declareCount(
      "tracking.history_length", defaults.history_length,
      "Matched states published per track.");
  }

  rclcpp::node_interfaces::NodeParametersInterface::SharedPtr parameters_;
  NodeSettings settings_;
  DetectorConfig detector_;
  TrackerConfig tracker_;
};

}  // namespace ssl_wheel_tracker

#endif  // SSL_WHEEL_TRACKER__COMPONENT__SSLWHEELTRACKERPARAMETERROS_HPP_
