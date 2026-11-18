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

#ifndef UFIL_VISUALIZATION__COMPONENT__OBJECTLISTMARKERARRAYPARAMETERROS_HPP_
#define UFIL_VISUALIZATION__COMPONENT__OBJECTLISTMARKERARRAYPARAMETERROS_HPP_

#include <algorithm>
#include <cctype>
#include <cstdint>
#include <set>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <rclcpp/node_interfaces/node_parameters_interface.hpp>

#include "parameter_helpers.hpp"
#include "ufil_visualization/object_markers.hpp"

namespace ufil_visualization
{

/// One visualized object list: its topics and the marker configuration with its colour.
struct ObjectListSource
{
  /// Name in the sources parameter; empty for the single default source.
  std::string name;
  std::string input_topic;
  std::string output_topic;
  ObjectMarkerConfig config;
};

/**
 * @brief Declares, validates and reads the parameters of the object list marker array node.
 *
 * All parameters are read-only. The node visualizes every object list named in sources; each
 * source <name> has the parameters <name>.input_topic, <name>.output_topic and <name>.color.
 * All other parameters are shared by the sources. Without sources, the node visualizes the
 * single object list object_list on object_markers with color; both topics can be remapped.
 */
class ObjectListMarkerArrayParameterRos
{
public:
  /// @throws std::invalid_argument if the parameters are invalid.
  explicit ObjectListMarkerArrayParameterRos(
    rclcpp::node_interfaces::NodeParametersInterface::SharedPtr parameters)
  : parameters_(std::move(parameters))
  {
    using parameters::declare;
    using parameters::declareColor;
    auto & p = *parameters_;
    const ObjectMarkerConfig defaults;

    config_.marker_lifetime = declare(
      p, "marker_lifetime", defaults.marker_lifetime,
      "Seconds until the markers disappear if no new object list arrives, 0 for never.");

    config_.color = declareColor(
      p, "color", defaults.color,
      "Colour of all markers, typically one per source (e.g. OSN, OBU)");
    config_.body_alpha = declare(
      p, "body.alpha", defaults.body_alpha,
      "Opacity of the object boxes relative to the alpha of color, in [0, 1].");

    config_.velocity_shaft_diameter = declare(
      p, "velocity.shaft_diameter", defaults.velocity_shaft_diameter,
      "Shaft diameter of the velocity arrows [m].");
    config_.velocity_head_diameter = declare(
      p, "velocity.head_diameter", defaults.velocity_head_diameter,
      "Head diameter of the velocity arrows [m].");
    config_.velocity_head_length = declare(
      p, "velocity.head_length", defaults.velocity_head_length,
      "Head length of the velocity arrows [m].");

    config_.axis_shaft_diameter = declare(
      p, "body_axes.shaft_diameter", defaults.axis_shaft_diameter,
      "Shaft diameter of the object axis arrows [m].");
    config_.axis_head_diameter = declare(
      p, "body_axes.head_diameter", defaults.axis_head_diameter,
      "Head diameter of the object axis arrows [m].");
    config_.axis_head_length = declare(
      p, "body_axes.head_length", defaults.axis_head_length,
      "Head length of the object axis arrows [m].");

    config_.covariance_line_width = declare(
      p, "covariance.line_width", defaults.covariance_line_width,
      "Line width of the covariance ellipses [m].");
    config_.ellipse_segments = static_cast<int>(declare<std::int64_t>(
        p, "covariance.ellipse_segments", defaults.ellipse_segments,
        "Line segments per covariance ellipse, in [3, 1024]."));

    config_.label_text_size = declare(
      p, "label.text_size", defaults.label_text_size, "Height of the text labels [m].");
    config_.label_icons = declare(
      p, "label.icons", defaults.label_icons,
      "Show the class in the labels as an icon (emoji) instead of its name.");

    if (const auto error = validate(config_)) {
      throw std::invalid_argument("invalid parameters: " + *error);
    }

    const auto names = declare(
      p, "sources", std::vector<std::string>{},
      "Names of the visualized object lists, each configured by <name>.input_topic, "
      "<name>.output_topic and <name>.color. Empty: object_list on object_markers.");
    if (names.empty()) {
      sources_.push_back({"", "object_list", "object_markers", config_});
    }
    std::set<std::string> unique_names;
    for (const auto & name : names) {
      if (!unique_names.insert(name).second) {
        throw std::invalid_argument("invalid parameters: source " + name + " repeated");
      }
      sources_.push_back(declareSource(p, name));
    }
    validateOutputTopics();
  }

  /// Shared configuration; the colour is the default of the sources.
  const ObjectMarkerConfig & config() const {return config_;}

  const std::vector<ObjectListSource> & sources() const {return sources_;}

private:
  ObjectListSource declareSource(
    rclcpp::node_interfaces::NodeParametersInterface & p, const std::string & name) const
  {
    using parameters::declare;
    using parameters::declareColor;
    static const std::set<std::string> reserved{
      "sources", "marker_lifetime", "color", "body", "velocity", "body_axes", "covariance",
      "label"};
    const bool valid_name = !name.empty() &&
      std::isalpha(static_cast<unsigned char>(name.front())) &&
      std::all_of(
      name.begin(), name.end(), [](char c) {
        return std::isalnum(static_cast<unsigned char>(c)) || c == '_';
      });
    if (!valid_name || reserved.count(name)) {
      throw std::invalid_argument(
              "invalid parameters: source name '" + name +
              "' must start with a letter, contain only letters, digits and underscores and "
              "must not be the name of another parameter");
    }

    ObjectListSource source;
    source.name = name;
    source.input_topic = declare(
      p, name + ".input_topic", name + "/object_list",
      "Visualized ufil_msgs/ObjectList of the source " + name + ".");
    source.output_topic = declare(
      p, name + ".output_topic", name + "/object_markers",
      "Published markers of the source " + name + ".");
    source.config = config_;
    source.config.color = declareColor(
      p, name + ".color", config_.color, "Colour of all markers of the source " + name);
    if (source.input_topic.empty() || source.output_topic.empty()) {
      throw std::invalid_argument("invalid parameters: topics of " + name + " must not be empty");
    }
    if (const auto error = validate(source.config)) {
      throw std::invalid_argument("invalid parameters: " + name + "." + *error);
    }
    return source;
  }

  /// Every source needs its own output topic: the markers of a source replace all previous
  /// markers on its output topic. The node also compares the resolved topic names.
  void validateOutputTopics() const
  {
    std::set<std::string> output_topics;
    for (const auto & source : sources_) {
      if (!output_topics.insert(source.output_topic).second) {
        throw std::invalid_argument(
                "invalid parameters: output topic " + source.output_topic +
                " used by several sources");
      }
    }
  }

  rclcpp::node_interfaces::NodeParametersInterface::SharedPtr parameters_;
  ObjectMarkerConfig config_;
  std::vector<ObjectListSource> sources_;
};

}  // namespace ufil_visualization

#endif  // UFIL_VISUALIZATION__COMPONENT__OBJECTLISTMARKERARRAYPARAMETERROS_HPP_
