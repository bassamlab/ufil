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

#ifndef UFIL_VISUALIZATION__COMPONENT__LANELETMAPMARKERARRAYPARAMETERROS_HPP_
#define UFIL_VISUALIZATION__COMPONENT__LANELETMAPMARKERARRAYPARAMETERROS_HPP_

#include <stdexcept>
#include <string>
#include <utility>

#include <rclcpp/node_interfaces/node_parameters_interface.hpp>

#include "parameter_helpers.hpp"
#include "ufil_visualization/lanelet_map_markers.hpp"

namespace ufil_visualization
{

/// Settings only used by the ROS 2 node.
struct LaneletMapSettings
{
  /// Lanelet2 OSM map, empty for none.
  std::string map_path;
  /// Frame of the published markers.
  std::string frame_id = "map";
};

/**
 * @brief Declares, validates and reads the parameters of the lanelet map marker array node.
 *
 * All parameters are read-only: the markers are built once when the node starts.
 */
class LaneletMapMarkerArrayParameterRos
{
public:
  /// @throws std::invalid_argument if the parameters are invalid.
  explicit LaneletMapMarkerArrayParameterRos(
    rclcpp::node_interfaces::NodeParametersInterface::SharedPtr parameters)
  : parameters_(std::move(parameters))
  {
    declareSettings();
    declareStyle();
    if (const auto error = validate(style_)) {
      throw std::invalid_argument("invalid parameters: " + *error);
    }
  }

  const LaneletMapSettings & settings() const {return settings_;}
  const LaneletMapStyle & style() const {return style_;}

private:
  void declareSettings()
  {
    using parameters::declare;
    auto & p = *parameters_;
    const LaneletMapSettings defaults;
    settings_.map_path = declare(
      p, "map_path", defaults.map_path,
      "Lanelet2 OSM map to visualize. If empty, no map is published.");
    settings_.frame_id = declare(
      p, "frame_id", defaults.frame_id,
      "Frame of the published markers, i.e. of the local UTM coordinates around the map "
      "origin.");
    if (settings_.frame_id.empty()) {
      throw std::invalid_argument("invalid parameters: frame_id must be set");
    }
  }

  void declareStyle()
  {
    using parameters::declare;
    using parameters::declareColor;
    auto & p = *parameters_;
    const LaneletMapStyle d;
    auto & s = style_;

    s.line_width = declare(
      p, "line_width", d.line_width,
      "Width of lanelet borders, lanelet separators and road markings [m].");

    s.lanelet_fill_color = declareColor(
      p, "lanelet.fill_color", d.lanelet_fill_color, "Fill colour of the lanelets");
    s.lanelet_fill_z = declare(
      p, "lanelet.fill_z", d.lanelet_fill_z, "Height of the lanelet fill [m].");
    s.lanelet_border_color = declareColor(
      p, "lanelet.border_color", d.lanelet_border_color, "Colour of the lanelet borders");
    s.lanelet_border_z = declare(
      p, "lanelet.border_z", d.lanelet_border_z, "Height of the lanelet borders [m].");
    s.lanelet_separator_color = declareColor(
      p, "lanelet.separator_color", d.lanelet_separator_color,
      "Colour of the lines where a lanelet ends and its successor starts");
    s.lanelet_separator_z = declare(
      p, "lanelet.separator_z", d.lanelet_separator_z,
      "Height of the lanelet separators [m].");

    s.show_lanelet_ids = declare(
      p, "lanelet_ids.show", d.show_lanelet_ids, "Show the id of every lanelet.");
    s.lanelet_id_color = declareColor(
      p, "lanelet_ids.color", d.lanelet_id_color, "Colour of the lanelet ids");
    s.lanelet_id_text_size = declare(
      p, "lanelet_ids.text_size", d.lanelet_id_text_size, "Height of the lanelet ids [m].");
    s.lanelet_id_z = declare(
      p, "lanelet_ids.z", d.lanelet_id_z, "Height above ground of the lanelet ids [m].");

    s.road_marking_color = declareColor(
      p, "road_marking.color", d.road_marking_color,
      "Colour of solid line_thin / line_thick road markings and stop lines");
    s.road_marking_z = declare(
      p, "road_marking.z", d.road_marking_z, "Height of the road markings [m].");
    s.stop_line_width = declare(
      p, "road_marking.stop_line_width", d.stop_line_width, "Width of the stop lines [m].");

    s.crosswalk_fill_color = declareColor(
      p, "crosswalk.fill_color", d.crosswalk_fill_color, "Fill colour of crosswalk lanelets");
    s.crosswalk_fill_z = declare(
      p, "crosswalk.fill_z", d.crosswalk_fill_z, "Height of the crosswalk fill [m].");
    s.crosswalk_stripe_color = declareColor(
      p, "crosswalk.stripe_color", d.crosswalk_stripe_color, "Colour of the zebra stripes");
    s.crosswalk_stripe_z = declare(
      p, "crosswalk.stripe_z", d.crosswalk_stripe_z, "Height of the zebra stripes [m].");
    s.crosswalk_stripe_width = declare(
      p, "crosswalk.stripe_width", d.crosswalk_stripe_width,
      "Width of a zebra stripe along the walking direction [m].");
    s.crosswalk_stripe_gap = declare(
      p, "crosswalk.stripe_gap", d.crosswalk_stripe_gap, "Gap between zebra stripes [m].");

    s.area_z = declare(p, "area.z", d.area_z, "Height of the area fills [m].");
    s.area_default_color = declareColor(
      p, "area.default_color", d.area_default_color,
      "Fill colour of areas without or with another subtype");
    s.area_walkway_color = declareColor(
      p, "area.walkway_color", d.area_walkway_color, "Fill colour of walkway areas");
    s.area_vegetation_color = declareColor(
      p, "area.vegetation_color", d.area_vegetation_color, "Fill colour of vegetation areas");
    s.area_parking_color = declareColor(
      p, "area.parking_color", d.area_parking_color, "Fill colour of parking areas");
    s.area_keepout_color = declareColor(
      p, "area.keepout_color", d.area_keepout_color, "Fill colour of keepout areas");
    s.area_building_color = declareColor(
      p, "area.building_color", d.area_building_color, "Fill colour of building areas");
    s.area_traffic_island_color = declareColor(
      p, "area.traffic_island_color", d.area_traffic_island_color,
      "Fill colour of traffic island areas");
  }

  rclcpp::node_interfaces::NodeParametersInterface::SharedPtr parameters_;
  LaneletMapSettings settings_;
  LaneletMapStyle style_;
};

}  // namespace ufil_visualization

#endif  // UFIL_VISUALIZATION__COMPONENT__LANELETMAPMARKERARRAYPARAMETERROS_HPP_
