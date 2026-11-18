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

#ifndef UFIL_VISUALIZATION__LANELET_MAP_MARKERS_HPP_
#define UFIL_VISUALIZATION__LANELET_MAP_MARKERS_HPP_

#include <lanelet2_core/LaneletMap.h>

#include <optional>
#include <string>
#include <vector>

#include "ufil_visualization/marker.hpp"
#include "ufil_visualization/visibility_control.hpp"

namespace ufil_visualization
{

/// Appearance of the lanelet map markers. The default palette mimics a dark map style.
struct LaneletMapStyle
{
  /// Width of lanelet borders, lanelet separators and road markings [m].
  double line_width = 0.1;

  Color lanelet_fill_color{0.290F, 0.290F, 0.306F, 1.0F};
  double lanelet_fill_z = -0.10;
  Color lanelet_border_color{0.408F, 0.408F, 0.427F, 1.0F};
  double lanelet_border_z = -0.05;
  /// Drawn where a lanelet ends and its successor starts.
  Color lanelet_separator_color{0.039F, 0.518F, 1.0F, 1.0F};
  double lanelet_separator_z = 0.1;

  bool show_lanelet_ids = true;
  Color lanelet_id_color{0.922F, 0.922F, 0.961F, 0.8F};
  double lanelet_id_text_size = 0.8;
  double lanelet_id_z = 0.2;

  /// Solid line_thin / line_thick line strings and stop lines.
  Color road_marking_color{0.851F, 0.851F, 0.863F, 1.0F};
  double road_marking_z = 0.05;
  double stop_line_width = 0.4;

  Color crosswalk_fill_color{0.251F, 0.251F, 0.267F, 1.0F};
  double crosswalk_fill_z = -0.08;
  Color crosswalk_stripe_color{0.851F, 0.851F, 0.863F, 1.0F};
  double crosswalk_stripe_z = 0.0;
  /// Zebra stripes along the walking direction [m].
  double crosswalk_stripe_width = 0.5;
  double crosswalk_stripe_gap = 0.5;

  double area_z = -0.15;
  /// Areas without or with an unknown subtype.
  Color area_default_color{0.141F, 0.141F, 0.149F, 1.0F};
  Color area_walkway_color{0.220F, 0.220F, 0.231F, 1.0F};
  Color area_vegetation_color{0.153F, 0.243F, 0.180F, 1.0F};
  Color area_parking_color{0.200F, 0.200F, 0.216F, 1.0F};
  Color area_keepout_color{0.302F, 0.141F, 0.149F, 1.0F};
  Color area_building_color{0.235F, 0.235F, 0.251F, 1.0F};
  Color area_traffic_island_color{0.196F, 0.247F, 0.208F, 1.0F};
};

/// @return an error message if the style is invalid.
UFIL_VISUALIZATION_PUBLIC std::optional<std::string> validate(const LaneletMapStyle & style);

/// Fill colour of an area with the given Lanelet2 subtype.
UFIL_VISUALIZATION_PUBLIC Color areaColor(
  const std::string & subtype, const LaneletMapStyle & style);

/**
 * @brief Markers of a lanelet map.
 *
 * Namespaces: lanelet_fill, crosswalk_fill, crosswalk_stripes (triangle lists), lanelet_border,
 * lanelet_separator, road_marking, stop_line (line lists), lanelet_id (one text per lanelet,
 * if enabled) and area_<subtype> (triangle lists). Empty markers are omitted. All markers are
 * frame locked.
 */
UFIL_VISUALIZATION_PUBLIC std::vector<Marker> laneletMapMarkers(
  const lanelet::LaneletMap & map, const LaneletMapStyle & style);

}  // namespace ufil_visualization

#endif  // UFIL_VISUALIZATION__LANELET_MAP_MARKERS_HPP_
