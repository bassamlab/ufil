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

#ifndef UFIL_VISUALIZATION__LANELET_MAP_LOADER_HPP_
#define UFIL_VISUALIZATION__LANELET_MAP_LOADER_HPP_

#include <lanelet2_core/LaneletMap.h>

#include <string>
#include <vector>

#include "ufil_visualization/visibility_control.hpp"

namespace ufil_visualization
{

struct GeoOrigin
{
  double latitude = 0.0;
  double longitude = 0.0;
};

/// Where the origin of the local map coordinates was taken from.
enum class OriginSource
{
  /// A node tagged origin=true.
  kOriginNode,
  /// The centre of the <bounds> element.
  kBoundsCentre,
  /// Neither exists, latitude 0 / longitude 0 is used.
  kDefault,
};

struct MapOrigin
{
  GeoOrigin origin;
  OriginSource source = OriginSource::kDefault;
};

/// A lanelet map projected into local UTM coordinates relative to its origin.
struct LoadedLaneletMap
{
  lanelet::LaneletMapPtr map;
  MapOrigin origin;
  /// Recoverable errors of the Lanelet2 parser; the affected primitives are missing in map.
  std::vector<std::string> errors;
};

/**
 * @brief Reads the origin of an OSM file: a node tagged origin=true, else the bounds centre.
 * @throws std::runtime_error if the file cannot be read or parsed.
 */
UFIL_VISUALIZATION_PUBLIC MapOrigin findMapOrigin(const std::string & path);

/**
 * @brief Loads a Lanelet2 OSM map with a UTM projection around the origin of findMapOrigin().
 * @throws std::runtime_error if the map cannot be read.
 */
UFIL_VISUALIZATION_PUBLIC LoadedLaneletMap loadLaneletMap(const std::string & path);

}  // namespace ufil_visualization

#endif  // UFIL_VISUALIZATION__LANELET_MAP_LOADER_HPP_
