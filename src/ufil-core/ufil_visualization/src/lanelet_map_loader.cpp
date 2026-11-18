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

#include "ufil_visualization/lanelet_map_loader.hpp"

#include <lanelet2_io/Io.h>
#include <lanelet2_projection/UTM.h>

#include <exception>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>

#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/xml_parser.hpp>

namespace ufil_visualization
{

MapOrigin findMapOrigin(const std::string & path)
{
  namespace pt = boost::property_tree;
  pt::ptree tree;
  try {
    pt::read_xml(path, tree);
  } catch (const pt::xml_parser_error & error) {
    throw std::runtime_error("cannot read '" + path + "': " + error.what());
  }

  const auto osm = tree.get_child_optional("osm");
  if (!osm) {
    throw std::runtime_error("'" + path + "' is not an OSM file");
  }

  for (const auto & [name, node] : *osm) {
    if (name != "node") {
      continue;
    }
    for (const auto & [tag_name, tag] : node) {
      if (tag_name == "tag" && tag.get<std::string>("<xmlattr>.k", "") == "origin" &&
        tag.get<std::string>("<xmlattr>.v", "") == "true")
      {
        return {{node.get<double>("<xmlattr>.lat"), node.get<double>("<xmlattr>.lon")},
          OriginSource::kOriginNode};
      }
    }
  }

  if (const auto bounds = osm->get_child_optional("bounds")) {
    return {{(bounds->get<double>("<xmlattr>.minlat") + bounds->get<double>("<xmlattr>.maxlat")) /
      2.0, (bounds->get<double>("<xmlattr>.minlon") + bounds->get<double>("<xmlattr>.maxlon")) /
      2.0}, OriginSource::kBoundsCentre};
  }

  return {};
}

LoadedLaneletMap loadLaneletMap(const std::string & path)
{
  LoadedLaneletMap result;
  result.origin = findMapOrigin(path);
  const lanelet::projection::UtmProjector projector(
    lanelet::Origin({result.origin.origin.latitude, result.origin.origin.longitude}));
  try {
    result.map = lanelet::load(path, projector, &result.errors);
  } catch (const std::exception & error) {
    throw std::runtime_error("cannot load lanelet map '" + path + "': " + error.what());
  }
  return result;
}

}  // namespace ufil_visualization
