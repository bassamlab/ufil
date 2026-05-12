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

#include <lanelet2_io/Io.h>
#include <lanelet2_projection/UTM.h>

#include <array>
#include <cmath>
#include <fstream>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/xml_parser.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "ufil_visualization/earcut.hpp"
#include "ufil_visualization/lanelet_map_marker_array_parameters.hpp"

using Marker = visualization_msgs::msg::Marker;
using MarkerArray = visualization_msgs::msg::MarkerArray;
using ColorRGBA = std_msgs::msg::ColorRGBA;
using ufil_visualization::LaneletMapMarkerArrayParameters;

static geometry_msgs::msg::Point pt(double x, double y, double z)
{
  geometry_msgs::msg::Point p;
  p.x = x; p.y = y; p.z = z;
  return p;
}

static Marker makeMarker(
  const std_msgs::msg::Header & h, const std::string & ns, int id,
  int type, ColorRGBA color, double scale)
{
  Marker marker;
  marker.header = h;
  marker.ns = ns;
  marker.id = id;
  marker.type = type;
  marker.action = Marker::ADD;
  marker.color = color;
  marker.scale.x = scale;
  if (type == Marker::TRIANGLE_LIST) {marker.scale.y = marker.scale.z = 1.0;}
  marker.frame_locked = true; marker.pose.orientation.w = 1.0;
  return marker;
}

static void addTriangles(
  Marker & m, const std::vector<std::array<double, 2>> & ring, double z)
{
  if (ring.size() < 3) {return;}
  using Ring = std::vector<std::array<double, 2>>;
  const auto idx = mapbox::earcut<uint32_t>(std::vector<Ring>{ring});
  for (size_t i = 0; i + 2 < idx.size(); i += 3) {
    for (int j = 0; j < 3; ++j) {
      m.points.push_back(pt(ring[idx[i + j]][0], ring[idx[i + j]][1], z));
    }
  }
}

static void addSegments(Marker & marker, const lanelet::ConstLineString3d & ls, double z)
{
  for (size_t i = 0; i + 1 < ls.size(); ++i) {
    marker.points.push_back(pt(ls[i].x(), ls[i].y(), z));
    marker.points.push_back(pt(ls[i + 1].x(), ls[i + 1].y(), z));
  }
}

// Find map origin: a node tagged origin=true, or the bounds centre
static std::pair<double, double> findOrigin(const std::string & path)
{
  namespace pt = boost::property_tree;
  pt::ptree tree;
  pt::read_xml(path, tree);

  for (const auto & child : tree.get_child("osm")) {
    if (child.first != "node") {continue;}
    for (const auto & sub : child.second) {
      if (sub.first != "tag") {continue;}
      if (sub.second.get<std::string>("<xmlattr>.k", "") == "origin" &&
        sub.second.get<std::string>("<xmlattr>.v", "") == "true")
      {
        return {child.second.get<double>("<xmlattr>.lat"),
          child.second.get<double>("<xmlattr>.lon")};
      }
    }
  }

  if (const auto b = tree.get_child_optional("osm.bounds")) {
    return {(b->get<double>("<xmlattr>.minlat") + b->get<double>("<xmlattr>.maxlat")) / 2,
      (b->get<double>("<xmlattr>.minlon") + b->get<double>("<xmlattr>.maxlon")) / 2};
  }

  return {0, 0};
}

class LaneletMapMarkerArrayNode : public rclcpp::Node
{
public:
  LaneletMapMarkerArrayNode()
  : Node("lanelet_map_marker_array_node"), params_(*this)
  {
    rclcpp::QoS qos(1);
    if (params_.use_transient_local) {
      qos.transient_local();
    }
    qos.reliable();
    marker_pub_ = create_publisher<MarkerArray>("map", qos);

    if (params_.map_path.empty()) {throw std::runtime_error("map_path is required");}

    auto markerArray = load(params_.map_path, params_.frame_id);

    marker_pub_->publish(markerArray);
    RCLCPP_INFO(get_logger(), "Published %zu marker groups from %s",
      markerArray.markers.size(), params_.map_path.c_str());
  }

private:
  LaneletMapMarkerArrayParameters params_;
  rclcpp::Publisher<MarkerArray>::SharedPtr marker_pub_;

  ColorRGBA getColor(const std::string & key) const
  {
    if (key == "lanelet_fill") {return params_.lanelet_fill_color;}
    if (key == "lanelet_border") {return params_.lanelet_border_color;}
    if (key == "road_marking") {return params_.road_marking_color;}
    if (key == "area_default") {return params_.area_default_color;}
    if (key == "area_vegetation") {return params_.area_vegetation_color;}
    if (key == "area_parking") {return params_.area_parking_color;}
    if (key == "area_keepout") {return params_.area_keepout_color;}
    if (key == "area_building") {return params_.area_building_color;}
    if (key == "area_traffic_island") {return params_.area_traffic_island_color;}
    return params_.area_default_color;
  }

  MarkerArray load(const std::string & map_path, const std::string & frame_id)
  {
    std::ifstream f(map_path);
    if (!f) {throw std::runtime_error("Cannot open: " + map_path);}

    const auto [lat, lon] = findOrigin(map_path);
    RCLCPP_INFO(get_logger(), "Origin: lat=%.7f lon=%.7f", lat, lon);

    auto map = lanelet::load(map_path,
      lanelet::projection::UtmProjector(lanelet::Origin({lat, lon})));

    std_msgs::msg::Header h; h.stamp = now(); h.frame_id = frame_id;

    return buildMarkers(*map, h);
  }

  MarkerArray buildMarkers(
    const lanelet::LaneletMap & map, const std_msgs::msg::Header & h) const
  {
    MarkerArray markerArray;
    int id = 0;

    auto push = [&](Marker marker) {
        if (!marker.points.empty()) {markerArray.markers.push_back(std::move(marker));}
      };

    // Lanelet fills
    auto fill = makeMarker(h, "lanelet_fill", id++, Marker::TRIANGLE_LIST, getColor("lanelet_fill"),
      params_.area_scale);
    for (const auto & ll : map.laneletLayer) {
      std::vector<std::array<double, 2>> ring;
      for (const auto & p : ll.leftBound()) {ring.push_back({p.x(), p.y()});}
      for (const auto & p : ll.rightBound().invert()) {ring.push_back({p.x(), p.y()});}
      addTriangles(fill, ring, params_.lanelet_fill_z);
    }
    push(std::move(fill));

    // Lanelet borders
    auto border = makeMarker(h, "lanelet_border", id++, Marker::LINE_LIST,
      getColor("lanelet_border"), params_.line_width);
    for (const auto & ll : map.laneletLayer) {
      addSegments(border, ll.leftBound(), params_.lanelet_border_z);
      addSegments(border, ll.rightBound(), params_.lanelet_border_z);
    }
    push(std::move(border));

    // Road markings (solid line_thin / line_thick only)
    auto marking = makeMarker(h, "road_marking", id++, Marker::LINE_LIST, getColor("road_marking"),
      params_.line_width);
    for (const auto & ls : map.lineStringLayer) {
      const auto type = ls.attributeOr<std::string>("type", "");
      if ((type == "line_thin" || type == "line_thick") &&
        ls.attributeOr<std::string>("subtype", "") == "solid")
      {
        addSegments(marking, ls, params_.road_marking_z);
      }
    }
    push(std::move(marking));

    // Area fills
    std::unordered_map<std::string, Marker> areas;
    for (const auto & area : map.areaLayer) {
      const auto sub = area.attributeOr<std::string>("subtype", "default");
      if (!areas.count(sub)) {
        areas.emplace(sub,
          makeMarker(h, "area_" + sub, id++, Marker::TRIANGLE_LIST, getColor("area_" + sub),
          params_.area_scale));
      }
      std::vector<std::array<double, 2>> ring;
      for (const auto & p : area.outerBoundPolygon()) {ring.push_back({p.x(), p.y()});}
      addTriangles(areas[sub], ring, params_.area_z);
    }
    for (auto & [_, m] : areas) {push(std::move(m));}

    return markerArray;
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<LaneletMapMarkerArrayNode>());
  } catch (const std::exception & e) {
    RCLCPP_FATAL(rclcpp::get_logger("lanelet_map_marker_array_node"), "%s", e.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
}
