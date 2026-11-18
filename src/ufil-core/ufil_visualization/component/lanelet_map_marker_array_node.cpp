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

#include <algorithm>
#include <cstddef>
#include <exception>
#include <stdexcept>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <std_msgs/msg/header.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "ufil_visualization/lanelet_map_loader.hpp"
#include "ufil_visualization/lanelet_map_markers.hpp"
#include "LaneletMapMarkerArrayParameterRos.hpp"
#include "ufil_visualization_ros.hpp"

namespace ufil_visualization
{

/**
 * @brief ROS 2 component publishing a Lanelet2 map as markers.
 *
 * The map given by map_path is loaded once and published latched (reliable, transient local).
 *
 * Publishes: map_markers (visualization_msgs/MarkerArray)
 */
class LaneletMapMarkerArrayNode : public rclcpp::Node
{
public:
  explicit LaneletMapMarkerArrayNode(const rclcpp::NodeOptions & options)
  : Node("lanelet_map_marker_array", options),
    parameters_(loadParameters(*this))
  {
    publisher_ = create_publisher<visualization_msgs::msg::MarkerArray>(
      "map_markers", rclcpp::QoS(1).reliable().transient_local());

    if (parameters_.settings().map_path.empty()) {
      RCLCPP_WARN(get_logger(), "Parameter map_path is empty, no lanelet map is published.");
      return;
    }
    publishMap(parameters_.settings().map_path);
  }

private:
  /// Logs invalid parameters before construction fails, so the reason ends up in the ROS log.
  static LaneletMapMarkerArrayParameterRos loadParameters(rclcpp::Node & node)
  {
    try {
      return LaneletMapMarkerArrayParameterRos(node.get_node_parameters_interface());
    } catch (const std::exception & error) {
      RCLCPP_FATAL(node.get_logger(), "%s", error.what());
      throw;
    }
  }

  void publishMap(const std::string & path)
  {
    LoadedLaneletMap loaded;
    try {
      loaded = loadLaneletMap(path);
    } catch (const std::runtime_error & error) {
      RCLCPP_FATAL(get_logger(), "%s", error.what());
      throw;
    }

    const auto & origin = loaded.origin.origin;
    switch (loaded.origin.source) {
      case OriginSource::kOriginNode:
        RCLCPP_INFO(
          get_logger(), "Map origin from the origin node: lat %.7f, lon %.7f.", origin.latitude,
          origin.longitude);
        break;
      case OriginSource::kBoundsCentre:
        RCLCPP_INFO(
          get_logger(), "Map origin from the bounds centre: lat %.7f, lon %.7f.",
          origin.latitude, origin.longitude);
        break;
      case OriginSource::kDefault:
        RCLCPP_WARN(
          get_logger(), "'%s' has neither a node tagged origin=true nor bounds; using lat 0, "
          "lon 0 as origin.", path.c_str());
        break;
    }

    if (!loaded.errors.empty()) {
      constexpr std::size_t kMaxReportedErrors = 5;
      RCLCPP_WARN(
        get_logger(), "%zu errors while parsing '%s', the map is incomplete.",
        loaded.errors.size(), path.c_str());
      for (std::size_t i = 0; i < std::min(kMaxReportedErrors, loaded.errors.size()); ++i) {
        RCLCPP_WARN(get_logger(), "  %s", loaded.errors[i].c_str());
      }
    }

    const auto markers = laneletMapMarkers(*loaded.map, parameters_.style());
    std_msgs::msg::Header header;
    header.frame_id = parameters_.settings().frame_id;
    header.stamp = now();
    publisher_->publish(conversions::markersToMsg(markers, header, false));
    RCLCPP_INFO(
      get_logger(), "Published %zu markers of %zu lanelets and %zu areas from '%s' in frame '%s'.",
      markers.size(), loaded.map->laneletLayer.size(), loaded.map->areaLayer.size(),
      path.c_str(), header.frame_id.c_str());
  }

  LaneletMapMarkerArrayParameterRos parameters_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr publisher_;
};

}  // namespace ufil_visualization

RCLCPP_COMPONENTS_REGISTER_NODE(ufil_visualization::LaneletMapMarkerArrayNode)
