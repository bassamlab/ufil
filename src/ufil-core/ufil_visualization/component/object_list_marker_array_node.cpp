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

#include <exception>
#include <set>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <ufil_msgs/msg/object_list.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "ufil_visualization/object_markers.hpp"
#include "ObjectListMarkerArrayParameterRos.hpp"
#include "ufil_visualization_ros.hpp"

namespace ufil_visualization
{

/**
 * @brief ROS 2 component visualizing object lists as markers.
 *
 * Visualizes every source of the sources parameter on its own output topic, with its own
 * colour. Every object list replaces the markers of the previous one of the same source.
 *
 * Subscribes: <source>.input_topic (ufil_msgs/ObjectList) for every source,
 *             object_list without sources
 * Publishes:  <source>.output_topic (visualization_msgs/MarkerArray) for every source,
 *             object_markers without sources
 */
class ObjectListMarkerArrayNode : public rclcpp::Node
{
public:
  explicit ObjectListMarkerArrayNode(const rclcpp::NodeOptions & options)
  : Node("object_list_marker_array", options),
    parameters_(loadParameters(*this))
  {
    std::set<std::string> output_topics;
    for (const auto & source : parameters_.sources()) {
      auto publisher = create_publisher<visualization_msgs::msg::MarkerArray>(
        source.output_topic, 10);
      if (!output_topics.insert(publisher->get_topic_name()).second) {
        const std::string error = std::string("invalid parameters: output topic ") +
          publisher->get_topic_name() + " used by several sources";
        RCLCPP_FATAL(get_logger(), "%s", error.c_str());
        throw std::invalid_argument(error);
      }
      // The sources live as long as the node, in parameters_.
      const ObjectListSource * source_ptr = &source;
      auto subscription = create_subscription<ufil_msgs::msg::ObjectList>(
        source.input_topic, 10,
        [publisher, source_ptr](const ufil_msgs::msg::ObjectList::ConstSharedPtr & msg) {
          const auto objects = conversions::objectsFromMsg(*msg);
          const auto markers = objectListMarkers(objects, source_ptr->config);
          publisher->publish(conversions::markersToMsg(markers, msg->header, true));
        });
      RCLCPP_INFO(
        get_logger(), "Visualizing the objects of '%s' on '%s'.", subscription->get_topic_name(),
        publisher->get_topic_name());
      publishers_.push_back(std::move(publisher));
      subscriptions_.push_back(std::move(subscription));
    }
  }

private:
  /// Logs invalid parameters before construction fails, so the reason ends up in the ROS log.
  static ObjectListMarkerArrayParameterRos loadParameters(rclcpp::Node & node)
  {
    try {
      return ObjectListMarkerArrayParameterRos(node.get_node_parameters_interface());
    } catch (const std::exception & error) {
      RCLCPP_FATAL(node.get_logger(), "%s", error.what());
      throw;
    }
  }

  ObjectListMarkerArrayParameterRos parameters_;
  std::vector<rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr> publishers_;
  std::vector<rclcpp::Subscription<ufil_msgs::msg::ObjectList>::SharedPtr> subscriptions_;
};

}  // namespace ufil_visualization

RCLCPP_COMPONENTS_REGISTER_NODE(ufil_visualization::ObjectListMarkerArrayNode)
