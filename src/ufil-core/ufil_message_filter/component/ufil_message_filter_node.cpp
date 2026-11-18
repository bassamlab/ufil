// Copyright 2025 Chair of Embedded Software (Computer Science 11) - RWTH Aachen University
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

#include <cstdio>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_updater/diagnostic_updater.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <ufil_msgs/msg/object_list.hpp>

#include "ufil_message_filter/position_filter.hpp"
#include "UfilMessageFilterParameterRos.hpp"
#include "ufil_message_filter_ros.hpp"

namespace ufil_message_filter
{

/**
 * @brief ROS 2 component forwarding the objects inside an area of interest.
 *
 * Subscribes:  objects (ufil_msgs/ObjectList)
 * Publishes:   objects_filtered (ufil_msgs/ObjectList) with the header of the input list and its
 *              objects inside the area of interest. Lists without such an object are not
 *              published.
 *              /diagnostics
 */
class ObjectListPositionFilterNode : public rclcpp::Node
{
public:
  explicit ObjectListPositionFilterNode(const rclcpp::NodeOptions & options)
  : Node("object_list_position_filter", options),
    filter_(loadParameters(*this).config()),
    diagnostics_(this)
  {
    publisher_ = create_publisher<ufil_msgs::msg::ObjectList>("objects_filtered", 10);
    subscription_ = create_subscription<ufil_msgs::msg::ObjectList>(
      "objects", 10,
      [this](const ufil_msgs::msg::ObjectList::ConstSharedPtr & msg) {onObjectList(*msg);});

    if (filter_.enabled()) {
      std::string vertices;
      for (const auto & vertex : filter_.config().area_of_interest) {
        char buffer[64];
        std::snprintf(buffer, sizeof(buffer), "(%.2f, %.2f)", vertex.x, vertex.y);
        vertices += (vertices.empty() ? "" : ", ") + std::string(buffer);
      }
      RCLCPP_INFO(
        get_logger(), "Forwarding objects from '%s' to '%s' inside the area of interest %s.",
        subscription_->get_topic_name(), publisher_->get_topic_name(), vertices.c_str());
    } else {
      RCLCPP_INFO(
        get_logger(), "Area of interest disabled, forwarding all objects from '%s' to '%s'.",
        subscription_->get_topic_name(), publisher_->get_topic_name());
    }

    diagnostics_.setHardwareID("none");
    diagnostics_.add("Position filter", this, &ObjectListPositionFilterNode::produceDiagnostics);
  }

private:
  /// Logs invalid parameters before construction fails, so the reason ends up in the ROS log.
  static UfilMessageFilterParameterRos loadParameters(rclcpp::Node & node)
  {
    try {
      return UfilMessageFilterParameterRos(node.get_node_parameters_interface());
    } catch (const std::invalid_argument & error) {
      RCLCPP_FATAL(node.get_logger(), "%s", error.what());
      throw;
    }
  }

  void onObjectList(const ufil_msgs::msg::ObjectList & msg)
  {
    const auto passed = filter_.filter(conversions::positionsFromMsg(msg));
    RCLCPP_DEBUG(
      get_logger(), "%zu of %zu objects inside the area of interest.", passed.size(),
      msg.objects.size());
    if (passed.empty()) {
      return;
    }
    publisher_->publish(conversions::selectObjects(msg, passed));
  }

  void produceDiagnostics(diagnostic_updater::DiagnosticStatusWrapper & diagnostic)
  {
    using diagnostic_msgs::msg::DiagnosticStatus;
    const auto & statistics = filter_.statistics();

    if (statistics.lists == 0) {
      diagnostic.summary(DiagnosticStatus::STALE, "Waiting for object lists");
    } else if (!filter_.enabled()) {
      diagnostic.summary(DiagnosticStatus::OK, "Area of interest disabled, forwarding all objects");
    } else {
      diagnostic.summary(
        DiagnosticStatus::OK,
        std::to_string(statistics.passed_objects) + " of " + std::to_string(statistics.objects) +
        " objects inside the area of interest");
    }

    diagnostic.add("Area of interest vertices", filter_.config().area_of_interest.size());
    diagnostic.add("Received object lists", statistics.lists);
    diagnostic.add("Unpublished empty lists", statistics.empty_lists);
    diagnostic.add("Received objects", statistics.objects);
    diagnostic.add("Passed objects", statistics.passed_objects);
    diagnostic.add("Filtered objects", statistics.rejected_objects);
  }

  PositionFilter filter_;

  rclcpp::Subscription<ufil_msgs::msg::ObjectList>::SharedPtr subscription_;
  rclcpp::Publisher<ufil_msgs::msg::ObjectList>::SharedPtr publisher_;

  diagnostic_updater::Updater diagnostics_;
};

}  // namespace ufil_message_filter

RCLCPP_COMPONENTS_REGISTER_NODE(ufil_message_filter::ObjectListPositionFilterNode)
