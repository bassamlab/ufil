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

#include <memory>
#include <stdexcept>

#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_updater/diagnostic_updater.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <ufil_msgs/msg/object_list.hpp>

#include "ufil_examples_pedestrian_tracker/pedestrian_tracker.hpp"
#include "TrackerParameterRos.hpp"
#include "tracker_ros.hpp"

namespace ufil_examples_pedestrian_tracker
{

/**
 * @brief ROS 2 component tracking pedestrians.
 *
 * Subscribes:  measured_objects (ufil_msgs/ObjectList)
 * Publishes:   tracked_objects (ufil_msgs/ObjectList) after every update, /diagnostics
 *
 * The tracker runs on the stamps of the received object lists.
 */
class PedestrianTrackerNode : public rclcpp::Node
{
public:
  explicit PedestrianTrackerNode(const rclcpp::NodeOptions & options)
  : Node("pedestrian_tracker", options),
    tracker_(loadParameters(*this).config()),
    diagnostics_(this)
  {
    publisher_ = create_publisher<ufil_msgs::msg::ObjectList>("tracked_objects", 10);
    subscription_ = create_subscription<ufil_msgs::msg::ObjectList>(
      "measured_objects", 10,
      [this](const ufil_msgs::msg::ObjectList::ConstSharedPtr & msg) {onDetections(*msg);});

    diagnostics_.setHardwareID("pedestrian_tracker");
    diagnostics_.add("Pedestrian tracker", this, &PedestrianTrackerNode::produceDiagnostics);

    RCLCPP_INFO(
      get_logger(), "Tracking pedestrians from '%s' on '%s'.", subscription_->get_topic_name(),
      publisher_->get_topic_name());
  }

private:
  /// Logs invalid parameters before construction fails, so the reason ends up in the ROS log.
  static TrackerParameterRos loadParameters(rclcpp::Node & node)
  {
    try {
      return TrackerParameterRos(node.get_node_parameters_interface());
    } catch (const std::invalid_argument & error) {
      RCLCPP_FATAL(node.get_logger(), "%s", error.what());
      throw;
    }
  }

  void onDetections(const ufil_msgs::msg::ObjectList & msg)
  {
    const auto event = tracker_.update(
      conversions::detectionsFromMsg(msg), conversions::timestampFromMsg(msg.header.stamp));
    if (event == UpdateEvent::kResetAfterTimeJump) {
      RCLCPP_WARN(get_logger(), "Time jump in the detections, the tracker was reset.");
    }
    publisher_->publish(conversions::tracksToMsg(tracker_.tracks(), msg.header));
  }

  void produceDiagnostics(diagnostic_updater::DiagnosticStatusWrapper & diagnostic)
  {
    using diagnostic_msgs::msg::DiagnosticStatus;
    const TrackerStatus status = tracker_.status();
    if (!status.last_update) {
      diagnostic.summary(DiagnosticStatus::STALE, "Waiting for detections");
      diagnostic.add("Data age [s]", "no data");
    } else {
      const double age = ufil::to_seconds(
        conversions::timestampFromRos(now()) - *status.last_update);
      diagnostic.add("Data age [s]", age);
      if (age > tracker_.config().max_time_gap) {
        diagnostic.summaryf(DiagnosticStatus::WARN, "No detections for %.1f s", age);
      } else {
        diagnostic.summaryf(DiagnosticStatus::OK, "Tracking %zu pedestrians", status.active_tracks);
      }
    }
    diagnostic.add("Active tracks", status.active_tracks);
    diagnostic.add("Updates", status.updates);
    diagnostic.add("Resets after time jumps", status.time_jump_resets);
  }

  PedestrianTracker tracker_;
  rclcpp::Publisher<ufil_msgs::msg::ObjectList>::SharedPtr publisher_;
  rclcpp::Subscription<ufil_msgs::msg::ObjectList>::SharedPtr subscription_;
  diagnostic_updater::Updater diagnostics_;
};

}  // namespace ufil_examples_pedestrian_tracker

RCLCPP_COMPONENTS_REGISTER_NODE(ufil_examples_pedestrian_tracker::PedestrianTrackerNode)
