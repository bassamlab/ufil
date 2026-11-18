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

#include <tf2_ros/static_transform_broadcaster.h>

#include <chrono>
#include <cinttypes>
#include <cstdint>
#include <memory>
#include <stdexcept>
#include <string>

#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_updater/diagnostic_updater.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <ufil_msgs/msg/object_list.hpp>

#include "ssl_simulator/mat_simulator.hpp"
#include "SslSimulatorParameterRos.hpp"
#include "ssl_simulator_ros.hpp"

namespace ssl_simulator
{

/**
 * @brief ROS 2 component simulating a sensitive surface layer (SSL) pressure mat.
 *
 * Subscribes:  object_list (ufil_msgs/ObjectList, map frame)
 * Publishes:   ssl_<node_id>/map (nav_msgs/OccupancyGrid, best effort) every frame_period
 *              while cells are pressed, and one empty grid after the mat became empty;
 *              the static transform map -> ssl_frame_<node_id>; /diagnostics
 */
class SslSimulatorNode : public rclcpp::Node
{
public:
  explicit SslSimulatorNode(const rclcpp::NodeOptions & options)
  : Node("ssl_simulator", options),
    parameters_(loadParameters(*this)),
    simulator_(parameters_.config()),
    diagnostics_(this)
  {
    const auto & settings = parameters_.settings();
    const auto & config = simulator_.config();

    subscription_ = create_subscription<ufil_msgs::msg::ObjectList>(
      "object_list", rclcpp::SensorDataQoS(),
      [this](const ufil_msgs::msg::ObjectList::ConstSharedPtr & msg) {onObjectList(*msg);});
    publisher_ = create_publisher<nav_msgs::msg::OccupancyGrid>(
      conversions::mapTopic(settings.node_id), rclcpp::QoS(10).best_effort().durability_volatile());

    tf_broadcaster_ = std::make_unique<tf2_ros::StaticTransformBroadcaster>(*this);
    tf_broadcaster_->sendTransform(
      conversions::matTransformToMsg(config.pose, settings.node_id, now()));

    timer_ = rclcpp::create_timer(
      this, get_clock(), std::chrono::duration<double>(settings.frame_period),
      [this] {onFrame();});

    diagnostics_.setHardwareID("ssl_" + std::to_string(settings.node_id));
    diagnostics_.add("Virtual SSL", this, &SslSimulatorNode::produceDiagnostics);

    RCLCPP_INFO(
      get_logger(),
      "Simulating mat %" PRId64 " (%zu x %zu cells of %.3f m) at x=%.3f, y=%.3f, heading=%.3f rad, "
      "reading '%s', publishing '%s'.",
      settings.node_id, config.columns(), config.rows(), config.resolution,
      config.pose.x, config.pose.y, config.pose.heading, subscription_->get_topic_name(),
      publisher_->get_topic_name());
  }

private:
  /// Logs invalid parameters before construction fails, so the reason ends up in the ROS log.
  static SslSimulatorParameterRos loadParameters(rclcpp::Node & node)
  {
    try {
      return SslSimulatorParameterRos(node.get_node_parameters_interface());
    } catch (const std::invalid_argument & error) {
      RCLCPP_FATAL(node.get_logger(), "%s", error.what());
      throw;
    }
  }

  void onObjectList(const ufil_msgs::msg::ObjectList & msg)
  {
    if (!msg.header.frame_id.empty() && msg.header.frame_id != conversions::kMapFrame) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Object list in frame '%s' is used as if it were in '%s'.", msg.header.frame_id.c_str(),
        conversions::kMapFrame);
    }
    const auto received = conversions::timestampFromRos(now());
    const auto rejected = simulator_.setVehicles(
      conversions::vehiclesFromMsg(msg, received), received);
    if (rejected > 0) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000, "Ignoring %zu objects with a non-finite state.",
        rejected);
    }
  }

  void onFrame()
  {
    const rclcpp::Time stamp = now();
    const Frame frame = simulator_.step(conversions::timestampFromRos(stamp));
    switch (frame.event) {
      case FrameEvent::kTimeJump:
        RCLCPP_WARN(
          get_logger(), "Time jumped backwards; dropped the vehicles received before the jump.");
        break;
      case FrameEvent::kVehiclesExpired:
        RCLCPP_DEBUG(get_logger(), "No recent object list, the mat is cleared.");
        break;
      case FrameEvent::kNone:
        break;
    }

    // Like the hardware in active point mode, an empty mat is only reported once.
    const bool empty = frame.pressure.activeCells() == 0;
    if (empty && last_frame_empty_) {
      return;
    }
    last_frame_empty_ = empty;
    publisher_->publish(conversions::pressureMapToMsg(frame.pressure, simulator_.config(), stamp));
    ++published_frames_;
  }

  void produceDiagnostics(diagnostic_updater::DiagnosticStatusWrapper & diagnostic)
  {
    using diagnostic_msgs::msg::DiagnosticStatus;
    const SimulatorStatus status = simulator_.status();

    if (!status.last_object_list) {
      diagnostic.summary(DiagnosticStatus::STALE, "Waiting for object lists");
      diagnostic.add("Object list age [s]", "no data");
    } else {
      const double age = ufil::to_seconds(
        conversions::timestampFromRos(now()) - *status.last_object_list);
      diagnostic.add("Object list age [s]", age);
      if (age > simulator_.config().stale_duration) {
        diagnostic.summaryf(DiagnosticStatus::WARN, "No object list for %.1f s", age);
      } else {
        diagnostic.summaryf(
          DiagnosticStatus::OK, "%zu vehicles, %zu on the mat", status.vehicles,
          status.vehicles_on_mat);
      }
    }

    diagnostic.add("Vehicles", status.vehicles);
    diagnostic.add("Vehicles on the mat", status.vehicles_on_mat);
    diagnostic.add("Active cells", status.active_cells);
    diagnostic.add("Simulated frames", status.frames);
    diagnostic.add("Published frames", published_frames_);
    diagnostic.add("Object lists", status.object_lists);
    diagnostic.add("Rejected objects", status.rejected_vehicles);
    diagnostic.add("Expirations", status.expirations);
    diagnostic.add("Backward time jumps", status.time_jumps);
  }

  SslSimulatorParameterRos parameters_;
  MatSimulator simulator_;
  bool last_frame_empty_ = false;
  std::uint64_t published_frames_ = 0;

  rclcpp::Subscription<ufil_msgs::msg::ObjectList>::SharedPtr subscription_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr publisher_;
  std::unique_ptr<tf2_ros::StaticTransformBroadcaster> tf_broadcaster_;
  rclcpp::TimerBase::SharedPtr timer_;

  diagnostic_updater::Updater diagnostics_;
};

}  // namespace ssl_simulator

RCLCPP_COMPONENTS_REGISTER_NODE(ssl_simulator::SslSimulatorNode)
