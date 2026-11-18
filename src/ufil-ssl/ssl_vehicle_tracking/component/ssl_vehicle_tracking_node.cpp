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

#include <cinttypes>
#include <cstdint>
#include <optional>
#include <stdexcept>
#include <string>

#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_updater/diagnostic_updater.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <ssl_vehicle_tracking_msgs/msg/track_array.hpp>
#include <ufil_msgs/msg/object_list.hpp>

#include "ssl_vehicle_tracking/vehicle_tracker.hpp"
#include "SslVehicleTrackingParameterRos.hpp"
#include "ssl_vehicle_tracking_ros.hpp"

namespace ssl_vehicle_tracking
{

/**
 * @brief ROS 2 component assembling and tracking vehicles from the wheel tracks of the sensitive
 *        surface layer (SSL).
 *
 * Subscribes:  wheel_tracks (ssl_vehicle_tracking_msgs/TrackArray, best effort)
 * Publishes:   vehicles (ufil_msgs/ObjectList) for every wheel track frame while vehicles
 *              exist and once after the last vehicle was dropped, with the header of the wheel
 *              tracks; /diagnostics
 */
class SslVehicleTrackingNode : public rclcpp::Node
{
public:
  explicit SslVehicleTrackingNode(const rclcpp::NodeOptions & options)
  : Node("ssl_vehicle_tracking", options),
    parameters_(loadParameters(*this)),
    tracker_(parameters_.tracker(), parameters_.templates()),
    diagnostics_(this)
  {
    subscription_ = create_subscription<ssl_vehicle_tracking_msgs::msg::TrackArray>(
      "wheel_tracks", rclcpp::QoS(10).best_effort().durability_volatile(),
      [this](const ssl_vehicle_tracking_msgs::msg::TrackArray::ConstSharedPtr & msg) {
        onWheelTracks(*msg);
      });
    publisher_ = create_publisher<ufil_msgs::msg::ObjectList>("vehicles", 10);

    diagnostics_.setHardwareID("ssl_vehicle_tracking");
    diagnostics_.add("Vehicle tracking", this, &SslVehicleTrackingNode::produceDiagnostics);

    std::string templates;
    for (const auto & path : parameters_.templatePaths()) {
      templates += (templates.empty() ? "'" : ", '") + path + "'";
    }
    RCLCPP_INFO(
      get_logger(), "Tracking vehicles with the templates %s, reading '%s', publishing '%s'.",
      templates.c_str(), subscription_->get_topic_name(), publisher_->get_topic_name());
  }

private:
  /// Logs invalid parameters before construction fails, so the reason ends up in the ROS log.
  static SslVehicleTrackingParameterRos loadParameters(rclcpp::Node & node)
  {
    try {
      return SslVehicleTrackingParameterRos(node.get_node_parameters_interface());
    } catch (const std::invalid_argument & error) {
      RCLCPP_FATAL(node.get_logger(), "%s", error.what());
      throw;
    }
  }

  void onWheelTracks(const ssl_vehicle_tracking_msgs::msg::TrackArray & msg)
  {
    last_input_ = now();
    const auto [tracks, rejected] = conversions::wheelTracksFromMsg(msg);
    if (rejected > 0) {
      rejected_tracks_ += rejected;
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Ignoring %zu wheel tracks without state or with a non-finite state.", rejected);
    }

    const UpdateResult result = tracker_.update(tracks);
    for (const auto id : result.created) {
      RCLCPP_DEBUG(get_logger(), "Detected vehicle %" PRIu32 ".", id);
    }
    for (const auto id : result.reassembled) {
      RCLCPP_DEBUG(get_logger(), "Reassigned missing wheels of vehicle %" PRIu32 ".", id);
    }
    for (const auto id : result.dropped) {
      RCLCPP_DEBUG(get_logger(), "Dropped vehicle %" PRIu32 ".", id);
    }

    auto objects = conversions::vehiclesToMsg(
      tracker_.vehicles(), tracker_.templates(), msg.header,
      parameters_.settings().publishing_threshold);
    // An empty object list is only published once.
    const bool empty = objects.objects.empty();
    if (empty && last_published_empty_) {
      return;
    }
    last_published_empty_ = empty;
    publisher_->publish(std::move(objects));
  }

  void produceDiagnostics(diagnostic_updater::DiagnosticStatusWrapper & diagnostic)
  {
    using diagnostic_msgs::msg::DiagnosticStatus;
    const TrackerStatus status = tracker_.status();

    const auto new_rejected = rejected_tracks_ - reported_rejected_tracks_;
    reported_rejected_tracks_ = rejected_tracks_;
    if (new_rejected > 0) {
      diagnostic.summaryf(
        DiagnosticStatus::WARN, "%" PRIu64 " invalid wheel tracks ignored", new_rejected);
    } else if (!last_input_) {
      diagnostic.summary(DiagnosticStatus::STALE, "Waiting for wheel tracks");
    } else {
      diagnostic.summaryf(
        DiagnosticStatus::OK, "%zu vehicles, %zu of %zu wheel tracks unassigned",
        status.vehicles, status.unassigned_tracks, status.wheel_tracks);
    }

    if (last_input_) {
      diagnostic.add("Last wheel tracks age [s]", (now() - *last_input_).seconds());
    } else {
      diagnostic.add("Last wheel tracks age [s]", "no data");
    }
    diagnostic.add("Vehicles", status.vehicles);
    diagnostic.add("Wheel tracks", status.wheel_tracks);
    diagnostic.add("Unassigned wheel tracks", status.unassigned_tracks);
    diagnostic.add("Frames", status.frames);
    diagnostic.add("Vehicles detected", status.vehicles_created);
    diagnostic.add("Vehicles dropped", status.vehicles_dropped);
    diagnostic.add("Reassemblies", status.reassemblies);
    diagnostic.add("Wheels dropped", status.wheels_dropped);
    diagnostic.add("Invalid wheel tracks", rejected_tracks_);
  }

  SslVehicleTrackingParameterRos parameters_;
  VehicleTracker tracker_;
  bool last_published_empty_ = true;
  std::optional<rclcpp::Time> last_input_;
  std::uint64_t rejected_tracks_ = 0;
  std::uint64_t reported_rejected_tracks_ = 0;

  rclcpp::Subscription<ssl_vehicle_tracking_msgs::msg::TrackArray>::SharedPtr subscription_;
  rclcpp::Publisher<ufil_msgs::msg::ObjectList>::SharedPtr publisher_;

  diagnostic_updater::Updater diagnostics_;
};

}  // namespace ssl_vehicle_tracking

RCLCPP_COMPONENTS_REGISTER_NODE(ssl_vehicle_tracking::SslVehicleTrackingNode)
