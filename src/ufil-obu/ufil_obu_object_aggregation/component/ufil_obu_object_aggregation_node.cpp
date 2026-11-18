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

#include <cstdint>
#include <functional>
#include <memory>

#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_updater/diagnostic_updater.hpp>
#include <geometry_msgs/msg/accel_with_covariance_stamped.hpp>
#include <message_filters/subscriber.hpp>
#include <message_filters/sync_policies/approximate_time.hpp>
#include <message_filters/synchronizer.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <ufil_msgs/msg/object_stamped.hpp>

#include "ufil_obu_object_aggregation/object_aggregator.hpp"
#include "UfilObuObjectAggregationParameterRos.hpp"
#include "ufil_obu_object_aggregation_ros.hpp"

namespace ufil_obu_object_aggregation
{

/**
 * @brief ROS 2 component publishing the own vehicle as a UFIL object.
 *
 * Subscribes:  accel (geometry_msgs/AccelWithCovarianceStamped), odometry (nav_msgs/Odometry),
 *              nav_sat_fix (sensor_msgs/NavSatFix), synchronised by approximate time
 * Publishes:   object (ufil_msgs/ObjectStamped) in the frame of the UTM zone, /diagnostics
 */
class ObjectAggregationNode : public rclcpp::Node
{
public:
  explicit ObjectAggregationNode(const rclcpp::NodeOptions & options)
  : Node("object_aggregation", options),
    parameters_(get_node_parameters_interface()),
    aggregator_(parameters_.config()),
    diagnostics_(this)
  {
    parameters_.onConfigChanged(
      [this](const AggregatorConfig & config) {
        aggregator_.configure(config);
        RCLCPP_INFO(get_logger(), "Applied updated object parameters.");
      });

    const auto & config = parameters_.config();
    RCLCPP_INFO(
      get_logger(), "Publishing own vehicle as object %u (%.2f m x %.2f m, class %d).",
      aggregator_.status().object_id, config.object_length, config.object_width,
      config.object_class);

    object_publisher_ = create_publisher<ufil_msgs::msg::ObjectStamped>("object", 10);

    accel_subscriber_.subscribe(this, "accel");
    odometry_subscriber_.subscribe(this, "odometry");
    fix_subscriber_.subscribe(this, "nav_sat_fix");
    sync_ = std::make_unique<message_filters::Synchronizer<SyncPolicy>>(
      SyncPolicy(kSyncQueueSize), accel_subscriber_, odometry_subscriber_, fix_subscriber_);
    sync_->setAgePenalty(1.0);
    sync_->registerCallback(
      std::bind(
        &ObjectAggregationNode::onSynchronizedInput, this, std::placeholders::_1,
        std::placeholders::_2, std::placeholders::_3));

    diagnostics_.setHardwareID("obu");
    diagnostics_.add("Own vehicle object", this, &ObjectAggregationNode::produceDiagnostics);
  }

private:
  using SyncPolicy = message_filters::sync_policies::ApproximateTime<
    geometry_msgs::msg::AccelWithCovarianceStamped, nav_msgs::msg::Odometry,
    sensor_msgs::msg::NavSatFix>;
  static constexpr std::uint32_t kSyncQueueSize = 10;

  void onSynchronizedInput(
    const geometry_msgs::msg::AccelWithCovarianceStamped::ConstSharedPtr & accel,
    const nav_msgs::msg::Odometry::ConstSharedPtr & odometry,
    const sensor_msgs::msg::NavSatFix::ConstSharedPtr & fix)
  {
    const AggregationEvent event = aggregator_.aggregate(
      conversions::fixFromMsg(*fix), conversions::odometryFromMsg(*odometry),
      conversions::accelerationFromMsg(*accel));

    switch (event) {
      case AggregationEvent::kRejectedInvalidFix:
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 5000,
          "No valid GNSS fix (status %d, lat %.6f, lon %.6f), not publishing the object.",
          fix->status.status, fix->latitude, fix->longitude);
        return;
      case AggregationEvent::kUtmZoneChanged:
        RCLCPP_INFO(
          get_logger(), "Publishing object in frame '%s'.",
          aggregator_.lastObject()->position.frameId().c_str());
        break;
      case AggregationEvent::kAggregated:
        break;
    }

    const auto stamp = conversions::latestStamp(
      {accel->header.stamp, odometry->header.stamp, fix->header.stamp});
    object_publisher_->publish(conversions::objectToMsg(*aggregator_.lastObject(), stamp));
  }

  void produceDiagnostics(diagnostic_updater::DiagnosticStatusWrapper & diagnostic)
  {
    using diagnostic_msgs::msg::DiagnosticStatus;
    const AggregatorStatus status = aggregator_.status();

    if (status.aggregated == 0 && status.rejected_invalid_fix == 0) {
      diagnostic.summary(
        DiagnosticStatus::STALE, "Waiting for synchronised acceleration, odometry and GNSS");
    } else if (status.last_rejected) {
      diagnostic.summary(DiagnosticStatus::WARN, "No valid GNSS fix");
    } else {
      diagnostic.summaryf(
        DiagnosticStatus::OK, "Publishing object in %s", status.last_position->frameId().c_str());
    }

    diagnostic.add("Object id", status.object_id);
    diagnostic.add("Published objects", status.aggregated);
    diagnostic.add("Rejected inputs without valid fix", status.rejected_invalid_fix);
    if (status.last_position) {
      diagnostic.add("Frame", status.last_position->frameId());
      diagnostic.add("Easting [m]", status.last_position->easting);
      diagnostic.add("Northing [m]", status.last_position->northing);
    }
  }

  UfilObuObjectAggregationParameterRos parameters_;
  ObjectAggregator aggregator_;

  message_filters::Subscriber<geometry_msgs::msg::AccelWithCovarianceStamped> accel_subscriber_;
  message_filters::Subscriber<nav_msgs::msg::Odometry> odometry_subscriber_;
  message_filters::Subscriber<sensor_msgs::msg::NavSatFix> fix_subscriber_;
  std::unique_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;
  rclcpp::Publisher<ufil_msgs::msg::ObjectStamped>::SharedPtr object_publisher_;

  diagnostic_updater::Updater diagnostics_;
};

}  // namespace ufil_obu_object_aggregation

RCLCPP_COMPONENTS_REGISTER_NODE(ufil_obu_object_aggregation::ObjectAggregationNode)
