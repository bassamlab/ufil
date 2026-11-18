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

#include <tf2/exceptions.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <chrono>
#include <cstdint>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_updater/diagnostic_updater.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <ufil_msgs/msg/object_list.hpp>
#include <ufil_msgs/msg/sensor_fov.hpp>

#include "ufil_central_fusion/central_fusion.hpp"
#include "UfilCentralFusionParameterRos.hpp"
#include "ufil_central_fusion_ros.hpp"

namespace ufil_central_fusion
{

/**
 * @brief ROS 2 component fusing the object lists of several sensors.
 *
 * Subscribes:  per sensor <name>.topic (ufil_msgs/ObjectList) and optionally <name>.fov_topic
 *              (ufil_msgs/SensorFOV) and <name>.occlusion_topic (nav_msgs/OccupancyGrid)
 * Publishes:   object_list (ufil_msgs/ObjectList) every fusion.period, /diagnostics
 */
class CentralFusionNode : public rclcpp::Node
{
public:
  explicit CentralFusionNode(const rclcpp::NodeOptions & options)
  : Node("central_fusion", options),
    parameters_(loadParameters(*this)),
    fusion_(parameters_.config(), parameters_.sensors()),
    transform_failures_(parameters_.sensors().size(), 0),
    diagnostics_(this)
  {
    const auto & settings = parameters_.settings();
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_, this);

    for (std::size_t i = 0; i < fusion_.sensors().size(); ++i) {
      const auto & name = fusion_.sensors()[i].name;
      const auto & topics = parameters_.topics()[i];
      object_subscriptions_.push_back(
        create_subscription<ufil_msgs::msg::ObjectList>(
          topics.objects, 10,
          [this, i](const ufil_msgs::msg::ObjectList::ConstSharedPtr & msg) {
            onObjectList(i, *msg);
          }));
      if (!topics.fov.empty()) {
        fov_subscriptions_.push_back(
          create_subscription<ufil_msgs::msg::SensorFOV>(
            topics.fov, 10, [this, i](const ufil_msgs::msg::SensorFOV::ConstSharedPtr & msg) {
              onFov(i, *msg);
            }));
      }
      if (!topics.occlusion.empty()) {
        occlusion_subscriptions_.push_back(
          create_subscription<nav_msgs::msg::OccupancyGrid>(
            topics.occlusion, 10,
            [this, i](const nav_msgs::msg::OccupancyGrid::ConstSharedPtr &) {onOcclusion(i);}));
      }
      RCLCPP_INFO(
        get_logger(), "Fusing sensor '%s' from '%s'%s.", name.c_str(),
        object_subscriptions_.back()->get_topic_name(),
        fusion_.sensors()[i].is_cam ? " (CAM)" : "");
    }

    publisher_ = create_publisher<ufil_msgs::msg::ObjectList>("object_list", 10);
    timer_ = rclcpp::create_timer(
      this, get_clock(), std::chrono::duration<double>(settings.period), [this] {onTimer();});

    diagnostics_.setHardwareID("rsu");
    diagnostics_.add("Central fusion", this, &CentralFusionNode::produceDiagnostics);
  }

private:
  /// Logs invalid parameters before construction fails, so the reason ends up in the ROS log.
  static UfilCentralFusionParameterRos loadParameters(rclcpp::Node & node)
  {
    try {
      return UfilCentralFusionParameterRos(node.get_node_parameters_interface());
    } catch (const std::invalid_argument & error) {
      RCLCPP_FATAL(node.get_logger(), "%s", error.what());
      throw;
    }
  }

  void onObjectList(std::size_t sensor, const ufil_msgs::msg::ObjectList & msg)
  {
    const auto & name = fusion_.sensors()[sensor].name;
    ufil_msgs::msg::ObjectList transformed;
    try {
      transformed = tf_buffer_->transform(msg, parameters_.settings().target_frame);
    } catch (const tf2::TransformException & ex) {
      ++transform_failures_[sensor];
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000, "Dropping objects of sensor '%s': %s", name.c_str(),
        ex.what());
      return;
    }
    transform_failures_[sensor] = 0;

    const auto event = fusion_.addMeasurements(
      sensor, conversions::measurementsFromMsg(transformed),
      conversions::timestampFromMsg(msg.header.stamp));
    switch (event) {
      case MeasurementEvent::kResetAfterTimeJump:
        RCLCPP_WARN(
          get_logger(), "Time jump in the data of sensor '%s', the fusion was reset.",
          name.c_str());
        break;
      case MeasurementEvent::kRejectedTimeJump:
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 5000,
          "Dropping CAM objects of sensor '%s' with an implausible stamp.", name.c_str());
        break;
      case MeasurementEvent::kRejectedTooOld:
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 5000,
          "Dropping objects of sensor '%s' older than the fusion buffer.", name.c_str());
        break;
      case MeasurementEvent::kBuffered:
        break;
    }
  }

  void onFov(std::size_t sensor, const ufil_msgs::msg::SensorFOV & msg)
  {
    ufil_msgs::msg::SensorFOV transformed;
    try {
      transformed = tf_buffer_->transform(msg, parameters_.settings().target_frame);
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000, "Ignoring field of view of sensor '%s': %s",
        fusion_.sensors()[sensor].name.c_str(), ex.what());
      return;
    }
    fusion_.setSensorFov(sensor, conversions::fovFromMsg(transformed));
  }

  void onOcclusion(std::size_t sensor)
  {
    if (!fusion_.status().sensors[sensor].occlusion_available) {
      RCLCPP_INFO(
        get_logger(), "Sensor '%s' provides an occlusion grid.",
        fusion_.sensors()[sensor].name.c_str());
    }
    fusion_.setOcclusionAvailable(sensor);
  }

  void onTimer()
  {
    const rclcpp::Time now = this->now();
    fusion_.update(conversions::timestampFromRos(now));

    std_msgs::msg::Header header;
    header.frame_id = parameters_.settings().target_frame;
    header.stamp = now;
    publisher_->publish(
      conversions::tracksToMsg(
        fusion_.tracks(), parameters_.settings().min_existence_probability, header));
  }

  void produceDiagnostics(diagnostic_updater::DiagnosticStatusWrapper & diagnostic)
  {
    using diagnostic_msgs::msg::DiagnosticStatus;
    const FusionStatus status = fusion_.status();
    const auto now = conversions::timestampFromRos(this->now());

    std::string silent;
    std::string failing_transforms;
    bool any_data = false;
    for (std::size_t i = 0; i < status.sensors.size(); ++i) {
      const auto & sensor = status.sensors[i];
      const std::string prefix = "Sensor " + sensor.name + ": ";
      if (sensor.last_measurement_time) {
        any_data = true;
        const double age = ufil::to_seconds(now - *sensor.last_measurement_time);
        diagnostic.add(prefix + "data age [s]", age);
        if (age > fusion_.config().buffer_duration) {
          silent += (silent.empty() ? "" : ", ") + sensor.name;
        }
      } else {
        diagnostic.add(prefix + "data age [s]", "no data");
        silent += (silent.empty() ? "" : ", ") + sensor.name;
      }
      if (transform_failures_[i] > 0) {
        failing_transforms += (failing_transforms.empty() ? "" : ", ") + sensor.name;
      }
      diagnostic.add(prefix + "received", sensor.received);
      diagnostic.add(prefix + "rejected (too old)", sensor.rejected_too_old);
      diagnostic.add(prefix + "rejected (time jump)", sensor.rejected_time_jump);
      diagnostic.add(prefix + "field of view", sensor.has_fov);
      diagnostic.add(prefix + "occlusion grid", sensor.occlusion_available);
    }

    if (!any_data) {
      diagnostic.summary(DiagnosticStatus::STALE, "Waiting for sensor data");
    } else if (!failing_transforms.empty()) {
      diagnostic.summaryf(
        DiagnosticStatus::WARN, "Transform unavailable for: %s", failing_transforms.c_str());
    } else if (!silent.empty()) {
      diagnostic.summaryf(DiagnosticStatus::WARN, "No recent data from: %s", silent.c_str());
    } else {
      diagnostic.summaryf(DiagnosticStatus::OK, "Fusing %zu tracks", status.active_tracks);
    }

    diagnostic.add("Active tracks", status.active_tracks);
    diagnostic.add("Buffered measurement sets", status.buffered_measurement_sets);
    diagnostic.add("Out-of-order replays", status.replays);
    diagnostic.add("Resets after time jumps", status.time_jump_resets);
  }

  UfilCentralFusionParameterRos parameters_;
  CentralFusion fusion_;
  // Transformations are done by the node, so their state is tracked here.
  std::vector<std::uint64_t> transform_failures_;

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  std::vector<rclcpp::Subscription<ufil_msgs::msg::ObjectList>::SharedPtr> object_subscriptions_;
  std::vector<rclcpp::Subscription<ufil_msgs::msg::SensorFOV>::SharedPtr> fov_subscriptions_;
  std::vector<rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr>
  occlusion_subscriptions_;
  rclcpp::Publisher<ufil_msgs::msg::ObjectList>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;

  diagnostic_updater::Updater diagnostics_;
};

}  // namespace ufil_central_fusion

RCLCPP_COMPONENTS_REGISTER_NODE(ufil_central_fusion::CentralFusionNode)
