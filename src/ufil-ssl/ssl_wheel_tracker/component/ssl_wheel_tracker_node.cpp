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

#include <tf2/exceptions.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <chrono>
#include <cinttypes>
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
#include <ssl_vehicle_tracking_msgs/msg/track_array.hpp>
#include <ssl_vehicle_tracking_msgs/msg/wheel_array.hpp>

#include "ssl_wheel_tracker/wheel_detector.hpp"
#include "ssl_wheel_tracker/wheel_tracker.hpp"
#include "SslWheelTrackerParameterRos.hpp"
#include "ssl_wheel_tracker_ros.hpp"

namespace ssl_wheel_tracker
{

/**
 * @brief ROS 2 component detecting and tracking wheels on sensitive surface layers (SSL).
 *
 * Subscribes:  every topic of input_topics (nav_msgs/OccupancyGrid, best effort)
 * Publishes:   wheel_detections (ssl_vehicle_tracking_msgs/WheelArray) for every grid,
 *              wheel_tracks (ssl_vehicle_tracking_msgs/TrackArray) every update_period while
 *              tracks exist and once after the last track was deleted, both in tracking_frame;
 *              /diagnostics
 */
class SslWheelTrackerNode : public rclcpp::Node
{
public:
  explicit SslWheelTrackerNode(const rclcpp::NodeOptions & options)
  : Node("ssl_wheel_tracker", options),
    parameters_(loadParameters(*this)),
    detector_(parameters_.detector()),
    tracker_(parameters_.tracker()),
    diagnostics_(this)
  {
    const auto & settings = parameters_.settings();

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_, this);

    const auto qos = rclcpp::QoS(10).best_effort().durability_volatile();
    detection_publisher_ =
      create_publisher<ssl_vehicle_tracking_msgs::msg::WheelArray>("wheel_detections", qos);
    track_publisher_ =
      create_publisher<ssl_vehicle_tracking_msgs::msg::TrackArray>("wheel_tracks", qos);
    for (const auto & topic : settings.input_topics) {
      subscriptions_.push_back(
        create_subscription<nav_msgs::msg::OccupancyGrid>(
          topic, qos,
          [this](const nav_msgs::msg::OccupancyGrid::ConstSharedPtr & msg) {onGrid(*msg);}));
    }

    timer_ = rclcpp::create_timer(
      this, get_clock(), std::chrono::duration<double>(settings.update_period),
      [this] {onUpdate();});

    diagnostics_.setHardwareID("ssl_wheel_tracker");
    diagnostics_.add("Wheel tracker", this, &SslWheelTrackerNode::produceDiagnostics);

    std::string topics;
    for (const auto & subscription : subscriptions_) {
      topics += (topics.empty() ? "'" : ", '") + std::string(subscription->get_topic_name()) + "'";
    }
    RCLCPP_INFO(
      get_logger(), "Tracking wheels of %s in frame '%s', publishing '%s' and '%s'.",
      topics.c_str(), settings.tracking_frame.c_str(), track_publisher_->get_topic_name(),
      detection_publisher_->get_topic_name());
  }

private:
  /// Logs invalid parameters before construction fails, so the reason ends up in the ROS log.
  static SslWheelTrackerParameterRos loadParameters(rclcpp::Node & node)
  {
    try {
      return SslWheelTrackerParameterRos(node.get_node_parameters_interface());
    } catch (const std::invalid_argument & error) {
      RCLCPP_FATAL(node.get_logger(), "%s", error.what());
      throw;
    }
  }

  void onGrid(const nav_msgs::msg::OccupancyGrid & msg)
  {
    const auto & frame = parameters_.settings().tracking_frame;
    const PressureGrid grid = conversions::gridFromMsg(msg);
    if (!grid.valid()) {
      ++invalid_grids_;
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Ignoring pressure grid in '%s' with %zu cells for %u x %u cells of %.3f m.",
        msg.header.frame_id.c_str(), msg.data.size(), msg.info.width, msg.info.height,
        msg.info.resolution);
      return;
    }

    // A grid without stamp is tracked at its reception time.
    rclcpp::Time stamp(msg.header.stamp, get_clock()->get_clock_type());
    if (stamp.nanoseconds() == 0) {
      stamp = now();
    }

    const auto detections = detector_.detect(grid);
    std::vector<Point2D> positions;
    positions.reserve(detections.size());
    if (msg.header.frame_id.empty() || msg.header.frame_id == frame) {
      for (const auto & detection : detections) {
        positions.push_back(detection.position);
      }
    } else {
      geometry_msgs::msg::TransformStamped transform;
      try {
        transform = tf_buffer_->lookupTransform(frame, msg.header.frame_id, stamp);
      } catch (const tf2::TransformException & error) {
        ++transform_failures_;
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 5000, "Dropping pressure grid: %s", error.what());
        return;
      }
      for (const auto & detection : detections) {
        positions.push_back(conversions::transformPoint(transform.transform, detection.position));
      }
    }

    detection_publisher_->publish(
      conversions::wheelsToMsg(positions, detections, stamp, frame, next_wheel_id_));
    next_wheel_id_ += static_cast<std::uint32_t>(detections.size());

    switch (tracker_.addFrame(conversions::timestampFromRos(stamp), positions)) {
      case FrameResult::kLate:
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 5000,
          "Dropping pressure grid of '%s' at %.3f s: not newer than the last tracked frame. "
          "Increase update_period if the mats are not synchronized.",
          msg.header.frame_id.c_str(), stamp.seconds());
        break;
      case FrameResult::kReset:
        RCLCPP_WARN(get_logger(), "Time jumped backwards; dropped all wheel tracks.");
        break;
      case FrameResult::kAccepted:
        break;
    }
  }

  void onUpdate()
  {
    const UpdateResult result = tracker_.update();
    if (result.frames == 0) {
      return;
    }
    for (const auto id : result.confirmed) {
      RCLCPP_DEBUG(get_logger(), "Confirmed wheel track %" PRIu64 ".", id);
    }
    for (const auto id : result.deleted) {
      RCLCPP_DEBUG(get_logger(), "Deleted wheel track %" PRIu64 ".", id);
    }

    // Like the input grids, an empty result is only published once.
    const auto & tracks = tracker_.tracks();
    if (tracks.empty() && last_published_empty_) {
      return;
    }
    last_published_empty_ = tracks.empty();
    track_publisher_->publish(
      conversions::tracksToMsg(
        tracks, conversions::timestampToMsg(*tracker_.status().last_frame),
        parameters_.settings().tracking_frame));
  }

  void produceDiagnostics(diagnostic_updater::DiagnosticStatusWrapper & diagnostic)
  {
    using diagnostic_msgs::msg::DiagnosticStatus;
    const TrackerStatus status = tracker_.status();

    const auto new_failures = transform_failures_ - reported_transform_failures_;
    const auto new_invalid = invalid_grids_ - reported_invalid_grids_;
    const auto new_late = status.late_frames - reported_late_frames_;
    reported_transform_failures_ = transform_failures_;
    reported_invalid_grids_ = invalid_grids_;
    reported_late_frames_ = status.late_frames;

    if (new_failures > 0) {
      diagnostic.summaryf(
        DiagnosticStatus::WARN, "%" PRIu64 " grids dropped: no transform to '%s'",
        new_failures, parameters_.settings().tracking_frame.c_str());
    } else if (new_invalid > 0) {
      diagnostic.summaryf(
        DiagnosticStatus::WARN, "%" PRIu64 " invalid grids dropped", new_invalid);
    } else if (new_late > 0) {
      diagnostic.summaryf(
        DiagnosticStatus::WARN, "%" PRIu64 " late grids dropped", new_late);
    } else if (!status.last_frame) {
      diagnostic.summary(DiagnosticStatus::STALE, "Waiting for pressure grids");
    } else {
      diagnostic.summaryf(
        DiagnosticStatus::OK, "%zu wheel tracks, %zu tentative", status.confirmed_tracks,
        status.tentative_tracks);
    }

    if (status.last_frame) {
      diagnostic.add(
        "Last frame age [s]",
        ufil::to_seconds(conversions::timestampFromRos(now()) - *status.last_frame));
    } else {
      diagnostic.add("Last frame age [s]", "no data");
    }
    diagnostic.add("Wheel tracks", status.confirmed_tracks);
    diagnostic.add("Tentative tracks", status.tentative_tracks);
    diagnostic.add("Tracked frames", status.frames);
    diagnostic.add("Detections", status.detections);
    diagnostic.add("Tracks confirmed", status.tracks_confirmed);
    diagnostic.add("Tracks deleted", status.tracks_deleted);
    diagnostic.add("Late frames", status.late_frames);
    diagnostic.add("Backward time jumps", status.resets);
    diagnostic.add("Transform failures", transform_failures_);
    diagnostic.add("Invalid grids", invalid_grids_);
  }

  SslWheelTrackerParameterRos parameters_;
  WheelDetector detector_;
  WheelTracker tracker_;
  std::uint32_t next_wheel_id_ = 0;
  bool last_published_empty_ = true;

  std::uint64_t transform_failures_ = 0;
  std::uint64_t invalid_grids_ = 0;
  std::uint64_t reported_transform_failures_ = 0;
  std::uint64_t reported_invalid_grids_ = 0;
  std::uint64_t reported_late_frames_ = 0;

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
  std::vector<rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr> subscriptions_;
  rclcpp::Publisher<ssl_vehicle_tracking_msgs::msg::WheelArray>::SharedPtr detection_publisher_;
  rclcpp::Publisher<ssl_vehicle_tracking_msgs::msg::TrackArray>::SharedPtr track_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;

  diagnostic_updater::Updater diagnostics_;
};

}  // namespace ssl_wheel_tracker

RCLCPP_COMPONENTS_REGISTER_NODE(ssl_wheel_tracker::SslWheelTrackerNode)
