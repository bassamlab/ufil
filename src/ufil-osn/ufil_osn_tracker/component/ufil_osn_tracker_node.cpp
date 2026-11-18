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

#include <cstdint>
#include <memory>
#include <string>

#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_updater/diagnostic_updater.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <ufil_msgs/msg/object_list.hpp>
#include <ufil_msgs/msg/sensor_fov.hpp>

#include "ufil_osn_tracker/ufil_osn_tracker.hpp"
#include "UfilOsnTrackerParameterRos.hpp"
#include "ufil_osn_tracker_ros.hpp"

namespace ufil_osn_tracker
{

/**
 * @brief ROS 2 component wrapping OsnObjectTracker.
 *
 * Subscribes:  dynamic_cloud (sensor_msgs/PointCloud2), dynamic_occlusion_grid
 *              (nav_msgs/OccupancyGrid), sensor_fov (ufil_msgs/SensorFOV)
 * Publishes:   object_list, associated_measurements, unassociated_measurements
 *              (ufil_msgs/ObjectList), /diagnostics
 */
class OsnTrackerNode : public rclcpp::Node
{
public:
  explicit OsnTrackerNode(const rclcpp::NodeOptions & options)
  : Node("ufil_osn_tracker", options),
    parameters_(get_node_parameters_interface()),
    tracker_(parameters_.config()),
    diagnostics_(this)
  {
    parameters_.onConfigChanged(
      [this](const TrackerConfig & config) {
        tracker_.configure(config);
        RCLCPP_INFO(get_logger(), "Applied updated tracking parameters.");
      });

    const auto & config = parameters_.config();
    RCLCPP_INFO(
      get_logger(),
      "Publishing tracks in frame '%s' (association threshold %.2f m, clusters of %d to %d "
      "points with tolerance %.2f m).",
      parameters_.settings().target_frame.c_str(), config.association_threshold,
      config.cluster_min_points, config.cluster_max_points, config.cluster_tolerance);

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_, this);

    track_publisher_ = create_publisher<ufil_msgs::msg::ObjectList>("object_list", 10);
    associated_publisher_ =
      create_publisher<ufil_msgs::msg::ObjectList>("associated_measurements", 10);
    unassociated_publisher_ =
      create_publisher<ufil_msgs::msg::ObjectList>("unassociated_measurements", 10);

    cloud_subscription_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      "dynamic_cloud", rclcpp::SensorDataQoS(),
      [this](const sensor_msgs::msg::PointCloud2::ConstSharedPtr & msg) {onPointCloud(*msg);});
    fov_subscription_ = create_subscription<ufil_msgs::msg::SensorFOV>(
      "sensor_fov", 10,
      [this](const ufil_msgs::msg::SensorFOV::ConstSharedPtr & msg) {
        tracker_.setCurrentFOV(conversions::fovFromMsg(*msg));
      });
    grid_subscription_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
      "dynamic_occlusion_grid", 10,
      [this](const nav_msgs::msg::OccupancyGrid::ConstSharedPtr & msg) {onOcclusionGrid(*msg);});

    diagnostics_.setHardwareID("osn_tracker");
    diagnostics_.add("Object tracking", this, &OsnTrackerNode::produceDiagnostics);
  }

private:
  void onOcclusionGrid(const nav_msgs::msg::OccupancyGrid & msg)
  {
    if (!tracker_.status().occlusion_grid_available) {
      RCLCPP_INFO(
        get_logger(), "Received first occlusion grid (%ux%u cells).", msg.info.width,
        msg.info.height);
    }
    tracker_.setOcclusionGrid(
      std::make_shared<const ufil::type::OccupancyGrid>(conversions::occlusionGridFromMsg(msg)));
  }

  void onPointCloud(const sensor_msgs::msg::PointCloud2 & msg)
  {
    const UpdateResult result = tracker_.update(
      conversions::detectionsFromMsg(msg), conversions::timestampFromMsg(msg.header.stamp));
    const TrackerStatus status = tracker_.status();

    switch (result) {
      case UpdateResult::kRejectedOutOfOrder:
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 5000,
          "Ignoring point cloud %.3f s older than the previous one.", -status.last_time_delta);
        return;
      case UpdateResult::kResetAfterTimeJump:
        RCLCPP_WARN(
          get_logger(), "Time jump of %.1f s between point clouds, all tracks were reset.",
          status.last_time_delta);
        break;
      case UpdateResult::kUpdated:
        break;
    }
    RCLCPP_DEBUG(
      get_logger(), "%zu tracks, %zu associations, %zu unassociated measurements.",
      status.active_tracks, status.associations, status.unassociated_measurements);

    publish(msg.header);
  }

  void publish(const std_msgs::msg::Header & header)
  {
    const auto & settings = parameters_.settings();
    const auto tracks = conversions::tracksToMsg(
      tracker_.tracks(), settings.min_existence_probability, header);

    // Publish an empty list once when the last object disappears, then stay silent.
    if (published_track_count_ == 0 && tracks.objects.empty()) {
      return;
    }

    ufil_msgs::msg::ObjectList transformed;
    try {
      transformed = tf_buffer_->transform(tracks, settings.target_frame);
    } catch (const tf2::TransformException & ex) {
      ++consecutive_transform_failures_;
      last_transform_error_ = ex.what();
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000, "Not publishing tracks, transform unavailable: %s",
        ex.what());
      return;
    }
    if (consecutive_transform_failures_ > 0) {
      RCLCPP_INFO(
        get_logger(), "Transform to '%s' available again.", settings.target_frame.c_str());
      consecutive_transform_failures_ = 0;
    }
    published_track_count_ = transformed.objects.size();
    track_publisher_->publish(transformed);

    associated_publisher_->publish(
      conversions::associationsToMsg(tracker_.associations(), header));
    unassociated_publisher_->publish(
      conversions::measurementsToMsg(tracker_.unassociatedMeasurements(), header));
  }

  void produceDiagnostics(diagnostic_updater::DiagnosticStatusWrapper & diagnostic)
  {
    using diagnostic_msgs::msg::DiagnosticStatus;
    const TrackerStatus status = tracker_.status();

    if (consecutive_transform_failures_ > 0) {
      diagnostic.summaryf(
        DiagnosticStatus::ERROR, "Not publishing tracks, transform unavailable: %s",
        last_transform_error_.c_str());
    } else if (status.processed_updates == 0) {
      diagnostic.summary(DiagnosticStatus::STALE, "Waiting for point clouds");
    } else if (status.last_time_delta < 0.0) {
      diagnostic.summary(DiagnosticStatus::WARN, "Receiving point clouds out of order");
    } else {
      diagnostic.summaryf(DiagnosticStatus::OK, "Tracking %zu objects", status.active_tracks);
    }

    diagnostic.add("Active tracks", status.active_tracks);
    diagnostic.add("Published tracks", published_track_count_);
    diagnostic.add("Associations (last update)", status.associations);
    diagnostic.add("Unassociated measurements (last update)", status.unassociated_measurements);
    diagnostic.add("Processed point clouds", status.processed_updates);
    diagnostic.add("Time between point clouds [s]", status.last_time_delta);
    diagnostic.add("Rejected out-of-order point clouds", status.rejected_out_of_order);
    diagnostic.add("Resets after time jumps", status.time_jump_resets);
    diagnostic.add("Occlusion grid received", status.occlusion_grid_available);
    diagnostic.add("Field of view received", status.fov_available);
    diagnostic.add("Consecutive transform failures", consecutive_transform_failures_);
  }

  UfilOsnTrackerParameterRos parameters_;
  OsnObjectTracker tracker_;

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_subscription_;
  rclcpp::Subscription<ufil_msgs::msg::SensorFOV>::SharedPtr fov_subscription_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr grid_subscription_;
  rclcpp::Publisher<ufil_msgs::msg::ObjectList>::SharedPtr track_publisher_;
  rclcpp::Publisher<ufil_msgs::msg::ObjectList>::SharedPtr associated_publisher_;
  rclcpp::Publisher<ufil_msgs::msg::ObjectList>::SharedPtr unassociated_publisher_;

  diagnostic_updater::Updater diagnostics_;
  // Publishing and transforming are done by the node, so their state is tracked here.
  std::size_t published_track_count_ = 0;
  std::uint64_t consecutive_transform_failures_ = 0;
  std::string last_transform_error_;
};

}  // namespace ufil_osn_tracker

RCLCPP_COMPONENTS_REGISTER_NODE(ufil_osn_tracker::OsnTrackerNode)
