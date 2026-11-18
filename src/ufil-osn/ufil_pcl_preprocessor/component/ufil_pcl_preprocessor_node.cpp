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
#include <tf2/time.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <cstdint>
#include <exception>
#include <memory>
#include <string>
#include <utility>

#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_updater/diagnostic_updater.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <ufil_msgs/msg/sensor_fov.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "ufil_pcl_preprocessor/ufil_pcl_preprocessor.hpp"
#include "UfilPclPreprocessorParameterRos.hpp"
#include "ufil_pcl_preprocessor_ros.hpp"

namespace ufil_pcl_preprocessor
{

/**
 * @brief ROS 2 component wrapping PclPreprocessor.
 *
 * Subscribes:  input_cloud (sensor_msgs/PointCloud2)
 * Publishes:   dynamic_cloud (sensor_msgs/PointCloud2), dynamic_occlusion_grid
 *              (nav_msgs/OccupancyGrid), sensor_fov (ufil_msgs/SensorFOV),
 *              filter_markers (visualization_msgs/MarkerArray), /diagnostics
 */
class PclPreprocessorNode : public rclcpp::Node
{
public:
  explicit PclPreprocessorNode(const rclcpp::NodeOptions & options)
  : Node("ufil_pcl_preprocessor", options),
    parameters_(get_node_parameters_interface()),
    preprocessor_(parameters_.config()),
    diagnostics_(this)
  {
    parameters_.onConfigChanged(
      [this](const PreprocessorConfig & config) {
        preprocessor_.configure(config);
        RCLCPP_INFO(get_logger(), "Applied updated preprocessing parameters.");
      });

    loadClippedAreas();

    const auto & settings = parameters_.settings();
    RCLCPP_INFO(
      get_logger(), "Publishing in frame '%s', estimating the field of view in frame '%s'%s.",
      settings.target_frame.c_str(), settings.sensor_frame.c_str(),
      settings.restamp_output ? ", restamping outputs with the node clock" : "");

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_, this);

    cloud_publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>("dynamic_cloud", 10);
    grid_publisher_ =
      create_publisher<nav_msgs::msg::OccupancyGrid>("dynamic_occlusion_grid", 10);
    fov_publisher_ = create_publisher<ufil_msgs::msg::SensorFOV>("sensor_fov", 10);
    marker_publisher_ =
      create_publisher<visualization_msgs::msg::MarkerArray>("filter_markers", 10);
    cloud_subscription_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      "input_cloud", rclcpp::SensorDataQoS(),
      [this](const sensor_msgs::msg::PointCloud2::ConstSharedPtr & msg) {onPointCloud(*msg);});

    diagnostics_.setHardwareID(settings.sensor_frame);
    diagnostics_.add("PCL preprocessing", this, &PclPreprocessorNode::produceDiagnostics);
  }

private:
  void loadClippedAreas()
  {
    const auto & file = parameters_.settings().static_object_file;
    if (file.empty()) {
      RCLCPP_INFO(get_logger(), "No static_object_file configured, no areas are clipped.");
      return;
    }
    // A configured but unusable file is a configuration error; fail construction.
    preprocessor_.setClippedAreas(loadClippedAreasFromCsv(file));
    RCLCPP_INFO(
      get_logger(), "Loaded %zu clipped areas from '%s'.",
      preprocessor_.clippedAreas().size(), file.c_str());
  }

  void onPointCloud(const sensor_msgs::msg::PointCloud2 & msg)
  {
    const auto & settings = parameters_.settings();

    geometry_msgs::msg::TransformStamped sensor_to_target;
    try {
      const auto cloud_to_sensor = tf_buffer_->lookupTransform(
        settings.sensor_frame, msg.header.frame_id, tf2::TimePointZero);
      sensor_to_target = tf_buffer_->lookupTransform(
        settings.target_frame, settings.sensor_frame, tf2::TimePointZero);
      preprocessor_.setTransforms(
        conversions::transformFromMsg(cloud_to_sensor),
        conversions::transformFromMsg(sensor_to_target));
    } catch (const tf2::TransformException & ex) {
      ++consecutive_transform_failures_;
      ++total_transform_failures_;
      last_transform_error_ = ex.what();
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000, "Dropping point cloud, transform unavailable: %s",
        ex.what());
      return;
    }
    if (consecutive_transform_failures_ > 0) {
      RCLCPP_INFO(
        get_logger(), "Transforms available again after %lu dropped point clouds.",
        static_cast<unsigned long>(consecutive_transform_failures_));  // NOLINT(runtime/int)
      consecutive_transform_failures_ = 0;
    }

    pcl::PCLPointCloud2 output;
    const FilterStatistics stats = preprocessor_.filter(conversions::cloudFromMsg(msg), output);
    RCLCPP_DEBUG(
      get_logger(),
      "Cloud: %zu in, %zu out of height, %zu static, %zu outliers, %zu dynamic, %zu out.",
      stats.input_points, stats.out_of_height_points, stats.static_points,
      stats.outlier_points, stats.dynamic_points, stats.output_points);

    std_msgs::msg::Header header;
    header.frame_id = settings.target_frame;
    header.stamp = settings.restamp_output ? now() : rclcpp::Time(msg.header.stamp);

    cloud_publisher_->publish(conversions::cloudToMsg(output, header));
    grid_publisher_->publish(
      conversions::occlusionGridToMsg(preprocessor_.occlusionGrid(), header));

    // The field of view is estimated in the sensor frame and published in the target frame.
    std_msgs::msg::Header sensor_header = header;
    sensor_header.frame_id = settings.sensor_frame;
    ufil_msgs::msg::SensorFOV fov_msg;
    tf2::doTransform(
      conversions::fovToMsg(preprocessor_.fov(), sensor_header), fov_msg, sensor_to_target);
    fov_msg.header = header;
    fov_publisher_->publish(fov_msg);

    if (marker_publisher_->get_subscription_count() > 0) {
      auto markers = conversions::clippedAreasToMarkers(preprocessor_.clippedAreas(), header);
      for (auto & marker : conversions::fovToMarkers(fov_msg).markers) {
        markers.markers.push_back(std::move(marker));
      }
      marker_publisher_->publish(markers);
    }
  }

  void produceDiagnostics(diagnostic_updater::DiagnosticStatusWrapper & status)
  {
    using diagnostic_msgs::msg::DiagnosticStatus;
    const PreprocessorStatus state = preprocessor_.status();

    if (consecutive_transform_failures_ > 0) {
      status.summaryf(
        DiagnosticStatus::ERROR, "Dropping point clouds, transform unavailable: %s",
        last_transform_error_.c_str());
    } else if (state.processed_clouds == 0) {
      status.summary(DiagnosticStatus::STALE, "Waiting for input point clouds");
    } else if (!state.fov_valid) {
      status.summary(
        DiagnosticStatus::WARN, "Field of view not estimated, too few points per azimuth bin");
    } else {
      status.summary(DiagnosticStatus::OK, "Processing point clouds");
    }

    status.add("Processed clouds", state.processed_clouds);
    status.add("Input points (last cloud)", state.last_cloud.input_points);
    status.add("Out-of-height points (last cloud)", state.last_cloud.out_of_height_points);
    status.add("Static points (last cloud)", state.last_cloud.static_points);
    status.add("Outlier points (last cloud)", state.last_cloud.outlier_points);
    status.add("Output points (last cloud)", state.last_cloud.output_points);
    status.add("Clipped areas", state.clipped_area_count);
    status.add("Field of view valid", state.fov_valid);
    status.add("Field of view max range [m]", state.fov.rMax());
    status.add("Consecutive transform failures", consecutive_transform_failures_);
    status.add("Total transform failures", total_transform_failures_);
  }

  UfilPclPreprocessorParameterRos parameters_;
  PclPreprocessor preprocessor_;

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_subscription_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_publisher_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr grid_publisher_;
  rclcpp::Publisher<ufil_msgs::msg::SensorFOV>::SharedPtr fov_publisher_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_publisher_;

  diagnostic_updater::Updater diagnostics_;
  // Transform lookups are done by the node, so their health is tracked here.
  std::uint64_t consecutive_transform_failures_ = 0;
  std::uint64_t total_transform_failures_ = 0;
  std::string last_transform_error_;
};

}  // namespace ufil_pcl_preprocessor

RCLCPP_COMPONENTS_REGISTER_NODE(ufil_pcl_preprocessor::PclPreprocessorNode)
