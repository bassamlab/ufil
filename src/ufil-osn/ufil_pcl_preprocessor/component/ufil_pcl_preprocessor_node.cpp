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

#include <Eigen/Dense>

#include <pcl_conversions/pcl_conversions.h>

#include <tf2/exceptions.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>

#include <memory>
#include <filesystem>
#include <cmath>


#include <ufil_pcl_preprocessor/ufil_pcl_preprocessor.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include <ufil_msgs/msg/sensor_fov.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>

#include <ufil_ros/ufil_ros.hpp>

#include "rapidcsv.hpp"
#include "ufil_pcl_preprocessor_ros.hpp"
#include "ufil_pcl_preprocessor_parameters.hpp"

class PclProcessingNode : public rclcpp::Node
{
private:
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr viz_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;

  rclcpp::Publisher<ufil_msgs::msg::SensorFOV>::SharedPtr fov_pub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr occ_grid_pub_;

  ufil_pcl_preprocessor::PclProcessing::UniquePtr pcl_preprocessor_;
  ufil_pcl_preprocessor::PCLPreprocessorParameterHandler param_handler_;
  std::vector<ufil_pcl_preprocessor::ClippedArea> clipped_areas_;

  void topic_callback(const sensor_msgs::msg::PointCloud2 & msg) const
  {
    const auto & params = param_handler_.getParameters();

    // Apply filter parameters
    this->pcl_preprocessor_->setVoxelFilterSize(params.filter_voxel_size);
    this->pcl_preprocessor_->setOutlierFilterParameter(
      params.filter_outlier_radius, params.filter_min_number_of_neighbors);
    this->pcl_preprocessor_->setZFilterRange(params.filter_z_min, params.filter_z_max);

    // Apply FOV estimator parameters
    this->pcl_preprocessor_->setNumberOfBins(params.fov_num_bins);
    this->pcl_preprocessor_->setFOVHistoryLength(params.fov_history_length);
    this->pcl_preprocessor_->setFOVMaxRange(params.fov_max_range);

    // Apply depth buffer parameters
    this->pcl_preprocessor_->setDepthBufferHistoryLength(params.depth_buffer_history_length);
    this->pcl_preprocessor_->setDepthBufferMinMaxRange(
      params.depth_buffer_min_range, params.depth_buffer_max_range);
    this->pcl_preprocessor_->setDepthBufferGridParameters(
      params.depth_buffer_grid_resolution,
      params.depth_buffer_grid_size_x,
      params.depth_buffer_grid_size_y);
    this->pcl_preprocessor_->setDepthBufferDecayRate(params.depth_buffer_decay_rate);
    this->pcl_preprocessor_->setDepthBufferBins(
      params.depth_buffer_az_bins, params.depth_buffer_el_bins);
    this->pcl_preprocessor_->setDepthBufferPreprocessThresholds(
      params.depth_buffer_preprocess_T_low,
      params.depth_buffer_preprocess_T_high,
      params.depth_buffer_preprocess_K,
      params.depth_buffer_preprocess_R,
      params.depth_buffer_preprocess_min_count);

    // Transform and filter
    Eigen::Affine3d transform;
    Eigen::Affine3d fov_transform;
    try {
      geometry_msgs::msg::TransformStamped fov_transformStamped =
        this->tf_buffer_->lookupTransform("sensor_node", msg.header.frame_id,
        tf2::timeFromSec(0));
      fov_transform = tf2::transformToEigen(fov_transformStamped.transform);

      geometry_msgs::msg::TransformStamped transformStamped =
        this->tf_buffer_->lookupTransform(params.target_frame, "sensor_node",
        tf2::timeFromSec(0));
      transform = tf2::transformToEigen(transformStamped.transform);
    } catch (tf2::TransformException & ex) {
      RCLCPP_WARN(this->get_logger(), "Transform lookup failed: %s", ex.what());
      return;
    }

    this->pcl_preprocessor_->setFovTransform(fov_transform);
    this->pcl_preprocessor_->setTransform(transform);
    this->pcl_preprocessor_->setClippedAreas(clipped_areas_);

    pcl::PCLPointCloud2::Ptr cloud(new pcl::PCLPointCloud2());
    pcl_conversions::toPCL(msg, *cloud);

    pcl::PCLPointCloud2::Ptr cloud_filtered(new pcl::PCLPointCloud2());
    this->pcl_preprocessor_->filter(cloud, *cloud_filtered);

    sensor_msgs::msg::PointCloud2 msg_out;
    pcl_conversions::fromPCL(*cloud_filtered, msg_out);
    msg_out.header.frame_id = params.target_frame;
    msg_out.header.stamp = params.use_ros_time ? this->now() : rclcpp::Time(msg.header.stamp);
    this->publisher_->publish(msg_out);

    const ufil::type::OccupancyGrid & grid = this->pcl_preprocessor_->getOcclusionMap();
    nav_msgs::msg::OccupancyGrid grid_msg = ufil_ros::occupancyGridToMsg(grid);
    grid_msg.header = msg_out.header;
    grid_msg.info.map_load_time = msg_out.header.stamp;
    this->occ_grid_pub_->publish(grid_msg);

    const ufil::type::SensorFOV & fov = this->pcl_preprocessor_->getFOV();
    ufil_msgs::msg::SensorFOV fov_msg = ufil_ros::sensorFovToMsg(fov);
    fov_msg.header = msg.header;
    fov_msg.header.frame_id = "sensor_node";


    ufil_msgs::msg::SensorFOV transformed_fov_msg;
    try {
      transformed_fov_msg = this->tf_buffer_->transform(fov_msg, params.target_frame);
    } catch (tf2::TransformException & ex) {
      RCLCPP_ERROR(this->get_logger(), "Could not transform: %s", ex.what());
      return;
    }
    this->fov_pub_->publish(transformed_fov_msg);


    visualization_msgs::msg::MarkerArray fov_marker_array =
      ufil_pcl_ros::createFovMarkerArray(transformed_fov_msg);
    this->viz_publisher_->publish(fov_marker_array);

    visualization_msgs::msg::MarkerArray viz_marker_array;
    int marker_id = 100;
    for (auto area : this->clipped_areas_) {
      Eigen::Quaternionf orientation;
      orientation = Eigen::AngleAxisf(area.yaw, Eigen::Vector3f::UnitZ());

      visualization_msgs::msg::Marker marker;
      marker.header = msg_out.header;
      marker.id = marker_id;
      marker.type = visualization_msgs::msg::Marker::CUBE;
      marker.action = visualization_msgs::msg::Marker::ADD;
      marker.pose.position.x = area.x;
      marker.pose.position.y = area.y;
      marker.pose.position.z = area.z;
      marker.pose.orientation.x = orientation.x();
      marker.pose.orientation.y = orientation.y();
      marker.pose.orientation.z = orientation.z();
      marker.pose.orientation.w = orientation.w();
      marker.scale.x = area.length;
      marker.scale.y = area.width;
      marker.scale.z = area.height;
      marker.color.a = 0.5;
      marker.color.r = 0.0;
      marker.color.g = 0.0;
      marker.color.b = 0.0;
      marker.lifetime = rclcpp::Duration(0, 500 * 1e6);
      viz_marker_array.markers.push_back(marker);

      marker_id++;
    }

    this->viz_publisher_->publish(viz_marker_array);
  }

public:
  PclProcessingNode()
  : Node("pcl_processing"), param_handler_(this)
  {
    // Only static_object_file is declared here; all other parameters are handled by param_handler_
    std::string package_share_directory =
      ament_index_cpp::get_package_share_directory("ufil_pcl_preprocessor");
    std::string path = package_share_directory + "/config/static_objects.csv";
    this->declare_parameter("static_object_file", path);

    this->pcl_preprocessor_ = std::make_unique<ufil_pcl_preprocessor::PclProcessing>();

    auto qos = rclcpp::SensorDataQoS();

    this->subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "input_cloud", qos,
      std::bind(&PclProcessingNode::topic_callback, this, std::placeholders::_1));
    this->publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("cloud", 10);
    this->viz_publisher_ =
      this->create_publisher<visualization_msgs::msg::MarkerArray>("filter_viz", 10);

    this->fov_pub_ = this->create_publisher<ufil_msgs::msg::SensorFOV>("sensor_fov", 10);
    this->occ_grid_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(
      "dynamic_occlusion_grid", 10);
    this->tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    this->tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    std::string static_object_file = this->get_parameter("static_object_file").as_string();
    const std::filesystem::path static_object_file_path{static_object_file};

    if (std::filesystem::exists(static_object_file_path)) {
      RCLCPP_INFO(
        this->get_logger(), "Loading static objects from file %s",
        static_object_file.c_str());

      rapidcsv::Document doc(static_object_file);

      const size_t rowCount = doc.GetRowCount();
      for (size_t i = 0; i < rowCount; ++i) {
        std::vector<float> row = doc.GetRow<float>(i);
        ufil_pcl_preprocessor::ClippedArea area{row[0], row[1], row[2], row[3], row[4], row[5],
          row[6]};
        this->clipped_areas_.push_back(area);
      }

      this->pcl_preprocessor_->setClippedAreas(this->clipped_areas_);
    } else {
      RCLCPP_WARN(
        this->get_logger(), "Could not load static object file %s.",
        static_object_file.c_str());
    }
  }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PclProcessingNode>());
  rclcpp::shutdown();
  return 0;
}
