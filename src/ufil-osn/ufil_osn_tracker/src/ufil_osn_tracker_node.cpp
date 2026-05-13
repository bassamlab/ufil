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

#include <tf2_ros/transform_listener.h>
#include <Eigen/Dense>
#include <pcl_conversions/pcl_conversions.h>

#include <tf2/exceptions.h>
#include <tf2_ros/buffer.h>

#include <memory>
#include <vector>
#include <map>
#include <filesystem>

#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <rclcpp/rclcpp.hpp>

#include <ufil_msgs/msg/detection.hpp>
#include <ufil_msgs/msg/detection_list.hpp>
#include <ufil_msgs/msg/object.hpp>
#include <ufil_msgs/msg/object_list.hpp>
#include <ufil_msgs/msg/sensor_fov.hpp>

#include <ufil_object_tracking/types/time.hpp>
#include <ufil_osn_tracker/ufil_osn_tracker.hpp>
#include <ufil_ros/ufil_ros.hpp>

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>

#include <ament_index_cpp/get_package_share_directory.hpp>

class PclObjectTrackerNode : public rclcpp::Node
{
private:
  rclcpp::Publisher<ufil_msgs::msg::ObjectList>::SharedPtr track_publisher_;
  rclcpp::Publisher<ufil_msgs::msg::ObjectList>::SharedPtr unassociated_measurements_publisher;
  rclcpp::Publisher<ufil_msgs::msg::ObjectList>::SharedPtr associated_measurements_publisher_;

  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_subscription_;
  ufil_osn_tracker::OsnObjectTracker::UniquePtr object_tracking_;

  rclcpp::Subscription<ufil_msgs::msg::SensorFOV>::SharedPtr fov_subscription_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr occ_grid_sub_;

  rclcpp::Time last_callback_time_ros_;

  std::string target_frame_ = "map";

  int number_of_object_in_last_push_ = 0;

  void publish_objects(const std_msgs::msg::Header & header)
  {
    // Convert tracked objects back to ROS message format
    ufil_msgs::msg::ObjectList tracked_objects =
      ufil_ros::tracksToMsg(this->object_tracking_->tracks(), true, 0.2);

    if(number_of_object_in_last_push_ == 0 && tracked_objects.objects.size() == 0) {
      return;
    }
    number_of_object_in_last_push_ = tracked_objects.objects.size();

    tracked_objects.header = header;

    ufil_msgs::msg::ObjectList transformed_ufil_msg;
    try {
      transformed_ufil_msg = this->tf_buffer_->transform(tracked_objects, target_frame_);
    } catch (tf2::TransformException & ex) {
      RCLCPP_ERROR(this->get_logger(), "Could not transform: %s", ex.what());
      return;
    }

    this->track_publisher_->publish(transformed_ufil_msg);


    // Convert tracked objects back to ROS message format
    ufil_msgs::msg::ObjectList unassociated_measurements_list =
      ufil_ros::measurementsToMsg<ufil_msgs::msg::ObjectList>(
      this->object_tracking_->unassociatedMeasurements());
    unassociated_measurements_list.header = header;
    this->unassociated_measurements_publisher->publish(unassociated_measurements_list);

    ufil_msgs::msg::ObjectList associated_measurement_msg;
    associated_measurement_msg.header = header;
    for (const auto & measurement_with_id : this->object_tracking_->associations()) {
      const ufil::type::measurement::Pose2DWithDimension3D & measurement =
        measurement_with_id.second;
      const ufil_msgs::msg::Object output_object =
        ufil_ros::measurementToMsg<ufil_msgs::msg::Object>(measurement);
      associated_measurement_msg.objects.push_back(output_object);
    }
    this->associated_measurements_publisher_->publish(associated_measurement_msg);

    RCLCPP_INFO(
      this->get_logger(),
      "Association counts total: detector=%zu associator=%zu",
      this->object_tracking_->totalDetectorAssociationCount(),
      this->object_tracking_->totalAssociatorAssociationCount());
  }

  void topic_callback_sensor_fov(const ufil_msgs::msg::SensorFOV & msg)
  {
    ufil::type::SensorFOV fov = ufil_ros::sensorFovFromMsg(msg);
    this->object_tracking_->setCurrentFOV(fov);
  }

  void topic_callback_point_cloud(const sensor_msgs::msg::PointCloud2 & msg)
  {
    // Update last callback time to calucate
    rclcpp::Time msg_time_ros = msg.header.stamp;
    const double delta_t_ros = (msg_time_ros - this->last_callback_time_ros_).seconds();
    this->last_callback_time_ros_ = msg_time_ros;

    // Check if bag file restarted and reset tracking
    if (delta_t_ros > 10.0) {
      RCLCPP_WARN(this->get_logger(), "Detecting time jump, resetting tracking with dt:%f.",
        delta_t_ros);
      this->reset();
    }
    if (delta_t_ros < 0.0) {
      RCLCPP_WARN(this->get_logger(), "Detecting wrong order of messages, ignore with dt:%f.",
        delta_t_ros);
      return;
    }

    pcl::PCLPointCloud2 cloud;
    pcl_conversions::toPCL(msg, cloud);

    ufil_osn_tracker::Detection detection(&cloud);
    std::set<ufil_osn_tracker::Detection> detections;
    detections.insert(detection);

    int cluster_min_points = static_cast<float>(this->get_parameter("cluster_min_points").as_int());
    int cluster_max_points = static_cast<float>(this->get_parameter("cluster_max_points").as_int());
    // RCLCPP_INFO(this->get_logger(), "Setting cluster range to %i %i", cluster_min_points,
    // cluster_max_points);
    this->object_tracking_->setClusterPointRange(cluster_min_points, cluster_max_points);

    double cluster_tolerance =
      static_cast<float>(this->get_parameter("cluster_tolerance").as_double());
    // RCLCPP_INFO(this->get_logger(), "Setting cluster tolerance %f", cluster_tolerance);
    this->object_tracking_->setClusterTolerance(cluster_tolerance);

    float dimension_covariance_mulitplier =
      static_cast<float>(this->get_parameter("dimension_covariance_mulitplier").as_double());
    this->object_tracking_->setDimensionCovarianceMultiplier(dimension_covariance_mulitplier);

    float position_covariance_multiplier =
      static_cast<float>(this->get_parameter("position_covariance_multiplier").as_double());
    this->object_tracking_->setPositionCovarianceMultiplier(position_covariance_multiplier);

    float orientation_covariance_multiplier =
      static_cast<float>(this->get_parameter("orientation_covariance_multiplier").as_double());
    this->object_tracking_->setOrientationCovarianceMultiplier(orientation_covariance_multiplier);

    auto msg_time_ufil = ufil::from_nanoseconds<ufil::type::Timestamp>(msg_time_ros.nanoseconds());
    this->object_tracking_->update(std::move(detections), std::move(msg_time_ufil));

    this->publish_objects(msg.header);
  }

public:
  PclObjectTrackerNode()
  : Node("ufil_osn_tracker")
  {
    this->point_cloud_subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "cloud", 10,
      std::bind(&PclObjectTrackerNode::topic_callback_point_cloud, this, std::placeholders::_1));
    this->track_publisher_ = this->create_publisher<ufil_msgs::msg::ObjectList>("tracks", 10);
    this->unassociated_measurements_publisher =
      this->create_publisher<ufil_msgs::msg::ObjectList>("unassociated_measurements", 10);
    this->associated_measurements_publisher_ =
      this->create_publisher<ufil_msgs::msg::ObjectList>("associated_measurements", 10);

    this->fov_subscription_ = this->create_subscription<ufil_msgs::msg::SensorFOV>(
      "sensor_fov", 10,
      std::bind(&PclObjectTrackerNode::topic_callback_sensor_fov, this, std::placeholders::_1));


    this->occ_grid_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
      "dynamic_occlusion_grid", 10, [this](const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
      {
        this->object_tracking_->setOcclusionGrid(
          std::shared_ptr<const nav_msgs::msg::OccupancyGrid>(msg));
      });

    // TF listener
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // std::string package_share_directory =
    //   ament_index_cpp::get_package_share_directory("ufil_osn_tracker");
    // std::string path_occluded = package_share_directory + "/config/costmap_occluded.png";
    // this->declare_parameter("occluded_costmap_file", path_occluded);
    // std::string path_visible = package_share_directory + "/config/costmap_visible.png";
    // this->declare_parameter("visible_costmap_file", path_visible);


    this->declare_parameter("target_frame", this->target_frame_);
    this->target_frame_ = this->get_parameter("target_frame").as_string();
    RCLCPP_INFO(
      this->get_logger(), "Transforming output to target frame %s.",
      this->target_frame_.c_str());

    auto dimension_covariance_mulitplier_desc = rcl_interfaces::msg::ParameterDescriptor{};
    dimension_covariance_mulitplier_desc.description =
      "Multiplier for the dimension covariance for the detector.";
    this->declare_parameter("dimension_covariance_mulitplier", 10.0,
      dimension_covariance_mulitplier_desc);

    auto position_covariance_multiplier_desc = rcl_interfaces::msg::ParameterDescriptor{};
    position_covariance_multiplier_desc.description =
      "Multiplier for the position covariance for the detector.";
    this->declare_parameter("position_covariance_multiplier", 0.5,
      position_covariance_multiplier_desc);

    auto orientation_covariance_multiplier_desc = rcl_interfaces::msg::ParameterDescriptor{};
    orientation_covariance_multiplier_desc.description =
      "Multiplier for the orientation covariance for the detector.";
    this->declare_parameter("orientation_covariance_multiplier", 0.01,
      orientation_covariance_multiplier_desc);

    auto association_threshold_desc = rcl_interfaces::msg::ParameterDescriptor{};
    association_threshold_desc.description =
      "Threshold for the maximum assoication distance between measurement and track.";
    this->declare_parameter("association_threshold", 4.0, association_threshold_desc);

    auto history_length_desc = rcl_interfaces::msg::ParameterDescriptor{};
    history_length_desc.description = "Length of the history stored in a track.";
    this->declare_parameter("history_length", 10.0, history_length_desc);

    auto cluster_tolerance_desc = rcl_interfaces::msg::ParameterDescriptor{};
    cluster_tolerance_desc.description = "Tolerance for distance-based clustering.";
    this->declare_parameter("cluster_tolerance", 0.5, cluster_tolerance_desc);

    auto cluster_min_points_desc = rcl_interfaces::msg::ParameterDescriptor{};
    cluster_min_points_desc.description =
      "Min number of points in cluster.";
    this->declare_parameter("cluster_min_points", 50, cluster_min_points_desc);

    auto cluster_max_points_desc = rcl_interfaces::msg::ParameterDescriptor{};
    cluster_max_points_desc.description = "Max number of points in cluster.";
    this->declare_parameter("cluster_max_points", 50000, cluster_max_points_desc);
  }

  void reset()
  {
    this->clear();
    this->initialise();
  }

  void initialise()
  {
    this->object_tracking_ = std::make_unique<ufil_osn_tracker::OsnObjectTracker>();

    // std::string occluded_costmap_file = this->get_parameter("occluded_costmap_file").as_string();
    // RCLCPP_INFO(
    //   this->get_logger(), "Loading occluded costmap from file %s",
    //   occluded_costmap_file.c_str());

    // std::string visible_costmap_file = this->get_parameter("visible_costmap_file").as_string();
    // RCLCPP_INFO(
    //   this->get_logger(), "Loading visible costmap from file %s",
    //   visible_costmap_file.c_str());

    // std::filesystem::path occluded_costmap_path = occluded_costmap_file;
    // std::filesystem::path visible_costmap_path = visible_costmap_file;
    // if (!std::filesystem::exists(occluded_costmap_path) ||
    //   !std::filesystem::exists(visible_costmap_path))
    // {
    //   RCLCPP_WARN(this->get_logger(), "Invalid costmap files");
    // } else {
    //   this->object_tracking_->loadCostmapFromFile(visible_costmap_file, occluded_costmap_file);
    // }

    float dimension_covariance_mulitplier =
      static_cast<float>(this->get_parameter("dimension_covariance_mulitplier").as_double());
    this->object_tracking_->setDimensionCovarianceMultiplier(dimension_covariance_mulitplier);

    float position_covariance_multiplier =
      static_cast<float>(this->get_parameter("position_covariance_multiplier").as_double());
    this->object_tracking_->setPositionCovarianceMultiplier(position_covariance_multiplier);

    float orientation_covariance_multiplier =
      static_cast<float>(this->get_parameter("orientation_covariance_multiplier").as_double());
    this->object_tracking_->setOrientationCovarianceMultiplier(orientation_covariance_multiplier);

    float association_threshold =
      static_cast<float>(this->get_parameter("association_threshold").as_double());
    RCLCPP_INFO(this->get_logger(), "Setting association threshold to %f", association_threshold);
    this->object_tracking_->setAssociationThreshold(association_threshold);

    float history_length = static_cast<float>(this->get_parameter("history_length").as_double());
    RCLCPP_INFO(this->get_logger(), "Setting history length to %f", history_length);
    this->object_tracking_->setHistoryLength(history_length);

    this->last_callback_time_ros_ = this->now();
  }

  void clear()
  {
  }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<PclObjectTrackerNode>();
  node->initialise();

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
