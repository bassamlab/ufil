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
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <rclcpp/rclcpp.hpp>

#include <cpm_lab_lab_msgs/msg/vehicle_state.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <ufil_msgs/msg/object_list.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>

#include <GeographicLib/UTMUPS.hpp>

#include <ufil_examples_cpm_lab_adapter/artificial_gnss.hpp>

using namespace std::chrono_literals;

  // Convert from ROS quaternion to yaw angle
inline double convertQuaternionToYaw(const geometry_msgs::msg::Quaternion & quaternion)
{
  tf2::Quaternion tf_quat;
  tf2::fromMsg(quaternion, tf_quat);
  double roll{0.0}, pitch{0.0}, yaw{0.0};
  tf2::Matrix3x3(tf_quat).getEulerYPR(yaw, pitch, roll);
  return yaw;
}

/**
 * @class CpmLabAdapterNode
 * @brief ROS2 node to translate vehicle state into GNSS, IMU, and Odometry messages.
 *
 * This node subscribes to a single vehicle's state and publishes its GNSS, IMU, and odometry data.
 */
class CpmLabAdapterNode : public rclcpp::Node
{
public:
  CpmLabAdapterNode()
  : Node("lab_adapter_node")
  {
    id_ = declare_parameter("id", "1");
    frequency_gnss_ = declare_parameter("frequency_gnss", 5.0);
    frequency_odometry_ = declare_parameter("frequency_odometry", 50.0);
    use_artificial_gnss_ = this->declare_parameter("use_artificial_gnss", true);
    this->declare_parameter("target_frame", "map");

    auto qos = rclcpp::QoS(10).reliability(rmw_qos_reliability_policy_from_str("best_effort"));
    vehicle_state_subscriber_ = create_subscription<cpm_lab_lab_msgs::msg::VehicleState>(
        "vehicle_state", qos,
      std::bind(&CpmLabAdapterNode::onVehicleState, this, std::placeholders::_1));

    imu_publisher_ = create_publisher<sensor_msgs::msg::Imu>("imu", 10);
    nav_sat_fix_publisher_ = create_publisher<sensor_msgs::msg::NavSatFix>("gps", 10);
    odometry_publisher_ = create_publisher<nav_msgs::msg::Odometry>("wheel", 10);

    gnss_timer_ = create_wall_timer(std::chrono::milliseconds(static_cast<int>(1000 /
        frequency_gnss_)),
                                    std::bind(&CpmLabAdapterNode::onGnssTimer, this));
    odometry_timer_ = create_wall_timer(std::chrono::milliseconds(static_cast<int>(1000 /
        frequency_odometry_)),
                                        std::bind(&CpmLabAdapterNode::onOdometryTimer, this));


    GeographicLib::UTMUPS::Forward(48.079772, 11.635734, this->zone_, this->northp_,
      this->offset_x_, this->offset_y_);

    if (this->use_artificial_gnss_) {
      this->artificial_gnss_ = std::make_shared<ufil_examples_cpm_lab_adapter::ArtificialGnss>();
    }

    // Transforms
    this->tf_static_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ =
      std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // Imu
    geometry_msgs::msg::TransformStamped t_imu;

    t_imu.header.stamp = this->get_clock()->now();
    t_imu.header.frame_id = "base_link_" + id_;
    t_imu.child_frame_id = "imu_" + id_;

    t_imu.transform.translation.x = 0;
    t_imu.transform.translation.y = 0;
    t_imu.transform.translation.z = 0;
    tf2::Quaternion q_imu;
    q_imu.setRPY(0, 0, 0);
    t_imu.transform.rotation = tf2::toMsg(q_imu);

    tf_static_broadcaster_->sendTransform(t_imu);

    // GNSS
    geometry_msgs::msg::TransformStamped t_gnss;

    t_gnss.header.stamp = this->get_clock()->now();
    t_gnss.header.frame_id = "base_link_" + id_;
    t_gnss.child_frame_id = "gnss_" + id_;

    t_gnss.transform.translation.x = 0;
    t_gnss.transform.translation.y = 0;
    t_gnss.transform.translation.z = 0;
    tf2::Quaternion q_gnss;
    q_gnss.setRPY(0, 0, 0);
    t_gnss.transform.rotation = tf2::toMsg(q_gnss);

    tf_static_broadcaster_->sendTransform(t_gnss);
  }

private:
  /**
   * @brief Callback function for vehicle state updates.
   * @param msg Vehicle state message.
   */
  void onVehicleState(const cpm_lab_lab_msgs::msg::VehicleState::SharedPtr msg)
  {
    last_vehicle_state_ = msg;
  }

  /**
   * @brief Publishes GNSS data at a specified frequency.
   */
  void onGnssTimer()
  {
    if (!last_vehicle_state_) {
      return;
    }

    auto state_stamp = rclcpp::Time(last_vehicle_state_->header.stamp);
    if (this->now() - state_stamp >= rclcpp::Duration(1s)) {
      last_vehicle_state_.reset();
      RCLCPP_WARN(this->get_logger(),
        "Reveived out of date vehicle state. Resseting sensor data creation.");
      return;
    }

    std::string source_frame = last_vehicle_state_->header.frame_id;

    std::string target_frame =
      this->get_parameter("target_frame").as_string();

    Eigen::Affine3d transform;
    try {
      geometry_msgs::msg::TransformStamped transform_message =
        tf_buffer_->lookupTransform(target_frame, source_frame,
                                            tf2::TimePointZero);

      auto transform_iso = tf2::transformToEigen(transform_message);
      transform = Eigen::Affine3d(transform_iso.matrix());
    } catch (const tf2::TransformException & ex) {
      RCLCPP_INFO(this->get_logger(), "Could not transform %s to %s: %s",
                        source_frame.c_str(), target_frame.c_str(), ex.what());
      return;
    }

    Eigen::Vector3d position = use_artificial_gnss_ ?
      artificial_gnss_->generateGnssLock(last_vehicle_state_) :
      Eigen::Vector3d(last_vehicle_state_->pose.position.x, last_vehicle_state_->pose.position.y,
      0);
    // std::cout << position << std::endl;
    position = transform * position * SCALING_FACTOR;
    // std::cout << position << std::endl;
    double latitude, longitude;
    GeographicLib::UTMUPS::Reverse(zone_, northp_, offset_x_ + position[0], offset_y_ + position[1],
      latitude, longitude);

    RCLCPP_INFO(this->get_logger(), "lat %f long %f", latitude, longitude);

    sensor_msgs::msg::NavSatFix nav_sat_fix_msg;
    nav_sat_fix_msg.header.stamp = state_stamp;
    nav_sat_fix_msg.header.frame_id = "gnss_" + id_;
    nav_sat_fix_msg.status.status = sensor_msgs::msg::NavSatStatus::STATUS_FIX;
    nav_sat_fix_msg.status.service = sensor_msgs::msg::NavSatStatus::SERVICE_GPS;
    nav_sat_fix_msg.latitude = latitude;
    nav_sat_fix_msg.longitude = longitude;
    nav_sat_fix_msg.altitude = 0;
    nav_sat_fix_msg.position_covariance_type =
      sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_DIAGONAL_KNOWN;
    nav_sat_fix_msg.position_covariance[0] = use_artificial_gnss_ ? 1.0 : 0.1;
    nav_sat_fix_msg.position_covariance[4] = use_artificial_gnss_ ? 1.0 : 0.1;
    nav_sat_fix_msg.position_covariance[8] = use_artificial_gnss_ ? 1.0 : 0.1;
    nav_sat_fix_publisher_->publish(nav_sat_fix_msg);
  }

  /**
   * @brief Publishes IMU and odometry data.
   */
  void onOdometryTimer()
  {
    if (!last_vehicle_state_) {
      return;
    }

    auto state_stamp = rclcpp::Time(last_vehicle_state_->header.stamp);
    if (this->now() - state_stamp >= rclcpp::Duration(1s)) {
      last_vehicle_state_.reset();
      RCLCPP_WARN(this->get_logger(),
        "Reveived out of date vehicle state. Resseting sensor data creation.");
      return;
    }

    std::string source_frame = last_vehicle_state_->header.frame_id;

    std::string target_frame =
      this->get_parameter("target_frame").as_string();

    Eigen::Affine3d transform;
    try {
      geometry_msgs::msg::TransformStamped transform_message =
        tf_buffer_->lookupTransform(target_frame, source_frame,
                                            tf2::TimePointZero);

      auto transform_iso = tf2::transformToEigen(transform_message);
      transform = Eigen::Affine3d(transform_iso.matrix());
    } catch (const tf2::TransformException & ex) {
      RCLCPP_INFO(this->get_logger(), "Could not transform %s to %s: %s",
                        source_frame.c_str(), target_frame.c_str(), ex.what());
      return;
    }


    // tf2::Quaternion q;
    // q.setRPY(0, 0, last_vehicle_state_->pose.theta);
    // auto orientation = tf2::toMsg(q);

    Eigen::Vector3d euler_angles = transform.linear().eulerAngles(0, 1, 2);
    double yaw = convertQuaternionToYaw(last_vehicle_state_->pose.orientation) + euler_angles.z();

    tf2::Quaternion q;
    q.setRPY(0, 0, yaw);
    auto orientation = tf2::toMsg(q);

    sensor_msgs::msg::Imu imu_msg;
    imu_msg.header.stamp = state_stamp;
    imu_msg.header.frame_id = "imu_" + id_;
    imu_msg.orientation = orientation;
    imu_msg.angular_velocity = last_vehicle_state_->twist.angular;
    imu_msg.linear_acceleration = last_vehicle_state_->accel.linear;
    imu_publisher_->publish(imu_msg);

    nav_msgs::msg::Odometry odometry_msg;
    odometry_msg.header.stamp = state_stamp;
    odometry_msg.header.frame_id = "odom_" + id_;
    odometry_msg.child_frame_id = "base_link_" + id_;

    Eigen::Vector3d velocity{last_vehicle_state_->twist.linear.x,
      last_vehicle_state_->twist.linear.y, 0};
    velocity = velocity * SCALING_FACTOR;

    odometry_msg.twist.twist.linear.x = velocity.x();
    odometry_msg.twist.twist.linear.y = velocity.y();
    odometry_publisher_->publish(odometry_msg);
  }

  static constexpr float SCALING_FACTOR = 18.0;
  bool use_artificial_gnss_;
  std::string id_;

  std::shared_ptr<ufil_examples_cpm_lab_adapter::ArtificialGnss> artificial_gnss_;
  cpm_lab_lab_msgs::msg::VehicleState::SharedPtr last_vehicle_state_;

  // Publishers
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr nav_sat_fix_publisher_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometry_publisher_;
  rclcpp::Publisher<ufil_msgs::msg::ObjectList>::SharedPtr object_list_publisher_;
  // Transforms
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_static_broadcaster_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;

  // Subscriber
  rclcpp::Subscription<cpm_lab_lab_msgs::msg::VehicleState>::SharedPtr vehicle_state_subscriber_;

  // Timers
  rclcpp::TimerBase::SharedPtr gnss_timer_;
  rclcpp::TimerBase::SharedPtr odometry_timer_;
  rclcpp::TimerBase::SharedPtr ground_truth_timer_;

  // UTM Offset Parameters
  double offset_x_ = 0, offset_y_ = 0;
  int zone_ = 0;
  bool northp_ = false;

  // IMU Parameters
  double abs_gyro_drift_ = 0;
  double frequency_gnss_;
  double frequency_odometry_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CpmLabAdapterNode>());
  rclcpp::shutdown();
  return 0;
}
