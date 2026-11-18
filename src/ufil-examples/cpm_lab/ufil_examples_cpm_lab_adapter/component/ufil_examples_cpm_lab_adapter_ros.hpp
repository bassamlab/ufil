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

#ifndef CPM_LAB__UFIL_EXAMPLES_CPM_LAB_ADAPTER__COMPONENT__UFIL_EXAMPLES_CPM_LAB_ADAPTER_ROS_HPP_
#define CPM_LAB__UFIL_EXAMPLES_CPM_LAB_ADAPTER__COMPONENT__UFIL_EXAMPLES_CPM_LAB_ADAPTER_ROS_HPP_

// Conversions between the ROS 2 messages and the types of the core library.

#include <Eigen/Geometry>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

#include <cstdint>
#include <string>

#include <builtin_interfaces/msg/time.hpp>
#include <cpm_lab_lab_msgs/msg/vehicle_state.hpp>
#include <cpm_lab_map_msgs/msg/lane_control_light.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/time.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/nav_sat_status.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <ufil_msgs/msg/classification.hpp>
#include <ufil_msgs/msg/dimension_with_covariance.hpp>
#include <ufil_msgs/msg/object.hpp>
#include <ufil_msgs/msg/state_with_covariance.hpp>
#include <ufil_object_tracking/types/dimension.hpp>
#include <ufil_object_tracking/types/state.hpp>
#include <ufil_ros/ufil_ros.hpp>

#include "ufil_examples_cpm_lab_adapter/lane_control_light.hpp"
#include "ufil_examples_cpm_lab_adapter/sensor_simulator.hpp"
#include "ufil_examples_cpm_lab_adapter/vehicle_object.hpp"
#include "ufil_examples_cpm_lab_adapter/vehicle_state.hpp"

namespace ufil_examples_cpm_lab_adapter::conversions
{

inline std::int64_t nanosecondsFromMsg(const builtin_interfaces::msg::Time & stamp)
{
  return rclcpp::Time(stamp).nanoseconds();
}

inline builtin_interfaces::msg::Time stampToMsg(std::int64_t nanoseconds)
{
  return rclcpp::Time(nanoseconds);
}

/// Rotation around the z axis [rad].
inline double yawFromMsg(const geometry_msgs::msg::Quaternion & msg)
{
  tf2::Quaternion quaternion;
  tf2::fromMsg(msg, quaternion);
  double roll = 0.0;
  double pitch = 0.0;
  double yaw = 0.0;
  tf2::Matrix3x3(quaternion).getEulerYPR(yaw, pitch, roll);
  return yaw;
}

inline geometry_msgs::msg::Quaternion yawToMsg(double yaw)
{
  tf2::Quaternion quaternion;
  quaternion.setRPY(0.0, 0.0, yaw);
  return tf2::toMsg(quaternion);
}

inline Eigen::Vector3d vectorFromMsg(const geometry_msgs::msg::Vector3 & msg)
{
  return {msg.x, msg.y, msg.z};
}

inline geometry_msgs::msg::Vector3 vectorToMsg(const Eigen::Vector3d & vector)
{
  geometry_msgs::msg::Vector3 msg;
  msg.x = vector.x();
  msg.y = vector.y();
  msg.z = vector.z();
  return msg;
}

inline VehicleState vehicleStateFromMsg(const cpm_lab_lab_msgs::msg::VehicleState & msg)
{
  VehicleState state;
  state.stamp_nanoseconds = nanosecondsFromMsg(msg.header.stamp);
  state.frame_id = msg.header.frame_id;
  state.position = {msg.pose.position.x, msg.pose.position.y, msg.pose.position.z};
  state.yaw = yawFromMsg(msg.pose.orientation);
  state.linear_velocity = vectorFromMsg(msg.twist.linear);
  state.angular_velocity = vectorFromMsg(msg.twist.angular);
  state.linear_acceleration = vectorFromMsg(msg.accel.linear);
  return state;
}

/// A car that is observed from all sides.
inline ufil_msgs::msg::Object objectToMsg(const VehicleObject & object)
{
  ufil_msgs::msg::Object msg;
  msg.id = object.id;
  msg.existence_probability = object.existence_probability;
  msg.classification.classification[ufil_msgs::msg::Classification::CAR] = 1.0;
  msg.features.fl = true;
  msg.features.fr = true;
  msg.features.bl = true;
  msg.features.br = true;
  msg.features.f = true;
  msg.features.b = true;
  msg.features.l = true;
  msg.features.r = true;
  msg.features.c = true;

  ufil::type::state::PoseVelocity2D state;
  state.x() = object.x;
  state.y() = object.y;
  state.vx() = object.vx;
  state.vy() = object.vy;
  state.yaw() = object.yaw;
  state.covariance().setZero();
  for (std::size_t i = 0; i < object.state_variances.size(); ++i) {
    const auto index = static_cast<Eigen::Index>(i);
    state.covariance()(index, index) = object.state_variances[i];
  }
  msg.state = ufil_ros::stateToMsg<ufil_msgs::msg::StateWithCovariance>(state);

  ufil::type::dimension::Dimension3D dimension;
  dimension.length() = object.length;
  dimension.width() = object.width;
  dimension.height() = object.height;
  dimension.covariance().setZero();
  dimension.covariance().diagonal() << object.dimension_variances[0],
    object.dimension_variances[1], object.dimension_variances[2];
  msg.dimension = ufil_ros::dimensionToMsg<ufil_msgs::msg::DimensionWithCovariance>(dimension);
  return msg;
}

inline LaneControlLight laneControlLightFromMsg(const cpm_lab_map_msgs::msg::LaneControlLight & msg)
{
  LaneControlLight lights;
  lights.stamp = rclcpp::Time(msg.header.stamp).seconds();
  lights.frame_id = msg.header.frame_id;
  lights.segments.reserve(msg.segments.size());
  for (const auto & segment : msg.segments) {
    lights.segments.push_back({segment.segment_index, segment.image_name});
  }
  return lights;
}

inline Eigen::Isometry3d isometryFromMsg(const geometry_msgs::msg::TransformStamped & msg)
{
  return tf2::transformToEigen(msg);
}

inline sensor_msgs::msg::NavSatFix navSatFixToMsg(
  const GnssFix & fix, const builtin_interfaces::msg::Time & stamp, const std::string & frame_id)
{
  sensor_msgs::msg::NavSatFix msg;
  msg.header.stamp = stamp;
  msg.header.frame_id = frame_id;
  msg.status.status = sensor_msgs::msg::NavSatStatus::STATUS_FIX;
  msg.status.service = sensor_msgs::msg::NavSatStatus::SERVICE_GPS;
  msg.latitude = fix.latitude;
  msg.longitude = fix.longitude;
  msg.altitude = 0.0;
  msg.position_covariance_type = sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_DIAGONAL_KNOWN;
  msg.position_covariance[0] = fix.position_variance;
  msg.position_covariance[4] = fix.position_variance;
  msg.position_covariance[8] = fix.position_variance;
  return msg;
}

inline sensor_msgs::msg::Imu imuToMsg(
  const ImuMeasurement & measurement, const builtin_interfaces::msg::Time & stamp,
  const std::string & frame_id)
{
  sensor_msgs::msg::Imu msg;
  msg.header.stamp = stamp;
  msg.header.frame_id = frame_id;
  msg.orientation = yawToMsg(measurement.yaw);
  msg.angular_velocity = vectorToMsg(measurement.angular_velocity);
  msg.linear_acceleration = vectorToMsg(measurement.linear_acceleration);
  return msg;
}

inline nav_msgs::msg::Odometry odometryToMsg(
  const WheelOdometry & odometry, const builtin_interfaces::msg::Time & stamp,
  const std::string & frame_id, const std::string & child_frame_id)
{
  nav_msgs::msg::Odometry msg;
  msg.header.stamp = stamp;
  msg.header.frame_id = frame_id;
  msg.child_frame_id = child_frame_id;
  msg.twist.twist.linear = vectorToMsg(odometry.linear_velocity);
  return msg;
}

}  // namespace ufil_examples_cpm_lab_adapter::conversions

#endif  // CPM_LAB__UFIL_EXAMPLES_CPM_LAB_ADAPTER__COMPONENT__UFIL_EXAMPLES_CPM_LAB_ADAPTER_ROS_HPP_
