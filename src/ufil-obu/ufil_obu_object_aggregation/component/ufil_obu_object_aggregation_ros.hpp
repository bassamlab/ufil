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


#ifndef UFIL_OBU_OBJECT_AGGREGATION__COMPONENT__UFIL_OBU_OBJECT_AGGREGATION_ROS_HPP_
#define UFIL_OBU_OBJECT_AGGREGATION__COMPONENT__UFIL_OBU_OBJECT_AGGREGATION_ROS_HPP_

// Conversions between ROS 2 messages and the ROS-independent ufil_obu_object_aggregation types.

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/utils.h>

#include <algorithm>
#include <initializer_list>

#include <builtin_interfaces/msg/time.hpp>
#include <geometry_msgs/msg/accel_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/nav_sat_status.hpp>
#include <ufil_msgs/msg/object_stamped.hpp>

#include "ufil_obu_object_aggregation/object_aggregator.hpp"

namespace ufil_obu_object_aggregation::conversions
{

inline GnssFix fixFromMsg(const sensor_msgs::msg::NavSatFix & msg)
{
  GnssFix fix;
  fix.latitude = msg.latitude;
  fix.longitude = msg.longitude;
  fix.has_fix = msg.status.status != sensor_msgs::msg::NavSatStatus::STATUS_NO_FIX;
  return fix;
}

inline Odometry odometryFromMsg(const nav_msgs::msg::Odometry & msg)
{
  Odometry odometry;
  const auto & q = msg.pose.pose.orientation;
  odometry.yaw = tf2::getYaw(tf2::Quaternion(q.x, q.y, q.z, q.w));
  odometry.v_x = msg.twist.twist.linear.x;
  odometry.v_y = msg.twist.twist.linear.y;
  odometry.yaw_rate = msg.twist.twist.angular.z;
  std::copy(
    msg.pose.covariance.begin(), msg.pose.covariance.end(), odometry.pose_covariance.begin());
  std::copy(
    msg.twist.covariance.begin(), msg.twist.covariance.end(), odometry.twist_covariance.begin());
  return odometry;
}

inline Acceleration accelerationFromMsg(const geometry_msgs::msg::AccelWithCovarianceStamped & msg)
{
  Acceleration acceleration;
  acceleration.a_x = msg.accel.accel.linear.x;
  acceleration.a_y = msg.accel.accel.linear.y;
  std::copy(
    msg.accel.covariance.begin(), msg.accel.covariance.end(), acceleration.covariance.begin());
  return acceleration;
}

/// The latest of the given stamps.
inline builtin_interfaces::msg::Time latestStamp(
  std::initializer_list<builtin_interfaces::msg::Time> stamps)
{
  return *std::max_element(
    stamps.begin(), stamps.end(), [](const auto & a, const auto & b) {
      return (a.sec == b.sec) ? (a.nanosec < b.nanosec) : (a.sec < b.sec);
    });
}

/// The object in the frame of its UTM zone, e.g. "utm_32n". Velocity and acceleration are
/// global, as in every UFIL object state.
inline ufil_msgs::msg::ObjectStamped objectToMsg(
  const OwnObject & object, const builtin_interfaces::msg::Time & stamp)
{
  ufil_msgs::msg::ObjectStamped msg;
  msg.header.stamp = stamp;
  msg.header.frame_id = object.position.frameId();

  auto & out = msg.object;
  out.id = object.id;
  out.state.state.x = object.position.easting;
  out.state.state.y = object.position.northing;
  out.state.state.v_x = object.v_x;
  out.state.state.v_y = object.v_y;
  out.state.state.a_x = object.a_x;
  out.state.state.a_y = object.a_y;
  out.state.state.yaw = object.yaw;
  out.state.state.yaw_rate = object.yaw_rate;
  std::copy(
    object.state_covariance.begin(), object.state_covariance.end(),
    out.state.covariance.begin());

  out.dimension.dimension.length = object.length;
  out.dimension.dimension.width = object.width;
  std::copy(
    object.dimension_covariance.begin(), object.dimension_covariance.end(),
    out.dimension.covariance.begin());

  out.existence_probability = 1.0;
  std::copy(
    object.classification.begin(), object.classification.end(),
    out.classification.classification.begin());

  // The own vehicle is known completely, so all reference points are valid.
  auto & features = out.features;
  features.fl = features.fr = features.bl = features.br = true;
  features.f = features.b = features.l = features.r = features.c = true;
  return msg;
}

}  // namespace ufil_obu_object_aggregation::conversions

#endif  // UFIL_OBU_OBJECT_AGGREGATION__COMPONENT__UFIL_OBU_OBJECT_AGGREGATION_ROS_HPP_
