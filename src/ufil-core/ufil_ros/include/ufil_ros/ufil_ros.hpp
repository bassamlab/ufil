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


#ifndef UFIL_ROS__UFIL_ROS_HPP_
#define UFIL_ROS__UFIL_ROS_HPP_

#include <tf2/exceptions.h>
#include <tf2/convert.h>
#include <tf2_ros/buffer_interface.h>

#include <array>
#include <cmath>
#include <set>
#include <string>
#include <utility>
#include <vector>
#include <algorithm>

#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>

#include <ufil_msgs/msg/object.hpp>
#include <ufil_msgs/msg/object_stamped.hpp>
#include <ufil_msgs/msg/object_list.hpp>
#include <ufil_msgs/msg/sensor_fov.hpp>

#include <ufil_object_tracking/types/classification.hpp>
#include <ufil_object_tracking/types/dimension.hpp>
#include <ufil_object_tracking/types/id.hpp>
#include <ufil_object_tracking/types/occupancy_grid.hpp>
#include <ufil_object_tracking/types/measurement.hpp>
#include <ufil_object_tracking/types/sensor_fov.hpp>
#include <ufil_object_tracking/types/state.hpp>
#include <ufil_object_tracking/types/track.hpp>

#include "ufil_ros/visibility_control.h"

namespace ufil_ros
{

template<typename R, typename V>
R classificationToMsg(const V & value);

template<typename R, typename V>
R classificationFromMsg(const V & value);

template<typename R, typename V>
R measurementToMsg(const V & value);

template<typename R, typename V>
R measurementFromMsg(const V & value);

template<typename R, typename V>
std::set<R> measurementsFromMsg(const V & value);

template<typename R, typename V>
R measurementsToMsg(const std::set<V> & value);

template<typename R, typename V>
R dimensionToMsg(const V & value);

template<typename R, typename V>
R dimensionFromMsg(const V & value);

template<typename R, typename V>
R stateToMsg(const V & value);

template<typename R, typename V>
R stateFromMsg(const V & value);

ufil_msgs::msg::SensorFOV sensorFovToMsg(const ufil::type::SensorFOV & fov);
ufil::type::SensorFOV sensorFovFromMsg(const ufil_msgs::msg::SensorFOV & msg);

nav_msgs::msg::OccupancyGrid occupancyGridToMsg(const ufil::type::OccupancyGrid & value);
ufil::type::OccupancyGrid occupancyGridFromMsg(const nav_msgs::msg::OccupancyGrid & value);

ufil_msgs::msg::Object trackToMsg(const auto & value)
{
  ufil_msgs::msg::Object result;
  result.state = stateToMsg<ufil_msgs::msg::StateWithCovariance>(value.currentState());
  result.dimension =
    dimensionToMsg<ufil_msgs::msg::DimensionWithCovariance>(value.currentDimension());
  result.classification =
    classificationToMsg<ufil_msgs::msg::Classification>(value.currentClassification());
  result.id = ufil::hashId<ufil_msgs::msg::Object::_id_type>(value.uuid());
  result.existence_probability = value.currentExistenceProbability().existence();
  return result;
}

template<typename TrackType>
TrackType trackFromMsg(const ufil_msgs::msg::Object & value)
{
  TrackType result;
  result.uuid(value.id);

  auto state = stateFromMsg<typename TrackType::StateType>(value.state);
  auto dimension = dimensionFromMsg<typename TrackType::DimensionType>(value.dimension);
  auto classification = classificationFromMsg<typename TrackType::ClassificationType>(
    value.classification);

  typename TrackType::HistoryEntryType entry(
    state, dimension, typename TrackType::ExistenceProbabilityType(), classification);
  entry.existenceProbability().existence() = value.existence_probability;

  result.insert(std::move(entry));
  return result;
}

ufil_msgs::msg::ObjectList tracksToMsg(
  const auto & value, bool perform_validation = true,
  double min_existence_probability = 0.0)
{
  ufil_msgs::msg::ObjectList result;
  for (const auto & track_with_id : value) {
    const auto & track = track_with_id.second;

    if (!track.valid() && perform_validation) {
      continue;
    }

    if(track.currentExistenceProbability().existence() < min_existence_probability) {
      continue;
    }

    ufil_msgs::msg::Object output_object = ufil_ros::trackToMsg(track);
    result.objects.push_back(output_object);
  }

  return result;
}

template<typename TrackType>
std::vector<TrackType> tracksFromMsg(const ufil_msgs::msg::ObjectList & value)
{
  std::vector<TrackType> result;
  result.reserve(value.objects.size());
  for (const auto & object : value.objects) {
    result.push_back(trackFromMsg<TrackType>(object));
  }
  return result;
}

}  // namespace ufil_ros


namespace tf2
{

template<>
inline void doTransform(
  const ufil_msgs::msg::ObjectList & data_in, ufil_msgs::msg::ObjectList & data_out,
  const geometry_msgs::msg::TransformStamped & transform_msg)
{
  Eigen::Affine3d transform;
  auto transform_iso = tf2::transformToEigen(transform_msg);
  transform = Eigen::Affine3d(transform_iso.matrix());

  for(auto & object_in : data_in.objects) {
    ufil_msgs::msg::Object object_out = object_in;

    Eigen::Vector3d position{object_in.state.state.x, object_in.state.state.y, 0.0f};
    position = transform * position;

    object_out.state.state.x = position.x();
    object_out.state.state.y = position.y();

    Eigen::Vector3d velocity{object_in.state.state.v_x, object_in.state.state.v_y, 0};
    velocity = transform.linear() * velocity;

    object_out.state.state.v_x = velocity.x();
    object_out.state.state.v_y = velocity.y();

    Eigen::Vector3d acceleration{object_in.state.state.a_x, object_in.state.state.a_y, 0};
    acceleration = transform.linear() * acceleration;

    object_out.state.state.a_x = acceleration.x();
    object_out.state.state.a_y = acceleration.y();

    Eigen::Matrix2d R = transform.linear().block<2, 2>(0, 0);
    double delta_yaw = std::atan2(R(1, 0), R(0, 0));
    object_out.state.state.yaw = object_in.state.state.yaw + delta_yaw;

    Eigen::Matrix<double, 8, 8> J = Eigen::Matrix<double, 8, 8>::Zero();
    // x,y
    J.block<2, 2>(0, 0) = R;
    // v_x, v_y
    J.block<2, 2>(2, 2) = R;
    // a_x, a_y
    J.block<2, 2>(4, 4) = R;
    // yaw
    J(6, 6) = 1.0;
    // yaw_rate
    J(7, 7) = 1.0;

    // Build P_in
    Eigen::Matrix<double, 8, 8> P_in = Eigen::Matrix<double, 8, 8>::Zero();
    std::array<bool, 8> invalid{};

    for (int i = 0; i < 8; ++i) {
      if (object_in.state.covariance[i * 8 + i] < 0) {
        invalid[i] = true;
      }
      for (int j = 0; j < 8; ++j) {
        double val = object_in.state.covariance[i * 8 + j];
        if (invalid[i] || invalid[j]) {val = 0.0;}  // zero out invalid row/col
        P_in(i, j) = val;
      }
    }

    // Transform
    Eigen::Matrix<double, 8, 8> P_out = J * P_in * J.transpose();

    // Restore -1 markers
    for (int i = 0; i < 8; ++i) {
      for (int j = 0; j < 8; ++j) {
        object_out.state.covariance[i * 8 + j] = P_out(i, j);
      }
      if (invalid[i]) {
        object_out.state.covariance[i * 8 + i] = -1.0;
      }
    }

    data_out.objects.push_back(object_out);
  }

  data_out.header = data_in.header;
  data_out.header.frame_id = transform_msg.header.frame_id;
}

template<>
inline void doTransform(
  const ufil_msgs::msg::ObjectStamped & data_in, ufil_msgs::msg::ObjectStamped & data_out,
  const geometry_msgs::msg::TransformStamped & transform_msg)
{
  Eigen::Affine3d transform;
  auto transform_iso = tf2::transformToEigen(transform_msg);
  transform = Eigen::Affine3d(transform_iso.matrix());

  ufil_msgs::msg::Object object_in = data_in.object;

  ufil_msgs::msg::Object object_out = object_in;

  Eigen::Vector3d position{object_in.state.state.x, object_in.state.state.y, 0.0f};
  position = transform * position;

  object_out.state.state.x = position.x();
  object_out.state.state.y = position.y();

  Eigen::Vector3d velocity{object_in.state.state.v_x, object_in.state.state.v_y, 0};
  velocity = transform.linear() * velocity;

  object_out.state.state.v_x = velocity.x();
  object_out.state.state.v_y = velocity.y();

  Eigen::Vector3d acceleration{object_in.state.state.a_x, object_in.state.state.a_y, 0};
  acceleration = transform.linear() * acceleration;

  object_out.state.state.a_x = acceleration.x();
  object_out.state.state.a_y = acceleration.y();

  Eigen::Matrix2d R = transform.linear().block<2, 2>(0, 0);
  double delta_yaw = std::atan2(R(1, 0), R(0, 0));
  object_out.state.state.yaw = object_in.state.state.yaw + delta_yaw;

  Eigen::Matrix<double, 8, 8> J = Eigen::Matrix<double, 8, 8>::Zero();
  // x,y
  J.block<2, 2>(0, 0) = R;
  // v_x, v_y
  J.block<2, 2>(2, 2) = R;
  // a_x, a_y
  J.block<2, 2>(4, 4) = R;
  // yaw
  J(6, 6) = 1.0;
  // yaw_rate
  J(7, 7) = 1.0;

  // Build P_in
  Eigen::Matrix<double, 8, 8> P_in = Eigen::Matrix<double, 8, 8>::Zero();
  std::array<bool, 8> invalid{};

  for (int i = 0; i < 8; ++i) {
    if (object_in.state.covariance[i * 8 + i] < 0) {
      invalid[i] = true;
    }
    for (int j = 0; j < 8; ++j) {
      double val = object_in.state.covariance[i * 8 + j];
      if (invalid[i] || invalid[j]) {val = 0.0;}  // zero out invalid row/col
      P_in(i, j) = val;
    }
  }

  // Transform
  Eigen::Matrix<double, 8, 8> P_out = J * P_in * J.transpose();

  // Restore -1 markers
  for (int i = 0; i < 8; ++i) {
    for (int j = 0; j < 8; ++j) {
      object_out.state.covariance[i * 8 + j] = P_out(i, j);
    }
    if (invalid[i]) {
      object_out.state.covariance[i * 8 + i] = -1.0;
    }
  }

  data_out.object = object_out;


  data_out.header = data_in.header;
  data_out.header.frame_id = transform_msg.child_frame_id;
}

template<>
inline void doTransform(
  const ufil_msgs::msg::SensorFOV & fov_in,
  ufil_msgs::msg::SensorFOV & fov_out,
  const geometry_msgs::msg::TransformStamped & transform_msg)
{
  Eigen::Affine3d T = tf2::transformToEigen(transform_msg);

    // Transform origin
  Eigen::Vector3d origin(fov_in.origin.x, fov_in.origin.y, fov_in.origin.z);
  Eigen::Vector3d origin_t = T * origin;
  fov_out.origin.x = static_cast<float>(origin_t.x());
  fov_out.origin.y = static_cast<float>(origin_t.y());
  fov_out.origin.z = static_cast<float>(origin_t.z());

    // Rotation part
  Eigen::Matrix3d R = T.linear();

  auto localDir = [](double phi, double theta) -> Eigen::Vector3d {
      double cth = std::cos(theta);
      return Eigen::Vector3d(cth * std::cos(phi), cth * std::sin(phi), std::sin(theta));
    };

  auto toAngles = [](const Eigen::Vector3d & d) -> std::pair<double, double> {
      Eigen::Vector3d dn = d.normalized();
      double phi = std::atan2(dn.y(), dn.x());
      double theta = std::atan2(dn.z(), std::sqrt(dn.x() * dn.x() + dn.y() * dn.y()));
      return {phi, theta};
    };

  std::array<Eigen::Vector3d, 4> dirs = {
    localDir(fov_in.phi_min, fov_in.theta_min),
    localDir(fov_in.phi_min, fov_in.theta_max),
    localDir(fov_in.phi_max, fov_in.theta_min),
    localDir(fov_in.phi_max, fov_in.theta_max)
  };

  std::vector<double> phis, thetas;
  phis.reserve(4); thetas.reserve(4);

  for (const auto & d : dirs) {
    Eigen::Vector3d dg = R * d;
    auto [p, t] = toAngles(dg);
    if (p > M_PI) {p -= 2.0 * M_PI;}
    if (p <= -M_PI) {p += 2.0 * M_PI;}
    phis.push_back(p);
    thetas.push_back(t);
  }

  // Extract roll/pitch/yaw (RPY) from the transform rotation and apply yaw to
  // the horizontal center and pitch to the vertical center while preserving
  // input angular spans. The previous approach recomputed phi from fully
  // rotated directions which means pitch will change phi values (rotation
  // mixes axes), causing the failures you observed.
  tf2::Quaternion q(transform_msg.transform.rotation.x,
    transform_msg.transform.rotation.y,
    transform_msg.transform.rotation.z,
    transform_msg.transform.rotation.w);
  double roll = 0.0, pitch = 0.0, yaw = 0.0;
  tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

  double phi_span = static_cast<double>(fov_in.phi_max) - static_cast<double>(fov_in.phi_min);
  if (phi_span < 0) {phi_span += 2.0 * M_PI;}
  double phi_center = static_cast<double>(fov_in.phi_min) + 0.5 * phi_span;
  double phi_center_out = phi_center + yaw;
  double phi_min = phi_center_out - 0.5 * phi_span;
  double phi_max = phi_center_out + 0.5 * phi_span;

  double theta_span = static_cast<double>(fov_in.theta_max) - static_cast<double>(fov_in.theta_min);
  double theta_center = static_cast<double>(fov_in.theta_min) + 0.5 * theta_span;
  double theta_center_out = theta_center + pitch;
  double theta_min = theta_center_out - 0.5 * theta_span;
  double theta_max = theta_center_out + 0.5 * theta_span;

  // Normalize to (-pi, pi]
  auto normalize_angle_inplace = [](double & a){
      while (a > M_PI) {
        a -= 2.0 * M_PI;
      }
      while (a <= -M_PI) {
        a += 2.0 * M_PI;
      }
    };
  normalize_angle_inplace(phi_min);
  normalize_angle_inplace(phi_max);
  normalize_angle_inplace(theta_min);
  normalize_angle_inplace(theta_max);

  fov_out.phi_min = static_cast<float>(phi_min);
  fov_out.phi_max = static_cast<float>(phi_max);
  fov_out.theta_min = static_cast<float>(theta_min);
  fov_out.theta_max = static_cast<float>(theta_max);

  fov_out.r_min = fov_in.r_min;
  fov_out.r_max = fov_in.r_max;

    // Covariance
  Eigen::Matrix<double, 6, 6> P;
  for (int i = 0; i < 6; ++i) {
    for (int j = 0; j < 6; ++j) {
      P(i, j) = fov_in.covariance[i * 6 + j];
    }
  }

  Eigen::Matrix<double, 6, 6> J = Eigen::Matrix<double, 6, 6>::Identity();
  Eigen::Matrix<double, 6, 6> P_out = J * P * J.transpose();

  for (int i = 0; i < 6; ++i) {
    for (int j = 0; j < 6; ++j) {
      fov_out.covariance[i * 6 + j] = static_cast<float>(P_out(i, j));
    }
  }

  fov_out.header = fov_in.header;
  fov_out.header.frame_id = transform_msg.header.frame_id;
}


template<>
inline tf2::TimePoint getTimestamp(const ufil_msgs::msg::ObjectList & msg)
{
  return tf2::timeFromSec(
    static_cast<double>(msg.header.stamp.sec) +
    1e-9 * static_cast<double>(msg.header.stamp.nanosec));
}

template<>
inline tf2::TimePoint getTimestamp(const ufil_msgs::msg::ObjectStamped & msg)
{
  return tf2::timeFromSec(
    static_cast<double>(msg.header.stamp.sec) +
    1e-9 * static_cast<double>(msg.header.stamp.nanosec));
}

template<>
inline tf2::TimePoint getTimestamp(const ufil_msgs::msg::SensorFOV & msg)
{
  return tf2::timeFromSec(
    static_cast<double>(msg.header.stamp.sec) +
    1e-9 * static_cast<double>(msg.header.stamp.nanosec));
}

template<>
inline std::string getFrameId(const ufil_msgs::msg::ObjectList & msg)
{
  return msg.header.frame_id;
}

template<>
inline std::string getFrameId(const ufil_msgs::msg::ObjectStamped & msg)
{
  return msg.header.frame_id;
}

template<>
inline std::string getFrameId(const ufil_msgs::msg::SensorFOV & msg)
{
  return msg.header.frame_id;
}


}  // namespace tf2

#endif  // UFIL_ROS__UFIL_ROS_HPP_
