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

#ifndef SSL_WHEEL_TRACKER__COMPONENT__SSL_WHEEL_TRACKER_ROS_HPP_
#define SSL_WHEEL_TRACKER__COMPONENT__SSL_WHEEL_TRACKER_ROS_HPP_

// Conversions between ROS 2 messages and the ROS-independent ssl_wheel_tracker types.

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

#include <builtin_interfaces/msg/time.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/transform.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <rclcpp/time.hpp>
#include <ssl_vehicle_tracking_msgs/msg/track_array.hpp>
#include <ssl_vehicle_tracking_msgs/msg/wheel_array.hpp>
#include <ufil_object_tracking/types/time.hpp>

#include "ssl_wheel_tracker/types.hpp"

namespace ssl_wheel_tracker::conversions
{

inline constexpr char kMatchedState[] = "matched";
inline constexpr char kPredictedState[] = "predicted";

inline Timestamp timestampFromRos(const rclcpp::Time & time)
{
  return ufil::from_nanoseconds<Timestamp>(time.nanoseconds());
}

inline builtin_interfaces::msg::Time timestampToMsg(const Timestamp & stamp)
{
  return rclcpp::Time(ufil::to_nanoseconds(stamp), RCL_ROS_TIME);
}

/// Rotation about z of a quaternion [rad].
inline double yawFromMsg(const geometry_msgs::msg::Quaternion & q)
{
  return std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

/// Pressure grid of a mat. Cells are read as unsigned pressure values [0, 255]. The origin is
/// reduced to its position and yaw; mats lie flat in the frame of the grid.
inline PressureGrid gridFromMsg(const nav_msgs::msg::OccupancyGrid & msg)
{
  PressureGrid grid;
  grid.columns = msg.info.width;
  grid.rows = msg.info.height;
  grid.resolution = msg.info.resolution;
  grid.origin.x = msg.info.origin.position.x;
  grid.origin.y = msg.info.origin.position.y;
  grid.origin.yaw = yawFromMsg(msg.info.origin.orientation);
  grid.values.reserve(msg.data.size());
  for (const auto value : msg.data) {
    grid.values.push_back(static_cast<std::uint8_t>(value));
  }
  return grid;
}

/// Applies a transform to a point in the plane z = 0 and projects the result onto the plane.
inline Point2D transformPoint(const geometry_msgs::msg::Transform & transform, const Point2D & p)
{
  const auto & q = transform.rotation;
  // v' = v + 2 w (u x v) + 2 u x (u x v) with u = (q.x, q.y, q.z) and v = (p.x, p.y, 0).
  const double cx = -q.z * p.y;
  const double cy = q.z * p.x;
  const double cz = q.x * p.y - q.y * p.x;
  const double ccx = q.y * cz - q.z * cy;
  const double ccy = q.z * cx - q.x * cz;
  return {
    transform.translation.x + p.x + 2.0 * (q.w * cx + ccx),
    transform.translation.y + p.y + 2.0 * (q.w * cy + ccy)};
}

/**
 * @brief Detected wheels in the tracking frame.
 * @param first_id wheel_id of the first wheel; the following wheels count up.
 */
inline ssl_vehicle_tracking_msgs::msg::WheelArray wheelsToMsg(
  const std::vector<Point2D> & positions, const std::vector<WheelDetection> & detections,
  const builtin_interfaces::msg::Time & stamp, const std::string & frame_id,
  std::uint32_t first_id)
{
  ssl_vehicle_tracking_msgs::msg::WheelArray msg;
  msg.header.stamp = stamp;
  msg.header.frame_id = frame_id;
  msg.wheels.reserve(positions.size());
  for (std::size_t i = 0; i < positions.size() && i < detections.size(); ++i) {
    ssl_vehicle_tracking_msgs::msg::Wheel wheel;
    wheel.wheel_id = static_cast<std::int32_t>(first_id + static_cast<std::uint32_t>(i));
    wheel.x = static_cast<float>(positions[i].x);
    wheel.y = static_cast<float>(positions[i].y);
    wheel.intensity = static_cast<float>(detections[i].pressure);
    msg.wheels.push_back(wheel);
  }
  return msg;
}

/// State with its covariance in the order [x, vx, y, vy], row-major.
inline ssl_vehicle_tracking_msgs::msg::TrackState trackStateToMsg(const TrackState & state)
{
  ssl_vehicle_tracking_msgs::msg::TrackState msg;
  msg.x = static_cast<float>(state.x());
  msg.vx = static_cast<float>(state.vx());
  msg.y = static_cast<float>(state.y());
  msg.vy = static_cast<float>(state.vy());
  for (Eigen::Index row = 0; row < 4; ++row) {
    for (Eigen::Index column = 0; column < 4; ++column) {
      msg.covariance[static_cast<std::size_t>(row * 4 + column)] =
        static_cast<float>(state.covariance(row, column));
    }
  }
  msg.state = state.matched ? kMatchedState : kPredictedState;
  return msg;
}

inline ssl_vehicle_tracking_msgs::msg::Track trackToMsg(const Track & track)
{
  ssl_vehicle_tracking_msgs::msg::Track msg;
  msg.track_id = std::to_string(track.id);
  msg.frames_unseen = static_cast<std::int8_t>(std::min<std::size_t>(track.misses, 127));
  msg.states.reserve(track.history.size());
  for (const auto & state : track.history) {
    msg.states.push_back(trackStateToMsg(state));
  }
  return msg;
}

inline ssl_vehicle_tracking_msgs::msg::TrackArray tracksToMsg(
  const std::vector<Track> & tracks, const builtin_interfaces::msg::Time & stamp,
  const std::string & frame_id)
{
  ssl_vehicle_tracking_msgs::msg::TrackArray msg;
  msg.header.stamp = stamp;
  msg.header.frame_id = frame_id;
  msg.tracks.reserve(tracks.size());
  for (const auto & track : tracks) {
    msg.tracks.push_back(trackToMsg(track));
  }
  return msg;
}

}  // namespace ssl_wheel_tracker::conversions

#endif  // SSL_WHEEL_TRACKER__COMPONENT__SSL_WHEEL_TRACKER_ROS_HPP_
