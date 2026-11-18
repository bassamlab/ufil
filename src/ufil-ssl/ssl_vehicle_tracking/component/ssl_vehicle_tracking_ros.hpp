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

#ifndef SSL_VEHICLE_TRACKING__COMPONENT__SSL_VEHICLE_TRACKING_ROS_HPP_
#define SSL_VEHICLE_TRACKING__COMPONENT__SSL_VEHICLE_TRACKING_ROS_HPP_

// Conversions between ROS 2 messages and the ROS-independent ssl_vehicle_tracking types.

#include <cmath>
#include <cstddef>
#include <utility>
#include <vector>

#include <ssl_vehicle_tracking_msgs/msg/track_array.hpp>
#include <std_msgs/msg/header.hpp>
#include <ufil_msgs/msg/object.hpp>
#include <ufil_msgs/msg/object_list.hpp>

#include "ssl_vehicle_tracking/types.hpp"
#include "ssl_vehicle_tracking/vehicle_template.hpp"

namespace ssl_vehicle_tracking::conversions
{

/// Variance marking a state or dimension component as not estimated.
inline constexpr double kNotEstimated = -1.0;

/**
 * @brief Latest state of every wheel track.
 * @return the tracks and the number of tracks without state or with a non-finite state, which
 *         are ignored
 */
inline std::pair<std::vector<WheelTrack>, std::size_t> wheelTracksFromMsg(
  const ssl_vehicle_tracking_msgs::msg::TrackArray & msg)
{
  std::vector<WheelTrack> tracks;
  std::size_t rejected = 0;
  tracks.reserve(msg.tracks.size());
  for (const auto & track : msg.tracks) {
    if (track.states.empty()) {
      ++rejected;
      continue;
    }
    const auto & state = track.states.back();
    if (!std::isfinite(state.x) || !std::isfinite(state.y) || !std::isfinite(state.vx) ||
      !std::isfinite(state.vy))
    {
      ++rejected;
      continue;
    }
    WheelTrack wheel;
    wheel.id = track.track_id;
    wheel.position = {state.x, state.y};
    wheel.velocity = {state.vx, state.vy};
    wheel.frames_unseen = track.frames_unseen;
    tracks.push_back(wheel);
  }
  return {tracks, rejected};
}

/// Object of a vehicle; position, velocity, yaw, dimension and axles are estimated.
inline ufil_msgs::msg::Object vehicleToMsg(
  const Vehicle & vehicle, const VehicleTemplate & vehicle_template)
{
  const auto & description = vehicle_template.description();
  ufil_msgs::msg::Object msg;
  msg.id = vehicle.id;

  auto & state = msg.state.state;
  state.x = vehicle.position.x;
  state.y = vehicle.position.y;
  state.v_x = vehicle.velocity.x;
  state.v_y = vehicle.velocity.y;
  state.yaw = vehicle.yaw;
  // Fixed variances of x, y, v_x, v_y and yaw; acceleration and yaw rate are not estimated.
  auto & covariance = msg.state.covariance;
  covariance[0] = 0.01;
  covariance[9] = 0.01;
  covariance[18] = 0.01;
  covariance[27] = 1e-9;
  covariance[36] = kNotEstimated;
  covariance[45] = kNotEstimated;
  covariance[54] = 0.01;
  covariance[63] = kNotEstimated;

  // The dimension is taken from the template; the height is unknown.
  msg.dimension.dimension.length = description.length;
  msg.dimension.dimension.width = description.width;
  msg.dimension.covariance[0] = 1.0;
  msg.dimension.covariance[4] = 1.0;
  msg.dimension.covariance[8] = kNotEstimated;

  msg.existence_probability = vehicle.existence_probability;
  msg.classification.classification[description.classification] = 1.0f;

  for (const auto & axle : description.axles) {
    ufil_msgs::msg::Axle axle_msg;
    axle_msg.single_track = axle.single_track;
    axle_msg.track_width = static_cast<float>(axle.track_width);
    axle_msg.track_width_variance = static_cast<float>(axle.track_width_variance);
    axle_msg.center_to_axle = static_cast<float>(axle.center_to_axle);
    axle_msg.center_to_axle_variance = static_cast<float>(axle.center_to_axle_variance);
    axle_msg.wheel_detected.assign(axle.wheel_detected.begin(), axle.wheel_detected.end());
    msg.axles.push_back(axle_msg);
  }
  return msg;
}

/// Vehicles with an existence probability above the threshold, with the header of the input.
inline ufil_msgs::msg::ObjectList vehiclesToMsg(
  const std::vector<Vehicle> & vehicles, const std::vector<VehicleTemplate> & templates,
  const std_msgs::msg::Header & header, double publishing_threshold)
{
  ufil_msgs::msg::ObjectList msg;
  msg.header = header;
  for (const auto & vehicle : vehicles) {
    if (vehicle.existence_probability > publishing_threshold) {
      msg.objects.push_back(vehicleToMsg(vehicle, templates[vehicle.template_index]));
    }
  }
  return msg;
}

}  // namespace ssl_vehicle_tracking::conversions

#endif  // SSL_VEHICLE_TRACKING__COMPONENT__SSL_VEHICLE_TRACKING_ROS_HPP_
