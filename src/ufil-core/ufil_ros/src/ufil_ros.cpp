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


#include "ufil_ros/ufil_ros.hpp"

#include <iomanip>
#include <sstream>

#include <rclcpp/rclcpp.hpp>

#include <nav_msgs/msg/occupancy_grid.hpp>

#include <ufil_msgs/msg/dimension_with_covariance.hpp>
#include <ufil_msgs/msg/object.hpp>
#include <ufil_msgs/msg/object_list.hpp>
#include <ufil_msgs/msg/state_with_covariance.hpp>

namespace ufil_ros
{
nav_msgs::msg::OccupancyGrid occupancyGridToMsg(const ufil::type::OccupancyGrid & value)
{
  nav_msgs::msg::OccupancyGrid msg;

  msg.info.resolution = value.resolution();
  msg.info.width = value.width();
  msg.info.height = value.height();

  msg.info.origin.position.x = value.originX();
  msg.info.origin.position.y = value.originY();
  msg.info.origin.position.z = value.originZ();

  msg.data = value.data();
  return msg;
}

ufil::type::OccupancyGrid occupancyGridFromMsg(const nav_msgs::msg::OccupancyGrid & value)
{
  ufil::type::OccupancyGrid result;
  result.resolution() = value.info.resolution;
  result.width() = value.info.width;
  result.height() = value.info.height;

  result.originX() = value.info.origin.position.x;
  result.originY() = value.info.origin.position.y;
  result.originZ() = value.info.origin.position.z;

  result.data() = value.data;
  return result;
}

// ########################################################################### //
// ############################## Dimension ################################## //
// ########################################################################### //

// ############################# Dimension2D ################################# //
template<>
ufil_msgs::msg::Dimension dimensionToMsg(const ufil::type::dimension::Dimension2D & value)
{
  ufil_msgs::msg::Dimension result;
  result.length = value.length();
  result.width = value.width();

  return result;
}

template<>
ufil::type::dimension::Dimension2D dimensionFromMsg(const ufil_msgs::msg::Dimension & value)
{
  ufil::type::dimension::Dimension2D result;
  result.length() = value.length;
  result.width() = value.width;

  return result;
}

template<>
ufil_msgs::msg::DimensionWithCovariance dimensionToMsg(
  const ufil::type::dimension::Dimension2D & value)
{
  ufil_msgs::msg::DimensionWithCovariance result;
  result.dimension = dimensionToMsg<ufil_msgs::msg::Dimension,
      ufil::type::dimension::Dimension2D>(value);
  result.covariance = {};  // Zero array
  result.covariance[0] = value.covariance()(0, 0);
  result.covariance[1] = value.covariance()(0, 1);
  result.covariance[3] = value.covariance()(1, 0);
  result.covariance[4] = value.covariance()(1, 1);

  // Indicate not measured
  result.covariance[8] = -1;
  return result;
}

template<>
ufil::type::dimension::Dimension2D dimensionFromMsg(
  const ufil_msgs::msg::DimensionWithCovariance & value)
{
  ufil::type::dimension::Dimension2D result =
    dimensionFromMsg<ufil::type::dimension::Dimension2D>(value.dimension);
  result.covariance()(0, 0) = value.covariance[0];
  result.covariance()(0, 1) = value.covariance[1];
  result.covariance()(1, 0) = value.covariance[3];
  result.covariance()(1, 1) = value.covariance[4];

  return result;
}

// ############################# Dimension3D ################################# //
template<>
ufil_msgs::msg::Dimension dimensionToMsg(const ufil::type::dimension::Dimension3D & value)
{
  ufil_msgs::msg::Dimension result;
  result.length = value.length();
  result.width = value.width();
  result.height = value.height();
  return result;
}

template<>
ufil::type::dimension::Dimension3D dimensionFromMsg(const ufil_msgs::msg::Dimension & value)
{
  ufil::type::dimension::Dimension3D result;
  result.length() = value.length;
  result.width() = value.width;
  result.height() = value.height;
  return result;
}

template<>
ufil_msgs::msg::DimensionWithCovariance dimensionToMsg(
  const ufil::type::dimension::Dimension3D & value)
{
  ufil_msgs::msg::DimensionWithCovariance result;
  result.dimension = dimensionToMsg<ufil_msgs::msg::Dimension,
      ufil::type::dimension::Dimension3D>(value);
  result.covariance = {};  // Zero array
  result.covariance[0] = value.covariance()(0, 0);
  result.covariance[1] = value.covariance()(0, 1);
  result.covariance[2] = value.covariance()(0, 2);
  result.covariance[3] = value.covariance()(1, 0);
  result.covariance[4] = value.covariance()(1, 1);
  result.covariance[5] = value.covariance()(1, 2);
  result.covariance[6] = value.covariance()(2, 0);
  result.covariance[7] = value.covariance()(2, 1);
  result.covariance[8] = value.covariance()(2, 2);
  return result;
}

template<>
ufil::type::dimension::Dimension3D dimensionFromMsg(
  const ufil_msgs::msg::DimensionWithCovariance & value)
{
  ufil::type::dimension::Dimension3D result =
    dimensionFromMsg<ufil::type::dimension::Dimension3D>(value.dimension);
  result.covariance()(0, 0) = value.covariance[0];
  result.covariance()(0, 1) = value.covariance[1];
  result.covariance()(0, 2) = value.covariance[2];
  result.covariance()(1, 0) = value.covariance[3];
  result.covariance()(1, 1) = value.covariance[4];
  result.covariance()(1, 2) = value.covariance[5];
  result.covariance()(2, 0) = value.covariance[6];
  result.covariance()(2, 1) = value.covariance[7];
  result.covariance()(2, 2) = value.covariance[8];

  return result;
}

// ########################################################################### //
// ################################ State #################################### //
// ########################################################################### //

// ############################## Position2D ################################# //
template<>
ufil_msgs::msg::State stateToMsg(const ufil::type::state::Position2D & value)
{
  ufil_msgs::msg::State result;
  result.x = value.x();
  result.y = value.y();
  return result;
}

template<>
ufil::type::state::Position2D stateFromMsg(const ufil_msgs::msg::State & value)
{
  ufil::type::state::Position2D result;
  result.x() = value.x;
  result.y() = value.y;
  return result;
}

template<>
ufil_msgs::msg::StateWithCovariance stateToMsg(const ufil::type::state::Position2D & value)
{
  ufil_msgs::msg::StateWithCovariance result;

  result.state = stateToMsg<ufil_msgs::msg::State, ufil::type::state::Position2D>(value);

  result.covariance = {};  // Zero array
  result.covariance[0] = value.covariance()(0, 0);
  result.covariance[1] = value.covariance()(0, 1);
  result.covariance[8] = value.covariance()(1, 0);
  result.covariance[9] = value.covariance()(1, 1);

  // Indicate not measured
  result.covariance[18] = -1;
  result.covariance[27] = -1;
  result.covariance[36] = -1;
  result.covariance[45] = -1;
  result.covariance[54] = -1;
  result.covariance[63] = -1;

  return result;
}

template<>
ufil::type::state::Position2D stateFromMsg(const ufil_msgs::msg::StateWithCovariance & value)
{
  ufil::type::state::Position2D result = stateFromMsg<ufil::type::state::Position2D>(value.state);
  result.covariance()(0, 0) = value.covariance[0];
  result.covariance()(0, 1) = value.covariance[1];
  result.covariance()(1, 0) = value.covariance[8];
  result.covariance()(1, 1) = value.covariance[9];

  return result;
}

// ########################## PositionVelocity2D ############################# //
template<>
ufil_msgs::msg::State stateToMsg(const ufil::type::state::PositionVelocity2D & value)
{
  ufil_msgs::msg::State result;
  result.x = value.x();
  result.y = value.y();
  result.v_x = value.vx();
  result.v_y = value.vy();
  return result;
}
template<>
ufil::type::state::PositionVelocity2D stateFromMsg(const ufil_msgs::msg::State & value)
{
  ufil::type::state::PositionVelocity2D result;
  result.x() = value.x;
  result.y() = value.y;
  result.vx() = value.v_x;
  result.vy() = value.v_y;
  return result;
}

template<>
ufil_msgs::msg::StateWithCovariance stateToMsg(const ufil::type::state::PositionVelocity2D & value)
{
  ufil_msgs::msg::StateWithCovariance result;

  result.state = stateToMsg<ufil_msgs::msg::State, ufil::type::state::PositionVelocity2D>(value);

  result.covariance = {};  // Zero array
  result.covariance[0] = value.covariance()(0, 0);
  result.covariance[1] = value.covariance()(0, 1);
  result.covariance[2] = value.covariance()(0, 2);
  result.covariance[3] = value.covariance()(0, 3);

  result.covariance[8] = value.covariance()(1, 0);
  result.covariance[9] = value.covariance()(1, 1);
  result.covariance[10] = value.covariance()(1, 2);
  result.covariance[11] = value.covariance()(1, 3);

  result.covariance[16] = value.covariance()(2, 0);
  result.covariance[17] = value.covariance()(2, 1);
  result.covariance[18] = value.covariance()(2, 2);
  result.covariance[19] = value.covariance()(2, 3);

  result.covariance[24] = value.covariance()(3, 0);
  result.covariance[25] = value.covariance()(3, 1);
  result.covariance[26] = value.covariance()(3, 2);
  result.covariance[27] = value.covariance()(3, 3);

  // Indicate not measured
  result.covariance[36] = -1;
  result.covariance[45] = -1;
  result.covariance[54] = -1;
  result.covariance[63] = -1;

  return result;
}
template<>
ufil::type::state::PositionVelocity2D stateFromMsg(
  const ufil_msgs::msg::StateWithCovariance & value)
{
  ufil::type::state::PositionVelocity2D result =
    stateFromMsg<ufil::type::state::PositionVelocity2D>(value.state);
  result.covariance()(0, 0) = value.covariance[0];
  result.covariance()(0, 1) = value.covariance[1];
  result.covariance()(0, 2) = value.covariance[2];
  result.covariance()(0, 3) = value.covariance[3];

  result.covariance()(1, 0) = value.covariance[8];
  result.covariance()(1, 1) = value.covariance[9];
  result.covariance()(1, 2) = value.covariance[10];
  result.covariance()(1, 3) = value.covariance[11];

  result.covariance()(2, 0) = value.covariance[16];
  result.covariance()(2, 1) = value.covariance[17];
  result.covariance()(2, 2) = value.covariance[18];
  result.covariance()(2, 3) = value.covariance[19];

  result.covariance()(3, 0) = value.covariance[24];
  result.covariance()(3, 1) = value.covariance[25];
  result.covariance()(3, 2) = value.covariance[26];
  result.covariance()(3, 3) = value.covariance[27];

  return result;
}

// ################################ Pose2D ################################### //
template<>
ufil_msgs::msg::State stateToMsg(const ufil::type::state::Pose2D & value)
{
  ufil_msgs::msg::State result;
  result.x = value.x();
  result.y = value.y();
  result.yaw = value.yaw();
  return result;
}
template<>
ufil::type::state::Pose2D stateFromMsg(const ufil_msgs::msg::State & value)
{
  ufil::type::state::Pose2D result;
  result.x() = value.x;
  result.y() = value.y;
  result.yaw() = value.yaw;

  return result;
}

template<>
ufil_msgs::msg::StateWithCovariance stateToMsg(const ufil::type::state::Pose2D & value)
{
  ufil_msgs::msg::StateWithCovariance result;

  result.state = stateToMsg<ufil_msgs::msg::State, ufil::type::state::Pose2D>(value);

  result.covariance = {};  // Zero array
  result.covariance[0] = value.covariance()(0, 0);
  result.covariance[1] = value.covariance()(0, 1);
  result.covariance[6] = value.covariance()(0, 2);

  result.covariance[8] = value.covariance()(1, 0);
  result.covariance[9] = value.covariance()(1, 1);
  result.covariance[14] = value.covariance()(1, 2);

  result.covariance[48] = value.covariance()(2, 0);
  result.covariance[49] = value.covariance()(2, 1);
  result.covariance[54] = value.covariance()(2, 2);

  // Indicate not measured
  result.covariance[18] = -1;
  result.covariance[27] = -1;
  result.covariance[36] = -1;
  result.covariance[45] = -1;
  result.covariance[63] = -1;
  return result;
}
template<>
ufil::type::state::Pose2D stateFromMsg(const ufil_msgs::msg::StateWithCovariance & value)
{
  ufil::type::state::Pose2D result = stateFromMsg<ufil::type::state::Pose2D>(value.state);
  result.covariance()(0, 0) = value.covariance[0];
  result.covariance()(0, 1) = value.covariance[1];
  result.covariance()(0, 2) = value.covariance[6];

  result.covariance()(1, 0) = value.covariance[8];
  result.covariance()(1, 1) = value.covariance[9];
  result.covariance()(1, 2) = value.covariance[14];

  result.covariance()(2, 0) = value.covariance[48];
  result.covariance()(2, 1) = value.covariance[49];
  result.covariance()(2, 2) = value.covariance[54];
  return result;
}

// ############################ PoseVelocity2D ############################### //
template<>
ufil_msgs::msg::State stateToMsg(const ufil::type::state::PoseVelocity2D & value)
{
  ufil_msgs::msg::State result;
  result.x = value.x();
  result.y = value.y();
  result.v_x = value.vx();
  result.v_y = value.vy();
  result.yaw = value.yaw();
  result.yaw_rate = value.yawRate();
  return result;
}
template<>
ufil::type::state::PoseVelocity2D stateFromMsg(const ufil_msgs::msg::State & value)
{
  ufil::type::state::PoseVelocity2D result;
  result.x() = value.x;
  result.y() = value.y;
  result.vx() = value.v_x;
  result.vy() = value.v_y;
  result.yaw() = value.yaw;
  result.yawRate() = value.yaw_rate;

  return result;
}

template<>
ufil_msgs::msg::StateWithCovariance stateToMsg(const ufil::type::state::PoseVelocity2D & value)
{
  ufil_msgs::msg::StateWithCovariance result;

  result.state = stateToMsg<ufil_msgs::msg::State, ufil::type::state::PoseVelocity2D>(value);

  result.covariance = {};  // Zero array
  result.covariance[0] = value.covariance()(0, 0);
  result.covariance[1] = value.covariance()(0, 1);
  result.covariance[2] = value.covariance()(0, 2);
  result.covariance[3] = value.covariance()(0, 3);
  result.covariance[6] = value.covariance()(0, 4);
  result.covariance[7] = value.covariance()(0, 5);

  result.covariance[8] = value.covariance()(1, 0);
  result.covariance[9] = value.covariance()(1, 1);
  result.covariance[10] = value.covariance()(1, 2);
  result.covariance[11] = value.covariance()(1, 3);
  result.covariance[14] = value.covariance()(1, 4);
  result.covariance[15] = value.covariance()(1, 5);

  result.covariance[16] = value.covariance()(2, 0);
  result.covariance[17] = value.covariance()(2, 1);
  result.covariance[18] = value.covariance()(2, 2);
  result.covariance[19] = value.covariance()(2, 3);
  result.covariance[22] = value.covariance()(2, 4);
  result.covariance[23] = value.covariance()(2, 5);

  result.covariance[24] = value.covariance()(3, 0);
  result.covariance[25] = value.covariance()(3, 1);
  result.covariance[26] = value.covariance()(3, 2);
  result.covariance[27] = value.covariance()(3, 3);
  result.covariance[30] = value.covariance()(3, 4);
  result.covariance[31] = value.covariance()(3, 5);

  result.covariance[48] = value.covariance()(4, 0);
  result.covariance[49] = value.covariance()(4, 1);
  result.covariance[50] = value.covariance()(4, 2);
  result.covariance[51] = value.covariance()(4, 3);
  result.covariance[54] = value.covariance()(4, 4);
  result.covariance[55] = value.covariance()(4, 5);

  result.covariance[56] = value.covariance()(5, 0);
  result.covariance[57] = value.covariance()(5, 1);
  result.covariance[58] = value.covariance()(5, 2);
  result.covariance[59] = value.covariance()(5, 3);
  result.covariance[62] = value.covariance()(5, 4);
  result.covariance[63] = value.covariance()(5, 5);

  // Indicate not measured
  result.covariance[36] = -1;
  result.covariance[45] = -1;

  return result;
}
template<>
ufil::type::state::PoseVelocity2D stateFromMsg(const ufil_msgs::msg::StateWithCovariance & value)
{
  ufil::type::state::PoseVelocity2D result =
    stateFromMsg<ufil::type::state::PoseVelocity2D>(value.state);
  result.covariance()(0, 0) = value.covariance[0];
  result.covariance()(0, 1) = value.covariance[1];
  result.covariance()(0, 2) = value.covariance[2];
  result.covariance()(0, 3) = value.covariance[3];
  result.covariance()(0, 4) = value.covariance[6];
  result.covariance()(0, 5) = value.covariance[7];

  result.covariance()(1, 0) = value.covariance[8];
  result.covariance()(1, 1) = value.covariance[9];
  result.covariance()(1, 2) = value.covariance[10];
  result.covariance()(1, 3) = value.covariance[11];
  result.covariance()(1, 4) = value.covariance[14];
  result.covariance()(1, 5) = value.covariance[15];

  result.covariance()(2, 0) = value.covariance[16];
  result.covariance()(2, 1) = value.covariance[17];
  result.covariance()(2, 2) = value.covariance[18];
  result.covariance()(2, 3) = value.covariance[19];
  result.covariance()(2, 4) = value.covariance[22];
  result.covariance()(2, 5) = value.covariance[23];

  result.covariance()(3, 0) = value.covariance[24];
  result.covariance()(3, 1) = value.covariance[25];
  result.covariance()(3, 2) = value.covariance[26];
  result.covariance()(3, 3) = value.covariance[27];
  result.covariance()(3, 4) = value.covariance[30];
  result.covariance()(3, 5) = value.covariance[31];

  result.covariance()(4, 0) = value.covariance[48];
  result.covariance()(4, 1) = value.covariance[49];
  result.covariance()(4, 2) = value.covariance[50];
  result.covariance()(4, 3) = value.covariance[51];
  result.covariance()(4, 4) = value.covariance[54];
  result.covariance()(4, 5) = value.covariance[55];

  result.covariance()(5, 0) = value.covariance[56];
  result.covariance()(5, 1) = value.covariance[57];
  result.covariance()(5, 2) = value.covariance[58];
  result.covariance()(5, 3) = value.covariance[59];
  result.covariance()(5, 4) = value.covariance[62];
  result.covariance()(5, 5) = value.covariance[63];
  return result;
}

// ################### PoseVelocityAcceleration2D ############################# //
template<>
ufil_msgs::msg::State stateToMsg(const ufil::type::state::PoseVelocityAcceleration2D & value)
{
  ufil_msgs::msg::State result;
  result.x = value.x();
  result.y = value.y();
  result.v_x = value.vx();
  result.v_y = value.vy();
  result.a_x = value.ax();
  result.a_y = value.ay();
  result.yaw = value.yaw();
  result.yaw_rate = value.yawRate();
  return result;
}

template<>
ufil::type::state::PoseVelocityAcceleration2D stateFromMsg(const ufil_msgs::msg::State & value)
{
  ufil::type::state::PoseVelocityAcceleration2D result;
  result.x() = value.x;
  result.y() = value.y;
  result.vx() = value.v_x;
  result.vy() = value.v_y;
  result.ax() = value.a_x;
  result.ay() = value.a_y;
  result.yaw() = value.yaw;
  result.yawRate() = value.yaw_rate;
  return result;
}

template<>
ufil_msgs::msg::StateWithCovariance stateToMsg(
  const ufil::type::state::PoseVelocityAcceleration2D & value)
{
  ufil_msgs::msg::StateWithCovariance result;
  result.state = stateToMsg<ufil_msgs::msg::State, ufil::type::state::PoseVelocityAcceleration2D>(
    value);
  result.covariance = {};  // Zero array
  for (int i = 0; i < 8; ++i) {
    for (int j = 0; j < 8; ++j) {
      result.covariance[i * 8 + j] = value.covariance()(i, j);
    }
  }
  return result;
}

template<>
ufil::type::state::PoseVelocityAcceleration2D stateFromMsg(
  const ufil_msgs::msg::StateWithCovariance & value)
{
  ufil::type::state::PoseVelocityAcceleration2D result =
    stateFromMsg<ufil::type::state::PoseVelocityAcceleration2D>(value.state);
  for (int i = 0; i < 8; ++i) {
    for (int j = 0; j < 8; ++j) {
      result.covariance()(i, j) = value.covariance[i * 8 + j];
    }
  }
  return result;
}

// ########################################################################### //
// ############################# Measurement ################################# //
// ########################################################################### //

// ############################## Position2D ################################# //

template<>
ufil_msgs::msg::State measurementToMsg(const ufil::type::measurement::Position2D & value)
{
  ufil_msgs::msg::State result;

  result.x = value.x();
  result.y = value.y();
  result.v_y = 0.0f;
  result.v_y = 0.0f;
  result.a_y = 0.0f;
  result.a_y = 0.0f;
  result.yaw = 0.0f;
  result.yaw_rate = 0.0f;
  return result;
}
template<>
ufil::type::measurement::Position2D measurementFromMsg(const ufil_msgs::msg::State & value)
{
  ufil::type::measurement::Position2D result;
  result.x() = value.x;
  result.y() = value.y;

  return result;
}

template<>
ufil_msgs::msg::StateWithCovariance measurementToMsg(
  const ufil::type::measurement::Position2D & value)
{
  ufil_msgs::msg::StateWithCovariance result;

  result.state = measurementToMsg<ufil_msgs::msg::State,
      ufil::type::measurement::Position2D>(value);

  result.covariance = {};  // Zero array
  result.covariance[0] = value.covariance()(0, 0);
  result.covariance[1] = value.covariance()(0, 1);
  result.covariance[8] = value.covariance()(1, 0);
  result.covariance[9] = value.covariance()(1, 1);

  // Indicate not measured
  result.covariance[18] = -1;
  result.covariance[27] = -1;
  result.covariance[36] = -1;
  result.covariance[45] = -1;
  result.covariance[54] = -1;
  result.covariance[63] = -1;
  return result;
}
template<>
ufil::type::measurement::Position2D measurementFromMsg(
  const ufil_msgs::msg::StateWithCovariance & value)
{
  ufil::type::measurement::Position2D result =
    measurementFromMsg<ufil::type::measurement::Position2D>(value.state);
  result.covariance()(0, 0) = value.covariance[0];
  result.covariance()(0, 1) = value.covariance[1];
  result.covariance()(1, 0) = value.covariance[8];
  result.covariance()(1, 1) = value.covariance[9];

  return result;
}
template<>
ufil::type::measurement::Position2D measurementFromMsg(const ufil_msgs::msg::Object & value)
{
  ufil::type::measurement::Position2D result =
    measurementFromMsg<ufil::type::measurement::Position2D>(value.state);

  return result;
}
template<>
std::set<ufil::type::measurement::Position2D> measurementsFromMsg(
  const ufil_msgs::msg::ObjectList & value)
{
  std::set<ufil::type::measurement::Position2D> result;
  for (const auto & object : value.objects) {
    ufil::type::measurement::Position2D element =
      measurementFromMsg<ufil::type::measurement::Position2D>(object);
    result.insert(element);
  }
  return result;
}

// ############################## Position3D ################################# //
template<>
ufil_msgs::msg::State measurementToMsg(const ufil::type::measurement::Position3D & value)
{
  ufil_msgs::msg::State result;
  result.x = value.x();
  result.y = value.y();
  result.v_y = 0.0f;
  result.v_y = 0.0f;
  result.a_y = 0.0f;
  result.a_y = 0.0f;
  result.yaw = 0.0f;
  result.yaw_rate = 0.0f;
  return result;
}
template<>
ufil::type::measurement::Position3D measurementFromMsg(const ufil_msgs::msg::State & value)
{
  ufil::type::measurement::Position3D result;
  result.x() = value.x;
  result.y() = value.y;
  return result;
}

template<>
ufil_msgs::msg::StateWithCovariance measurementToMsg(
  const ufil::type::measurement::Position3D & value)
{
  ufil_msgs::msg::StateWithCovariance result;
  result.state = measurementToMsg<ufil_msgs::msg::State,
      ufil::type::measurement::Position3D>(value);

  result.covariance = {};  // Zero array
  result.covariance[0] = value.covariance()(0, 0);
  result.covariance[1] = value.covariance()(0, 1);
  result.covariance[8] = value.covariance()(1, 0);
  result.covariance[9] = value.covariance()(1, 1);

  // Indicate not measured
  result.covariance[18] = -1;
  result.covariance[27] = -1;
  result.covariance[36] = -1;
  result.covariance[45] = -1;
  result.covariance[54] = -1;
  result.covariance[63] = -1;
  return result;
}
template<>
ufil::type::measurement::Position3D measurementFromMsg(
  const ufil_msgs::msg::StateWithCovariance & value)
{
  ufil::type::measurement::Position3D result =
    measurementFromMsg<ufil::type::measurement::Position3D>(value.state);
  result.covariance()(0, 0) = value.covariance[0];
  result.covariance()(0, 1) = value.covariance[1];
  result.covariance()(1, 0) = value.covariance[8];
  result.covariance()(1, 1) = value.covariance[9];

  return result;
}
template<>
ufil::type::measurement::Position3D measurementFromMsg(const ufil_msgs::msg::Object & value)
{
  ufil::type::measurement::Position3D result =
    measurementFromMsg<ufil::type::measurement::Position3D>(value.state);

  return result;
}
template<>
std::set<ufil::type::measurement::Position3D> measurementsFromMsg(
  const ufil_msgs::msg::ObjectList & value)
{
  std::set<ufil::type::measurement::Position3D> result;
  for (const auto & object : value.objects) {
    ufil::type::measurement::Position3D element =
      measurementFromMsg<ufil::type::measurement::Position3D>(object);
    result.insert(element);
  }
  return result;
}

// ################################ Pose2D ################################### //
template<>
ufil_msgs::msg::State measurementToMsg(const ufil::type::measurement::Pose2D & value)
{
  ufil_msgs::msg::State result;
  result.x = value.x();
  result.y = value.y();
  result.v_y = 0.0f;
  result.v_y = 0.0f;
  result.a_y = 0.0f;
  result.a_y = 0.0f;
  result.yaw = value.yaw();
  result.yaw_rate = 0.0f;
  return result;
}
template<>
ufil::type::measurement::Pose2D measurementFromMsg(const ufil_msgs::msg::State & value)
{
  ufil::type::measurement::Pose2D result;
  result.x() = value.x;
  result.y() = value.y;
  result.yaw() = value.yaw;
  return result;
}

template<>
ufil_msgs::msg::StateWithCovariance measurementToMsg(const ufil::type::measurement::Pose2D & value)
{
  ufil_msgs::msg::StateWithCovariance result;
  result.state = measurementToMsg<ufil_msgs::msg::State, ufil::type::measurement::Pose2D>(value);

  result.covariance = {};  // Zero array
  result.covariance[0] = value.covariance()(0, 0);
  result.covariance[1] = value.covariance()(0, 1);
  result.covariance[6] = value.covariance()(0, 2);

  result.covariance[8] = value.covariance()(1, 0);
  result.covariance[9] = value.covariance()(1, 1);
  result.covariance[14] = value.covariance()(1, 2);

  result.covariance[48] = value.covariance()(2, 0);
  result.covariance[49] = value.covariance()(2, 1);
  result.covariance[54] = value.covariance()(2, 2);

  // Indicate not measured
  result.covariance[18] = -1;
  result.covariance[27] = -1;
  result.covariance[36] = -1;
  result.covariance[45] = -1;
  result.covariance[63] = -1;
  return result;
}
template<>
ufil::type::measurement::Pose2D measurementFromMsg(
  const ufil_msgs::msg::StateWithCovariance & value)
{
  ufil::type::measurement::Pose2D result =
    measurementFromMsg<ufil::type::measurement::Pose2D>(value.state);
  result.covariance()(0, 0) = value.covariance[0];
  result.covariance()(0, 1) = value.covariance[1];
  result.covariance()(0, 2) = value.covariance[6];

  result.covariance()(1, 0) = value.covariance[8];
  result.covariance()(1, 1) = value.covariance[9];
  result.covariance()(1, 2) = value.covariance[14];

  result.covariance()(2, 0) = value.covariance[48];
  result.covariance()(2, 1) = value.covariance[49];
  result.covariance()(2, 2) = value.covariance[54];
  return result;
}
template<>
ufil::type::measurement::Pose2D measurementFromMsg(const ufil_msgs::msg::Object & value)
{
  ufil::type::measurement::Pose2D result =
    measurementFromMsg<ufil::type::measurement::Pose2D>(value.state);

  return result;
}
template<>
std::set<ufil::type::measurement::Pose2D> measurementsFromMsg(
  const ufil_msgs::msg::ObjectList & value)
{
  std::set<ufil::type::measurement::Pose2D> result;
  for (const auto & object : value.objects) {
    ufil::type::measurement::Pose2D element =
      measurementFromMsg<ufil::type::measurement::Pose2D>(object);
    result.insert(element);
  }
  return result;
}

// ################################ Pose2D ################################### //
template<>
ufil_msgs::msg::State measurementToMsg(const ufil::type::measurement::Pose2DWithDimension2D & value)
{
  ufil_msgs::msg::State result;
  result.x = value.x();
  result.y = value.y();
  result.v_y = 0.0f;
  result.v_y = 0.0f;
  result.a_y = 0.0f;
  result.a_y = 0.0f;
  result.yaw = value.yaw();
  result.yaw_rate = 0.0f;
  return result;
}
template<>
ufil::type::measurement::Pose2DWithDimension2D measurementFromMsg(
  const ufil_msgs::msg::State & value)
{
  ufil::type::measurement::Pose2DWithDimension2D result;
  result.x() = value.x;
  result.y() = value.y;
  result.yaw() = value.yaw;
  return result;
}

template<>
ufil_msgs::msg::StateWithCovariance measurementToMsg(
  const ufil::type::measurement::Pose2DWithDimension2D & value)
{
  ufil_msgs::msg::StateWithCovariance result;
  result.state = measurementToMsg<ufil_msgs::msg::State, ufil::type::measurement::Pose2D>(value);

  result.covariance = {};  // Zero array
  result.covariance[0] = value.covariance()(0, 0);
  result.covariance[1] = value.covariance()(0, 1);
  result.covariance[6] = value.covariance()(0, 2);

  result.covariance[8] = value.covariance()(1, 0);
  result.covariance[9] = value.covariance()(1, 1);
  result.covariance[14] = value.covariance()(1, 2);

  result.covariance[48] = value.covariance()(2, 0);
  result.covariance[49] = value.covariance()(2, 1);
  result.covariance[54] = value.covariance()(2, 2);

  // Indicate not measured
  result.covariance[18] = -1;
  result.covariance[27] = -1;
  result.covariance[36] = -1;
  result.covariance[45] = -1;
  result.covariance[63] = -1;

  return result;
}
template<>
ufil::type::measurement::Pose2DWithDimension2D measurementFromMsg(
  const ufil_msgs::msg::StateWithCovariance & value)
{
  ufil::type::measurement::Pose2DWithDimension2D result =
    measurementFromMsg<ufil::type::measurement::Pose2DWithDimension2D>(value.state);
  result.covariance()(0, 0) = value.covariance[0];
  result.covariance()(0, 1) = value.covariance[1];
  result.covariance()(0, 2) = value.covariance[6];

  result.covariance()(1, 0) = value.covariance[8];
  result.covariance()(1, 1) = value.covariance[9];
  result.covariance()(1, 2) = value.covariance[14];

  result.covariance()(2, 0) = value.covariance[48];
  result.covariance()(2, 1) = value.covariance[49];
  result.covariance()(2, 2) = value.covariance[54];
  return result;
}

template<>
ufil_msgs::msg::State measurementToMsg(const ufil::type::measurement::Pose2DWithDimension3D & value)
{
  ufil_msgs::msg::State result;
  result.x = value.x();
  result.y = value.y();
  result.v_y = 0.0f;
  result.v_y = 0.0f;
  result.a_y = 0.0f;
  result.a_y = 0.0f;
  result.yaw = value.yaw();
  result.yaw_rate = 0.0f;
  return result;
}
template<>
ufil::type::measurement::Pose2DWithDimension3D measurementFromMsg(
  const ufil_msgs::msg::State & value)
{
  ufil::type::measurement::Pose2DWithDimension3D result;
  result.x() = value.x;
  result.y() = value.y;
  result.yaw() = value.yaw;
  return result;
}

template<>
ufil_msgs::msg::StateWithCovariance measurementToMsg(
  const ufil::type::measurement::Pose2DWithDimension3D & value)
{
  ufil_msgs::msg::StateWithCovariance result;
  result.state = measurementToMsg<ufil_msgs::msg::State, ufil::type::measurement::Pose2D>(value);

  result.covariance = {};  // Zero array
  result.covariance[0] = value.covariance()(0, 0);
  result.covariance[1] = value.covariance()(0, 1);
  result.covariance[6] = value.covariance()(0, 2);

  result.covariance[8] = value.covariance()(1, 0);
  result.covariance[9] = value.covariance()(1, 1);
  result.covariance[14] = value.covariance()(1, 2);

  result.covariance[48] = value.covariance()(2, 0);
  result.covariance[49] = value.covariance()(2, 1);
  result.covariance[54] = value.covariance()(2, 2);

  // Indicate not measured
  result.covariance[18] = -1;
  result.covariance[27] = -1;
  result.covariance[36] = -1;
  result.covariance[45] = -1;
  result.covariance[63] = -1;

  return result;
}
template<>
ufil::type::measurement::Pose2DWithDimension3D measurementFromMsg(
  const ufil_msgs::msg::StateWithCovariance & value)
{
  ufil::type::measurement::Pose2DWithDimension3D result =
    measurementFromMsg<ufil::type::measurement::Pose2DWithDimension3D>(value.state);
  result.covariance()(0, 0) = value.covariance[0];
  result.covariance()(0, 1) = value.covariance[1];
  result.covariance()(0, 2) = value.covariance[6];

  result.covariance()(1, 0) = value.covariance[8];
  result.covariance()(1, 1) = value.covariance[9];
  result.covariance()(1, 2) = value.covariance[14];

  result.covariance()(2, 0) = value.covariance[48];
  result.covariance()(2, 1) = value.covariance[49];
  result.covariance()(2, 2) = value.covariance[54];
  return result;
}

template<>
ufil_msgs::msg::Object measurementToMsg(
  const ufil::type::measurement::Pose2DWithDimension2D & value)
{
  ufil_msgs::msg::Object result;
  result.state =
    measurementToMsg<ufil_msgs::msg::StateWithCovariance,
      ufil::type::measurement::Pose2DWithDimension2D>(value);
  result.dimension =
    dimensionToMsg<ufil_msgs::msg::DimensionWithCovariance,
      ufil::type::dimension::Dimension2D>(value.dimension());
  result.id = ufil::hashId<ufil_msgs::msg::Object::_id_type>(value.uuid());
  return result;
}
template<>
ufil::type::measurement::Pose2DWithDimension2D measurementFromMsg(
  const ufil_msgs::msg::Object & value)
{
  ufil::type::measurement::Pose2DWithDimension2D result;
  result = measurementFromMsg<ufil::type::measurement::Pose2DWithDimension2D>(value.state);
  result.dimension() = dimensionFromMsg<ufil::type::dimension::Dimension2D>(value.dimension);
  return result;
}

template<>
ufil_msgs::msg::Object measurementToMsg(
  const ufil::type::measurement::Pose2DWithDimension3D & value)
{
  ufil_msgs::msg::Object result;
  result.state =
    measurementToMsg<ufil_msgs::msg::StateWithCovariance,
      ufil::type::measurement::Pose2DWithDimension3D>(value);
  result.dimension =
    dimensionToMsg<ufil_msgs::msg::DimensionWithCovariance,
      ufil::type::dimension::Dimension3D>(value.dimension());
  result.id = ufil::hashId<ufil_msgs::msg::Object::_id_type>(value.uuid());
  return result;
}
template<>
ufil_msgs::msg::ObjectList measurementsToMsg(
  const std::set<ufil::type::measurement::Pose2DWithDimension3D> & value)
{
  ufil_msgs::msg::ObjectList result;
  for (const auto & measurement : value) {
    ufil_msgs::msg::Object output_object =
      ufil_ros::measurementToMsg<ufil_msgs::msg::Object>(measurement);
    result.objects.push_back(output_object);
  }

  return result;
}

template<>
ufil::type::measurement::Pose2DWithDimension3D measurementFromMsg(
  const ufil_msgs::msg::Object & value)
{
  ufil::type::measurement::Pose2DWithDimension3D result;
  result = measurementFromMsg<ufil::type::measurement::Pose2DWithDimension3D>(value.state);
  result.dimension() = dimensionFromMsg<ufil::type::dimension::Dimension3D>(value.dimension);
  return result;
}

template<>
std::set<ufil::type::measurement::Pose2DWithDimension3D> measurementsFromMsg(
  const ufil_msgs::msg::ObjectList & value)
{
  std::set<ufil::type::measurement::Pose2DWithDimension3D> result;
  for (const auto & object : value.objects) {
    ufil::type::measurement::Pose2DWithDimension3D element =
      measurementFromMsg<ufil::type::measurement::Pose2DWithDimension3D>(object);
    result.insert(element);
  }
  return result;
}

// ########################################################################### //
// ################################ Track #################################### //
// ########################################################################### //

template<>
ufil_msgs::msg::Classification classificationToMsg(
  const ufil::type::classification::ObjectClassification & value)
{
  ufil_msgs::msg::Classification result;
  result.classification[ufil_msgs::msg::Classification::CAR] = value.car();
  result.classification[ufil_msgs::msg::Classification::TRUCK] = value.truck();
  result.classification[ufil_msgs::msg::Classification::MOTORCYCLE] = value.motorcycle();
  result.classification[ufil_msgs::msg::Classification::BICYCLE] = value.bicycle();
  result.classification[ufil_msgs::msg::Classification::PEDESTRIAN] = value.pedestrian();
  result.classification[ufil_msgs::msg::Classification::STATIONARY] = value.stationary();
  result.classification[ufil_msgs::msg::Classification::OTHER] = value.other();

  return result;
}

template<>
ufil::type::classification::ObjectClassification classificationFromMsg(
  const ufil_msgs::msg::Classification & value)
{
  ufil::type::classification::ObjectClassification result;
  result.car() = value.classification[ufil_msgs::msg::Classification::CAR];
  result.truck() = value.classification[ufil_msgs::msg::Classification::TRUCK];
  result.motorcycle() = value.classification[ufil_msgs::msg::Classification::MOTORCYCLE];
  result.bicycle() = value.classification[ufil_msgs::msg::Classification::BICYCLE];
  result.pedestrian() = value.classification[ufil_msgs::msg::Classification::PEDESTRIAN];
  result.stationary() = value.classification[ufil_msgs::msg::Classification::STATIONARY];
  result.other() = value.classification[ufil_msgs::msg::Classification::OTHER];

  return result;
}


// ########################################################################### //
// ############################## SensorFOV ################################## //
// ########################################################################### //

ufil_msgs::msg::SensorFOV sensorFovToMsg(const ufil::type::SensorFOV & fov)
{
  ufil_msgs::msg::SensorFOV msg;
  msg.r_min = fov.rMin();
  msg.r_max = fov.rMax();
  msg.phi_min = fov.phiMin();
  msg.phi_max = fov.phiMax();
  msg.theta_min = fov.thetaMin();
  msg.theta_max = fov.thetaMax();

  msg.origin.x = fov.originX();
  msg.origin.y = fov.originY();
  msg.origin.z = fov.originZ();

    // Copy covariance (row-major)
  for (int i = 0; i < 6; ++i) {
    for (int j = 0; j < 6; ++j) {
      msg.covariance[i * 6 + j] = fov.covariance()(i, j);
    }
  }
  return msg;
}

ufil::type::SensorFOV sensorFovFromMsg(const ufil_msgs::msg::SensorFOV & msg)
{
  ufil::type::SensorFOV fov;
  fov.rMin() = msg.r_min;
  fov.rMax() = msg.r_max;
  fov.phiMin() = msg.phi_min;
  fov.phiMax() = msg.phi_max;
  fov.thetaMin() = msg.theta_min;
  fov.thetaMax() = msg.theta_max;

  for (int i = 0; i < 6; ++i) {
    for (int j = 0; j < 6; ++j) {
      fov.covariance()(i, j) = msg.covariance[i * 6 + j];
    }
  }

  return fov;
}


}  // namespace ufil_ros

// TODO(dummy): Check for missing conversions.
// TODO(dummy): Add unit tests.
