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

#ifndef UFIL_ETSI__COMPONENT__UFIL_ETSI_ROS_HPP_
#define UFIL_ETSI__COMPONENT__UFIL_ETSI_ROS_HPP_

// Conversions between ROS 2 messages and the ROS-independent ufil_etsi types.
//
// The CAM conversion copies the values encoded by the core library into the message and sets
// the fields that UFIL does not support to constant values (ITS PDU header, curvature,
// exterior lights, optional containers). It does not compute anything.

#include <algorithm>
#include <cstddef>
#include <cstdint>

#include <builtin_interfaces/msg/time.hpp>
#include <etsi_its_cam_msgs/msg/cam.hpp>
#include <rclcpp/time.hpp>
#include <ufil_msgs/msg/classification.hpp>
#include <ufil_msgs/msg/object.hpp>

#include "ufil_etsi/cam.hpp"
#include "ufil_etsi/types.hpp"

namespace ufil_etsi::conversions
{

// --- Time --------------------------------------------------------------------------------

inline Time timeFromMsg(const builtin_interfaces::msg::Time & stamp)
{
  return Time(rclcpp::Time(stamp).nanoseconds());
}

inline Time timeFromRos(const rclcpp::Time & time)
{
  return Time(time.nanoseconds());
}

inline builtin_interfaces::msg::Time timeToMsg(Time time)
{
  return rclcpp::Time(time.count(), RCL_ROS_TIME);
}

// --- Objects -----------------------------------------------------------------------------

/// Index of an ObjectClass in ufil_msgs/Classification.
inline std::size_t classificationIndex(ObjectClass object_class)
{
  using ufil_msgs::msg::Classification;
  switch (object_class) {
    case ObjectClass::kCar:
      return Classification::CAR;
    case ObjectClass::kTruck:
      return Classification::TRUCK;
    case ObjectClass::kMotorcycle:
      return Classification::MOTORCYCLE;
    case ObjectClass::kBicycle:
      return Classification::BICYCLE;
    case ObjectClass::kPedestrian:
      return Classification::PEDESTRIAN;
    case ObjectClass::kStationary:
      return Classification::STATIONARY;
    case ObjectClass::kOther:
      break;
  }
  return Classification::OTHER;
}

inline Object objectFromMsg(const ufil_msgs::msg::Object & msg)
{
  Object object;
  object.id = msg.id;
  const auto & state = msg.state.state;
  object.x = state.x;
  object.y = state.y;
  object.v_x = state.v_x;
  object.v_y = state.v_y;
  object.a_x = state.a_x;
  object.a_y = state.a_y;
  object.yaw = state.yaw;
  object.yaw_rate = state.yaw_rate;
  std::copy(msg.state.covariance.begin(), msg.state.covariance.end(), object.covariance.begin());
  object.length = msg.dimension.dimension.length;
  object.width = msg.dimension.dimension.width;
  object.height = msg.dimension.dimension.height;
  std::copy(
    msg.dimension.covariance.begin(), msg.dimension.covariance.end(),
    object.dimension_covariance.begin());
  object.existence_probability = msg.existence_probability;
  for (std::size_t i = 0; i < kObjectClassCount; ++i) {
    object.classification[i] =
      msg.classification.classification[classificationIndex(static_cast<ObjectClass>(i))];
  }
  return object;
}

/// Object message of an object decoded from a CAM. All features count as detected.
inline ufil_msgs::msg::Object objectToMsg(const Object & object)
{
  ufil_msgs::msg::Object msg;
  msg.id = object.id;
  auto & state = msg.state.state;
  state.x = object.x;
  state.y = object.y;
  state.v_x = object.v_x;
  state.v_y = object.v_y;
  state.a_x = object.a_x;
  state.a_y = object.a_y;
  state.yaw = object.yaw;
  state.yaw_rate = object.yaw_rate;
  std::copy(object.covariance.begin(), object.covariance.end(), msg.state.covariance.begin());
  msg.dimension.dimension.length = object.length;
  msg.dimension.dimension.width = object.width;
  msg.dimension.dimension.height = object.height;
  std::copy(
    object.dimension_covariance.begin(), object.dimension_covariance.end(),
    msg.dimension.covariance.begin());
  msg.existence_probability = object.existence_probability;
  for (std::size_t i = 0; i < kObjectClassCount; ++i) {
    msg.classification.classification[classificationIndex(static_cast<ObjectClass>(i))] =
      static_cast<float>(object.classification[i]);
  }
  msg.features.fl = true;
  msg.features.fr = true;
  msg.features.bl = true;
  msg.features.br = true;
  msg.features.f = true;
  msg.features.b = true;
  msg.features.l = true;
  msg.features.r = true;
  msg.features.c = true;
  return msg;
}

// --- CAM ---------------------------------------------------------------------------------

/// Protocol version of EN 302 637-2 V1.4.1.
inline constexpr std::uint8_t kCamProtocolVersion = 2;

inline etsi_its_cam_msgs::msg::CAM camToMsg(const Cam & cam)
{
  namespace msg = etsi_its_cam_msgs::msg;
  msg::CAM result;
  result.header.protocol_version = kCamProtocolVersion;
  result.header.message_id = msg::ItsPduHeader::MESSAGE_ID_CAM;
  result.header.station_id.value = cam.station_id;
  result.cam.generation_delta_time.value = cam.generation_delta_time;

  auto & parameters = result.cam.cam_parameters;
  parameters.basic_container.station_type.value = cam.station_type;
  auto & reference = parameters.basic_container.reference_position;
  reference.latitude.value = cam.reference_position.latitude;
  reference.longitude.value = cam.reference_position.longitude;
  reference.position_confidence_ellipse.semi_major_confidence.value =
    cam.reference_position.semi_major_confidence;
  reference.position_confidence_ellipse.semi_minor_confidence.value =
    cam.reference_position.semi_minor_confidence;
  reference.position_confidence_ellipse.semi_major_orientation.value =
    cam.reference_position.semi_major_orientation;
  reference.altitude.altitude_value.value = cam.reference_position.altitude;
  reference.altitude.altitude_confidence.value = cam.reference_position.altitude_confidence;

  auto & high_frequency = parameters.high_frequency_container;
  if (cam.vehicle_high_frequency) {
    const auto & in = *cam.vehicle_high_frequency;
    high_frequency.choice =
      msg::HighFrequencyContainer::CHOICE_BASIC_VEHICLE_CONTAINER_HIGH_FREQUENCY;
    auto & out = high_frequency.basic_vehicle_container_high_frequency;
    out.heading.heading_value.value = in.heading;
    out.heading.heading_confidence.value = in.heading_confidence;
    out.speed.speed_value.value = in.speed;
    out.speed.speed_confidence.value = in.speed_confidence;
    out.drive_direction.value = in.drive_direction;
    out.vehicle_length.vehicle_length_value.value = in.vehicle_length;
    out.vehicle_length.vehicle_length_confidence_indication.value =
      in.vehicle_length_confidence_indication;
    out.vehicle_width.value = in.vehicle_width;
    out.longitudinal_acceleration.longitudinal_acceleration_value.value =
      in.longitudinal_acceleration;
    out.longitudinal_acceleration.longitudinal_acceleration_confidence.value =
      in.longitudinal_acceleration_confidence;
    out.lateral_acceleration_is_present = in.lateral_acceleration.has_value();
    if (in.lateral_acceleration) {
      out.lateral_acceleration.lateral_acceleration_value.value = in.lateral_acceleration->value;
      out.lateral_acceleration.lateral_acceleration_confidence.value =
        in.lateral_acceleration->confidence;
    }
    out.yaw_rate.yaw_rate_value.value = in.yaw_rate;
    out.yaw_rate.yaw_rate_confidence.value = static_cast<std::uint8_t>(in.yaw_rate_confidence);
    // Not supported.
    out.curvature.curvature_value.value = msg::CurvatureValue::UNAVAILABLE;
    out.curvature.curvature_confidence.value = msg::CurvatureConfidence::UNAVAILABLE;
    out.curvature_calculation_mode.value = msg::CurvatureCalculationMode::UNAVAILABLE;
    out.acceleration_control_is_present = false;
    out.lane_position_is_present = false;
    out.steering_wheel_angle_is_present = false;
    out.vertical_acceleration_is_present = false;
    out.performance_class_is_present = false;
    out.cen_dsrc_tolling_zone_is_present = false;
  } else {
    high_frequency.choice = msg::HighFrequencyContainer::CHOICE_RSU_CONTAINER_HIGH_FREQUENCY;
    high_frequency.rsu_container_high_frequency.protected_communication_zones_rsu_is_present =
      false;
  }

  parameters.low_frequency_container_is_present = cam.vehicle_low_frequency.has_value();
  if (cam.vehicle_low_frequency) {
    auto & low_frequency = parameters.low_frequency_container;
    low_frequency.choice = msg::LowFrequencyContainer::CHOICE_BASIC_VEHICLE_CONTAINER_LOW_FREQUENCY;
    auto & out = low_frequency.basic_vehicle_container_low_frequency;
    out.vehicle_role.value = cam.vehicle_low_frequency->vehicle_role;
    // Exterior lights are not supported: all bits 0 ("off or not available").
    out.exterior_lights.value = {0};
    out.exterior_lights.bits_unused = 0;
    for (const auto & point : cam.vehicle_low_frequency->path_history) {
      msg::PathPoint path_point;
      path_point.path_position.delta_latitude.value = point.delta_latitude;
      path_point.path_position.delta_longitude.value = point.delta_longitude;
      path_point.path_position.delta_altitude.value = point.delta_altitude;
      path_point.path_delta_time_is_present = point.delta_time.has_value();
      path_point.path_delta_time.value = point.delta_time.value_or(0);
      out.path_history.array.push_back(path_point);
    }
  }
  parameters.special_vehicle_container_is_present = false;
  return result;
}

inline Cam camFromMsg(const etsi_its_cam_msgs::msg::CAM & msg)
{
  namespace etsi_msg = etsi_its_cam_msgs::msg;
  Cam cam;
  cam.station_id = msg.header.station_id.value;
  cam.generation_delta_time = msg.cam.generation_delta_time.value;

  const auto & parameters = msg.cam.cam_parameters;
  cam.station_type = parameters.basic_container.station_type.value;
  const auto & reference = parameters.basic_container.reference_position;
  cam.reference_position.latitude = reference.latitude.value;
  cam.reference_position.longitude = reference.longitude.value;
  cam.reference_position.semi_major_confidence =
    reference.position_confidence_ellipse.semi_major_confidence.value;
  cam.reference_position.semi_minor_confidence =
    reference.position_confidence_ellipse.semi_minor_confidence.value;
  cam.reference_position.semi_major_orientation =
    reference.position_confidence_ellipse.semi_major_orientation.value;
  cam.reference_position.altitude = reference.altitude.altitude_value.value;
  cam.reference_position.altitude_confidence = reference.altitude.altitude_confidence.value;

  const auto & high_frequency = parameters.high_frequency_container;
  if (high_frequency.choice ==
    etsi_msg::HighFrequencyContainer::CHOICE_BASIC_VEHICLE_CONTAINER_HIGH_FREQUENCY)
  {
    const auto & in = high_frequency.basic_vehicle_container_high_frequency;
    CamVehicleHighFrequency out;
    out.heading = in.heading.heading_value.value;
    out.heading_confidence = in.heading.heading_confidence.value;
    out.speed = in.speed.speed_value.value;
    out.speed_confidence = in.speed.speed_confidence.value;
    out.drive_direction = in.drive_direction.value;
    out.vehicle_length = in.vehicle_length.vehicle_length_value.value;
    out.vehicle_length_confidence_indication =
      in.vehicle_length.vehicle_length_confidence_indication.value;
    out.vehicle_width = in.vehicle_width.value;
    out.longitudinal_acceleration =
      in.longitudinal_acceleration.longitudinal_acceleration_value.value;
    out.longitudinal_acceleration_confidence =
      in.longitudinal_acceleration.longitudinal_acceleration_confidence.value;
    if (in.lateral_acceleration_is_present) {
      out.lateral_acceleration = CamLateralAcceleration{
        in.lateral_acceleration.lateral_acceleration_value.value,
        in.lateral_acceleration.lateral_acceleration_confidence.value};
    }
    out.yaw_rate = in.yaw_rate.yaw_rate_value.value;
    out.yaw_rate_confidence = static_cast<etsi::YawRateConfidence>(
      std::min<std::uint8_t>(
        in.yaw_rate.yaw_rate_confidence.value,
        static_cast<std::uint8_t>(etsi::YawRateConfidence::kUnavailable)));
    cam.vehicle_high_frequency = out;
  }

  if (parameters.low_frequency_container_is_present &&
    parameters.low_frequency_container.choice ==
    etsi_msg::LowFrequencyContainer::CHOICE_BASIC_VEHICLE_CONTAINER_LOW_FREQUENCY)
  {
    const auto & in = parameters.low_frequency_container.basic_vehicle_container_low_frequency;
    CamVehicleLowFrequency out;
    out.vehicle_role = in.vehicle_role.value;
    for (const auto & point : in.path_history.array) {
      CamPathPoint path_point;
      path_point.delta_latitude = point.path_position.delta_latitude.value;
      path_point.delta_longitude = point.path_position.delta_longitude.value;
      path_point.delta_altitude = point.path_position.delta_altitude.value;
      if (point.path_delta_time_is_present) {
        path_point.delta_time = point.path_delta_time.value;
      }
      out.path_history.push_back(path_point);
    }
    cam.vehicle_low_frequency = out;
  }
  return cam;
}

}  // namespace ufil_etsi::conversions

#endif  // UFIL_ETSI__COMPONENT__UFIL_ETSI_ROS_HPP_
