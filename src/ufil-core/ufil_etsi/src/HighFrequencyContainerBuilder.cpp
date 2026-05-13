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


#include "ufil_etsi/HighFrequencyContainerBuilder.hpp"

#include <numbers>
#include <cassert>
#include <ufil_etsi/Utilities.hpp>

using etsi_its_cam_msgs::msg::YawRateConfidence;
using etsi_its_cam_msgs::msg::YawRateValue;
using etsi_its_cam_msgs::msg::AccelerationConfidence;
using etsi_its_cam_msgs::msg::LateralAccelerationValue;
using etsi_its_cam_msgs::msg::LongitudinalAccelerationValue;
using etsi_its_cam_msgs::msg::HeadingConfidence;
using etsi_its_cam_msgs::msg::HeadingValue;
using etsi_its_cam_msgs::msg::YawRate;
using etsi_its_cam_msgs::msg::DriveDirection;
using etsi_its_cam_msgs::msg::SpeedConfidence;
using etsi_its_cam_msgs::msg::SpeedValue;
using etsi_its_cam_msgs::msg::VehicleWidth;
using etsi_its_cam_msgs::msg::VehicleLengthConfidenceIndication;
using etsi_its_cam_msgs::msg::VehicleLengthValue;
using etsi_its_cam_msgs::msg::CurvatureConfidence;
using etsi_its_cam_msgs::msg::CurvatureValue;
using etsi_its_cam_msgs::msg::Speed;
using etsi_its_cam_msgs::msg::Heading;


namespace etsi_message_converter
{
HighFrequencyContainerBuilder::HighFrequencyContainerBuilder()
{
  // Curvature (not supported)
  this->curvature_.curvature_value.value = CurvatureValue::UNAVAILABLE;
  this->curvature_.curvature_confidence.value = CurvatureConfidence::UNAVAILABLE;

  // Curvature Calculation Mode (not supported)
  this->curvature_calculation_mode_.value = CurvatureCalculationMode::UNAVAILABLE;
}

void HighFrequencyContainerBuilder::reset()
{
  // Reset container
  this->high_frequency_container_ = std::make_unique<HighFrequencyContainer>();

  // Prepare next container
  this->high_frequency_container_->choice =
    HighFrequencyContainer::CHOICE_BASIC_VEHICLE_CONTAINER_HIGH_FREQUENCY;
  this->high_frequency_container_->basic_vehicle_container_high_frequency.curvature =
    this->curvature_;
  this->high_frequency_container_->basic_vehicle_container_high_frequency
  .curvature_calculation_mode = this->curvature_calculation_mode_;
  this->high_frequency_container_->basic_vehicle_container_high_frequency
  .acceleration_control_is_present = false;
  this->high_frequency_container_->basic_vehicle_container_high_frequency.lane_position_is_present =
    false;
  this->high_frequency_container_->basic_vehicle_container_high_frequency
  .steering_wheel_angle_is_present = false;
  this->high_frequency_container_->basic_vehicle_container_high_frequency
  .performance_class_is_present = false;
  this->high_frequency_container_->basic_vehicle_container_high_frequency
  .cen_dsrc_tolling_zone_is_present = false;
}

HighFrequencyContainerBuilder & HighFrequencyContainerBuilder::dimension(
  const ufil_msgs::msg::ObjectStamped::UniquePtr & object_stamped)
{
  // Vehicle Length

  // Convert vehicle length from meters to decimeters
  auto vehicle_length_dm = static_cast<VehicleLengthValue::_value_type>(
    object_stamped->object.dimension.dimension.length * 10);
  // Cap to maximum length
  auto vehicle_length_capped =
    capValueToRange(vehicle_length_dm, VehicleLengthValue::MIN, VehicleLengthValue::OUT_OF_RANGE);
  this->high_frequency_container_->basic_vehicle_container_high_frequency.vehicle_length
  .vehicle_length_value.value = vehicle_length_capped;
  // Trailers are not supported
  this->high_frequency_container_->basic_vehicle_container_high_frequency.vehicle_length
  .vehicle_length_confidence_indication.value =
    VehicleLengthConfidenceIndication::NO_TRAILER_PRESENT;

  // Vehicle Width

  // Convert vehicle width from meters to decimeters
  auto vehicle_width_dm =
    static_cast<VehicleWidth::_value_type>(object_stamped->object.dimension.dimension.width * 10);
  // Cap to maximum length
  auto vehicle_width_capped =
    capValueToRange(vehicle_width_dm, VehicleWidth::MIN, VehicleWidth::OUT_OF_RANGE);
  this->high_frequency_container_->basic_vehicle_container_high_frequency.vehicle_width.value =
    vehicle_width_capped;

  return *this;
}

HighFrequencyContainerBuilder & HighFrequencyContainerBuilder::heading(
  const ufil_msgs::msg::ObjectStamped::UniquePtr & object_stamped)
{
  Heading heading;

  // Heading Value
  auto yaw_rad = object_stamped->object.state.state.yaw;
  auto yaw_deg = yaw_rad * 180.0 / std::numbers::pi;  // Convert rad to deg
  yaw_deg = std::fmod(yaw_deg, 360.0);                // Adjust to fit within 0 to 360 degrees
  if (yaw_deg < 0) {
    yaw_deg += 360.0;  // Move into positive range
  }
  // Convert to int [0,3600]
  auto yaw_int = static_cast<HeadingValue::_value_type>(yaw_deg * 10);
  // Correct 0 yaw (from east to north)
  auto yaw_int_corrected = ((900 - yaw_int) + 3600) % 3600;
  assert(HeadingValue::MIN <= yaw_int_corrected);
  assert(yaw_int_corrected <= HeadingValue::MAX);

  heading.heading_value.value = yaw_int_corrected;

  // Heading Confidence

  // Last entry on diagonal is yaw variance
  auto variance = object_stamped->object.state.covariance[54];
  // Calculate standard deviation
  auto sigma_rad = std::sqrt(variance);

  // Convert radians to 1/10 degrees
  auto sigma_deg = sigma_rad * (180.0 / std::numbers::pi) * 10;

  // Determine confidence with confidence level of 95%
  // (we assume a normal distribution) => 1.96 * sigma
  auto confidence = static_cast<HeadingConfidence::_value_type>(1.96 * sigma_deg);

  // Map confidence according to standard
  if (confidence <= HeadingConfidence::EQUAL_OR_WITHIN_ZERO_POINT_ONE_DEGREE) {
    heading.heading_confidence.value = HeadingConfidence::EQUAL_OR_WITHIN_ZERO_POINT_ONE_DEGREE;
  } else if (confidence >= HeadingConfidence::OUT_OF_RANGE) {
    heading.heading_confidence.value = HeadingConfidence::OUT_OF_RANGE;
  } else {
    assert(HeadingConfidence::MIN <= confidence && confidence <= HeadingConfidence::MAX);
    heading.heading_confidence.value = confidence;
  }

  this->high_frequency_container_->basic_vehicle_container_high_frequency.heading = heading;

  return *this;
}

HighFrequencyContainerBuilder & HighFrequencyContainerBuilder::speed(
  const ufil_msgs::msg::ObjectStamped::UniquePtr & object_stamped)
{
  Speed speed;

  // Speed Value

  // Convert speed from m/s to cm/s
  auto speed_m_s = std::sqrt(std::pow(object_stamped->object.state.state.v_x, 2.0f) +
    std::pow(object_stamped->object.state.state.v_y, 2.0f));
     // The standard defines the speed here as the speed in driving direction => x
  auto speed_cm_s = static_cast<SpeedValue::_value_type>(
    std::abs(speed_m_s * 100));  // Absolute value as int in cm/s
  // Cap speed value according to standard
  auto speed_capped = capValueToRange(speed_cm_s, SpeedValue::MIN, SpeedValue::UNAVAILABLE);
  assert(SpeedValue::MIN <= speed_capped && speed_capped <= SpeedValue::MAX);
  speed.speed_value.value = speed_capped;

  // Speed Confidence

  // First entry on diagonal is speed in x direction
  auto variance = object_stamped->object.state.covariance[18];
  // Calculate standard deviation
  auto sigma_m_s = std::sqrt(variance);

  // Convert m/s to cm/s
  auto sigma_cm_s = sigma_m_s * 100;

  // Determine confidence with confidence level of 95%
  // (we assume a normal distribution) => 1.96 * sigma
  auto confidence = static_cast<SpeedConfidence::_value_type>(1.96 * sigma_cm_s);

  // Map confidence according to standard
  if (confidence <= SpeedConfidence::EQUAL_OR_WITHIN_ONE_CENTIMETER_PER_SEC) {
    speed.speed_confidence.value = SpeedConfidence::EQUAL_OR_WITHIN_ONE_CENTIMETER_PER_SEC;
  } else if (confidence >= SpeedConfidence::OUT_OF_RANGE) {
    speed.speed_confidence.value = SpeedConfidence::OUT_OF_RANGE;
  } else {
    assert(SpeedConfidence::MIN <= confidence && confidence <= SpeedConfidence::MAX);
    speed.speed_confidence.value = confidence;
  }

  this->high_frequency_container_->basic_vehicle_container_high_frequency.speed = speed;

  return *this;
}

HighFrequencyContainerBuilder & HighFrequencyContainerBuilder::driveDirection(
  const ufil_msgs::msg::ObjectStamped::UniquePtr & object_stamped)
{
  // Decide drive direction based on speed in driving direction
  const float orientation = object_stamped->object.state.state.yaw;
  const float driving_orientation = std::atan2(object_stamped->object.state.state.v_y,
      object_stamped->object.state.state.v_x);
  const float angular_diff = std::atan2(std::sin(orientation - driving_orientation),
      std::cos(orientation - driving_orientation));


  this->high_frequency_container_->basic_vehicle_container_high_frequency.drive_direction.value =
    angular_diff <= 1.57f ? DriveDirection::FORWARD :
    DriveDirection::BACKWARD;

  return *this;
}

HighFrequencyContainerBuilder & HighFrequencyContainerBuilder::acceleration(
  const ufil_msgs::msg::ObjectStamped::UniquePtr & object_stamped)
{
  // Longitudinal Acceleration
  {
    // Longitudinal Acceleration Value
    auto long_accel_dm = static_cast<LongitudinalAccelerationValue::_value_type>(
      object_stamped->object.state.state.a_x * 10);
    auto long_accel_capped = capValueToRange(
      long_accel_dm, LongitudinalAccelerationValue::MIN,
      static_cast<LongitudinalAccelerationValue::_value_type>(160));

    this->high_frequency_container_->basic_vehicle_container_high_frequency
    .longitudinal_acceleration.longitudinal_acceleration_value.value = long_accel_capped;
    // Longitudinal Acceleration Confidence

    // First entry on diagonal is acceleration in x direction
    auto accel_long_variance = object_stamped->object.state.covariance[36];
    // Calculate standard deviation
    auto accel_long_sigma_m_s = std::sqrt(accel_long_variance);

    // Convert m/s to dm/s
    auto accel_long_sigma_dm_s = accel_long_sigma_m_s * 10;

    // Determine confidence with confidence level of 95%
    // (we assume a normal distribution) => 1.96 * sigma
    auto accel_long_confidence =
      static_cast<AccelerationConfidence::_value_type>(1.96 * accel_long_sigma_dm_s);

    // Map confidence according to standard
    if (accel_long_confidence <= AccelerationConfidence::POINT_ONE_METER_PER_SEC_SQUARED) {
      this->high_frequency_container_->basic_vehicle_container_high_frequency
      .longitudinal_acceleration.longitudinal_acceleration_confidence.value =
        AccelerationConfidence::POINT_ONE_METER_PER_SEC_SQUARED;
    } else if (accel_long_confidence >= AccelerationConfidence::OUT_OF_RANGE) {
      this->high_frequency_container_->basic_vehicle_container_high_frequency
      .longitudinal_acceleration.longitudinal_acceleration_confidence.value =
        AccelerationConfidence::OUT_OF_RANGE;
    } else {
      assert(
        AccelerationConfidence::MIN <= accel_long_confidence &&
        accel_long_confidence <= AccelerationConfidence::MAX);
      this->high_frequency_container_->basic_vehicle_container_high_frequency
      .longitudinal_acceleration.longitudinal_acceleration_confidence.value =
        accel_long_confidence;
    }
  }

  // Lateral Acceleration
  {
    // Lateral Acceleration Value
    auto lat_accel_dm = static_cast<LongitudinalAccelerationValue::_value_type>(
      object_stamped->object.state.state.a_y * 10);
    auto lat_accel_capped = capValueToRange(
      lat_accel_dm, LateralAccelerationValue::MIN,
      static_cast<LateralAccelerationValue::_value_type>(160));

    this->high_frequency_container_->basic_vehicle_container_high_frequency.lateral_acceleration
    .lateral_acceleration_value.value = lat_accel_capped;

    // Lateral Acceleration Confidence

    // Second entry on diagonal is acceleration in y direction
    auto accel_long_variance = object_stamped->object.state.covariance[45];
    // Calculate standard deviation
    auto accel_long_sigma_m_s = std::sqrt(accel_long_variance);

    // Convert m/s to dm/s
    auto accel_long_sigma_dm_s = accel_long_sigma_m_s * 10;

    // Determine confidence with confidence level of 95%
    // (we assume a normal distribution) => 1.96 * sigma
    auto accel_lat_confidence =
      static_cast<AccelerationConfidence::_value_type>(1.96 * accel_long_sigma_dm_s);

    // Map confidence according to standard
    if (accel_lat_confidence <= AccelerationConfidence::POINT_ONE_METER_PER_SEC_SQUARED) {
      this->high_frequency_container_->basic_vehicle_container_high_frequency.lateral_acceleration
      .lateral_acceleration_confidence.value =
        AccelerationConfidence::POINT_ONE_METER_PER_SEC_SQUARED;
    } else if (accel_lat_confidence >= AccelerationConfidence::OUT_OF_RANGE) {
      this->high_frequency_container_->basic_vehicle_container_high_frequency.lateral_acceleration
      .lateral_acceleration_confidence.value = AccelerationConfidence::OUT_OF_RANGE;
    } else {
      assert(
        AccelerationConfidence::MIN <= accel_lat_confidence &&
        accel_lat_confidence <= AccelerationConfidence::MAX);
      this->high_frequency_container_->basic_vehicle_container_high_frequency.lateral_acceleration
      .lateral_acceleration_confidence.value = accel_lat_confidence;
    }
  }

  // Vertical Acceleration
  {
    // Vertical Acceleration Value
    this->high_frequency_container_->basic_vehicle_container_high_frequency.vertical_acceleration
    .vertical_acceleration_value.value = AccelerationConfidence::UNAVAILABLE;

    // Vertical Acceleration Confidence
    this->high_frequency_container_->basic_vehicle_container_high_frequency.vertical_acceleration
    .vertical_acceleration_confidence.value = AccelerationConfidence::UNAVAILABLE;
  }

  return *this;
}

HighFrequencyContainerBuilder & HighFrequencyContainerBuilder::yawRate(
  const ufil_msgs::msg::ObjectStamped::UniquePtr & object_stamped)
{
  YawRate yaw_rate;

  // Yaw Rate Value
  // Here, different to the heading,
  // positive values indicate that the rotation is anti-clockwise (i.e. to the left).
  // This is compatible with the ROS2 standard.
  auto yaw_rate_rad = object_stamped->object.state.state.yaw_rate;
  // Convert to 0.01 deg/sec (centidegree/seconds ?)
  auto yaw_rate_cd_s =
    static_cast<YawRateValue::_value_type>(yaw_rate_rad * (180.0 / std::numbers::pi) * 100.0);
  // Cap to allowed value range
  auto yaw_rate_capped =
    capValueToRange(yaw_rate_cd_s, YawRateValue::MIN, YawRateValue::UNAVAILABLE);

  yaw_rate.yaw_rate_value.value = yaw_rate_capped;

  // Yaw Rate Confidence
  // Last entry on diagonal is yaw rate variance
  auto variance = object_stamped->object.state.covariance[63];
  // Convert to 0.01 deg/sec (centidegree/seconds ?)
  auto sigma_rad = std::sqrt(variance);

  // Convert radians to degrees
  auto sigma_deg = sigma_rad * (180.0 / std::numbers::pi);

  // Determine confidence with confidence level of 95%
  // (we assume a normal distribution) => 1.96 * sigma
  auto confidence = static_cast<HeadingConfidence::_value_type>(1.96 * sigma_deg);

  // Set confidence according to table defined in standard
  if (confidence <= 0.01) {
    yaw_rate.yaw_rate_confidence.value = YawRateConfidence::DEG_SEC_000_01;
  } else if (confidence <= 0.05) {
    yaw_rate.yaw_rate_confidence.value = YawRateConfidence::DEG_SEC_000_05;
  } else if (confidence <= 0.1) {
    yaw_rate.yaw_rate_confidence.value = YawRateConfidence::DEG_SEC_000_10;
  } else if (confidence <= 1) {
    yaw_rate.yaw_rate_confidence.value = YawRateConfidence::DEG_SEC_001_00;
  } else if (confidence <= 5) {
    yaw_rate.yaw_rate_confidence.value = YawRateConfidence::DEG_SEC_005_00;
  } else if (confidence <= 10) {
    yaw_rate.yaw_rate_confidence.value = YawRateConfidence::DEG_SEC_010_00;
  } else if (confidence <= 100) {
    yaw_rate.yaw_rate_confidence.value = YawRateConfidence::DEG_SEC_100_00;
  } else {
    yaw_rate.yaw_rate_confidence.value = YawRateConfidence::OUT_OF_RANGE;
  }

  this->high_frequency_container_->basic_vehicle_container_high_frequency.yaw_rate = yaw_rate;

  return *this;
}

std::unique_ptr<HighFrequencyContainer> HighFrequencyContainerBuilder::get()
{
  auto result = std::move(this->high_frequency_container_);
  this->reset();
  return result;
}

}  // namespace etsi_message_converter
