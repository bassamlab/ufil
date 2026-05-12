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

#include <eigen3/Eigen/Dense>
#include <tf2/exceptions.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <ufil_ros/ufil_ros.hpp>

#include <GeographicLib/UTMUPS.hpp>
#include <etsi_its_cam_msgs/msg/cam.hpp>
#include <etsi_its_msgs_utils/cam_access.hpp>
#include <ufil_msgs/msg/object_list.hpp>
#include <rclcpp/rclcpp.hpp>

// Use using-declarations instead of using-directives
using std::chrono_literals::operator""ms;
using std::chrono_literals::operator""s;
using etsi_its_cam_msgs::msg::CAM;
using etsi_its_cam_msgs::msg::YawRateConfidence;
using etsi_its_cam_msgs::msg::StationType;
using etsi_its_cam_msgs::msg::VehicleWidth;
using etsi_its_cam_msgs::msg::VehicleLengthValue;
using etsi_its_cam_msgs::msg::VehicleLengthConfidenceIndication;
using etsi_its_cam_msgs::msg::YawRateValue;
using etsi_its_cam_msgs::msg::HeadingConfidence;
using etsi_its_cam_msgs::msg::HeadingValue;
using etsi_its_cam_msgs::msg::AccelerationConfidence;
using etsi_its_cam_msgs::msg::LateralAccelerationValue;
using etsi_its_cam_msgs::msg::LongitudinalAccelerationValue;
using etsi_its_cam_msgs::msg::SpeedConfidence;
using etsi_its_cam_msgs::msg::DriveDirection;
using etsi_its_cam_msgs::msg::SpeedValue;
using etsi_its_cam_msgs::msg::SemiAxisLength;
using etsi_its_cam_msgs::msg::VehicleLengthValue;
using etsi_its_cam_msgs::msg::HeadingValue;
using etsi_its_cam_msgs::msg::Latitude;
using etsi_its_cam_msgs::msg::Longitude;

class VehicleCamReceiverNode : public rclcpp::Node
{
private:
  // Subscribers
  rclcpp::Subscription<CAM>::SharedPtr cam_subscriber_;

  // Publishers
  rclcpp::Publisher<ufil_msgs::msg::ObjectList>::SharedPtr object_list_publisher_;

  // TF Buffer
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;

  // Last received messages
  std::string target_frame_ = "utm_32n";

  template<typename T>
  static inline T degreesToRadians(T degrees)
  {
    return degrees * static_cast<T>(std::numbers::pi / 180.0);
  }

  template<typename T>
  static inline T normalize_angle(T angle)
  {
    const T result = fmod(angle + std::numbers::pi, 2.0 * std::numbers::pi);
    if (result <= 0.0) {
      return result + std::numbers::pi;
    }
    return result - std::numbers::pi;
  }

  rclcpp::Time extractStamp(const CAM::SharedPtr & cam_msg)
  {
    int64_t shift_ns = this->get_parameter("time_shift_nanosec").as_int();
    int64_t now_ns = this->now().nanoseconds();
    int64_t shifted_time = shift_ns + now_ns;
    auto unix_timestamp = etsi_its_cam_msgs::access::getUnixNanosecondsFromGenerationDeltaTime(
        cam_msg->cam.generation_delta_time, shifted_time);
    auto time_stamp = rclcpp::Time(static_cast<int64_t>(unix_timestamp));
    time_stamp = time_stamp -
      rclcpp::Duration(std::chrono::nanoseconds(this->get_parameter(
      "time_shift_nanosec").as_int()));

    return time_stamp;
  }

  static std::string extractFrameID(const CAM::SharedPtr & cam_msg)
  {
    auto latitude_cam =
      cam_msg->cam.cam_parameters.basic_container.reference_position.latitude.value;
    auto longitude_cam =
      cam_msg->cam.cam_parameters.basic_container.reference_position.longitude.value;

    if (latitude_cam == Latitude::UNAVAILABLE || longitude_cam == Longitude::UNAVAILABLE) {
      std::cerr << "latitude or longitude not available for frame id";
    }
    // All values needed are valid -> we can correctly determine position
    // Convert values to normal, ROS2-conform units
    auto latitude = latitude_cam * 1e-7;
    auto longitude = longitude_cam * 1e-7;
    int zone;
    bool northp;
    GeographicLib::Math::real easting_front_center, northing_front_center;
    GeographicLib::UTMUPS::Forward(latitude, longitude, zone, northp, easting_front_center,
      northing_front_center);
    std::string zone_encoded =
      GeographicLib::UTMUPS::EncodeZone(zone, northp);
    std::string source_frame = "utm_" + zone_encoded;
    return source_frame;
  }

  static std::tuple<Eigen::Vector2d,
    Eigen::Matrix2d> extractPositionWithCovariance(const CAM::SharedPtr & cam_msg)
  {
    // Position

    // Latitude & Longitude
    auto latitude_cam =
      cam_msg->cam.cam_parameters.basic_container.reference_position.latitude.value;
    auto longitude_cam =
      cam_msg->cam.cam_parameters.basic_container.reference_position.longitude.value;

    // Check whether both values are valid
    if (latitude_cam == Latitude::UNAVAILABLE || longitude_cam == Longitude::UNAVAILABLE) {
      // Position not known -> 0 for value and -1 for covariance
      Eigen::Vector2d position{0., 0.};
      Eigen::Matrix2d covariance{{-1., 0.}, {0., -1.}};
      return {position, covariance};
    }

    // Heading (needed for projecting the position back to the center of the object,
    // CAM reference position is front
    // middle)
    // TODO(dummy) Strictly, we would have to consider the variance here as well
    auto heading_cam =
      cam_msg->cam.cam_parameters.high_frequency_container.basic_vehicle_container_high_frequency
      .heading.heading_value.value;

    // Check whether heading is valid
    if (heading_cam == HeadingValue::UNAVAILABLE) {
      // We cannot correctly determine object center
      // -> position not known -> 0 for value and -1 for covariance
      Eigen::Vector2d position{0., 0.};
      Eigen::Matrix2d covariance{{-1., 0.}, {0., -1.}};
      return {position, covariance};
    }

    // Vehicle Length (needed for projecting the position back to the
    // center of the object, CAM reference position is front middle)
    // TODO(dummy): Strictly, we would have to consider the variance here as well
    auto vehicle_length_cam = cam_msg->cam.cam_parameters.high_frequency_container
      .basic_vehicle_container_high_frequency.vehicle_length.vehicle_length_value.value;

    // Check whether heading is valid
    if (vehicle_length_cam == VehicleLengthValue::UNAVAILABLE ||
      vehicle_length_cam == VehicleLengthValue::OUT_OF_RANGE)
    {
      // We cannot correctly determine object center
      // -> position not known -> 0 for value and -1 for covariance
      Eigen::Vector2d position{0., 0.};
      Eigen::Matrix2d covariance{{-1., 0.}, {0., -1.}};
      return {position, covariance};
    }

    // All values needed are valid -> we can correctly determine position
    // Convert values to normal, ROS2-conform units
    auto latitude = latitude_cam * 1e-7;
    auto longitude = longitude_cam * 1e-7;
    // auto yaw = degreesToRadians(std::fmod((90. - (heading_cam / 10.)), 360));
    // auto yaw_normalized = normalize_angle(yaw);
    // auto vehicle_length = vehicle_length_cam / 100.;

    // Convert LatLong to UTM
    int zone;
    bool northp;
    GeographicLib::Math::real easting_front_center, northing_front_center;
    GeographicLib::UTMUPS::Forward(latitude, longitude, zone, northp, easting_front_center,
      northing_front_center);

    // TODO(Simon Schäfer): Find out why Lucas Hegerath commented this seciton out.
    // Calculate displacement to project position to center
    // double displacement_easting = vehicle_length * std::cos(yaw_normalized);
    // double displacement_northing = vehicle_length * std::sin(yaw_normalized);

    // Apply displacement
    auto easting = easting_front_center;    //- (displacement_easting / 2);
    auto northing = northing_front_center;  //- (displacement_northing / 2);

    Eigen::Vector2d position{easting, northing};

    // Confidences
    auto confidence_orientation = cam_msg->cam.cam_parameters.basic_container.reference_position
      .position_confidence_ellipse.semi_major_orientation.value;
    auto semi_minor_confidence_cam = cam_msg->cam.cam_parameters.basic_container.reference_position
      .position_confidence_ellipse.semi_minor_confidence.value;
    auto semi_major_confidence_cam = cam_msg->cam.cam_parameters.basic_container.reference_position
      .position_confidence_ellipse.semi_major_confidence.value;

    // Check if confidences are valid
    if (confidence_orientation == HeadingValue::UNAVAILABLE ||
      semi_minor_confidence_cam == SemiAxisLength::UNAVAILABLE ||
      semi_minor_confidence_cam == SemiAxisLength::OUT_OF_RANGE ||
      semi_major_confidence_cam == SemiAxisLength::UNAVAILABLE ||
      semi_major_confidence_cam == SemiAxisLength::OUT_OF_RANGE)
    {
      // We cannot determine the covariance -> 0 for covariances
      Eigen::Matrix2d covariance{{0., 0.}, {0., 0.}};
      return {position, covariance};
    }

    // Accuracy with confidence level of 95%
    // (we assume a normal distribution) => std_dev = accuracy / 1.96
    auto semi_minor_std_dev_cam = semi_minor_confidence_cam / 1.96;
    auto semi_major_std_dev_cam = semi_major_confidence_cam / 1.96;

    // Convert axis lengths to from cm to m
    auto semi_minor_std_dev_m = semi_minor_std_dev_cam / 100.;
    auto semi_major_std_dev_m = semi_major_std_dev_cam / 100.;

    // Calculate variance (square std_dev)
    auto semi_minor_variance = semi_minor_std_dev_m * semi_minor_std_dev_m;
    auto semi_major_variance = semi_major_std_dev_m * semi_major_std_dev_m;

    Eigen::Matrix2d covariance_ellipse{{semi_major_variance, 0.}, {0., semi_minor_variance}};

    // Convert orientation of covariance from 0.1 deg to rad
    auto confidence_orientation_rad = degreesToRadians(confidence_orientation / 10.);

    // Rotation Matrix

    // Rotate by 90 - confidence_orientation clockwise
    //  = -90 + confidence_orientation to align semi major with easting
    // in UTM coordinate frame
    Eigen::Matrix2d rotation{{std::sin(confidence_orientation_rad),
      std::cos(confidence_orientation_rad)},
      {-std::cos(confidence_orientation_rad), std::sin(confidence_orientation_rad)}};

    // Rotate covariance matrix
    Eigen::Matrix2d covariance_utm = rotation * covariance_ellipse * rotation.transpose();

    return {position, covariance_utm};
  }

  static std::tuple<Eigen::Vector2d,
    Eigen::Matrix2d> extractVelocityWithCovariance(const CAM::SharedPtr & cam_msg)
  {
    // Velocity
    auto speed_cam =
      cam_msg->cam.cam_parameters.high_frequency_container.basic_vehicle_container_high_frequency.
      speed
      .speed_value.value;
    auto drive_direction =
      cam_msg->cam.cam_parameters.high_frequency_container.basic_vehicle_container_high_frequency
      .drive_direction.value;

    // Check whether values are valid
    if (speed_cam == SpeedValue::UNAVAILABLE || drive_direction == DriveDirection::UNAVAILABLE) {
      // We cannot determine the velocity -> 0 for value and -1 for covariance
      Eigen::Vector2d velocity{0., 0.};
      Eigen::Matrix2d covariance{{-1., 0.}, {0., -1.}};
      return {velocity, covariance};
    }

    // Convert speed from cm/s to m/s
    auto speed = speed_cam / 100.;

    // Combine with driving direction to get velocity in x direction of vehicle
    auto v_x_vehicle = speed * (drive_direction == DriveDirection::FORWARD ? 1 : -1);

    // Velocity in y direction of vehicle is 0
    auto v_y_vehicle = 0.;

    Eigen::Vector2d velocity_vehicle{v_x_vehicle, v_y_vehicle};

    // Covariance

    // Get confidence
    auto speed_confidence_cam = cam_msg->cam.cam_parameters.high_frequency_container
      .basic_vehicle_container_high_frequency.speed.speed_confidence.value;

    // Check if confidence is valid
    if (speed_confidence_cam == SpeedConfidence::UNAVAILABLE ||
      speed_confidence_cam == SpeedConfidence::OUT_OF_RANGE)
    {
      // We don't know the confidence -> 0 for covariances
      Eigen::Matrix2d covariance{{0., 0.}, {0., 0.}};
      return {velocity_vehicle, covariance};
    }

    // Accuracy with confidence level of 95% (we assume a normal distribution)
    // => std_dev = accuracy / 1.96
    auto speed_std_deviation_cam = speed_confidence_cam / 1.96;

    // Convert from cm/s to m/s
    auto speed_std_deviation_ms = speed_std_deviation_cam / 100.;

    // Calculate variance (square std_dev)
    auto v_x_variance = speed_std_deviation_ms * speed_std_deviation_ms;
    auto v_y_variance = 1e-9;  // Set to a small value to avoid instabilities

    // Create covariance matrix
    Eigen::Matrix2d covariance_vehicle{{v_x_variance, 0}, {0, v_y_variance}};

    return {velocity_vehicle, covariance_vehicle};
  }

  static std::tuple<Eigen::Vector2d,
    Eigen::Matrix2d> extractAccelerationWithCovariance(const CAM::SharedPtr & cam_msg)
  {
    // Acceleration
    auto long_accel_cam =
      cam_msg->cam.cam_parameters.high_frequency_container.basic_vehicle_container_high_frequency
      .longitudinal_acceleration.longitudinal_acceleration_value.value;
    auto lat_accel_cam =
      cam_msg->cam.cam_parameters.high_frequency_container.basic_vehicle_container_high_frequency
      .lateral_acceleration.lateral_acceleration_value.value;

    // Check whether values are valid
    if (long_accel_cam == LongitudinalAccelerationValue::UNAVAILABLE ||
      lat_accel_cam == LateralAccelerationValue::UNAVAILABLE)
    {
      // We cannot determine the acceleration -> 0 for value and -1 for covariance
      Eigen::Vector2d acceleration{0., 0.};
      Eigen::Matrix2d covariance{{-1., 0.}, {0., -1.}};
      return {acceleration, covariance};
    }

    // Convert acceleration from dm/s^2 to m/s^2
    auto a_x_vehicle = long_accel_cam / 10.;
    auto a_y_vehicle = lat_accel_cam / 10.;

    Eigen::Vector2d acceleration_vehicle{a_x_vehicle, a_y_vehicle};

    // Covariance

    // Get confidences
    auto long_accel_confidence =
      cam_msg->cam.cam_parameters.high_frequency_container.basic_vehicle_container_high_frequency
      .longitudinal_acceleration.longitudinal_acceleration_value.value;
    auto lat_accel_confidence =
      cam_msg->cam.cam_parameters.high_frequency_container.basic_vehicle_container_high_frequency.
      lateral_acceleration
      .lateral_acceleration_value.value;

    // Check if confidences are valid
    if (long_accel_confidence == AccelerationConfidence::UNAVAILABLE ||
      long_accel_confidence == AccelerationConfidence::OUT_OF_RANGE ||
      lat_accel_confidence == AccelerationConfidence::UNAVAILABLE ||
      lat_accel_confidence == AccelerationConfidence::OUT_OF_RANGE)
    {
      // We don't know the confidences -> 0 for variance
      Eigen::Matrix2d covariance{{0., 0.}, {0., 0.}};
      return {acceleration_vehicle, covariance};
    }

    // Accuracy with confidence level of 95%
    // (we assume a normal distribution) => std_dev = accuracy / 1.96
    auto long_accel_std_deviation_cam = long_accel_confidence / 1.96;
    auto lat_accel_std_deviation_cam = lat_accel_confidence / 1.96;

    // Convert from dm/s^2 to m/s^2
    auto long_accel_std_deviation_mss = long_accel_std_deviation_cam / 10.;
    auto lat_accel_std_deviation_mss = lat_accel_std_deviation_cam / 10.;

    // Calculate variance (square std_dev)
    auto a_x_variance = long_accel_std_deviation_mss * long_accel_std_deviation_mss;
    auto a_y_variance = lat_accel_std_deviation_mss * lat_accel_std_deviation_mss;

    // Create covariance matrix
    Eigen::Matrix2d covariance_vehicle{{a_x_variance, 0}, {0, a_y_variance}};

    return {acceleration_vehicle, covariance_vehicle};
  }

  static std::tuple<double, double> extractYawWithVariance(const CAM::SharedPtr & cam_msg)
  {
    // Yaw
    auto yaw_cam =
      cam_msg->cam.cam_parameters.high_frequency_container.basic_vehicle_container_high_frequency.
      heading
      .heading_value.value;

    // Check if yaw is valid
    if (yaw_cam == HeadingValue::UNAVAILABLE) {
      // We don't know the yaw -> 0 for value and -1 for variance
      return {0., -1.};
    }

    // Correct 0 yaw (from north to east)
    auto yaw_corrected = ((900 - yaw_cam) + 3600) % 3600;

    // Convert from 0.1 deg to rad
    auto yaw = degreesToRadians(yaw_corrected / 10.);
    auto yaw_normalized = normalize_angle(yaw);
    // Yaw Variance
    auto yaw_confidence_cam = cam_msg->cam.cam_parameters.high_frequency_container
      .basic_vehicle_container_high_frequency.heading.heading_confidence.value;

    // Check if variance is valid
    if (yaw_confidence_cam == HeadingConfidence::UNAVAILABLE ||
      yaw_confidence_cam == HeadingConfidence::OUT_OF_RANGE)
    {
      // We don't know the variance -> 0 for variance
      return {yaw_normalized, 0.};
    }

    // Accuracy with confidence level of 95%
    // (we assume a normal distribution) => std_dev = accuracy / 1.96
    auto yaw_std_deviation_cam = yaw_confidence_cam / 1.96;

    // Convert from 0.1 deg to rad
    auto yaw_std_deviation_rad = degreesToRadians(yaw_std_deviation_cam / 10.);

    // Calculate variance (square std_dev)
    auto yaw_variance = yaw_std_deviation_rad * yaw_std_deviation_rad;

    return {yaw_normalized, yaw_variance};
  }

  static std::tuple<double, double> extractYawRateWithVariance(const CAM::SharedPtr & cam_msg)
  {
    // Yaw Rate
    auto yaw_rate_cam =
      cam_msg->cam.cam_parameters.high_frequency_container.basic_vehicle_container_high_frequency
      .yaw_rate.yaw_rate_value.value;

    // Check if yaw rate is valid
    if (yaw_rate_cam == YawRateValue::UNAVAILABLE) {
      // We don't know the yaw rate -> 0 for value and -1 for variance
      return {0., -1.};
    }

    // Convert from 0.01 deg/s to rad/s
    auto yaw_rate = degreesToRadians(yaw_rate_cam / 100.);

    // Yaw Rate Variance
    auto yaw_rate_confidence_level_cam = cam_msg->cam.cam_parameters.high_frequency_container
      .basic_vehicle_container_high_frequency.yaw_rate.yaw_rate_confidence.value;

    // Check if variance is valid
    if (yaw_rate_confidence_level_cam == YawRateConfidence::UNAVAILABLE ||
      yaw_rate_confidence_level_cam == YawRateConfidence::OUT_OF_RANGE)
    {
      // We don't know the variance -> 0 for variance
      return {yaw_rate, 0.};
    }

    // Set variance according to table defined in standard
    // We just use the middle of the corresponding interval -> set value with unit deg/s
    double yaw_rate_confidence_cam = 0.;  // Initialization just to get rid of warning
    if (yaw_rate_confidence_level_cam == YawRateConfidence::DEG_SEC_000_01) {
      yaw_rate_confidence_cam = 0.005;
    } else if (yaw_rate_confidence_level_cam == YawRateConfidence::DEG_SEC_000_05) {
      yaw_rate_confidence_cam = 0.03;
    } else if (yaw_rate_confidence_level_cam == YawRateConfidence::DEG_SEC_000_10) {
      yaw_rate_confidence_cam = 0.075;
    } else if (yaw_rate_confidence_level_cam == YawRateConfidence::DEG_SEC_001_00) {
      yaw_rate_confidence_cam = 0.5;
    } else if (yaw_rate_confidence_level_cam == YawRateConfidence::DEG_SEC_005_00) {
      yaw_rate_confidence_cam = 2.5;
    } else if (yaw_rate_confidence_level_cam == YawRateConfidence::DEG_SEC_010_00) {
      yaw_rate_confidence_cam = 7.5;
    } else if (yaw_rate_confidence_level_cam == YawRateConfidence::DEG_SEC_100_00) {
      yaw_rate_confidence_cam = 55.0;
    }

    // Accuracy with confidence level of 95%
    // (we assume a normal distribution) => std_dev = accuracy / 1.96
    auto yaw_rate_std_deviation_cam = yaw_rate_confidence_cam / 1.96;

    // Convert from deg to rad
    auto yaw_rate_std_deviation_rad = degreesToRadians(yaw_rate_std_deviation_cam);

    // Calculate variance (square std_dev)
    auto yaw_rate_variance = yaw_rate_std_deviation_rad * yaw_rate_std_deviation_rad;

    return {yaw_rate, yaw_rate_variance};
  }

  static ufil_msgs::msg::StateWithCovariance extractState(const CAM::SharedPtr & cam_msg)
  {
    ufil_msgs::msg::StateWithCovariance state_with_covariance;

    // Position
    auto [position, position_covariance] = extractPositionWithCovariance(cam_msg);
    state_with_covariance.state.x = position(0);
    state_with_covariance.state.y = position(1);
    state_with_covariance.covariance[0] = position_covariance(0, 0);
    state_with_covariance.covariance[1] = position_covariance(0, 1);
    state_with_covariance.covariance[8] = position_covariance(1, 0);
    state_with_covariance.covariance[9] = position_covariance(1, 1);

    // Yaw
    auto [yaw, yaw_covariance] = extractYawWithVariance(cam_msg);
    state_with_covariance.state.yaw = yaw;
    state_with_covariance.covariance[54] = yaw_covariance;
    Eigen::Rotation2Dd rotation(yaw);

    // Velocity
    auto [velocity, velocity_covariance] = extractVelocityWithCovariance(cam_msg);
    velocity = rotation * velocity;
    velocity_covariance = rotation * velocity_covariance;

    state_with_covariance.state.v_x = velocity(0);
    state_with_covariance.state.v_y = velocity(1);
    state_with_covariance.covariance[18] = velocity_covariance(0, 0);
    state_with_covariance.covariance[19] = velocity_covariance(0, 1);
    state_with_covariance.covariance[26] = velocity_covariance(1, 0);
    state_with_covariance.covariance[27] = velocity_covariance(1, 1);

    // Acceleration
    auto [acceleration, acceleration_covariance] = extractAccelerationWithCovariance(cam_msg);
    acceleration = rotation * acceleration;
    acceleration_covariance = rotation * acceleration_covariance;

    state_with_covariance.state.a_x = acceleration(0);
    state_with_covariance.state.a_y = acceleration(1);
    state_with_covariance.covariance[36] = acceleration_covariance(0, 0);
    state_with_covariance.covariance[37] = acceleration_covariance(0, 1);
    state_with_covariance.covariance[44] = acceleration_covariance(1, 0);
    state_with_covariance.covariance[45] = acceleration_covariance(1, 1);

    // Yaw Rate
    auto [yaw_rate, yaw_rate_covariance] = extractYawRateWithVariance(cam_msg);
    state_with_covariance.state.yaw_rate = yaw_rate;
    state_with_covariance.covariance[63] = yaw_rate_covariance;

    return state_with_covariance;
  }

  static ufil_msgs::msg::DimensionWithCovariance extractDimension(const CAM::SharedPtr & cam_msg)
  {
    ufil_msgs::msg::DimensionWithCovariance dimension_with_covariance;

    // Object Length
    auto cam_length =
      cam_msg->cam.cam_parameters.high_frequency_container.basic_vehicle_container_high_frequency
      .vehicle_length.vehicle_length_value.value;
    auto length_confidence_indication =
      cam_msg->cam.cam_parameters.high_frequency_container.basic_vehicle_container_high_frequency.
      vehicle_length
      .vehicle_length_confidence_indication.value;

    // There are several cases in which we do not know the length
    auto length_unknown =
      length_confidence_indication ==
      VehicleLengthConfidenceIndication::TRAILER_PRESENT_WITH_UNKNOWN_LENGTH;
    length_unknown = length_unknown ||
      (length_confidence_indication ==
      VehicleLengthConfidenceIndication::TRAILER_PRESENCE_IS_UNKNOWN);
    length_unknown = length_unknown ||
      (length_confidence_indication == VehicleLengthConfidenceIndication::UNAVAILABLE);
    length_unknown = length_unknown || (cam_length == VehicleLengthValue::OUT_OF_RANGE);
    length_unknown = length_unknown || (cam_length == VehicleLengthValue::UNAVAILABLE);

    // If the length is know, we set it and set the confidence
    // to a tiny value to avoid instabilities.
    // Otherwise, we leave it at 0 and set the confidence
    // to -1 to indicate that the value is unknown.
    if (!length_unknown) {
      dimension_with_covariance.dimension.length = cam_length / 10.;
      dimension_with_covariance.covariance[0] = 0.1;  // Resolution is 0.1m
    } else {
      dimension_with_covariance.covariance[0] = -1;
    }

    // Object Width
    auto cam_width =
      cam_msg->cam.cam_parameters.high_frequency_container.basic_vehicle_container_high_frequency.
      vehicle_width.value;

    // There are two cases in which we do not know the length
    auto width_unknown = (cam_width == VehicleWidth::OUT_OF_RANGE) ||
      (cam_width == VehicleWidth::UNAVAILABLE);

    // If the width is know, we set it and set the confidence
    // to a tiny value to avoid instabilities.
    // Otherwise, we leave it at 0 and set the confidence to -1
    // to indicate that the value is unknown.
    if (!width_unknown) {
      dimension_with_covariance.dimension.width = cam_width / 10.;
      dimension_with_covariance.covariance[4] = 0.1;  // Resolution is 0.1m
    } else {
      dimension_with_covariance.covariance[4] = -1;
    }

    dimension_with_covariance.covariance[8] = -1;  // Height not provided

    return dimension_with_covariance;
  }

  static ufil_msgs::msg::Classification::_classification_type extractClassification(
    const CAM::SharedPtr & cam_msg)
  {
    // Since we have a slightly different classification scheme, we merge some classes together
    auto cam_classification = cam_msg->cam.cam_parameters.basic_container.station_type.value;
    ufil_msgs::msg::Classification::_classification_type object_classification;
    object_classification.fill(0);

    if (cam_classification == StationType::PASSENGER_CAR) {
      object_classification[ufil_msgs::msg::Classification::CAR] = 1;
    } else if (cam_classification == (StationType::LIGHT_TRUCK | StationType::HEAVY_TRUCK)) {
      object_classification[ufil_msgs::msg::Classification::TRUCK] = 1;
    } else if (cam_classification == (StationType::MOPED | StationType::MOTORCYCLE)) {
      object_classification[ufil_msgs::msg::Classification::MOTORCYCLE] = 1;
    } else if (cam_classification == StationType::CYCLIST) {
      object_classification[ufil_msgs::msg::Classification::BICYCLE] = 1;
    } else if (cam_classification == StationType::PEDESTRIAN) {
      object_classification[ufil_msgs::msg::Classification::PEDESTRIAN] = 1;
    } else if (cam_classification == StationType::ROAD_SIDE_UNIT) {
      object_classification[ufil_msgs::msg::Classification::STATIONARY] = 1;
    } else {
      object_classification[ufil_msgs::msg::Classification::OTHER] = 1;  // Set default
    }

    return object_classification;
  }

  void onCam(const CAM::SharedPtr msg)
  {
    // Since we only consider Vehicle CAMs here, we filter out any cam messages that come from a
    auto station_type = msg->cam.cam_parameters.basic_container.station_type.value;

    if (!(3 <= station_type && station_type <= 10)) {
      // Station type is unknown, pedestrian, cyclist or RSU, discard it
      return;
    }

    // Object List
    ufil_msgs::msg::ObjectList object_list;

    // Header
    object_list.header.stamp = extractStamp(msg);
    // We will convert the CAM coordinates to UTM
    object_list.header.frame_id = extractFrameID(msg);

    // We can now begin with creating the object
    ufil_msgs::msg::Object object;

    // ID (We use the station ID for this)
    object.id = msg->header.station_id.value;

    // State
    object.state = extractState(msg);

    // Dimension
    object.dimension = extractDimension(msg);

    // Existence Probability
    // We just got a message from it, it definitely existed at this time
    object.existence_probability = 1;

    // Classification
    object.classification.classification = extractClassification(msg);

    // Features
    // We "detected" all features
    object.features.fl = true;
    object.features.fr = true;
    object.features.bl = true;
    object.features.br = true;
    object.features.f = true;
    object.features.b = true;
    object.features.l = true;
    object.features.r = true;
    object.features.c = true;

    // Axle Geometry
    // We have no information about this

    // Add object to object list
    object_list.objects.push_back(object);

    // Apply tf
    ufil_msgs::msg::ObjectList transformed_msg;
    try {
      transformed_msg = this->tf_buffer_->transform(object_list, this->target_frame_);
    } catch (tf2::TransformException & ex) {
      RCLCPP_ERROR(this->get_logger(), "Could not transform: %s", ex.what());
      return;
    }

    // Publish finished object list
    this->object_list_publisher_->publish(transformed_msg);
  }

public:
  VehicleCamReceiverNode()
  : Node("vehicle_cam_receiver_node")
  {
    this->declare_parameter("time_shift_nanosec", 0);
    int64_t shift_ns = this->get_parameter("time_shift_nanosec").as_int();
    RCLCPP_INFO(this->get_logger(), "Time shift in nanoseconds: %lu", shift_ns);

    this->declare_parameter("target_frame", this->target_frame_);
    this->target_frame_ = this->get_parameter("target_frame").as_string();
    RCLCPP_INFO(this->get_logger(), "Transform input to target frane: %s",
      this->target_frame_.c_str());

    // Subscribers
    this->cam_subscriber_ = this->create_subscription<CAM>(
        "cam", 10, std::bind(&VehicleCamReceiverNode::onCam, this, std::placeholders::_1));

    // Publishers
    this->object_list_publisher_ = this->create_publisher<ufil_msgs::msg::ObjectList>("object_list",
      10);

    // TF listener
    this->tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    this->tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<VehicleCamReceiverNode>();

  RCLCPP_INFO(node->get_logger(), "Started Vehicle CAM receiver node.");
  rclcpp::spin(node);

  rclcpp::shutdown();
  RCLCPP_INFO(node->get_logger(), "Stopped Vehicle CAM receiver node.");

  return 0;
}
