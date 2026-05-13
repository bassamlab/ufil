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


#include "ufil_etsi/BasicContainerBuilder.hpp"

#include <cassert>
#include <GeographicLib/UTMUPS.hpp>

#include "ufil_etsi/Utilities.hpp"

using etsi_its_cam_msgs::msg::StationType;
using etsi_its_cam_msgs::msg::AltitudeValue;
using etsi_its_cam_msgs::msg::SemiAxisLength;
using etsi_its_cam_msgs::msg::Latitude;
using etsi_its_cam_msgs::msg::Longitude;
using etsi_its_cam_msgs::msg::ReferencePosition;
using etsi_its_cam_msgs::msg::HeadingValue;
using etsi_its_cam_msgs::msg::PosConfidenceEllipse;


namespace etsi_message_converter
{

void BasicContainerBuilder::reset()
{
  // Reset CAM
  this->basic_container_ = std::make_unique<BasicContainer>();
}

BasicContainerBuilder & etsi_message_converter::BasicContainerBuilder::stationType(
  const ufil_msgs::msg::ObjectStamped::UniquePtr & object_stamped)
{
  auto & classification_array = object_stamped->object.classification.classification;
  ufil_msgs::msg::Classification::_classification_type::size_type classification = std::distance(
      classification_array.begin(),
      std::max_element(classification_array.begin(), classification_array.end()));

  if (classification == ufil_msgs::msg::Classification::CAR) {
    this->basic_container_->station_type.value = StationType::PASSENGER_CAR;
  } else if (classification == ufil_msgs::msg::Classification::TRUCK) {
    this->basic_container_->station_type.value = StationType::LIGHT_TRUCK;  // Choose light truck
  } else if (classification == ufil_msgs::msg::Classification::MOTORCYCLE) {
    this->basic_container_->station_type.value = StationType::MOTORCYCLE;
  } else if (classification == ufil_msgs::msg::Classification::BICYCLE) {
    this->basic_container_->station_type.value = StationType::CYCLIST;
  } else if (classification == ufil_msgs::msg::Classification::PEDESTRIAN) {
    this->basic_container_->station_type.value = StationType::PEDESTRIAN;
  } else if (classification == ufil_msgs::msg::Classification::STATIONARY) {
    this->basic_container_->station_type.value = StationType::ROAD_SIDE_UNIT;
  } else {
    this->basic_container_->station_type.value = StationType::UNKNOWN;
  }
  return *this;
}

BasicContainerBuilder & etsi_message_converter::BasicContainerBuilder::referencePosition(
  const ufil_msgs::msg::ObjectStamped::UniquePtr & object_stamped)
{
  // The reference point shall be the ground position of the
  // centre of the front side of the bounding box of the
  // vehicle. Thus, we have to project the position we get from
  // the NavSatFix (which is in the center of the vehicle) to
  // the demanded position. This is done based on the heading and
  // the vehicle length. We need cartesian coordinates to
  // calculate this. Thus, we convert to and from UTM.

  // Convert LatLong to UTM (altitude stays the same, since we don't consider pitch)
  std::string utm_zone = object_stamped->header.frame_id;
  utm_zone = utm_zone.substr(utm_zone.find('_') + 1);

  int zone;
  bool northp = true;
  GeographicLib::UTMUPS::DecodeZone(utm_zone, zone, northp);

  // TODO(Simon Schäfer): Find out why Lucas Hegerath commented this seciton out.
  // Calculate displacements along x and y axes
  // double displacement_x =
  //     object_stamped->
  //      object.dimension.dimension.length * std::cos(object_stamped->object.state.state.yaw);
  // double displacement_y =
  //     object_stamped->
  //      object.dimension.dimension.length * std::sin(object_stamped->object.state.state.yaw);

  // Calculate coordinates of the front center
  // + displacement_x / 2;  // Half of the length
  double front_center_easting = object_stamped->object.state.state.x;
  // + displacement_y / 2;  // Half of the length
  double front_center_northing = object_stamped->object.state.state.y;

  // Convert back to LatLong
  GeographicLib::Math::real latitude_front, longitude_front;
  GeographicLib::UTMUPS::Reverse(zone, northp, front_center_easting, front_center_northing,
      latitude_front,
                                 longitude_front);

  // Convert values to 0.1 microdegree
  auto latitude_deg = static_cast<Latitude::_value_type>(latitude_front * 1e7);
  auto longitude_deg = static_cast<Longitude::_value_type>(longitude_front * 1e7);

  assert(Latitude::MIN <= latitude_deg && latitude_deg <= Latitude::MAX - 1);
  assert(Longitude::MIN <= longitude_deg && longitude_deg <= Longitude::MAX - 1);

  ReferencePosition reference_position;
  reference_position.latitude.value = latitude_deg;
  reference_position.longitude.value = longitude_deg;
  reference_position.altitude.altitude_value.value = AltitudeValue::UNAVAILABLE;

  // Altitude Confidence
  reference_position.altitude.altitude_confidence.value =
    etsi_its_cam_msgs::msg::AltitudeConfidence::UNAVAILABLE;

  // Position Confidence Ellipse

  PosConfidenceEllipse pos_confidence_ellipse;
  // We fix the orientation to north, this makes the calculations easier.
  pos_confidence_ellipse.semi_major_orientation.value = HeadingValue::WGS84_NORTH;

  // Position covariance [m^2] defined relative to a tangential plane through the reported position.
  // The components are East, North, and Up (ENU), in row-major order.
  auto longitude_variance_m2 = object_stamped->object.state.covariance[0];
  auto latitude_variance_m2 = object_stamped->object.state.covariance[9];

  // Calculate standard deviation in centimeters
  auto longitude_sigma_cm = std::sqrt(longitude_variance_m2) * 100;
  auto latitude_sigma_cm = std::sqrt(latitude_variance_m2) * 100;

  // Determine confidence with confidence level of 95%
  // (we assume a normal distribution) => 1.96 * sigma
  auto longitude_confidence = static_cast<SemiAxisLength::_value_type>(1.96 * longitude_sigma_cm);
  auto latitude_confidence = static_cast<SemiAxisLength::_value_type>(1.96 * latitude_sigma_cm);

  // Cap confidence values to defined intervals
  auto longitude_confidence_capped =
    capValueToRange(longitude_confidence, SemiAxisLength::ONE_CENTIMETER,
      SemiAxisLength::OUT_OF_RANGE);
  auto latitude_confidence_capped =
    capValueToRange(latitude_confidence, SemiAxisLength::ONE_CENTIMETER,
      SemiAxisLength::OUT_OF_RANGE);

  // Semi Major is Latitude, since we defined the semi major orientation as north
  pos_confidence_ellipse.semi_major_confidence.value = latitude_confidence_capped;
  pos_confidence_ellipse.semi_minor_confidence.value = longitude_confidence_capped;

  // Set confidence ellipse
  reference_position.position_confidence_ellipse = pos_confidence_ellipse;

  // Finally set reference position
  this->basic_container_->reference_position = reference_position;

  return *this;
}

std::unique_ptr<BasicContainer> BasicContainerBuilder::get()
{
  auto result = std::move(this->basic_container_);
  this->reset();
  return result;
}

}  // namespace etsi_message_converter
