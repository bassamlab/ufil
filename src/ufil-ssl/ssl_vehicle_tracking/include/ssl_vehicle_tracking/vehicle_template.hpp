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

#ifndef SSL_VEHICLE_TRACKING__VEHICLE_TEMPLATE_HPP_
#define SSL_VEHICLE_TRACKING__VEHICLE_TEMPLATE_HPP_

#include <cstddef>
#include <string>
#include <utility>
#include <vector>

#include "ssl_vehicle_tracking/types.hpp"
#include "ssl_vehicle_tracking/visibility_control.hpp"

namespace ssl_vehicle_tracking
{

/// Number of object classes; the classification of a template is an index below.
inline constexpr std::size_t kClassCount = 7;

/// Axle of a vehicle template.
struct Axle
{
  /// True if the axle has a single wheel on the longitudinal axis of the vehicle.
  bool single_track = false;
  double track_width = 0.0;           ///< [m], ignored for single track axles
  double track_width_variance = 0.0;  ///< [m^2]
  /// Longitudinal distance from the vehicle center to the axle, positive to the front [m].
  double center_to_axle = 0.0;
  double center_to_axle_variance = 0.0;  ///< [m^2]
  /// Published with the vehicle as given in the template.
  std::vector<bool> wheel_detected;
};

/// Description of a vehicle template.
struct TemplateDescription
{
  std::string name;
  double length = 0.0;  ///< [m]
  double width = 0.0;   ///< [m]
  /// Object class index (0 car, 1 truck, 2 motorcycle, 3 bicycle, 4 pedestrian, 5 stationary,
  /// 6 other).
  std::size_t classification = 0;
  std::vector<Axle> axles;
};

/**
 * @brief Wheel layout of a detectable vehicle type.
 *
 * The wheels are numbered axle by axle; a two-track axle contributes the wheel at -track_width/2
 * and then the wheel at +track_width/2 in the vehicle y direction.
 */
class SSL_VEHICLE_TRACKING_PUBLIC VehicleTemplate
{
public:
  /// @throws std::invalid_argument if the description is invalid or has fewer than two wheels.
  explicit VehicleTemplate(TemplateDescription description);

  const TemplateDescription & description() const {return description_;}
  std::size_t wheelCount() const {return wheels_.size();}
  /// Wheel positions in the vehicle frame [m].
  const std::vector<Point2D> & wheels() const {return wheels_;}
  double distance(std::size_t a, std::size_t b) const {return distances_[a * wheels_.size() + b];}
  /// Direction from wheel a to wheel b [rad].
  double angle(std::size_t a, std::size_t b) const {return angles_[a * wheels_.size() + b];}
  /// Unordered wheel pairs, longest distance first.
  const std::vector<std::pair<std::size_t, std::size_t>> & pairsByDistance() const
  {
    return pairs_by_distance_;
  }
  /// The pair with the longest distance (the first in row-major order).
  std::pair<std::size_t, std::size_t> longestPair() const {return longest_pair_;}

private:
  TemplateDescription description_;
  std::vector<Point2D> wheels_;
  std::vector<double> distances_;
  std::vector<double> angles_;
  std::vector<std::pair<std::size_t, std::size_t>> pairs_by_distance_;
  std::pair<std::size_t, std::size_t> longest_pair_;
};

/**
 * @brief Loads a vehicle template from a JSON (or YAML) file with the keys length, width,
 *        classification and axles (single_track, track_width, track_width_variance,
 *        center_to_axle, center_to_axle_variance, wheel_detected).
 * @param name name of the template, used in messages
 * @throws std::runtime_error if the file cannot be read or is invalid.
 */
SSL_VEHICLE_TRACKING_PUBLIC VehicleTemplate loadTemplate(
  const std::string & path, const std::string & name);

/**
 * @brief Pose of the template that maps the given template wheels onto the points.
 *
 * The rotation is the circular mean of the angle differences of the first max_pairs template
 * pairs (longest first) whose wheels are both known, weighted by their length. The translation
 * maps the first wheel onto the first point.
 *
 * @param points measured wheel positions
 * @param wheels template wheel of every point
 */
SSL_VEHICLE_TRACKING_PUBLIC Pose2D fitTemplate(
  const VehicleTemplate & vehicle_template, const std::vector<Point2D> & points,
  const std::vector<std::size_t> & wheels, std::size_t max_pairs);

/// Applies the pose to a point.
SSL_VEHICLE_TRACKING_PUBLIC Point2D transformPoint(const Pose2D & pose, const Point2D & point);

}  // namespace ssl_vehicle_tracking

#endif  // SSL_VEHICLE_TRACKING__VEHICLE_TEMPLATE_HPP_
