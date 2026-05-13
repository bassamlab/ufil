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


#ifndef UFIL_OSN_TRACKER__UFIL_OSN_TRACKER_HPP_
#define UFIL_OSN_TRACKER__UFIL_OSN_TRACKER_HPP_

#include <map>
#include <set>
#include <string>
#include <memory>

#include "ufil_osn_tracker/visibility_control.h"
#include "ufil_osn_tracker/definitions.hpp"

#include <ufil_object_tracking/association/association_functions.hpp>
#include <ufil_object_tracking/association/associator.hpp>
#include <ufil_object_tracking/management/initiator/function_initiator.hpp>
#include <ufil_object_tracking/management/pruner/timed_pruner.hpp>
#include <ufil_object_tracking/tracking/multi_target_tracker.hpp>
#include <ufil_object_tracking/types/measurement.hpp>
#include <ufil_object_tracking/types/sensor_fov.hpp>

#include "ufil_osn_tracker/detection.hpp"
#include "ufil_osn_tracker/detector.hpp"

#include "ufil_osn_tracker/existence_predictor_bayesian.hpp"
#include "ufil_osn_tracker/occlusion_predictor.hpp"
#include "ufil_osn_tracker/occlusion_initiator.hpp"

#include <ufil_msgs/msg/sensor_fov.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>

namespace ufil_osn_tracker
{

class OsnObjectTracker
{
public:
  using SharedPtr = std::shared_ptr<OsnObjectTracker>;
  using UniquePtr = std::unique_ptr<OsnObjectTracker>;

public:
  OsnObjectTracker();

  void update(std::set<Detection> && detections, ufil::type::Timestamp timestamp);

  bool setAssociationThreshold(const ufil::type::Scalar threshold);
  bool setHistoryLength(const ufil::type::Scalar history_length);
  bool setClusterPointRange(
    const ufil::type::Scalar min_points,
    const ufil::type::Scalar max_points);
  bool setClusterTolerance(const ufil::type::Scalar tolerance);

  bool setDimensionCovarianceMultiplier(
    const ufil::type::Scalar dimension_covariance_multiplier);

  bool setPositionCovarianceMultiplier(
    const ufil::type::Scalar position_covariance_multiplier);

  bool setOrientationCovarianceMultiplier(
    const ufil::type::Scalar orientation_covariance_multiplier);

  void setCurrentFOV(const ufil::type::SensorFOV & fov);
  void setOcclusionGrid(std::shared_ptr<const nav_msgs::msg::OccupancyGrid> grid);


  const std::set<ufil::type::measurement::Pose2DWithDimension3D> & unassociatedMeasurements() const;
  const std::map<ufil::type::Id,
    ufil::type::measurement::Pose2DWithDimension3D> & associations() const;
  std::size_t detectorAssociationCount() const;
  std::size_t associatorAssociationCount() const;
  std::size_t totalDetectorAssociationCount() const;
  std::size_t totalAssociatorAssociationCount() const;
  const std::map<ufil::type::Id, Track> & tracks() const;

private:
  // Deleter::SharedPtr deleter_;
  ufil::association::FunctionAssociator<Track>::SharedPtr associator_;
  ufil::management::TimedPruner<Track>::SharedPtr pruner_;
  ufil_osn_tracker::Detector::SharedPtr detector_;
  ufil::tracking::MultiTargetTracker<Track, Detection>::UniquePtr tracker_;
  OcclusionPredictor::SharedPtr occlusion_predictor_;
  BayesianExistencePredictor::SharedPtr existence_predictor_;
  OcclusionInitiator::SharedPtr occlusion_initiator_;
};

}  // namespace ufil_osn_tracker

#endif  // UFIL_OSN_TRACKER__UFIL_OSN_TRACKER_HPP_
