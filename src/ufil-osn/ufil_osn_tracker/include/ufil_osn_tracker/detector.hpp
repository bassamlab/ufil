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


#ifndef UFIL_OSN_TRACKER__DETECTOR_HPP_
#define UFIL_OSN_TRACKER__DETECTOR_HPP_

#include <pcl/point_types.h>
#include <pcl/common/pca.h>

#include <map>
#include <memory>
#include <set>
#include <vector>

#include "ufil_osn_tracker/visibility_control.h"
#include "ufil_osn_tracker/detection.hpp"
#include "ufil_osn_tracker/definitions.hpp"
#include "ufil_osn_tracker/detector_types.hpp"
#include "ufil_osn_tracker/clustering.hpp"
#include "ufil_osn_tracker/shape_fitter.hpp"

#include <ufil_object_tracking/detection/detector.hpp>

namespace ufil_osn_tracker
{

class Detector : public ufil::detection::Detector<Track, Detection>
{
public:
  using UniquePtr = std::unique_ptr<Detector>;
  using SharedPtr = std::shared_ptr<Detector>;

  Detector();

  void convert(
    const std::map<ufil::type::Id, Track> & tracks, std::set<Detection> && detections,
    std::map<ufil::type::Id, ufil::type::measurement::Pose2DWithDimension3D> & associations,
    std::set<ufil::type::Id> & unassociated_tracks,
    std::map<ufil::type::Id,
    ufil::type::measurement::Pose2DWithDimension3D> & unassociated_measurements) override;
  std::unique_ptr<Clustering> clustering_;
  std::unique_ptr<ShapeFitter> shape_fitter_;

  ufil::type::Scalar dimension_covariance_multiplier_ = 10.0f;
  ufil::type::Scalar position_covariance_multiplier_ = 10.0f;
  ufil::type::Scalar orientation_covariance_multiplier_ = 1.0f;

  int cluster_assignment_min_number_of_points_ = 1;

public:
  void setDimensionCovarianceMultiplier(
    const ufil::type::Scalar dimension_covariance_multiplier);
  void setPositionCovarianceMultiplier(
    const ufil::type::Scalar position_covariance_multiplier);
  void setOrientationCovarianceMultiplier(
    const ufil::type::Scalar orientation_covariance_multiplier);

  void setClusterPointRange(
    const ufil::type::Scalar min_points,
    const ufil::type::Scalar max_points);
  void setClusterMinDimension(
    const ufil::type::Scalar min_dim_x, const ufil::type::Scalar min_dim_y,
    const ufil::type::Scalar min_dim_z);
  void setClusterTolerance(const ufil::type::Scalar tolerance);
  void setClusterAssignmentMinNumberOfPoints(const int cluster_assignment_min_number_of_points);
};

}  // namespace ufil_osn_tracker

#endif  // UFIL_OSN_TRACKER__DETECTOR_HPP_
