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

#ifndef UFIL_OBJECT_TRACKING__DETECTION__POINT_CLOUD_DETECTOR_HPP_
#define UFIL_OBJECT_TRACKING__DETECTION__POINT_CLOUD_DETECTOR_HPP_

#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <pcl/conversions.h>

#include <algorithm>
#include <cmath>
#include <map>
#include <memory>
#include <set>
#include <stdexcept>
#include <type_traits>
#include <vector>

// Because of boost/geometry.hpp
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/polygon.hpp>
#include <boost/geometry/strategies/transform/matrix_transformers.hpp>

#include "ufil_object_tracking/detection/detector.hpp"
#include "ufil_object_tracking/detection/detector_types.hpp"
#include "ufil_object_tracking/detection/point_cloud_detection.hpp"
#include "ufil_object_tracking/detection/clustering.hpp"
#include "ufil_object_tracking/detection/shape_fitter.hpp"
#include "ufil_object_tracking/types/measurement.hpp"

namespace ufil
{
namespace detection
{

typedef boost::geometry::model::d2::point_xy<float> point;
typedef boost::geometry::model::box<point> box;
typedef boost::geometry::model::polygon<point> polygon;

inline polygon convertToBoostPolygon(
  const float yaw, const float x, const float y, const float length,
  const float width)
{
  polygon raw, boost_bounding_box;
  box axis_aligned_box{{-0.5f * length, -0.5f * width}, {0.5f * length, 0.5f * width}};
  boost::geometry::assign(raw, axis_aligned_box);
  boost::geometry::strategy::transform::matrix_transformer<float, 2, 2> matrix(
    std::cos(yaw), std::sin(yaw), x, -std::sin(yaw), std::cos(yaw), y, 0, 0, 1);

  boost::geometry::transform(raw, boost_bounding_box, matrix);
  return boost_bounding_box;
}

// A generic, ROS-unaware point-cloud detector. It clusters a PCL point cloud,
// associates clusters to existing tracks via their (oriented) bounding boxes,
// fits a shape (cylinder for pedestrian-like clusters, L-shape otherwise) and
// produces Pose2DWithDimension3D measurements.
//
// It is templated on the track type T; the only requirement is that
// T::MeasurementType is ufil::type::measurement::Pose2DWithDimension3D and that
// the state/dimension/classification types expose the accessors used below.
template<typename T>
class PointCloudDetector : public Detector<T, PointCloudDetection>
{
public:
  using TrackType = T;
  using MeasurementType = typename T::MeasurementType;
  using UniquePtr = std::unique_ptr<PointCloudDetector>;
  using SharedPtr = std::shared_ptr<PointCloudDetector>;

  static_assert(
    std::is_same_v<MeasurementType, ufil::type::measurement::Pose2DWithDimension3D>,
    "PointCloudDetector currently only supports Pose2DWithDimension3D measurements.");

  PointCloudDetector()
  : clustering_(std::make_unique<Clustering>()),
    shape_fitter_(std::make_unique<ShapeFitter>())
  {
  }

  void convert(
    const std::map<ufil::type::Id, T> & tracks, std::set<PointCloudDetection> && detections,
    std::map<ufil::type::Id, MeasurementType> & associations,
    std::set<ufil::type::Id> & unassociated_tracks,
    std::map<ufil::type::Id, MeasurementType> & unassociated_measurements) override
  {
    const PointCloudDetection & detection = *detections.begin();
    const pcl::PCLPointCloud2 & input_cloud = detection.cloud();

    if (input_cloud.width <= 0) {
      for (const auto & track_with_id : tracks) {
        const ufil::type::Id & id = track_with_id.first;
        unassociated_tracks.insert(id);
      }
      return;
    }

    // Cluster cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromPCLPointCloud2(input_cloud, *cloud);

    std::vector<Cluster> euclidean_clusters = clustering_->cluster(cloud);

    for (const auto & track_with_id : tracks) {
      const T & track = track_with_id.second;
      const ufil::type::Id & id = track_with_id.first;

      std::vector<Cluster> clusters_match_object;
      polygon boost_bounding_box = convertToBoostPolygon(
        track.currentState().yaw(), track.currentState().x(), track.currentState().y(),
        track.currentDimension().length(), track.currentDimension().width());
      for (const auto & cluster : euclidean_clusters) {
        int points_in_object = 0;
        for (const auto & cluster_point : cluster.cloud->points) {
          point boost_cluster_point = {cluster_point.x, cluster_point.y};
          if (boost::geometry::within(boost_cluster_point, boost_bounding_box)) {
            points_in_object++;
          }
        }
        if (points_in_object > this->cluster_assignment_min_number_of_points_) {
          clusters_match_object.push_back(cluster);
        }
      }

      if (clusters_match_object.size() > 1) {
        // Merging
        pcl::PointCloud<pcl::PointXYZ>::Ptr new_cloud_cluster(new pcl::PointCloud<pcl::PointXYZ>);
        for (const auto & cluster : clusters_match_object) {
          (*new_cloud_cluster) += (*cluster.cloud);
        }
        new_cloud_cluster->width = new_cloud_cluster->size();
        new_cloud_cluster->height = 1;
        new_cloud_cluster->is_dense = true;

        auto it = std::remove_if(
            euclidean_clusters.begin(), euclidean_clusters.end(),
          [clusters_match_object](Cluster cluster) {
            return std::find_if(clusters_match_object.begin(), clusters_match_object.end(),
                   [&](Cluster const & p) {
                     return p.id == cluster.id;
              }) != clusters_match_object.end();
            });
        euclidean_clusters.erase(it, euclidean_clusters.end());

        clusters_match_object.clear();
        int cluster_id = clusters_match_object.empty() ? 0 : clusters_match_object.front().id;
        Cluster cluster_with_id{cluster_id, new_cloud_cluster};
        clusters_match_object.push_back(cluster_with_id);
      }

      if (clusters_match_object.size() <= 0) {
        unassociated_tracks.insert(id);
        continue;
      }
      Cluster assigned_cluster = clusters_match_object.back();

      pcl::PointXYZ cloud_min_point, cloud_max_point;
      pcl::getMinMax3D<pcl::PointXYZ>(*(assigned_cluster.cloud), cloud_min_point, cloud_max_point);
      float dimension_x = cloud_max_point.x - cloud_min_point.x;
      float dimension_y = cloud_max_point.y - cloud_min_point.y;
      float dimension_z = cloud_max_point.z;

      BoundingBox bounding_box;
      bounding_box.id = assigned_cluster.id;
      // Fix object to ground
      bounding_box.dimension_z = dimension_z;
      bounding_box.position_z = dimension_z * 0.5;

      float yaw = track.currentState().yaw();
      double diameter = 2.0 * sqrt((dimension_x * dimension_y) / M_PI);
      double min_diameter = 2.0 *
        sqrt((shape_fitter_->getMinDimensionX() * shape_fitter_->getMinDimensionY()) / M_PI);

      bool is_predestrian = (track.currentClassification().pedestrian() > 0.8) ||
        (diameter < min_diameter);
      if (is_predestrian) {
        shape_fitter_->fitCylinder(assigned_cluster.cloud, bounding_box);
        bounding_box.yaw = yaw;
      } else {
        shape_fitter_->fitLShape(assigned_cluster.cloud, bounding_box, yaw - M_PI * 0.30f,
            yaw + M_PI * 0.30f);
      }

      MeasurementType measurement;
      measurement.x() = bounding_box.position_x;
      measurement.y() = bounding_box.position_y;

      measurement.yaw() = bounding_box.yaw;
      measurement.dimension().length() = bounding_box.dimension_x;
      measurement.dimension().width() = bounding_box.dimension_y;

      measurement.dimension().height() = bounding_box.dimension_z;
      double distance = measurement.position().norm();
      size_t num_points = assigned_cluster.cloud->size();

      const double sigma_base = 0.1;              // sensor floor
      const double sigma_dist = 0.02 * distance;  // grows with distance
      const double sigma_points = 1.0 / std::sqrt(std::max<double>(num_points, 1.0));

      double sigma = sigma_base + sigma_dist + sigma_points;

      // Clamp to avoid degeneracy
      sigma = std::clamp(sigma, 0.05, 5.0);

      double variance = sigma * sigma;

      measurement.covariance() = MeasurementType::CovarianceMatrixType::Identity();
      measurement.covariance()(MeasurementType::X, MeasurementType::X) =
        position_covariance_multiplier_ * variance;
      measurement.covariance()(MeasurementType::Y, MeasurementType::Y) =
        position_covariance_multiplier_ * variance;
      measurement.covariance()(MeasurementType::YAW, MeasurementType::YAW) =
        orientation_covariance_multiplier_ * variance;

      measurement.dimension().covariance() = distance * variance *
        ufil::type::dimension::Dimension3D::CovarianceMatrixType::Identity();
      associations.insert({id, measurement});

      double N = static_cast<double>(num_points);

      // tuning parameters
      const double N0 = 100.0;   // midpoint (50% confidence)
      const double k = 0.05;     // slope

      double existence = 1.0 / (1.0 + std::exp(-k * (N - N0)));

      measurement.existenceProbability() = std::min(1.0, existence);

      auto it =
        std::remove_if(euclidean_clusters.begin(), euclidean_clusters.end(),
          [clusters_match_object](Cluster cluster) {
            return std::find_if(clusters_match_object.begin(), clusters_match_object.end(),
                   [&](Cluster const & p) {
                     return p.id == cluster.id;
            }) != clusters_match_object.end();
          });
      euclidean_clusters.erase(it, euclidean_clusters.end());
    }

    for (const auto & cluster : euclidean_clusters) {
      pcl::PointXYZ cloud_min_point, cloud_max_point;
      pcl::getMinMax3D<pcl::PointXYZ>(*(cluster.cloud), cloud_min_point, cloud_max_point);
      float dimension_x = cloud_max_point.x - cloud_min_point.x;
      float dimension_y = cloud_max_point.y - cloud_min_point.y;
      float dimension_z = cloud_max_point.z;

      BoundingBox bounding_box;
      bounding_box.id = cluster.id;
      // Fix object to ground
      bounding_box.dimension_z = dimension_z;
      bounding_box.position_z = dimension_z * 0.5;

      if (dimension_x < shape_fitter_->getMinDimensionX() ||
        dimension_y < shape_fitter_->getMinDimensionY())
      {
        shape_fitter_->fitCylinder(cluster.cloud, bounding_box);
      } else {
        shape_fitter_->fitLShape(cluster.cloud, bounding_box);
      }

      MeasurementType measurement;
      measurement.x() = bounding_box.position_x;
      measurement.y() = bounding_box.position_y;
      measurement.yaw() = bounding_box.yaw;
      measurement.dimension().length() = bounding_box.dimension_x;
      measurement.dimension().width() = bounding_box.dimension_y;

      measurement.dimension().height() = bounding_box.dimension_z;

      double distance = measurement.position().norm();
      size_t num_points = cluster.cloud->size();

      const double sigma_base = 0.1;              // sensor floor
      const double sigma_dist = 0.02 * distance;  // grows with distance
      const double sigma_points = 1.0 / std::sqrt(std::max<double>(num_points, 1.0));

      double sigma = sigma_base + sigma_dist + sigma_points;

      // Clamp to avoid degeneracy
      sigma = std::clamp(sigma, 0.05, 5.0);

      double variance = sigma * sigma;

      measurement.covariance() = MeasurementType::CovarianceMatrixType::Identity();
      measurement.covariance()(MeasurementType::X, MeasurementType::X) =
        position_covariance_multiplier_ * variance;
      measurement.covariance()(MeasurementType::Y, MeasurementType::Y) =
        position_covariance_multiplier_ * variance;
      measurement.covariance()(MeasurementType::YAW, MeasurementType::YAW) =
        orientation_covariance_multiplier_ * variance;

      measurement.dimension().covariance() = distance * variance *
        ufil::type::dimension::Dimension3D::CovarianceMatrixType::Identity();

      double N = static_cast<double>(num_points);

      // tuning parameters
      const double N0 = 100.0;   // midpoint (50% confidence)
      const double k = 0.05;     // slope

      double existence = 1.0 / (1.0 + std::exp(-k * (N - N0)));

      measurement.existenceProbability() = std::min(1.0, existence);

      unassociated_measurements.insert({measurement.uuid(), measurement});
    }
  }

  void setDimensionCovarianceMultiplier(
    const ufil::type::Scalar dimension_covariance_multiplier)
  {
    dimension_covariance_multiplier_ = dimension_covariance_multiplier;
  }

  void setPositionCovarianceMultiplier(
    const ufil::type::Scalar position_covariance_multiplier)
  {
    position_covariance_multiplier_ = position_covariance_multiplier;
  }

  void setOrientationCovarianceMultiplier(
    const ufil::type::Scalar orientation_covariance_multiplier)
  {
    orientation_covariance_multiplier_ = orientation_covariance_multiplier;
  }

  void setClusterPointRange(
    const ufil::type::Scalar min_points,
    const ufil::type::Scalar max_points)
  {
    if (min_points > max_points) {
      throw std::invalid_argument("Minimum value must be smaller or equal to maximum value.");
    }
    clustering_->setPointRange(min_points, max_points);
  }

  void setClusterMinDimension(
    const ufil::type::Scalar min_dim_x, const ufil::type::Scalar min_dim_y,
    const ufil::type::Scalar min_dim_z)
  {
    if (min_dim_x < 0 || min_dim_y < 0 || min_dim_z < 0) {
      throw std::invalid_argument("Minimum value must be greater or equal to zero.");
    }
    shape_fitter_->setMinDimensions(min_dim_x, min_dim_y, min_dim_z);
  }

  void setClusterTolerance(const ufil::type::Scalar tolerance)
  {
    if (tolerance <= 0) {
      throw std::invalid_argument("Cluster tolerance must be greater then zero.");
    }
    clustering_->setTolerance(tolerance);
  }

  void setClusterAssignmentMinNumberOfPoints(const int cluster_assignment_min_number_of_points)
  {
    if (cluster_assignment_min_number_of_points <= 0) {
      throw std::invalid_argument("Cluster assignment minimum must be greater then zero.");
    }
    this->cluster_assignment_min_number_of_points_ = cluster_assignment_min_number_of_points;
  }

  std::unique_ptr<Clustering> clustering_;
  std::unique_ptr<ShapeFitter> shape_fitter_;

  ufil::type::Scalar dimension_covariance_multiplier_ = 10.0f;
  ufil::type::Scalar position_covariance_multiplier_ = 10.0f;
  ufil::type::Scalar orientation_covariance_multiplier_ = 1.0f;

  int cluster_assignment_min_number_of_points_ = 1;
};

}  // namespace detection
}  // namespace ufil

#endif  // UFIL_OBJECT_TRACKING__DETECTION__POINT_CLOUD_DETECTOR_HPP_
