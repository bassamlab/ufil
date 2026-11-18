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

#ifndef UFIL_OBJECT_TRACKING__DETECTION__CLUSTERING_HPP_
#define UFIL_OBJECT_TRACKING__DETECTION__CLUSTERING_HPP_

#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>

#include <vector>

#include "ufil_object_tracking/detection/detector_types.hpp"

namespace ufil
{
namespace detection
{

class Clustering
{
public:
  explicit Clustering(
    ufil::type::Scalar tolerance = 0.5,
    ufil::type::Scalar min_points = 50,
    ufil::type::Scalar max_points = 50000)
  : cluster_tolerance_(tolerance),
    cluster_min_points_(min_points),
    cluster_max_points_(max_points)
  {
  }

  void setTolerance(ufil::type::Scalar tolerance)
  {
    cluster_tolerance_ = tolerance;
  }

  void setPointRange(ufil::type::Scalar min_points, ufil::type::Scalar max_points)
  {
    cluster_min_points_ = min_points;
    cluster_max_points_ = max_points;
  }

  std::vector<Cluster> cluster(const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud)
  {
    std::vector<Cluster> clusters;

    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(cloud);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;

    ec.setClusterTolerance(cluster_tolerance_);
    ec.setMinClusterSize(cluster_min_points_);
    ec.setMaxClusterSize(cluster_max_points_);
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud);
    ec.extract(cluster_indices);

    int id = 0;
    for (auto it = cluster_indices.begin(); it != cluster_indices.end(); ++it) {
      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZ>);
      for (const auto & idx : it->indices) {
        cloud_cluster->push_back((*cloud)[idx]);
      }
      cloud_cluster->width = cloud_cluster->size();
      cloud_cluster->height = 1;
      cloud_cluster->is_dense = true;
      clusters.push_back({id++, cloud_cluster});
    }
    return clusters;
  }

private:
  ufil::type::Scalar cluster_tolerance_;
  ufil::type::Scalar cluster_min_points_;
  ufil::type::Scalar cluster_max_points_;
};

}  // namespace detection
}  // namespace ufil

#endif  // UFIL_OBJECT_TRACKING__DETECTION__CLUSTERING_HPP_
