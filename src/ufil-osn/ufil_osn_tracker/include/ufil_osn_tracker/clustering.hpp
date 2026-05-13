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

#ifndef UFIL_OSN_TRACKER__CLUSTERING_HPP_
#define UFIL_OSN_TRACKER__CLUSTERING_HPP_

#include <pcl/point_types.h>
#include <vector>

#include "ufil_osn_tracker/visibility_control.h"
#include "ufil_osn_tracker/detector_types.hpp"

namespace ufil_osn_tracker
{

class Clustering
{
public:
  explicit Clustering(
    ufil::type::Scalar tolerance = 0.5,
    ufil::type::Scalar min_points = 50,
    ufil::type::Scalar max_points = 50000);

  void setTolerance(ufil::type::Scalar tolerance);
  void setPointRange(ufil::type::Scalar min_points, ufil::type::Scalar max_points);

  std::vector<Cluster> cluster(const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud);

private:
  ufil::type::Scalar cluster_tolerance_;
  ufil::type::Scalar cluster_min_points_;
  ufil::type::Scalar cluster_max_points_;
};

}  // namespace ufil_osn_tracker

#endif  // UFIL_OSN_TRACKER__CLUSTERING_HPP_
