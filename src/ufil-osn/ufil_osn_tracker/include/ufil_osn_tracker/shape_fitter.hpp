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

#ifndef UFIL_OSN_TRACKER__SHAPE_FITTER_HPP_
#define UFIL_OSN_TRACKER__SHAPE_FITTER_HPP_

#include <pcl/point_types.h>

#include "ufil_osn_tracker/visibility_control.h"
#include "ufil_osn_tracker/detector_types.hpp"  // for BoundingBox struct

namespace ufil_osn_tracker
{

class ShapeFitter
{
public:
  explicit ShapeFitter(
    float min_dimension_x = 0.7f,
    float min_dimension_y = 0.7f,
    float min_dimension_z = 0.7f);

  void setMinDimensions(float min_x, float min_y, float min_z);

  float getMinDimensionX() const {return cluster_min_dimension_x_;}
  float getMinDimensionY() const {return cluster_min_dimension_y_;}

  void fitLShape(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr & cluster_cloud, BoundingBox & output_object,
    const float min_angle = -M_PI * 0.5f, const float max_angle = 0.0f);
  void fitPca(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr & cluster_cloud,
    BoundingBox & output_object);
  void fitCylinder(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr & cluster_cloud,
    BoundingBox & output_object);

private:
  float cluster_min_dimension_x_;
  float cluster_min_dimension_y_;
  float cluster_min_dimension_z_;
};

}  // namespace ufil_osn_tracker

#endif  // UFIL_OSN_TRACKER__SHAPE_FITTER_HPP_
