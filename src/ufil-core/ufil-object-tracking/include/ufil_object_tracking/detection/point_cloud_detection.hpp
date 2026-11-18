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

#ifndef UFIL_OBJECT_TRACKING__DETECTION__POINT_CLOUD_DETECTION_HPP_
#define UFIL_OBJECT_TRACKING__DETECTION__POINT_CLOUD_DETECTION_HPP_

#include <pcl/PCLPointCloud2.h>

#include <utility>

#include "ufil_object_tracking/types/detection.hpp"

namespace ufil
{
namespace detection
{

// A generic, ROS-unaware detection that owns a PCL point cloud. The conversion
// from any sensor representation (e.g. a ROS sensor_msgs/PointCloud2) into a
// pcl::PCLPointCloud2 is expected to happen in the calling layer.
class PointCloudDetection : public ufil::type::Detection
{
public:
  explicit PointCloudDetection(pcl::PCLPointCloud2 cloud)
  : ufil::type::Detection(),
    cloud_(std::move(cloud))
  {
  }

  const pcl::PCLPointCloud2 & cloud() const
  {
    return this->cloud_;
  }

private:
  pcl::PCLPointCloud2 cloud_;
};

}  // namespace detection
}  // namespace ufil

#endif  // UFIL_OBJECT_TRACKING__DETECTION__POINT_CLOUD_DETECTION_HPP_
