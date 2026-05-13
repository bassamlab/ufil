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


#ifndef UFIL_OSN_TRACKER__DETECTION_HPP_
#define UFIL_OSN_TRACKER__DETECTION_HPP_

#include <pcl/common/common.h>

#include "ufil_osn_tracker/visibility_control.h"

#include <ufil_object_tracking/types/detection.hpp>

namespace ufil_osn_tracker
{

class Detection : public ufil::type::Detection
{
public:
  explicit Detection(pcl::PCLPointCloud2 * cloud);

  pcl::PCLPointCloud2 * cloud() const;

private:
  pcl::PCLPointCloud2 * cloud_;
};

}  // namespace ufil_osn_tracker

#endif  // UFIL_OSN_TRACKER__DETECTION_HPP_
