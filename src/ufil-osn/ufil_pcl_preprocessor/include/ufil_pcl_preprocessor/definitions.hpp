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


#ifndef UFIL_PCL_PREPROCESSOR__DEFINITIONS_HPP_
#define UFIL_PCL_PREPROCESSOR__DEFINITIONS_HPP_

#include <vector>
#include <limits>

namespace ufil_pcl_preprocessor
{

struct ClippedArea
{
  float x = 0.0f;
  float y = 0.0f;
  float z = 0.0f;
  float yaw = 0.0f;
  float length = 0.0f;
  float width = 0.0f;
  float height = 0.0f;
};

struct Cell
{
  double range = std::numeric_limits<double>::infinity();
  Eigen::Vector3d p_hit = Eigen::Vector3d::Zero();
};

}  // namespace ufil_pcl_preprocessor

#endif  // UFIL_PCL_PREPROCESSOR__DEFINITIONS_HPP_
