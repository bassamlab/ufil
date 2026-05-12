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


/*
 * Patched 3DBoxClipper from PCL 1.14 into an older version of PCL. PCL fixed
 * a critical bug in this version that made the 3DBoxClipper unusable
 * in previous versions.
 */

/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2010-2011, Willow Garage, Inc.
 *
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the copyright holder(s) nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 */

#ifndef BOX_CLIPPER_3D_HPP_
#define BOX_CLIPPER_3D_HPP_

#include <pcl/memory.h>
#include <pcl/pcl_macros.h>
#include <pcl/filters/clipper3D.h>

#include <vector>
#include <memory>

namespace pcl
{
/**
 * \author Suat Gedikli <gedikli@willowgarage.com>
 * \brief Implementation of a box clipper in 3D. Actually it allows affine transformations, thus any parallelepiped in
 * general pose. The affine transformation is used to transform the point before clipping it using a cube centered at
 * origin and with an extend of -1 to +1 in each dimension \sa CropBox \ingroup filters
 */
template<typename PointT>
class BoxClipper3D : public pcl::Clipper3D<PointT>
{
public:
  using Ptr = std::shared_ptr<BoxClipper3D<PointT>>;
  using ConstPtr = std::shared_ptr<const BoxClipper3D<PointT>>;

  /**
   * \author Suat Gedikli <gedikli@willowgarage.com>
   * \brief Constructor taking an affine transformation matrix, which allows also shearing of the clipping area
   * \param[in] transformation the 3 dimensional affine transformation that is used to describe the cube ([-1; +1] in
   * each dimension). The transformation is applied to the point(s)!
   */
  explicit BoxClipper3D(const Eigen::Affine3f & transformation);

  /**
   * \brief creates a BoxClipper object with a scaled box in general pose
   * \param[in] rodrigues the rotation axis and angle given by the vector direction and length respectively
   * \param[in] translation the position of the box center
   * \param[in] box_size the size of the box for each dimension
   */
  BoxClipper3D(
    const Eigen::Vector3f & rodrigues, const Eigen::Vector3f & translation,
    const Eigen::Vector3f & box_size);

  /**
   * \brief Set the affine transformation
   * \param[in] transformation applied to the point(s)
   */
  void setTransformation(const Eigen::Affine3f & transformation);

  /**
   * \brief sets the box in general pose given by the orientation position and size
   * \param[in] rodrigues the rotation axis and angle given by the vector direction and length respectively
   * \param[in] translation the position of the box center
   * \param[in] box_size the size of the box for each dimension
   */
  void setTransformation(
    const Eigen::Vector3f & rodrigues, const Eigen::Vector3f & translation,
    const Eigen::Vector3f & box_size);

  /**
   * \brief virtual destructor
   */
  ~BoxClipper3D() noexcept override;

  bool clipPoint3D(const PointT & point) const override;

  bool clipLineSegment3D(PointT & from, PointT & to) const override;

  void clipPlanarPolygon3D(
    std::vector<PointT,
    Eigen::aligned_allocator<PointT>> & polygon) const override;

  void clipPlanarPolygon3D(
    const std::vector<PointT, Eigen::aligned_allocator<PointT>> & polygon,
    std::vector<PointT, Eigen::aligned_allocator<PointT>> & clipped_polygon) const override;

  void clipPointCloud3D(
    const pcl::PointCloud<PointT> & cloud_in, pcl::Indices & clipped,
    const pcl::Indices & indices = pcl::Indices()) const override;

  pcl::Clipper3D<PointT> * clone() const override;

protected:
  float getDistance(const PointT & point) const;
  void transformPoint(const PointT & pointIn, PointT & pointOut) const;

private:
  /**
   * \brief the affine transformation that is applied before clipping is done on the [-1; +1] cube.
   */
  Eigen::Affine3f transformation_;

public:
  PCL_MAKE_ALIGNED_OPERATOR_NEW
};
}  // namespace pcl

template<typename PointT>
pcl::BoxClipper3D<PointT>::BoxClipper3D(const Eigen::Affine3f & transformation)
: transformation_(transformation)
{
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template<typename PointT>
pcl::BoxClipper3D<PointT>::BoxClipper3D(
  const Eigen::Vector3f & rodrigues, const Eigen::Vector3f & translation,
  const Eigen::Vector3f & box_size)
{
  setTransformation(rodrigues, translation, box_size);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template<typename PointT>
pcl::BoxClipper3D<PointT>::~BoxClipper3D() noexcept = default;

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template<typename PointT>
void pcl::BoxClipper3D<PointT>::setTransformation(const Eigen::Affine3f & transformation)
{
  transformation_ = transformation;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template<typename PointT>
void pcl::BoxClipper3D<PointT>::setTransformation(
  const Eigen::Vector3f & rodrigues, const Eigen::Vector3f & translation,
  const Eigen::Vector3f & box_size)
{
  transformation_ = (Eigen::Translation3f(translation) * Eigen::AngleAxisf(rodrigues.norm(),
    rodrigues.normalized()) *
    Eigen::Scaling(0.5f * box_size))
    .inverse();
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template<typename PointT>
pcl::Clipper3D<PointT> * pcl::BoxClipper3D<PointT>::clone() const
{
  return new BoxClipper3D<PointT>(transformation_);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template<typename PointT>
void pcl::BoxClipper3D<PointT>::transformPoint(const PointT & pointIn, PointT & pointOut) const
{
  const Eigen::Vector4f & point = pointIn.getVector4fMap();
  pointOut.getVector4fMap() = transformation_ * point;

  // homogeneous value might not be 1
  if (point[3] != 1) {
    // homogeneous component might be uninitialized -> invalid
    if (point[3] != 0) {
      pointOut.x += (1 - point[3]) * transformation_.data()[9];
      pointOut.y += (1 - point[3]) * transformation_.data()[10];
      pointOut.z += (1 - point[3]) * transformation_.data()[11];
    } else {
      pointOut.x += transformation_.data()[9];
      pointOut.y += transformation_.data()[10];
      pointOut.z += transformation_.data()[11];
    }
  }
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template<typename PointT>
bool pcl::BoxClipper3D<PointT>::clipPoint3D(const PointT & point) const
{
  Eigen::Vector4f point_coordinates(transformation_.matrix() * point.getVector4fMap());
  return (point_coordinates.array().abs() <= 1).all();
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**
 * @attention untested code
 */
template<typename PointT>
bool pcl::BoxClipper3D<PointT>::clipLineSegment3D(PointT &, PointT &) const
{
  /*
  PointT pt1, pt2;
  transformPoint (point1, pt1);
  transformPoint (point2, pt2);

  //
  bool pt1InBox = (std::abs(pt1.x) <= 1.0 && std::abs (pt1.y) <= 1.0 && std::abs (pt1.z) <= 1.0);
  bool pt2InBox = (std::abs(pt2.x) <= 1.0 && std::abs (pt2.y) <= 1.0 && std::abs (pt2.z) <= 1.0);

  // one is outside the other one inside the box
  //if (pt1InBox ^ pt2InBox)
  if (pt1InBox && !pt2InBox)
  {
    PointT diff;
    PointT lambda;
    diff.getVector3fMap () = pt2.getVector3fMap () - pt1.getVector3fMap ();

    if (diff.x > 0)
      lambda.x = (1.0 - pt1.x) / diff.x;
    else
      lambda.x = (-1.0 - pt1.x) / diff.x;

    if (diff.y > 0)
      lambda.y = (1.0 - pt1.y) / diff.y;
    else
      lambda.y = (-1.0 - pt1.y) / diff.y;

    if (diff.z > 0)
      lambda.z = (1.0 - pt1.z) / diff.z;
    else
      lambda.z = (-1.0 - pt1.z) / diff.z;

    pt2 = pt1 + std::min(std::min(lambda.x, lambda.y), lambda.z) * diff;

    // inverse transformation
    inverseTransformPoint (pt2, point2);
    return true;
  }
  else if (!pt1InBox && pt2InBox)
  {
    return true;
  }
  */
  throw std::logic_error("Not implemented");
  return false;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**
 * @attention untested code
 */
template<typename PointT>
void pcl::BoxClipper3D<PointT>::clipPlanarPolygon3D(
  const std::vector<PointT, Eigen::aligned_allocator<PointT>> &,
  std::vector<PointT, Eigen::aligned_allocator<PointT>> & clipped_polygon) const
{
  // not implemented -> clip everything
  clipped_polygon.clear();
  throw std::logic_error("Not implemented");
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**
 * @attention untested code
 */
template<typename PointT>
void pcl::BoxClipper3D<PointT>::clipPlanarPolygon3D(
  std::vector<PointT, Eigen::aligned_allocator<PointT>> & polygon) const
{
  // not implemented -> clip everything
  polygon.clear();
  throw std::logic_error("Not implemented");
}

////////////////////////////////////////////////////////////////////////////
// /ToDo: write fast version using eigen map and single matrix vector multiplication,
// that uses advantages of eigens SSE operations.
template<typename PointT>
void pcl::BoxClipper3D<PointT>::clipPointCloud3D(
  const pcl::PointCloud<PointT> & cloud_in, pcl::Indices & clipped,
  const pcl::Indices & indices) const
{
  clipped.clear();
  if (indices.empty()) {
    clipped.reserve(cloud_in.size());
    for (std::size_t pIdx = 0; pIdx < cloud_in.size(); ++pIdx) {
      if (clipPoint3D(cloud_in[pIdx])) {
        clipped.push_back(pIdx);
      }
    }
  } else {
    for (const auto & index : indices) {
      if (clipPoint3D(cloud_in[index])) {
        clipped.push_back(index);
      }
    }
  }
}
#endif  // BOX_CLIPPER_3D_HPP_
