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

#include <pcl/common/common.h>
#include <pcl/common/pca.h>

#include <algorithm>
#include <stdexcept>
#include <utility>

#include "ufil_osn_tracker/shape_fitter.hpp"

namespace ufil_osn_tracker
{

inline float calcClosenessCriterion(const std::vector<float> & C_1, const std::vector<float> & C_2)
{
  // Paper : Algo.4 Closeness Criterion
  const float min_c_1 = *std::min_element(C_1.begin(), C_1.end());  // col.2, Algo.4
  const float max_c_1 = *std::max_element(C_1.begin(), C_1.end());  // col.2, Algo.4
  const float min_c_2 = *std::min_element(C_2.begin(), C_2.end());  // col.3, Algo.4
  const float max_c_2 = *std::max_element(C_2.begin(), C_2.end());  // col.3, Algo.4

  // return 1.0 / ((max_c_2 - min_c_2) * (max_c_1 - min_c_1));
  std::vector<float> D_1;  // col.4, Algo.4
  for (const auto & c_1_element : C_1) {
    const float v = std::min(max_c_1 - c_1_element, c_1_element - min_c_1);
    D_1.push_back(v * v);
  }

  std::vector<float> D_2;  // col.5, Algo.4
  for (const auto & c_2_element : C_2) {
    const float v = std::min(max_c_2 - c_2_element, c_2_element - min_c_2);
    D_2.push_back(v * v);
  }
  constexpr float d_min = 0.1 * 0.1;
  constexpr float d_max = 0.4 * 0.4;
  float beta = 0;  // col.6, Algo.4
  for (size_t i = 0; i < D_1.size(); ++i) {
    if (d_max < std::min(D_1.at(i), D_2.at(i))) {
      continue;
    }
    const float d = std::max(std::min(D_1.at(i), D_2.at(i)), d_min);
    beta += 1.0 / d;
  }
  return beta;
}

inline float optimize(
  const pcl::PointCloud<pcl::PointXYZ>::Ptr & cluster, const float min_angle,
  const float max_angle)
{
  constexpr float epsilon = 0.001;
  std::vector<std::pair<float /*theta*/, float /*q*/>> Q;
  constexpr float angle_resolution = M_PI / 180.0;
  for (float theta = min_angle; theta <= max_angle + epsilon; theta += angle_resolution) {
    Eigen::Vector2f e_1;
    e_1 << std::cos(theta), std::sin(theta);  // col.3, Algo.2
    Eigen::Vector2f e_2;
    e_2 << -std::sin(theta), std::cos(theta);  // col.4, Algo.2
    std::vector<float> C_1;                    // col.5, Algo.2
    std::vector<float> C_2;                    // col.6, Algo.2
    for (const auto & point : *cluster) {
      C_1.push_back(point.x * e_1.x() + point.y * e_1.y());
      C_2.push_back(point.x * e_2.x() + point.y * e_2.y());
    }
    float q = calcClosenessCriterion(C_1, C_2);  // col.7, Algo.2
    Q.push_back({theta, q});                     // col.8, Algo.2
  }

  float theta_star{0.0};    // col.10, Algo.2
  float max_q = 0.0;
  for (size_t i = 0; i < Q.size(); ++i) {
    if (max_q < Q.at(i).second || i == 0) {
      max_q = Q.at(i).second;
      theta_star = Q.at(i).first;
    }
  }

  return theta_star;
}

ShapeFitter::ShapeFitter(
  float min_dimension_x,
  float min_dimension_y,
  float min_dimension_z)
: cluster_min_dimension_x_(min_dimension_x),
  cluster_min_dimension_y_(min_dimension_y),
  cluster_min_dimension_z_(min_dimension_z)
{
}

void ShapeFitter::setMinDimensions(float min_x, float min_y, float min_z)
{
  cluster_min_dimension_x_ = min_x;
  cluster_min_dimension_y_ = min_y;
  cluster_min_dimension_z_ = min_z;
}

void ShapeFitter::fitLShape(
  const pcl::PointCloud<pcl::PointXYZ>::Ptr & cluster, BoundingBox & output_object,
  const float min_angle, const float max_angle)
{
  // calc min and max z for height
  float min_z = cluster->empty() ? 0.0 : cluster->at(0).z;
  float max_z = cluster->empty() ? 0.0 : cluster->at(0).z;
  for (const auto & point : *cluster) {
    min_z = std::min(point.z, min_z);
    max_z = std::max(point.z, max_z);
  }

  // Paper : Algo.2 Search-Based Rectangle Fitting
  double theta_star = optimize(cluster, min_angle, max_angle);

  const float sin_theta_star = std::sin(theta_star);
  const float cos_theta_star = std::cos(theta_star);

  Eigen::Vector2f e_1_star;  // col.11, Algo.2
  Eigen::Vector2f e_2_star;
  e_1_star << cos_theta_star, sin_theta_star;
  e_2_star << -sin_theta_star, cos_theta_star;
  std::vector<float> C_1_star;  // col.11, Algo.2
  std::vector<float> C_2_star;  // col.11, Algo.2
  for (const auto & point : *cluster) {
    C_1_star.push_back(point.x * e_1_star.x() + point.y * e_1_star.y());
    C_2_star.push_back(point.x * e_2_star.x() + point.y * e_2_star.y());
  }

  // col.12, Algo.2
  const float min_C_1_star = *std::min_element(C_1_star.begin(), C_1_star.end());
  const float max_C_1_star = *std::max_element(C_1_star.begin(), C_1_star.end());
  const float min_C_2_star = *std::min_element(C_2_star.begin(), C_2_star.end());
  const float max_C_2_star = *std::max_element(C_2_star.begin(), C_2_star.end());

  const float a_1 = cos_theta_star;
  const float b_1 = sin_theta_star;
  const float c_1 = min_C_1_star;
  const float a_2 = -1.0 * sin_theta_star;
  const float b_2 = cos_theta_star;
  const float c_2 = min_C_2_star;
  const float a_3 = cos_theta_star;
  const float b_3 = sin_theta_star;
  const float c_3 = max_C_1_star;
  const float a_4 = -1.0 * sin_theta_star;
  const float b_4 = cos_theta_star;
  const float c_4 = max_C_2_star;

  // calc center of bounding box
  float intersection_x_1 = (b_1 * c_2 - b_2 * c_1) / (a_2 * b_1 - a_1 * b_2);
  float intersection_y_1 = (a_1 * c_2 - a_2 * c_1) / (a_1 * b_2 - a_2 * b_1);
  float intersection_x_2 = (b_3 * c_4 - b_4 * c_3) / (a_4 * b_3 - a_3 * b_4);
  float intersection_y_2 = (a_3 * c_4 - a_4 * c_3) / (a_3 * b_4 - a_4 * b_3);

  // calc dimension of bounding box
  Eigen::Vector2f e_x;
  Eigen::Vector2f e_y;
  e_x << a_1 / (std::sqrt(a_1 * a_1 + b_1 * b_1)), b_1 / (std::sqrt(a_1 * a_1 + b_1 * b_1));
  e_y << a_2 / (std::sqrt(a_2 * a_2 + b_2 * b_2)), b_2 / (std::sqrt(a_2 * a_2 + b_2 * b_2));
  Eigen::Vector2f diagonal_vec;
  diagonal_vec << intersection_x_1 - intersection_x_2, intersection_y_1 - intersection_y_2;

  // output
  output_object.dimension_x = std::fabs(e_x.dot(diagonal_vec));
  output_object.dimension_y = std::fabs(e_y.dot(diagonal_vec));
  output_object.dimension_z = static_cast<ufil::type::Scalar>(std::max(max_z,
      cluster_min_dimension_z_));
  output_object.position_x = (intersection_x_1 + intersection_x_2) * 0.5;
  output_object.position_y = (intersection_y_1 + intersection_y_2) * 0.5;
  output_object.position_z = output_object.dimension_z * 0.5;
  output_object.yaw = std::atan2(e_1_star.y(), e_1_star.x());
  output_object.oriented = true;

  // check wrong output
  output_object.dimension_x =
    static_cast<ufil::type::Scalar>(std::max(static_cast<float>(output_object.dimension_x),
      cluster_min_dimension_x_));
  output_object.dimension_y =
    static_cast<ufil::type::Scalar>(std::max(static_cast<float>(output_object.dimension_y),
      cluster_min_dimension_y_));
}

void ShapeFitter::fitPca(
  const pcl::PointCloud<pcl::PointXYZ>::Ptr & cluster,
  BoundingBox & output_object)
{
  pcl::PCA<pcl::PointXYZ> principal_component_analysis(new pcl::PCA<pcl::PointXYZ>);
  principal_component_analysis.setInputCloud(cluster);

  // Position is the mean value or origin of the pca
  Eigen::Vector4f position = principal_component_analysis.getMean();
  output_object.position_x = position.x();
  output_object.position_y = position.y();

  // Yaw is the orientation of the first principle component
  Eigen::Matrix3f princiable_components = principal_component_analysis.getEigenVectors();
  Eigen::Vector3f first_principle_component = princiable_components.col(0);
  output_object.yaw = std::atan2(first_principle_component.y(), first_principle_component.x());
  output_object.oriented = true;

  // Dimension is the size or the rotated pointcloud
  pcl::PointCloud<pcl::PointXYZ>::Ptr oriented_cluster(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointXYZ oriented_min_point, oriented_max_point;
  principal_component_analysis.project(*cluster, *oriented_cluster);
  pcl::getMinMax3D<pcl::PointXYZ>(*oriented_cluster, oriented_min_point, oriented_max_point);
  output_object.dimension_x = static_cast<ufil::type::Scalar>(std::max(oriented_max_point.x -
      oriented_min_point.x,
      cluster_min_dimension_x_));
  output_object.dimension_y = static_cast<ufil::type::Scalar>(std::max(oriented_max_point.y -
      oriented_min_point.y,
      cluster_min_dimension_y_));
}

void ShapeFitter::fitCylinder(
  const pcl::PointCloud<pcl::PointXYZ>::Ptr & cluster_cloud,
  BoundingBox & output_object)
{
  // Position is the center of the original pointcoud
  // Dimension is the size of the original pointcloud
  pcl::PointXYZ cloud_min_point, cloud_max_point;
  pcl::getMinMax3D<pcl::PointXYZ>(*cluster_cloud, cloud_min_point, cloud_max_point);
  output_object.dimension_x = static_cast<ufil::type::Scalar>(std::max(cloud_max_point.x -
      cloud_min_point.x,
      cluster_min_dimension_x_));
  output_object.dimension_y = static_cast<ufil::type::Scalar>(std::max(cloud_max_point.y -
      cloud_min_point.y,
      cluster_min_dimension_y_));
  output_object.position_x = cloud_min_point.x + output_object.dimension_x * 0.5;
  output_object.position_y = cloud_min_point.y + output_object.dimension_y * 0.5;
  output_object.yaw = 0.0f;  // Axis algined so no orientation
  output_object.oriented = false;
}

}  // namespace ufil_osn_tracker
