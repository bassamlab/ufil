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

#include <gtest/gtest.h>

#include <pcl/conversions.h>

#include <vector>

#include "ufil_pcl_preprocessor_ros.hpp"

namespace conversions = ufil_pcl_preprocessor::conversions;

TEST(RosConversions, CloudRoundTrip)
{
  pcl::PointCloud<pcl::PointXYZ> cloud;
  cloud.push_back(pcl::PointXYZ(1.0f, 2.0f, 3.0f));
  pcl::PCLPointCloud2 pcl_cloud;
  pcl::toPCLPointCloud2(cloud, pcl_cloud);

  std_msgs::msg::Header header;
  header.frame_id = "target";
  header.stamp.sec = 42;
  const auto msg = conversions::cloudToMsg(pcl_cloud, header);
  EXPECT_EQ(msg.header.frame_id, "target");
  EXPECT_EQ(msg.header.stamp.sec, 42);
  EXPECT_EQ(msg.width * msg.height, 1u);

  pcl::PointCloud<pcl::PointXYZ> back;
  pcl::fromPCLPointCloud2(conversions::cloudFromMsg(msg), back);
  ASSERT_EQ(back.size(), 1u);
  EXPECT_FLOAT_EQ(back.front().z, 3.0f);
}

TEST(RosConversions, TransformFromMsg)
{
  geometry_msgs::msg::TransformStamped msg;
  msg.transform.translation.x = 1.0;
  msg.transform.rotation.z = 1.0;  // 180 degrees about z
  msg.transform.rotation.w = 0.0;
  const Eigen::Isometry3d transform = conversions::transformFromMsg(msg);
  const Eigen::Vector3d point = transform * Eigen::Vector3d(1.0, 0.0, 0.0);
  EXPECT_NEAR(point.x(), 0.0, 1e-9);
  EXPECT_NEAR(point.y(), 0.0, 1e-9);
}

TEST(RosConversions, OcclusionGridKeepsHeaderAndGeometry)
{
  ufil::type::OccupancyGrid grid;
  grid.width() = 2;
  grid.height() = 3;
  grid.resolution() = 0.5;
  grid.data().assign(6, 50);
  std_msgs::msg::Header header;
  header.frame_id = "target";
  const auto msg = conversions::occlusionGridToMsg(grid, header);
  EXPECT_EQ(msg.header.frame_id, "target");
  EXPECT_EQ(msg.info.width, 2u);
  EXPECT_EQ(msg.info.height, 3u);
  EXPECT_EQ(msg.data.size(), 6u);
}

TEST(RosConversions, MarkersForClippedAreasAndFov)
{
  std_msgs::msg::Header header;
  header.frame_id = "target";
  const auto clipped = conversions::clippedAreasToMarkers(
    {ufil_pcl_preprocessor::ClippedArea{1.0f, 2.0f, 3.0f, 0.0f, 4.0f, 5.0f, 6.0f}}, header);
  ASSERT_EQ(clipped.markers.size(), 1u);
  EXPECT_EQ(clipped.markers[0].type, visualization_msgs::msg::Marker::CUBE);
  EXPECT_DOUBLE_EQ(clipped.markers[0].scale.y, 5.0);
  EXPECT_DOUBLE_EQ(clipped.markers[0].pose.orientation.w, 1.0);

  ufil_msgs::msg::SensorFOV fov;
  fov.header = header;
  fov.r_max = 10.0;
  fov.phi_min = -0.5;
  fov.phi_max = 0.5;
  fov.theta_min = -0.2;
  fov.theta_max = 0.2;
  const auto fov_markers = conversions::fovToMarkers(fov);
  ASSERT_EQ(fov_markers.markers.size(), 2u);
  EXPECT_NE(fov_markers.markers[0].id, fov_markers.markers[1].id);
  EXPECT_GT(fov_markers.markers[0].points.size(), 10u);
}
