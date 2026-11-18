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
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <memory>
#include <set>
#include <stdexcept>

#include "ufil_osn_tracker/ufil_osn_tracker.hpp"

using ufil_osn_tracker::Detection;
using ufil_osn_tracker::OsnObjectTracker;
using ufil_osn_tracker::TrackerConfig;
using ufil_osn_tracker::UpdateResult;

namespace
{

/// Point cloud of a 4 m x 2 m x 1.5 m box sampled every 0.1 m, centred at (x, y).
std::set<Detection> boxAt(float x, float y)
{
  pcl::PointCloud<pcl::PointXYZ> cloud;
  for (float dx = -2.0f; dx <= 2.0f; dx += 0.1f) {
    for (float dy = -1.0f; dy <= 1.0f; dy += 0.1f) {
      for (float z = 0.0f; z <= 1.5f; z += 0.5f) {
        cloud.push_back(pcl::PointXYZ(x + dx, y + dy, z));
      }
    }
  }
  pcl::PCLPointCloud2 msg;
  pcl::toPCLPointCloud2(cloud, msg);
  std::set<Detection> detections;
  detections.insert(Detection(msg));
  return detections;
}

ufil::type::Timestamp at(double seconds)
{
  return ufil::from_seconds<ufil::type::Timestamp>(1000.0 + seconds);
}

}  // namespace

TEST(OsnObjectTracker, InvalidConfigurationIsRejected)
{
  TrackerConfig config;
  config.cluster_min_points = 0;
  EXPECT_THROW(OsnObjectTracker{config}, std::invalid_argument);

  OsnObjectTracker tracker;
  EXPECT_THROW(tracker.configure(config), std::invalid_argument);
  EXPECT_EQ(tracker.config().cluster_min_points, TrackerConfig{}.cluster_min_points);
}

TEST(OsnObjectTracker, TracksMovingObject)
{
  OsnObjectTracker tracker;
  for (int i = 0; i < 30; ++i) {
    const double t = 0.1 * i;
    ASSERT_EQ(tracker.update(boxAt(static_cast<float>(t), 0.0f), at(t)), UpdateResult::kUpdated);
  }

  const auto status = tracker.status();
  EXPECT_EQ(status.processed_updates, 30u);
  EXPECT_EQ(status.active_tracks, 1u);
  ASSERT_EQ(tracker.tracks().size(), 1u);
  EXPECT_GT(status.total_detector_associations + status.total_associator_associations, 20u);

  const auto & track = tracker.tracks().begin()->second;
  EXPECT_NEAR(track.currentState().x(), 2.9, 0.5);
  EXPECT_NEAR(track.currentState().y(), 0.0, 0.5);
}

TEST(OsnObjectTracker, ResetsAfterTimeJump)
{
  TrackerConfig config;
  config.time_jump_threshold = 5.0;
  OsnObjectTracker tracker(config);
  tracker.update(boxAt(0.0f, 0.0f), at(0.0));
  tracker.update(boxAt(0.0f, 0.0f), at(0.1));
  ASSERT_FALSE(tracker.tracks().empty());
  const auto old_id = tracker.tracks().begin()->first;

  EXPECT_EQ(tracker.update(boxAt(0.0f, 0.0f), at(10.0)), UpdateResult::kResetAfterTimeJump);
  EXPECT_EQ(tracker.status().time_jump_resets, 1u);
  EXPECT_NEAR(tracker.status().last_time_delta, 9.9, 1e-6);
  for (const auto & [id, track] : tracker.tracks()) {
    EXPECT_NE(id, old_id);
  }
}

TEST(OsnObjectTracker, RejectsOutOfOrderUpdates)
{
  OsnObjectTracker tracker;
  tracker.update(boxAt(0.0f, 0.0f), at(1.0));
  EXPECT_EQ(tracker.update(boxAt(0.0f, 0.0f), at(0.5)), UpdateResult::kRejectedOutOfOrder);
  const auto status = tracker.status();
  EXPECT_EQ(status.rejected_out_of_order, 1u);
  EXPECT_EQ(status.processed_updates, 1u);
  EXPECT_LT(status.last_time_delta, 0.0);
}

TEST(OsnObjectTracker, ResetKeepsOcclusionGridAndFov)
{
  OsnObjectTracker tracker;
  EXPECT_FALSE(tracker.status().occlusion_grid_available);
  EXPECT_FALSE(tracker.status().fov_available);

  tracker.setOcclusionGrid(std::make_shared<const ufil::type::OccupancyGrid>());
  tracker.setCurrentFOV(ufil::type::SensorFOV());
  tracker.update(boxAt(0.0f, 0.0f), at(0.0));
  tracker.reset();

  const auto status = tracker.status();
  EXPECT_TRUE(status.occlusion_grid_available);
  EXPECT_TRUE(status.fov_available);
  EXPECT_TRUE(tracker.tracks().empty());
}

TEST(OsnObjectTracker, SmallClustersAreIgnored)
{
  TrackerConfig config;
  config.cluster_min_points = 100000;
  config.cluster_max_points = 200000;
  OsnObjectTracker tracker(config);
  tracker.update(boxAt(0.0f, 0.0f), at(0.0));
  EXPECT_TRUE(tracker.tracks().empty());
  EXPECT_EQ(tracker.status().unassociated_measurements, 0u);
}
