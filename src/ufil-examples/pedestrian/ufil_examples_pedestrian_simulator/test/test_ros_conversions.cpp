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

#include <ufil_msgs/msg/classification.hpp>

#include "simulator_ros.hpp"

using ufil_examples_pedestrian_simulator::SimulatedObject;
using ufil_examples_pedestrian_simulator::SimulationStep;
namespace conversions = ufil_examples_pedestrian_simulator::conversions;

namespace
{

SimulationStep step()
{
  SimulatedObject agent;
  agent.id = 4;
  agent.ground_truth.position << 1.0F, 2.0F;
  agent.ground_truth.dimension << 1.0F, 0.5F, 1.8F;
  agent.ground_truth.orientation = 0.5;
  agent.measured = agent.ground_truth;
  agent.measured.position << 1.1F, 2.2F;

  SimulatedObject missed = agent;
  missed.id = 5;
  missed.detected = false;

  SimulatedObject clutter;
  clutter.id = 1000000;
  clutter.has_ground_truth = false;
  clutter.measured.position << 9.0F, 9.0F;

  return SimulationStep{1.5, {agent, missed, clutter}};
}

}  // namespace

TEST(RosConversionsTest, MeasuredObjectsContainDetectionsAndClutter)
{
  std_msgs::msg::Header header;
  header.frame_id = "map";
  const auto msg = conversions::measuredObjectsToMsg(step(), header);
  EXPECT_EQ(msg.header.frame_id, "map");
  ASSERT_EQ(msg.objects.size(), 2u);
  EXPECT_EQ(msg.objects[0].id, 4u);
  EXPECT_EQ(msg.objects[1].id, 1000000u);

  const auto & object = msg.objects[0];
  EXPECT_FLOAT_EQ(static_cast<float>(object.state.state.x), 1.1F);
  EXPECT_FLOAT_EQ(static_cast<float>(object.state.state.y), 2.2F);
  EXPECT_DOUBLE_EQ(object.state.state.yaw, 0.5);
  EXPECT_FLOAT_EQ(static_cast<float>(object.dimension.dimension.height), 1.8F);
  EXPECT_DOUBLE_EQ(object.state.covariance[0], 1.0);
  EXPECT_DOUBLE_EQ(object.state.covariance[9], 1.0);
  EXPECT_DOUBLE_EQ(object.state.covariance[54], 0.1);
  // Velocity, acceleration and yaw rate are not measured.
  for (const auto index : {18, 27, 36, 45, 63}) {
    EXPECT_DOUBLE_EQ(object.state.covariance[index], -1.0) << index;
  }
  EXPECT_DOUBLE_EQ(object.dimension.covariance[4], 1.0);
  EXPECT_DOUBLE_EQ(
    object.classification.classification[ufil_msgs::msg::Classification::PEDESTRIAN], 1.0);
}

TEST(RosConversionsTest, ReferenceObjectsContainGroundTruthOfAllAgents)
{
  const auto msg = conversions::referenceObjectsToMsg(step(), std_msgs::msg::Header());
  ASSERT_EQ(msg.objects.size(), 2u);
  EXPECT_EQ(msg.objects[0].id, 4u);
  EXPECT_EQ(msg.objects[1].id, 5u);
  EXPECT_FLOAT_EQ(static_cast<float>(msg.objects[0].state.state.x), 1.0F);
  EXPECT_FLOAT_EQ(static_cast<float>(msg.objects[0].state.state.y), 2.0F);
  EXPECT_DOUBLE_EQ(msg.objects[0].state.covariance[0], 0.0);
}
