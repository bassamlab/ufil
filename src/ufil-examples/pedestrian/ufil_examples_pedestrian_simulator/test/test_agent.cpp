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

#include <numbers>

#include <cmath>
#include <stdexcept>
#include <vector>

#include "ufil_examples_pedestrian_simulator/agent.hpp"

using ufil_examples_pedestrian_simulator::Agent;
using ufil_examples_pedestrian_simulator::Occlusion;
using ufil_examples_pedestrian_simulator::Waypoint;

namespace
{

const Eigen::Vector3f kDimension(1.0F, 0.5F, 1.8F);

Waypoint waypoint(float x, float y, double time)
{
  return Waypoint{Eigen::Vector2f(x, y), time};
}

}  // namespace

TEST(AgentTest, InterpolatesBetweenWaypoints)
{
  const Agent agent(7, kDimension, {waypoint(0, 0, 1.0), waypoint(10, 0, 11.0)});
  EXPECT_EQ(agent.id(), 7);
  EXPECT_FALSE(agent.state(0.5));
  EXPECT_FALSE(agent.state(11.5));

  const auto state = agent.state(6.0);
  ASSERT_TRUE(state);
  EXPECT_FLOAT_EQ(state->position.x(), 5.0F);
  EXPECT_FLOAT_EQ(state->position.y(), 0.0F);
  EXPECT_EQ(state->dimension, kDimension);
  EXPECT_DOUBLE_EQ(state->orientation, 0.0);

  // The end time belongs to the trajectory.
  ASSERT_TRUE(agent.state(11.0));
  EXPECT_FLOAT_EQ(agent.state(11.0)->position.x(), 10.0F);
}

TEST(AgentTest, FacesWalkingDirectionAndKeepsHeadingWhileStanding)
{
  const Agent agent(
    1, kDimension,
    {waypoint(0, 0, 0.0), waypoint(0, 0, 2.0), waypoint(0, 10, 12.0), waypoint(0, 10, 15.0),
      waypoint(-10, 10, 25.0)});
  const double north = std::numbers::pi / 2.0;
  // Standing before the first movement takes the heading of the next movement.
  EXPECT_NEAR(agent.state(1.0)->orientation, north, 1e-6);
  EXPECT_NEAR(agent.state(5.0)->orientation, north, 1e-6);
  // Standing after a movement keeps its heading.
  EXPECT_NEAR(agent.state(13.0)->orientation, north, 1e-6);
  EXPECT_NEAR(std::abs(agent.state(20.0)->orientation), std::numbers::pi, 1e-6);
}

TEST(AgentTest, UsesFixedOrientation)
{
  const Agent agent(1, kDimension, {waypoint(0, 0, 0.0), waypoint(10, 0, 1.0)}, 0.25);
  EXPECT_DOUBLE_EQ(agent.state(0.5)->orientation, 0.25);
}

TEST(AgentTest, IsInvisibleDuringOcclusions)
{
  const Agent agent(
    1, kDimension, {waypoint(0, 0, 0.0), waypoint(10, 0, 10.0)}, std::nullopt,
    {Occlusion{2.0, 4.0}});
  EXPECT_TRUE(agent.visible(1.9));
  EXPECT_FALSE(agent.visible(2.0));
  EXPECT_FALSE(agent.visible(3.9));
  EXPECT_TRUE(agent.visible(4.0));
}

TEST(AgentTest, RejectsInvalidTrajectories)
{
  EXPECT_THROW(Agent(1, kDimension, {waypoint(0, 0, 0.0)}), std::invalid_argument);
  EXPECT_THROW(
    Agent(1, kDimension, {waypoint(0, 0, 1.0), waypoint(1, 0, 1.0)}), std::invalid_argument);
  EXPECT_THROW(
    Agent(
      1, kDimension, {waypoint(0, 0, 0.0), waypoint(1, 0, 1.0)}, std::nullopt,
      {Occlusion{0.5, 0.5}}),
    std::invalid_argument);
}
