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

#include <gtest/gtest.h>

#include <ufil_object_tracking/types/state.hpp>

TEST(State, noStateSetAndGet)
{
  ufil::type::state::NoState state;
  EXPECT_EQ(state.stateVector().size(), 0);
}

TEST(State, position2dSetAndGet)
{
  ufil::type::state::Position2D state;
  EXPECT_EQ(state.x(), 0.0f);
  EXPECT_EQ(state.y(), 0.0f);

  state.x() = 1.1f;
  state.y() = 2.2f;

  const auto & vec = state.stateVector();
  EXPECT_EQ(vec.size(), 2);
  EXPECT_EQ(vec[ufil::type::state::Position2D::X], 1.1f);
  EXPECT_EQ(vec[ufil::type::state::Position2D::Y], 2.2f);

  const auto pos = state.position();
  EXPECT_EQ(pos.size(), 2);
  EXPECT_EQ(pos[0], 1.1f);
  EXPECT_EQ(pos[1], 2.2f);

  EXPECT_TRUE(state.covariance().isIdentity());
}

TEST(State, positionVelocity2dSetAndGet)
{
  ufil::type::state::PositionVelocity2D state;
  EXPECT_EQ(state.x(), 0.0f);
  EXPECT_EQ(state.y(), 0.0f);
  EXPECT_EQ(state.vx(), 0.0f);
  EXPECT_EQ(state.vy(), 0.0f);

  state.x() = 1.1f;
  state.y() = 2.2f;
  state.vx() = 3.3f;
  state.vy() = 4.4f;

  const auto & vec = state.stateVector();
  EXPECT_EQ(vec.size(), 4);
  EXPECT_EQ(vec[ufil::type::state::PositionVelocity2D::X], 1.1f);
  EXPECT_EQ(vec[ufil::type::state::PositionVelocity2D::Y], 2.2f);
  EXPECT_EQ(vec[ufil::type::state::PositionVelocity2D::VX], 3.3f);
  EXPECT_EQ(vec[ufil::type::state::PositionVelocity2D::VY], 4.4f);

  const auto pos = state.position();
  EXPECT_EQ(pos.size(), 2);
  EXPECT_EQ(pos[0], 1.1f);
  EXPECT_EQ(pos[1], 2.2f);
  const auto vel = state.velocity();
  EXPECT_EQ(vel.size(), 2);
  EXPECT_EQ(vel[0], 3.3f);
  EXPECT_EQ(vel[1], 4.4f);

  EXPECT_TRUE(state.covariance().isIdentity());
}

TEST(State, positionVelocityAcceleration2dSetAndGet)
{
  ufil::type::state::PositionVelocityAcceleration2D state;
  EXPECT_EQ(state.x(), 0.0f);
  EXPECT_EQ(state.y(), 0.0f);
  EXPECT_EQ(state.vx(), 0.0f);
  EXPECT_EQ(state.vy(), 0.0f);
  EXPECT_EQ(state.ax(), 0.0f);
  EXPECT_EQ(state.ay(), 0.0f);

  state.x() = 1.1f;
  state.y() = 2.2f;
  state.vx() = 3.3f;
  state.vy() = 4.4f;
  state.ax() = 5.5f;
  state.ay() = 6.6f;

  const auto & vec = state.stateVector();
  EXPECT_EQ(vec.size(), 6);
  EXPECT_EQ(vec[ufil::type::state::PositionVelocityAcceleration2D::X], 1.1f);
  EXPECT_EQ(vec[ufil::type::state::PositionVelocityAcceleration2D::Y], 2.2f);
  EXPECT_EQ(vec[ufil::type::state::PositionVelocityAcceleration2D::VX], 3.3f);
  EXPECT_EQ(vec[ufil::type::state::PositionVelocityAcceleration2D::VY], 4.4f);
  EXPECT_EQ(vec[ufil::type::state::PositionVelocityAcceleration2D::AX], 5.5f);
  EXPECT_EQ(vec[ufil::type::state::PositionVelocityAcceleration2D::AY], 6.6f);

  const auto pos = state.position();
  EXPECT_EQ(pos.size(), 2);
  EXPECT_EQ(pos[0], 1.1f);
  EXPECT_EQ(pos[1], 2.2f);
  const auto vel = state.velocity();
  EXPECT_EQ(vel.size(), 2);
  EXPECT_EQ(vel[0], 3.3f);
  EXPECT_EQ(vel[1], 4.4f);
  const auto acc = state.acceleration();
  EXPECT_EQ(acc.size(), 2);
  EXPECT_EQ(acc[0], 5.5f);
  EXPECT_EQ(acc[1], 6.6f);

  EXPECT_TRUE(state.covariance().isIdentity());
}

TEST(State, pose2dSetGet)
{
  ufil::type::state::Pose2D state;
  EXPECT_EQ(state.x(), 0.0f);
  EXPECT_EQ(state.y(), 0.0f);
  EXPECT_EQ(state.yaw(), 0.0f);

  state.x() = 1.1f;
  state.y() = 2.2f;
  state.yaw() = 3.3f;

  const auto & vec = state.stateVector();
  EXPECT_EQ(vec.size(), 3);
  EXPECT_EQ(vec[ufil::type::state::Pose2D::X], 1.1f);
  EXPECT_EQ(vec[ufil::type::state::Pose2D::Y], 2.2f);
  EXPECT_EQ(vec[ufil::type::state::Pose2D::YAW], 3.3f);

  const auto pos = state.position();
  EXPECT_EQ(pos.size(), 2);
  EXPECT_EQ(pos[0], 1.1f);
  EXPECT_EQ(pos[1], 2.2f);

  EXPECT_TRUE(state.covariance().isIdentity());
}

TEST(State, poseVelocity2dSetGet)
{
  ufil::type::state::PoseVelocity2D state;
  EXPECT_EQ(state.x(), 0.0f);
  EXPECT_EQ(state.y(), 0.0f);
  EXPECT_EQ(state.yaw(), 0.0f);
  EXPECT_EQ(state.vx(), 0.0f);
  EXPECT_EQ(state.vy(), 0.0f);
  EXPECT_EQ(state.yawRate(), 0.0f);

  state.x() = 1.1f;
  state.y() = 2.2f;
  state.yaw() = 3.3f;
  state.vx() = 4.4f;
  state.vy() = 5.5f;
  state.yawRate() = 6.6f;

  const auto & vec = state.stateVector();
  EXPECT_EQ(vec.size(), 6);
  EXPECT_EQ(vec[ufil::type::state::PoseVelocity2D::X], 1.1f);
  EXPECT_EQ(vec[ufil::type::state::PoseVelocity2D::Y], 2.2f);
  EXPECT_EQ(vec[ufil::type::state::PoseVelocity2D::YAW], 3.3f);
  EXPECT_EQ(vec[ufil::type::state::PoseVelocity2D::VX], 4.4f);
  EXPECT_EQ(vec[ufil::type::state::PoseVelocity2D::VY], 5.5f);
  EXPECT_EQ(vec[ufil::type::state::PoseVelocity2D::YAW_RATE], 6.6f);

  const auto pos = state.position();
  EXPECT_EQ(pos.size(), 2);
  EXPECT_EQ(pos[0], 1.1f);
  EXPECT_EQ(pos[1], 2.2f);
  const auto vel = state.velocity();
  EXPECT_EQ(vel.size(), 2);
  EXPECT_EQ(vel[0], 4.4f);
  EXPECT_EQ(vel[1], 5.5f);

  EXPECT_TRUE(state.covariance().isIdentity());
}

TEST(State, poseVelocityAcceleration2dSetGet)
{
  ufil::type::state::PoseVelocityAcceleration2D state;
  EXPECT_EQ(state.x(), 0.0f);
  EXPECT_EQ(state.y(), 0.0f);
  EXPECT_EQ(state.yaw(), 0.0f);
  EXPECT_EQ(state.vx(), 0.0f);
  EXPECT_EQ(state.vy(), 0.0f);
  EXPECT_EQ(state.ax(), 0.0f);
  EXPECT_EQ(state.ay(), 0.0f);
  EXPECT_EQ(state.yawRate(), 0.0f);

  state.x() = 1.1f;
  state.y() = 2.2f;
  state.yaw() = 3.3f;
  state.vx() = 4.4f;
  state.vy() = 5.5f;
  state.yawRate() = 6.6f;
  state.ax() = 7.7f;
  state.ay() = 8.8f;

  const auto & vec = state.stateVector();
  EXPECT_EQ(vec.size(), 8);
  EXPECT_EQ(vec[ufil::type::state::PoseVelocityAcceleration2D::X], 1.1f);
  EXPECT_EQ(vec[ufil::type::state::PoseVelocityAcceleration2D::Y], 2.2f);
  EXPECT_EQ(vec[ufil::type::state::PoseVelocityAcceleration2D::YAW], 3.3f);
  EXPECT_EQ(vec[ufil::type::state::PoseVelocityAcceleration2D::VX], 4.4f);
  EXPECT_EQ(vec[ufil::type::state::PoseVelocityAcceleration2D::VY], 5.5f);
  EXPECT_EQ(vec[ufil::type::state::PoseVelocityAcceleration2D::YAW_RATE], 6.6f);
  EXPECT_EQ(vec[ufil::type::state::PoseVelocityAcceleration2D::AX], 7.7f);
  EXPECT_EQ(vec[ufil::type::state::PoseVelocityAcceleration2D::AY], 8.8f);

  const auto pos = state.position();
  EXPECT_EQ(pos.size(), 2);
  EXPECT_EQ(pos[0], 1.1f);
  EXPECT_EQ(pos[1], 2.2f);
  const auto vel = state.velocity();
  EXPECT_EQ(vel.size(), 2);
  EXPECT_EQ(vel[0], 4.4f);
  EXPECT_EQ(vel[1], 5.5f);
  const auto acc = state.acceleration();
  EXPECT_EQ(acc.size(), 2);
  EXPECT_EQ(acc[0], 7.7f);
  EXPECT_EQ(acc[1], 8.8f);

  EXPECT_TRUE(state.covariance().isIdentity());
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
