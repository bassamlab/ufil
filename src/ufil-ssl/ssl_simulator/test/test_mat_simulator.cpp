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

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <vector>

#include "ssl_simulator/config.hpp"
#include "ssl_simulator/mat_simulator.hpp"

using ssl_simulator::Axle;
using ssl_simulator::Cell;
using ssl_simulator::FrameEvent;
using ssl_simulator::MatPose;
using ssl_simulator::MatSimulator;
using ssl_simulator::Point2D;
using ssl_simulator::PressureMap;
using ssl_simulator::SimulatorConfig;
using ssl_simulator::Timestamp;
using ssl_simulator::Vehicle;
using ssl_simulator::VehicleState;

namespace
{

constexpr double kPi = 3.14159265358979323846;

Timestamp at(double seconds)
{
  return ufil::from_seconds<Timestamp>(seconds);
}

/// 10 m x 5 m mat with 0.2 m cells at (100, 50).
SimulatorConfig matConfig(double heading = 0.0)
{
  SimulatorConfig config;
  config.pose = MatPose{100.0, 50.0, 0.011, heading};
  config.seed = 42;
  return config;
}

/// Car with a track width of 1.6 m and axles 1.4 m in front of and behind the center.
Vehicle car(
  double x, double y, double yaw, const Timestamp & time, double vx = 0.0,
  double vy = 0.0)
{
  Vehicle vehicle;
  vehicle.id = 7;
  vehicle.state = VehicleState(time);
  vehicle.state.x() = x;
  vehicle.state.y() = y;
  vehicle.state.vx() = vx;
  vehicle.state.vy() = vy;
  vehicle.state.yaw() = yaw;
  vehicle.axles = {Axle{false, 1.6, 1.4}, Axle{false, 1.6, -1.4}};
  return vehicle;
}

/// The four cells around a cell corner, i.e. the contact cells of a wheel placed on the corner.
std::vector<Cell> cellsAroundCorner(std::size_t column, std::size_t row)
{
  return {{column - 1, row - 1}, {column, row - 1}, {column - 1, row}, {column, row}};
}

/// Wheel contacts of car() centered at mat (3.0, 2.4) with yaw 0 in the mat frame.
std::vector<std::vector<Cell>> expectedContacts()
{
  return {
    cellsAroundCorner(22, 16), cellsAroundCorner(22, 8),
    cellsAroundCorner(8, 16), cellsAroundCorner(8, 8)};
}

void expectContacts(const PressureMap & pressure, const std::vector<std::vector<Cell>> & wheels)
{
  for (const auto & cells : wheels) {
    std::uint8_t maximum = 0;
    for (const auto & cell : cells) {
      EXPECT_GE(pressure.at(cell), 18) << cell.column << ", " << cell.row;
      maximum = std::max(maximum, pressure.at(cell));
    }
    EXPECT_EQ(maximum, 27);
  }
  // Crosstalk only spreads along the rows, so every other row is empty.
  for (std::size_t row = 0; row < pressure.rows; ++row) {
    const bool wheel_row = row == 7 || row == 8 || row == 15 || row == 16;
    for (std::size_t column = 0; column < pressure.columns && !wheel_row; ++column) {
      EXPECT_EQ(pressure.at(column, row), 0) << column << ", " << row;
    }
  }
}

}  // namespace

TEST(Config, CellCounts)
{
  SimulatorConfig config;
  EXPECT_EQ(config.columns(), 50u);
  EXPECT_EQ(config.rows(), 25u);
  config.width = 8.64;
  config.height = 17.28;
  EXPECT_EQ(config.columns(), 44u);
  EXPECT_EQ(config.rows(), 87u);
  EXPECT_FALSE(validate(config).has_value());
}

TEST(Config, InvalidValues)
{
  const auto invalid = [](auto modify) {
      SimulatorConfig config;
      modify(config);
      return validate(config).has_value();
    };
  EXPECT_TRUE(invalid([](SimulatorConfig & c) {c.width = 0.0;}));
  EXPECT_TRUE(invalid([](SimulatorConfig & c) {c.height = -1.0;}));
  EXPECT_TRUE(invalid([](SimulatorConfig & c) {c.resolution = 0.0;}));
  EXPECT_TRUE(invalid([](SimulatorConfig & c) {c.resolution = 1e-6;}));
  EXPECT_TRUE(invalid([](SimulatorConfig & c) {c.threshold = 0;}));
  EXPECT_TRUE(invalid([](SimulatorConfig & c) {c.threshold = 256;}));
  EXPECT_TRUE(invalid([](SimulatorConfig & c) {c.stale_duration = 0.0;}));
  EXPECT_TRUE(invalid([](SimulatorConfig & c) {c.wheel_contact_size = 0.0;}));
  EXPECT_TRUE(
    invalid([](SimulatorConfig & c) {c.pose.heading = std::numeric_limits<double>::quiet_NaN();}));

  SimulatorConfig config;
  config.threshold = 0;
  EXPECT_THROW(MatSimulator{config}, std::invalid_argument);
}

TEST(Prediction, ConstantVelocityAndYawRate)
{
  VehicleState state(at(100.0));
  state.x() = 1.0;
  state.y() = 2.0;
  state.vx() = 3.0;
  state.vy() = -1.0;
  state.yaw() = 0.5;
  state.yawRate() = 0.2;

  const auto prediction = ssl_simulator::predict(state, at(100.5));
  EXPECT_NEAR(prediction.x(), 2.5, 1e-9);
  EXPECT_NEAR(prediction.y(), 1.5, 1e-9);
  EXPECT_NEAR(prediction.vx(), 3.0, 1e-9);
  EXPECT_NEAR(prediction.vy(), -1.0, 1e-9);
  EXPECT_NEAR(prediction.yaw(), 0.6, 1e-9);
  EXPECT_NEAR(prediction.yawRate(), 0.2, 1e-9);
  EXPECT_EQ(prediction.timestamp(), at(100.5));
}

TEST(Prediction, VelocityIsAbsoluteNotAlongHeading)
{
  // Heading north, but moving east (e.g. a drifting vehicle): the velocity is absolute.
  VehicleState state(at(0.0));
  state.vx() = 2.0;
  state.yaw() = kPi / 2.0;
  const auto prediction = ssl_simulator::predict(state, at(1.0));
  EXPECT_NEAR(prediction.x(), 2.0, 1e-9);
  EXPECT_NEAR(prediction.y(), 0.0, 1e-9);
}

TEST(Prediction, YawIsNormalized)
{
  VehicleState state(at(0.0));
  state.yaw() = 3.0;
  state.yawRate() = 1.0;
  const auto prediction = ssl_simulator::predict(state, at(1.0));
  EXPECT_NEAR(prediction.yaw(), 4.0 - 2.0 * kPi, 1e-9);
}

TEST(Geometry, MapToMat)
{
  const MatPose pose{10.0, 20.0, 0.0, kPi / 2.0};
  const auto point = ssl_simulator::mapToMat(pose, {10.0, 23.0});
  EXPECT_NEAR(point.x, 3.0, 1e-9);
  EXPECT_NEAR(point.y, 0.0, 1e-9);

  const auto origin = ssl_simulator::mapToMat(MatPose{1.0, 2.0, 0.0, 0.0}, {4.0, 7.0});
  EXPECT_NEAR(origin.x, 3.0, 1e-9);
  EXPECT_NEAR(origin.y, 5.0, 1e-9);
}

TEST(Geometry, WheelPositions)
{
  const std::vector<Axle> axles{{false, 1.6, 1.4}, {true, 0.0, -1.0}};
  auto wheels = ssl_simulator::wheelPositions({5.0, 5.0}, 0.0, axles);
  ASSERT_EQ(wheels.size(), 3u);
  EXPECT_NEAR(wheels[0].x, 6.4, 1e-9);
  EXPECT_NEAR(wheels[0].y, 5.8, 1e-9);
  EXPECT_NEAR(wheels[1].x, 6.4, 1e-9);
  EXPECT_NEAR(wheels[1].y, 4.2, 1e-9);
  EXPECT_NEAR(wheels[2].x, 4.0, 1e-9);
  EXPECT_NEAR(wheels[2].y, 5.0, 1e-9);

  wheels = ssl_simulator::wheelPositions({5.0, 5.0}, kPi / 2.0, axles);
  EXPECT_NEAR(wheels[0].x, 4.2, 1e-9);
  EXPECT_NEAR(wheels[0].y, 6.4, 1e-9);
  EXPECT_NEAR(wheels[2].x, 5.0, 1e-9);
  EXPECT_NEAR(wheels[2].y, 4.0, 1e-9);
}

TEST(Geometry, ContactCells)
{
  using ssl_simulator::contactCells;
  const std::vector<Cell> around_corner{{4, 4}, {5, 4}, {4, 5}, {5, 5}};
  EXPECT_EQ(contactCells({1.0, 1.0}, 0.0, 0.405, 0.2, 50, 25), around_corner);
  EXPECT_EQ(contactCells({1.0, 1.0}, kPi / 4.0, 0.405, 0.2, 50, 25), around_corner);
  // A wheel on a cell center also covers the neighbours, whose centers are 0.2 m away.
  EXPECT_EQ(contactCells({1.1, 1.1}, 0.0, 0.405, 0.2, 50, 25).size(), 9u);

  // Clipped at the mat border.
  EXPECT_EQ(contactCells({0.0, 0.0}, 0.0, 0.405, 0.2, 50, 25), (std::vector<Cell>{{0, 0}}));
  EXPECT_EQ(contactCells({10.0, 5.0}, 0.0, 0.405, 0.2, 50, 25), (std::vector<Cell>{{49, 24}}));
  EXPECT_TRUE(contactCells({-1.0, 1.0}, 0.0, 0.405, 0.2, 50, 25).empty());
  EXPECT_TRUE(contactCells({1.0, 5.5}, 0.0, 0.405, 0.2, 50, 25).empty());
}

TEST(Layer, LineCrosstalk)
{
  PressureMap pressure(20, 2);
  pressure.at(5, 1) = 27;
  pressure.at(6, 1) = 21;
  const auto result = ssl_simulator::addLineCrosstalk(pressure);
  EXPECT_EQ(result.at(5, 1), 27);
  EXPECT_EQ(result.at(6, 1), 21);
  EXPECT_EQ(result.at(4, 1), 18);
  EXPECT_EQ(result.at(3, 1), 12);
  EXPECT_EQ(result.at(0, 1), 3);
  EXPECT_EQ(result.at(7, 1), 14);   // max(12 from 5, 14 from 6)
  EXPECT_EQ(result.at(12, 1), 1);   // 27 -> 18 -> 12 -> 8 -> 5 -> 3 -> 2 -> 1
  EXPECT_EQ(result.at(13, 1), 0);
  for (std::size_t column = 0; column < 20; ++column) {
    EXPECT_EQ(result.at(column, 0), 0);
  }
}

TEST(Simulator, RasterizesWheelsAtKnownPose)
{
  MatSimulator simulator(matConfig());
  // Center at mat (3.0, 2.4): the wheels lie on the cell corners (4.4, 3.2), (4.4, 1.6),
  // (1.6, 3.2) and (1.6, 1.6).
  simulator.setVehicles({car(103.0, 52.4, 0.0, at(10.0))}, at(10.0));
  const auto frame = simulator.step(at(10.0));
  EXPECT_EQ(frame.vehicles_on_mat, 1u);
  EXPECT_EQ(frame.event, FrameEvent::kNone);
  ASSERT_EQ(frame.pressure.columns, 50u);
  ASSERT_EQ(frame.pressure.rows, 25u);
  expectContacts(frame.pressure, expectedContacts());
  EXPECT_EQ(simulator.status().vehicles_on_mat, 1u);
  EXPECT_EQ(simulator.status().active_cells, frame.pressure.activeCells());
}

TEST(Simulator, MatHeadingAndOffset)
{
  // The mat x axis points north: mat (u, v) is map (100 - v, 50 + u). A car heading north
  // centered at mat (3.0, 2.4) produces the same contacts as in RasterizesWheelsAtKnownPose.
  MatSimulator simulator(matConfig(kPi / 2.0));
  simulator.setVehicles({car(97.6, 53.0, kPi / 2.0, at(10.0))}, at(10.0));
  const auto frame = simulator.step(at(10.0));
  EXPECT_EQ(frame.vehicles_on_mat, 1u);
  expectContacts(frame.pressure, expectedContacts());
}

TEST(Simulator, PredictsToFrameTime)
{
  MatSimulator simulator(matConfig());
  // Measured 0.3 s before the frame 3 m further west, driving east with 10 m/s.
  simulator.setVehicles({car(100.0, 52.4, 0.0, at(9.7), 10.0)}, at(9.95));
  expectContacts(simulator.step(at(10.0)).pressure, expectedContacts());
}

TEST(Simulator, Threshold)
{
  auto config = matConfig();
  config.threshold = 25;
  MatSimulator simulator(config);
  simulator.setVehicles({car(103.0, 52.4, 0.0, at(10.0))}, at(10.0));
  const auto frame = simulator.step(at(10.0));
  // Only the strongest cell of each wheel (27) passes; crosstalk (<= 18) and noise do not.
  EXPECT_EQ(frame.pressure.activeCells(), 4u);
  for (const auto value : frame.pressure.values) {
    EXPECT_TRUE(value == 0 || value == 27);
  }
}

TEST(Simulator, NoiseIsBelowTheDefaultThreshold)
{
  MatSimulator quiet(matConfig());
  EXPECT_EQ(quiet.step(at(1.0)).pressure.activeCells(), 0u);

  auto config = matConfig();
  config.threshold = 1;
  MatSimulator noisy(config);
  const auto frame = noisy.step(at(1.0));
  EXPECT_GT(frame.pressure.activeCells(), 0u);
  EXPECT_LE(frame.pressure.activeCells(), 50u * 25u / 64u);
  for (const auto value : frame.pressure.values) {
    EXPECT_LE(value, 4);
  }
}

TEST(Simulator, SeedMakesFramesReproducible)
{
  auto config = matConfig();
  config.threshold = 1;
  MatSimulator first(config);
  MatSimulator second(config);
  for (auto * simulator : {&first, &second}) {
    simulator->setVehicles({car(103.0, 52.4, 0.3, at(10.0), 1.0, 0.5)}, at(10.0));
  }
  EXPECT_EQ(first.step(at(10.1)).pressure.values, second.step(at(10.1)).pressure.values);
}

TEST(Simulator, ObjectsOutsideTheMat)
{
  MatSimulator simulator(matConfig());
  simulator.setVehicles(
    {car(90.0, 52.4, 0.0, at(10.0)), car(103.0, 60.0, 0.0, at(10.0)),
      car(103.0, 30.0, kPi / 2.0, at(10.0))}, at(10.0));
  const auto frame = simulator.step(at(10.0));
  EXPECT_EQ(frame.vehicles_on_mat, 0u);
  EXPECT_EQ(frame.pressure.activeCells(), 0u);
  EXPECT_EQ(simulator.status().vehicles, 3u);

  // Center outside, front wheels on the mat at mat x = 0.2.
  simulator.setVehicles({car(98.8, 52.4, 0.0, at(10.0))}, at(10.0));
  const auto partial = simulator.step(at(10.0));
  EXPECT_EQ(partial.vehicles_on_mat, 1u);
  EXPECT_GE(partial.pressure.at(0, 15), 18);
  EXPECT_GE(partial.pressure.at(0, 8), 18);
}

TEST(Simulator, VehicleWithoutAxlesIsInvisible)
{
  MatSimulator simulator(matConfig());
  auto vehicle = car(103.0, 52.4, 0.0, at(10.0));
  vehicle.axles.clear();
  simulator.setVehicles({vehicle}, at(10.0));
  EXPECT_EQ(simulator.step(at(10.0)).pressure.activeCells(), 0u);
}

TEST(Simulator, StaleVehiclesExpire)
{
  MatSimulator simulator(matConfig());
  simulator.setVehicles({car(103.0, 52.4, 0.0, at(10.0))}, at(10.0));
  EXPECT_EQ(simulator.step(at(10.4)).vehicles_on_mat, 1u);
  const auto frame = simulator.step(at(10.6));
  EXPECT_EQ(frame.event, FrameEvent::kVehiclesExpired);
  EXPECT_EQ(frame.pressure.activeCells(), 0u);
  EXPECT_EQ(simulator.status().vehicles, 0u);
  EXPECT_EQ(simulator.status().expirations, 1u);
  EXPECT_EQ(simulator.step(at(10.7)).event, FrameEvent::kNone);
}

TEST(Simulator, BackwardTimeJumpDropsOldVehicles)
{
  MatSimulator simulator(matConfig());
  simulator.setVehicles({car(103.0, 52.4, 0.0, at(10.0))}, at(10.0));
  EXPECT_EQ(simulator.step(at(10.1)).vehicles_on_mat, 1u);
  const auto frame = simulator.step(at(1.0));
  EXPECT_EQ(frame.event, FrameEvent::kTimeJump);
  EXPECT_EQ(frame.vehicles_on_mat, 0u);
  EXPECT_EQ(simulator.status().time_jumps, 1u);
  EXPECT_EQ(simulator.status().vehicles, 0u);
  EXPECT_FALSE(simulator.status().last_object_list.has_value());

  // Vehicles received after the jump are simulated.
  simulator.setVehicles({car(103.0, 52.4, 0.0, at(1.0))}, at(1.0));
  EXPECT_EQ(simulator.step(at(1.05)).vehicles_on_mat, 1u);
}

TEST(Simulator, RejectsNonFiniteStates)
{
  MatSimulator simulator(matConfig());
  auto broken = car(103.0, 52.4, 0.0, at(10.0));
  broken.state.yaw() = std::numeric_limits<double>::quiet_NaN();
  EXPECT_EQ(simulator.setVehicles({broken, car(103.0, 52.4, 0.0, at(10.0))}, at(10.0)), 1u);
  const auto status = simulator.status();
  EXPECT_EQ(status.vehicles, 1u);
  EXPECT_EQ(status.rejected_vehicles, 1u);
  EXPECT_EQ(status.object_lists, 1u);
  EXPECT_EQ(simulator.step(at(10.0)).vehicles_on_mat, 1u);
}
