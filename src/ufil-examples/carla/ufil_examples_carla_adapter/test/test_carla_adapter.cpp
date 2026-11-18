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

#include <cmath>
#include <stdexcept>
#include <vector>

#include "ufil_examples_carla_adapter/carla_adapter.hpp"

using ufil_examples_carla_adapter::ActorInfo;
using ufil_examples_carla_adapter::CarlaAdapter;
using ufil_examples_carla_adapter::CarlaAdapterConfig;
using ufil_examples_carla_adapter::Object;
using ufil_examples_carla_adapter::ObjectClass;
using ufil_examples_carla_adapter::Pose2D;
using ufil_examples_carla_adapter::SimulatorObject;

namespace
{

constexpr double kPi = 3.14159265358979323846;

SimulatorObject makeObject(std::uint32_t id, double x, double y)
{
  SimulatorObject object;
  object.id = id;
  object.state.x = x;
  object.state.y = y;
  object.dimensions = {4.0, 2.0, 1.5};
  return object;
}

CarlaAdapterConfig cooperative(const std::vector<std::int64_t> & ids)
{
  return CarlaAdapterConfig{true, ids};
}

}  // namespace

TEST(CarlaAdapterTest, ParsesCooperativeVehicleIds)
{
  using ufil_examples_carla_adapter::cooperativeVehicleId;
  EXPECT_EQ(cooperativeVehicleId("vehicle_9"), 9);
  EXPECT_EQ(cooperativeVehicleId("vehicle_012"), 12);
  EXPECT_FALSE(cooperativeVehicleId("vehicle_"));
  EXPECT_FALSE(cooperativeVehicleId("vehicle_-1"));
  EXPECT_FALSE(cooperativeVehicleId("vehicle_1a"));
  EXPECT_FALSE(cooperativeVehicleId("hero"));
  EXPECT_FALSE(cooperativeVehicleId("autopilot_1"));
  EXPECT_FALSE(cooperativeVehicleId("vehicle_99999999999999999999999"));
}

TEST(CarlaAdapterTest, ClassifiesBlueprints)
{
  using ufil_examples_carla_adapter::axleGeometry;
  using ufil_examples_carla_adapter::classify;
  EXPECT_EQ(classify("vehicle.tesla.model3"), ObjectClass::kCar);
  EXPECT_EQ(classify("vehicle.kawasaki.ninja"), ObjectClass::kMotorcycle);
  EXPECT_EQ(classify("vehicle.carlamotors.european_hgv"), ObjectClass::kTruck);
  EXPECT_EQ(classify("walker.pedestrian.0001"), ObjectClass::kOther);

  EXPECT_EQ(axleGeometry("vehicle.tesla.model3").size(), 2u);
  EXPECT_EQ(axleGeometry("vehicle.kawasaki.ninja").size(), 2u);
  const auto truck = axleGeometry("vehicle.carlamotors.european_hgv");
  ASSERT_EQ(truck.size(), 3u);
  EXPECT_GT(truck[0].center_to_axle, 0.0);
  EXPECT_LT(truck[1].center_to_axle, truck[2].center_to_axle);
  EXPECT_EQ(truck[1].wheel_detected.size(), 4u);
  EXPECT_TRUE(axleGeometry("walker.pedestrian.0001").empty());
}

TEST(CarlaAdapterTest, ValidatesConfiguration)
{
  using ufil_examples_carla_adapter::validate;
  EXPECT_FALSE(validate(CarlaAdapterConfig{}));
  EXPECT_FALSE(validate(cooperative({1, 2, 3})));
  EXPECT_TRUE(validate(cooperative({})));
  EXPECT_TRUE(validate(cooperative({1, -2})));
  EXPECT_TRUE(validate(cooperative({1, 1})));
  EXPECT_THROW(CarlaAdapter(cooperative({})), std::invalid_argument);
}

TEST(CarlaAdapterTest, ConvertsAllObjectsByDefault)
{
  CarlaAdapter adapter(CarlaAdapterConfig{});
  EXPECT_FALSE(adapter.hasActors());
  adapter.updateActors({ActorInfo{1, "vehicle.tesla.model3", "vehicle_1"}});
  EXPECT_TRUE(adapter.hasActors());

  auto input = makeObject(1, 3.0, 4.0);
  input.state.v_x = 5.0;
  input.state.yaw = 0.5;
  const auto objects = adapter.convert({input, makeObject(2, 0.0, 0.0)});
  ASSERT_EQ(objects.size(), 2u);

  EXPECT_EQ(objects[0].id, 1u);
  EXPECT_DOUBLE_EQ(objects[0].state.x, 3.0);
  EXPECT_DOUBLE_EQ(objects[0].state.v_x, 5.0);
  EXPECT_DOUBLE_EQ(objects[0].state.yaw, 0.5);
  EXPECT_DOUBLE_EQ(objects[0].dimensions.length, 4.0);
  EXPECT_DOUBLE_EQ(objects[0].existence_probability, 1.0);
  EXPECT_GT(objects[0].state_variances[0], 0.0);
  EXPECT_GT(objects[0].dimension_variances[2], 0.0);
  EXPECT_EQ(objects[0].object_class, ObjectClass::kCar);
  EXPECT_EQ(objects[0].axles.size(), 2u);

  // Not in the actor list.
  EXPECT_EQ(objects[1].object_class, ObjectClass::kOther);
  EXPECT_TRUE(objects[1].axles.empty());

  const auto & statistics = adapter.statistics();
  EXPECT_EQ(statistics.actor_lists, 1u);
  EXPECT_EQ(statistics.object_arrays, 1u);
  EXPECT_EQ(statistics.objects, 2u);
  EXPECT_EQ(statistics.converted_objects, 2u);
  EXPECT_EQ(statistics.skipped_objects, 0u);
  EXPECT_EQ(statistics.unknown_actors, 1u);
}

TEST(CarlaAdapterTest, FiltersCooperativeVehicles)
{
  CarlaAdapter adapter(cooperative({1, 3}));
  adapter.updateActors(
    {ActorInfo{10, "vehicle.tesla.model3", "vehicle_1"},
      ActorInfo{11, "vehicle.tesla.model3", "vehicle_2"},
      ActorInfo{12, "vehicle.carlamotors.european_hgv", "vehicle_3"},
      ActorInfo{13, "vehicle.tesla.model3", "hero"}});

  const auto objects = adapter.convert(
    {makeObject(10, 0.0, 0.0), makeObject(11, 0.0, 0.0), makeObject(12, 0.0, 0.0),
      makeObject(13, 0.0, 0.0), makeObject(14, 0.0, 0.0)});
  ASSERT_EQ(objects.size(), 2u);
  EXPECT_EQ(objects[0].id, 10u);
  EXPECT_EQ(objects[1].id, 12u);
  EXPECT_EQ(objects[1].object_class, ObjectClass::kTruck);
  EXPECT_EQ(adapter.statistics().skipped_objects, 3u);

  // A new actor list replaces the old one.
  adapter.updateActors({ActorInfo{11, "vehicle.tesla.model3", "vehicle_3"}});
  const auto updated = adapter.convert({makeObject(10, 0.0, 0.0), makeObject(11, 0.0, 0.0)});
  ASSERT_EQ(updated.size(), 1u);
  EXPECT_EQ(updated[0].id, 11u);
}

TEST(CarlaAdapterTest, TransformsObjects)
{
  Object object;
  object.state.x = 1.0;
  object.state.y = 0.0;
  object.state.v_x = 2.0;
  object.state.a_y = 3.0;
  object.state.yaw = 0.25;
  object.state.yaw_rate = 0.1;

  const auto transformed = ufil_examples_carla_adapter::transformObjects(
    {object}, Pose2D{10.0, 20.0, kPi / 2.0});
  ASSERT_EQ(transformed.size(), 1u);
  const auto & state = transformed[0].state;
  EXPECT_NEAR(state.x, 10.0, 1e-9);
  EXPECT_NEAR(state.y, 21.0, 1e-9);
  EXPECT_NEAR(state.v_x, 0.0, 1e-9);
  EXPECT_NEAR(state.v_y, 2.0, 1e-9);
  EXPECT_NEAR(state.a_x, -3.0, 1e-9);
  EXPECT_NEAR(state.a_y, 0.0, 1e-9);
  EXPECT_NEAR(state.yaw, 0.25 + kPi / 2.0, 1e-9);
  EXPECT_DOUBLE_EQ(state.yaw_rate, 0.1);

  const auto identity = ufil_examples_carla_adapter::transformObjects({object}, Pose2D{});
  EXPECT_DOUBLE_EQ(identity[0].state.x, 1.0);
  EXPECT_DOUBLE_EQ(identity[0].state.yaw, 0.25);
}
