// Copyright 2025 Chair of Embedded Software (Computer Science 11) - RWHT Aachen University
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

#ifndef UFIL_CENTRAL_FUSION__CLASSIFICATION_UPDATER_HPP_
#define UFIL_CENTRAL_FUSION__CLASSIFICATION_UPDATER_HPP_

#include <set>
#include <vector>
#include <optional>

#include <ufil_object_tracking/update/classification/classification_updater.hpp>

#include "ufil_central_fusion/classification.hpp"
#include "ufil_central_fusion/definitions.hpp"

namespace ufil_central_fusion
{

/// Extra indices for composite BBAs (beyond the elementary classes).
enum ExtraBBAIndex
{
  VEHICLE = 0,
  VRU = 1,
  TRAFFIC = 2,
  VEHICLE_STATIONARY = 3,
  VRU_STATIONARY = 4,
  DISCERNMENT = 5
};

/// Implements classification fusion using Dempster-Shafer theory.
class ClassificationDempsterShaferUpdater
  : public ufil::update::classification::ClassificationUpdater<Track>
{
public:
  const int EXTRA_COUNT = 6;

  void update(
    const Track & track,
    const std::optional<DynamicMeasurement> & measurement,
    const ufil::type::Timestamp & timestamp,
    Classification & resulting_classification) override;

  void setCurrentSensorData(const SensorData & sensor_data);

private:
  SensorData sensor_data_;
  // Elementary BBA types (basic belief sets)
  const std::vector<int> all_bbas {
    BasicBeliefClassification::CAR,
    BasicBeliefClassification::TRUCK,
    BasicBeliefClassification::MOTORCYCLE,
    BasicBeliefClassification::PEDESTRIAN,
    BasicBeliefClassification::BICYCLE,
    BasicBeliefClassification::STATIONARY,
    BasicBeliefClassification::VEHICLES,
    BasicBeliefClassification::VRU,
    BasicBeliefClassification::TRAFFIC,
    BasicBeliefClassification::VEHICLE_STATIONARY,
    BasicBeliefClassification::VRU_STATIONARY,
    BasicBeliefClassification::DISCERNMENT
  };

  // Atomic classification types
  const std::vector<int> all_classes{
    ufil::type::classification::ObjectClassification::CAR,
    ufil::type::classification::ObjectClassification::TRUCK,
    ufil::type::classification::ObjectClassification::MOTORCYCLE,
    ufil::type::classification::ObjectClassification::PEDESTRIAN,
    ufil::type::classification::ObjectClassification::BICYCLE,
    ufil::type::classification::ObjectClassification::STATIONARY
  };

  // Mapping from BBA type → set of classes it represents
  const std::set<int> array_of_sets[12] = {
    {ufil::type::classification::ObjectClassification::CAR},
    {ufil::type::classification::ObjectClassification::TRUCK},
    {ufil::type::classification::ObjectClassification::MOTORCYCLE},
    {ufil::type::classification::ObjectClassification::PEDESTRIAN},
    {ufil::type::classification::ObjectClassification::BICYCLE},
    {ufil::type::classification::ObjectClassification::STATIONARY},
    {ufil::type::classification::ObjectClassification::CAR,
      ufil::type::classification::ObjectClassification::TRUCK,
      ufil::type::classification::ObjectClassification::MOTORCYCLE},
    {ufil::type::classification::ObjectClassification::PEDESTRIAN,
      ufil::type::classification::ObjectClassification::BICYCLE},
    {ufil::type::classification::ObjectClassification::CAR,
      ufil::type::classification::ObjectClassification::TRUCK,
      ufil::type::classification::ObjectClassification::MOTORCYCLE,
      ufil::type::classification::ObjectClassification::PEDESTRIAN,
      ufil::type::classification::ObjectClassification::BICYCLE},
    {ufil::type::classification::ObjectClassification::CAR,
      ufil::type::classification::ObjectClassification::TRUCK,
      ufil::type::classification::ObjectClassification::MOTORCYCLE,
      ufil::type::classification::ObjectClassification::STATIONARY},
    {ufil::type::classification::ObjectClassification::PEDESTRIAN,
      ufil::type::classification::ObjectClassification::BICYCLE,
      ufil::type::classification::ObjectClassification::STATIONARY},
    {ufil::type::classification::ObjectClassification::CAR,
      ufil::type::classification::ObjectClassification::TRUCK,
      ufil::type::classification::ObjectClassification::MOTORCYCLE,
      ufil::type::classification::ObjectClassification::PEDESTRIAN,
      ufil::type::classification::ObjectClassification::BICYCLE,
      ufil::type::classification::ObjectClassification::STATIONARY}
  };
};

}  // namespace ufil_central_fusion

#endif  // UFIL_CENTRAL_FUSION__CLASSIFICATION_UPDATER_HPP_
