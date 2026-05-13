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

#include "ufil_central_fusion/classification_updater.hpp"
#include <numeric>
#include <iostream>
#include <cmath>
#include <algorithm>

namespace ufil_central_fusion
{

// ---------------------------------------------------------------------------
// Helper functions
// ---------------------------------------------------------------------------

/// Compute movement probability based on velocity.
ufil::type::Scalar calculateMoveProbability(
  const std::optional<DynamicMeasurement> & measurement)
{
  if (!measurement || !measurement->hasVx() || !measurement->hasVy()) {
    return 0.0f;
  }

  const ufil::type::Scalar vx = std::abs(measurement->vx());
  const ufil::type::Scalar vy = std::abs(measurement->vy());
  const ufil::type::Scalar speed = vx + vy;
  return 1.0f - std::exp(-0.5f * speed);
}

/// Return true if `a` is a subset of `b`.
bool isSubset(const std::set<int> & a, const std::set<int> & b)
{
  return std::includes(b.begin(), b.end(), a.begin(), a.end());
}

/// Return true if two sets overlap.
bool intersects(const std::set<int> & a, const std::set<int> & b)
{
  std::set<int> intersection;
  std::set_intersection(a.begin(), a.end(), b.begin(), b.end(),
                        std::inserter(intersection, intersection.begin()));
  return !intersection.empty();
}

/// Normalize ratio of sums over specific indices.
ufil::type::Scalar normalizedSum(
  const std::vector<ufil::type::Scalar> & values,
  const std::vector<size_t> & numIdx,
  const std::vector<size_t> & denomIdx)
{
  ufil::type::Scalar num = 0.0f, denom = 0.0f;
  for (auto i : numIdx) {
    num += values[i];
  }
  for (auto i : denomIdx) {
    denom += values[i];
  }
  return denom > 0 ? num / denom : 0.0f;
}

// ---------------------------------------------------------------------------
// Composite BBA calculators
// ---------------------------------------------------------------------------

ufil::type::Scalar computeVehicleBBA(
  ufil::type::Scalar moveProb,
  ufil::type::Scalar probVehicle,
  const SensorData & sensor,
  const std::vector<ufil::type::Scalar> & probs)
{
  return moveProb * probVehicle *
         ((1.0f - sensor.carTrust()) * probs[0] +
         (1.0f - sensor.truckTrust()) * probs[1] +
         (1.0f - sensor.motorcycleTrust()) * probs[2]);
}

ufil::type::Scalar computeVRUBBA(
  ufil::type::Scalar moveProb,
  ufil::type::Scalar probVru,
  const SensorData & sensor,
  const std::vector<ufil::type::Scalar> & probs)
{
  return moveProb * probVru *
         ((1.0f - sensor.bicycleTrust()) * probs[4] +
         (1.0f - sensor.pedestrianTrust()) * probs[3]);
}

ufil::type::Scalar computeTrafficBBA(
  ufil::type::Scalar moveProb,
  ufil::type::Scalar probVehicle,
  ufil::type::Scalar probVru,
  const SensorData & sensor,
  const std::vector<ufil::type::Scalar> & probs)
{
  return moveProb * probVehicle *
         ((1.0f - sensor.carTrust()) * probs[0] +
         (1.0f - sensor.truckTrust()) * probs[1] +
         (1.0f - sensor.motorcycleTrust()) * probs[2]) +
         moveProb * probVru *
         ((1.0f - sensor.bicycleTrust()) * probs[4] +
         (1.0f - sensor.pedestrianTrust()) * probs[3]);
}

ufil::type::Scalar computeVehicleStationaryBBA(
  ufil::type::Scalar moveProb,
  ufil::type::Scalar probVehicle,
  const SensorData & sensor,
  const std::vector<ufil::type::Scalar> & probs)
{
  return (1.0f - moveProb) * probVehicle *
         ((1.0f - sensor.carTrust()) * probs[0] +
         (1.0f - sensor.truckTrust()) * probs[1] +
         (1.0f - sensor.motorcycleTrust()) * probs[2]);
}

ufil::type::Scalar computeVRUStationaryBBA(
  ufil::type::Scalar moveProb,
  ufil::type::Scalar probVru,
  const SensorData & sensor,
  const std::vector<ufil::type::Scalar> & probs)
{
  return (1.0f - moveProb) * probVru *
         ((1.0f - sensor.bicycleTrust()) * probs[4] +
         (1.0f - sensor.pedestrianTrust()) * probs[3]);
}

// ---------------------------------------------------------------------------
// Main update
// ---------------------------------------------------------------------------

void ClassificationDempsterShaferUpdater::update(
  const Track & track,
  const std::optional<DynamicMeasurement> & measurement,
  const ufil::type::Timestamp & /*timestamp*/,
  Classification & resulting_classification)
{
  if (!measurement) {return;}

  const auto & sensor = this->sensor_data_;
  ufil::type::Scalar moveProb = calculateMoveProbability(measurement);
  const size_t N = all_classes.size();

  // -------------------------------------------------------------------------
  // Step 1: Compute elementary BBAs
  // -------------------------------------------------------------------------
  std::vector<ufil::type::Scalar> probs(N, 0.0f);
  std::vector<ufil::type::Scalar> bbas(N + EXTRA_COUNT, 0.0f);

  for (size_t i = 0; i < N; ++i) {
    int type = all_classes[i];
    probs[i] = measurement->classification().classificationVector()(type);
    bbas[i] = probs[i] * sensor.classificationTrustVector()(type);
  }

  // -------------------------------------------------------------------------
  // Step 2: Compute composite BBAs
  // -------------------------------------------------------------------------
  ufil::type::Scalar probVehicle = normalizedSum(bbas, {0, 1, 2}, {0, 1, 2, 3, 4});
  ufil::type::Scalar probVru = normalizedSum(bbas, {3, 4}, {0, 1, 2, 3, 4});

  bbas[N + VEHICLE] = computeVehicleBBA(moveProb, probVehicle, sensor, probs);
  bbas[N + VRU] = computeVRUBBA(moveProb, probVru, sensor, probs);
  bbas[N + TRAFFIC] = computeTrafficBBA(moveProb, probVehicle, probVru, sensor, probs);
  bbas[N + VEHICLE_STATIONARY] = computeVehicleStationaryBBA(moveProb, probVehicle, sensor, probs);
  bbas[N + VRU_STATIONARY] = computeVRUStationaryBBA(moveProb, probVru, sensor, probs);

  // Discernment mass = remaining uncertainty
  ufil::type::Scalar massSum = std::accumulate(bbas.begin(), bbas.begin() + N + EXTRA_COUNT - 1,
      0.0f);
  bbas[N + DISCERNMENT] = std::max(static_cast<ufil::type::Scalar>(0.0),
      static_cast<ufil::type::Scalar>(1.0) - massSum);

  // -------------------------------------------------------------------------
  // Step 3: Conflict calculation
  // -------------------------------------------------------------------------
  const auto track_bba = track.previousClassification().basicBeliefClassification();
  ufil::type::Scalar conflict = 0.0f;

  for (size_t i = 0; i < all_bbas.size(); ++i) {
    const auto & globalSet = array_of_sets[i];
    for (size_t j = 0; j < bbas.size(); ++j) {
      const auto & sensorSet = array_of_sets[j];  // reuse mapping cyclically
      if (!intersects(globalSet, sensorSet)) {
        conflict += track_bba.classificationVector()(all_bbas[i]) * bbas[j];
      }
    }
  }

  // -------------------------------------------------------------------------
  // Step 4: Fusion with conflict handling
  // -------------------------------------------------------------------------
  std::vector<ufil::type::Scalar> global_bba(N + EXTRA_COUNT, 0.0f);

  // Smooth alpha curve: alpha ∈ [0,1], decreases as conflict increases
  ufil::type::Scalar alpha = static_cast<ufil::type::Scalar>(1.0) /
    (static_cast<ufil::type::Scalar>(1.0) + conflict);

  for (size_t i = 0; i < all_bbas.size(); ++i) {
    const auto & globalSet = array_of_sets[i];
    ufil::type::Scalar bba_value = 0.0f;

    for (size_t j = 0; j < bbas.size(); ++j) {
      const auto & sensorSet = array_of_sets[j];
      if (isSubset(globalSet, sensorSet)) {
        bba_value += track_bba.classificationVector()(all_bbas[i]) * bbas[j];
      }
    }

    ufil::type::Scalar denom = std::max(static_cast<ufil::type::Scalar>(1.0) - conflict,
        static_cast<ufil::type::Scalar>(1e-6));
    ufil::type::Scalar fusedBBA = std::clamp(bba_value / denom,
        static_cast<ufil::type::Scalar>(0.0), static_cast<ufil::type::Scalar>(1.0));

      // Soft update: blend previous BBA with newly fused BBA
    global_bba[i] = alpha * fusedBBA + (1.0f - alpha) *
      track_bba.classificationVector()(all_bbas[i]);
  }

  // Optional: handle DISCERNMENT / ignorance mass softly
  size_t ignorance_index = N + EXTRA_COUNT - 1;
  global_bba[ignorance_index] = alpha * global_bba[ignorance_index] + (1.0f - alpha) *
    track_bba.classificationVector()(ignorance_index);

  // -------------------------------------------------------------------------
  // Step 5: Convert global BBA → probabilities
  // -------------------------------------------------------------------------
  std::vector<ufil::type::Scalar> global_prob(N, 0.0f);

  global_prob[0] = global_bba[0] + (1.0f / 3) * global_bba[N + VEHICLE] + (1.0f / 5) *
    global_bba[N + TRAFFIC] + (1.0f / 4) * global_bba[N + VEHICLE_STATIONARY];
  global_prob[1] = global_bba[1] + (1.0f / 3) * global_bba[N + VEHICLE] + (1.0f / 5) *
    global_bba[N + TRAFFIC] + (1.0f / 4) * global_bba[N + VEHICLE_STATIONARY];
  global_prob[2] = global_bba[2] + (1.0f / 3) * global_bba[N + VEHICLE] + (1.0f / 5) *
    global_bba[N + TRAFFIC] + (1.0f / 4) * global_bba[N + VEHICLE_STATIONARY];
  global_prob[3] = global_bba[3] + (1.0f / 2) * global_bba[N + VRU] + (1.0f / 5) *
    global_bba[N + TRAFFIC] + (1.0f / 3) * global_bba[N + VRU_STATIONARY];
  global_prob[4] = global_bba[4] + (1.0f / 2) * global_bba[N + VRU] + (1.0f / 5) *
    global_bba[N + TRAFFIC] + (1.0f / 3) * global_bba[N + VRU_STATIONARY];
  global_prob[5] = global_bba[5] + (1.0f / 4) * global_bba[N + VEHICLE_STATIONARY] + (1.0f / 3) *
    global_bba[N + VRU_STATIONARY];

  for (auto & p : global_prob) {
    p = std::clamp(p, static_cast<ufil::type::Scalar>(0.0f), static_cast<ufil::type::Scalar>(1.0f));
  }

  // -------------------------------------------------------------------------
  // Step 6: Update classification result
  // -------------------------------------------------------------------------
  resulting_classification.classificationVector()(ufil::type::classification::ObjectClassification::
    CAR) = global_prob[0];
  resulting_classification.classificationVector()(ufil::type::classification::ObjectClassification::
    TRUCK) = global_prob[1];
  resulting_classification.classificationVector()(ufil::type::classification::ObjectClassification::
    MOTORCYCLE) = global_prob[2];
  resulting_classification.classificationVector()(ufil::type::classification::ObjectClassification::
    PEDESTRIAN) = global_prob[3];
  resulting_classification.classificationVector()(ufil::type::classification::ObjectClassification::
    BICYCLE) = global_prob[4];
  resulting_classification.classificationVector()(ufil::type::classification::ObjectClassification::
    STATIONARY) = global_prob[5];

  ufil::type::Scalar sum = std::accumulate(global_prob.begin(), global_prob.end(),
      static_cast<ufil::type::Scalar>(0.0));
  resulting_classification.classificationVector()(ufil::type::classification::ObjectClassification::
    OTHER) =
    std::max(static_cast<ufil::type::Scalar>(0.0), static_cast<ufil::type::Scalar>(1.0) - sum);
}

void ClassificationDempsterShaferUpdater::setCurrentSensorData(const SensorData & sensor_data)
{
  this->sensor_data_ = sensor_data;
}

}  // namespace ufil_central_fusion
