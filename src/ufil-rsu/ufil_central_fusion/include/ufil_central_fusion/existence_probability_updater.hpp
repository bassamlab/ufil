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

#ifndef UFIL_CENTRAL_FUSION__EXISTENCE_PROBABILITY_UPDATER_HPP_
#define UFIL_CENTRAL_FUSION__EXISTENCE_PROBABILITY_UPDATER_HPP_

#include <string>

#include <ufil_object_tracking/update/probability/existence_updater.hpp>

#include "ufil_central_fusion/definitions.hpp"

namespace ufil_central_fusion
{

/**
 * @brief Existence probability updater using a Dempster–Shafer based approach.
 *
 * The public interface (constructor + update()) is unchanged — only internal
 * layout, comments and formatting were improved to increase readability.
 */
class ProbDempsterShaferUpdater : public ufil::update::probability::ExistenceUpdater<Track>
{
public:
  /**
   * @brief Construct a ProbDempsterShaferUpdater.
   *
   * The constructor signature is unchanged to avoid ABI differences.
   *
   * @param weight_min minimum fusion weight
   * @param weight_max maximum fusion weight
   * @param decay_factor factor applied when decaying existence probability
   */
  explicit ProbDempsterShaferUpdater(
    ufil::type::Scalar weight_min,
    ufil::type::Scalar weight_max,
    ufil::type::Scalar decay_factor);

  /**
   * @brief Update existence probability for `track` using an optional
   * measurement. Implements the base-class virtual function.
   */
  void update(
    const TrackType & track,
    const std::optional<MeasurementType> & measurement,
    const ufil::type::Timestamp & timestamp,
    ExistenceProbabilityType & resulting_existence_probability) override;

  void setCurrentSensorData(const SensorData & sensor_data);

private:
  SensorData sensor_data_;

  // --- convenient aliases used throughout the class
  using Base = ufil::update::probability::ExistenceUpdater<Track>;
  using TrackType = typename Base::TrackType;
  using MeasurementType = typename Base::MeasurementType;
  using ExistenceProbabilityType = typename Base::ExistenceProbabilityType;

  // --- configuration and state
  std::string occluded_map;

  ufil::type::Scalar weight_min_;
  ufil::type::Scalar weight_max_;
  ufil::type::Scalar decay_factor_;

  /**
   * @brief Predict existence / non-existence / uncertainty of a track at given
   * timestamp based on internal dynamics and previous state.
   */
  void predict_track(
    const TrackType & track,
    const ufil::type::Timestamp & timestamp,
    ufil::type::Scalar & exist_prediction,
    ufil::type::Scalar & non_exist_prediction,
    ufil::type::Scalar & uncertainty_prediction);

  /**
   * @brief Calculate the persistence probability using a number of geometric
   * and sensor-based parameters.
   *
   * The function keeps the same signature as before. The `steepness` parameter
   * has a default value matching the original implementation.
   */
  ufil::type::Scalar calculate_persistence_prob(
    const TrackType & track,
    ufil::type::SensorFOV sensor_fov);

  /** @brief Compute a decay factor for the existence probability of the track. */
  ufil::type::Scalar decay_probability(const TrackType & track, const double dt);
};

}  // namespace ufil_central_fusion

#endif  // UFIL_CENTRAL_FUSION__EXISTENCE_PROBABILITY_UPDATER_HPP_
