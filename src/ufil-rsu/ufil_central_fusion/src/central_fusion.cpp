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

#include "ufil_central_fusion/central_fusion.hpp"

#include <algorithm>
#include <cmath>
#include <map>
#include <optional>
#include <set>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "object_tracker.hpp"

namespace ufil_central_fusion
{

namespace
{

SensorData toSensorData(const SensorConfig & config)
{
  SensorData data;
  data.currentTopic() = config.name;
  data.isCam() = config.is_cam;
  data.existenceProbTrust() = config.trust.existence;
  data.carTrust() = config.trust.car;
  data.truckTrust() = config.trust.truck;
  data.motorcycleTrust() = config.trust.motorcycle;
  data.bicycleTrust() = config.trust.bicycle;
  data.pedestrianTrust() = config.trust.pedestrian;
  data.stationaryTrust() = config.trust.stationary;
  data.otherTrust() = config.trust.other;
  return data;
}

double seconds(ufil::type::Timestamp a, ufil::type::Timestamp b)
{
  return ufil::to_seconds(a - b);
}

}  // namespace

class CentralFusion::Impl
{
public:
  Impl(const FusionConfig & config, const std::vector<SensorConfig> & sensors)
  : config(config), sensors(sensors)
  {
    for (const auto & sensor : sensors) {
      sensor_data.push_back(toSensorData(sensor));
      SensorStatus status;
      status.name = sensor.name;
      sensor_status.push_back(status);
    }
    prediction_sensor.currentTopic() = "prediction";
    prediction_sensor.isPredictionSensor() = true;
    createTracker();
  }

  void createTracker()
  {
    tracker = std::make_unique<ObjectTracker>(
      config.existence_min_weight, config.existence_max_weight, config.existence_decay_factor,
      config.dimension_grid_resolution, config.dimension_alpha, config.dimension_p_min,
      config.dimension_p_ref, config.dimension_p_max, config.association_threshold);
  }

  void clear()
  {
    buffer.clear();
    earliest_recalc_time.reset();
    last_measurement_time.reset();
    latest_fused.reset();
  }

  struct BufferEntry
  {
    ufil::type::Timestamp stamp;
    std::set<Measurement> measurements;
    /// Sensor state (field of view, occlusion) at the time the measurements were received.
    SensorData sensor;
  };

  FusionConfig config;
  std::vector<SensorConfig> sensors;
  std::vector<SensorData> sensor_data;
  std::vector<SensorStatus> sensor_status;
  SensorData prediction_sensor;
  std::unique_ptr<ObjectTracker> tracker;

  /// Sorted by stamp.
  std::vector<BufferEntry> buffer;
  std::optional<ufil::type::Timestamp> earliest_recalc_time;
  std::optional<ufil::type::Timestamp> last_measurement_time;
  /// Stamp of the latest measurements fused into the tracks.
  std::optional<ufil::type::Timestamp> latest_fused;

  std::uint64_t updates = 0;
  std::uint64_t replays = 0;
  std::uint64_t time_jump_resets = 0;
};

CentralFusion::CentralFusion(
  const FusionConfig & config, const std::vector<SensorConfig> & sensors)
{
  if (const auto error = validate(config)) {
    throw std::invalid_argument("invalid fusion configuration: " + *error);
  }
  if (const auto error = validate(sensors)) {
    throw std::invalid_argument("invalid sensor configuration: " + *error);
  }
  impl_ = std::make_unique<Impl>(config, sensors);
}

CentralFusion::~CentralFusion() = default;

const FusionConfig & CentralFusion::config() const
{
  return impl_->config;
}

const std::vector<SensorConfig> & CentralFusion::sensors() const
{
  return impl_->sensors;
}

MeasurementEvent CentralFusion::addMeasurements(
  std::size_t sensor, std::set<Measurement> measurements, ufil::type::Timestamp stamp)
{
  auto & impl = *impl_;
  auto & status = impl.sensor_status.at(sensor);
  const bool is_cam = impl.sensors.at(sensor).is_cam;

  MeasurementEvent event = MeasurementEvent::kBuffered;
  if (impl.last_measurement_time &&
    std::abs(seconds(stamp, *impl.last_measurement_time)) > impl.config.time_jump_threshold)
  {
    if (is_cam) {
      // CAM stamps come from the sender; a jump indicates a broken message, not a restart.
      ++status.rejected_time_jump;
      return MeasurementEvent::kRejectedTimeJump;
    }
    reset();
    ++impl.time_jump_resets;
    event = MeasurementEvent::kResetAfterTimeJump;
  }

  if (!impl.buffer.empty() &&
    seconds(impl.buffer.back().stamp, stamp) > impl.config.buffer_duration)
  {
    ++status.rejected_too_old;
    return MeasurementEvent::kRejectedTooOld;
  }

  std::set<Measurement> flagged;
  for (auto measurement : measurements) {
    measurement.isCam() = is_cam;
    flagged.insert(std::move(measurement));
  }

  const auto position = std::upper_bound(
    impl.buffer.begin(), impl.buffer.end(), stamp,
    [](const ufil::type::Timestamp & value, const Impl::BufferEntry & entry) {
      return value < entry.stamp;
    });
  impl.buffer.insert(
    position, Impl::BufferEntry{stamp, std::move(flagged), impl.sensor_data.at(sensor)});

  if (!impl.earliest_recalc_time || stamp < *impl.earliest_recalc_time) {
    impl.earliest_recalc_time = stamp;
  }
  impl.last_measurement_time = stamp;
  ++status.received;
  status.last_measurement_time = stamp;
  return event;
}

void CentralFusion::setSensorFov(std::size_t sensor, const ufil::type::SensorFOV & fov)
{
  auto & data = impl_->sensor_data.at(sensor);
  data.sensorFOV() = fov;
  data.hasFov() = true;
  data.persistencePossible() = true;
  impl_->sensor_status.at(sensor).has_fov = true;
}

void CentralFusion::setOcclusionAvailable(std::size_t sensor)
{
  impl_->sensor_data.at(sensor).occlusionPossible() = true;
  impl_->sensor_status.at(sensor).occlusion_available = true;
}

void CentralFusion::update(ufil::type::Timestamp now)
{
  auto & impl = *impl_;

  // Drop buffered measurements that are outside the buffer window.
  while (!impl.buffer.empty() &&
    seconds(impl.buffer.back().stamp, impl.buffer.front().stamp) > impl.config.buffer_duration)
  {
    impl.buffer.erase(impl.buffer.begin());
  }

  // Replay everything from the earliest new measurement on, in stamp order.
  if (impl.earliest_recalc_time) {
    const auto recalc_from = *impl.earliest_recalc_time;
    impl.earliest_recalc_time.reset();

    if (impl.latest_fused && recalc_from < *impl.latest_fused) {
      ++impl.replays;
    }
    impl.tracker->rollbackHistory(recalc_from);
    for (const auto & entry : impl.buffer) {
      if (entry.stamp < recalc_from) {
        continue;
      }
      impl.tracker->setCurrentSensorData(entry.sensor);
      auto measurements = entry.measurements;
      impl.tracker->update(std::move(measurements), entry.stamp);
    }
    if (!impl.buffer.empty()) {
      impl.latest_fused = impl.buffer.back().stamp;
    }
  }

  impl.tracker->setCurrentSensorData(impl.prediction_sensor);
  impl.tracker->predict(now);
  ++impl.updates;
}

void CentralFusion::reset()
{
  impl_->clear();
  impl_->createTracker();
}

const std::map<ufil::type::Id, Track> & CentralFusion::tracks() const
{
  return impl_->tracker->tracks();
}

FusionStatus CentralFusion::status() const
{
  FusionStatus status;
  status.sensors = impl_->sensor_status;
  status.active_tracks = impl_->tracker->tracks().size();
  status.buffered_measurement_sets = impl_->buffer.size();
  status.updates = impl_->updates;
  status.replays = impl_->replays;
  status.time_jump_resets = impl_->time_jump_resets;
  return status;
}

}  // namespace ufil_central_fusion
