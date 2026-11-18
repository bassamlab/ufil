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

#include "ssl_wheel_tracker/wheel_tracker.hpp"

#include <Eigen/Dense>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <optional>
#include <stdexcept>
#include <utility>
#include <vector>

#include <ufil_object_tracking/association/association_functions.hpp>

namespace ssl_wheel_tracker
{

namespace
{

using MeasurementMatrix = Eigen::Matrix<double, 2, 4>;

MeasurementMatrix measurementMatrix()
{
  MeasurementMatrix h = MeasurementMatrix::Zero();
  h(0, 0) = 1.0;
  h(1, 2) = 1.0;
  return h;
}

double distance(const TrackState & state, const Point2D & detection)
{
  return std::hypot(state.x() - detection.x, state.y() - detection.y);
}

}  // namespace

std::vector<std::optional<std::size_t>> assign(
  const Eigen::MatrixXd & distances, double gate_distance)
{
  const auto tracks = static_cast<std::size_t>(distances.rows());
  const auto detections = static_cast<std::size_t>(distances.cols());
  std::vector<std::optional<std::size_t>> assignment(tracks);
  if (tracks == 0 || detections == 0) {
    return assignment;
  }

  // Square problem: tracks may take a detection or their own "missed" column (cost
  // gate_distance); every detection row may stay unassigned at no cost. Forbidden entries cost
  // more than leaving every track without detection.
  const std::size_t size = tracks + detections;
  const double forbidden = gate_distance * static_cast<double>(tracks + 1) + 1.0;
  ufil::type::CostMatrix cost = ufil::type::CostMatrix::Constant(
    static_cast<Eigen::Index>(size), static_cast<Eigen::Index>(size), forbidden);
  for (std::size_t track = 0; track < tracks; ++track) {
    for (std::size_t detection = 0; detection < detections; ++detection) {
      const double d = distances(
        static_cast<Eigen::Index>(track), static_cast<Eigen::Index>(detection));
      if (std::isfinite(d) && d < gate_distance) {
        cost(static_cast<Eigen::Index>(track), static_cast<Eigen::Index>(detection)) = d;
      }
    }
    cost(static_cast<Eigen::Index>(track), static_cast<Eigen::Index>(detections + track)) =
      gate_distance;
  }
  for (std::size_t detection = 0; detection < detections; ++detection) {
    const auto row = static_cast<Eigen::Index>(tracks + detection);
    cost(row, static_cast<Eigen::Index>(detection)) = 0.0;
    cost.block(row, static_cast<Eigen::Index>(detections), 1, static_cast<Eigen::Index>(tracks))
    .setZero();
  }

  ufil::type::AssignmentMatrix solution;
  ufil::association::hungarian(cost, solution);
  for (std::size_t track = 0; track < tracks; ++track) {
    for (std::size_t detection = 0; detection < detections; ++detection) {
      const auto row = static_cast<Eigen::Index>(track);
      const auto column = static_cast<Eigen::Index>(detection);
      if (solution(row, column) != 0 && cost(row, column) < gate_distance) {
        assignment[track] = detection;
      }
    }
  }
  return assignment;
}

TrackState predict(const TrackState & state, const Timestamp & stamp, double process_noise)
{
  const double dt = ufil::to_seconds(stamp - state.stamp);
  Eigen::Matrix4d f = Eigen::Matrix4d::Identity();
  f(0, 1) = dt;
  f(2, 3) = dt;
  // Continuous white-noise acceleration, discretized per axis.
  Eigen::Matrix2d q_axis;
  q_axis << dt * dt * dt / 3.0, dt * dt / 2.0, dt * dt / 2.0, dt;
  Eigen::Matrix4d q = Eigen::Matrix4d::Zero();
  q.block<2, 2>(0, 0) = process_noise * q_axis;
  q.block<2, 2>(2, 2) = process_noise * q_axis;

  TrackState prediction;
  prediction.stamp = stamp;
  prediction.mean = f * state.mean;
  prediction.covariance = f * state.covariance * f.transpose() + q;
  prediction.matched = false;
  return prediction;
}

TrackState correct(
  const TrackState & prediction, const Point2D & detection, double measurement_noise)
{
  const MeasurementMatrix h = measurementMatrix();
  const Eigen::Matrix2d r = measurement_noise * Eigen::Matrix2d::Identity();
  const Eigen::Vector2d innovation =
    Eigen::Vector2d(detection.x, detection.y) - h * prediction.mean;
  const Eigen::Matrix2d s = h * prediction.covariance * h.transpose() + r;
  const Eigen::Matrix<double, 4, 2> gain = prediction.covariance * h.transpose() * s.inverse();

  TrackState state;
  state.stamp = prediction.stamp;
  state.mean = prediction.mean + gain * innovation;
  const Eigen::Matrix4d covariance = prediction.covariance - gain * s * gain.transpose();
  state.covariance = (covariance + covariance.transpose()) / 2.0;
  state.matched = true;
  return state;
}

WheelTracker::WheelTracker(const TrackerConfig & config)
: config_(config)
{
  if (const auto error = validate(config_)) {
    throw std::invalid_argument("invalid tracker configuration: " + *error);
  }
}

FrameResult WheelTracker::addFrame(
  const Timestamp & stamp, const std::vector<Point2D> & detections)
{
  FrameResult result = FrameResult::kAccepted;
  if (last_frame_ && stamp <= *last_frame_) {
    const auto timeout = ufil::from_seconds<Duration>(config_.deletion_timeout);
    if (stamp >= *last_frame_ - timeout) {
      ++status_.late_frames;
      return FrameResult::kLate;
    }
    // All tracks would be deleted relative to the old time anyway.
    confirmed_.clear();
    tentative_.clear();
    pending_.clear();
    last_frame_.reset();
    ++status_.resets;
    result = FrameResult::kReset;
  }
  auto & frame = pending_[stamp];
  frame.insert(frame.end(), detections.begin(), detections.end());
  return result;
}

UpdateResult WheelTracker::update()
{
  UpdateResult result;
  for (const auto & [stamp, detections] : pending_) {
    step(stamp, detections, result);
    ++result.frames;
  }
  pending_.clear();
  return result;
}

void WheelTracker::step(
  const Timestamp & stamp, const std::vector<Point2D> & detections, UpdateResult & result)
{
  ++status_.frames;
  status_.detections += detections.size();
  last_frame_ = stamp;
  std::vector<bool> used(detections.size(), false);

  associate(confirmed_, stamp, detections, used);
  for (auto it = confirmed_.begin(); it != confirmed_.end(); ) {
    if (timedOut(*it, stamp)) {
      result.deleted.push_back(it->id);
      ++status_.tracks_deleted;
      it = confirmed_.erase(it);
    } else {
      ++it;
    }
  }

  associate(tentative_, stamp, detections, used);
  for (auto it = tentative_.begin(); it != tentative_.end(); ) {
    if (it->hits >= config_.confirmation_hits) {
      it->id = next_id_++;
      result.confirmed.push_back(it->id);
      ++status_.tracks_confirmed;
      confirmed_.push_back(std::move(*it));
      it = tentative_.erase(it);
    } else if (timedOut(*it, stamp)) {
      it = tentative_.erase(it);
    } else {
      ++it;
    }
  }

  for (std::size_t i = 0; i < detections.size(); ++i) {
    if (used[i]) {
      continue;
    }
    TrackState state;
    state.stamp = stamp;
    state.mean << detections[i].x, config_.initial_velocity.x, detections[i].y,
      config_.initial_velocity.y;
    state.covariance = Eigen::Vector4d(
      config_.measurement_noise, config_.initial_velocity_variance, config_.measurement_noise,
      config_.initial_velocity_variance).asDiagonal();
    state.matched = true;

    Track track;
    track.history.push_back(state);
    track.hits = 1;
    track.last_update = stamp;
    if (track.hits >= config_.confirmation_hits) {
      track.id = next_id_++;
      result.confirmed.push_back(track.id);
      ++status_.tracks_confirmed;
      confirmed_.push_back(std::move(track));
    } else {
      tentative_.push_back(std::move(track));
    }
  }
}

void WheelTracker::associate(
  std::vector<Track> & tracks, const Timestamp & stamp,
  const std::vector<Point2D> & detections, std::vector<bool> & used) const
{
  std::vector<TrackState> predictions;
  predictions.reserve(tracks.size());
  for (const auto & track : tracks) {
    predictions.push_back(predict(track.state(), stamp, config_.process_noise));
  }

  // Used detections are excluded by an infinite distance.
  Eigen::MatrixXd distances(
    static_cast<Eigen::Index>(tracks.size()), static_cast<Eigen::Index>(detections.size()));
  for (std::size_t track = 0; track < tracks.size(); ++track) {
    for (std::size_t detection = 0; detection < detections.size(); ++detection) {
      distances(static_cast<Eigen::Index>(track), static_cast<Eigen::Index>(detection)) =
        used[detection] ? std::numeric_limits<double>::infinity() :
        distance(predictions[track], detections[detection]);
    }
  }
  const auto assignment = assign(distances, config_.gate_distance);

  for (std::size_t i = 0; i < tracks.size(); ++i) {
    auto & track = tracks[i];
    if (assignment[i]) {
      used[*assignment[i]] = true;
      append(
        track, correct(predictions[i], detections[*assignment[i]], config_.measurement_noise));
      ++track.hits;
      track.misses = 0;
      track.last_update = stamp;
    } else {
      append(track, predictions[i]);
      ++track.misses;
    }
  }
}

void WheelTracker::append(Track & track, const TrackState & state) const
{
  if (state.matched) {
    // Predictions before the last match are not part of the history.
    std::erase_if(track.history, [](const TrackState & s) {return !s.matched;});
    track.history.push_back(state);
    if (track.history.size() > config_.history_length) {
      track.history.erase(
        track.history.begin(),
        track.history.end() - static_cast<std::ptrdiff_t>(config_.history_length));
    }
  } else {
    track.history.push_back(state);
  }
}

bool WheelTracker::timedOut(const Track & track, const Timestamp & stamp) const
{
  return ufil::to_seconds(stamp - track.last_update) > config_.deletion_timeout;
}

TrackerStatus WheelTracker::status() const
{
  TrackerStatus status = status_;
  status.confirmed_tracks = confirmed_.size();
  status.tentative_tracks = tentative_.size();
  status.last_frame = last_frame_;
  return status;
}

}  // namespace ssl_wheel_tracker
