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

#ifndef SSL_WHEEL_TRACKER__WHEEL_TRACKER_HPP_
#define SSL_WHEEL_TRACKER__WHEEL_TRACKER_HPP_

#include <Eigen/Core>

#include <cstddef>
#include <cstdint>
#include <map>
#include <optional>
#include <vector>


#include "ssl_wheel_tracker/config.hpp"
#include "ssl_wheel_tracker/types.hpp"
#include "ssl_wheel_tracker/visibility_control.hpp"

namespace ssl_wheel_tracker
{

/// Outcome of WheelTracker::addFrame().
enum class FrameResult
{
  /// The frame is buffered for the next update().
  kAccepted,
  /// The frame is not newer than the last processed frame and was dropped.
  kLate,
  /// The frame is older than the last processed frame by more than the deletion timeout (time
  /// jumped backwards); all tracks were dropped and the frame is buffered.
  kReset,
};

/// Result of WheelTracker::update().
struct UpdateResult
{
  /// Number of processed frames; 0 if no frame was buffered.
  std::size_t frames = 0;
  /// Ids of the tracks confirmed by this update.
  std::vector<std::uint64_t> confirmed;
  /// Ids of the confirmed tracks deleted by this update.
  std::vector<std::uint64_t> deleted;
};

/// Counters and state of the tracker for diagnostics.
struct TrackerStatus
{
  std::size_t confirmed_tracks = 0;
  std::size_t tentative_tracks = 0;
  std::uint64_t frames = 0;
  std::uint64_t detections = 0;
  std::uint64_t late_frames = 0;
  std::uint64_t resets = 0;
  std::uint64_t tracks_confirmed = 0;
  std::uint64_t tracks_deleted = 0;
  /// Time of the last processed frame.
  std::optional<Timestamp> last_frame;
};

/**
 * @brief Multi-target tracker for wheel contacts on sensitive surface layers (SSL).
 *
 * Each wheel is tracked with a linear Kalman filter with a constant velocity model per axis.
 * Every processed frame
 * 1. predicts the confirmed tracks to the frame time and associates them with the detections
 *    by global nearest neighbour (assign()); a matched track is updated with its detection,
 *    an unmatched track keeps the prediction,
 * 2. deletes confirmed tracks without associated detection for longer than the deletion
 *    timeout,
 * 3. associates the remaining detections with the tentative tracks in the same way, confirms
 *    tentative tracks with confirmation_hits associated detections and deletes timed out
 *    tentative tracks,
 * 4. starts a tentative track for every detection that is still unassociated.
 *
 * Frames are buffered by addFrame() and processed in time order by update(), so that
 * detections of several mats with the same time are associated together.
 *
 * The class is not thread-safe.
 */
class SSL_WHEEL_TRACKER_PUBLIC WheelTracker
{
public:
  /// @throws std::invalid_argument if the configuration is invalid.
  explicit WheelTracker(const TrackerConfig & config);

  /**
   * @brief Buffers the detections of a frame.
   * @param detections wheel positions in the tracking frame; detections of frames with the same
   *        stamp are merged.
   */
  FrameResult addFrame(const Timestamp & stamp, const std::vector<Point2D> & detections);

  /// Processes all buffered frames in time order.
  UpdateResult update();

  /// Confirmed tracks.
  const std::vector<Track> & tracks() const {return confirmed_;}
  /// Tracks that do not have enough associated detections yet.
  const std::vector<Track> & tentativeTracks() const {return tentative_;}

  const TrackerConfig & config() const {return config_;}
  TrackerStatus status() const;

private:
  void step(
    const Timestamp & stamp, const std::vector<Point2D> & detections,
    UpdateResult & result);
  /// Associates the tracks with the detections that are not yet used and updates them.
  void associate(
    std::vector<Track> & tracks, const Timestamp & stamp,
    const std::vector<Point2D> & detections, std::vector<bool> & used) const;
  void append(Track & track, const TrackState & state) const;
  bool timedOut(const Track & track, const Timestamp & stamp) const;

  TrackerConfig config_;
  std::vector<Track> confirmed_;
  std::vector<Track> tentative_;
  std::map<Timestamp, std::vector<Point2D>> pending_;
  std::optional<Timestamp> last_frame_;
  std::uint64_t next_id_ = 1;
  TrackerStatus status_;
};

/**
 * @brief Global nearest neighbour assignment of tracks to detections.
 *
 * Minimizes the sum of the distances of the assigned pairs plus gate_distance for every track
 * without detection. Pairs at or beyond the gate distance are never assigned.
 *
 * @param distances distance of every track (row) to every detection (column)
 * @return the assigned detection of every track, or std::nullopt
 */
SSL_WHEEL_TRACKER_PUBLIC std::vector<std::optional<std::size_t>> assign(
  const Eigen::MatrixXd & distances, double gate_distance);

/// Predicts the state to the given time with the constant velocity model.
SSL_WHEEL_TRACKER_PUBLIC TrackState predict(
  const TrackState & state, const Timestamp & stamp, double process_noise);

/// Kalman update of a predicted state with a detected position.
SSL_WHEEL_TRACKER_PUBLIC TrackState correct(
  const TrackState & prediction, const Point2D & detection, double measurement_noise);

}  // namespace ssl_wheel_tracker

#endif  // SSL_WHEEL_TRACKER__WHEEL_TRACKER_HPP_
