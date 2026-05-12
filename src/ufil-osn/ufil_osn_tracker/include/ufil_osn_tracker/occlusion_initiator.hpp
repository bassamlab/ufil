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

#ifndef UFIL_OSN_TRACKER__OCCLUSION_INITIATOR_HPP_
#define UFIL_OSN_TRACKER__OCCLUSION_INITIATOR_HPP_

#include <functional>
#include <map>
#include <memory>
#include <set>
#include <iomanip>
#include <utility>

#include <nav_msgs/msg/occupancy_grid.hpp>

#include <ufil_object_tracking/management/initiator/initiator.hpp>
#include <ufil_object_tracking/types/history_entry.hpp>

#include "ufil_osn_tracker/definitions.hpp"

namespace ufil_osn_tracker
{
class OcclusionInitiator : public ufil::management::Initiator<Track>
{
public:
  using TrackType = Track;
  using MeasurementType = Track::MeasurementType;
  using StateType = Track::StateType;
  using DimensionType = Track::DimensionType;
  using ClassificationType = Track::ClassificationType;
  using ExistenceProbabilityType = ufil_osn_tracker::ExistenceProbability;
  using HistoryEntryType = Track::HistoryEntryType;
  using OccupancyGrid = nav_msgs::msg::OccupancyGrid;
  using SharedPtr = std::shared_ptr<OcclusionInitiator>;
  using UniquePtr = std::unique_ptr<OcclusionInitiator>;
  using InitFn = std::function<
    void (const MeasurementType &, StateType &, DimensionType &,
    ClassificationType &, ExistenceProbabilityType &)>;
  explicit OcclusionInitiator(InitFn init_fn)
  : init_fn_(std::move(init_fn))
  {
  }
  void setGrid(std::shared_ptr<const OccupancyGrid> grid)
  {
    this->occ_grid_ = grid;
  }
  void initiate(
    const std::set<MeasurementType> & measurements,
    const ufil::type::Timestamp & timestamp,
    std::map<ufil::type::Id, TrackType> & new_tracks) override
  {
    for (const auto & measurement : measurements) {
      if (!isOccluded(
          measurement.position().x(), measurement.position().y(),
          measurement.dimension().length(), measurement.dimension().width()))
      {
        // Create state
        StateType state(timestamp);
        DimensionType dimension;
        ClassificationType classification;
        ExistenceProbability existence_probability;
        this->init_fn_(
          measurement, state, dimension,
          classification, existence_probability);
        // Create track
        TrackType track;
        track.insert(
          HistoryEntryType(
            state, dimension, existence_probability,
            classification, std::nullopt, std::make_optional(measurement)));
        new_tracks.emplace(track.uuid(), std::move(track));
      }
    }
  }

private:
  bool isOccluded(double x, double y, double dim_w, double dim_l) const
  {
    if (!this->occ_grid_ || this->occ_grid_->data.empty()) {
      return false;
    }
    const double ox = occ_grid_->info.origin.position.x - 0.2;
    const double oy = occ_grid_->info.origin.position.y + 0.2;
    const double res = occ_grid_->info.resolution;
    const int width = occ_grid_->info.width;
    const int height = occ_grid_->info.height;
    const auto & data = occ_grid_->data;
    auto worldToGrid = [&](double wx, double wy, int & ix, int & iy)
      {
        ix = static_cast<int>(std::round((wx - ox) / res));
        iy = static_cast<int>(std::round((wy - oy) / res));
      };
    std::array<std::pair<double, double>, 5> corners = {{
      {x + 0.5 * dim_w, y + 0.5 * dim_l},
      {x + 0.5 * dim_w, y - 0.5 * dim_l},
      {x - 0.5 * dim_w, y + 0.5 * dim_l},
      {x - 0.5 * dim_w, y - 0.5 * dim_l},
      {x, y}
    }};
    int occl_count = 0;
    for (auto & c : corners) {
      int ix, iy;
      worldToGrid(c.first, c.second, ix, iy);
      // check only the base cell
      if (ix >= 0 && ix < width && iy >= 0 && iy < height) {
        int idx = iy * width + ix;
        if (data[idx] > 35) {
          if (++occl_count >= 3) {
            return true;
          }
        }
      }
    }
    return false;
  }
  std::shared_ptr<const OccupancyGrid> occ_grid_;
  InitFn init_fn_;
};
}  // namespace ufil_osn_tracker

#endif  // UFIL_OSN_TRACKER__OCCLUSION_INITIATOR_HPP_
