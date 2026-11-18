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

#ifndef UFIL_OBJECT_TRACKING__PREDICT__OCCLUSION_PREDICTOR_HPP_
#define UFIL_OBJECT_TRACKING__PREDICT__OCCLUSION_PREDICTOR_HPP_

#include <array>
#include <cmath>
#include <iostream>
#include <memory>
#include <optional>
#include <utility>

#include "ufil_object_tracking/predict/predictor.hpp"
#include "ufil_object_tracking/types/occupancy_grid.hpp"

namespace ufil
{
namespace predict
{

// A generic, ROS-unaware occlusion predictor. It flags a track as occluded when
// at least three corners of its (axis-aligned) bounding box fall on occupied
// cells of an occupancy grid. The grid is the ROS-unaware
// ufil::type::OccupancyGrid; converting any sensor/ROS grid into it is the
// responsibility of the calling layer.
template<typename T>
class OcclusionPredictor : public ufil::predict::Predictor<T>
{
public:
  using Base = ufil::predict::Predictor<T>;
  using TrackType = T;
  using ControlType = typename T::ControlType;
  using SharedPtr = std::shared_ptr<OcclusionPredictor>;
  using UniquePtr = std::unique_ptr<OcclusionPredictor>;

  void predict(
    T & track,
    const std::optional<ControlType> & control,
    const ufil::type::Timestamp & /*timestamp*/) override
  {
    this->storeControlInput(track, control);

    double x = track.currentState().position().x();
    double y = track.currentState().position().y();
    double width = track.currentDimension().width();
    double length = track.currentDimension().length();

    bool occ = this->isOccluded(x, y, width, length);

    auto & hist_entry = track.currentHistoryEntry();
    hist_entry.occluded() = occ;
  }

  void setGrid(std::shared_ptr<const ufil::type::OccupancyGrid> grid)
  {
    this->occ_grid_ = grid;
  }

private:
  std::shared_ptr<const ufil::type::OccupancyGrid> occ_grid_;

  bool isOccluded(double x, double y, double box_w, double box_l) const
  {
    if (!this->occ_grid_ || this->occ_grid_->data().empty()) {
      std::cout << "Grid not defined in occ predictor" << std::endl;
      return false;
    }
    const double ox = occ_grid_->originX() - 0.2;
    const double oy = occ_grid_->originY() + 0.2;
    const double res = occ_grid_->resolution();
    const int width = occ_grid_->width();
    const int height = occ_grid_->height();
    const auto & data = occ_grid_->data();

    auto worldToGrid = [&](double wx, double wy, int & ix, int & iy)
      {
        ix = static_cast<int>(std::round((wx - ox) / res));
        iy = static_cast<int>(std::round((wy - oy) / res));
      };

    std::array<std::pair<double, double>, 4> corners = {{
      {x + 0.5 * box_w, y + 0.5 * box_l},
      {x + 0.5 * box_w, y - 0.5 * box_l},
      {x - 0.5 * box_w, y + 0.5 * box_l},
      {x - 0.5 * box_w, y - 0.5 * box_l},
    }};

    int occl_count = 0;
    for (auto & c : corners) {
      int ix, iy;
      worldToGrid(c.first, c.second, ix, iy);

      // check only the base cell
      if (ix >= 0 && ix < width && iy >= 0 && iy < height) {
        int idx = iy * width + ix;
        if (data[idx] > 50) {
          if (++occl_count >= 3) {
            return true;
          }
        }
      }
    }

    return false;
  }
};

}  // namespace predict
}  // namespace ufil

#endif  // UFIL_OBJECT_TRACKING__PREDICT__OCCLUSION_PREDICTOR_HPP_
