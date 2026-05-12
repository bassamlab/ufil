// Copyright 2025 Chair of Embedded Software (Computer Science 11) - RWTH Aachen University
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

#include <iostream>
#include <algorithm>
#include <boost/uuid/uuid_io.hpp>
#include <boost/math/distributions/chi_squared.hpp>

#include "ufil_osn_tracker/occlusion_predictor.hpp"


namespace ufil_osn_tracker
{
void OcclusionPredictor::predict(
  Track & track,
  const std::optional<ControlType> & control,
  const ufil::type::Timestamp & /*timestamp*/)
{
  this->storeControlInput(track, control);

  double x = track.currentState().position().x();
  double y = track.currentState().position().y();
  double width = track.currentDimension().width();
  double length = track.currentDimension().length();

  bool occ = this->isOccluded(x, y, width, length);

  // if (occ) {
  //   std::cout << "Track is occluded."
  //             << "Track x= " << x
  //             << " Track y= " << y
  //             << " History size=" << track.history().size() << std::endl;
  // }

  auto & hist_entry = track.currentHistoryEntry();
  hist_entry.occluded() = occ;
}

bool OcclusionPredictor::isOccluded(double x, double y, double box_w, double box_l) const
{
  if (!this->occ_grid_ || this->occ_grid_->data.empty()) {
    std::cout << "Grid not defined in occ predictor" << std::endl;
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

void OcclusionPredictor::setGrid(std::shared_ptr<const OccupancyGrid> grid)
{
  this->occ_grid_ = grid;
}
}   // namespace ufil_osn_tracker
