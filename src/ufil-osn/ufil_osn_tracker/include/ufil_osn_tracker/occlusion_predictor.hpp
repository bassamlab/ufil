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

#ifndef UFIL_OSN_TRACKER__OCCLUSION_PREDICTOR_HPP_
#define UFIL_OSN_TRACKER__OCCLUSION_PREDICTOR_HPP_

#include <memory>
#include <optional>
#include <array>
#include <utility>
#include <cmath>

#include <ufil_osn_tracker/definitions.hpp>
#include <ufil_object_tracking/predict/predictor.hpp>

#include <nav_msgs/msg/occupancy_grid.hpp>

namespace ufil_osn_tracker
{
class OcclusionPredictor : public ufil::predict::Predictor<Track>
{
public:
  using Base = ufil::predict::Predictor<Track>;
  using TrackType = Track;
  using ControlType = typename Track::ControlType;
  using OccupancyGrid = nav_msgs::msg::OccupancyGrid;
  using SharedPtr = std::shared_ptr<OcclusionPredictor>;
  using UniquePtr = std::unique_ptr<OcclusionPredictor>;

  void predict(
    TrackType & track,
    const std::optional<ControlType> & control,
    const ufil::type::Timestamp & timestamp) override;
  void setGrid(std::shared_ptr<const OccupancyGrid> grid);

private:
  std::shared_ptr<const OccupancyGrid> occ_grid_;
  bool isOccluded(double x, double y, double box_w, double box_h) const;
};
}  // namespace ufil_osn_tracker

#endif  // UFIL_OSN_TRACKER__OCCLUSION_PREDICTOR_HPP_
