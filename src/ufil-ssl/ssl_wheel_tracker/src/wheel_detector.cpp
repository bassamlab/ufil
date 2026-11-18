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

#include "ssl_wheel_tracker/wheel_detector.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <limits>
#include <stdexcept>
#include <vector>

namespace ssl_wheel_tracker
{

namespace
{

/// Maximum of the square window of the given radius around every cell; the grid is extended
/// with its border values.
std::vector<std::uint8_t> maximumFilter(const PressureGrid & grid, std::size_t radius)
{
  const std::size_t columns = grid.columns;
  const std::size_t rows = grid.rows;
  std::vector<std::uint8_t> along_columns(grid.values.size());
  for (std::size_t row = 0; row < rows; ++row) {
    for (std::size_t column = 0; column < columns; ++column) {
      const std::size_t first = column > radius ? column - radius : 0;
      const std::size_t last = std::min(column + radius, columns - 1);
      std::uint8_t maximum = 0;
      for (std::size_t c = first; c <= last; ++c) {
        maximum = std::max(maximum, grid.at(c, row));
      }
      along_columns[row * columns + column] = maximum;
    }
  }

  std::vector<std::uint8_t> result(grid.values.size());
  for (std::size_t row = 0; row < rows; ++row) {
    const std::size_t first = row > radius ? row - radius : 0;
    const std::size_t last = std::min(row + radius, rows - 1);
    for (std::size_t column = 0; column < columns; ++column) {
      std::uint8_t maximum = 0;
      for (std::size_t r = first; r <= last; ++r) {
        maximum = std::max(maximum, along_columns[r * columns + column]);
      }
      result[row * columns + column] = maximum;
    }
  }
  return result;
}

std::size_t chebyshevDistance(const Cell & a, const Cell & b)
{
  const auto difference = [](std::size_t x, std::size_t y) {return x > y ? x - y : y - x;};
  return std::max(difference(a.column, b.column), difference(a.row, b.row));
}

/// Cell of a local maximum with the label of its 4-connected component of maxima.
struct PeakCell
{
  Cell cell;
  std::size_t label = 0;
};

/// Labels the 4-connected components of the peak cells. Labels start at 1 and are assigned in
/// column-major order of the first cell of each component.
std::vector<PeakCell> labelPeaks(const PressureGrid & grid, const std::vector<Cell> & peaks)
{
  const std::size_t columns = grid.columns;
  const std::size_t rows = grid.rows;
  // 0 = no peak, kUnlabeled = peak without label yet.
  constexpr std::size_t kUnlabeled = std::numeric_limits<std::size_t>::max();
  std::vector<std::size_t> labels(grid.values.size(), 0);
  for (const auto & peak : peaks) {
    labels[peak.row * columns + peak.column] = kUnlabeled;
  }

  std::vector<PeakCell> result;
  std::size_t next_label = 1;
  std::vector<Cell> stack;
  for (std::size_t column = 0; column < columns; ++column) {
    for (std::size_t row = 0; row < rows; ++row) {
      if (labels[row * columns + column] != kUnlabeled) {
        continue;
      }
      const std::size_t label = next_label++;
      labels[row * columns + column] = label;
      stack.push_back({column, row});
      while (!stack.empty()) {
        const Cell cell = stack.back();
        stack.pop_back();
        const auto visit = [&](std::size_t c, std::size_t r) {
            if (labels[r * columns + c] == kUnlabeled) {
              labels[r * columns + c] = label;
              stack.push_back({c, r});
            }
          };
        if (cell.column > 0) {visit(cell.column - 1, cell.row);}
        if (cell.column + 1 < columns) {visit(cell.column + 1, cell.row);}
        if (cell.row > 0) {visit(cell.column, cell.row - 1);}
        if (cell.row + 1 < rows) {visit(cell.column, cell.row + 1);}
      }
    }
  }

  // Collect in column-major order, which makes the nearest-peak search deterministic.
  for (std::size_t column = 0; column < columns; ++column) {
    for (std::size_t row = 0; row < rows; ++row) {
      const std::size_t label = labels[row * columns + column];
      if (label != 0) {
        result.push_back({{column, row}, label});
      }
    }
  }
  return result;
}

}  // namespace

WheelDetector::WheelDetector(const DetectorConfig & config)
: config_(config)
{
  if (const auto error = validate(config_)) {
    throw std::invalid_argument("invalid detector configuration: " + *error);
  }
}

std::vector<Cell> findPeaks(const PressureGrid & grid, const DetectorConfig & config)
{
  if (grid.values.empty()) {
    return {};
  }
  const auto maxima = maximumFilter(grid, config.min_peak_distance);
  // A grid without any variation (e.g. all zero) has no peaks.
  if (std::equal(grid.values.begin(), grid.values.end(), maxima.begin())) {
    return {};
  }

  const std::size_t border = config.exclude_border;
  std::vector<Cell> candidates;
  for (std::size_t column = border; column + border < grid.columns; ++column) {
    for (std::size_t row = border; row + border < grid.rows; ++row) {
      const std::uint8_t value = grid.at(column, row);
      if (value > config.threshold && value == maxima[row * grid.columns + column]) {
        candidates.push_back({column, row});
      }
    }
  }
  std::stable_sort(
    candidates.begin(), candidates.end(),
    [&grid](const Cell & a, const Cell & b) {return grid.at(a) > grid.at(b);});

  // Maxima of a plateau closer than min_peak_distance to a stronger (or earlier) maximum are
  // dropped.
  std::vector<bool> rejected(candidates.size(), false);
  std::vector<Cell> peaks;
  for (std::size_t i = 0; i < candidates.size(); ++i) {
    if (rejected[i]) {
      continue;
    }
    peaks.push_back(candidates[i]);
    for (std::size_t j = i + 1; j < candidates.size(); ++j) {
      if (chebyshevDistance(candidates[i], candidates[j]) < config.min_peak_distance) {
        rejected[j] = true;
      }
    }
  }
  return peaks;
}

std::vector<WheelDetection> WheelDetector::detect(const PressureGrid & grid) const
{
  const auto peaks = findPeaks(grid, config_);
  if (peaks.empty()) {
    return {};
  }
  const auto peak_cells = labelPeaks(grid, peaks);
  std::size_t wheel_count = 0;
  for (const auto & peak : peak_cells) {
    wheel_count = std::max(wheel_count, peak.label);
  }

  // Cells within the contact radius (L1) of any peak cell belong to a wheel.
  const std::size_t columns = grid.columns;
  const std::size_t rows = grid.rows;
  const auto radius = static_cast<std::ptrdiff_t>(config_.contact_radius);
  std::vector<bool> contact(grid.values.size(), false);
  for (const auto & peak : peak_cells) {
    for (std::ptrdiff_t dc = -radius; dc <= radius; ++dc) {
      const std::ptrdiff_t remaining = radius - std::abs(dc);
      for (std::ptrdiff_t dr = -remaining; dr <= remaining; ++dr) {
        const std::ptrdiff_t column = static_cast<std::ptrdiff_t>(peak.cell.column) + dc;
        const std::ptrdiff_t row = static_cast<std::ptrdiff_t>(peak.cell.row) + dr;
        if (column >= 0 && row >= 0 && column < static_cast<std::ptrdiff_t>(columns) &&
          row < static_cast<std::ptrdiff_t>(rows))
        {
          contact[static_cast<std::size_t>(row) * columns +
            static_cast<std::size_t>(column)] = true;
        }
      }
    }
  }

  struct Accumulator
  {
    double weight = 0.0;
    double weighted_column = 0.0;
    double weighted_row = 0.0;
    double pressure = 0.0;
    std::size_t cells = 0;
  };
  std::vector<Accumulator> wheels(wheel_count + 1);  // index 0 = background
  for (std::size_t column = 0; column < columns; ++column) {
    for (std::size_t row = 0; row < rows; ++row) {
      std::size_t label = 0;
      if (contact[row * columns + column]) {
        // The nearest peak cell owns the cell; ties go to the first peak cell (column-major).
        double best = std::numeric_limits<double>::infinity();
        for (const auto & peak : peak_cells) {
          const double dc = static_cast<double>(peak.cell.column) - static_cast<double>(column);
          const double dr = static_cast<double>(peak.cell.row) - static_cast<double>(row);
          const double distance = dc * dc + dr * dr;
          if (distance < best) {
            best = distance;
            label = peak.label;
          }
        }
      }
      const double value = grid.at(column, row);
      auto & wheel = wheels[label];
      wheel.weight += value;
      wheel.weighted_column += value * static_cast<double>(column);
      wheel.weighted_row += value * static_cast<double>(row);
      wheel.pressure += value;
      ++wheel.cells;
    }
  }

  const auto & background = wheels.front();
  const double noise_level =
    background.cells > 0 ? background.pressure / static_cast<double>(background.cells) : 0.0;
  const double cos_yaw = std::cos(grid.origin.yaw);
  const double sin_yaw = std::sin(grid.origin.yaw);

  std::vector<WheelDetection> detections;
  detections.reserve(wheel_count);
  for (std::size_t label = 1; label <= wheel_count; ++label) {
    const auto & wheel = wheels[label];
    // Every wheel contains a peak above the threshold (>= 0), so its weight is positive.
    const double x = (wheel.weighted_column / wheel.weight + 0.5) * grid.resolution;
    const double y = (wheel.weighted_row / wheel.weight + 0.5) * grid.resolution;
    WheelDetection detection;
    detection.position.x = grid.origin.x + cos_yaw * x - sin_yaw * y;
    detection.position.y = grid.origin.y + sin_yaw * x + cos_yaw * y;
    detection.pressure = wheel.pressure - noise_level * static_cast<double>(wheel.cells);
    detection.cells = wheel.cells;
    detections.push_back(detection);
  }
  return detections;
}

}  // namespace ssl_wheel_tracker
