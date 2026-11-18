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

#ifndef UFIL_MESSAGE_FILTER__POSITION_FILTER_HPP_
#define UFIL_MESSAGE_FILTER__POSITION_FILTER_HPP_

#include <cstddef>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "ufil_message_filter/visibility_control.hpp"

namespace ufil_message_filter
{

/// 2D position of an object [m].
struct Position
{
  double x = 0.0;
  double y = 0.0;
};

/// Configuration of the PositionFilter.
struct PositionFilterConfig
{
  /**
   * Vertices of the area of interest polygon in the frame of the filtered objects, in either
   * orientation and without repeating the first vertex. An empty area disables the filter: every
   * object passes.
   */
  std::vector<Position> area_of_interest;
};

/// @return a description of the first problem, or std::nullopt if the configuration is valid.
UFIL_MESSAGE_FILTER_PUBLIC
std::optional<std::string> validate(const PositionFilterConfig & config);

/// Counters of the PositionFilter since its construction.
struct FilterStatistics
{
  /// Filtered object lists.
  std::uint64_t lists = 0;
  /// Object lists without any object inside the area of interest.
  std::uint64_t empty_lists = 0;
  /// Objects of all filtered lists.
  std::uint64_t objects = 0;
  /// Objects inside the area of interest.
  std::uint64_t passed_objects = 0;
  /// Objects outside the area of interest (or on its boundary).
  std::uint64_t rejected_objects = 0;
};

/**
 * @brief Keeps the objects whose position lies strictly inside the area of interest.
 *
 * Positions on the boundary of the area are outside.
 */
class UFIL_MESSAGE_FILTER_PUBLIC PositionFilter
{
public:
  /// @throws std::invalid_argument if the configuration is invalid, see validate().
  explicit PositionFilter(PositionFilterConfig config);
  ~PositionFilter();
  PositionFilter(PositionFilter &&) noexcept;
  PositionFilter & operator=(PositionFilter &&) noexcept;
  PositionFilter(const PositionFilter &) = delete;
  PositionFilter & operator=(const PositionFilter &) = delete;

  /// @return false if the area of interest is empty and every object passes.
  bool enabled() const;

  /// @return true if the position passes the filter.
  bool contains(const Position & position) const;

  /**
   * @brief Filters the positions of the objects of one object list and updates the statistics.
   * @return the indices of the passing positions, in ascending order.
   */
  std::vector<std::size_t> filter(const std::vector<Position> & positions);

  const PositionFilterConfig & config() const;
  const FilterStatistics & statistics() const;

private:
  struct Area;

  PositionFilterConfig config_;
  std::unique_ptr<Area> area_;
  FilterStatistics statistics_;
};

}  // namespace ufil_message_filter

#endif  // UFIL_MESSAGE_FILTER__POSITION_FILTER_HPP_
