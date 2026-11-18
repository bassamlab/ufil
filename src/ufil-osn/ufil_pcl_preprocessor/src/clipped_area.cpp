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

#include "ufil_pcl_preprocessor/clipped_area.hpp"

#include <cmath>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace ufil_pcl_preprocessor
{

namespace
{

constexpr std::size_t kColumnCount = 7;

std::string trim(const std::string & value)
{
  const auto begin = value.find_first_not_of(" \t\r");
  if (begin == std::string::npos) {
    return "";
  }
  const auto end = value.find_last_not_of(" \t\r");
  return value.substr(begin, end - begin + 1);
}

bool parseRow(const std::string & line, std::vector<float> & values)
{
  values.clear();
  std::stringstream stream(line);
  std::string cell;
  while (std::getline(stream, cell, ',')) {
    const std::string trimmed = trim(cell);
    std::size_t consumed = 0;
    try {
      values.push_back(std::stof(trimmed, &consumed));
    } catch (const std::exception &) {
      return false;
    }
    if (consumed != trimmed.size() || !std::isfinite(values.back())) {
      return false;
    }
  }
  return true;
}

}  // namespace

bool ClippedArea::contains(float px, float py, float pz) const
{
  const float dx = px - x;
  const float dy = py - y;
  const float dz = pz - z;

  const float c = std::cos(yaw);
  const float s = std::sin(yaw);
  const float local_x = c * dx + s * dy;
  const float local_y = -s * dx + c * dy;

  return std::abs(local_x) <= length * 0.5f &&
         std::abs(local_y) <= width * 0.5f &&
         std::abs(dz) <= height * 0.5f;
}

std::vector<ClippedArea> parseClippedAreasCsv(std::istream & input)
{
  std::vector<ClippedArea> areas;
  std::vector<float> values;
  std::string line;
  std::size_t line_number = 0;
  bool first_content_line = true;

  while (std::getline(input, line)) {
    ++line_number;
    const std::string trimmed = trim(line);
    if (trimmed.empty() || trimmed.front() == '#') {
      continue;
    }

    const bool numeric = parseRow(trimmed, values);
    if (first_content_line) {
      first_content_line = false;
      if (!numeric) {
        continue;  // header
      }
    }
    if (!numeric || values.size() != kColumnCount) {
      throw std::runtime_error(
              "line " + std::to_string(line_number) + ": expected " +
              std::to_string(kColumnCount) +
              " numeric values (x, y, z, yaw, length, width, height)");
    }
    areas.push_back(
      ClippedArea{values[0], values[1], values[2], values[3], values[4], values[5], values[6]});
  }
  return areas;
}

std::vector<ClippedArea> loadClippedAreasFromCsv(const std::string & path)
{
  std::ifstream file(path);
  if (!file.is_open()) {
    throw std::runtime_error("cannot open clipped area file '" + path + "'");
  }
  try {
    return parseClippedAreasCsv(file);
  } catch (const std::runtime_error & error) {
    throw std::runtime_error("invalid clipped area file '" + path + "', " + error.what());
  }
}

}  // namespace ufil_pcl_preprocessor
