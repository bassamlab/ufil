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

#include <gtest/gtest.h>

#include <algorithm>
#include <cstddef>
#include <filesystem>
#include <fstream>
#include <stdexcept>
#include <string>
#include <vector>

#include "ufil_visualization/lanelet_map_loader.hpp"
#include "ufil_visualization/lanelet_map_markers.hpp"

using ufil_visualization::LaneletMapStyle;
using ufil_visualization::Marker;
using ufil_visualization::MarkerType;
using ufil_visualization::OriginSource;

namespace
{

const Marker * find(const std::vector<Marker> & markers, const std::string & ns)
{
  const auto it = std::find_if(
    markers.begin(), markers.end(), [&ns](const Marker & marker) {return marker.ns == ns;});
  return it == markers.end() ? nullptr : &*it;
}

std::size_t count(const std::vector<Marker> & markers, const std::string & ns)
{
  return static_cast<std::size_t>(std::count_if(
           markers.begin(), markers.end(),
           [&ns](const Marker & marker) {return marker.ns == ns;}));
}

/// Writes an OSM file with the given content into the temporary directory.
std::string writeOsm(const std::string & name, const std::string & content)
{
  const auto path = std::filesystem::temp_directory_path() / name;
  std::ofstream(path) << content;
  return path.string();
}

class LaneletMapMarkersTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite() {map_ = ufil_visualization::loadLaneletMap(TEST_MAP);}

  static ufil_visualization::LoadedLaneletMap map_;
};

ufil_visualization::LoadedLaneletMap LaneletMapMarkersTest::map_;

}  // namespace

TEST_F(LaneletMapMarkersTest, LoadsMapAroundOriginNode)
{
  ASSERT_NE(map_.map, nullptr);
  EXPECT_TRUE(map_.errors.empty());
  EXPECT_EQ(map_.origin.source, OriginSource::kOriginNode);
  EXPECT_DOUBLE_EQ(map_.origin.origin.latitude, 50.0);
  EXPECT_DOUBLE_EQ(map_.origin.origin.longitude, 9.0);
  EXPECT_EQ(map_.map->laneletLayer.size(), 3u);
  EXPECT_EQ(map_.map->areaLayer.size(), 1u);

  // The local coordinates are relative to the origin.
  const auto point = map_.map->pointLayer.get(13);
  EXPECT_NEAR(point.x(), 20.0, 0.1);
  EXPECT_NEAR(point.y(), 3.5, 0.1);
}

TEST_F(LaneletMapMarkersTest, BuildsMarkersOfAllPrimitives)
{
  const LaneletMapStyle style;
  const auto markers = ufil_visualization::laneletMapMarkers(*map_.map, style);

  // Two road lanelets with four boundary points each: two triangles each.
  const Marker * fill = find(markers, "lanelet_fill");
  ASSERT_NE(fill, nullptr);
  EXPECT_EQ(fill->type, MarkerType::kTriangleList);
  EXPECT_EQ(fill->points.size(), 2u * 2u * 3u);
  EXPECT_DOUBLE_EQ(fill->points.front().z, style.lanelet_fill_z);
  EXPECT_DOUBLE_EQ(fill->scale.x, 1.0);
  EXPECT_TRUE(fill->frame_locked);

  ASSERT_NE(find(markers, "crosswalk_fill"), nullptr);
  EXPECT_EQ(find(markers, "crosswalk_fill")->points.size(), 2u * 3u);

  // Crosswalk of 6 m with 0.5 m stripes and gaps: six stripes of two triangles.
  ASSERT_NE(find(markers, "crosswalk_stripes"), nullptr);
  EXPECT_EQ(find(markers, "crosswalk_stripes")->points.size(), 6u * 2u * 3u);

  // Three lanelets with two single-segment bounds each.
  const Marker * border = find(markers, "lanelet_border");
  ASSERT_NE(border, nullptr);
  EXPECT_EQ(border->type, MarkerType::kLineList);
  EXPECT_EQ(border->points.size(), 3u * 2u * 2u);
  EXPECT_DOUBLE_EQ(border->scale.x, style.line_width);

  // One separator between the two consecutive road lanelets at x = 10 m.
  const Marker * separator = find(markers, "lanelet_separator");
  ASSERT_NE(separator, nullptr);
  ASSERT_EQ(separator->points.size(), 2u);
  EXPECT_NEAR(separator->points[0].x, 10.0, 0.1);
  EXPECT_NEAR(separator->points[1].x, 10.0, 0.1);

  // The two solid right bounds; the dashed lines are not drawn.
  ASSERT_NE(find(markers, "road_marking"), nullptr);
  EXPECT_EQ(find(markers, "road_marking")->points.size(), 2u * 2u);
  const Marker * stop_line = find(markers, "stop_line");
  ASSERT_NE(stop_line, nullptr);
  EXPECT_EQ(stop_line->points.size(), 2u);
  EXPECT_DOUBLE_EQ(stop_line->scale.x, style.stop_line_width);

  EXPECT_EQ(count(markers, "lanelet_id"), 3u);

  const Marker * vegetation = find(markers, "area_vegetation");
  ASSERT_NE(vegetation, nullptr);
  EXPECT_EQ(vegetation->points.size(), 2u * 3u);
  EXPECT_FLOAT_EQ(vegetation->color.g, style.area_vegetation_color.g);

  // Marker ids are unique.
  std::vector<int> ids;
  for (const auto & marker : markers) {
    ids.push_back(marker.id);
  }
  std::sort(ids.begin(), ids.end());
  EXPECT_EQ(std::adjacent_find(ids.begin(), ids.end()), ids.end());
}

TEST_F(LaneletMapMarkersTest, LaneletIdsCanBeHidden)
{
  LaneletMapStyle style;
  style.show_lanelet_ids = false;
  const auto markers = ufil_visualization::laneletMapMarkers(*map_.map, style);
  EXPECT_EQ(count(markers, "lanelet_id"), 0u);
}

TEST(LaneletMapStyleTest, Validation)
{
  EXPECT_FALSE(ufil_visualization::validate(LaneletMapStyle{}));

  LaneletMapStyle style;
  style.line_width = 0.0;
  EXPECT_TRUE(ufil_visualization::validate(style));

  style = LaneletMapStyle{};
  style.crosswalk_stripe_gap = -0.1;
  EXPECT_TRUE(ufil_visualization::validate(style));

  style = LaneletMapStyle{};
  style.area_traffic_island_color.r = 1.1F;
  EXPECT_TRUE(ufil_visualization::validate(style));
}

TEST(LaneletMapStyleTest, AreaColorBySubtype)
{
  const LaneletMapStyle style;
  EXPECT_FLOAT_EQ(
    ufil_visualization::areaColor("parking", style).r, style.area_parking_color.r);
  EXPECT_FLOAT_EQ(
    ufil_visualization::areaColor("unknown", style).r, style.area_default_color.r);
}

TEST(LaneletMapLoaderTest, OriginFromBoundsCentre)
{
  const auto path = writeOsm(
    "ufil_visualization_bounds.osm",
    "<osm version=\"0.6\"><bounds minlat=\"49.0\" minlon=\"8.0\" maxlat=\"51.0\" "
    "maxlon=\"10.0\"/></osm>");
  const auto origin = ufil_visualization::findMapOrigin(path);
  EXPECT_EQ(origin.source, OriginSource::kBoundsCentre);
  EXPECT_DOUBLE_EQ(origin.origin.latitude, 50.0);
  EXPECT_DOUBLE_EQ(origin.origin.longitude, 9.0);
}

TEST(LaneletMapLoaderTest, DefaultOriginWithoutOriginNodeAndBounds)
{
  const auto path = writeOsm("ufil_visualization_empty.osm", "<osm version=\"0.6\"/>");
  EXPECT_EQ(ufil_visualization::findMapOrigin(path).source, OriginSource::kDefault);
}

TEST(LaneletMapLoaderTest, MissingOrInvalidFileThrows)
{
  EXPECT_THROW(
    ufil_visualization::loadLaneletMap("/nonexistent/ufil_visualization.osm"),
    std::runtime_error);
  const auto path = writeOsm("ufil_visualization_invalid.osm", "<html></html>");
  EXPECT_THROW(ufil_visualization::loadLaneletMap(path), std::runtime_error);
}
