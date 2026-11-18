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

#include <string>

#include "ufil_examples_cpm_lab_adapter/lane_control_light.hpp"

using ufil_examples_cpm_lab_adapter::LaneControlLight;
using ufil_examples_cpm_lab_adapter::toJson;

TEST(LaneControlLightTest, EncodesJson)
{
  LaneControlLight lights;
  lights.stamp = 12.5;
  lights.frame_id = "map";
  lights.segments = {{0, "lane-closed-black"}, {1, ""}, {2, "282-black"}};
  EXPECT_EQ(
    toJson(lights),
    R"({"lane_control_light":{"header":{"seconds":12.500000,"frame_id":"map"},"segments":[)"
    R"({"segment_index":0,"image_name":"lane-closed-black"},)"
    R"({"segment_index":1,"image_name":"-"},)"
    R"({"segment_index":2,"image_name":"282-black"}]}})");
}

TEST(LaneControlLightTest, EncodesEmptyState)
{
  EXPECT_EQ(
    toJson(LaneControlLight{}),
    R"({"lane_control_light":{"header":{"seconds":0.000000,"frame_id":""},"segments":[]}})");
}

TEST(LaneControlLightTest, EscapesStrings)
{
  LaneControlLight lights;
  lights.frame_id = std::string("a\"b\\c\n") + '\0' + '\x01';
  lights.segments = {{-1, "\t"}};
  EXPECT_EQ(
    toJson(lights),
    R"({"lane_control_light":{"header":{"seconds":0.000000,"frame_id":"a\"b\\c\n\u0000\u0001"},)"
    R"("segments":[{"segment_index":-1,"image_name":"\t"}]}})");
}
