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

#include "ufil_osc/config.hpp"

using ufil_osc::OscConfig;
using ufil_osc::validate;

TEST(Config, DefaultsAreValid)
{
  EXPECT_FALSE(validate(OscConfig{}));
}

TEST(Config, RejectsInvalidValues)
{
  const auto invalid = [](auto modify) {
      OscConfig config;
      modify(config);
      return validate(config).has_value();
    };
  EXPECT_TRUE(invalid([](OscConfig & c) {c.host = "";}));
  EXPECT_TRUE(invalid([](OscConfig & c) {c.port = 0;}));
  EXPECT_TRUE(invalid([](OscConfig & c) {c.port = 65536;}));
  EXPECT_TRUE(invalid([](OscConfig & c) {c.address = "";}));
  EXPECT_TRUE(invalid([](OscConfig & c) {c.address = "/";}));
  EXPECT_TRUE(invalid([](OscConfig & c) {c.address = "objects";}));
  EXPECT_TRUE(invalid([](OscConfig & c) {c.address = "/obj ects";}));
  EXPECT_TRUE(invalid([](OscConfig & c) {c.address = "/objects/*";}));
  EXPECT_TRUE(invalid([](OscConfig & c) {c.address = "/objects,s";}));

  EXPECT_FALSE(invalid([](OscConfig & c) {c.host = "localhost";}));
  EXPECT_FALSE(invalid([](OscConfig & c) {c.port = 65535;}));
  EXPECT_FALSE(invalid([](OscConfig & c) {c.address = "/ssl1";}));
  EXPECT_FALSE(invalid([](OscConfig & c) {c.address = "/lab/objects_1";}));
}
