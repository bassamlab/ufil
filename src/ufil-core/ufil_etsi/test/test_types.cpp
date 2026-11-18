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

#include "ufil_etsi/types.hpp"

// Core library tests, no ROS 2 required.

using ufil_etsi::UtmZone;

TEST(Types, UtmZoneFromFrame)
{
  EXPECT_EQ(ufil_etsi::utmZoneFromFrame("utm_32n"), (UtmZone{32, true}));
  EXPECT_EQ(ufil_etsi::utmZoneFromFrame("utm_1S"), (UtmZone{1, false}));
  EXPECT_EQ(ufil_etsi::utmZoneFromFrame("utm_60n"), (UtmZone{60, true}));
  EXPECT_FALSE(ufil_etsi::utmZoneFromFrame("map").has_value());
  EXPECT_FALSE(ufil_etsi::utmZoneFromFrame("utm_32").has_value());
  EXPECT_FALSE(ufil_etsi::utmZoneFromFrame("utm_61n").has_value());
  EXPECT_FALSE(ufil_etsi::utmZoneFromFrame("utm_0n").has_value());
  EXPECT_FALSE(ufil_etsi::utmZoneFromFrame("utm_3xn").has_value());
  EXPECT_EQ(ufil_etsi::frameFromUtmZone(UtmZone{32, true}), "utm_32n");
  EXPECT_EQ(ufil_etsi::frameFromUtmZone(UtmZone{7, false}), "utm_7s");
}

TEST(Types, GeoAndUtmConversions)
{
  const auto utm = ufil_etsi::geoToUtm({48.079772, 11.635735});
  ASSERT_TRUE(utm.has_value());
  EXPECT_EQ(utm->zone, (UtmZone{32, true}));
  const auto geo = ufil_etsi::utmToGeo(utm->easting, utm->northing, utm->zone);
  ASSERT_TRUE(geo.has_value());
  EXPECT_NEAR(geo->latitude, 48.079772, 1e-9);
  EXPECT_NEAR(geo->longitude, 11.635735, 1e-9);

  EXPECT_FALSE(ufil_etsi::geoToUtm({91.0, 0.0}).has_value());
  EXPECT_FALSE(ufil_etsi::geoToUtm({89.5, 0.0}).has_value());  // UPS
  EXPECT_FALSE(ufil_etsi::utmToGeo(1e9, 0.0, UtmZone{32, true}).has_value());
}

TEST(Types, MostLikelyClass)
{
  ufil_etsi::Object object;
  EXPECT_EQ(ufil_etsi::mostLikelyClass(object), ufil_etsi::ObjectClass::kOther);
  object.classification[1] = 0.6;
  object.classification[0] = 0.4;
  EXPECT_EQ(ufil_etsi::mostLikelyClass(object), ufil_etsi::ObjectClass::kTruck);
}
