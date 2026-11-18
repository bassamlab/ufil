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
#include <vector>

#include "ufil_osc/osc_message.hpp"

using ufil_osc::encodeStringMessage;
using ufil_osc::stringMessageSize;

namespace
{

std::vector<char> bytes(const std::string & value) {return {value.begin(), value.end()};}

}  // namespace

TEST(OscMessage, EncodesStringMessage)
{
  // Each string is NUL terminated and padded to a multiple of four bytes.
  EXPECT_EQ(
    encodeStringMessage("/a", "abc"),
    bytes(std::string("/a\0\0,s\0\0abc\0", 12)));
  EXPECT_EQ(
    encodeStringMessage("/abc", "abcd"),
    bytes(std::string("/abc\0\0\0\0,s\0\0abcd\0\0\0\0", 20)));
  EXPECT_EQ(
    encodeStringMessage("/objects", ""),
    bytes(std::string("/objects\0\0\0\0,s\0\0\0\0\0\0", 20)));
}

TEST(OscMessage, SizeMatchesEncoding)
{
  for (const std::string address : {"/a", "/ab", "/abc", "/objects"}) {
    for (std::size_t length = 0; length < 9; ++length) {
      const std::string argument(length, 'x');
      EXPECT_EQ(
        stringMessageSize(address, length), encodeStringMessage(address, argument).size());
      EXPECT_EQ(stringMessageSize(address, length) % 4, 0u);
    }
  }
}
