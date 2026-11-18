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

#include "ufil_osc/osc_message.hpp"

#include <cstddef>
#include <string>
#include <vector>

namespace ufil_osc
{

namespace
{

/// Size of an OSC string of length characters: NUL terminated and padded to four bytes.
constexpr std::size_t paddedSize(std::size_t length) {return (length + 4) & ~std::size_t{3};}

void appendString(std::vector<char> & packet, const std::string & value)
{
  packet.insert(packet.end(), value.begin(), value.end());
  packet.resize(packet.size() + paddedSize(value.size()) - value.size(), '\0');
}

constexpr char kTypeTags[] = ",s";

}  // namespace

std::size_t stringMessageSize(const std::string & address, std::size_t argument_size)
{
  return paddedSize(address.size()) + paddedSize(sizeof(kTypeTags) - 1) +
         paddedSize(argument_size);
}

std::vector<char> encodeStringMessage(const std::string & address, const std::string & argument)
{
  std::vector<char> packet;
  packet.reserve(stringMessageSize(address, argument.size()));
  appendString(packet, address);
  appendString(packet, kTypeTags);
  appendString(packet, argument);
  return packet;
}

}  // namespace ufil_osc
