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

#ifndef UFIL_OSC__OSC_MESSAGE_HPP_
#define UFIL_OSC__OSC_MESSAGE_HPP_

#include <cstddef>
#include <string>
#include <vector>

#include "ufil_osc/visibility_control.hpp"

namespace ufil_osc
{

/**
 * @brief Size of an OSC 1.0 message with one string argument [bytes].
 *
 * The address, the type tag string ",s" and the argument are each terminated by a NUL character
 * and padded with NUL characters to a multiple of four bytes.
 */
UFIL_OSC_PUBLIC std::size_t stringMessageSize(
  const std::string & address,
  std::size_t argument_size);

/// Encodes an OSC 1.0 message with one string argument. argument must not contain NUL characters.
UFIL_OSC_PUBLIC std::vector<char> encodeStringMessage(
  const std::string & address, const std::string & argument);

}  // namespace ufil_osc

#endif  // UFIL_OSC__OSC_MESSAGE_HPP_
