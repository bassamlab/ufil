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

#ifndef UFIL_ETSI__VISIBILITY_CONTROL_HPP_
#define UFIL_ETSI__VISIBILITY_CONTROL_HPP_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
#ifdef __GNUC__
#define UFIL_ETSI_EXPORT __attribute__ ((dllexport))
#define UFIL_ETSI_IMPORT __attribute__ ((dllimport))
#else
#define UFIL_ETSI_EXPORT __declspec(dllexport)
#define UFIL_ETSI_IMPORT __declspec(dllimport)
#endif
#ifdef UFIL_ETSI_BUILDING_LIBRARY
#define UFIL_ETSI_PUBLIC UFIL_ETSI_EXPORT
#else
#define UFIL_ETSI_PUBLIC UFIL_ETSI_IMPORT
#endif
#define UFIL_ETSI_PUBLIC_TYPE UFIL_ETSI_PUBLIC
#define UFIL_ETSI_LOCAL
#else
#define UFIL_ETSI_EXPORT __attribute__ ((visibility("default")))
#define UFIL_ETSI_IMPORT
#if __GNUC__ >= 4
#define UFIL_ETSI_PUBLIC __attribute__ ((visibility("default")))
#define UFIL_ETSI_LOCAL  __attribute__ ((visibility("hidden")))
#else
#define UFIL_ETSI_PUBLIC
#define UFIL_ETSI_LOCAL
#endif
#define UFIL_ETSI_PUBLIC_TYPE
#endif

#endif  // UFIL_ETSI__VISIBILITY_CONTROL_HPP_
