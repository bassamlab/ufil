// Copyright 2025 Chair of Embedded Software (Computer Science 11) - RWHT Aachen University
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

#ifndef UFIL_CENTRAL_FUSION__VISIBILITY_CONTROL_H_
#define UFIL_CENTRAL_FUSION__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if __INTELLISENSE__
#undef __ARM_NEON
#undef __ARM_NEON__
#endif

#if defined _WIN32 || defined __CYGWIN__
#ifdef __GNUC__
#define UFIL_RSU_FUSION_EXPORT __attribute__((dllexport))
#define UFIL_RSU_FUSION_IMPORT __attribute__((dllimport))
#else
#define UFIL_RSU_FUSION_EXPORT __declspec(dllexport)
#define UFIL_RSU_FUSION_IMPORT __declspec(dllimport)
#endif
#ifdef UFIL_RSU_FUSION_BUILDING_LIBRARY
#define UFIL_RSU_FUSION_PUBLIC UFIL_RSU_FUSION_EXPORT
#else
#define UFIL_RSU_FUSION_PUBLIC UFIL_RSU_FUSION_IMPORT
#endif
#define UFIL_RSU_FUSION_PUBLIC_TYPE UFIL_RSU_FUSION_PUBLIC
#define UFIL_RSU_FUSION_LOCAL
#else
#define UFIL_RSU_FUSION_EXPORT __attribute__((visibility("default")))
#define UFIL_RSU_FUSION_IMPORT
#if __GNUC__ >= 4
#define UFIL_RSU_FUSION_PUBLIC __attribute__((visibility("default")))
#define UFIL_RSU_FUSION_LOCAL __attribute__((visibility("hidden")))
#else
#define UFIL_RSU_FUSION_PUBLIC
#define UFIL_RSU_FUSION_LOCAL
#endif
#define UFIL_RSU_FUSION_PUBLIC_TYPE
#endif

#endif  // UFIL_CENTRAL_FUSION__VISIBILITY_CONTROL_H_
