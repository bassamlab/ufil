# Copyright 2025 Chair of Embedded Software (Computer Science 11) - RWTH Aachen University
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
# THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.

include(FindPackageHandleStandardArgs)

set(OSCPACK_ROOT $ENV{OSCPACK_ROOT})
if(NOT DEFINED ENV{OSCPACK_ROOT})
    set(OSCPACK_ROOT "/opt/oscpack")
endif()

find_path(OSCPACK_INCLUDE_DIRS "osc" PATH_SUFFIXES "oscpack")
find_library(OSCPACK_LIBRARY oscpack REQUIRED)

find_package_handle_standard_args(OSCPACK DEFAULT_MSG OSCPACK_LIBRARY OSCPACK_INCLUDE_DIRS)
mark_as_advanced(OSCPACK_FOUND OSCPACK_INCLUDE_DIRS OSCPACK_LIBRARY OSCPACK_DIR)


if(OSCPACK_FOUND)
    add_library(OscPack INTERFACE IMPORTED)
    target_link_libraries(OscPack INTERFACE ${OSCPACK_LIBRARY})
    target_include_directories(OscPack INTERFACE ${OSCPACK_INCLUDE_DIRS})
endif()