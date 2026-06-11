# Copyright (c) 2026 Anton Matosov
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

find_package(SDL3 QUIET CONFIG)

if(SDL3_FOUND AND NOT TARGET SDL3::SDL3 AND SDL3_INCLUDE_DIRS AND SDL3_LIBRARIES)
  add_library(SDL3::SDL3 INTERFACE IMPORTED)
  set_property(TARGET SDL3::SDL3 PROPERTY INTERFACE_INCLUDE_DIRECTORIES "${SDL3_INCLUDE_DIRS}")
  target_link_libraries(SDL3::SDL3 INTERFACE ${SDL3_LIBRARIES})
endif()

if(TARGET SDL3::SDL3)
  set(SDL3_CUSTOM_FOUND TRUE)
else()
  set(SDL3_CUSTOM_FOUND FALSE)
endif()

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(SDL3_CUSTOM DEFAULT_MSG SDL3_CUSTOM_FOUND)
