cmake_minimum_required(VERSION 2.8.3)
project(geotf_python)

find_package(catkin_simple REQUIRED)

catkin_simple()

# Set up the python exports.
SET(PY_PROJECT_NAME ${PROJECT_NAME})
SET(PY_PACKAGE_DIR python/geotf)

# Taken from minkindr_python: 
# https://github.com/ethz-asl/minkindr/blob/master/minkindr_python/CMakeLists.txt
set (CMAKE_CXX_FLAGS "-Wall  ${CMAKE_CXX_FLAGS}")

# Enable C++11 support.
if (CMAKE_VERSION VERSION_LESS "3.1")
  if (MSVC)
    message(FATAL_ERROR "CMake version 3.1 or later is required to compiler ${PROJECT_NAME} with Microsoft Visual C++")
  endif ()
  if (CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
    set (CMAKE_CXX_FLAGS "-std=c++0x ${CMAKE_CXX_FLAGS}")
  else ()
    set (CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")
  endif ()
else ()
  set (CMAKE_CXX_STANDARD 11)
endif ()

add_python_export_library(${PY_PROJECT_NAME} ${PY_PACKAGE_DIR}
  src/module.cc
  src/export_geodetic_converter.cc
)

cs_install()
cs_export()
