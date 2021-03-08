cmake_minimum_required(VERSION 2.8.3)
project(roscpp_initializer)

# Check if Python 3 installed
find_package(PythonInterp)

if(PYTHON_VERSION_STRING EQUAL "3.6.*")
  add_definitions(-std=c++11 -I /usr/include/python3.6)
elseif(PYTHON_VERSION_STRING EQUAL "2.7.*")
  add_definitions(-std=c++11 -I /usr/include/python2.7)
else()
  message(FATAL_ERROR "Neither Python 3.6 nor 2.7 was found. Make sure either one is installed. Found version: ${PYTHON_VERSION_STRING}")
endif(PYTHON_VERSION_STRING EQUAL "3.6.*")

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

# Set up the python exports.
SET(PY_PROJECT_NAME ${PROJECT_NAME})
SET(PY_PACKAGE_DIR python/roscpp_initializer)

# cs_add_library(${PROJECT_NAME} src/roscpp_initializer.cc
#               src/wrap_roscpp_initializer.cc)

add_python_export_library(${PY_PROJECT_NAME} ${PY_PACKAGE_DIR}
  src/wrap_roscpp_initializer.cc
  src/roscpp_initializer.cc
)

cs_install()
cs_export()
