cmake_minimum_required(VERSION 2.8.3)
project(geotf_python)

find_package(catkin_simple REQUIRED)

catkin_simple()

# Set up the python exports.
SET(PY_PROJECT_NAME ${PROJECT_NAME})
SET(PY_PACKAGE_DIR python/geotf)

add_python_export_library(${PY_PROJECT_NAME} ${PY_PACKAGE_DIR}
  src/module.cc
)

cs_install()
cs_export()