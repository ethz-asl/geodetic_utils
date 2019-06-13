cmake_minimum_required(VERSION 2.8.3)
project(geotf)

add_definitions(-std=c++11)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

find_package(GDAL REQUIRED)

cs_add_library(${PROJECT_NAME} src/geodetic_converter.cc)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} gdal)

cs_add_executable(demo_node src/demo_node.cc)
target_link_libraries(demo_node ${PROJECT_NAME})


cs_install()
cs_export()
