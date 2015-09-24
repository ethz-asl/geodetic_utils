cmake_minimum_required(VERSION 2.8.3)
project(geodetic_utils)

find_package(catkin REQUIRED cmake_modules)
find_package(Eigen REQUIRED)

add_definitions(-std=c++11)

catkin_package(
  INCLUDE_DIRS include ${EIGEN_INCLUDE_DIRS}
  DEPENDS Eigen
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.hpp"
  PATTERN ".svn" EXCLUDE)
