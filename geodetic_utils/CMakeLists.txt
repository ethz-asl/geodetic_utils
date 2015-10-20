cmake_minimum_required(VERSION 2.8.3)
project(geodetic_utils)

set(CMAKE_BUILD_TYPE Release)

set(CXXFLAGS ${CXXFLAGS} -fPIC)
# Linux specific flags
if(${CMAKE_SYSTEM_NAME} STREQUAL "Linux")
	set(CXXFLAGS ${CXXFLAGS} -fPIC)
	add_definitions(-std=c++0x)
endif()

find_package(catkin REQUIRED COMPONENTS
  roslib
  tf
  geometry_msgs
  roscpp
  sensor_msgs
  cmake_modules
)

find_package(Eigen REQUIRED)

add_definitions(-std=c++11)

catkin_package(
  INCLUDE_DIRS include ${Eigen_INCLUDE_DIRS}
  LIBRARIES geodetic_utils
  CATKIN_DEPENDS geometry_msgs roscpp sensor_msgs geometry_msgs
  DEPENDS Eigen
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
)


add_executable(set_gps_reference_node src/set_gps_reference_node.cpp)
add_dependencies(set_gps_reference_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(set_gps_reference_node  ${catkin_LIBRARIES})

add_executable(gps_to_pose_conversion_node src/gps_to_pose_conversion_node.cpp)
add_dependencies(gps_to_pose_conversion_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(gps_to_pose_conversion_node  ${catkin_LIBRARIES})

add_executable(imu_compass_asl src/imu_compass_asl.cpp)
add_dependencies(imu_compass_asl ${catkin_EXPORTED_TARGETS})
target_link_libraries(imu_compass_asl  ${catkin_LIBRARIES})

