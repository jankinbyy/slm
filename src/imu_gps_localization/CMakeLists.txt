cmake_minimum_required(VERSION 3.0)
project(imu_gps_localization)

add_compile_options(-std=c++14)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  nav_msgs
)

catkin_package()
include(cmake/yaml.cmake)
add_subdirectory(imu_gps_localizer)
add_subdirectory(imu_wheel)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  imu_gps_localizer/include
  imu_wheel/include
  ros_wrapper/include
)

add_library(ros_wrapper_lib
	ros_wrapper/src/localization_wrapper.cpp
  ros_wrapper/src/config.cpp
)
target_link_libraries(ros_wrapper_lib
  ${catkin_LIBRARIES}
  ${ALL_TARGET_LIBRARIES}
  imu_gps_localizer_lib
  imu_wheel_lib
  glog
)

## Localization node.
add_executable(${PROJECT_NAME}_node ros_wrapper/src/localization_node.cpp)
target_link_libraries(${PROJECT_NAME}_node
  ros_wrapper_lib
)
