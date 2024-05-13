cmake_minimum_required(VERSION 2.8.3)
project(imu_wheel)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14")

find_package(Eigen3  REQUIRED)

include_directories(
  include
  ${EIGEN3_INCLUDE_DIR}
)

add_library(imu_wheel_lib
	src/data_type.cpp
  src/dr_odo_flow.cpp
  src/Estimator.cpp
)

target_link_libraries(imu_wheel_lib
	${EIGEN3_LIBS}
)