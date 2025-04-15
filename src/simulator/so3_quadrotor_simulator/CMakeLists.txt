cmake_minimum_required(VERSION 3.0.2)
project(so3_quadrotor_simulator)

add_compile_options(-std=c++11)
# set(CMAKE_BUILD_TYPE "Debug")
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS_DEBUG "-O0 -Wall -g -Wextra")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  quadrotor_msgs
  uav_utils
  cmake_utils
)

find_package(Eigen3 REQUIRED)
find_package(Armadillo REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  # LIBRARIES irobot_msgs
  CATKIN_DEPENDS roscpp quadrotor_msgs uav_utils
  # DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${ARMADILLO_INCLUDE_DIRS}
  ${CMAKE_CURRENT_SOURCE_DIR}/include/ode
)

add_library(quadrotor_dynamics src/dynamics/Quadrotor.cpp)

add_executable(quadrotor_simulator_so3 src/quadrotor_simulator_so3.cpp)

target_link_libraries(quadrotor_simulator_so3
  ${catkin_LIBRARIES}
  quadrotor_dynamics
)
