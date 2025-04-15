cmake_minimum_required(VERSION 3.0.2)
project(pose_utils)

add_compile_options(-std=c++14)
# set(CMAKE_BUILD_TYPE "Debug")
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS_DEBUG "-O0 -Wall -g -Wextra")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(catkin REQUIRED COMPONENTS roscpp)

find_package(Armadillo REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES pose_utils
  # CATKIN_DEPENDS geometry_msgs nav_msgs
  # DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${ARMADILLO_INCLUDE_DIRS}
)

add_library(pose_utils
  ${ARMADILLO_LIBRARIES}
  src/pose_utils.cpp
)
