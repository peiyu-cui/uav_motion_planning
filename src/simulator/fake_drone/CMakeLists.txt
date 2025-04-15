cmake_minimum_required(VERSION 3.0.2)
project(fake_drone)

add_compile_options(-std=c++14)
# set(CMAKE_BUILD_TYPE "Debug")
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS_DEBUG "-O0 -Wall -g -Wextra")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  nav_msgs
  quadrotor_msgs
)

find_package(Eigen3 REQUIRED)

catkin_package()

include_directories(
  # include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

add_executable(fake_drone src/fake_drone.cpp)

target_link_libraries(fake_drone
  ${catkin_LIBRARIES}
)
