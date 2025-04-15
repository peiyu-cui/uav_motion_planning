cmake_minimum_required(VERSION 3.0.2)
project(path_searching)

find_package(Eigen3 REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  plan_env
  cv_bridge
)

find_package(OpenCV REQUIRED)
find_package(PCL 1.7 REQUIRED)


SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14")
set(CMAKE_BUILD_TYPE "RelWithDebInfo")




catkin_package(
  INCLUDE_DIRS include
  LIBRARIES path_searching kdtree
  CATKIN_DEPENDS plan_env
  DEPENDS PCL
)


## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

link_directories(${PCL_LIBRARY_DIRS})

add_library(kdtree
  src/kdtree/kdtree.cpp
)

add_library(path_searching
  src/a_star.cpp
  src/rrt.cpp
  src/rrt_star.cpp
  src/kino_astar.cpp
)

target_link_libraries(path_searching
  ${catkin_LIBRARYS}
  kdtree
)
