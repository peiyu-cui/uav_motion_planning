cmake_minimum_required(VERSION 3.0.2)
project(map_generator)

add_compile_options(-std=c++14)
# set(CMAKE_BUILD_TYPE "Debug")
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS_DEBUG "-O0 -Wall -g -Wextra")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  pcl_conversions
)

find_package(PCL REQUIRED)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

add_executable(random_forest src/random_forest.cpp)

target_link_libraries(random_forest
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)
