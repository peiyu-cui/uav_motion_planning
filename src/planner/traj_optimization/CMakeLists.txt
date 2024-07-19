cmake_minimum_required(VERSION 3.0.2)
project(traj_optimization)

find_package(Eigen3 REQUIRED)
find_package(OsqpEigen REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
)



catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES traj_optimization
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_library(traj_optimization
  src/minimum_jerk.cpp)

target_link_libraries(traj_optimization
  ${catkin_LIBRARIES}
  OsqpEigen::OsqpEigen)
