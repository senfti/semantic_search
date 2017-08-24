cmake_minimum_required(VERSION 2.8.3)
project(hardware)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  cmake_modules
  roscpp
  rospy
  std_msgs
  tf
)

find_package( OpenCV REQUIRED )
catkin_package(
)
include_directories(${catkin_INCLUDE_DIRS})
