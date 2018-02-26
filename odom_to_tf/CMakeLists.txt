cmake_minimum_required(VERSION 2.8.3)
project(odom_to_tf)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  cmake_modules
  roscpp
  rospy
  sensor_msgs
  std_msgs
  geometry_msgs
  tf
)

add_definitions(-std=c++11 -g)
catkin_package(
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(odom_to_tf src/main.cpp)

target_link_libraries(odom_to_tf
  ${catkin_LIBRARIES}
 )
