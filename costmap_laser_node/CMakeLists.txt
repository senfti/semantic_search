cmake_minimum_required(VERSION 2.8.3)
project(costmap_laser_node)

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
  tf2
  tf2_ros
  tf2_sensor_msgs
)

add_definitions(-std=c++11 -O3 -Wreturn-type)
catkin_package(
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(costmap_laser_node src/main.cpp)

target_link_libraries(costmap_laser_node
  ${catkin_LIBRARIES}
 )
