cmake_minimum_required(VERSION 2.8.3)
project(semantic_mapping)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  cmake_modules
  cv_bridge
  image_transport
  roscpp
  rospy
  sensor_msgs
  std_msgs
  nav_msgs
  tf
  message_generation
  vision
)

include_directories(include ${catkin_INCLUDE_DIRS})
add_definitions(-std=c++11 -g)

catkin_package(
)

add_executable(semantic_mapping src/main.cpp)

target_link_libraries(semantic_mapping
  ${catkin_LIBRARIES}
 )

add_dependencies(semantic_mapping ${catkin_EXPORTED_TARGETS})
