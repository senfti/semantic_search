cmake_minimum_required(VERSION 2.8.3)
project(testing_data)

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
  nav_msgs
  sensor_msgs
  image_transport
  cv_bridge
  tf_conversions
)

find_package( OpenCV REQUIRED )
catkin_package(
)
include_directories(${catkin_INCLUDE_DIRS})
add_executable(offline_test src/main.cpp)


target_link_libraries(offline_test
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
 )
