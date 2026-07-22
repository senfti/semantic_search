cmake_minimum_required(VERSION 2.8.3)
project(velocity_scale)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  rospy
  std_msgs
)

add_definitions(-std=c++11 -O3 -Wreturn-type)
catkin_package(
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(velocity_scale src/main.cpp)

target_link_libraries(velocity_scale
  ${catkin_LIBRARIES}
  )