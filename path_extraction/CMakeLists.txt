cmake_minimum_required(VERSION 2.8.3)
project(path_extraction)

add_definitions(-std=c++11 -g)

find_package(catkin REQUIRED COMPONENTS
  nav_msgs
  rosbag
  roscpp
  tf
  rospy
  std_msgs
  semantic_mapping_v2
)

find_package( OpenCV REQUIRED )

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

catkin_package(
  #  INCLUDE_DIRS include
  #  LIBRARIES path_extraction
  #  CATKIN_DEPENDS nav_msgs rosbag roscpp rospy std_msgs
  #  DEPENDS system_lib
)

add_executable(path_extraction src/main.cpp)

target_link_libraries(path_extraction
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
  )