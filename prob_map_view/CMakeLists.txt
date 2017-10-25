cmake_minimum_required(VERSION 2.8.3)
project(prob_map_view)

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
  kb_query
)
find_package( wxWidgets COMPONENTS core base adv REQUIRED )
include( "${wxWidgets_USE_FILE}" )

include_directories(include ${catkin_INCLUDE_DIRS})

add_message_files(
  FILES
  CvImgMsg.msg
  ProbMapMsg.msg
)
generate_messages(
  DEPENDENCIES
  sensor_msgs
  std_msgs
)

add_definitions(-std=c++11 -g)

catkin_package(
)

add_executable(prob_map_view src/main.cpp src/ProbViewer_B.cpp src/ProbViewer.cpp src/ImagePanel.cpp)

target_link_libraries(prob_map_view
  ${catkin_LIBRARIES}
  ${wxWidgets_LIBRARIES}
 )

add_dependencies(prob_map_view prob_map_view_generate_messages_cpp)
