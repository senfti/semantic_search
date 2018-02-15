cmake_minimum_required(VERSION 2.8.3)
project(execution_test)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  actionlib
  geometry_msgs
  semantic_mapping_v2
  vision
  nav_msgs
  tf
  roscpp
  rospy
  std_msgs
  octomap_msgs
  pcl_ros
)

find_package( OpenCV REQUIRED )
find_package(octomap REQUIRED)

add_definitions(-DOCTOMAP_NODEBUGOUT -std=c++11 -O3 -Wreturn-type)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   geometry_msgs#   std_msgs
# )

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include ${catkin_INCLUDE_DIRS} ${OCTOMAP_INCLUDE_DIRS})

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES execution_test
#  CATKIN_DEPENDS actionlib geometry_msgs message_generation roscpp rospy std_msgs
  CATKIN_DEPENDS ${PACKAGE_DEPENDENCIES}
)

###########
## Build ##
###########

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
add_executable(${PROJECT_NAME} src/main.cpp src/ExecuteActionServer.cpp src/StartRotationStateMachine.cpp src/Explorer.cpp include/execution_test/Searcher.h src/Searcher.cpp
  src/ObjectMapper.cpp src/OctoMapper.cpp include/execution_test/OctoMapper.h include/execution_test/ObjectMapper.h)
## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${OpenCV_LIBS} ${OCTOMAP_LIBRARIES})

