cmake_minimum_required(VERSION 2.8.3)
project(semantic_mapping_v2)

find_package(catkin REQUIRED COMPONENTS
  genmsg
  nav_msgs
  openslam_gmapping
  pcl_ros
  roscpp
  rospy
  std_msgs
  tf
  vision
  ${PACKAGE_DEPENDENCIES}
)
find_package(Boost REQUIRED signals)
find_package(octomap REQUIRED)

add_definitions(-DOCTOMAP_NODEBUGOUT -std=c++11 -O3 -Wreturn-type)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${OCTOMAP_INCLUDE_DIRS}
)

catkin_package(
  CATKIN_DEPENDS ${PACKAGE_DEPENDENCIES}
)

add_executable(semantic_mapping src/main.cpp src/ObjectMapper.cpp src/HierarchyMapper.cpp src/OctoMapper.cpp src/RoomMapper.cpp src/SlamGMapping.cpp src/DoorMapper.cpp src/RoomTypeMapper.cpp)
target_link_libraries(semantic_mapping ${Boost_LIBRARIES} ${catkin_LIBRARIES} ${OCTOMAP_LIBRARIES} ${catkin_LIBRARIES})
if(catkin_EXPORTED_TARGETS)
  add_dependencies(semantic_mapping ${catkin_EXPORTED_TARGETS})
endif()
