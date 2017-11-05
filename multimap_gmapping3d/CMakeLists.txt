cmake_minimum_required(VERSION 2.8)
project(multimap_gmapping3d)

find_package(catkin REQUIRED nav_msgs nodelet openslam_gmapping roscpp tf rosbag_storage pcl_ros ${PACKAGE_DEPENDENCIES})
find_package(Boost REQUIRED signals)
find_package(octomap REQUIRED)

add_definitions(-DOCTOMAP_NODEBUGOUT -std=c++11 -O3 -mavx2)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${OCTOMAP_INCLUDE_DIRS}
)

catkin_package(
  CATKIN_DEPENDS ${PACKAGE_DEPENDENCIES}
)

add_executable(multimap_gmapping3d src/slam_gmapping.cpp src/main.cpp src/OctoGMapper.cpp src/OctoMapper.cpp)
target_link_libraries(multimap_gmapping3d ${Boost_LIBRARIES} ${catkin_LIBRARIES} ${OCTOMAP_LIBRARIES} ${catkin_LIBRARIES})
if(catkin_EXPORTED_TARGETS)
  add_dependencies(multimap_gmapping3d ${catkin_EXPORTED_TARGETS})
endif()
