cmake_minimum_required(VERSION 2.8.3)
project(mouse_control)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
)


add_definitions(-std=c++11 -g)

find_package( wxWidgets COMPONENTS core base adv REQUIRED )
include( "${wxWidgets_USE_FILE}" )

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

catkin_package(
  #  INCLUDE_DIRS include
  #  LIBRARIES mouse_control
  #  CATKIN_DEPENDS geometry_msgs roscpp
  #  DEPENDS system_lib
)

add_executable(${PROJECT_NAME} src/main.cpp)
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${wxWidgets_LIBRARIES}
)