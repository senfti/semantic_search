cmake_minimum_required(VERSION 2.8.3)
project(coco_place_detection)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages

add_definitions(-std=c++11 -g)
add_definitions(${Caffe_DEFINITIONS} )

find_package( OpenCV REQUIRED )
find_package( Caffe REQUIRED )

include_directories(include ${Caffe_INCLUDE_DIRS})

add_executable(coco_place_detection main.cpp CaffeClassifier.cpp)

target_link_libraries(coco_place_detection
  ${OpenCV_LIBS}
  ${Caffe_LIBRARIES}
 )
