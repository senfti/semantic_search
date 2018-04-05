cmake_minimum_required(VERSION 3.9)
project(coco_place_classifier)

find_package( OpenCV REQUIRED )
find_package( Caffe REQUIRED )

add_definitions(-std=c++11 -O3)
add_definitions(${Caffe_DEFINITIONS} )

include_directories(include ${Caffe_INCLUDE_DIRS})

add_executable(coco_place_classifier main.cpp CaffeClassifier.cpp CaffeClassifier.h)

target_link_libraries(coco_place_classifier
  ${OpenCV_LIBS}
  ${Caffe_LIBRARIES}
  )
