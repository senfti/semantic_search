cmake_minimum_required(VERSION 2.8.3)
project(vision)

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
  geometry_msgs
  tf
  pcl_ros
  message_generation
)

add_message_files(
  FILES
  ObjectDetectionMsg.msg
  PlaceRecognitionMsg.msg
  VisionMsg.msg
  DetectionSample.msg
  ObjectFoundMsg.msg
)
generate_messages(
  DEPENDENCIES
  geometry_msgs
  sensor_msgs
  std_msgs
)

add_definitions(-std=c++11 -O3)
add_definitions(${Caffe_DEFINITIONS} )

find_package( OpenCV REQUIRED )
find_package( Caffe REQUIRED )

catkin_package(
)

link_directories(
  /home/thomas/darknet
)

include_directories(include ${catkin_INCLUDE_DIRS} ${Caffe_INCLUDE_DIRS} "/home/thomas/darknet/include" "/home/thomas/darknet/src")

add_executable(vision src/main.cpp src/CaffeClassifier.cpp src/YoloDetector.cpp)

target_link_libraries(vision
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
  ${Caffe_LIBRARIES}
  libdarknet.so
 )

add_dependencies(vision vision_generate_messages_cpp)
