//
// Created by thomas on 09.11.17.
//

#include "door_detection/DoorDetector.h"

int main(int argc, char** argv){
  ros::init (argc, argv, "door_detection");
  ros::NodeHandle nh;
  DoorDetector door_detector;
  door_detector.run();

  return 0;
}

