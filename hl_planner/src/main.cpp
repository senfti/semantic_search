//
// Created by thomas on 20.12.17.
//

#include <ros/ros.h>
#include <hl_planner/Planner.h>

int main(int argc, char** argv){
  ros::init(argc, argv, "hl_planner");
  Planner planner;
  planner.exploreAll();

  return 0;
}
