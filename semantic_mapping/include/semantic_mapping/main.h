//
// Created by thomas on 13.08.17.
//

#include <ros/ros.h>
#include <list>

#include <nav_msgs/OccupancyGrid.h>
#include "vision/VisionMsg.h"

#ifndef SEMANTIC_MAPPING_MAIN_H
#define SEMANTIC_MAPPING_MAIN_H

class SemanticMappingApp{
  public:
    ros::NodeHandle nh_;
    ros::Subscriber vision_sub_;
    ros::Subscriber map_sub_;
    std::list<vision::VisionMsg> vision_msgs_;

    SemanticMappingApp(ros::NodeHandle& nh);
    ~SemanticMappingApp();

    void run();
    void visionCb(const vision::VisionMsgConstPtr& msg);
    void mapCb(const nav_msgs::OccupancyGridConstPtr& msg);
};

#endif //SEMANTIC_MAPPING_MAIN_H
