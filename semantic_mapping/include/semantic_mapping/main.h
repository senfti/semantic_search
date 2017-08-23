//
// Created by thomas on 13.08.17.
//

#include <ros/ros.h>
#include <map>

#include <nav_msgs/OccupancyGrid.h>
#include "vision/VisionMsg.h"
#include "semantic_mapping/VisionResult.h"
#include "semantic_mapping/KnowledgeBase.h"
#include <opencv2/opencv.hpp>

#ifndef SEMANTIC_MAPPING_MAIN_H
#define SEMANTIC_MAPPING_MAIN_H

class SemanticMappingApp{
  private:
    static constexpr int PUBLISH_VISION_POSES = 1;
    const float MAP_UPDATE_FACTOR = 0.3;

    ros::NodeHandle nh_;
    ros::Subscriber vision_sub_;
    ros::Subscriber map_sub_;
    ros::Publisher vision_pose_pub_;

    std::vector<VisionResult> vision_results_;
    std::vector<VisionResult> improved_vision_result_;
    int processed_views_ = 0;

    KnowledgeBase kb_;

    std::vector<cv::Mat_<float>> place_maps_;
    std::vector<std::string> place_labels_;
    nav_msgs::MapMetaData old_map_data_;

    void publishVisionPoses(const vision::VisionMsgConstPtr& msg);
    void updatePlaceMaps(const nav_msgs::OccupancyGridConstPtr& msg);

  public:
    SemanticMappingApp(ros::NodeHandle& nh);
    ~SemanticMappingApp();

    void run();
    void visionCb(const vision::VisionMsgConstPtr& msg);
    void mapCb(const nav_msgs::OccupancyGridConstPtr& msg);
};

#endif //SEMANTIC_MAPPING_MAIN_H
