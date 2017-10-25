//
// Created by thomas on 13.08.17.
//

#ifndef SEMANTIC_MAPPING_MAIN_H
#define SEMANTIC_MAPPING_MAIN_H

#include <ros/ros.h>
#include <map>

#include <nav_msgs/OccupancyGrid.h>
#include <std_msgs/Float64.h>
#include "vision/VisionMsg.h"
#include "prob_map_view/ProbMapMsg.h"
#include "semantic_mapping/VisionResult.h"
#include "semantic_mapping/KnowledgeBase.h"
#include <opencv2/opencv.hpp>


class SemanticMappingApp{
  private:
    static constexpr int PUBLISH_VISION_POSES = 1;

    ros::NodeHandle nh_;
    ros::Subscriber vision_sub_;
    ros::Subscriber map_sub_;
    ros::Subscriber entropy_sub_;
    ros::Publisher vision_pose_pub_;
    ros::Publisher obj_map_pub_;
    ros::Publisher place_map_pub_;

    std::vector<VisionResult> vision_results_;

    const double HORIZONTAL_CAMERA_SPREAD = 1.1;                    // parameters of xtion pro: position in world = (pixel/num_pixels - 0.5) * camera_spread * distance
    const double VERTICAL_CAMERA_SPREAD = 0.825;
    const double XTION_FOV_2 = std::atan(HORIZONTAL_CAMERA_SPREAD/2.0);
    const double MIN_VISION_DIST = 0.5;
    const double MAX_VISION_DIST = 4.0;

    const int NUM_OBJECT_TYPES = 80;
    const int NUM_PLACE_TYPES = 205;
    const double OBJ_PIXEL_PER_METER = 5.0;
    const int PROB_DRAW_OVERSAMPLING_FACTOR = 5;
    const double OBJ_MAP_PRIOR = 0.01;
    const int MAP_BASE_SIZE = 20;
    const double SIGMA_PER_ENTROPY = 0.05;
    const double PLACE_VISIBLE_PROB = 0.01;

    double pose_sigma_ = 0.2;

    std::vector<double> place_prior_ = std::vector<double>(NUM_PLACE_TYPES);
    std::vector<double> object_prior_p_ = std::vector<double>(NUM_PLACE_TYPES);
    std::vector<double> object_prior_l_ = std::vector<double>(NUM_PLACE_TYPES);
    std::vector<cv::Mat_<double>> object_maps_;
    std::vector<cv::Mat_<double>> place_maps_;
    cv::Point origin_;
    std::vector<std::string> object_labels_;
    std::vector<std::string> place_labels_;

    cv::Mat_<double> generateSeenMap(const VisionResult& vision_result);

    std::vector<cv::Point> getCornersFromDetection(const Object& obj, const cv::Point2d& position, double direction);
    std::vector<cv::Mat_<double>> generateDetectionProbMaps(const VisionResult& vision_result, const cv::Mat_<double>& seen_map);

    void updateObjectMaps(const VisionResult& vision_result, const cv::Mat_<double>& seen_map);
    void updatePlaceMaps(const VisionResult& vision_result, const cv::Mat_<double>& seen_map);
    void publishVisionPoses(const vision::VisionMsgConstPtr& msg);

  public:
    SemanticMappingApp(ros::NodeHandle& nh);
    ~SemanticMappingApp();

    void run();
    void visionCb(const vision::VisionMsgConstPtr& msg);
    void mapCb(const nav_msgs::OccupancyGridConstPtr& msg);
    void entropyCb(const std_msgs::Float64ConstPtr& msg);
};

/*class SemanticMappingApp{
  private:
    static constexpr int PUBLISH_VISION_POSES = 1;

    ros::NodeHandle nh_;
    ros::Subscriber vision_sub_;
    ros::Subscriber map_sub_;
    ros::Publisher vision_pose_pub_;

    std::vector<VisionResult> vision_results_;
    std::vector<VisionResult> improved_vision_result_;
    int processed_views_ = 0;

    KnowledgeBase kb_;

    const int NUM_OBJECT_TYPES = 80;
    std::vector<cv::Mat_<double>> object_maps_;
    std::vector<cv::Mat_<double>> place_maps_;
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
};*/

#endif //SEMANTIC_MAPPING_MAIN_H
