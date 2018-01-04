//
// Created by thomas on 02.01.18.
//

#ifndef EXECUTION_SEARCHER_H
#define EXECUTION_SEARCHER_H

#include <execution/ObjectMapper.h>
#include <execution/OctoMapper.h>
#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/Pose.h>
#include <vector>
#include <tf/transform_listener.h>
#include <opencv2/opencv.hpp>
#include <std_msgs/Int8.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <vision/VisionMsg.h>


class Searcher{
  public:
    const float SEARCH_MAX_ROT_VEL = 1.0;
    const float SEARCH_MAX_TRANS_VEL = 0.3;
    const int VIEW_DIST = 1.0/0.05;
    const int ROBOT_SIZE = 0.3/0.05;

    float OBJ_PRIOR_PROB = 0.01f;
    float OBJ_MIN_PROB = 0.0001f;
    float OBJ_MAX_PROB = 0.9f;

    float OBJ_DEFAULT_RESOLUTION = 4.f;
    float OBJ_DEFUALT_MAX_HEIGHT = 1.6f;
    float ROOM_EXPECTED_SIZE = 32.f;

    float V_H = 0.3;
    float V_M = 0.0002;

    float POINTCLOUD_MIN_Z = 0.0f;
    float POINTCLOUD_MAX_Z = 1.8f;

    float OBJECT_FOUND_THRESH = 0.7;

    int VIEW_ANGLE_STEPS = 12;
    int SAMPLE_COUNT_THRESH = 10000;

    float VIEW_MIN_DIST = 0.8f;
    float VIEW_MAX_DIST = 3.5f;
    float VIEW_ANGLE = 50.f;

    float TURN_SPEED = 0.5;
    float MOVE_SPEED = 0.1;
    float VIEW_TIME = 0.2;

  private:
    ros::Subscriber map_sub_;
    ros::Subscriber vision_sub_;
    tf::TransformListener* tf_listener_;

    ros::Publisher octomap_pub_;

    int searched_obj_ = 0;
    ObjectMap* obj_map_ = nullptr;
    ObjectMap* prior_prob_map_ = nullptr;
    OctoMapper* octo_mapper_ = nullptr;

    ros::Publisher map_pub_;
    cv::Mat_<uchar> accessible_mat_;
    cv::Point accessible_origin_;
    float accessible_resolution_;

    geometry_msgs::Pose curr_view_pose_;
    bool curr_view_changed_;
    bool finished_ = false;

    cv::Point getNearestFree(const cv::Mat_<uchar>& valid_cells, int x, int y) const;
    cv::Mat_<float> getProbMap(cv::Point& origin);
    cv::Mat_<float> getViewKernel(int angle_step) const;
    cv::Mat_<float> calcMoveTime(int width, int height, int angle_step, const cv::Point& curr_pos, float curr_angle);

  public:
    Searcher(int searched_obj, int curr_room, tf::TransformListener* tf_listener);
    ~Searcher();

    void mapCb(const nav_msgs::OccupancyGridConstPtr& msg);
    void visionCb(const vision::VisionMsgConstPtr& msg);
    void insertObject(const pcl::PointCloud<pcl::PointXYZ>& cloud, const vision::ObjectDetectionMsg& msg);
    void insertCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, const tf::Point& sensorOriginTf);

    bool objFound();

    void calcNextViewpoint(const tf::Transform& curr_pose);
};


#endif //EXECUTION_SEARCHER_H
