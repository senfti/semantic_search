//
// Created by thomas on 18.12.17.
//

#ifndef EXECUTION_EXPLORER_H
#define EXECUTION_EXPLORER_H

#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/Pose.h>
#include <vector>
#include <tf/transform_listener.h>
#include <opencv2/opencv.hpp>
#include <std_msgs/Int8.h>
#include <semantic_mapping_v2/ObjFoundMsg.h>

class Explorer{
  public:
    float EXPLORE_MAX_ROT_VEL = 1.0;
    float EXPLORE_MAX_TRANS_VEL = 0.3;
    int VIEW_DIST = 0.5/0.05;
    int ROBOT_SIZE = 0.35/0.05;
    float OBJECT_FOUND_THRESH = 0.5;

  private:
    ros::Subscriber map_sub_;
    ros::Subscriber door_found_sub_;
    ros::Subscriber obj_found_sub_;
    cv::Point curr_point_;
    geometry_msgs::Pose curr_frontier_;
    bool frontier_changed_;
    int finished_count_ = 0;
    bool finished_ = false;
    bool door_found_stopped_ = false;
    bool obj_found_stopped_ = false;
    geometry_msgs::PoseStamped found_pose_;
    int searched_obj_ = -1;
    bool running_ = false;
    nav_msgs::OccupancyGrid last_map_;
    tf::TransformListener* tf_listener_;

    cv::Point getNearestFree(const cv::Mat_<uchar>& valid_cells, int x, int y) const;
    void calcFrontier();

    ros::Publisher map_pub_;
    ros::Publisher obj_found_pub_;

    std::vector<cv::Point> circle_points_;
    std::vector<cv::Point> near_circle_points_;

  public:
    Explorer(tf::TransformListener* tf_listener);

    void mapCb(const nav_msgs::OccupancyGridConstPtr& msg);
    void doorFoundCb(const std_msgs::Int8& msg);
    void objFoundCb(const semantic_mapping_v2::ObjFoundMsgConstPtr& msg);

    geometry_msgs::Pose getNextFrontier();
    bool hasFrontierChanged() const { return frontier_changed_; }
    bool finished() const { return finished_; }
    bool doorFoundStopped() const { return door_found_stopped_; }
    bool objFoundStopped() const { return obj_found_stopped_; }
    bool running() const { return running_; }

    void start(int searched_obj);
    void stop();

    bool did_abort_ = false;
};

#endif //EXECUTION_EXPLORER_H
