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

class Explorer{
  public:
    const float EXPLORE_MAX_ROT_VEL = 1.0;
    const float EXPLORE_MAX_TRANS_VEL = 0.3;
    const int VIEW_DIST = 0.5/0.05;
    const int ROBOT_SIZE = 0.35/0.05;

  private:
    ros::Subscriber map_sub_;
    ros::Subscriber door_found_sub_;
    cv::Point curr_point_;
    geometry_msgs::Pose curr_frontier_;
    bool frontier_changed_;
    bool finished_ = false;
    bool door_found_stopped_ = false;
    bool running_ = false;
    nav_msgs::OccupancyGrid last_map_;
    tf::TransformListener* tf_listener_;

    cv::Point getNearestFree(const cv::Mat_<uchar>& valid_cells, int x, int y) const;
    void calcFrontier();

    ros::Publisher map_pub_;

    std::vector<cv::Point> circle_points_;

  public:
    Explorer(tf::TransformListener* tf_listener);

    void mapCb(const nav_msgs::OccupancyGridConstPtr& msg);
    void doorFoundCb(const std_msgs::Int8& msg);

    geometry_msgs::Pose getNextFrontier();
    bool hasFrontierChanged() const { return frontier_changed_; }
    bool finished() const { return finished_; }
    bool doorFoundStopped() const { return door_found_stopped_; }
    bool running() const { return running_; }

    void start();
    void stop();

    bool did_abort_ = false;
};

#endif //EXECUTION_EXPLORER_H
