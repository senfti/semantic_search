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

class Explorer{
  private:
    ros::Subscriber map_sub_;
    cv::Point curr_point_;
    geometry_msgs::Pose curr_frontier_;
    bool frontier_changed_;
    bool finished_ = false;
    tf::TransformListener* tf_listener_;

    cv::Point getNearestFree(const cv::Mat_<uchar>& valid_cells, int x, int y) const;

  public:
    Explorer(tf::TransformListener* tf_listener);

    void mapCb(const nav_msgs::OccupancyGridConstPtr& msg);

    geometry_msgs::Pose getNextFrontier();
    bool hasFrontierChanged() const { return frontier_changed_; }
    bool finished() const { return finished_; }

    bool did_abort_ = false;
};

#endif //EXECUTION_EXPLORER_H
