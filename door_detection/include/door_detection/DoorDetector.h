//
// Created by thomas on 09.11.17.
//

#ifndef DOOR_DETECTION_DOOR_DETECT_H
#define DOOR_DETECTION_DOOR_DETECT_H

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <opencv2/opencv.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

class DoorDetector{
  private:
    float MIN_DOOR_HEIGHT = 1.95f;
    float MAX_DOOR_HEIGHT = 2.05f;
    float OCC_RES = 50.f;
    cv::Size IMG_SIZE = cv::Size(3*OCC_RES, 4*OCC_RES);
    float FILL_THRESH = 0.8f;
    float MIN_WIDTH = 0.6f;
    float MIN_DEPTH = 0.1f;
    float MAX_DEPTH = 0.5f;
    float LASER_IN_DOOR_NARROWING_FACTOR = 0.8;
    float LASER_IN_FRONT_OF_DOOR_NARROWING_FACTOR = 0.5;
    float LASER_IN_FRONT_OF_DOOR_DEPTH_AREA = 0.3;
    int MAX_LASER_IN_ZONE = 5;
    float LASER_DOORFRAME_WIDTH = 0.3;
    float LASER_DOORFRAME_DEPTH = 0.1;
    int MIN_LASER_IN_DOORFRAME = 50;
    float RATE = 10;
    float MIN_ANGLE_DIFF = 0.05;
    float MIN_DIST_DIFF = 0.05;
    float MAX_DISCARD_TIME = 2.0;

    ros::NodeHandle nh_;
    ros::Subscriber cloud_sub_;
    ros::Subscriber laser_sub_;
    ros::Publisher door_pose_pub_;
    tf::TransformListener tf_listener_;
    tf::StampedTransform last_used_transform_;

    tf::StampedTransform camera_to_base_transform_;
    tf::StampedTransform laser_to_base_transform_;
    pcl::PointCloud<pcl::PointXYZ> laser_cloud_;

    bool useCloud();
    cv::Point2f pointToPixel(const pcl::PointXYZ& p) const;
    geometry_msgs::Point pixelToPoint(const cv::Point2f& p) const;
    geometry_msgs::Pose rectToPose(const cv::RotatedRect& rect) const;
    bool isInRotatedRect(const pcl::PointXYZ& p, const cv::RotatedRect& rect) const;
    cv::RotatedRect isContourDoor(const std::vector<cv::Point>& contour) const;

  public:
    DoorDetector();

    void run();
    void cloudCb(const sensor_msgs::PointCloud2ConstPtr& msg);
    void laserCb(const sensor_msgs::LaserScanConstPtr& msg);

};

#endif //DOOR_DETECTION_DOOR_DETECT_H
