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

class DoorDetector{
  private:
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;
    image_transport::Subscriber depth_image_sub_;
    ros::Subscriber cloud_sub_;
    ros::Subscriber laser_sub_;
    ros::Publisher tmp_cloud_pub_;
    ros::Publisher image_pub_;
    tf::TransformListener tf_listener_;

    cv::Mat depth_img_;
    tf::StampedTransform camera_to_base_transform_;

  public:
    DoorDetector();

    void run();
    void imageCb(const sensor_msgs::ImageConstPtr& msg);
    void depthImageCb(const sensor_msgs::ImageConstPtr& msg);
    void cloudCb(const sensor_msgs::PointCloud2ConstPtr& msg);
    void laserCb(const sensor_msgs::LaserScanConstPtr& msg);

};

#endif //DOOR_DETECTION_DOOR_DETECT_H
