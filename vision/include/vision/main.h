//
// Created by thomas on 13.08.17.
//

#ifndef VISION_MAIN_H
#define VISION_MAIN_H

#include "vision/CaffeClassifier.h"
#include "vision/YoloDetector.h"
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>

#include <opencv2/opencv.hpp>
#include "vision/VisionMsg.h"

class VisionApp{
  public:
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;
    ros::Publisher result_pub_;
    tf::TransformListener tf_listener_;

    CaffeClassifier* classifier_ = nullptr;
    YoloDetector* detector_ = nullptr;

    bool is_ok_ = false;
    bool run_ = true;

    VisionApp(char* exe_name, ros::NodeHandle& nh);
    ~VisionApp();

    void run();
    void imageCb(const sensor_msgs::ImageConstPtr& msg);
};

#endif //VISION_MAIN_H
