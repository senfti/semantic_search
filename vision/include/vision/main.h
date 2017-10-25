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
    const std::string model_file   = "/home/thomas/BVLCcaffe/models/placescnn/places205CNN_deploy.prototxt";
    const std::string trained_file = "/home/thomas/BVLCcaffe/models/placescnn/places205CNN_iter_300000.caffemodel";
    const std::string mean_file    = "/home/thomas/BVLCcaffe/models/placescnn/places_mean.mat";
    const std::string label_file   = "/home/thomas/BVLCcaffe/models/placescnn/categories_places205.csv";
    //const int num_place_proposals = 5;
    const float min_place_prob = 0.f;

    const std::string object_label_file = "/home/thomas/darknet/data/coco.names";
    const std::string yolo_cfg = "/home/thomas/darknet/cfg/yolo.cfg";
    const std::string yolo_weights = "/home/thomas/darknet/data/yolo.weights";
    float thresh = 0.01f;
    float hier_thresh = 0.5f;
    float nms = 0.8f;

    const int VIEW_DIST_SEGMENTS = 32;
    const float horizontal_camera_spread_ = 1.1;                    // parameters of xtion pro: position in world = (pixel/num_pixels - 0.5) * camera_spread * distance
    const float vertical_camera_spread_ = 0.825;
    const float dist_cutoff_ = 0.1;                                 // only use dist_cutoff to (1-dist_cutoff) depth values for depth estimate
    const float psnr_thresh = 4.f;

    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;
    image_transport::Subscriber depth_image_sub_;
    ros::Publisher result_pub_;
    tf::TransformListener tf_listener_;

    CaffeClassifier* classifier_ = nullptr;
    YoloDetector* detector_ = nullptr;

    cv::Mat depth_img_;
    cv::Mat old_img_;
    cv::Mat old_gradients_;

    bool is_ok_ = false;
    bool run_ = true;

    VisionApp(char* exe_name, ros::NodeHandle& nh);
    ~VisionApp();

    void run();
    void imageCb(const sensor_msgs::ImageConstPtr& msg);
    void depthImageCb(const sensor_msgs::ImageConstPtr& msg);

    bool useImage(const cv::Mat& img);
    bool fillTransform(vision::VisionMsg& vision_msg) const;
    std::vector<CaffeRecognition> fillPlaceGuesses(const cv::Mat& img, vision::VisionMsg& vision_msg) const;
    std::vector<YoloDetection> fillObjectDetections(const cv::Mat& img, vision::VisionMsg& vision_msg) const;
    void fillViewDistances(vision::VisionMsg& vision_msg) const;
    void showDebugImage(cv::Mat img, std::vector<CaffeRecognition>& predictions, std::vector<YoloDetection>& detections);
};

#endif //VISION_MAIN_H
