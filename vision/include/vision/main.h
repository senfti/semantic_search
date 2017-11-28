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
#include <sensor_msgs/PointCloud2.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <pcl/point_types.h>
#include <pcl/common/centroid.h>
#include <pcl_conversions/pcl_conversions.h>

#include <mutex>
#include <thread>
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
    const float donwsample_factor = 2.f;
    const float MIN_Z = 0.8;
    const float MAX_Z = 3.5;

    const int DETECTION_SAMPLE_NUM = 1000;
    const float MIN_OBJECT_PROB = 0.001f;

    const float MAX_DISCARD_TIME = 5.0;
    const float MIN_ANGLE_DIFF = 0.1;
    const float MIN_DIST_DIFF = 0.5;
    const float MAX_ROT_VELOCITY = 0.3;

    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;
    image_transport::Subscriber depth_image_sub_;
    ros::Subscriber cloud_sub_;
    ros::Publisher result_pub_;

    std::thread* nn_thread_ = nullptr;

    tf::TransformListener tf_listener_;
    tf::StampedTransform last_transform_;
    tf::StampedTransform last_used_transform_;

    CaffeClassifier* classifier_ = nullptr;
    YoloDetector* detector_ = nullptr;

    std::mutex img_mutex_;
    cv::Mat curr_img_;
    std::mutex cloud_mutex_;
    sensor_msgs::PointCloud2ConstPtr point_cloud_ = nullptr;
    cv_bridge::CvImagePtr depth_img_ = nullptr;

    bool is_ok_ = false;
    bool run_ = true;

    VisionApp(char* exe_name, ros::NodeHandle& nh);
    ~VisionApp();

    void run();
    void nnThreadRun();
    void imageCb(const sensor_msgs::ImageConstPtr& msg);
    void depthImageCb(const sensor_msgs::ImageConstPtr& msg);
    void cloudCb(const sensor_msgs::PointCloud2ConstPtr& msg);

    bool useImage(const cv::Mat& img);
    std::vector<CaffeRecognition> fillPlaceGuesses(const cv::Mat& img, vision::VisionMsg& vision_msg) const;
    void fillObjectGaussian(const pcl::PointCloud<pcl::PointXYZ>& cloud, vision::ObjectDetectionMsg &msg) const;
    std::vector<YoloDetection> fillObjectDetections(const cv::Mat& img, const pcl::PointCloud<pcl::PointXYZ>& cloud, vision::VisionMsg& vision_msg) const;
    void fillObjectDetectionSamples(const std::vector<YoloDetection>& detections, const pcl::PointCloud<pcl::PointXYZ>& cloud, vision::VisionMsg& vision_msg) const;
    void showDebugImage(cv::Mat img, std::vector<CaffeRecognition>& predictions, std::vector<YoloDetection>& detections);
};

#endif //VISION_MAIN_H
