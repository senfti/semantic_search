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
    std::string PLACE_MODEL_FILE   = "/home/thomas/BVLCcaffe/models/placescnn/places205CNN_deploy.prototxt";
    std::string PLACE_TRAINED_FILE = "/home/thomas/BVLCcaffe/models/placescnn/places205CNN_iter_300000.caffemodel";
    std::string PLACE_MEAN_FILE    = "/home/thomas/BVLCcaffe/models/placescnn/places_mean.mat";
    std::string PLACE_LABEL_FILE   = "/home/thomas/BVLCcaffe/models/placescnn/categories_places205.csv";

    std::string OBJ_LABEL_FILE = "/home/thomas/darknet/data/coco.names";
    std::string YOLO_CFG = "/home/thomas/darknet/cfg/yolo.cfg";
    std::string YOLO_WEIGHTS = "/home/thomas/darknet/data/yolo.weights";
    std::string RESULT_TOPIC = "vision_result";
    float OBJ_THRESH = 0.01f;
    float OBJ_NMS = 0.8f;

    float MIN_Z = 0.8;
    float MAX_Z = 3.5;

    int DETECTION_SAMPLE_NUM = 2000;
    float MIN_OBJECT_PROB = 0.001f;

    float MAX_DISCARD_TIME = 5.0;
    float MIN_ANGLE_DIFF = 0.1;
    float MIN_DIST_DIFF = 0.5;
    float MAX_ROT_VELOCITY = 0.3;

    int DEBUG_IMAGES = 1;
    int IMAGE_SAVE = 0;

    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;
    //image_transport::Subscriber depth_image_sub_;
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
    //cv_bridge::CvImagePtr depth_img_ = nullptr;

    bool is_ok_ = false;
    bool run_ = true;

    VisionApp(char* exe_name, ros::NodeHandle& nh);
    ~VisionApp();

    void run();
    void nnThreadRun();
    void imageCb(const sensor_msgs::ImageConstPtr& msg);
    //void depthImageCb(const sensor_msgs::ImageConstPtr& msg);
    void cloudCb(const sensor_msgs::PointCloud2ConstPtr& msg);

    bool useImage(const cv::Mat& img, ros::Time stamp);
    std::vector<CaffeRecognition> fillPlaceGuesses(const cv::Mat& img, vision::VisionMsg& vision_msg) const;
    std::vector<YoloDetection> fillObjectDetections(const cv::Mat& img, const pcl::PointCloud<pcl::PointXYZ>& cloud, vision::VisionMsg& vision_msg) const;
    void showDebugImage(cv::Mat img, std::vector<CaffeRecognition>& predictions, std::vector<YoloDetection>& detections);
};

#endif //VISION_MAIN_H
