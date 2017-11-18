#include "vision/main.h"
#include <geometry_msgs/TransformStamped.h>
#include <chrono>
#include <cmath>
#include <algorithm>

VisionApp::VisionApp(char* exe_name, ros::NodeHandle& nh)
  : nh_(nh), it_(nh)
{
  ::google::InitGoogleLogging(exe_name);

  auto begin = std::chrono::steady_clock::now();
  classifier_ = new CaffeClassifier(model_file, trained_file, mean_file, label_file);
  if(!classifier_->isOk()){
    return;
  }
  std::cout << "Places init in " <<std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - begin).count() << " ms" << std::endl;

  begin = std::chrono::steady_clock::now();
  detector_ = new YoloDetector(object_label_file, yolo_cfg, yolo_weights);
  if(!detector_->isOk()){
    return;
  }
  std::cout << "Yolo init in " <<std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - begin).count() << " ms" << std::endl;

  image_sub_ = it_.subscribe("/camera/rgb/image_raw", 1, &VisionApp::imageCb, this);
  depth_image_sub_ = it_.subscribe("/camera/depth_registered/image_raw", 1, &VisionApp::depthImageCb, this);
  cloud_sub_ = nh_.subscribe("/camera/depth_registered/points", 1, &VisionApp::cloudCb, this);
  result_pub_ = nh_.advertise<vision::VisionMsg>("vision_result", 10);

  is_ok_ = true;
}

VisionApp::~VisionApp(){
  delete classifier_;
  delete detector_;
}

void VisionApp::run(){
  ros::Rate rate(2);
  while(ros::ok() && run_){
    ros::spinOnce();
    rate.sleep();
  }
}

void VisionApp::imageCb(const sensor_msgs::ImageConstPtr &msg){
  cv_bridge::CvImagePtr cv_ptr;
  try{
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  }
  catch (cv_bridge::Exception& e){
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }
  cv::Mat img;
  cv_ptr->image.copyTo(img);

  if(!useImage(img))
    return;

  vision::VisionMsg result_msg;
  std::vector<CaffeRecognition> predictions = fillPlaceGuesses(img, result_msg);
  std::vector<YoloDetection> detections = fillObjectDetections(img, result_msg);

  result_pub_.publish(result_msg);
  showDebugImage(img, predictions, detections);
}

void VisionApp::depthImageCb(const sensor_msgs::ImageConstPtr &msg){
  try{
    depth_img_ = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_32FC1);
  }
  catch (cv_bridge::Exception& e){
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }
}


void VisionApp::cloudCb(const sensor_msgs::PointCloud2ConstPtr &msg){
  point_cloud_ = msg;
}


bool VisionApp::useImage(const cv::Mat& img){
  if(!depth_img_ || !point_cloud_){
    std::cout << "No depth image" << std::endl;
    return false;
  }

  return true;
}


std::vector<CaffeRecognition> VisionApp::fillPlaceGuesses(const cv::Mat& img, vision::VisionMsg& vision_msg) const{
  auto begin = std::chrono::steady_clock::now();
  std::vector<CaffeRecognition> predictions = classifier_->classify(img);

  for(int i=0; i<predictions.size(); i++){
    vision::PlaceRecognitionMsg place_msg;
    place_msg.name = predictions[i].label_;
    place_msg.id = predictions[i].id_;
    place_msg.prob = predictions[i].prob_;
    vision_msg.place_guesses.push_back(place_msg);
  }
  std::cout << "Places in " <<std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - begin).count() << " ms" << std::endl;

  return predictions;
}


void VisionApp::fillObjectGaussian(const pcl::PointCloud<pcl::PointXYZ>& cloud, vision::ObjectDetectionMsg &msg) const{
  int x1=std::round(msg.x1), x2=std::round(msg.x2), y1=std::round(msg.y1), y2=std::round(msg.y2);
  int num_points = (x2-x1+1)*(y2-y1+1);
  pcl::PointCloud<pcl::PointXYZ> sub_cloud;
  sub_cloud.is_dense = true;
  sub_cloud.reserve(num_points);
  int i = 0;
  for(int x=msg.x1; x<=msg.x2; x++){
    for(int y=msg.y1; y<=msg.y2; y++){
      if(pcl::isFinite(cloud.at(x, y)))
        sub_cloud.push_back(cloud.at(x,y));
    }
  }

  Eigen::Vector4f mean;
  Eigen::Matrix3f covariance;
  pcl::computeMeanAndCovarianceMatrix(sub_cloud, covariance, mean);
  msg.center = std::vector<float>(mean.data(), mean.data()+3);
  msg.covariance = std::vector<float>(covariance.data(), covariance.data()+3*3);
}


std::vector<YoloDetection> VisionApp::fillObjectDetections(const cv::Mat& img, vision::VisionMsg& vision_msg) const{
  auto begin = std::chrono::steady_clock::now();
  pcl::PointCloud<pcl::PointXYZ> cloud;
  pcl::fromROSMsg(*point_cloud_, cloud);
  std::vector<YoloDetection> detections = detector_->detect(img, thresh, hier_thresh, nms);

  auto t1 = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - begin).count();
  vision_msg.objects.resize(detections.size());
  for(int i=0; i<detections.size(); i++){
    vision_msg.objects[i] = detections[i].getAsMsg();
    fillObjectGaussian(cloud, vision_msg.objects[i]);
  }

  std::cout << detections.size() << " Objects in " << t1 << "/" << std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - begin).count() - t1 << " ms" << std::endl;

  return detections;
}


void VisionApp::showDebugImage(cv::Mat img, std::vector<CaffeRecognition>& predictions, std::vector<YoloDetection>& detections){
  float scale_factor = 2.f;
  auto begin = std::chrono::steady_clock::now();
  std::sort(predictions.begin(), predictions.end(), probGreater);
  cv::Mat debug_img;
  cv::resize(img, debug_img, cv::Size(img.cols*scale_factor, img.rows*scale_factor));
  for (size_t i = 0; i < predictions.size() && predictions[i].prob_ > 0.05; ++i)
    cv::putText(debug_img, predictions[i].label_ + " " + std::to_string(predictions[i].prob_).substr(0, 5),
                cv::Point(0, 20 + 20*i), cv::FONT_HERSHEY_COMPLEX, 0.7, cv::Scalar(0,255,0), 1);

  for(int i=0; i<detections.size(); i++){
    if(*std::max_element(detections[i].prob_.begin(), detections[i].prob_.end()) > 0.1){
      detections[i].scale(2.f);
      detections[i].draw(debug_img, cv::Scalar(255, 255, 255), 1, cv::Scalar(0, 0, 255), 0.5, 1);
    }
  }
  cv::imshow("img", debug_img);
  uchar key = cv::waitKey(1) & 255;
  if(key == 27)
    run_ = false;
  else if(key == 'a')
    thresh += 0.05;
  else if(key == 'y')
    thresh -= 0.05;
  else if(key == 's')
    nms += 0.05;
  else if(key == 'x')
    nms -= 0.05;
  std::cout << "Debug Images in " <<std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - begin).count() << " ms" << std::endl;
}


int main(int argc, char** argv){
  ros::init (argc, argv, "vision");
  ros::NodeHandle nh;
  VisionApp vision_app(argv[0], nh);
  if(!vision_app.is_ok_)
    return -1;

  vision_app.run();
  return 0;
}

