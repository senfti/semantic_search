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
  if(!fillTransform(result_msg))
    return;
  std::vector<CaffeRecognition> predictions = fillPlaceGuesses(img, result_msg);
  std::vector<YoloDetection> detections = fillObjectDetections(img, result_msg);
  fillViewDistances(result_msg);

  result_pub_.publish(result_msg);
  old_img_ = img;
  showDebugImage(img, predictions, detections);
}

void VisionApp::depthImageCb(const sensor_msgs::ImageConstPtr &msg){
  cv_bridge::CvImagePtr cv_ptr;
  try{
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_32FC1);
  }
  catch (cv_bridge::Exception& e){
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  cv::Mat img;
  cv_ptr->image.copyTo(depth_img_);
}


bool VisionApp::useImage(const cv::Mat& img){
  if(depth_img_.empty()){
    std::cout << "No depth image" << std::endl;
    return false;
  }
  if(old_gradients_.empty()){
    return true;
  }

  // discarding not moved images
  cv::Mat working;
  cv::resize(img, working, cv::Size(img.cols / 4, img.rows / 4));
  cv::blur(working, working, cv::Size(5,5));
  cv::Mat dx, dy, gradient;
  cv::Sobel(img, dx, CV_32F, 1, 0, 3);
  cv::Sobel(img, dy, CV_32F, 0, 1, 3);
  cv::add(dx, dy, gradient);
  cv::normalize(gradient, gradient, 0.0, 1.0, cv::NORM_MINMAX);
  cv::Mat diff;
  cv::subtract(gradient, old_gradients_, diff);
  old_gradients_ = gradient;
  cv::Mat pow;
  cv::pow(diff, 6, pow);
  cv::Scalar sum = cv::sum(pow);
  float psnr = log(sum.val[0] / diff.size().area());
  if(psnr < psnr_thresh)
    return false;

  return true;
}


bool VisionApp::fillTransform(vision::VisionMsg& vision_msg) const {
  try{
    tf::StampedTransform transform;
    tf_listener_.lookupTransform("map", "base_link", ros::Time(0), transform);
    tf::Stamped<tf::Pose> tmp;
    tmp.stamp_ = transform.stamp_;
    tmp.frame_id_ = transform.frame_id_;
    tmp.setOrigin(transform.getOrigin());
    tmp.setRotation(transform.getRotation());
    tf::poseStampedTFToMsg(tmp, vision_msg.pose);
  }
  catch(std::exception& e){
    std::cout << e.what() << std::endl;
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


std::vector<YoloDetection> VisionApp::fillObjectDetections(const cv::Mat& img, vision::VisionMsg& vision_msg) const{
  auto begin = std::chrono::steady_clock::now();
  std::vector<YoloDetection> detections = detector_->detect(img, thresh, hier_thresh, nms);

  cv::Mat_<float> depth_filtered;
  depth_img_.copyTo(depth_filtered);
  for(int x=0; x<depth_img_.cols; x++){
    for(int y=0; y<depth_img_.rows; y++){
      if(!std::isfinite(depth_filtered.at<float>(y,x)))
        depth_filtered(y,x) = 5.f;
    }
  }

  cv::GaussianBlur(depth_filtered, depth_filtered, cv::Size(7,7), 2.0, 2.0);
  for(const auto& d : detections){
    vision::ObjectDetectionMsg object_msg;
    object_msg.name = d.label_;
    object_msg.id = d.id_;
    object_msg.prob = d.prob_;

    cv::Mat tmp = depth_filtered((cv::Rect(cv::Point(std::max(int(d.x1_),0), std::max(int(d.y1_),0)),
                                       cv::Point(std::min(int(d.x2_),depth_img_.cols-1), std::min(int(d.y2_),depth_img_.rows-1)))));
    double z_mean = cv::mean(tmp)[0];
    double z_min, z_max;
    cv::minMaxIdx(tmp, &z_min, &z_max);

    object_msg.x1 = (d.x1_/img.cols - 0.5) * horizontal_camera_spread_ * z_mean;
    object_msg.x2 = (d.x2_/img.cols - 0.5) * horizontal_camera_spread_ * z_mean;
    object_msg.y1 = (d.y1_/img.rows - 0.5) * vertical_camera_spread_ * z_mean;
    object_msg.y2 = (d.y2_/img.rows - 0.5) * vertical_camera_spread_ * z_mean;
    object_msg.z1 = z_min;
    object_msg.z2 = z_max;
    vision_msg.objects.push_back(object_msg);

    if(!std::isfinite(z_mean))
      std::cout << "sldkfjsdlkfj" << std::endl;
  }

  std::cout << "Objects in " <<std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - begin).count() << " ms" << std::endl;

  return detections;
}


void VisionApp::fillViewDistances(vision::VisionMsg& vision_msg) const{
  vision_msg.view_dists.resize(VIEW_DIST_SEGMENTS);
  cv::Mat dist_mat;
  cv::medianBlur(depth_img_, dist_mat, 5);
  int pixel_per_seg = depth_img_.cols / VIEW_DIST_SEGMENTS;
  for(int i=0; i<VIEW_DIST_SEGMENTS; i++){
    double mi, ma;
    cv::minMaxIdx(depth_img_(cv::Rect(i*pixel_per_seg, 0, pixel_per_seg, depth_img_.rows)), &mi, &ma);
    vision_msg.view_dists[i] = ma;
  }
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

