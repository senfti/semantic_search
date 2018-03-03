#include "vision/main.h"
#include <geometry_msgs/TransformStamped.h>
#include <chrono>
#include <cmath>
#include <algorithm>
#include <cstdlib>

#define PRINT_PARAM(x) std::cout << #x << " = " << x << " " << std::endl;

VisionApp::VisionApp(char* exe_name, ros::NodeHandle& nh)
  : nh_(nh), it_(nh)
{
  ::google::InitGoogleLogging(exe_name);

  ros::NodeHandle private_nh("~");
  private_nh.param("vision/PLACE_MODEL_FILE", PLACE_MODEL_FILE, PLACE_MODEL_FILE);
  private_nh.param("vision/PLACE_TRAINED_FILE", PLACE_TRAINED_FILE, PLACE_TRAINED_FILE);
  private_nh.param("vision/PLACE_MEAN_FILE", PLACE_MEAN_FILE, PLACE_MEAN_FILE);
  private_nh.param("vision/PLACE_LABEL_FILE", PLACE_LABEL_FILE, PLACE_LABEL_FILE);

  private_nh.param("vision/OBJ_LABEL_FILE", OBJ_LABEL_FILE, OBJ_LABEL_FILE);
  private_nh.param("vision/YOLO_CFG", YOLO_CFG, YOLO_CFG);
  private_nh.param("vision/YOLO_WEIGHTS", YOLO_WEIGHTS, YOLO_WEIGHTS);
  private_nh.param("vision/OBJ_THRESH", OBJ_THRESH, OBJ_THRESH);
  private_nh.param("vision/OBJ_NMS", OBJ_NMS, OBJ_NMS);

  private_nh.param("vision/MIN_Z", MIN_Z, MIN_Z);
  private_nh.param("vision/MAX_Z", MAX_Z, MAX_Z);

  private_nh.param("vision/DETECTION_SAMPLE_NUM", DETECTION_SAMPLE_NUM, DETECTION_SAMPLE_NUM);
  private_nh.param("vision/MIN_OBJECT_PROB", MIN_OBJECT_PROB, MIN_OBJECT_PROB);

  private_nh.param("vision/MAX_DISCARD_TIME", MAX_DISCARD_TIME, MAX_DISCARD_TIME);
  private_nh.param("vision/MIN_ANGLE_DIFF", MIN_ANGLE_DIFF, MIN_ANGLE_DIFF);
  private_nh.param("vision/MIN_DIST_DIFF", MIN_DIST_DIFF, MIN_DIST_DIFF);
  private_nh.param("vision/MAX_ROT_VELOCITY", MAX_ROT_VELOCITY, MAX_ROT_VELOCITY);

  private_nh.param("vision/DEBUG_IMAGES", DEBUG_IMAGES, DEBUG_IMAGES);
  private_nh.param("vision/IMAGE_SAVE", IMAGE_SAVE, IMAGE_SAVE);

  PRINT_PARAM(PLACE_MODEL_FILE)
  PRINT_PARAM(PLACE_TRAINED_FILE)
  PRINT_PARAM(PLACE_MEAN_FILE)
  PRINT_PARAM(PLACE_LABEL_FILE)
  PRINT_PARAM(OBJ_LABEL_FILE)
  PRINT_PARAM(YOLO_CFG)
  PRINT_PARAM(YOLO_WEIGHTS)
  PRINT_PARAM(OBJ_THRESH)
  PRINT_PARAM(OBJ_NMS)
  PRINT_PARAM(MIN_Z)
  PRINT_PARAM(MAX_Z)
  PRINT_PARAM(DETECTION_SAMPLE_NUM)
  PRINT_PARAM(MIN_OBJECT_PROB)
  PRINT_PARAM(MAX_DISCARD_TIME)
  PRINT_PARAM(MIN_ANGLE_DIFF)
  PRINT_PARAM(MIN_DIST_DIFF)
  PRINT_PARAM(MAX_ROT_VELOCITY)
  PRINT_PARAM(DEBUG_IMAGES)
  PRINT_PARAM(IMAGE_SAVE)

  auto begin = std::chrono::steady_clock::now();
  classifier_ = new CaffeClassifier(PLACE_MODEL_FILE, PLACE_TRAINED_FILE, PLACE_MEAN_FILE, PLACE_LABEL_FILE);
  if(!classifier_->isOk()){
    return;
  }
  std::cout << "Places init in " <<std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - begin).count() << " ms" << std::endl;

  begin = std::chrono::steady_clock::now();
  detector_ = new YoloDetector(OBJ_LABEL_FILE, YOLO_CFG, YOLO_WEIGHTS);
  if(!detector_->isOk()){
    return;
  }
  std::cout << "Yolo init in " <<std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - begin).count() << " ms" << std::endl;

  image_sub_ = it_.subscribe("/camera/rgb/image_raw", 1, &VisionApp::imageCb, this);
  depth_image_sub_ = it_.subscribe("/camera/depth_registered/image_raw", 1, &VisionApp::depthImageCb, this);
  cloud_sub_ = nh_.subscribe("/camera/depth_registered/points", 1, &VisionApp::cloudCb, this);
  result_pub_ = nh_.advertise<vision::VisionMsg>("vision_result", 10);

  is_ok_ = true;
  std::srand(ros::Time::now().nsec);
}

VisionApp::~VisionApp(){
  delete classifier_;
  delete detector_;
}

void VisionApp::run(){
  ros::Rate rate(10);
  nn_thread_ = new std::thread(&VisionApp::nnThreadRun, this);
  while(ros::ok() && run_){
    ros::spinOnce();
    rate.sleep();
  }
  run_ = false;
  nn_thread_->join();
  delete nn_thread_;
}

void VisionApp::nnThreadRun(){
  ros::Rate rate(100);
  while(ros::ok() && run_){
    std::unique_lock<std::mutex> lock(img_mutex_);
    if(curr_img_.rows > 0 && point_cloud_){
      cv::Mat img;
      curr_img_.copyTo(img);
      curr_img_ = cv::Mat();
      lock.unlock();

      vision::VisionMsg result_msg;
      std::unique_lock<std::mutex> lock(cloud_mutex_);
      pcl::PointCloud<pcl::PointXYZ> cloud;
      pcl::fromROSMsg(*point_cloud_, cloud);
      static int seq = 0;
      result_msg.header.stamp = point_cloud_->header.stamp;
      result_msg.header.seq = seq++;
      result_msg.header.frame_id = point_cloud_->header.frame_id;
      lock.unlock();

      std::vector<CaffeRecognition> predictions = fillPlaceGuesses(img, result_msg);
      std::vector<YoloDetection> detections = fillObjectDetections(img, cloud, result_msg);
      result_pub_.publish(result_msg);
      if(DEBUG_IMAGES)
        showDebugImage(img, predictions, detections);
    }
    else
      lock.unlock();

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

  std::lock_guard<std::mutex> lock(img_mutex_);
  if(useImage(cv_ptr->image, msg->header.stamp));
    cv_ptr->image.copyTo(curr_img_);
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
  std::lock_guard<std::mutex> lock(cloud_mutex_);
  point_cloud_ = msg;
}


inline double makeBetweenPi(double angle){
  while(angle > M_PI)
    angle -= 2*M_PI;
  while(angle < -M_PI)
    angle += 2*M_PI;
  return angle;
}


bool VisionApp::useImage(const cv::Mat& img, ros::Time stamp){
  if(!depth_img_ || !point_cloud_){
    std::cout << "No depth image" << std::endl;
    return false;
  }

  tf::StampedTransform transform;
  try{
    tf_listener_.waitForTransform("base_link", "odom", stamp, ros::Duration(0.2));
    tf_listener_.lookupTransform("base_link", "odom", stamp, transform);
  }
  catch (tf::TransformException& ex){
    ROS_ERROR("%s", ex.what());
    return false;
  }

  if(last_transform_.frame_id_.empty()){
    last_transform_ = transform;
    last_used_transform_ = transform;
    return true;
  }

  double angle = tf::getYaw(transform.getRotation()) - tf::getYaw(last_used_transform_.getRotation());
  angle = makeBetweenPi(angle);
  double dist = (transform.getOrigin() - last_used_transform_.getOrigin()).length();
  double ang_vel = tf::getYaw(transform.getRotation()) - tf::getYaw(last_transform_.getRotation());
  ang_vel = std::abs(makeBetweenPi(ang_vel));
  ang_vel /= (transform.stamp_ - last_transform_.stamp_).toSec();
  last_transform_ = transform;

  if(std::abs(angle) < MIN_ANGLE_DIFF && dist < MIN_DIST_DIFF && (last_used_transform_.stamp_ - transform.stamp_).toSec() < MAX_DISCARD_TIME){
    last_transform_ = transform;
    std::cout << "NOTHING CHANGED" << std::endl;
    return false;
  }

  if(ang_vel > MAX_ROT_VELOCITY){
    last_transform_ = transform;
    std::cout << "ROTATING TOO FAST" << std::endl;
    return false;
  }

  last_transform_ = transform;
  last_used_transform_ = transform;
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


inline bool isSampleInDetection(int x, int y, const YoloDetection& detection){
  return (x>detection.x1_ && x<detection.x2_ && y>detection.y1_ && y<detection.y2_);
}


std::vector<YoloDetection> VisionApp::fillObjectDetections(const cv::Mat& img, const pcl::PointCloud<pcl::PointXYZ>& cloud, vision::VisionMsg& vision_msg) const{
  auto begin = std::chrono::steady_clock::now();
  std::vector<YoloDetection> detections = detector_->detect(img, OBJ_THRESH, 0.5, OBJ_NMS);

  auto t1 = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - begin).count();
  vision_msg.objects.num_objects = NUM_OBJECT_TYPES;
  vision_msg.objects.num_boxes = detections.size();
  vision_msg.objects.probs.resize(NUM_OBJECT_TYPES*detections.size());
  for(int d=0; d<detections.size(); d++){
    for(int o=0; o<NUM_OBJECT_TYPES; o++){
      vision_msg.objects.probs[d*NUM_OBJECT_TYPES+o] = detections[d].prob_[o];
    }
  }

  int i = 0;
  for(int c=0; i<DETECTION_SAMPLE_NUM && c < 5*DETECTION_SAMPLE_NUM; c++){
    int x=std::rand()%cloud.width;
    int y=std::rand()%cloud.height;
    if(!(cloud.at(x,y).z >= MIN_Z && cloud.at(x,y).z <= MAX_Z))
      continue;

    //mask(y,x) = 255;
    vision::DetectionSample sample;
    sample.point.x = cloud.at(x,y).x;
    sample.point.y = cloud.at(x,y).y;
    sample.point.z = cloud.at(x,y).z;

    for(int d=0; d<detections.size(); d++){
      if(isSampleInDetection(x,y,detections[d])){
        sample.boxes.push_back(int16_t(d));
      }
    }
    vision_msg.objects.samples.push_back(sample);
    i++;
  }

  std::cout << detections.size() << " Objects in " << t1 << "/" << std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - begin).count() - t1 << " ms" << std::endl;

  return detections;
}

void visualizeProbMap(cv::Mat_<double> map, const std::string& win_name, cv::Mat_<uchar> mask = cv::Mat_<uchar>()){
  cv::Mat out;
  map.convertTo(out, CV_8U, 120.f);
  //cv::flip(out, out, 0);
  cv::Mat tmp(out.rows, out.cols, CV_8UC1, cv::Scalar(255));
  if(mask.rows > 0)
    tmp = mask;
  cv::merge(std::vector<cv::Mat>({out, tmp, tmp}), out);
  cv::cvtColor(out, out, CV_HSV2BGR);
  cv::imshow(win_name, out);
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
  if(IMAGE_SAVE){
    static int sdf = 0;
    cv::imwrite("/tmp/" + std::to_string(sdf) + ".png", debug_img);
    sdf++;
  }
  uchar key = cv::waitKey(1) & 255;
  if(key == 27)
    run_ = false;
//  else if(key == 'a')
//    thresh += 0.05;
//  else if(key == 'y')
//    thresh -= 0.05;
//  else if(key == 's')
//    nms += 0.05;
//  else if(key == 'x')
//    nms -= 0.05;
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

