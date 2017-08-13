#include "vision/main.h"
#include <chrono>

const std::string model_file   = "/home/thomas/BVLCcaffe/models/placescnn/places205CNN_deploy.prototxt";
const std::string trained_file = "/home/thomas/BVLCcaffe/models/placescnn/places205CNN_iter_300000.caffemodel";
const std::string mean_file    = "/home/thomas/BVLCcaffe/models/placescnn/places_mean.mat";
const std::string label_file   = "/home/thomas/BVLCcaffe/models/placescnn/categoryIndex_places205.csv";

const std::string object_label_file = "/home/thomas/darknet/data/coco.names";
const std::string yolo_cfg = "/home/thomas/pyyolo/darknet/cfg/yolo.cfg";
const std::string yolo_weights = "/home/thomas/pyyolo/yolo.weights";
const float thresh = 0.1;
const float hier_thresh = 0.5;
const float nms = 0.4f;

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

  is_ok_ = true;
}

VisionApp::~VisionApp(){
  delete classifier_;
  delete detector_;
}

void VisionApp::run(){
  ros::Rate rate(4);
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
  auto begin = std::chrono::steady_clock::now();
  std::vector<std::pair<std::string, float>> predictions = classifier_->classify(img);
  std::cout << "Places in " <<std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - begin).count() << " ms" << std::endl;

  begin = std::chrono::steady_clock::now();
  std::vector<YoloDetection> detections = detector_->detect(img, thresh, hier_thresh, nms);
  std::cout << "Objects in " <<std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - begin).count() << " ms" << std::endl;

  cv::resize(img, img, cv::Size(img.cols*2, img.rows*2));

  for (size_t i = 0; i < predictions.size(); ++i)
    cv::putText(img, predictions[i].first + " " + std::to_string(predictions[i].second).substr(0, 5),
                cv::Point(0, 20 + 20*i), cv::FONT_HERSHEY_COMPLEX, 0.7, cv::Scalar(0,255,0), 1);

  for(int i=0; i<detections.size(); i++){
    detections[i].scale(2.f);
    detections[i].draw(img, cv::Scalar(255,255,255), 1, cv::Scalar(0,0,255), 0.5, 1);
  }

  cv::imshow("img", img);
  if((cv::waitKey(1) & 255) == 27)
    run_ = false;
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