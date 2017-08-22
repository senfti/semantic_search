#include "vision/main.h"
#include <geometry_msgs/TransformStamped.h>
#include <chrono>

const std::string model_file   = "/home/thomas/BVLCcaffe/models/placescnn/places205CNN_deploy.prototxt";
const std::string trained_file = "/home/thomas/BVLCcaffe/models/placescnn/places205CNN_iter_300000.caffemodel";
const std::string mean_file    = "/home/thomas/BVLCcaffe/models/placescnn/places_mean.mat";
const std::string label_file   = "/home/thomas/BVLCcaffe/models/placescnn/categories_places205.csv";
//const int num_place_proposals = 5;
const float min_place_prob = 0.0;

const std::string object_label_file = "/home/thomas/darknet/data/coco.names";
const std::string yolo_cfg = "/home/thomas/darknet/cfg/yolo.cfg";
const std::string yolo_weights = "/home/thomas/darknet/data/yolo.weights";
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
  result_pub_ = nh_.advertise<vision::VisionMsg>("vision_result", 10);

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
  std::vector<CaffeRecognition> predictions = classifier_->classify(img, min_place_prob);
  std::cout << "Places in " <<std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - begin).count() << " ms" << std::endl;

  begin = std::chrono::steady_clock::now();
  std::vector<YoloDetection> detections = detector_->detect(img, thresh, hier_thresh, nms);
  std::cout << "Objects in " <<std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - begin).count() << " ms" << std::endl;

  cv::resize(img, img, cv::Size(img.cols*2, img.rows*2));

  for (size_t i = 0; i < predictions.size(); ++i)
    cv::putText(img, predictions[i].label_ + " " + std::to_string(predictions[i].prob_).substr(0, 5),
                cv::Point(0, 20 + 20*i), cv::FONT_HERSHEY_COMPLEX, 0.7, cv::Scalar(0,255,0), 1);

  for(int i=0; i<detections.size(); i++){
    detections[i].scale(2.f);
    detections[i].draw(img, cv::Scalar(255,255,255), 1, cv::Scalar(0,0,255), 0.5, 1);
  }

  vision::VisionMsg result_msg;
  for(int i=0; i<predictions.size(); i++){
    vision::PlaceRecognitionMsg place_msg;
    place_msg.name = predictions[i].label_;
    place_msg.id = predictions[i].id_;
    place_msg.prob = predictions[i].prob_;
    result_msg.place_guesses.push_back(place_msg);
  }
  for(int i=0; i<detections.size(); i++){
    vision::ObjectDetectionMsg object_msg;
    object_msg.name = detections[i].label_;
    object_msg.id = detections[i].id_;
    object_msg.prob = detections[i].prob_;
    object_msg.x1 = detections[i].x1_;
    object_msg.x2 = detections[i].x2_;
    object_msg.y1 = detections[i].y1_;
    object_msg.y2 = detections[i].y1_;
    result_msg.objects.push_back(object_msg);
  }

  try{
    tf::StampedTransform transform;
    tf_listener_.lookupTransform("map", "base_link", ros::Time(0), transform);
    tf::Stamped<tf::Pose> tmp;
    tmp.stamp_ = transform.stamp_;
    tmp.frame_id_ = transform.frame_id_;
    tmp.setOrigin(transform.getOrigin());
    tmp.setRotation(transform.getRotation());
    tf::poseStampedTFToMsg(tmp, result_msg.pose);
  }
  catch(std::exception& e){
    std::cout << e.what() << std::endl;
  }

  result_pub_.publish(result_msg);

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