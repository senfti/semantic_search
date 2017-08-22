#include "semantic_mapping/main.h"
#include <geometry_msgs/PoseArray.h>
#include <kb_query/KbQuerySrv.h>
#include <fstream>

const std::string label_file = "/home/thomas/BVLCcaffe/models/placescnn/categories_places205.csv";

SemanticMappingApp::SemanticMappingApp(ros::NodeHandle& nh)
  : nh_(nh), kb_(nh)
{
  vision_sub_ = nh_.subscribe("vision_result", 10, &SemanticMappingApp::visionCb, this);
  map_sub_ = nh_.subscribe("map", 10, &SemanticMappingApp::mapCb, this);
  vision_pose_pub_ = nh_.advertise<geometry_msgs::PoseArray>("vision_poses", 2);

  std::ifstream labels(label_file);
  if(!labels.good()){
    std::cout << "Unable to open labels file " << label_file;
    return;
  }
  std::string line;
  while (std::getline(labels, line))
    place_labels_.push_back(line);
  place_maps_.resize(place_labels_.size());
}

SemanticMappingApp::~SemanticMappingApp(){
}

void SemanticMappingApp::run(){
  ros::Rate rate(4);
  while(ros::ok()){
    ros::spinOnce();
    rate.sleep();
  }
}

void SemanticMappingApp::publishVisionPoses(const vision::VisionMsgConstPtr &msg){
  if(vision_results_.size() > 0){
    geometry_msgs::PoseArray m;
    m.header = msg->pose.header;
    m.poses.reserve(vision_results_.size());
    for(const auto &v : vision_results_){
      geometry_msgs::Pose p;
      tf::poseTFToMsg(v.pose_, p);
      m.poses.push_back(p);
    }
    vision_pose_pub_.publish(m);
  }
}

void SemanticMappingApp::updatePlaceMaps(const nav_msgs::OccupancyGridConstPtr& msg){
  int width = msg->info.width;
  int height = msg->info.height;
  float resolution = msg->info.resolution;
  float orig_x = msg->info.origin.position.x;
  float orig_y = msg->info.origin.position.y;

//  if(place_maps_[0].empty()){
//    for(auto& map : place_maps_)
//      map = cv::Mat_<float>(msg->info.height, msg->info.width, 1.f/place_maps_.size());
//  }
//  else if(place_maps_[0].rows != height || place_maps_[0].cols != width){
//    int left = static_cast<int>((old_map_data_.origin.position.x - msg->info.origin.position.x) / resolution);
//    int top = static_cast<int>((old_map_data_.origin.position.y - msg->info.origin.position.y) / resolution);
//    int right = width - place_maps_[0].cols - left;
//    int bottom = height - place_maps_[0].rows - top;
//    for(auto& map : place_maps_)
//      cv::copyMakeBorder(map, map, top, bottom, left, right, cv::BORDER_CONSTANT, cv::Scalar(1.f/place_maps_.size()));
//  }
//  old_map_data_ = msg->info;
  for(auto& map : place_maps_)
    map = cv::Mat_<float>(msg->info.height, msg->info.width, 0.f);

  for(int x=0; x<msg->info.width; x++){
    for(int y=0; y<msg->info.height; y++){
      if(msg->data[y*msg->info.width+x] >= 0){
        for(auto& map : place_maps_)
          map(y,x) = 1.f/place_maps_.size();
        for(const auto& v : vision_results_){
          float importance = static_cast<float>(v.getImportance(x*resolution + orig_x, y*resolution + orig_y));
          if(importance > 0.f){
            for(int i=0; i<v.place_guesses_.size(); i++){
              place_maps_[v.place_guesses_[i].id_](y,x) += MAP_UPDATE_FACTOR * importance * v.place_guesses_[i].prob_;
              place_maps_[v.place_guesses_[i].id_](y,x) /= (1.f + MAP_UPDATE_FACTOR * importance);
              //place_maps_[v.place_guesses_[i].id_](y,x) *= v.place_guesses_[i].prob_;
            }
          }
        }
      }
    }
  }

  cv::Mat office, corridor, kitchen, kitchenette;
  cv::Mat visible = place_maps_[129]*10000.f;
  cv::Mat tmp(visible.rows, visible.cols, CV_8UC1, cv::Scalar(255));
  cv::flip(visible, visible, 0);
  cv::imshow("visible", visible);

  place_maps_[129].convertTo(office, CV_8U, 180.f);
  cv::flip(office, office, 0);
  cv::merge(std::vector<cv::Mat>({office, tmp, tmp}), office);
  cv::cvtColor(office, office, CV_HSV2BGR);
  cv::imshow(place_labels_[129], office);

  place_maps_[54].convertTo(corridor, CV_8U, 180.f);
  cv::flip(corridor, corridor, 0);
  cv::merge(std::vector<cv::Mat>({corridor, tmp, tmp}), corridor);
  cv::cvtColor(corridor, corridor, CV_HSV2BGR);
  cv::imshow(place_labels_[54], corridor);

  place_maps_[108].convertTo(kitchen, CV_8U, 180.f);
  cv::flip(kitchen, kitchen, 0);
  cv::merge(std::vector<cv::Mat>({kitchen, tmp, tmp}), kitchen);
  cv::cvtColor(kitchen, kitchen, CV_HSV2BGR);
  cv::imshow(place_labels_[108], kitchen);

  place_maps_[109].convertTo(kitchenette, CV_8U, 180.f);
  cv::flip(kitchenette, kitchenette, 0);
  cv::merge(std::vector<cv::Mat>({kitchenette, tmp, tmp}), kitchenette);
  cv::cvtColor(kitchenette, kitchenette, CV_HSV2BGR);
  cv::imshow(place_labels_[109], kitchenette);

  cv::waitKey(1);
}

void SemanticMappingApp::visionCb(const vision::VisionMsgConstPtr& msg){
  VisionResult res = VisionResult(*msg);
  //std::cout << res;
  std::cout << res.place_guesses_[0].class_ << " " << res.place_guesses_[0].prob_ << std::endl;
  res.improve(kb_);
  //std::cout << res;
  std::cout << res.place_guesses_[0].class_ << " " << res.place_guesses_[0].prob_ << std::endl << std::endl;
  vision_results_.push_back(res);

  if(PUBLISH_VISION_POSES)
    publishVisionPoses(msg);
}

void SemanticMappingApp::mapCb(const nav_msgs::OccupancyGridConstPtr& msg){
  updatePlaceMaps(msg);
}

int main(int argc, char** argv){
  ros::init (argc, argv, "semantic_mapping");
  ros::NodeHandle nh;
  SemanticMappingApp app(nh);

  app.run();
  return 0;
}