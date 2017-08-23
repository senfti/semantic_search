#include "semantic_mapping/main.h"
#include <geometry_msgs/PoseArray.h>
#include <kb_query/KbQuerySrv.h>
#include <fstream>
#include <chrono>

const std::string label_file = "/home/thomas/BVLCcaffe/models/placescnn/categories_places205.csv";

SemanticMappingApp::SemanticMappingApp(ros::NodeHandle& nh)
  : nh_(nh), kb_(nh)
{
  vision_sub_ = nh_.subscribe("vision_result", 100, &SemanticMappingApp::visionCb, this);
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
  static int i=-1;
  i++;
  if(i%5)
    return;
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
        std::vector<double> log_probs(place_labels_.size(), -std::log2(double(place_maps_.size())));
        for(const auto& v : vision_results_){
          float importance = static_cast<float>(v.getImportance(x*resolution + orig_x, y*resolution + orig_y));
          if(importance > 0.0){
            for(int i=0; i<v.place_guesses_.size(); i++){
              log_probs[v.place_guesses_[i].id_] += std::log(v.place_guesses_[i].prob_);
            }
          }
        }
        double sum_probs = 0.0;
        for(int i=0; i<place_labels_.size(); i++){
          log_probs[i] = std::pow(2, log_probs[i]);
          sum_probs += log_probs[i];
        }
        for(int i=0; i<place_labels_.size(); i++){
          place_maps_[i](y, x) = float(log_probs[i] / sum_probs);
        }
      }
    }
  }

  cv::Mat visible = place_maps_[0]*10000.f;
  for(const auto& p : place_maps_)
    visible = visible + p;
  cv::flip(visible, visible, 0);
  cv::imshow("visible", visible);
  cv::imwrite("/tmp/visible.png", visible);
  cv::Mat tmp(place_maps_[0].rows, place_maps_[0].cols, CV_8UC1, cv::Scalar(255));
  for(int i=0; i<place_maps_.size(); i++){
    cv::Mat out;
    place_maps_[i].convertTo(out, CV_8U, 150.f);
    cv::flip(out, out, 0);
    cv::merge(std::vector<cv::Mat>({out, tmp, tmp}), out);
    cv::cvtColor(out, out, CV_HSV2BGR);
    cv::imwrite("/tmp/" + place_labels_[i] + ".png", out);
    if(!place_labels_[i].compare("office") || !place_labels_[i].compare("corridor") || !place_labels_[i].compare("kitchen") || !place_labels_[i].compare("kitchenette"))
      cv::imshow(place_labels_[i], out);
  }
  cv::waitKey(1);
}

void SemanticMappingApp::visionCb(const vision::VisionMsgConstPtr& msg){
  auto begin = std::chrono::steady_clock::now();
  VisionResult res = VisionResult(*msg);
  //std::cout << res;
  //std::cout << res.place_guesses_[0].class_ << " " << res.place_guesses_[0].prob_ << std::endl;
  res.improve(kb_);
  //std::cout << res;
  //std::cout << res.place_guesses_[0].class_ << " " << res.place_guesses_[0].prob_ << std::endl << std::endl;
  vision_results_.push_back(res);

  if(PUBLISH_VISION_POSES)
    publishVisionPoses(msg);
  //std::cout << "Vision input processing in " <<std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - begin).count() << " ms" << std::endl;
}

void SemanticMappingApp::mapCb(const nav_msgs::OccupancyGridConstPtr& msg){
  auto begin = std::chrono::steady_clock::now();
  updatePlaceMaps(msg);
  std::cout << "Map update in " <<std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - begin).count() << " ms" << std::endl;
}

int main(int argc, char** argv){
  ros::init (argc, argv, "semantic_mapping");
  ros::NodeHandle nh;
  SemanticMappingApp app(nh);

  app.run();
  return 0;
}