#include "semantic_mapping/main.h"

SemanticMappingApp::SemanticMappingApp(ros::NodeHandle& nh)
  : nh_(nh)
{
  vision_sub_ = nh_.subscribe("vision_result", 10, &SemanticMappingApp::visionCb, this);
  map_sub_ = nh_.subscribe("map", 10, &SemanticMappingApp::mapCb, this);
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

void SemanticMappingApp::visionCb(const vision::VisionMsgConstPtr& msg){
  vision_msgs_.push_back(*msg);
}

void SemanticMappingApp::mapCb(const nav_msgs::OccupancyGridConstPtr& msg){
  std::cout << "new map: " << std::endl;
  static int i=0;
  for(auto& m : vision_msgs_){
    std::cout << ++i << ": " << m.pose.header.stamp << ", " << m.pose.pose.position.x << ", " << m.pose.pose.position.y << ", " << m.pose.pose.position.z << " " << std::endl;
    for(auto& p : m.place_proposals)
      std::cout << p.name << " " << std::to_string(p.prob).substr(0, 5) << "; ";
    std::cout << std::endl;
    for(auto& o : m.objects)
      std::cout << o.name << " " << std::to_string(o.prob).substr(0, 5) << "; ";
    std::cout << std::endl;
  }
  std::cout << std::endl << std::endl;
  vision_msgs_.clear();
}

int main(int argc, char** argv){
  ros::init (argc, argv, "semantic_mapping");
  ros::NodeHandle nh;
  SemanticMappingApp app(nh);

  app.run();
  return 0;
}