//
// Created by thomas on 15.11.17.
//

#include "semantic_mapping_v2/ObjectMapper.h"
#include <pcl/common/common.h>
#include <pcl/filters/passthrough.h>
#include <semantic_mapping_v2/OctoMapper.h>

#include <fstream>

std::vector<std::string> ObjectMapper::getObjNames(){
  static std::vector<std::string> OBJ_NAMES;
  ros::NodeHandle private_nh("~");
  std::string obj_name_file = "/home/thomas/darknet/data/coco.names";
  private_nh.param("ObjectMapper/OBJ_NAME_FILE", obj_name_file, obj_name_file);
  if(OBJ_NAMES.empty()){
    std::ifstream f(obj_name_file);
    if(!f.good()){
      ROS_WARN("CANNOT OPEN OBJECT NAME FILE %s", obj_name_file.c_str());
    }
    while(!f.eof()){
      OBJ_NAMES.push_back("");
      std::getline(f, *OBJ_NAMES.rbegin());
    }
  }
  return OBJ_NAMES;
}

std::string ObjectMapper::getObjName(int idx) {
  std::vector<std::string> names = getObjNames();
  return (idx < names.size() ? names[idx] : std::to_string(idx));
}

ObjectMap::ObjectMap(float resolution, float start_size, float max_height, float initial_value)
    : resolution_(resolution), base_size_(start_size*resolution), max_height_(max_height)
{
  origin_ = cv::Point(base_size_/2, base_size_/2);
  prob_maps_.resize(getZSteps());
  for(auto& map : prob_maps_)
    map = cv::Mat_<float>(base_size_, base_size_, initial_value);
  count_maps_.resize(getZSteps());
  for(auto& map : count_maps_)
    map = cv::Mat_<uchar>(base_size_, base_size_, uchar(0));
}


ObjectMap::ObjectMap(float resolution, int base_size, int width, int height, const cv::Point& origin, float max_height, float initial_value)
      : resolution_(resolution), base_size_(base_size), max_height_(max_height), origin_(origin)
{
  prob_maps_.resize(getZSteps());
  for(auto& map : prob_maps_)
    map = cv::Mat_<float>(height, width, initial_value);
  count_maps_.resize(getZSteps());
  for(auto& map : count_maps_)
    map = cv::Mat_<uchar>(height, width, uchar(0));
}


ObjectMap::ObjectMap(const ObjectMap& rhs){
  resolution_ = rhs.resolution_;
  base_size_ = rhs.base_size_;
  max_height_ = rhs.max_height_;
  origin_ = rhs.origin_;
  prob_maps_.resize(rhs.prob_maps_.size());
  for(int i=0; i<prob_maps_.size(); i++)
    rhs.prob_maps_[i].copyTo(prob_maps_[i]);
  count_maps_.resize(rhs.count_maps_.size());
  for(int i=0; i<count_maps_.size(); i++)
    rhs.count_maps_[i].copyTo(count_maps_[i]);
}

ObjectMap& ObjectMap::operator=(const ObjectMap& rhs){
  if(this == &rhs)
    return *this;

  resolution_ = rhs.resolution_;
  base_size_ = rhs.base_size_;
  max_height_ = rhs.max_height_;
  origin_ = rhs.origin_;
  prob_maps_.resize(rhs.prob_maps_.size());
  for(int i=0; i<prob_maps_.size(); i++)
    rhs.prob_maps_[i].copyTo(prob_maps_[i]);
  count_maps_.resize(rhs.count_maps_.size());
  for(int i=0; i<count_maps_.size(); i++)
    rhs.count_maps_[i].copyTo(count_maps_[i]);
  return *this;
}


void ObjectMap::resize(int left, int right, int top, int bottom, float prior){
  origin_ += cv::Point(left, top);
  for(auto& map : prob_maps_)
    cv::copyMakeBorder(map, map, top, bottom, left, right, cv::BORDER_CONSTANT, cv::Scalar(prior));
  for(auto& map : count_maps_)
    cv::copyMakeBorder(map, map, top, bottom, left, right, cv::BORDER_CONSTANT, cv::Scalar(0));
}


void ObjectMap::insertMax(int x, int y, int z, float prob){
  prob_maps_[z](y,x) = std::max(prob, prob_maps_[z](y,x));
  count_maps_[z](y,x) = count_maps_[z](y,x) == uchar(255) ? uchar(255) : count_maps_[z](y,x)+uchar(1);
}


visualization_msgs::MarkerArray ObjectMap::getProbMsg(int id) const{
  static int seq=0;
  visualization_msgs::Marker def;

  def.header.frame_id = "map";
  def.header.stamp = ros::Time::now();
  def.ns = "objects";
  def.id = id;
  def.type = visualization_msgs::Marker::CUBE_LIST;
  def.scale.x = 1/(2*resolution_);
  def.scale.y = def.scale.x;
  def.scale.z = def.scale.x;
  def.action = visualization_msgs::Marker::ADD;

  visualization_msgs::MarkerArray markers;
  markers.markers.resize(1);
  markers.markers[0] = def;
  float min=1.f, max=0.f;
  for(int x=0; x<getWidth(); x++){
    for(int y=0; y<getHeight(); y++){
      for(int z=0; z<getZSteps(); z++){
        if(count_maps_[z](y,x) > uchar(0)){
          min = std::min(min, getProb(x,y,z));
          max = std::max(max, getProb(x,y,z));

          cv::Mat_<cv::Vec3b> color(1,1,cv::Vec3b(std::sqrt(getProb(x,y,z))*150, 255, 255));
          cv::cvtColor(color, color, cv::COLOR_HSV2RGB);
          std_msgs::ColorRGBA c;
          c.r = color(0,0)[0]/255.f;  c.g = color(0,0)[1]/255.f;  c.b = color(0,0)[2]/255.f;  c.a = 1.0;
          markers.markers[0].colors.push_back(c);
          geometry_msgs::Point p;
          p.x = (x-origin_.x+0.5)/resolution_;  p.y = (y-origin_.y+0.5)/resolution_;  p.z = (z+0.5)/resolution_;
          markers.markers[0].points.push_back(p);
        }
      }
    }
  }
  //std::cout << "id: " << id << " MIN: " << min << " MAX: " << max << std::endl;

  return markers;
}


float ObjectMap::getObjectProb(const OctoMapper &octo_mapper) const{
  double prob = 1.0;
  int num = 0;
  float scale_2 = 1.f/(resolution_*2);
  for(int x=0; x<getWidth(); x++){
    for(int y=0; y<getHeight(); y++){
      for(int z=0; z<getZSteps(); z++){
        if(count_maps_[z](y,x) > uchar(0)){
          geometry_msgs::Point p;
          p.x = (x - origin_.x + 0.5) / resolution_;
          p.y = (y - origin_.y + 0.5) / resolution_;
          p.z = (z + 0.5) / resolution_;
          float occ = octo_mapper.getOccupancy(p.x - scale_2, p.y - scale_2, p.z - scale_2, p.x + scale_2, p.y + scale_2, p.z + scale_2);
          prob *= (1.0 - getProb(x, y, z)*occ);
        }
      }
    }
  }

  return (1.0-prob);
}



ObjectMapper::ObjectMapper(){
  ros::NodeHandle private_nh("~");
  private_nh.param("ObjectMapper/OBJ_PRIOR_PROB", OBJ_PRIOR_PROB, OBJ_PRIOR_PROB);
  private_nh.param("ObjectMapper/OBJ_MIN_PROB", OBJ_MIN_PROB, OBJ_MIN_PROB);
  private_nh.param("ObjectMapper/OBJ_MAX_PROB", OBJ_MAX_PROB, OBJ_MAX_PROB);
  private_nh.param("ObjectMapper/OBJ_DEFAULT_RESOLUTION", OBJ_DEFAULT_RESOLUTION, OBJ_DEFAULT_RESOLUTION);
  private_nh.param("ObjectMapper/OBJ_DEFUALT_MAX_HEIGHT", OBJ_DEFUALT_MAX_HEIGHT, OBJ_DEFUALT_MAX_HEIGHT);
  private_nh.param("ObjectMapper/V_H", V_H, V_H);
  private_nh.param("ObjectMapper/V_M", V_M, V_M);

  std::cout << V_H << " " << V_M << " " << OBJ_PRIOR_PROB << " " << std::endl;
}


void ObjectMapper::addCloud(const pcl::PointCloud<pcl::PointXYZ> &cloud, const vision::ObjectDetectionMsg& msg){
  if(cloud.empty())
    return;

  if(maps_.empty()){
    maps_.resize(msg.num_objects, ObjectMap(OBJ_DEFAULT_RESOLUTION, 5.f, OBJ_DEFUALT_MAX_HEIGHT, OBJ_PRIOR_PROB));
  }

  pcl::PointXYZ min, max;
  pcl::getMinMax3D(cloud, min, max);
  expandUntilFitting(min, max);

  std::vector<ObjectMap> tmp(maps_.size(), ObjectMap(maps_[0].getResolution(), maps_[0].getBaseSize(), maps_[0].getWidth(), maps_[0].getHeight(), maps_[0].getOrigin(), max_height_, -1.f));

  for(int i=0; i<cloud.size(); i++){
    int x = maps_[0].getXPixel(cloud[i].x);
    int y = maps_[0].getYPixel(cloud[i].y);
    int z = maps_[0].getZPixel(cloud[i].z);
    for(int m=0; m<tmp.size(); m++){
      tmp[m].insertMax(x,y,z,OBJ_MIN_PROB);
      for(const auto& b : msg.samples[i].boxes){
        tmp[m].insertMax(x,y,z,msg.probs[b*msg.num_objects+m]);
      }
    }
  }

  for(int x=0; x<tmp[0].getWidth(); x++){
    for(int y=0; y<tmp[0].getHeight(); y++){
      for(int z=0; z<tmp[0].getZSteps(); z++){
        if(tmp[0].getProb(x,y,z) >= 0.f){
          for(int m=0; m<tmp.size(); m++){
            maps_[m].insertProb(x,y,z,tmp[m].getProb(x,y,z), OBJ_PRIOR_PROB, V_H, V_M, OBJ_MIN_PROB, OBJ_MAX_PROB);
          }
        }
      }
    }
  }
}


bool ObjectMapper::expandUntilFitting(const pcl::PointXYZ& min, const pcl::PointXYZ& max){
  int x1 = maps_[0].getXPixel(min.x);
  int x2 = maps_[0].getXPixel(max.x);
  int y1 = maps_[0].getYPixel(min.y);
  int y2 = maps_[0].getYPixel(max.y);

  int top=0, bottom=0, left=0, right=0, base_size = maps_[0].getBaseSize(), old_width=maps_[0].getWidth(), old_height=maps_[0].getHeight();
  while(x1+left < 0)
    left += base_size;
  while(x2 >= old_width+right)
    right += base_size;
  while(y1+top < 0)
    top += base_size;
  while(y2 >= old_height+bottom)
    bottom += base_size;

  if(top==0 && bottom==0 && left==0 && right==0)
    return false;

  for(auto& map : maps_)
    map.resize(left, right, top, bottom, OBJ_PRIOR_PROB);
  return true;
}


template <typename T>
std::vector<size_t> ordered(std::vector<T> const& values) {
  std::vector<size_t> indices(values.size());
  std::iota(begin(indices), end(indices), static_cast<size_t>(0));
  std::sort(begin(indices), end(indices),[&](size_t a, size_t b) { return values[a] > values[b]; });
  return indices;
}

std::vector<float> ObjectMapper::getObjectProbs(const OctoMapper& octo_mapper, std::vector<size_t>& order) const{
  ros::Time t = ros::Time::now();
  std::vector<float> probs(maps_.size());
  for(int i=0; i<maps_.size(); i++){
    probs[i] = maps_[i].getObjectProb(octo_mapper);
  }
  order = ordered(probs);
  std::cout << "Object Probs in :" << (ros::Time::now() - t).toSec() << std::endl;
  return probs;
}
