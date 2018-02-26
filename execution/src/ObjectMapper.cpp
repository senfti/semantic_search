//
// Created by thomas on 15.11.17.
//

#include "execution/ObjectMapper.h"
#include <pcl/common/common.h>
#include <pcl/filters/passthrough.h>
#include <execution/OctoMapper.h>

#include <fstream>

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

ObjectMap::ObjectMap(float resolution, int base_size, int width, int height, const cv::Point& origin, float max_height, OctoMapper& octomap, ObjectMap* count_map)
  : resolution_(resolution), base_size_(base_size), max_height_(max_height), origin_(origin)
{
  prob_maps_.resize(getZSteps());
  for(auto& map : prob_maps_)
    map = cv::Mat_<float>(height, width, 0.f);
  count_maps_.resize(getZSteps());
  for(auto& map : count_maps_)
    map = cv::Mat_<uchar>(height, width, uchar(0));

  for(int x=0; x<getWidth(); x++){
    for(int y=0; y<getHeight(); y++){
      for(int z=0; z<getZSteps(); z++){
        geometry_msgs::Point p;
        p.x = getXWorld(x);
        p.y = getYWorld(y);
        p.z = getZWorld(z);
        insertMax(x,y,z,octomap.getOccupancy(p.x, p.y, p.z));
      }
    }
  }
  if(count_map != 0){
    *count_map = ObjectMap(resolution, base_size, width, height, origin, max_height, 0.f);
    for(int x=0; x<getWidth(); x++){
      for(int y=0; y<getHeight(); y++){
        for(int z=0; z<getZSteps(); z++){
          geometry_msgs::Point p;
          p.x = getXWorld(x);
          p.y = getYWorld(y);
          p.z = getZWorld(z);
          count_map->insertMax(x,y,z,octomap.getCount(p.x, p.y, p.z));
        }
      }
    }
  }
}


ObjectMap::ObjectMap(const ObjectMap& object_map, const ObjectMap& occ_map, cv::Mat_<float> obj_from_room){
  *this = object_map;
  for(int z=0; z<getZSteps(); z++){
    cv::Mat_<float> tmp = 1.f - prob_maps_[z];
    prob_maps_[z] = object_map.prob_maps_[z].mul(obj_from_room);
    tmp = tmp.mul(1.f-obj_from_room);
    cv::divide(prob_maps_[z],prob_maps_[z]+tmp,prob_maps_[z]);
    prob_maps_[z] = prob_maps_[z].mul(occ_map.prob_maps_[z]);
  }
}


ObjectMap::ObjectMap(const semantic_mapping_v2::ObjectMapMsg &msg){
  resolution_ = msg.resolution;
  origin_ = cv::Point(msg.origin_x, msg.origin_y);
  max_height_ = msg.z_steps/resolution_;
  prob_maps_.resize(msg.z_steps);
  count_maps_.resize(msg.z_steps, cv::Mat_<uchar>::ones(msg.height, msg.width));
  int i=0;
  for(int z=0; z<msg.z_steps; z++){
    prob_maps_[z] = cv::Mat_<float>(msg.height, msg.width, 0.f);
    for(int y=0; y<msg.height; y++){
      for(int x=0; x<msg.width; x++){
        prob_maps_[z](y, x) = msg.data[i];
        i++;
      }
    }
  }
}


ObjectMap ObjectMap::operator*(const ObjectMap &rhs) const{
  ObjectMap res(resolution_, base_size_, getWidth(), getHeight(), origin_, max_height_, 0.f);
  for(int z=0; z<getZSteps(); z++){
    res.prob_maps_[z] = prob_maps_[z].mul(rhs.prob_maps_[z]);
    count_maps_[z].copyTo(res.count_maps_[z]);
  }
  return res;
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


std::vector<int> ObjectMap::expandUntilFitting(int x1, int x2, int y1, int y2, float prior){
  int top=0, bottom=0, left=0, right=0, old_width=getWidth(), old_height=getHeight();
  while(x1+left < 0)
    left += base_size_;
  while(x2 >= old_width+right)
    right += base_size_;
  while(y1+top < 0)
    top += base_size_;
  while(y2 >= old_height+bottom)
    bottom += base_size_;

  if(top!=0 || bottom!=0 || left!=0 || right!=0){
    resize(left, right, top, bottom, prior);
    return std::vector<int>({top, bottom, left, right});
  }

  return std::vector<int>();
}


void ObjectMap::resample(const ObjectMap &target, float prior){
  float exponent = 1.f/((target.getResolution()/getResolution())*(target.getResolution()/getResolution())*(target.getResolution()/getResolution()));
  std::vector<cv::Mat_<float>> prob_tmp(target.getZSteps());
  std::vector<cv::Mat_<uchar>> count_tmp(target.getZSteps());
  for(int z=0; z<target.getZSteps(); z++){
    std::cout << target.getHeight() << " " << target.getWidth() << std::endl;
    prob_tmp[z] = cv::Mat_<float>(target.getHeight(), target.getWidth(), prior);
    count_tmp[z] = cv::Mat_<uchar>(target.getHeight(), target.getWidth(), uchar(0));
    for(int x=0; x<target.getWidth(); x++){
      for(int y=0; y<target.getHeight(); y++){
        if(isWithin(target.getXWorld(x), target.getYWorld(y), target.getZWorld(z))){
          prob_tmp[z](y,x) = getProb(target.getXWorld(x), target.getYWorld(y), target.getZWorld(z)); //1.f-std::pow(1.f-getProb(target.getXWorld(x), target.getYWorld(y), target.getZWorld(z)), exponent);
          count_tmp[z](y,x) = getCount(target.getXWorld(x), target.getYWorld(y), target.getZWorld(z));
        }
      }
    }
  }
  prob_maps_ = prob_tmp;
  count_maps_ = count_tmp;
  resolution_ = target.getResolution();
  base_size_ = target.getBaseSize();
  max_height_ = target.getMaxHeight();
  origin_ = target.getOrigin();
}


void ObjectMap::insertMax(int x, int y, int z, float prob){
  prob_maps_[z](y,x) = std::max(prob, prob_maps_[z](y,x));
  count_maps_[z](y,x) = count_maps_[z](y,x) == uchar(255) ? uchar(255) : count_maps_[z](y,x)+uchar(1);
}


void ObjectMap::applyObjAppearVanish(float still_there_prob, float got_there_prob){
  for(int z=0; z<getZSteps(); z++){
    prob_maps_[z] = prob_maps_[z]*still_there_prob + (1.f-prob_maps_[z])*got_there_prob;
  }
}


void ObjectMap::resetProbs(float prior){
  for(int z=0; z<getZSteps(); z++){
    count_maps_[z] = cv::Mat_<uchar>(count_maps_[z].rows, count_maps_[z].cols, uchar(0));
    prob_maps_[z] = cv::Mat_<float>(prob_maps_[z].rows, prob_maps_[z].cols, prior);
  }
}


visualization_msgs::MarkerArray ObjectMap::getProbMsg(int id, float thresh) const{
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
  def.pose.orientation.x = def.pose.orientation.y = def.pose.orientation.z = 0.0;
  def.pose.orientation.w = 1.0;

  visualization_msgs::MarkerArray markers;
  markers.markers.resize(1);
  markers.markers[0] = def;
  float min=1.f, max=0.f;
  for(int x=0; x<getWidth(); x++){
    for(int y=0; y<getHeight(); y++){
      for(int z=0; z<getZSteps(); z++){
        if(count_maps_[z](y,x) > uchar(0) && getProb(x,y,z) >= thresh){
          min = std::min(min, getProb(x,y,z));
          max = std::max(max, getProb(x,y,z));

          cv::Mat_<cv::Vec3b> color(1,1,cv::Vec3b(std::max(std::log10(getProb(x,y,z)) + 4.f, 0.f)*150/4.f, 255, 255));
          cv::cvtColor(color, color, cv::COLOR_HSV2RGB);
          std_msgs::ColorRGBA c;
          c.r = color(0,0)[0]/255.f;  c.g = color(0,0)[1]/255.f;  c.b = color(0,0)[2]/255.f;  c.a = 1.0;
          markers.markers[0].colors.push_back(c);
          geometry_msgs::Point p;
          p.x = getXWorld(x);  p.y = getYWorld(y);  p.z = getZWorld(z);
          markers.markers[0].points.push_back(p);
        }
      }
    }
  }
  //std::cout << "id: " << id << " MIN: " << min << " MAX: " << max << std::endl;

  return markers;
}


visualization_msgs::MarkerArray ObjectMap::getProbMsg(float scale) const{
  static int seq=0;
  visualization_msgs::Marker def;

  def.header.frame_id = "map";
  def.header.stamp = ros::Time::now();
  def.ns = "objects";
  def.id = 0;
  def.type = visualization_msgs::Marker::CUBE_LIST;
  def.scale.x = 1/(2*resolution_);
  def.scale.y = def.scale.x;
  def.scale.z = def.scale.x;
  def.action = visualization_msgs::Marker::ADD;
  def.pose.orientation.x = def.pose.orientation.y = def.pose.orientation.z = 0.0;
  def.pose.orientation.w = 1.0;

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

          cv::Mat_<cv::Vec3b> color(1,1,(cv::Vec3b(std::min(getProb(x,y,z)*scale,1.f)*150, 255, 255)));
          cv::cvtColor(color, color, cv::COLOR_HSV2RGB);
          std_msgs::ColorRGBA c;
          c.r = color(0,0)[0]/255.f;  c.g = color(0,0)[1]/255.f;  c.b = color(0,0)[2]/255.f;  c.a = 1.0;
          markers.markers[0].colors.push_back(c);
          geometry_msgs::Point p;
          p.x = getXWorld(x);  p.y = getYWorld(y);  p.z = getZWorld(z);
          markers.markers[0].points.push_back(p);
        }
      }
    }
  }
  //std::cout << "id: " << id << " MIN: " << min << " MAX: " << max << std::endl;

  return markers;
}


float ObjectMap::getObjectProb(const ObjectMap& occupancy_map, float prior, float expected_room_size) const{
  double prob = 1.0;
  int num = 0;
  float scale_2 = 1.f/(resolution_*2);
  for(int x=0; x<getWidth(); x++){
    for(int y=0; y<getHeight(); y++){
      for(int z=0; z<getZSteps(); z++){
        if(count_maps_[z](y,x) > uchar(0)){
          prob *= (1.0 - getProb(x, y, z)*occupancy_map.getProb(x,y,z));
          num++;
        }
      }
    }
  }
  prob *= (std::pow(1.f-prior, expected_room_size));

  return (1.0-prob);
}


float ObjectMap::getObjectProb(const cv::Mat_<float>& behind_door_mask, float prior, float expected_room_size) const{
  expected_room_size*=resolution_*resolution_*resolution_;
  double prob = 1.0;
  int num = 0;
  for(int x=0; x<getWidth(); x++){
    for(int y=0; y<getHeight(); y++){
      for(int z=0; z<getZSteps(); z++){
        if(count_maps_[z](y,x) > uchar(0) && behind_door_mask(y,x) < 0.5f){
          prob *= (1.0 - getProb(x, y, z));
          num++;
        }
      }
    }
  }
  prob *= (std::pow(1.f-prior, std::max(expected_room_size-num, 0.f)));

  return (1.0-prob);
}


float ObjectMap::getMaxObjectProb(const ObjectMap &occupancy_map) const{
  float max_prob = 0.0;
  for(int x=0; x<getWidth(); x++){
    for(int y=0; y<getHeight(); y++){
      for(int z=0; z<getZSteps(); z++){
        if(count_maps_[z](y,x) > uchar(0)){
          max_prob = std::max(max_prob, getProb(x, y, z)*occupancy_map.getProb(x,y,z));
        }
      }
    }
  }
  return max_prob;
}


cv::Mat_<float> ObjectMap::get2D(const cv::Mat_<float>& behind_door_mask, const ObjectMap& occ_map) const{
  cv::Mat_<float> map2D(prob_maps_[0].rows, prob_maps_[0].cols, 1.f);
  for(int z=0; z<getZSteps()/3; z++){
    cv::Mat_<float> tmp(prob_maps_[0].rows, prob_maps_[0].cols, 0.f);
    prob_maps_[3*z].copyTo(tmp, count_maps_[3*z]);
    cv::Mat_<float> tmp_max = tmp.mul(occ_map.prob_maps_[3*z]);
    prob_maps_[3*z+1].copyTo(tmp, count_maps_[3*z+1]);
    tmp_max = cv::max(tmp_max, tmp.mul(occ_map.prob_maps_[3*z+1]));
    prob_maps_[3*z+2].copyTo(tmp, count_maps_[3*z+2]);
    tmp_max = cv::max(tmp_max, tmp.mul(occ_map.prob_maps_[3*z+2]));
    map2D = map2D.mul(1.f-tmp_max);
  }
  if(getZSteps()%3 == 1){
    cv::Mat_<float> tmp(prob_maps_[getZSteps() - 1].rows, prob_maps_[getZSteps() - 1].cols, 0.f);
    prob_maps_[getZSteps() - 1].copyTo(tmp, count_maps_[getZSteps() - 1]);
    map2D = map2D.mul(1.f - tmp.mul(occ_map.prob_maps_[getZSteps() - 1]));
  }
  else if(getZSteps()%3 == 2){
    cv::Mat_<float> tmp(prob_maps_[getZSteps() - 1].rows, prob_maps_[getZSteps() - 1].cols, 0.f);
    prob_maps_[getZSteps() - 1].copyTo(tmp, count_maps_[getZSteps() - 1]);
    cv::Mat_<float> tmp_max = tmp.mul(occ_map.prob_maps_[getZSteps()-1]);
    prob_maps_[getZSteps() - 2].copyTo(tmp, count_maps_[getZSteps() - 2]);
    tmp_max = cv::max(tmp_max, tmp.mul(occ_map.prob_maps_[getZSteps()-2]));
    map2D = map2D.mul(1.f-tmp_max);
  }

  return (1.f - map2D).mul(1.f-behind_door_mask);
}


semantic_mapping_v2::ObjectMapMsg ObjectMap::getObjMapMsg() const{
  semantic_mapping_v2::ObjectMapMsg msg;
  msg.header.stamp = ros::Time::now();
  msg.header.frame_id = "/map";
  msg.height = getHeight();
  msg.width = getWidth();
  msg.z_steps = getZSteps();
  msg.resolution = getResolution();
  msg.origin_x = getOrigin().x;
  msg.origin_y = getOrigin().y;
  msg.data.resize(msg.height*msg.width*msg.z_steps);
  int i=0;
  for(int z=0; z<msg.z_steps; z++){
    for(int y=0; y<msg.height; y++){
      for(int x=0; x<msg.width; x++){
        msg.data[i] = getProb(x,y,z);
        i++;
      }
    }
  }
  return msg;
}


cv::Mat_<float> ObjectMap::get2D(ObjectMap occ_map, ObjectMap prior_map, ObjectMap count_map, int count_thresh, cv::Point& origin) const{
  cv::Mat_<float> map2D(prob_maps_[0].rows, prob_maps_[0].cols, 1.f);
  for(int z=0; z<getZSteps(); z++){
    for(int x=0; x<getWidth(); x++){
      for(int y=0; y<getHeight(); y++){
        if(count_map.getProb(x,y,z) < count_thresh){
          float s = float(count_map.getProb(x,y,z)/count_thresh);
          float val = prior_map.getProb(x,y,z)*(1.f-s) + getProb(x,y,z)*occ_map.getProb(x,y,z)*s;
          map2D(y,x) = map2D(y,x) * (1.f-val);
        }
      }
    }
  }
  return (1.f - map2D);
}

