//
// Created by thomas on 15.11.17.
//

#include "semantic_mapping_v2/ObjectMapper.h"
#include <pcl/common/common.h>
#include <pcl/filters/passthrough.h>
#include <semantic_mapping_v2/OctoMapper.h>
#include <semantic_mapping_v2/RoomMapper.h>

#include <fstream>

float ObjectMapper::OBJ_PRIOR_PROB = 0.01f;
float ObjectMapper::OBJ_MIN_PROB = 0.0001f;
float ObjectMapper::OBJ_MAX_PROB = 0.9f;

float ObjectMapper::OBJ_DEFAULT_RESOLUTION = 4.f;
float ObjectMapper::OBJ_DEFUALT_MAX_HEIGHT = 1.6f;
float ObjectMapper::ROOM_EXPECTED_SIZE = 32.f;

float ObjectMapper::V_H = 0.3;
float ObjectMapper::V_M = 0.0002;

float ObjectMapper::STILL_THERE_PROB = 0.9f;
float ObjectMapper::GOT_THERE_PROB = 0.0005f;
bool ObjectMapper::PARAMS_LOADED = false;

float ObjectMapper::OBJ_FROM_ROOM_CONFIDENCE = 0.7f;

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
      if(OBJ_NAMES.rbegin()->empty())
        OBJ_NAMES.pop_back();
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


ObjectMap::ObjectMap(float resolution, int base_size, int width, int height, const cv::Point& origin, float max_height, float initial_value, uchar initial_count)
      : resolution_(resolution), base_size_(base_size), max_height_(max_height), origin_(origin)
{
  prob_maps_.resize(getZSteps());
  for(auto& map : prob_maps_)
    map = cv::Mat_<float>(height, width, initial_value);
  count_maps_.resize(getZSteps());
  for(auto& map : count_maps_)
    map = cv::Mat_<uchar>(height, width, initial_count);
}

ObjectMap::ObjectMap(float resolution, int base_size, int width, int height, const cv::Point& origin, float max_height, OctoMapper& octomap, const std::vector<Door>& doors)
  : resolution_(resolution), base_size_(base_size), max_height_(max_height), origin_(origin)
{
  prob_maps_.resize(getZSteps());
  for(auto& map : prob_maps_)
    map = cv::Mat_<float>(height, width, 0.f);
  count_maps_.resize(getZSteps());
  for(auto& map : count_maps_)
    map = cv::Mat_<uchar>(height, width, uchar(0));

  cv::Mat_<uchar> behind_door(getHeight(), getWidth(), uchar(0));
  for(int x=0; x<behind_door.cols; x++){
    for(int y=0; y<behind_door.rows; y++){
      for(const auto& door : doors){
        if(door.isBehindDoor(getXWorld(x), getYWorld(y))){
          assert(x>=0 && x<behind_door.cols && y>=0 && y<behind_door.rows);
          behind_door(y,x) = 255;
          break;
        }
      }
    }
  }

  float scale_2 = 1.f/(resolution*2);
  for(int x=0; x<getWidth(); x++){
    for(int y=0; y<getHeight(); y++){
      if(behind_door(y,x) == 0){
        for(int z = 0; z < getZSteps(); z++){
          geometry_msgs::Point p;
          p.x = getXWorld(x);
          p.y = getYWorld(y);
          p.z = getZWorld(z);
          float occ = octomap.getOccupancy(p.x - scale_2, p.y - scale_2, p.z - scale_2, p.x + scale_2, p.y + scale_2, p.z + scale_2);
          if(occ >= 0.f)
            insertMax(x, y, z, occ);
        }
      }
    }
  }
}


ObjectMap::ObjectMap(const ObjectMap& object_map, const ObjectMap& occ_map, cv::Mat_<float> obj_from_room, const std::vector<cv::Point>& only_laser_points,
                     const std::vector<cv::Mat_<float>>& room_type_probs_maps, int obj, const cv::Mat_<uchar>& occ_2d)
{
  *this = object_map;
  //cv::Mat_<float> obj_from_room_new = obj_from_room*ObjectMapper::OBJ_FROM_ROOM_CONFIDENCE + (1.f-obj_from_room)*(1.f-ObjectMapper::OBJ_FROM_ROOM_CONFIDENCE);
  for(int z=0; z<getZSteps(); z++){
    prob_maps_[z] = prob_maps_[z].mul(occ_map.prob_maps_[z]);
    cv::Mat_<float> tmp = 1.f - prob_maps_[z];
    prob_maps_[z] = prob_maps_[z].mul(obj_from_room);
    tmp = tmp.mul(1.f-obj_from_room);
    cv::divide(prob_maps_[z],prob_maps_[z]+tmp,prob_maps_[z]);
    cv::threshold(prob_maps_[z], prob_maps_[z], ObjectMapper::OBJ_MAX_PROB, ObjectMapper::OBJ_MAX_PROB, cv::THRESH_TRUNC);
    cv::threshold(1-prob_maps_[z], prob_maps_[z], 1-ObjectMapper::OBJ_MIN_PROB, 1-ObjectMapper::OBJ_MIN_PROB, cv::THRESH_TRUNC);
    prob_maps_[z] = 1-prob_maps_[z];
  }

  for(const auto& p : only_laser_points){
    float prob = 0.f;
    for(int r = 0; r < room_type_probs_maps.size(); r++){
      prob += RoomMapper::getObjProbGivenRoomPerCell(obj,r)*room_type_probs_maps[r](p);
    }
    for(int z=0; z<getZSteps(); z++){
      prob_maps_[z](p) = prob;
      count_maps_[z](p) = 1;
    }
  }
  for(int z=0; z<getZSteps(); z++){
    cv::bitwise_or(count_maps_[z], occ_2d, count_maps_[z]);
  }
}


ObjectMap ObjectMap::operator*(const ObjectMap &rhs) const{
  ObjectMap res(resolution_, base_size_, getWidth(), getHeight(), origin_, max_height_, 0.f);
  for(int z=0; z<getZSteps(); z++){
    res.prob_maps_[z] = prob_maps_[z].mul(rhs.prob_maps_[z]);
    prob_maps_[z].copyTo(res.count_maps_[z]);
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


void ObjectMap::applyObjAppearVanish(float still_there_prob, float got_there_prob){
  for(int z=0; z<getZSteps(); z++){
    prob_maps_[z] = prob_maps_[z]*still_there_prob + (1.f-prob_maps_[z])*got_there_prob;
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
        if(count_maps_[z](y,x) > uchar(0)){
          min = std::min(min, getProb(x,y,z));
          max = std::max(max, getProb(x,y,z));
          if(getProb(x,y,z) >= thresh){
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
  }
  //std::cout << "id: " << id << " MIN: " << min << " MAX: " << max << std::endl;

  return markers;
}


visualization_msgs::MarkerArray ObjectMap::getProbMsg(OctoMapper& occ, int id, float thresh) const{
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
        if(count_maps_[z](y,x) > uchar(0)){
          min = std::min(min, getProb(x,y,z));
          max = std::max(max, getProb(x,y,z));
          if(getProb(x,y,z) >= thresh){
            cv::Mat_<cv::Vec3b> color(1,1,cv::Vec3b(std::max(std::log10(getProb(x,y,z)*occ.getOccupancy(getXWorld(x), getYWorld(y), getZWorld(z))) + 4.f, 0.f)*150/4.f, 255, 255));
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
  }
  //std::cout << "id: " << id << " MIN: " << min << " MAX: " << max << std::endl;

  return markers;
}


float ObjectMap::getObjectProb(const ObjectMap& occupancy_map, bool multiply_occ) const{
  double prob = 1.0;
  int num = 0;
  for(int x=0; x<getWidth(); x++){
    for(int y=0; y<getHeight(); y++){
      for(int z=0; z<getZSteps(); z++){
        if(count_maps_[z](y,x) > uchar(0))
          prob *= (1.0 - getProb(x, y, z)*(multiply_occ ? occupancy_map.getProb(x,y,z) : 1.f));
        if(occupancy_map.getCount(x,y,z))
          num++;
      }
    }
  }

  return (1.0-prob);
}


float ObjectMap::getObjectProb(float prior, float expected_room_size) const{
  expected_room_size*=resolution_*resolution_*resolution_;
  double prob = 1.0;
  int num = 0;
  for(int x=0; x<getWidth(); x++){
    for(int y=0; y<getHeight(); y++){
      for(int z=0; z<getZSteps(); z++){
        if(count_maps_[z](y,x) > uchar(0)){
          prob *= (1.0 - getProb(x, y, z));
          num++;
        }
      }
    }
  }
  prob *= (std::pow(1.f-prior, std::max(expected_room_size-num, 0.f)));

  return (1.0-prob);
}


std::pair<geometry_msgs::Pose, float> ObjectMap::getObjMax(const ObjectMap& occupancy_map){
  float max_prob = 0.0;
  geometry_msgs::Pose max_pose;
  max_pose.orientation.x = max_pose.orientation.y = max_pose.orientation.z = 0.0;
  max_pose.orientation.w = 1.0;
  for(int x=0; x<getWidth(); x++){
    for(int y=0; y<getHeight(); y++){
      for(int z=0; z<getZSteps(); z++){
        if(count_maps_[z](y,x) > uchar(0)){
          float prob = getProb(x,y,z)*occupancy_map.getProb(x,y,z);
          if(prob > max_prob){
            max_prob = prob;
            max_pose.position.x = getXWorld(x);
            max_pose.position.y = getYWorld(y);
            max_pose.position.z = getZWorld(z);
          }
        }
      }
    }
  }

  return std::pair<geometry_msgs::Pose, float>(max_pose, max_prob);
}


cv::Mat_<float> ObjectMap::get2D(const cv::Mat_<float>& behind_door_mask, const ObjectMap& occ_map) const{
  cv::Mat_<float> map2D(prob_maps_[0].rows, prob_maps_[0].cols, 1.f);
  for(int z=0; z<getZSteps(); z++){
    cv::Mat_<float> tmp(prob_maps_[z].rows, prob_maps_[z].cols, 0.f);
    prob_maps_[z].copyTo(tmp, count_maps_[z]);
    map2D = map2D.mul(1.f-tmp.mul(occ_map.prob_maps_[z]));
  }
  return (1.f - map2D).mul(1.f-behind_door_mask);
}


cv::Mat_<float> ObjectMap::get2D(const cv::Mat_<float> &behind_door_mask) const{
  cv::Mat_<float> map2D(prob_maps_[0].rows, prob_maps_[0].cols, 1.f);
  for(int z=0; z<getZSteps(); z++){
    cv::Mat_<float> tmp(prob_maps_[z].rows, prob_maps_[z].cols, 0.f);
    prob_maps_[z].copyTo(tmp, count_maps_[z]);
    map2D = map2D.mul(1.f-tmp);
  }
  return (1.f - map2D).mul(1.f-behind_door_mask);
}


cv::Mat_<uchar> ObjectMap::getCount2D(const cv::Mat_<float>& behind_door_mask) const{
  cv::Mat_<uchar> map2D(count_maps_[0].rows, count_maps_[0].cols, uchar(0));
  for(int y=0; y<getHeight(); y++){
    for(int x=0; x<getWidth(); x++){
      if(behind_door_mask(y,x) < 0.5f){
        for(int z=0; z<getZSteps(); z++){
          map2D(y, x) = map2D(y, x) | count_maps_[z](y, x);
        }
      }
    }
  }
  return map2D;
}


std::vector<float> ObjectMap::getProbDistribution(const cv::Mat_<float>& behind_door_mask) const{
  cv::Mat_<float> prob_map = get2D(behind_door_mask);
  cv::Mat_<uchar> count_map = getCount2D(behind_door_mask);

  std::vector<float> probs;
  for(int y=0; y<getHeight(); y++){
    for(int x=0; x<getWidth(); x++){
      if(count_map(y,x) > 0){
        probs.push_back(prob_map(y,x));
      }
    }
  }
  std::sort(probs.begin(), probs.end(), std::greater<float>());
  return probs;
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
        msg.data[i] = (getCount(x,y,z) > 0 ? getProb(x,y,z) : 0.f);
        i++;
      }
    }
  }
  return msg;
}



ObjectMapper::ObjectMapper(){
  if(!PARAMS_LOADED){
    ros::NodeHandle private_nh("~");
    private_nh.param("ObjectMapper/OBJ_PRIOR_PROB", OBJ_PRIOR_PROB, OBJ_PRIOR_PROB);
    private_nh.param("ObjectMapper/OBJ_MIN_PROB", OBJ_MIN_PROB, OBJ_MIN_PROB);
    private_nh.param("ObjectMapper/OBJ_MAX_PROB", OBJ_MAX_PROB, OBJ_MAX_PROB);
    private_nh.param("ObjectMapper/OBJ_DEFAULT_RESOLUTION", OBJ_DEFAULT_RESOLUTION, OBJ_DEFAULT_RESOLUTION);
    private_nh.param("ObjectMapper/OBJ_DEFUALT_MAX_HEIGHT", OBJ_DEFUALT_MAX_HEIGHT, OBJ_DEFUALT_MAX_HEIGHT);
    private_nh.param("ObjectMapper/V_H", V_H, V_H);
    private_nh.param("ObjectMapper/V_M", V_M, V_M);
    private_nh.param("ObjectMapper/ROOM_EXPECTED_SIZE", ROOM_EXPECTED_SIZE, ROOM_EXPECTED_SIZE);
    private_nh.param("ObjectMapper/STILL_THERE_PROB", STILL_THERE_PROB, STILL_THERE_PROB);
    private_nh.param("ObjectMapper/GOT_THERE_PROB", GOT_THERE_PROB, GOT_THERE_PROB);
    private_nh.param("ObjectMapper/OBJ_FROM_ROOM_CONFIDENCE", OBJ_FROM_ROOM_CONFIDENCE, OBJ_FROM_ROOM_CONFIDENCE);

    ROS_ERROR("OBJ PARAMETERS LOADED: %f %f %f %f %f %f %f %f %f %f %f", OBJ_PRIOR_PROB, OBJ_MIN_PROB,OBJ_MAX_PROB, OBJ_DEFAULT_RESOLUTION,
              OBJ_DEFUALT_MAX_HEIGHT, V_H, V_M, ROOM_EXPECTED_SIZE, STILL_THERE_PROB, GOT_THERE_PROB, OBJ_FROM_ROOM_CONFIDENCE);

    PARAMS_LOADED = true;
  }
}


ObjectMapper::ObjectMapper(const ObjectMapper& rhs){
  max_height_ = rhs.max_height_;
  maps_ = rhs.maps_;
}


ObjectMapper& ObjectMapper::operator=(const ObjectMapper &rhs){
  if(this == &rhs)
    return *this;

  max_height_ = rhs.max_height_;
  maps_ = rhs.maps_;
  return *this;
}


std::pair<cv::Point,cv::Size> ObjectMapper::addCloud(const pcl::PointCloud<pcl::PointXYZ> &cloud, const vision::ObjectDetectionMsg& msg, float min_z, float max_z){
  if(cloud.empty())
    return std::pair<cv::Point,cv::Size>(cv::Point(-1,-1), cv::Size(-1,-1));

  if(maps_.empty()){
    boost::lock_guard<boost::mutex> lock(maps_mutex_);
    maps_.reserve(msg.num_objects);
    for(int o=0; o<msg.num_objects; o++)
      maps_.push_back(ObjectMap(OBJ_DEFAULT_RESOLUTION, 4.f, OBJ_DEFUALT_MAX_HEIGHT, RoomMapper::getObjProbGivenRoomObjPriorPerCell(o)/6));
  }

  pcl::PointXYZ min, max;
  pcl::getMinMax3D(cloud, min, max);
  expandUntilFitting(min, max);
  min_z = std::max(min_z, 0.f);
  max_z = std::min(max_z, max_height_-0.001f);

  boost::lock_guard<boost::mutex> lock(maps_mutex_);
  std::vector<ObjectMap> tmp(maps_.size(), ObjectMap(maps_[0].getResolution(), maps_[0].getBaseSize(), maps_[0].getWidth(), maps_[0].getHeight(), maps_[0].getOrigin(), max_height_, -1.f));
  ObjectMap count_map(maps_[0].getResolution(), maps_[0].getBaseSize(), maps_[0].getWidth(), maps_[0].getHeight(), maps_[0].getOrigin(), max_height_, 0.f);
  for(int i=0; i<cloud.size(); i++){
    if(cloud[i].z>=min_z && cloud[i].z<=max_z){
      int x = maps_[0].getXPixel(cloud[i].x);
      int y = maps_[0].getYPixel(cloud[i].y);
      int z = maps_[0].getZPixel(cloud[i].z);
      for(int m=0; m<tmp.size(); m++){
        tmp[m].insertMax(x,y,z,OBJ_MIN_PROB);
        for(const auto& b : msg.samples[i].boxes){
          tmp[m].insertMax(x,y,z,msg.probs[b*msg.num_objects+m]);
        }
      }
      count_map.insertMax(x,y,z,0.f);
    }
  }

  for(int x=0; x<tmp[0].getWidth(); x++){
    for(int y=0; y<tmp[0].getHeight(); y++){
      for(int z=0; z<tmp[0].getZSteps(); z++){
        if(tmp[0].getProb(x,y,z) >= 0.f){
          float c = std::min(count_map.getCount(x,y,z)/5.f, 1.f);
          float vh = c*V_H + (1-c)*OBJ_PRIOR_PROB;
          float vm = c*V_M + (1-c)*OBJ_PRIOR_PROB;
          for(int m=0; m<tmp.size(); m++){
            maps_[m].insertProb(x,y,z,tmp[m].getProb(x,y,z), OBJ_PRIOR_PROB, vh, vm, RoomMapper::getObjProbGivenRoomObjPriorPerCell(m)/6/2, OBJ_MAX_PROB);
          }
        }
      }
    }
  }
  return std::pair<cv::Point,cv::Size>(maps_[0].getOrigin(), cv::Size(maps_[0].getWidth(), maps_[0].getHeight()));
}


void ObjectMapper::applyObjAppearVanish(){
  boost::lock_guard<boost::mutex> lock(maps_mutex_);
  for(auto& m : maps_)
    m.applyObjAppearVanish(STILL_THERE_PROB, GOT_THERE_PROB);
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

  boost::lock_guard<boost::mutex> lock(maps_mutex_);
  for(int o=0; o<maps_.size(); o++)
    maps_[o].resize(left, right, top, bottom, RoomMapper::getObjProbGivenRoomObjPriorPerCell(o)/6);
  return true;
}


template <typename T>
std::vector<size_t> ordered(std::vector<T> const& values) {
  std::vector<size_t> indices(values.size());
  std::iota(begin(indices), end(indices), static_cast<size_t>(0));
  std::sort(begin(indices), end(indices),[&](size_t a, size_t b) { return values[a] > values[b]; });
  return indices;
}

std::vector<float> ObjectMapper::getObjectProbs(OctoMapper& octo_mapper, const std::vector<Door>& doors, std::vector<size_t>& order){
  boost::lock_guard<boost::mutex> lock(maps_mutex_);
  if(maps_.empty())
    return std::vector<float>();

  ros::Time t = ros::Time::now();

  cv::Mat_<uchar> behind_door(maps_[0].getHeight(), maps_[0].getWidth(), uchar(0));
  for(int x=0; x<behind_door.cols; x++){
    for(int y=0; y<behind_door.rows; y++){
      for(const auto& door : doors){
        if(door.isBehindDoor(maps_[0].getXWorld(x), maps_[0].getYWorld(y))){
          behind_door(y,x) = 255;
          break;
        }
      }
    }
  }

  ObjectMap occ_map(maps_[0].getResolution(), maps_[0].getBaseSize(), maps_[0].getWidth(), maps_[0].getHeight(), maps_[0].getOrigin(), max_height_, 0.f);
  float scale_2 = 1.f/(maps_[0].getResolution()*2);
  for(int x=0; x<occ_map.getWidth(); x++){
    for(int y=0; y<occ_map.getHeight(); y++){
      if(behind_door(y,x) == 0){
        for(int z=0; z<occ_map.getZSteps(); z++){
          geometry_msgs::Point p;
          p.x = maps_[0].getXWorld(x);
          p.y = maps_[0].getYWorld(y);
          p.z = maps_[0].getZWorld(z);
          float occ = octo_mapper.getOccupancy(p.x - scale_2, p.y - scale_2, p.z - scale_2, p.x + scale_2, p.y + scale_2, p.z + scale_2);
          if(occ >= 0.f)
            occ_map.insertMax(x,y,z,occ);
        }
      }
    }
  }

  std::vector<float> probs(maps_.size());
  for(int i=0; i<maps_.size(); i++){
    probs[i] = maps_[i].getObjectProb(occ_map);
  }
  std::cout << "Object Probs in :" << (ros::Time::now() - t).toSec() << std::endl;

  order = ordered(probs);
  return probs;
}


std::vector<std::pair<geometry_msgs::Pose, float>> ObjectMapper::getObjMax(OctoMapper& octo_mapper, const std::vector<Door>& doors){
  std::vector<std::pair<geometry_msgs::Pose, float>> res;
  boost::lock_guard<boost::mutex> lock(maps_mutex_);
  if(maps_.empty())
    return res;

  ros::Time t = ros::Time::now();

  cv::Mat_<uchar> behind_door(maps_[0].getHeight(), maps_[0].getWidth(), uchar(0));
  for(int x=0; x<behind_door.cols; x++){
    for(int y=0; y<behind_door.rows; y++){
      for(const auto& door : doors){
        if(door.isBehindDoor(maps_[0].getXWorld(x), maps_[0].getYWorld(y))){
          behind_door(y,x) = 255;
          break;
        }
      }
    }
  }

  ObjectMap occ_map(maps_[0].getResolution(), maps_[0].getBaseSize(), maps_[0].getWidth(), maps_[0].getHeight(), maps_[0].getOrigin(), max_height_, 0.f);
  float scale_2 = 1.f/(maps_[0].getResolution()*2);
  for(int x=0; x<occ_map.getWidth(); x++){
    for(int y=0; y<occ_map.getHeight(); y++){
      if(behind_door(y,x) == 0){
        for(int z=0; z<occ_map.getZSteps(); z++){
          geometry_msgs::Point p;
          p.x = maps_[0].getXWorld(x);
          p.y = maps_[0].getYWorld(y);
          p.z = maps_[0].getZWorld(z);
          float occ = octo_mapper.getOccupancy(p.x - scale_2, p.y - scale_2, p.z - scale_2, p.x + scale_2, p.y + scale_2, p.z + scale_2);
          if(occ >= 0.f)
            occ_map.insertMax(x,y,z,occ);
        }
      }
    }
  }

  for(int i=0; i<maps_.size(); i++){
    res.push_back(maps_[i].getObjMax(occ_map));
  }
  std::cout << "Object Max Probs in :" << (ros::Time::now() - t).toSec() << std::endl;

  return res;
};


std::vector<semantic_mapping_v2::ObjectMapMsg> ObjectMapper::getAllObjMapMsgs() {
  std::vector<semantic_mapping_v2::ObjectMapMsg> res;
  boost::lock_guard<boost::mutex> lock(maps_mutex_);
  for(const auto& map : maps_)
    res.push_back(map.getObjMapMsg());
  return res;
}
