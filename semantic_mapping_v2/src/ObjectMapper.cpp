//
// Created by thomas on 15.11.17.
//

#include "semantic_mapping_v2/ObjectMapper.h"
#include <pcl/common/common.h>
#include <pcl/filters/passthrough.h>

ObjectMap::ObjectMap(float resolution, float start_size, float max_height, float initial_value)
    : resolution_(resolution), base_size_(start_size*resolution), max_height_(max_height)
{
  prob_maps_.resize(getZSteps());
  origin_ = cv::Point(base_size_/2, base_size_/2);
  for(auto& map : prob_maps_)
    map = cv::Mat_<float>(base_size_, base_size_, initial_value);
}


ObjectMap::ObjectMap(float resolution, int base_size, int width, int height, const cv::Point& origin, float max_height, float initial_value)
      : resolution_(resolution), base_size_(base_size), max_height_(max_height), origin_(origin)
{
  prob_maps_.resize(getZSteps());
  for(auto& map : prob_maps_)
    map = cv::Mat_<float>(height, width, initial_value);
}


ObjectMap::ObjectMap(const ObjectMap& rhs){
  resolution_ = rhs.resolution_;
  base_size_ = rhs.base_size_;
  max_height_ = rhs.max_height_;
  origin_ = rhs.origin_;
  prob_maps_.resize(rhs.prob_maps_.size());
  for(int i=0; i<prob_maps_.size(); i++){
    rhs.prob_maps_[i].copyTo(prob_maps_[i]);
  }
}

ObjectMap& ObjectMap::operator=(const ObjectMap& rhs){
  if(this == &rhs)
    return *this;

  resolution_ = rhs.resolution_;
  base_size_ = rhs.base_size_;
  max_height_ = rhs.max_height_;
  origin_ = rhs.origin_;
  prob_maps_.resize(rhs.prob_maps_.size());
  for(int i=0; i<prob_maps_.size(); i++){
    rhs.prob_maps_[i].copyTo(prob_maps_[i]);
  }
  return *this;
}


void ObjectMap::resize(float left, float right, float top, float bottom){
  origin_ += cv::Point(left, top);
  for(auto& map : prob_maps_)
    cv::copyMakeBorder(map, map, top, bottom, left, right, cv::BORDER_CONSTANT, cv::Scalar(OBJ_PRIOR_PROB));
}


void ObjectMap::insertMax(int x, int y, int z, float prob){
  prob_maps_[z](y,x) = std::max(prob, prob_maps_[z](y,x));
}


void ObjectMap::insertProb(int x, int y, int z, float prob){
  if(prob < 0.f)
    return;
  
  float p = prob_maps_[z](y,x);
  float tmp = (OBJ_CONFIDENCE*prob+(1-OBJ_CONFIDENCE)*OBJ_PRIOR_PROB)/OBJ_PRIOR_PROB*p;
  float tmp2 = (OBJ_CONFIDENCE*(1-prob)+(1-OBJ_CONFIDENCE)*(1-OBJ_PRIOR_PROB))/OBJ_PRIOR_PROB*(1-p);
  prob_maps_[z](y,x) = std::min(OBJ_MAX_PROB, std::max(OBJ_MIN_PROB, tmp/(tmp+tmp2)));
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
        min = std::min(min, getProb(x,y,z));
        max = std::max(max, getProb(x,y,z));

        cv::Mat_<cv::Vec3b> color(1,1,cv::Vec3b(getProb(x,y,z)*150, 255, 255));
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
  std::cout << "MIN: " << min << "MAX: " << max << std::endl;

  return markers;
}




ObjectMapper::ObjectMapper(){
  maps_.resize(NUM_OBJECTS, ObjectMap());
}


void ObjectMapper::addCloud(const pcl::PointCloud<pcl::PointXYZ> &cloud, const std::vector<vision::ObjectDetectionSample>& samples){
  if(cloud.empty())
    return;

  pcl::PointXYZ min, max;
  pcl::getMinMax3D(cloud, min, max);
  expandUntilFitting(min, max);

  std::vector<ObjectMap> tmp(maps_.size(), ObjectMap(maps_[0].getResolution(), maps_[0].getBaseSize(), maps_[0].getWidth(), maps_[0].getHeight(), maps_[0].getOrigin(), max_height_, -1.f));

  for(int i=0; i<cloud.size(); i++){
    int x = maps_[0].getXPixel(cloud[i].x);
    int y = maps_[0].getYPixel(cloud[i].y);
    int z = maps_[0].getZPixel(cloud[i].z);
    for(int m=0; m<tmp.size(); m++)
      tmp[m].insertMax(x,y,z,samples[i].probs[m]);
  }

  for(int x=0; x<tmp[0].getWidth(); x++){
    for(int y=0; y<tmp[0].getHeight(); y++){
      for(int z=0; z<tmp[0].getZSteps(); z++){
        if(tmp[0].getProb(x,y,z) >= 0.f){
          for(int m=0; m<tmp.size(); m++){
            maps_[m].insertProb(x,y,z,tmp[m].getProb(x,y,z));
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
    map.resize(left, right, top, bottom);
  return true;
}


