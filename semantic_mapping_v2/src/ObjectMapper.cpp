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
  origin_ = cv::Point(base_size_/2, base_size_/2);
  for(auto& map : prob_maps_)
    map = cv::Mat_<float>(width, height, initial_value);
}


void ObjectMap::resize(float left, float right, float top, float bottom){
  origin_ += cv::Point(left, top);
  for(auto& map : prob_maps_)
    cv::copyMakeBorder(map, map, top, bottom, left, right, cv::BORDER_CONSTANT, cv::Scalar(OBJ_PRIOR_PROB));
}


void ObjectMap::insertMax(int x, int y, int z, float prob){
  if(prob_maps_[z](y,x) < prob)
    prob_maps_[z](y,x) = prob;
}


void ObjectMap::insertProb(int x, int y, int z, float prob){
  if(prob < 0.f)
    return;

  float p = prob_maps_[z](y,x);
  float tmp = (OBJ_CONFIDENCE*prob+(1-OBJ_CONFIDENCE)*p)/OBJ_PRIOR_PROB*p;
  float tmp2 = (OBJ_CONFIDENCE*(1-prob)+(1-OBJ_CONFIDENCE)*(1-p))/OBJ_PRIOR_PROB*(1-p);
  prob_maps_[z](y,x) = std::min(OBJ_MAX_PROB, std::max(OBJ_MIN_PROB, tmp/(tmp+tmp2)));
}





ObjectMapper::ObjectMapper(){
  maps_.resize(NUM_OBJECTS, ObjectMap());
}


void ObjectMapper::addCloud(const pcl::PointCloud<pcl::PointXYZ> &cloud, const std::vector<std::vector<float>> &probs){
  if(cloud.empty())
    return;

  pcl::PointXYZ min, max;
  pcl::getMinMax3D(cloud, min, max);
  expandUntilFitting(min, max);

  std::vector<ObjectMap> tmp(maps_.size(), ObjectMap(maps_[0].getResolution(), maps_[0].getBaseSize(), maps_[0].getWidth(), maps_[0].getHeight(),
                                                     maps_[0].getOrigin(), max_height_, -1.f));
  for(int i=0; i<cloud.size(); i++){
    int x = maps_[0].getXPixel(cloud[i].x);
    int y = maps_[0].getYPixel(cloud[i].y);
    int z = maps_[0].getZPixel(cloud[i].z);
    for(int m=0; m<tmp.size(); m++)
      tmp[m].insertMax(x,y,z,probs[i][m]);
  }

  for(int m=0; m<tmp.size(); m++){
    for(int x=0; x<tmp[m].getWidth(); x++){
      for(int y=0; y<tmp[m].getHeight(); y++){
        for(int z=0; z<tmp[m].getZSteps(); z++){
          maps_[m].insertProb(x,y,z,tmp[m].getProb(x,y,z));
        }
      }
    }
  }
}


bool ObjectMapper::expandUntilFitting(const pcl::PointXYZ& min, const pcl::PointXYZ& max){
  int x1 = maps_[0].getXPixel(min.x);
  int x2 = maps_[0].getXPixel(max.y);
  int y1 = maps_[0].getXPixel(min.x);
  int y2 = maps_[0].getXPixel(max.y);

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
