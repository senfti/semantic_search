//
// Created by thomas on 23.11.17.
//

#include "semantic_mapping_v2/RoomTypeMapper.h"
#include <ros/ros.h>
#include <fstream>


RoomTypeMap::RoomTypeMap(float resolution, float start_size, float initial_value, const std::string& name)
  : resolution_(resolution), base_size_(start_size*resolution), name_(name)
{
  origin_ = cv::Point(base_size_/2, base_size_/2);
  prob_map_ = cv::Mat_<float>(base_size_, base_size_, initial_value);
  seen_map_ = cv::Mat_<uchar>(base_size_, base_size_, uchar(0));
}

RoomTypeMap::RoomTypeMap(float resolution, int base_size, int width, int height, const cv::Point& origin, float initial_value, const std::string& name)
  : resolution_(resolution), base_size_(base_size), origin_(origin), name_(name)
{
  prob_map_ = cv::Mat_<float>(base_size_, base_size_, initial_value);
  seen_map_ = cv::Mat_<uchar>(base_size_, base_size_, uchar(0));
}

RoomTypeMap::RoomTypeMap(const cv::Mat_<float>& prob_map, const cv::Mat_<uchar>& seen_map, const cv::Point& origin, float resolution, int base_size)
  : resolution_(resolution), base_size_(base_size)
{
  prob_map.copyTo(prob_map_);
  seen_map.copyTo(seen_map_);
  origin_ = origin;
}

RoomTypeMap::RoomTypeMap(const RoomTypeMap& rhs){
  resolution_ = rhs.resolution_;
  base_size_ = rhs.base_size_;
  origin_ = rhs.origin_;
  rhs.prob_map_.copyTo(prob_map_);
  rhs.seen_map_.copyTo(seen_map_);
  name_ = rhs.name_;
}

RoomTypeMap& RoomTypeMap::operator=(const RoomTypeMap& rhs){
  if(this == &rhs)
    return *this;

  resolution_ = rhs.resolution_;
  base_size_ = rhs.base_size_;
  origin_ = rhs.origin_;
  rhs.prob_map_.copyTo(prob_map_);
  rhs.seen_map_.copyTo(seen_map_);
  name_ = rhs.name_;
  return *this;
}

void RoomTypeMap::resize(int left, int right, int top, int bottom, float prior){
  origin_ += cv::Point(left, top);
  cv::copyMakeBorder(prob_map_, prob_map_, top, bottom, left, right, cv::BORDER_CONSTANT, cv::Scalar(prior));
  cv::copyMakeBorder(seen_map_, seen_map_, top, bottom, left, right, cv::BORDER_CONSTANT, cv::Scalar(0));
}

visualization_msgs::MarkerArray RoomTypeMap::getProbMsg(int id) const{
  static int seq=0;
  visualization_msgs::Marker def;

  def.header.frame_id = "map";
  def.header.stamp = ros::Time::now();
  def.ns = "rooms";
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
      min = std::min(min, getProb(x,y));
      max = std::max(max, getProb(x,y));

      cv::Mat_<cv::Vec3b> color(1,1,cv::Vec3b(std::sqrt(getProb(x,y))*150, 255, 255));
      cv::cvtColor(color, color, cv::COLOR_HSV2RGB);
      std_msgs::ColorRGBA c;
      c.r = color(0,0)[0]/255.f;  c.g = color(0,0)[1]/255.f;  c.b = color(0,0)[2]/255.f;  c.a = 1.0;
      markers.markers[0].colors.push_back(c);
      geometry_msgs::Point p;
      p.x = (x-origin_.x+0.5)/resolution_;  p.y = (y-origin_.y+0.5)/resolution_;  p.z = 0.0;
      markers.markers[0].points.push_back(p);
    }
  }
  //std::cout << "id: " << id << " name: " << name_ << " MIN: " << min << " MAX: " << max << std::endl;

  return markers;
}


semantic_mapping_v2::RoomTypeMapMsg RoomTypeMap::getRoomTypeMapMsg() const{
  semantic_mapping_v2::RoomTypeMapMsg msg;
  msg.header.stamp = ros::Time::now();
  msg.header.frame_id = "/map";
  msg.height = getHeight();
  msg.width = getWidth();
  msg.resolution = getResolution();
  msg.origin_x = getOrigin().x;
  msg.origin_y = getOrigin().y;
  msg.data.resize(msg.height*msg.width);
  int i=0;
  for(int y=0; y<msg.height; y++){
    for(int x=0; x<msg.width; x++){
      msg.data[i] = getProb(x,y);
      i++;
    }
  }
  return msg;
}



float RoomTypeMapper::getRoomSimilarity(int i, int j){
  static std::vector<std::vector<float>> similarity;
  if(similarity.empty()){
    ros::NodeHandle private_nh("~");
    std::string sim_file = "/home/thomas/semantic_search/src/semantic_mapping_v2/data/room_spread.dat";
    private_nh.param("RoomTypeMapper/SIMILARITY_FILE", sim_file, sim_file);
    std::ifstream f(sim_file);
    if(!f.good()){
      ROS_WARN("CANNOT OPEN SIMILARITY FILE %s", sim_file.c_str());
      return 1.f;
    }
    std::vector<float> tmp;
    while(!f.eof()){
      float v;
      f >> v;
      tmp.push_back(v);
    }
    int n = std::sqrt(tmp.size());
    similarity.resize(n);
    for(int r=0; r<n; r++){
      similarity[r].resize(n);
      for(int c=0; c<n; c++){
        similarity[r][c] = tmp[r*n+c];
      }
    }
//    std::ofstream o("/home/thomas/sdfsdfsdfsdf.csv");
//    for(int c=0; c<n; c++){
//      for(int r=0; r<n; r++){
//        if(similarity[c][r] > 1.0){
//          std::cout << r << " " << c << " " << similarity[c][r] << std::endl;
//        }
//        o << similarity[c][r] << ",";
//      }
//      o << std::endl;
//    }
  }
  return similarity[i][j];
}

float RoomTypeMapper::CELL_MIN_PROB = 0.0005;
float RoomTypeMapper::CELL_MAX_PROB = 0.75;
float RoomTypeMapper::ROOM_MAX_PROB = 0.9;
float RoomTypeMapper::ROOM_DEFAULT_RESOLUTION = 4.f;
float RoomTypeMapper::ASUS_FOV = 29.f;
float RoomTypeMapper::MIN_DIST = 1.f;
float RoomTypeMapper::MAX_DIST = 4.0f;
float RoomTypeMapper::CELL_HIT_MISS_RATIO = 5.0;
bool RoomTypeMapper::PARAMS_LOADED = false;

RoomTypeMapper::RoomTypeMapper(){
  if(!PARAMS_LOADED){
    PARAMS_LOADED = true;
    ros::NodeHandle private_nh("~");
    private_nh.param("RoomTypeMapper/CELL_HIT_MISS_RATIO", CELL_HIT_MISS_RATIO, CELL_HIT_MISS_RATIO);
    private_nh.param("RoomTypeMapper/CELL_MAX_PROB", CELL_MAX_PROB, CELL_MAX_PROB);
    private_nh.param("RoomTypeMapper/CELL_MIN_PROB", CELL_MIN_PROB, CELL_MIN_PROB);
    private_nh.param("RoomTypeMapper/ROOM_DEFAULT_RESOLUTION", ROOM_DEFAULT_RESOLUTION, ROOM_DEFAULT_RESOLUTION);
    private_nh.param("RoomTypeMapper/ASUS_FOV", ASUS_FOV, ASUS_FOV);
    private_nh.param("RoomTypeMapper/MIN_DIST", MIN_DIST, MIN_DIST);
    private_nh.param("RoomTypeMapper/MAX_DIST", MAX_DIST, MAX_DIST);
    private_nh.param("ROOM_MAX_PROB", ROOM_MAX_PROB, ROOM_MAX_PROB);
    ASUS_FOV *= M_PI / 180.f;

    ROS_ERROR("ROOM TYPE PARAMETERS LOADED: %f %f %f %f %f %f %f %f", CELL_HIT_MISS_RATIO,CELL_MAX_PROB, CELL_MIN_PROB,
              ROOM_DEFAULT_RESOLUTION, ASUS_FOV, MIN_DIST, MAX_DIST, ROOM_MAX_PROB);
  }
}


RoomTypeMapper::RoomTypeMapper(const RoomTypeMapper& rhs){
  prob_maps_ = rhs.prob_maps_;
  names_ = rhs.names_;
}


RoomTypeMapper::RoomTypeMapper(const std::vector<cv::Mat_<float>> &prob_maps, const cv::Mat_<uchar> &seen_map, const cv::Point &origin, float resolution, int base_size){
  if(!PARAMS_LOADED){
    PARAMS_LOADED = true;
    ros::NodeHandle private_nh("~");
    private_nh.param("RoomTypeMapper/CELL_HIT_MISS_RATIO", CELL_HIT_MISS_RATIO, CELL_HIT_MISS_RATIO);
    private_nh.param("RoomTypeMapper/CELL_MAX_PROB", CELL_MAX_PROB, CELL_MAX_PROB);
    private_nh.param("RoomTypeMapper/CELL_MIN_PROB", CELL_MIN_PROB, CELL_MIN_PROB);
    private_nh.param("RoomTypeMapper/ROOM_DEFAULT_RESOLUTION", ROOM_DEFAULT_RESOLUTION, ROOM_DEFAULT_RESOLUTION);
    private_nh.param("RoomTypeMapper/ASUS_FOV", ASUS_FOV, ASUS_FOV);
    private_nh.param("RoomTypeMapper/MIN_DIST", MIN_DIST, MIN_DIST);
    private_nh.param("RoomTypeMapper/MAX_DIST", MAX_DIST, MAX_DIST);
    private_nh.param("ROOM_MAX_PROB", ROOM_MAX_PROB, ROOM_MAX_PROB);
    ASUS_FOV *= M_PI / 180.f;

    ROS_ERROR("ROOM TYPE PARAMETERS LOADED: %f %f %f %f %f %f %f %f", CELL_HIT_MISS_RATIO,CELL_MAX_PROB, CELL_MIN_PROB,
              ROOM_DEFAULT_RESOLUTION, ASUS_FOV, MIN_DIST, MAX_DIST, ROOM_MAX_PROB);
  }
  for(const auto& m : prob_maps){
    prob_maps_.push_back(RoomTypeMap(m,seen_map,origin,resolution, base_size));
  }
}


bool RoomTypeMapper::resizeUntilFitting(std::vector<cv::Point>& points){
  int x1 = points[0].x;
  int x2 = points[0].x;
  int y1 = points[0].y;
  int y2 = points[0].y;
  for(int i=1; i<points.size(); i++){
    x1 = std::min(points[i].x, x1);
    x2 = std::max(points[i].x, x2);
    y1 = std::min(points[i].y, y1);
    y2 = std::max(points[i].y, y2);
  }

  int top=0, bottom=0, left=0, right=0, base_size = prob_maps_[0].getBaseSize(), old_width=prob_maps_[0].getWidth(), old_height=prob_maps_[0].getHeight();
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

  boost::unique_lock<boost::mutex> lock(maps_mutex_);
  for(auto& map : prob_maps_)
    map.resize(left, right, top, bottom, ROOM_PRIOR_PROB);
  lock.unlock();

  for(int i=0; i<points.size(); i++){
    points[i].x += left;
    points[i].y += top;
  }
  return true;
}


void RoomTypeMapper::resizeToObjMap(const cv::Point &origin, const cv::Size &size){
  int top = std::max(origin.y-prob_maps_[0].getOrigin().y, 0);
  int left = std::max(origin.x-prob_maps_[0].getOrigin().x, 0);
  int bottom = std::max((size.height-origin.y)-(prob_maps_[0].getHeight()-prob_maps_[0].getOrigin().y), 0);
  int right = std::max((size.width-origin.x)-(prob_maps_[0].getWidth()-prob_maps_[0].getOrigin().x), 0);
  for(auto& map : prob_maps_)
    map.resize(left, right, top, bottom, ROOM_PRIOR_PROB);
}


void applyMinAndMax(std::vector<double>& probs, double min, double max){
  double sum = 0.0;
  int clamp_idx = -1;
  for(int i=0; i<probs.size(); i++){
    if(probs[i] > max){
      clamp_idx = i;
      probs[i] = max;
    }
    else
      sum += probs[i];
  }
  if(clamp_idx > 0){
    for(int i=0; i<probs.size(); i++){
      if(i!=clamp_idx)
        probs[i] = probs[i]/sum*(1.0-max);
    }
  }
  sum = 0.0;
  for(int i=0; i<probs.size(); i++){
    sum += probs[i];
  }
  if(sum < 0.99 || sum > 1.01)
    std::cout << "1 _______________________________ sum bad " << sum << std::endl;

  std::vector<int> not_low_idx(probs.size());
  for(int i=0; i<probs.size(); i++){
    not_low_idx[i] = i;
  }
  while(true){
    std::vector<int> old_not_low_idx = not_low_idx;
    not_low_idx.clear();
    bool did_thresh = false;
    for(int i=0; i<old_not_low_idx.size(); i++){
      if(probs[old_not_low_idx[i]] < min){
        probs[old_not_low_idx[i]] = min;
        did_thresh = true;
      }
      else
        not_low_idx.push_back(old_not_low_idx[i]);
    }
    if(!did_thresh)
      break;

    double rest_sum = 1.0-(probs.size()-not_low_idx.size())*min;
    sum = 0.0;
    for(int i=0; i<not_low_idx.size(); i++){
      sum += probs[not_low_idx[i]];
    }
    for(int i=0; i<not_low_idx.size(); i++){
      probs[not_low_idx[i]] *= rest_sum/sum;
    }
  }
  sum = 0.0;
  for(int i=0; i<probs.size(); i++){
    sum += probs[i];
  }
  if(sum < 0.99 || sum > 1.01)
    std::cout << "2 _______________________________ sum bad " << sum << std::endl;
}


void RoomTypeMapper::updateProbs(const vision::VisionMsgConstPtr &msg, int x, int y){
  std::vector<double> probs(prob_maps_.size());
  for(int i=0; i<prob_maps_.size(); i++){
    probs[i] = prob_maps_[i].getProb(x,y);
  }

  double sum = 0.0;
  for(int i=0; i<NUM_CLASSES; i++){
    double prob = msg->place_guesses[i].prob;
    probs[i] = (prob*CELL_HIT_MISS_RATIO + (1-prob)) * probs[i];      // division by ROOM_PRIOR_PROB cancels out with normalization
    sum += probs[i];
  }
  for(int i=0; i<NUM_CLASSES; i++){
    probs[i] /= sum;
  }

  applyMinAndMax(probs, CELL_MIN_PROB, CELL_MAX_PROB);

  sum = 0.0;
  for(int i=0; i<prob_maps_.size(); i++){
    prob_maps_[i].setProb(x,y,probs[i]);
    sum += probs[i];
  }
  if(sum < 0.99 || sum > 1.01)
    std::cout << "_______________________________ sum bad " << sum << std::endl;
}


void RoomTypeMapper::processMsg(const vision::VisionMsgConstPtr& msg, const GMapping::OrientedPoint& pose){
  if(NUM_CLASSES == 0){
    NUM_CLASSES = msg->place_guesses.size();
    ROOM_PRIOR_PROB = 1.f/NUM_CLASSES;

    boost::lock_guard<boost::mutex> lock(maps_mutex_);
    prob_maps_.resize(NUM_CLASSES, RoomTypeMap(ROOM_DEFAULT_RESOLUTION, 4.f, ROOM_PRIOR_PROB));
    names_.resize(NUM_CLASSES);
    for(int i=0; i<NUM_CLASSES; i++){
      names_[i] = msg->place_guesses[i].name;
      prob_maps_[i].setName(names_[i]);
    }
  }

  std::vector<cv::Point> points(4);
  cv::Point2d orig = prob_maps_[0].getOrigin();
  points[0] = cv::Point2d(MIN_DIST*std::cos(pose.theta+ASUS_FOV)+pose.x, MIN_DIST*std::sin(pose.theta+ASUS_FOV)+pose.y) * prob_maps_[0].getResolution() + orig;
  points[1] = cv::Point2d(MAX_DIST*std::cos(pose.theta+ASUS_FOV)+pose.x, MAX_DIST*std::sin(pose.theta+ASUS_FOV)+pose.y) * prob_maps_[0].getResolution() + orig;
  points[2] = cv::Point2d(MAX_DIST*std::cos(pose.theta-ASUS_FOV)+pose.x, MAX_DIST*std::sin(pose.theta-ASUS_FOV)+pose.y) * prob_maps_[0].getResolution() + orig;
  points[3] = cv::Point2d(MIN_DIST*std::cos(pose.theta-ASUS_FOV)+pose.x, MIN_DIST*std::sin(pose.theta-ASUS_FOV)+pose.y) * prob_maps_[0].getResolution() + orig;

  resizeUntilFitting(points);
  boost::lock_guard<boost::mutex> lock(maps_mutex_);
  cv::Mat_<uchar> mask(prob_maps_[0].getHeight(), prob_maps_[0].getWidth(), uchar(0));
  cv::fillConvexPoly(mask, points, cv::Scalar(255));
  for(int x=0; x<mask.cols; x++){
    for(int y=0; y<mask.rows; y++){
      if(mask(y,x))
        updateProbs(msg, x, y);
    }
  }
}


template <typename T>
std::vector<size_t> ordered(std::vector<T> const& values) {
  std::vector<size_t> indices(values.size());
  std::iota(begin(indices), end(indices), static_cast<size_t>(0));
  std::sort(begin(indices), end(indices),[&](size_t a, size_t b) { return values[a] > values[b]; });
  return indices;
}

std::vector<float> RoomTypeMapper::getRoomProb(const nav_msgs::OccupancyGrid& map, const std::vector<Door>& doors, std::vector<size_t>& order){
  if(prob_maps_.empty() || map.data.size() == 0)
    return std::vector<float>();

  ros::Time t = ros::Time::now();
  cv::Mat_<uchar> mask(map.info.height, map.info.width, uchar(0));
  for(int x=0; x<mask.cols; x++){
    for(int y=0; y<mask.rows; y++){
      if(map.data[y * map.info.width + x] >= 0)
        mask(y,x) = 255;
    }
  }
  cv::dilate(mask, mask, cv::Mat_<uchar>::ones(3,3), cv::Point(-1,-1), 5);
  cv::erode(mask, mask, cv::Mat_<uchar>::ones(3,3), cv::Point(-1,-1), 5);

  boost::lock_guard<boost::mutex> lock(maps_mutex_);
  std::vector<double> probs(prob_maps_.size(), 0.0);
  double res = 1.00 / map.info.resolution;
  double v = std::log(ROOM_PRIOR_PROB);
  for(int x=0; x<prob_maps_[0].getWidth(); x++){
    for(int y=0; y<prob_maps_[0].getHeight(); y++){
      if(prob_maps_[0].wasSeen(x,y)){
        float x_world = prob_maps_[0].getXWorld(x);
        float y_world = prob_maps_[0].getYWorld(y);
        int x_map = ((x_world - map.info.origin.position.x)) * res;
        int y_map = ((y_world - map.info.origin.position.y)) * res;
        if(x_map >= 0 && x_map < mask.cols && y_map >= 0 && y_map < mask.rows && mask(y_map, x_map) > 0){
          bool behind = false;
          for(const auto &door : doors){
            if(door.isBehindDoor(x_world, y_world)){
              behind = true;
              break;
            }
          }
          if(!behind){
            for(int i = 0; i < probs.size(); i++){
              double prob = 0.0;
              for(int j = 0; j < probs.size(); j++){
                prob += getRoomSimilarity(i, j) * prob_maps_[j].getProb(x, y);
              }
              probs[i] += std::log(prob) - v;
            }
          }
        }
      }
    }
  }

  double sum = 0.0;
  for(int i=0; i<probs.size(); i++){
    probs[i] = std::exp(probs[i]);
    sum += probs[i];
  }
  for(int i=0; i<probs.size(); i++){
    probs[i] = probs[i]/sum;
  }
  applyMinAndMax(probs, CELL_MIN_PROB, ROOM_MAX_PROB);

  std::vector<float> probs_float(prob_maps_.size());
  for(int i=0; i<probs.size(); i++){
    probs_float[i] = probs[i];
  }
  std::cout << "Room Probs in :" << (ros::Time::now() - t).toSec() << std::endl;

  order = ordered(probs_float);
  return probs_float;
}


std::vector<float> RoomTypeMapper::getTypeCellNumberEstimate(const nav_msgs::OccupancyGrid& map, const std::vector<Door>& doors){
  if(prob_maps_.empty() || map.data.size() == 0)
    return std::vector<float>();

  ros::Time t = ros::Time::now();
  cv::Mat_<uchar> mask(map.info.height, map.info.width, uchar(0));
  for(int x=0; x<mask.cols; x++){
    for(int y=0; y<mask.rows; y++){
      if(map.data[y * map.info.width + x] >= 0)
        mask(y,x) = 255;
    }
  }
  cv::dilate(mask, mask, cv::Mat_<uchar>::ones(3,3), cv::Point(-1,-1), 2);
  cv::erode(mask, mask, cv::Mat_<uchar>::ones(3,3), cv::Point(-1,-1), 2);

  boost::lock_guard<boost::mutex> lock(maps_mutex_);
  std::vector<float> probs(prob_maps_.size(), 0.0);
  double res = 1.00 / map.info.resolution;
  double v = std::log(ROOM_PRIOR_PROB);
  int num = 0;
  for(int x=0; x<prob_maps_[0].getWidth(); x++){
    for(int y=0; y<prob_maps_[0].getHeight(); y++){
      if(prob_maps_[0].wasSeen(x,y)){
        float x_world = prob_maps_[0].getXWorld(x);
        float y_world = prob_maps_[0].getYWorld(y);
        int x_map = ((x_world - map.info.origin.position.x)) * res;
        int y_map = ((y_world - map.info.origin.position.y)) * res;
        if(x_map >= 0 && x_map < mask.cols && y_map >= 0 && y_map < mask.rows && mask(y_map, x_map) > 0){
          bool behind = false;
          for(const auto &door : doors){
            if(door.isBehindDoor(x_world, y_world)){
              behind = true;
              break;
            }
          }
          if(!behind){
            for(int i = 0; i < probs.size(); i++){
              probs[i] += prob_maps_[i].getProb(x, y);
            }
            num++;
          }
        }
      }
    }
  }
  for(int i = 0; i < probs.size(); i++){
    probs[i] /= num;
  }

  return probs;
}


std::vector<semantic_mapping_v2::RoomTypeMapMsg> RoomTypeMapper::getAllRoomTypeMapMsgs(){
  std::vector<semantic_mapping_v2::RoomTypeMapMsg> res;
  boost::lock_guard<boost::mutex> lock(maps_mutex_);
  for(const auto& map : prob_maps_){
    res.push_back(map.getRoomTypeMapMsg());
  }
  return res;
}
