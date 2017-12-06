//
// Created by thomas on 23.11.17.
//

#include "semantic_mapping_v2/RoomTypeMapper.h"


RoomTypeMap::RoomTypeMap(float resolution, float start_size, float initial_value, const std::string& name)
  : resolution_(resolution), base_size_(start_size*resolution), name_(name)
{
  origin_ = cv::Point(base_size_/2, base_size_/2);
  prob_map_ = cv::Mat_<float>(base_size_, base_size_, initial_value);
}

RoomTypeMap::RoomTypeMap(float resolution, int base_size, int width, int height, const cv::Point& origin, float initial_value, const std::string& name)
  : resolution_(resolution), base_size_(base_size), origin_(origin), name_(name)
{
  prob_map_ = cv::Mat_<float>(base_size_, base_size_, initial_value);
}

RoomTypeMap::RoomTypeMap(const RoomTypeMap& rhs){
  resolution_ = rhs.resolution_;
  base_size_ = rhs.base_size_;
  origin_ = rhs.origin_;
  rhs.prob_map_.copyTo(prob_map_);
  name_ = rhs.name_;
}

RoomTypeMap& RoomTypeMap::operator=(const RoomTypeMap& rhs){
  if(this == &rhs)
    return *this;

  resolution_ = rhs.resolution_;
  base_size_ = rhs.base_size_;
  origin_ = rhs.origin_;
  rhs.prob_map_.copyTo(prob_map_);
  name_ = rhs.name_;
  return *this;
}

void RoomTypeMap::resize(int left, int right, int top, int bottom, float prior){
  origin_ += cv::Point(left, top);
  cv::copyMakeBorder(prob_map_, prob_map_, top, bottom, left, right, cv::BORDER_CONSTANT, cv::Scalar(prior));
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

  visualization_msgs::MarkerArray markers;
  markers.markers.resize(1);
  markers.markers[0] = def;
  float min=1.f, max=0.f;
  for(int x=0; x<getWidth(); x++){
    for(int y=0; y<getHeight(); y++){
      min = std::min(min, getProb(x,y));
      max = std::max(max, getProb(x,y));

      cv::Mat_<cv::Vec3b> color(1,1,cv::Vec3b(getProb(x,y)*150, 255, 255));
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

  for(auto& map : prob_maps_)
    map.resize(left, right, top, bottom, ROOM_PRIOR_PROB);

  for(int i=0; i<points.size(); i++){
    points[i].x += left;
    points[i].y += top;
  }
  return true;
}


void RoomTypeMapper::updateProbs(const vision::VisionMsgConstPtr &msg, int x, int y){
  std::vector<float> probs(prob_maps_.size());
  for(int i=0; i<prob_maps_.size(); i++){
    probs[i] = prob_maps_[i].getProb(x,y);
  }

  double sum = 0.0;
  for(int i=0; i<num_types_; i++){
    probs[i] = (ROOM_CONFIDENCE * msg->place_guesses[i].prob/ROOM_PRIOR_PROB + 1-ROOM_CONFIDENCE) * probs[i];
    sum += probs[i];
  }
  std::vector<int> not_low_idx(num_types_);
  for(int i=0; i<num_types_; i++){
    probs[i] /= sum;
    not_low_idx[i] = i;
  }

  while(true){
    std::vector<int> old_not_low_idx = not_low_idx;
    not_low_idx.clear();
    bool did_thresh = false;
    for(int i=0; i<old_not_low_idx.size(); i++){
      if(probs[old_not_low_idx[i]] <= ROOM_MIN_PROB){
        probs[old_not_low_idx[i]] = ROOM_MIN_PROB;
        did_thresh = true;
      }
      else
        not_low_idx.push_back(old_not_low_idx[i]);
    }
    if(!did_thresh)
      break;

    double rest_sum = 1.0-(num_types_-not_low_idx.size())*ROOM_MIN_PROB;
    sum = 0.0;
    for(int i=0; i<not_low_idx.size(); i++){
      sum += probs[not_low_idx[i]];
    }
    for(int i=0; i<not_low_idx.size(); i++){
      probs[not_low_idx[i]] *= rest_sum/sum;
    }
  }

  sum = 0.0;
  for(int i=0; i<prob_maps_.size(); i++){
    prob_maps_[i].setProb(x,y,probs[i]);
    sum += probs[i];
  }
  if(sum < 0.99 || sum > 1.01)
    std::cout << "_______________________________ sum bad " << sum << std::endl;
}


void RoomTypeMapper::processMsg(const vision::VisionMsgConstPtr& msg, const GMapping::OrientedPoint& pose){
  if(num_types_ == 0){
    num_types_ = msg->place_guesses.size();
    ROOM_PRIOR_PROB = 1.f/num_types_;
    prob_maps_.resize(num_types_, RoomTypeMap(ROOM_DEFAULT_RESOLUTION, 10.f, ROOM_PRIOR_PROB));
    names_.resize(num_types_);
    for(int i=0; i<num_types_; i++){
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
  cv::Mat_<uchar> mask(prob_maps_[0].getHeight(), prob_maps_[0].getWidth(), uchar(0));
  cv::fillConvexPoly(mask, points, cv::Scalar(255));
  for(int x=0; x<mask.cols; x++){
    for(int y=0; y<mask.rows; y++){
      if(mask(y,x))
        updateProbs(msg, x, y);
    }
  }

  curr_probs_.clear();
}


template <typename T>
std::vector<size_t> ordered(std::vector<T> const& values) {
  std::vector<size_t> indices(values.size());
  std::iota(begin(indices), end(indices), static_cast<size_t>(0));
  std::sort(begin(indices), end(indices),[&](size_t a, size_t b) { return values[a] > values[b]; });
  return indices;
}

std::vector<float> RoomTypeMapper::getRoomProb(const nav_msgs::OccupancyGrid& map, std::vector<size_t>& order){
  if(prob_maps_.empty())
    return std::vector<float>();

  if(curr_probs_.empty()){
    ros::Time t = ros::Time::now();
    cv::Mat_<uchar> mask(map.info.height, map.info.width, uchar(0));
    for(int x=0; x<mask.cols; x++){
      for(int y=0; y<mask.rows; y++){
        if(map.data[y * map.info.width + x] >= 0)
          mask(y,x) = 255;
      }
    }
    cv::dilate(mask, mask, cv::Mat_<uchar>::ones(3,3), cv::Point(-1,-1), 5);

    std::vector<double> probs(prob_maps_.size(), 0.0);
    double v = std::log(ROOM_PRIOR_PROB);
    double res = 1.00 / map.info.resolution;
    int num = 0;
    for(int x=0; x<prob_maps_[0].getWidth(); x++){
      for(int y=0; y<prob_maps_[0].getHeight(); y++){
        bool is_valid = false;
        int x_map = ((prob_maps_[0].getXWorld(x) - map.info.origin.position.x))*res;
        int y_map = ((prob_maps_[0].getYWorld(y) - map.info.origin.position.y))*res;
        if(x_map>=0 && x_map < mask.cols && y_map>=0 && y_map<mask.rows && mask(y_map,x_map) > 0){
          for(int i=0; i<probs.size(); i++){
            probs[i] += std::log(prob_maps_[i].getProb(x,y)) - v;
          }
          num++;
        }
      }
    }

    double sum = 0.0;
    for(int i=0; i<probs.size(); i++){
      probs[i] /= num;
      std::cout << probs[i] << std::endl;
      probs[i] = std::exp(probs[i]);
      sum += probs[i];
    }
    curr_probs_.resize(probs.size());
    for(int i=0; i<probs.size(); i++){
      curr_probs_[i] = probs[i]/sum;
    }
    std::cout << "Room Probs in :" << (ros::Time::now() - t).toSec() << std::endl;
  }

  order = ordered(curr_probs_);
  return curr_probs_;
}
