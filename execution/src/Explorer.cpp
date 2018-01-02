//
// Created by thomas on 18.12.17.
//

#include <execution/Explorer.h>
#include <deque>
#include <set>

Explorer::Explorer(tf::TransformListener* tf_listener)
  : tf_listener_(tf_listener)
{
  map_sub_ = ros::NodeHandle().subscribe("map_door_blocked", 1, &Explorer::mapCb, this);
  door_found_sub_ = ros::NodeHandle().subscribe("door_found", 1, &Explorer::doorFoundCb, this);
  map_pub_ = ros::NodeHandle().advertise<nav_msgs::OccupancyGrid>("frontier_map", 1, true);

  cv::Mat_<uchar> mat(VIEW_DIST*4, VIEW_DIST*4, uchar(0));
  cv::circle(mat, cv::Point(2*VIEW_DIST, 2*VIEW_DIST), VIEW_DIST, cv::Scalar(255), 1);
  for(int x=0;x<mat.cols;x++){
    for(int y=0; y<mat.rows; y++){
      if(mat(y,x))
        circle_points_.push_back(cv::Point(x-2*VIEW_DIST,y-2*VIEW_DIST));
    }
  }
}

geometry_msgs::Pose Explorer::getNextFrontier(){
  frontier_changed_ = false;
  did_abort_ = false;
  return curr_frontier_;
}

void Explorer::start(){
  running_ = true;
  door_found_stopped_ = false;
  finished_ = false;
  std::system(("rosrun dynamic_reconfigure dynparam set /move_base/DWAPlannerROS max_rot_vel " + std::to_string(EXPLORE_MAX_ROT_VEL)).c_str());
  std::system(("rosrun dynamic_reconfigure dynparam set /move_base/DWAPlannerROS max_trans_vel " + std::to_string(EXPLORE_MAX_TRANS_VEL)).c_str());
  calcFrontier();
  ROS_INFO("EXPLORATION STARTED");
}

void Explorer::stop(){
  if(running_)
    ROS_INFO("EXPLORATION STOPPED");
  running_ = false;
  door_found_stopped_ = false;
  finished_ = false;
}

void insertNeighbors(const cv::Point& p, cv::Mat_<uchar>& already_inserted, std::deque<cv::Point>& list){
  for(int x=std::max(0,p.x-1); x<=std::min(already_inserted.cols-1, p.x+1); x++){
    for(int y=std::max(0,p.y-1); y<=std::min(already_inserted.rows-1, p.y+1); y++){
      if((x!=p.x || y!=p.y) && !already_inserted(y,x)){
        list.push_back(cv::Point(x,y));
        already_inserted(y,x) = 255;
      }
    }
  }
}

// based on Bresenham on German Wikipedia
bool lineFree(const cv::Point& start, const cv::Point& end, const cv::Mat_<uchar> free_mat){
  int x0=start.x;
  int y0=start.y;
  int dx =  abs(end.x-x0), sx = x0<end.x ? 1 : -1;
  int dy = -abs(end.y-y0), sy = y0<end.y ? 1 : -1;
  int err = dx+dy, e2; /* error value e_xy */

  while(1){
    if(!free_mat(y0,x0))
      return false;
    if (x0==end.x && y0==end.y)
      break;
    e2 = 2*err;
    if (e2 > dy) { err += dy; x0 += sx; } /* e_xy+e_x > 0 */
    if (e2 < dx) { err += dx; y0 += sy; } /* e_xy+e_y < 0 */
  }
  return true;
}

void Explorer::mapCb(const nav_msgs::OccupancyGridConstPtr &msg){
  last_map_ = *msg;
//  std::cout << (running_ ? "run" : "stop") << std::endl;
  if(running_)
    calcFrontier();
}


void Explorer::doorFoundCb(const std_msgs::Int8& msg){
  door_found_stopped_ = true;
  finished_ = true;
}


struct CompareFrontier{
  bool operator() (const std::pair<cv::Point,double>& lhs, const std::pair<cv::Point,double>& rhs) const
  {return lhs.second<rhs.second;}
};

void Explorer::calcFrontier(){
  ros::Time t = ros::Time::now();
  cv::Mat_<uchar> unknown(last_map_.info.height, last_map_.info.width, uchar(0));
  cv::Mat_<uchar> free(last_map_.info.height, last_map_.info.width, uchar(0));
  cv::Mat_<uchar> occupied(last_map_.info.height, last_map_.info.width, uchar(0));
  for(int x=0; x<last_map_.info.width; x++){
    for(int y=0; y<last_map_.info.height; y++){
      if(last_map_.data[y * last_map_.info.width + x] == 0)
        free(y,x) = 255;
      else if(last_map_.data[y * last_map_.info.width + x] < 0)
        unknown(y,x) = 255;
      else
        occupied(y,x) = 255;
    }
  }
  cv::Mat_<uchar> unknown_dilate;
  cv::dilate(unknown, unknown_dilate, cv::Mat_<uchar>::ones(3,3));
  cv::Mat_<uchar> frontiers_mask;
  cv::bitwise_and(unknown_dilate, free, frontiers_mask);

  cv::Mat_<uchar> occupied_dilate, not_forbidden;
  int robot_kernel_size = ROBOT_SIZE*2+1;
  cv::dilate(occupied, occupied_dilate, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(robot_kernel_size,robot_kernel_size)));
  cv::bitwise_not(occupied_dilate, not_forbidden);
  cv::erode(free, free, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(robot_kernel_size,robot_kernel_size)));
  cv::dilate(free, free, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(robot_kernel_size,robot_kernel_size)));
  cv::bitwise_and(not_forbidden, free, not_forbidden);

  nav_msgs::OccupancyGrid map = last_map_;
  for(int x=0; x<last_map_.info.width; x++){
    for(int y=0; y<last_map_.info.height; y++){
      map.data[y * last_map_.info.width + x] = (not_forbidden(y,x) ? 0 : 100);
    }
  }
  map_pub_.publish(map);

  tf::StampedTransform transform;
  try{
    tf_listener_->lookupTransform("map", "base_link", ros::Time(0), transform);
  }
  catch (tf::TransformException ex){
    ROS_ERROR("%s",ex.what());
  }
  cv::Point pos(int((transform.getOrigin().x()-last_map_.info.origin.position.x)/last_map_.info.resolution),
                int((transform.getOrigin().y()-last_map_.info.origin.position.y)/last_map_.info.resolution));
  cv::Point start = getNearestFree(not_forbidden, pos.x, pos.y);

  cv::Mat_<uchar> accessible(last_map_.info.height, last_map_.info.width, uchar(0));
  std::deque<cv::Point> next[2];
  cv::Mat_<uchar> already_inserted;
  cv::bitwise_not(not_forbidden, already_inserted);
  already_inserted(start) = 255;
  int i=0;
  next[i].push_back(start);
  while(!next[i&1].empty()){
    for(const auto& p : next[i&1]){
      if(not_forbidden(p)){
        accessible(p) = 255;
      }
      insertNeighbors(p, already_inserted, next[(i+1)&1]);
    }
    next[i&1].clear();
    i++;
  }

  cv::Mat_<uchar> good_frontiers_mask;
  cv::bitwise_and(accessible, frontiers_mask, good_frontiers_mask);

  cv::Point best_pos = pos + cv::Point(transform.getBasis().getColumn(0).x() * ROBOT_SIZE, transform.getBasis().getColumn(0).y() * ROBOT_SIZE);
  std::set<std::pair<cv::Point,double>, CompareFrontier> frontiers;
  for(int x=0; x<last_map_.info.width; x++){
    for(int y=0; y<last_map_.info.height; y++){
      if(good_frontiers_mask(y,x)){
        double dist = (x-best_pos.x)*(x-best_pos.x) + (y-best_pos.y)*(y-best_pos.y);
        frontiers.insert(std::pair<cv::Point,double>(cv::Point(x,y), dist));
      }
    }
  }

  geometry_msgs::Pose old_frontier = curr_frontier_;
  for(const auto& f : frontiers){
    std::set<std::pair<cv::Point,double>, CompareFrontier> view_points;
    for(const auto& offset : circle_points_){
      cv::Point p = f.first+offset;
      if(p.x>=0 && p.y>=0 && p.x<accessible.cols && p.y<accessible.rows && accessible(p)){
        double dist = (p.x-best_pos.x)*(p.x-best_pos.x) + (p.y-best_pos.y)*(p.y-best_pos.y);
        view_points.insert(std::pair<cv::Point,double>(p, dist));
      }
    }
    for(const auto& v : view_points){
      if(lineFree(f.first, v.first, free)){
        double angle = std::atan2(f.first.y-v.first.y, f.first.x-v.first.x);
        tf::Transform tf_t(tf::Quaternion(tf::Vector3(0.0,0.0,1.0),angle),
                           tf::Vector3(v.first.x*last_map_.info.resolution+last_map_.info.origin.position.x,
                                       v.first.y*last_map_.info.resolution+last_map_.info.origin.position.y, 0.0));
        tf::Transform diff_t = transform.inverse()*tf_t;
        if(diff_t.getOrigin().length() < 0.1 && tf::getYaw(diff_t.getRotation()) < 0.05)
          continue;

        tf::poseTFToMsg(tf_t, curr_frontier_);
        if(old_frontier.position.x != curr_frontier_.position.x || old_frontier.position.y != curr_frontier_.position.y || old_frontier.orientation.w != curr_frontier_.orientation.w){
          frontier_changed_ = true;
        }
        std::cout << "NEW FRONTIER in" << (ros::Time::now()-t).toSec() << std::endl;
        return;
      }
    }
  }
  std::cout << "EXPLORATION FINISHED FOUND in" << (ros::Time::now()-t).toSec() << std::endl;
  finished_ = true;
}


cv::Point Explorer::getNearestFree(const cv::Mat_<uchar>& valid, int x, int y) const{
  if(valid(y,x))
    return cv::Point(x,y);

  std::deque<cv::Point> next[2];
  cv::Mat_<uchar> already_inserted(valid.rows, valid.cols, uchar(0));
  already_inserted(y,x) = 255;
  int i=0;
  next[i].push_back(cv::Point(x,y));
  while(!next[i&1].empty()){
    for(const auto& p : next[i&1]){
      if(valid(p)){
        return p;
      }
      insertNeighbors(p, already_inserted, next[(i+1)&1]);
    }
    next[i&1].clear();
    i++;
  }
  return cv::Point(-1,-1);
}
