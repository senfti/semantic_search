//
// Created by thomas on 18.12.17.
//

#include <execution_test/Explorer.h>
#include <deque>
#include <set>
#include <std_msgs/Int8.h>

Explorer::Explorer(tf::TransformListener* tf_listener)
  : tf_listener_(tf_listener)
{
  map_sub_ = ros::NodeHandle().subscribe("map_door_blocked", 1, &Explorer::mapCb, this);
  door_found_sub_ = ros::NodeHandle().subscribe("door_found", 1, &Explorer::doorFoundCb, this);
  obj_found_sub_ = ros::NodeHandle().subscribe("obj_found", 1, &Explorer::objFoundCb, this);
  map_pub_ = ros::NodeHandle().advertise<nav_msgs::OccupancyGrid>("frontier_map", 1, true);
  acc_map_pub_ = ros::NodeHandle().advertise<nav_msgs::OccupancyGrid>("acc_map", 1, true);
  obj_found_pub_ = ros::NodeHandle().advertise<geometry_msgs::PoseStamped>("object_found_pose", 1);

  ros::NodeHandle("~").param("EXPLORE_MAX_ROT_VEL", EXPLORE_MAX_ROT_VEL, EXPLORE_MAX_ROT_VEL);
  ros::NodeHandle("~").param("EXPLORE_MAX_TRANS_VEL", EXPLORE_MAX_TRANS_VEL, EXPLORE_MAX_TRANS_VEL);
  ros::NodeHandle("~").param("VIEW_DIST", VIEW_DIST, VIEW_DIST);
  ros::NodeHandle("~").param("ROBOT_SIZE", ROBOT_SIZE, ROBOT_SIZE);
  ros::NodeHandle("~").param("OBJECT_FOUND_THRESH", OBJECT_FOUND_THRESH, OBJECT_FOUND_THRESH);

  cv::Mat_<uchar> mat(VIEW_DIST*4, VIEW_DIST*4, uchar(0));
  cv::circle(mat, cv::Point(2*VIEW_DIST, 2*VIEW_DIST), VIEW_DIST, cv::Scalar(255), 1);
  for(int x=0;x<mat.cols;x++){
    for(int y=0; y<mat.rows; y++){
      if(mat(y,x))
        circle_points_.push_back(cv::Point(x-2*VIEW_DIST,y-2*VIEW_DIST));
    }
  }
  for(int x=-2;x<=2;x++){
    for(int y=-2; y<=2; y++){
      if(x==-2 || x==2 || y==-2 || y==2)
        near_circle_points_.push_back(cv::Point(x,y));
    }
  }
}

geometry_msgs::Pose Explorer::getNextFrontier(){
  frontier_changed_ = false;
  did_abort_ = false;
  return curr_frontier_;
}

void Explorer::start(int searched_obj){
  running_ = true;
  door_found_stopped_ = false;
  searched_obj_ = searched_obj;
  obj_found_stopped_ = false;
  finished_ = false;
  finished_count_ = 0;
  curr_frontier_.position.x = -99999999999.9;
  curr_frontier_.orientation.z = 0.0;
  curr_frontier_.orientation.w = 1.0;
//  std::system(("rosrun dynamic_reconfigure dynparam set /move_base/DWAPlannerROS max_rot_vel " + std::to_string(EXPLORE_MAX_ROT_VEL)).c_str());
//  std::system(("rosrun dynamic_reconfigure dynparam set /move_base/DWAPlannerROS max_trans_vel " + std::to_string(EXPLORE_MAX_TRANS_VEL)).c_str());
  calcFrontier();
  ROS_INFO("EXPLORATION STARTED");
}

void Explorer::stop(){
  if(running_)
    ROS_INFO("EXPLORATION STOPPED");
  running_ = false;
  door_found_stopped_ = false;
  obj_found_stopped_ = false;
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
bool lineFree(const cv::Point& start, const cv::Point& end, const cv::Mat_<uchar> occupied_mat){
  int x0=start.x;
  int y0=start.y;
  int dx =  abs(end.x-x0), sx = x0<end.x ? 1 : -1;
  int dy = -abs(end.y-y0), sy = y0<end.y ? 1 : -1;
  int err = dx+dy, e2; /* error value e_xy */

  while(1){
    if(occupied_mat(y0,x0))
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


void Explorer::objFoundCb(const semantic_mapping_v2::ObjFoundMsgConstPtr& msg){
  if(searched_obj_ >= int(msg->poses.size()) || searched_obj_ < 0 || !running_)
    return;
  std::cout << "Max Obj Prob: " << msg->probs[searched_obj_] << " / " << OBJECT_FOUND_THRESH << std::endl;
  if(msg->probs[searched_obj_] > OBJECT_FOUND_THRESH){
    found_pose_.header.stamp = ros::Time::now();
    found_pose_.header.frame_id = "map";
    found_pose_.pose = msg->poses[searched_obj_];
    obj_found_pub_.publish(found_pose_);
    obj_found_stopped_ = true;
    finished_ = true;
  }
}


void imout(std::string n, cv::Mat sdf){
  cv::Mat asd = sdf(cv::Rect(cv::Point(146,158), cv::Point(289,323)));
  cv::resize(asd,asd,cv::Size(sdf.cols*4, sdf.rows*4),0,0,cv::INTER_NEAREST);
  cv::imwrite(n, asd);
}


struct CompareFrontier{
  bool operator() (const std::pair<cv::Point,double>& lhs, const std::pair<cv::Point,double>& rhs) const
  {return lhs.second<rhs.second;}
};

void Explorer::calcFrontier(){
  static ros::Time last_calculation(0.0);
  ros::Time t = ros::Time::now();
//  if(t-last_calculation < ros::Duration(0.5))
//    return;
  last_calculation = t;

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
  imout("/tmp/unknown.png", unknown);
  imout("/tmp/free.png", free);
  imout("/tmp/occupied.png", occupied);
  imout("/tmp/unknown_dilate.png", unknown_dilate);
  imout("/tmp/frontiers_mask.png", frontiers_mask);

  cv::Mat_<uchar> occupied_dilate, not_forbidden;
  int robot_kernel_size = ROBOT_SIZE*2+1;
  cv::dilate(occupied, occupied_dilate, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(robot_kernel_size,robot_kernel_size)));
  imout("/tmp/occupied_dilate.png", occupied_dilate);
  cv::bitwise_not(occupied_dilate, not_forbidden);
  cv::erode(free, free, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(robot_kernel_size,robot_kernel_size)));
  cv::dilate(free, free, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(robot_kernel_size,robot_kernel_size)));
  imout("/tmp/free2.png", free);
  imout("/tmp/not_forbidden.png", not_forbidden);
  cv::bitwise_and(not_forbidden, free, not_forbidden);
  imout("/tmp/not_forbidden2.png", not_forbidden);

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
  imout("/tmp/accessible.png", accessible);

  cv::Mat_<uchar> good_frontiers_mask;
  cv::bitwise_and(accessible, frontiers_mask, good_frontiers_mask);
  imout("/tmp/good_frontiers_mask.png", good_frontiers_mask);
  std::vector<cv::Mat_<uchar>> good_accessibles(5);
  cv::erode(accessible, good_accessibles[0], cv::Mat_<uchar>::ones(3,3));
  cv::Mat_<uchar> good_tmp(accessible.rows, accessible.cols, uchar(255));
  good_tmp = good_tmp - good_accessibles[0]/5;
  for(int i=1; i<good_accessibles.size(); i++){
    cv::erode(good_accessibles[i-1], good_accessibles[i], cv::Mat_<uchar>::ones(3,3));
    good_tmp = good_tmp - good_accessibles[i]/5;
  }
  imout("/tmp/good_tmp.png", good_tmp);
  nav_msgs::OccupancyGrid map = last_map_;
  nav_msgs::OccupancyGrid acc_map = last_map_;
  for(int x=0; x<last_map_.info.width; x++){
    for(int y=0; y<last_map_.info.height; y++){
      if(good_frontiers_mask(y,x))
        map.data[y * last_map_.info.width + x] = 254;
      else if(accessible(y,x))
        map.data[y * last_map_.info.width + x] = 50;
      else
        map.data[y * last_map_.info.width + x] = -1;
      //map.data[y * last_map_.info.width + x] = (good_frontiers_mask(y,x) ? 0 : -1);
      if(accessible(y,x))
        acc_map.data[y * last_map_.info.width + x] = 0;
      else
        acc_map.data[y * last_map_.info.width + x] = 100;
    }
  }
  map_pub_.publish(map);
  acc_map_pub_.publish(acc_map);

//  cv::imshow("explore_accessible", accessible);
//  cv::imshow("good_frontiers", good_frontiers_mask);
//  cv::waitKey(1);

  cv::Mat_<uchar> poseses=255-accessible/255*230;
  cv::Point best_pos(-1,-1);
  double best_dir = 0.0;
  double best_dist = 999999999999999.9;
  for(int x=0; x<last_map_.info.width; x++){
    for(int y=0; y<last_map_.info.height; y++){
      if(good_frontiers_mask(y,x)){
        for(const auto& cp : near_circle_points_){
          cv::Point p = cv::Point(x,y)+cp;
          double dist = std::sqrt((p.x-pos.x)*(p.x-pos.x) + (p.y-pos.y)*(p.y-pos.y));
          dist += (dist < ROBOT_SIZE ? 40.0 : 0.0);
          for(const auto& m : good_accessibles)
            dist += (m(p) ? 0.0 : 20.0);
          if(dist < best_dist){
            best_dist = dist;
            best_pos = cv::Point(p);
            best_dir = std::atan2(-cp.y,-cp.x);
          }
          if(accessible(p))
            poseses(p) = 128;
          if(good_frontiers_mask(p))
            poseses(p) = 0;
        }
      }
    }
  }
  imout("/tmp/poseses.png", poseses);


  if(best_pos.x < 0){
    if(finished_count_ > 3){
      finished_ = true;
      std::cout << "EXPLORATION FINISHED FOUND in" << (ros::Time::now() - t).toSec() << std::endl;
    }
    finished_count_++;
    return;
  }

  bool inside = false;
  if((pos-best_pos).ddot(pos-best_pos) < ROBOT_SIZE*ROBOT_SIZE*1.5){
    bool pos_found = false;
    for(const auto& offset : circle_points_){
      cv::Point p = best_pos+offset;
      if(p.x>=0 && p.y>=0 && p.x<accessible.cols && p.y<accessible.rows && good_accessibles[3](p)){
        best_pos = p;
        pos_found = true;
        best_dir = std::atan2(-offset.y,-offset.x);
        break;
      }
    }
    if(!pos_found){
      for(const auto &offset : circle_points_){
        cv::Point p = best_pos + offset;
        if(p.x >= 0 && p.y >= 0 && p.x < accessible.cols && p.y < accessible.rows && accessible(p)){
          best_pos = p;
          pos_found = true;
          best_dir = std::atan2(-offset.y,-offset.x);
          break;
        }
      }
    }
    if(!pos_found){
      if(finished_count_ > 3){
        finished_ = true;
        std::cout << "EXPLORATION FINISHED FOUND in" << (ros::Time::now() - t).toSec() << std::endl;
      }
      finished_count_++;
      return;
    }
  }

  tf::Transform old_frontier;
  tf::poseMsgToTF(curr_frontier_, old_frontier);
  tf::Transform tf_t(tf::createQuaternionFromYaw(best_dir),
                           tf::Vector3(best_pos.x*last_map_.info.resolution+last_map_.info.origin.position.x,
                                       best_pos.y*last_map_.info.resolution+last_map_.info.origin.position.y, 0.0));
  tf::Transform diff = old_frontier.inverseTimes(tf_t);
  static ros::Time last_frontier_change = ros::Time(0);
  static tf::Vector3 last_frontier_calc_pos = tf::Vector3(-999999999999.9,0.0,0.0);
  if(diff.getOrigin().length() > 0.2 || std::abs(tf::getYaw(diff.getRotation())) > 0.1
     || (ros::Time::now()-last_frontier_change > ros::Duration(5.0) && (transform.getOrigin()-last_frontier_calc_pos).length() < 0.3)){
    frontier_changed_ = true;
    last_frontier_calc_pos = transform.getOrigin();
    last_frontier_change = ros::Time::now();
    tf::poseTFToMsg(tf_t, curr_frontier_);
    std::cout << "NEW FRONTIER in " << (ros::Time::now()-t).toSec() << std::endl;
  }
  std::cout << "No frontier change in " << (ros::Time::now()-t).toSec() << std::endl;
  finished_count_ = 0;
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
