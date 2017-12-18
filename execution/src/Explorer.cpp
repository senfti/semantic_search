//
// Created by thomas on 18.12.17.
//

#include <execution/Explorer.h>
#include <deque>

Explorer::Explorer(tf::TransformListener* tf_listener)
  : tf_listener_(tf_listener)
{
  map_sub_ = ros::NodeHandle().subscribe("map_door_blocked", 1, &Explorer::mapCb, this);
}

geometry_msgs::Pose Explorer::getNextFrontier(){
  frontier_changed_ = false;
  did_abort_ = false;
  return curr_frontier_;
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
  ros::Time t = ros::Time::now();
  cv::Mat_<uchar> unknown(msg->info.height, msg->info.width, uchar(0));
  cv::Mat_<uchar> free(msg->info.height, msg->info.width, uchar(0));
  cv::Mat_<uchar> occupied(msg->info.height, msg->info.width, uchar(0));
  for(int x=0; x<msg->info.width; x++){
    for(int y=0; y<msg->info.height; y++){
      if(msg->data[y * msg->info.width + x] == 0)
        free(y,x) = 255;
      else if(msg->data[y * msg->info.width + x] < 0)
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
  int robot_kernel_size = int(0.25/msg->info.resolution)*2+1;
  cv::dilate(occupied, occupied_dilate, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(robot_kernel_size,robot_kernel_size)));
  cv::bitwise_not(occupied_dilate, not_forbidden);
  cv::bitwise_and(not_forbidden, free, not_forbidden);

  tf::StampedTransform transform;
  try{
    tf_listener_->lookupTransform("map", "base_link", ros::Time(0), transform);
  }
  catch (tf::TransformException ex){
    ROS_ERROR("%s",ex.what());
  }
  cv::Point pos(int((transform.getOrigin().x()-msg->info.origin.position.x)/msg->info.resolution), int((transform.getOrigin().y()-msg->info.origin.position.y)/msg->info.resolution));
  //std::cout << pos.x << " " << pos.y << std::endl;
  cv::Point start = getNearestFree(not_forbidden, pos.x, pos.y);
  //std::cout << start.x << " " << start.y << std::endl;

  cv::Mat_<uchar> accessible(msg->info.height, msg->info.width, uchar(0));
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
//  std::vector<std::vector<cv::Point>> contours;
//  cv::findContours(not_forbidden, contours, cv::RETR_LIST, cv::CHAIN_APPROX_NONE);
//  for(int i=0; i<contours.size(); i++){
//    cv::drawContours(accessible, contours, i, cv::Scalar(255), -1);
//    cv::dilate(accessible, accessible, cv::Mat_<uchar>::ones(3,3));
//    if(accessible(start))
//      break;
//    accessible = cv::Mat_<uchar>(msg->info.height, msg->info.width, uchar(0));
//  }
//  accessible(start) = 100;
//  not_forbidden(start) = 100;

  cv::Mat_<uchar> good_frontiers_mask;
  cv::bitwise_and(accessible, frontiers_mask, good_frontiers_mask);

  double min_dist = std::numeric_limits<double>::max();
  cv::Point frontier(-1,-1);
  for(int x=0; x<msg->info.width; x++){
    for(int y=0; y<msg->info.height; y++){
      if(good_frontiers_mask(y,x)){
        double dist = (x-pos.x)*(x-pos.x) + (y-pos.y)*(y-pos.y);
        if(dist < min_dist){
          frontier = cv::Point(x,y);
          min_dist = dist;
        }
      }
    }
  }
  if(frontier.x == -1){
    finished_ = true;
    return;
  }
  else
    finished_ = false;

  if(frontier.x != curr_point_.x || frontier.y != curr_point_.y){
    //std::cout << frontier.x << " " << frontier.y << std::endl;
    curr_point_ = frontier;
    frontier_changed_ = true;
//    double cx = 0.0, cy=0.0, sum=0.0;
//    for(int x=std::max(0,frontier.x-20); x<=std::min(good_frontiers_mask.cols-1,frontier.x+20); x++){
//      for(int y=std::max(0,frontier.y-20); y<=std::min(good_frontiers_mask.rows-1,frontier.y+20); y++){
//        if(good_frontiers_mask(y,x)){
//          cx+=x-frontier.x;
//          cy+=y-frontier.y;
//        }
//      }
//    }
//    double angle = std::atan2(cy,cx);
    double angle = std::atan2(frontier.y-pos.y, frontier.x-pos.x);
    cv::Point view_pos = frontier;
    for(double angle_diff=0.0; angle_diff<M_PI; angle_diff+=M_PI/30){
      cv::Point p(int(frontier.y-10*std::sin(angle+angle_diff)), int(frontier.x-10*std::cos(angle+angle_diff)));
      if(lineFree(frontier, p, accessible)){
        view_pos = p;
        angle = angle+angle_diff;
        break;
      }
      p = cv::Point(int(frontier.y-10*std::sin(angle-angle_diff)), int(frontier.x-10*std::cos(angle-angle_diff)));
      if(lineFree(frontier, p, accessible)){
        view_pos = p;
        angle = angle-angle_diff;
        break;
      }
    }

    tf::Transform tf_t(tf::Quaternion(tf::Vector3(0.0,0.0,1.0),angle),
                       tf::Vector3(view_pos.x*msg->info.resolution+msg->info.origin.position.x, view_pos.y*msg->info.resolution+msg->info.origin.position.y, 0.0));
    tf::poseTFToMsg(tf_t, curr_frontier_);
  }

  //std::cout << (ros::Time::now()-t).toSec() << std::endl;

  //cv::resize(frontiers_mask, frontiers_mask, cv::Size(msg->info.width*2, msg->info.height*2), 0, 0, cv::INTER_NEAREST);
  cv::imshow("frontiers", frontiers_mask);
  //cv::resize(not_forbidden, not_forbidden, cv::Size(msg->info.width*2, msg->info.height*2), 0, 0, cv::INTER_NEAREST);
  cv::imshow("not_forbidden", not_forbidden);
  //cv::resize(accessible, accessible, cv::Size(msg->info.width*2, msg->info.height*2), 0, 0, cv::INTER_NEAREST);
  cv::imshow("accessible", accessible);
  //cv::resize(good_frontiers_mask, good_frontiers_mask, cv::Size(msg->info.width*2, msg->info.height*2), 0, 0, cv::INTER_NEAREST);
  cv::imshow("good_frontiers_mask", good_frontiers_mask);
  cv::waitKey(1);
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
