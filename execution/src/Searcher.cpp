//
// Created by thomas on 02.01.18.
//

#include <execution/Searcher.h>
#include <pcl/common/common.h>
#include <semantic_mapping_v2/ObjectMapSrv.h>
#include <tf/transform_datatypes.h>

template <class T>
Map<T>::Map(const Map<T> &rhs)
  : resolution_(rhs.resolution_), origin_(rhs.origin_), default_value_(rhs.default_value_)
{
  rhs.map_.copyTo(map_);
}

template <class T>
Map<T>::Map(const cv::Mat_<T> &map, float resolution, const cv::Point &origin, double default_value)
  : resolution_(resolution), origin_(origin), default_value_(default_value)
{
  map.copyTo(map_);
  makeNiceSize();
}

template <class T>
void Map<T>::resize(int x1, int x2, int y1, int y2){
  int top=0, bottom=0, left=0, right=0, old_width=map_.cols, old_height=map_.rows;
  while(x1+left < 0)
    left += BASE_SIZE;
  while(x2 >= old_width+right)
    right += BASE_SIZE;
  while(y1+top < 0)
    top += BASE_SIZE;
  while(y2 >= old_height+bottom)
    bottom += BASE_SIZE;

  if(top!=0 || bottom!=0 || left!=0 || right!=0){
    cv::copyMakeBorder(map_, map_, top, bottom, left, right, cv::BORDER_CONSTANT, default_value_);
    origin_ += cv::Point(left, top);
  }
}


template <class T>
void Map<T>::makeNiceSize(){
  int x1 = (origin_.x-BASE_SIZE/2)-std::ceil(float(origin_.x-BASE_SIZE/2)/BASE_SIZE)*BASE_SIZE;
  int y1 = (origin_.y-BASE_SIZE/2)-std::ceil(float(origin_.y-BASE_SIZE/2)/BASE_SIZE)*BASE_SIZE;
  int x2 = std::ceil(float(map_.cols-x1)/BASE_SIZE)*BASE_SIZE-(map_.cols-x1);
  int y2 = std::ceil(float(map_.rows-y1)/BASE_SIZE)*BASE_SIZE-(map_.rows-y1);
  resize(x1, x2, y1, y2);
}

template <class T>
void Map<T>::resample(float new_resolution){
  float factor = new_resolution/resolution_;
  if(factor == 1.f)
    return;

  cv::resize(map_, map_, cv::Size(map_.cols*factor, map_.rows*factor), 0, 0, cv::INTER_NEAREST);
  origin_ = cv::Point(origin_.x*factor, origin_.y*factor);
  makeNiceSize();
}

template <class T>
void Map<T>::makeSame(Map<T>& other_map, float resolution){
  resample(resolution);
  other_map.resample(resolution);

  if(origin_.x < other_map.origin_.x)
    resize(origin_.x-other_map.origin_.x, 0, 0, 0);
  else if(origin_.x > other_map.origin_.x)
    other_map.resize(other_map.origin_.x-origin_.x, 0, 0, 0);

  if(origin_.y < other_map.origin_.y)
    resize(0, origin_.y-other_map.origin_.y, 0, 0);
  else if(origin_.y > other_map.origin_.y)
    other_map.resize(0, other_map.origin_.y-origin_.y, 0, 0);

  if(map_.cols < other_map.map_.cols)
    resize(0, 0, other_map.map_.cols, 0);
  else if(map_.cols > other_map.map_.cols)
    other_map.resize(0, 0, map_.cols, 0);

  if(map_.rows < other_map.map_.rows)
    resize(0, 0, 0, other_map.map_.rows);
  else if(map_.rows > other_map.map_.rows)
    other_map.resize(0, 0, 0, map_.rows);
}

template <class T>
Map<T> Map<T>::operator*(const Map<T> &rhs){
  Map res(*this);
  if(resolution_ != rhs.resolution_ || map_.cols != rhs.map_.cols || map_.rows != rhs.map_.rows){
    Map m2(rhs);
    res.makeSame(res, std::max(resolution_, rhs.resolution_));
    res.map_ = map_.mul(m2.map_);
  }
  else{
    res.map_ = map_.mul(rhs.map_);
  }
  return res;
}

template <class T>
Map<T> Map<T>::operator*(const cv::Mat_<T>& rhs){
  Map res(*this);
  res.map_ = map_.mul(rhs);
  return res;
}


Searcher::Searcher(int searched_obj, int curr_room, tf::TransformListener *tf_listener)
  : searched_obj_(searched_obj), tf_listener_(tf_listener)
{
  ros::NodeHandle private_nh("~");
  private_nh.param("ObjectMapper/OBJ_PRIOR_PROB", OBJ_PRIOR_PROB, OBJ_PRIOR_PROB);
  private_nh.param("ObjectMapper/OBJ_MIN_PROB", OBJ_MIN_PROB, OBJ_MIN_PROB);
  private_nh.param("ObjectMapper/OBJ_MAX_PROB", OBJ_MAX_PROB, OBJ_MAX_PROB);
  private_nh.param("RESOLUTION", RESOLUTION, RESOLUTION);
  private_nh.param("ObjectMapper/OBJ_DEFUALT_MAX_HEIGHT", OBJ_DEFUALT_MAX_HEIGHT, OBJ_DEFUALT_MAX_HEIGHT);
  private_nh.param("ObjectMapper/V_H", V_H, V_H);
  private_nh.param("ObjectMapper/V_M", V_M, V_M);
  private_nh.param("ObjectMapper/ROOM_EXPECTED_SIZE", ROOM_EXPECTED_SIZE, ROOM_EXPECTED_SIZE);
  private_nh.param("Octomap/pointcloud_min_z", POINTCLOUD_MIN_Z,POINTCLOUD_MIN_Z);
  private_nh.param("Octomap/pointcloud_max_z", POINTCLOUD_MAX_Z,POINTCLOUD_MAX_Z);

  map_sub_ = ros::NodeHandle().subscribe("map_door_blocked", 1, &Searcher::mapCb, this);
  vision_sub_ = ros::NodeHandle().subscribe("vision_result", 1, &Searcher::visionCb, this);
  octo_mapper_ = new OctoMapper(RESOLUTION);

  ros::ServiceClient service_client(ros::NodeHandle().serviceClient<semantic_mapping_v2::ObjectMapSrv>("obj_map_srv"));
  while(!service_client.waitForExistence(ros::Duration(0.1))){
    ROS_WARN("HIERARCHY SERVICE NOT EXISTING");
    ros::spinOnce();
  }
  semantic_mapping_v2::ObjectMapSrvRequest req;
  req.id = searched_obj_;
  req.room_id = curr_room;
  semantic_mapping_v2::ObjectMapSrvResponse res;
  if(!service_client.call(req, res)){
    ROS_WARN("TRY SEMANTIC MAP CALL FAILED");
    return;
  }
  prior_prob_map_ = new ObjectMap(res.maps[0]);
  obj_map_ = new ObjectMap(RESOLUTION, 4.0, prior_prob_map_->getMaxHeight(), OBJ_PRIOR_PROB);
  obj_map_->expandUntilFitting(prior_prob_map_->getMinX(), prior_prob_map_->getMaxX(), prior_prob_map_->getMinY(), prior_prob_map_->getMaxY(), OBJ_PRIOR_PROB);
  prior_prob_map_->resample(*obj_map_, OBJ_PRIOR_PROB);
  seen_maps_.resize(SEEN_MAP_STEPS);
  for(auto& map : seen_maps_)
    map = cv::Mat_<float>(obj_map_->getHeight(), obj_map_->getWidth(), 0.f);

  octomap_pub_ = ros::NodeHandle().advertise<visualization_msgs::MarkerArray>("searcher_occ", 1, true);
}


Searcher::~Searcher(){
  delete obj_map_;
  delete prior_prob_map_;
  delete octo_mapper_;
}


void Searcher::resize(float x1, float x2, float y1, float y2){
  std::vector<int> border = obj_map_->expandUntilFitting(x1, x2, y1, y2, OBJ_PRIOR_PROB);
  if(!border.empty()){
    prior_prob_map_->resize(border[2], border[3], border[0], border[1], OBJ_PRIOR_PROB);
    for(auto& m : seen_maps_){
      cv::copyMakeBorder(m, m, border[0], border[1], border[2], border[3], cv::BORDER_CONSTANT, 0.0);
    }
    cv::copyMakeBorder(accessible_map_, accessible_map_, border[0], border[1], border[2], border[3], cv::BORDER_CONSTANT, 0.0);
    cv::copyMakeBorder(border_map_, border_map_, border[0], border[1], border[2], border[3], cv::BORDER_CONSTANT, 0.0);
    cv::copyMakeBorder(border_dir_map_, border_dir_map_, border[0], border[1], border[2], border[3], cv::BORDER_CONSTANT, 0.0);
  }
}


void insertNeighbors(const cv::Point& p, cv::Mat_<uchar>& already_inserted, std::deque<cv::Point>& list);

void Searcher::mapCb(const nav_msgs::OccupancyGridConstPtr &msg){
  ros::Time t = ros::Time::now();
  cv::Mat_<uchar> free(msg->info.height, msg->info.width, uchar(0));
  cv::Mat_<uchar> occupied(msg->info.height, msg->info.width, uchar(0));
  for(int x=0; x<msg->info.width; x++){
    for(int y=0; y<msg->info.height; y++){
      if(msg->data[y * msg->info.width + x] == 0)
        free(y,x) = 255;
      else if(msg->data[y * msg->info.width + x] > 0)
        occupied(y,x) = 255;
    }
  }
  cv::Mat_<uchar> occupied_dilate, not_forbidden;
  int robot_kernel_size = ROBOT_SIZE*2+1;
  cv::dilate(occupied, occupied_dilate, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(robot_kernel_size,robot_kernel_size)));
  cv::bitwise_not(occupied_dilate, not_forbidden);
  cv::erode(free, free, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(robot_kernel_size,robot_kernel_size)));
  cv::dilate(free, free, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(robot_kernel_size,robot_kernel_size)));
  cv::bitwise_and(not_forbidden, free, not_forbidden);

  tf::StampedTransform transform;
  try{
    tf_listener_->lookupTransform("map", "base_link", ros::Time(0), transform);
  }
  catch (tf::TransformException ex){
    ROS_ERROR("%s",ex.what());
  }
  cv::Point pos(int((transform.getOrigin().x()-msg->info.origin.position.x)/msg->info.resolution),
                int((transform.getOrigin().y()-msg->info.origin.position.y)/msg->info.resolution));
  cv::Point start = getNearestFree(not_forbidden, pos.x, pos.y);

  cv::Mat_<float> accessible_mat = cv::Mat_<uchar>(msg->info.height, msg->info.width, uchar(0));
  std::deque<cv::Point> next[2];
  cv::Mat_<uchar> already_inserted;
  cv::bitwise_not(not_forbidden, already_inserted);
  already_inserted(start) = 255;
  int i=0;
  next[i].push_back(start);
  while(!next[i&1].empty()){
    for(const auto& p : next[i&1]){
      if(not_forbidden(p)){
        accessible_mat(p) = 1.f;
      }
      insertNeighbors(p, already_inserted, next[(i+1)&1]);
    }
    next[i&1].clear();
    i++;
  }

  resize(msg->info.origin.position.x, msg->info.width*msg->info.resolution+msg->info.origin.position.x,
         msg->info.origin.position.y, msg->info.height*msg->info.resolution+msg->info.origin.position.y);

  double factor = obj_map_->getResolution()*msg->info.resolution;
  cv::resize(accessible_mat, accessible_mat, cv::Size(accessible_mat.cols*factor, accessible_mat.rows*factor), 0, 0, cv::INTER_NEAREST);
  accessible_map_ = cv::Mat_<float>(obj_map_->getHeight(), obj_map_->getWidth(), 0.f);
  accessible_mat.copyTo(accessible_map_(cv::Rect(obj_map_->getXPixel(msg->info.origin.position.x), obj_map_->getYPixel(msg->info.origin.position.y), accessible_mat.cols, accessible_mat.rows)));

  cv::Mat_<uchar> tmp;
  accessible_map_.convertTo(tmp, CV_8UC1, -1, 1);
  cv::Mat dists, grad_x, grad_y, mag, dir;
  cv::distanceTransform(tmp, dists, cv::DIST_L1, 3, CV_32F);
  cv::Sobel(dists, grad_x, CV_32F, 1, 0);
  cv::Sobel(dists, grad_y, CV_32F, 0, 1);
  cv::cartToPolar(grad_x, grad_y, mag, border_dir_map_);
  border_map_ = (dists == 1.f);

  std::cout << "Map processed" << std::endl;
}


cv::Point Searcher::getNearestFree(const cv::Mat_<uchar>& valid, int x, int y) const{
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


void Searcher::visionCb(const vision::VisionMsgConstPtr &msg){
  ros::Time t = ros::Time::now();
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>(msg->objects.samples.size(), 1));
  for(int i=0; i<cloud->size(); i++){
    (*cloud)[i] = pcl::PointXYZ(msg->objects.samples[i].point.x, msg->objects.samples[i].point.y, msg->objects.samples[i].point.z);
  }
  tf::StampedTransform transform;
  try{
    tf_listener_->lookupTransform("map", "camera_rgb_optical_frame", msg->header.stamp, transform);
  }
  catch (tf::TransformException ex){
    ROS_ERROR("%s",ex.what());
  }
  Eigen::Matrix4f cam_to_base;
  pcl_ros::transformAsMatrix(transform, cam_to_base);
  pcl::transformPointCloud(*cloud, *cloud, cam_to_base);
  pcl::PointXYZ min, max;
  pcl::getMinMax3D(*cloud, min, max);
  resize(min.x, max.x, min.y, max.y);
  std::cout << "1 in " << (ros::Time::now()-t).toSec() << std::endl;

  insertObject(*cloud, msg->objects);
  std::cout << "2 in " << (ros::Time::now()-t).toSec() << std::endl;
  insertCloud(cloud, transform.getOrigin());
  std::cout << "3 in " << (ros::Time::now()-t).toSec() << std::endl;

  calcNextViewpoint(transform);
  std::cout << "4 in " << (ros::Time::now()-t).toSec() << std::endl;
  insertIntoSeenMaps(transform);

  std::cout << "VISION CALLBACK in " << (ros::Time::now()-t).toSec() << std::endl;
}


void Searcher::insertObject(const pcl::PointCloud<pcl::PointXYZ>& cloud, const vision::ObjectDetectionMsg& msg){
  float min_z = std::max(POINTCLOUD_MIN_Z, 0.f);
  float max_z = std::min(POINTCLOUD_MAX_Z, obj_map_->getMaxHeight()-0.001f);

  ObjectMap tmp(ObjectMap(obj_map_->getResolution(), obj_map_->getBaseSize(), obj_map_->getWidth(), obj_map_->getHeight(), obj_map_->getOrigin(), obj_map_->getMaxHeight(), -1.f));

  for(int i=0; i<cloud.size(); i++){
    if(cloud[i].z>=min_z && cloud[i].z<=max_z){
      int x = obj_map_->getXPixel(cloud[i].x);
      int y = obj_map_->getYPixel(cloud[i].y);
      int z = obj_map_->getZPixel(cloud[i].z);
      tmp.insertMax(x,y,z,OBJ_MIN_PROB);
      for(const auto& b : msg.samples[i].boxes){
        tmp.insertMax(x,y,z,msg.probs[b*msg.num_objects+searched_obj_]);
      }
    }
  }

  for(int x=0; x<tmp.getWidth(); x++){
    for(int y=0; y<tmp.getHeight(); y++){
      for(int z=0; z<tmp.getZSteps(); z++){
        if(tmp.getProb(x,y,z) >= 0.f){
          obj_map_->insertProb(x,y,z,tmp.getProb(x,y,z), OBJ_PRIOR_PROB, V_H, V_M, OBJ_MIN_PROB, OBJ_MAX_PROB);
        }
      }
    }
  }
  std::cout << "Obj inserted" << std::endl;
}


void Searcher::insertCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, const tf::Point& sensorOriginTf){
  pcl::PointCloud<pcl::PointXYZ> pc, pc_ground;
  pcl::PassThrough<OctoMapper::PCLPoint> pass_z;
  pass_z.setFilterFieldName("z");
  pass_z.setFilterLimits(POINTCLOUD_MIN_Z, POINTCLOUD_MAX_Z);
  pass_z.setInputCloud(cloud);
  pass_z.filter(pc);

  pass_z.setFilterFieldName("z");
  pass_z.setFilterLimits(-1.0, POINTCLOUD_MIN_Z);
  pass_z.setInputCloud(cloud);
  pass_z.filter(pc_ground);

  std::cout << pc.size() << " " << pc_ground.size() << std::endl;

  octo_mapper_->insertCloud(pc, pc_ground, sensorOriginTf);
  std::cout << "Cloud inserted" << std::endl;
}


bool Searcher::objFound(){
  float max_prob = 0.0;
  for(int x=0; x<obj_map_->getWidth(); x++){
    for(int y=0; y<obj_map_->getHeight(); y++){
      for(int z=0; z<obj_map_->getZSteps(); z++){
        geometry_msgs::Point p;
        p.x = obj_map_->getXWorld(x);
        p.y = obj_map_->getYWorld(y);
        p.z = obj_map_->getZWorld(z);
        max_prob = std::max(max_prob, obj_map_->getProb(x, y, z)*octo_mapper_->getOccupancy(p.x, p.y, p.z));
      }
    }
  }
  std::cout << "Max: " << max_prob << std::endl;
  //octomap_pub_.publish(octo_mapper_->getOccupiedCellMsg(ros::Time::now()));
  return max_prob > OBJECT_FOUND_THRESH;
}


cv::Mat_<float> Searcher::getProbMap(cv::Point& origin){
  ObjectMap count_map(1.f,1.f,1.f,0.f);
  ObjectMap occ_map(obj_map_->getResolution(), obj_map_->getBaseSize(), obj_map_->getWidth(), obj_map_->getHeight(),
                    obj_map_->getOrigin(), obj_map_->getMaxHeight(), *octo_mapper_, &count_map);

  return obj_map_->get2D(occ_map, *prior_prob_map_, count_map, SAMPLE_COUNT_THRESH, origin);
//  cv::Mat_<float> tmp = obj_map_->get2D(occ_map, *prior_prob_map_, count_map, 1000);
//  float factor = 1.f/(VIEW_POS_RESOLUTION*obj_map_->getResolution();
//  cv::resize(tmp, tmp, cv::Size(factor*obj_map_->getWidth(),factor*obj_map_->getHeight()));
}


cv::Mat_<float> Searcher::getViewKernel(float angle, float max_dist, float resolution) const{
  cv::Mat_<float> kernel(int(resolution*max_dist)*2+1, int(resolution*max_dist)*2+1, 0.f);
  std::vector<cv::Point> points;
  points.push_back(cv::Point(std::cos(angle-VIEW_ANGLE/2.f)*VIEW_MIN_DIST, std::sin(angle-VIEW_ANGLE/2.f)*VIEW_MIN_DIST));
  points.push_back(cv::Point(std::cos(angle-VIEW_ANGLE/2.f)*max_dist, std::sin(angle-VIEW_ANGLE/2.f)*max_dist));
  points.push_back(cv::Point(std::cos(angle+VIEW_ANGLE/2.f)*max_dist, std::sin(angle+VIEW_ANGLE/2.f)*max_dist));
  points.push_back(cv::Point(std::cos(angle+VIEW_ANGLE/2.f)*VIEW_MIN_DIST, std::sin(angle+VIEW_ANGLE/2.f)*VIEW_MIN_DIST));
  cv::fillConvexPoly(kernel, points, cv::Scalar(1.f));

  return kernel;
}


inline float angleDist(float angle1, float angle2){
  float angle = angle1-angle2;
  if(angle > M_PI)
    return std::abs(angle-2*M_PI);
  if(angle <= -M_PI)
    return std::abs(angle+2*M_PI);
  return std::abs(angle);
}


cv::Mat_<float> Searcher::calcMoveTime(int width, int height, int angle_step, const cv::Point& curr_pos, float curr_angle){
  float view_angle = angle_step/VIEW_ANGLE_STEPS*M_PI*2.f;
  cv::Mat_<float> move_times(height, width);
  for(int x=0; x<width; x++){
    for(int y=0; y<height; y++){
      cv::Point diff = cv::Point(x,y)-curr_pos;
      float move_angle = std::atan2(float(x),float(y));
      move_times(y,x) = 1.f/((angleDist(curr_angle,move_angle)+angleDist(move_angle,view_angle))*TURN_SPEED + diff.ddot(diff)*MOVE_SPEED + VIEW_TIME);
    }
  }
  return move_times;
}


inline cv::Point poseToPoint(const tf::Transform& pose, cv::Point origin, float resolution){
  return cv::Point(pose.getOrigin().x()*resolution + origin.x, pose.getOrigin().y()*resolution + origin.y);
}


void showProbImage(const std::string& name, const cv::Mat mat, float resize_factor){
  cv::Mat tmp;
  cv::resize(mat, tmp, cv::Size(mat.cols*resize_factor, mat.rows*resize_factor));
  double min, max;
  cv::minMaxIdx(tmp, &min, &max);
  cv::flip(tmp, tmp, 0);

  tmp.convertTo(tmp, CV_8U, 150.f);

  cv::Mat o(tmp.rows, tmp.cols, CV_8UC1, cv::Scalar(255));
  cv::merge(std::vector<cv::Mat>({tmp, o, o}), tmp);
  cv::cvtColor(tmp, tmp, CV_HSV2RGB);
  cv::imshow(name, tmp);
}


void Searcher::calcNextViewpoint(const tf::Transform& curr_pose){
  cv::Point new_origin;
  cv::Mat_<float> prob_map = getProbMap(new_origin);
  cv::resize(prob_map, prob_map, cv::Size(obj_map_->getWidth()/2, obj_map_->getHeight()/2));

  double max = 0.0;
  int max_i;
  cv::Point max_loc;
  for(int i=0; i<VIEW_ANGLE_STEPS; i++){
    cv::Mat_<float> view_probs;
    cv::Mat_<float> kernel = getViewKernel(i/VIEW_ANGLE_STEPS*M_PI*2.f, VIEW_MAX_DIST, RESOLUTION/2);
    cv::filter2D(prob_map, view_probs, -1, kernel, cv::Point(-1,-1), 0.0, cv::BORDER_REPLICATE);
    cv::resize(view_probs, view_probs, cv::Size(obj_map_->getWidth(), obj_map_->getHeight()));
    view_probs = view_probs.mul(accessible_map_.mul(calcMoveTime(view_probs.cols, view_probs.rows, i,
                     poseToPoint(curr_pose, obj_map_->getOrigin(), obj_map_->getResolution()), tf::getYaw(curr_pose.getRotation()))));

    double min, tmp_max;
    cv::Point tmp_max_loc;
    cv::minMaxLoc(view_probs, &min, &tmp_max, 0, &tmp_max_loc);
    if(tmp_max > max){
      max = tmp_max;
      max_i = i;
      max_loc = tmp_max_loc;
    }
  }

  if(max <= 0.0){
    finished_ = true;
    return;
  }
  showProbImage("prob_map", prob_map, 4);
  showProbImage("accessible_map_", accessible_map_, 2);
  showProbImage("border_dir", border_dir_map_, 2);
  cv::Mat tmp;
  cv::resize(border_map_, tmp, cv::Size(border_map_.cols*2, border_map_.rows*2), 0, 0, cv::INTER_NEAREST);
  cv::imshow("border_map", border_map_);
  cv::waitKey(1);

  tf::Transform curr_view(tf::createQuaternionFromYaw(max_i/VIEW_ANGLE_STEPS*M_PI*2.f),
                          tf::Vector3((max_loc.x-obj_map_->getOrigin().x)*RESOLUTION, (max_loc.y-obj_map_->getOrigin().y)*RESOLUTION, 0.0));
  tf::poseTFToMsg(curr_view, curr_view_pose_);
}


bool Searcher::insertIntoSeenMaps(const tf::Transform &curr_pose){
  double angle = tf::getYaw(curr_pose.getRotation());
  while(angle < 0)
    angle += 2*M_PI;
  while(angle >= 2*M_PI)
    angle -= 2*M_PI;
  int idx = angle/(2*M_PI)*SEEN_MAP_STEPS;

  cv::Point pos = poseToPoint(curr_pose, obj_map_->getOrigin(), RESOLUTION);
  cv::Mat_<float> kernel = getViewKernel(angle, SEEN_MAP_MAX_DIST, RESOLUTION);
  int x1=pos.x-kernel.cols, y1=pos.y-kernel.rows;
  cv::Mat(seen_maps_[idx](cv::Rect(x1,y1,kernel.cols,kernel.rows)) + kernel).copyTo(seen_maps_[idx](cv::Rect(x1,y1,kernel.cols,kernel.rows)));

  for(int x=0; x<border_dir_map_.cols; x++){
    for(int y=0; y<border_dir_map_.rows; y++){
      if(seen_maps_[border_dir_map_(y,x)/(2*M_PI)*SEEN_MAP_STEPS](y,x) < BORDER_SEEN_THRESH)
        return false;
    }
  }
  return true;
}
