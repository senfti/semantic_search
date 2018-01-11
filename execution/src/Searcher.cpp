//
// Created by thomas on 02.01.18.
//

#include <execution/Searcher.h>
#include <pcl/common/common.h>
#include <semantic_mapping_v2/ObjectMapSrv.h>
#include <tf/transform_datatypes.h>


void showProbImage(const std::string& name, const cv::Mat mat, float resize_factor){
  cv::Mat tmp;
  cv::resize(mat, tmp, cv::Size(mat.cols*resize_factor, mat.rows*resize_factor), 0, 0, cv::INTER_NEAREST);
  double min, max;
  cv::minMaxIdx(tmp, &min, &max);
  //std::cout << "minmax " << min << " " << max << std::endl;
  cv::flip(tmp, tmp, 0);

  tmp.convertTo(tmp, CV_8U, 150.f);

  cv::Mat o(tmp.rows, tmp.cols, CV_8UC1, cv::Scalar(255));
  cv::merge(std::vector<cv::Mat>({tmp, o, o}), tmp);
  cv::cvtColor(tmp, tmp, CV_HSV2BGR);
  cv::imshow(name, tmp);
  cv::waitKey(1);
}


Searcher::Searcher(tf::TransformListener *tf_listener)
  : tf_listener_(tf_listener), obj_map_service_client_(ros::NodeHandle().serviceClient<semantic_mapping_v2::ObjectMapSrv>("obj_map_srv"))
{
  ros::NodeHandle private_nh("~");
  private_nh.param("SEARCH_MAX_ROT_VEL", SEARCH_MAX_ROT_VEL, SEARCH_MAX_ROT_VEL);
  private_nh.param("SEARCH_MAX_TRANS_VEL", SEARCH_MAX_TRANS_VEL, SEARCH_MAX_TRANS_VEL);
  private_nh.param("ROBOT_SIZE", ROBOT_SIZE, ROBOT_SIZE);

  private_nh.param("OBJ_PRIOR_PROB", OBJ_PRIOR_PROB, OBJ_PRIOR_PROB);
  private_nh.param("OBJ_MIN_PROB", OBJ_MIN_PROB, OBJ_MIN_PROB);
  private_nh.param("OBJ_MAX_PROB", OBJ_MAX_PROB, OBJ_MAX_PROB);

  private_nh.param("RESOLUTION", RESOLUTION, RESOLUTION);
  private_nh.param("OBJ_DEFUALT_MAX_HEIGHT", OBJ_DEFUALT_MAX_HEIGHT, OBJ_DEFUALT_MAX_HEIGHT);
  private_nh.param("ROOM_EXPECTED_SIZE", ROOM_EXPECTED_SIZE, ROOM_EXPECTED_SIZE);

  private_nh.param("V_H", V_H, V_H);
  private_nh.param("V_M", V_M, V_M);
  private_nh.param("POINTCLOUD_MIN_Z", POINTCLOUD_MIN_Z, POINTCLOUD_MIN_Z);
  private_nh.param("POINTCLOUD_MIN_Z", POINTCLOUD_MIN_Z, POINTCLOUD_MIN_Z);

  private_nh.param("OBJECT_FOUND_THRESH", OBJECT_FOUND_THRESH, OBJECT_FOUND_THRESH);

  private_nh.param("VIEW_ANGLE_STEPS", VIEW_ANGLE_STEPS, VIEW_ANGLE_STEPS);
  private_nh.param("SEEN_MAP_STEPS", SEEN_MAP_STEPS, SEEN_MAP_STEPS);
  private_nh.param("SAMPLE_COUNT_THRESH", SAMPLE_COUNT_THRESH, SAMPLE_COUNT_THRESH);

  private_nh.param("VIEW_MIN_DIST", VIEW_MIN_DIST, VIEW_MIN_DIST);
  private_nh.param("VIEW_MAX_DIST", VIEW_MAX_DIST, VIEW_MAX_DIST);
  private_nh.param("VIEW_ANGLE", VIEW_ANGLE, VIEW_ANGLE);

  private_nh.param("TURN_SPEED", TURN_SPEED, TURN_SPEED);
  private_nh.param("MOVE_SPEED", MOVE_SPEED, MOVE_SPEED);
  private_nh.param("VIEW_TIME", VIEW_TIME, VIEW_TIME);

  private_nh.param("BORDER_SEEN_THRESH", BORDER_SEEN_THRESH, BORDER_SEEN_THRESH);
  private_nh.param("BORDER_SEEN_MAX_ANGLE", BORDER_SEEN_MAX_ANGLE, BORDER_SEEN_MAX_ANGLE);
  private_nh.param("SEEN_MAP_MAX_DIST", SEEN_MAP_MAX_DIST, SEEN_MAP_MAX_DIST);

  while(!obj_map_service_client_.waitForExistence(ros::Duration(0.1))){
    ROS_WARN("HIERARCHY SERVICE NOT EXISTING");
    ros::spinOnce();
  }

  map_sub_ = ros::NodeHandle().subscribe("map_door_blocked", 1, &Searcher::mapCb, this);
  vision_sub_ = ros::NodeHandle().subscribe("vision_result", 1, &Searcher::visionCb, this);

  octomap_pub_ = ros::NodeHandle().advertise<visualization_msgs::MarkerArray>("searcher_occ", 1, true);
  obj_pub_ = ros::NodeHandle().advertise<visualization_msgs::MarkerArray>("searcher_obj", 1, true);
  full_pub_ = ros::NodeHandle().advertise<visualization_msgs::MarkerArray>("searcher_full_obj", 1, true);
  next_pose_pub_ = ros::NodeHandle().advertise<geometry_msgs::PoseStamped>("searcher_pose", 1, true);

  calcSeenKernels();
}


Searcher::~Searcher(){
  delete obj_map_;
  delete prior_prob_map_;
  delete octo_mapper_;
}


geometry_msgs::Pose Searcher::getNextViewPose(){
  curr_view_changed_ = false;
  did_abort_ = false;
  return curr_view_pose_;
}

void Searcher::start(int searched_obj){
  if(running_)
    return;

  int searched_obj_ = searched_obj;
  running_ = true;
  finished_ = false;
  obj_found_ = false;
  have_curr_view_ = false;
  curr_view_changed_= true;
  got_map_ = false;
//  std::system(("rosrun dynamic_reconfigure dynparam set /move_base/DWAPlannerROS max_rot_vel " + std::to_string(SEARCH_MAX_ROT_VEL)).c_str());
//  std::system(("rosrun dynamic_reconfigure dynparam set /move_base/DWAPlannerROS max_trans_vel " + std::to_string(SEARCH_MAX_TRANS_VEL)).c_str());
  octo_mapper_ = new OctoMapper(RESOLUTION);

  semantic_mapping_v2::ObjectMapSrvRequest req;
  req.id = searched_obj_;
  req.room_id = -1;
  semantic_mapping_v2::ObjectMapSrvResponse res;
  if(!obj_map_service_client_.call(req, res)){
    ROS_WARN("SEMANTIC MAP CALL FAILED");
    return;
  }
  prior_prob_map_ = new ObjectMap(res.maps[0]);
  obj_map_ = new ObjectMap(RESOLUTION, 4.0, prior_prob_map_->getMaxHeight(), OBJ_PRIOR_PROB);
  obj_map_->expandUntilFitting(prior_prob_map_->getMinX(), prior_prob_map_->getMaxX(), prior_prob_map_->getMinY(), prior_prob_map_->getMaxY(), OBJ_PRIOR_PROB);
  prior_prob_map_->resample(*obj_map_, OBJ_PRIOR_PROB);
  seen_maps_.resize(SEEN_MAP_STEPS);
  for(auto& map : seen_maps_)
    map = cv::Mat_<float>(obj_map_->getHeight(), obj_map_->getWidth(), 0.f);
  ROS_INFO("SEARCH STARTED");
}

void Searcher::stop(){
  if(running_)
    ROS_INFO("SEARCH STOPPED");
  running_ = false;
  finished_ = false;
  have_curr_view_ = false;

  seen_maps_.clear();
  accessible_map_ = cv::Mat_<uchar>();
  border_map_ = cv::Mat_<uchar>();
  border_dir_map_ = cv::Mat_<float>();
  not_fully_viewed_border_ = cv::Mat_<uchar>();

  delete obj_map_;
  delete prior_prob_map_;
  delete octo_mapper_;
}


void Searcher::calcSeenKernels(){
  seen_kernel_points_.resize(VIEW_ANGLE_STEPS);
  seen_kernel_points_value_.resize(VIEW_ANGLE_STEPS);
  for(int i=0;i<VIEW_ANGLE_STEPS; i++){
    float angle = float(i)/VIEW_ANGLE_STEPS*M_PI*2;
    cv::Mat_<uchar> kernel(int(RESOLUTION*VIEW_MAX_DIST)*2+1, int(RESOLUTION*VIEW_MAX_DIST)*2+1, 0.f);
    cv::Point center(RESOLUTION*VIEW_MAX_DIST,RESOLUTION*VIEW_MAX_DIST);
    std::vector<cv::Point> points;
    points.push_back(cv::Point(std::cos(angle-VIEW_ANGLE/2.f)*VIEW_MIN_DIST*RESOLUTION, std::sin(angle-VIEW_ANGLE/2.f)*VIEW_MIN_DIST*RESOLUTION)+center);
    points.push_back(cv::Point(std::cos(angle-VIEW_ANGLE/2.f)*VIEW_MAX_DIST*RESOLUTION, std::sin(angle-VIEW_ANGLE/2.f)*VIEW_MAX_DIST*RESOLUTION)+center);
    points.push_back(cv::Point(std::cos(angle+VIEW_ANGLE/2.f)*VIEW_MAX_DIST*RESOLUTION, std::sin(angle+VIEW_ANGLE/2.f)*VIEW_MAX_DIST*RESOLUTION)+center);
    points.push_back(cv::Point(std::cos(angle+VIEW_ANGLE/2.f)*VIEW_MIN_DIST*RESOLUTION, std::sin(angle+VIEW_ANGLE/2.f)*VIEW_MIN_DIST*RESOLUTION)+center);
    cv::fillConvexPoly(kernel, points, cv::Scalar(255));
    for(int x=0;x<kernel.cols; x++){
      for(int y=0; y<kernel.rows; y++){
        if(kernel(y,x)){
          cv::Point diff = cv::Point(x,y)-center;
          seen_kernel_points_[i].push_back(diff);
          seen_kernel_points_value_[i].push_back(1.f-std::abs(std::sqrt(float(diff.x)*diff.x+diff.y*diff.y)/(RESOLUTION) - 2.f)/4.f);
        }
      }
    }
  }
}


void Searcher::resize(float x1, float x2, float y1, float y2){
  std::vector<int> border = obj_map_->expandUntilFitting(x1, x2, y1, y2, OBJ_PRIOR_PROB);
  if(!border.empty()){
    prior_prob_map_->resize(border[2], border[3], border[0], border[1], 0.f);
    for(auto& m : seen_maps_){
      cv::copyMakeBorder(m, m, border[0], border[1], border[2], border[3], cv::BORDER_CONSTANT, 0.0);
    }
    cv::copyMakeBorder(accessible_map_, accessible_map_, border[0], border[1], border[2], border[3], cv::BORDER_CONSTANT, 0.0);
    cv::copyMakeBorder(border_map_, border_map_, border[0], border[1], border[2], border[3], cv::BORDER_CONSTANT, 0.0);
    cv::copyMakeBorder(border_dir_map_, border_dir_map_, border[0], border[1], border[2], border[3], cv::BORDER_CONSTANT, 0.0);
    cv::copyMakeBorder(not_fully_viewed_border_, not_fully_viewed_border_, border[0], border[1], border[2], border[3], cv::BORDER_CONSTANT, 0.0);
  }
}


void insertNeighbors(const cv::Point& p, cv::Mat_<uchar>& already_inserted, std::deque<cv::Point>& list);

void Searcher::mapCb(const nav_msgs::OccupancyGridConstPtr &msg){
  if(!running_)
    return;

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

  cv::Mat_<uchar> accessible_mat = cv::Mat_<uchar>(msg->info.height, msg->info.width, uchar(0));
  std::deque<cv::Point> next[2];
  cv::Mat_<uchar> already_inserted;
  cv::bitwise_not(not_forbidden, already_inserted);
  already_inserted(start) = 255;
  int i=0;
  next[i].push_back(start);
  while(!next[i&1].empty()){
    for(const auto& p : next[i&1]){
      if(not_forbidden(p)){
        accessible_mat(p) = 255;
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
  accessible_map_ = cv::Mat_<uchar>(obj_map_->getHeight(), obj_map_->getWidth(), 0.f);
  accessible_mat.copyTo(accessible_map_(cv::Rect(obj_map_->getXPixel(msg->info.origin.position.x), obj_map_->getYPixel(msg->info.origin.position.y), accessible_mat.cols, accessible_mat.rows)));

  cv::Mat dists, grad_x, grad_y, mag, dir;
  cv::distanceTransform(255-accessible_map_, dists, cv::DIST_L1, 3, CV_32F);
  cv::Sobel(dists, grad_x, CV_32F, 1, 0, 5);
  cv::Sobel(dists, grad_y, CV_32F, 0, 1, 5);
  cv::cartToPolar(grad_x, grad_y, mag, border_dir_map_);

  cv::Mat_<uchar> tmp, tmp2;
  cv::dilate(accessible_map_, tmp, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(robot_kernel_size,robot_kernel_size)));
  cv::dilate(tmp, tmp2, cv::Mat_<uchar>::ones(3,3));
  border_map_ = tmp2-tmp;

  got_map_ = true;
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
  if(!got_map_ || !running_)
    return;

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

  insertObject(*cloud, msg->objects);
  insertCloud(cloud, transform.getOrigin());

  if(objFound()){
    finished_ = true;
    std::cout << "OBJECT FOUND" << std::endl;
  }

  try{
    tf_listener_->lookupTransform("map", "base_link", msg->header.stamp, transform);
  }
  catch (tf::TransformException ex){
    ROS_ERROR("%s",ex.what());
  }
  if(insertIntoSeenMaps(transform)){
    finished_ = true;
    std::cout << "FINISHED, ALL BORDER SEEN" << std::endl;
  }
  if(calcNextViewpoint(transform)){
    finished_ = true;
    std::cout << "FINISHED, NO POSSIBLE POSITIONS LEFT" << std::endl;
  }

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
  obj_pub_.publish(obj_map_->getProbMsg());
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
  ObjectMap multiplied = *obj_map_*ObjectMap(obj_map_->getResolution(), obj_map_->getBaseSize(), obj_map_->getWidth(), obj_map_->getHeight(),
                                                     obj_map_->getOrigin(), obj_map_->getMaxHeight(), *octo_mapper_);
  float max_prob = 0.f;
  for(int x=0; x<obj_map_->getWidth(); x++){
    for(int y=0; y<obj_map_->getHeight(); y++){
      for(int z=0; z<obj_map_->getZSteps(); z++){
        if(obj_map_->getCount(x,y,z) > 0){
          float prob = 1.f;
          for(int xk=std::max(0,x-1); xk<std::min(x+2, obj_map_->getWidth()); xk++){
            for(int yk=std::max(0,y-1); yk<std::min(y+2, obj_map_->getHeight()); yk++){
              for(int zk=std::max(0,z-1); zk<std::min(z+2, obj_map_->getZSteps()); zk++){
                prob *= (1.f-multiplied.getProb(xk,yk,zk));
              }
            }
          }
          max_prob = std::max(max_prob, 1.f-prob);
        }
      }
    }
  }
  octomap_pub_.publish(octo_mapper_->getOccupiedCellMsg(ros::Time::now()));
  if(max_prob > OBJECT_FOUND_THRESH)
    obj_found_ = true;
  return obj_found_;
}


cv::Mat_<float> Searcher::getProbMap(cv::Point& origin){
  ObjectMap count_map(1.f,1.f,1.f,0.f);
  ObjectMap occ_map(obj_map_->getResolution(), obj_map_->getBaseSize(), obj_map_->getWidth(), obj_map_->getHeight(),
                    obj_map_->getOrigin(), obj_map_->getMaxHeight(), *octo_mapper_, &count_map);

  full_pub_.publish((*obj_map_*occ_map).getProbMsg());

  return obj_map_->get2D(occ_map, *prior_prob_map_, count_map, SAMPLE_COUNT_THRESH, origin);
//  cv::Mat_<float> tmp = obj_map_->get2D(occ_map, *prior_prob_map_, count_map, 1000);
//  float factor = 1.f/(VIEW_POS_RESOLUTION*obj_map_->getResolution();
//  cv::resize(tmp, tmp, cv::Size(factor*obj_map_->getWidth(),factor*obj_map_->getHeight()));
}


cv::Mat_<uchar> Searcher::getViewKernel(float angle, float max_dist, float resolution) const{
  cv::Mat_<float> kernel(int(resolution*max_dist)*2+1, int(resolution*max_dist)*2+1, 0.f);
  cv::Point center(resolution*max_dist,resolution*max_dist);
  std::vector<cv::Point> points;
  points.push_back(cv::Point(std::cos(angle-VIEW_ANGLE/2.f)*VIEW_MIN_DIST*resolution, std::sin(angle-VIEW_ANGLE/2.f)*VIEW_MIN_DIST*resolution)+center);
  points.push_back(cv::Point(std::cos(angle-VIEW_ANGLE/2.f)*max_dist*resolution, std::sin(angle-VIEW_ANGLE/2.f)*max_dist*resolution)+center);
  points.push_back(cv::Point(std::cos(angle+VIEW_ANGLE/2.f)*max_dist*resolution, std::sin(angle+VIEW_ANGLE/2.f)*max_dist*resolution)+center);
  points.push_back(cv::Point(std::cos(angle+VIEW_ANGLE/2.f)*VIEW_MIN_DIST*resolution, std::sin(angle+VIEW_ANGLE/2.f)*VIEW_MIN_DIST*resolution)+center);
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


float Searcher::calcMoveTime(const cv::Point& pos, float angle, const cv::Point& curr_pos, float curr_angle){
  cv::Point diff = pos-curr_pos;
  float move_angle = std::atan2(float(diff.y),float(diff.x));
  return (angleDist(curr_angle,move_angle)+angleDist(move_angle,angle))/TURN_SPEED + std::sqrt(float(diff.x)*diff.x+diff.y*diff.y)/MOVE_SPEED + VIEW_TIME;
}


inline cv::Point poseToPoint(const tf::Transform& pose, cv::Point origin, float resolution){
  return cv::Point(pose.getOrigin().x()*resolution + origin.x, pose.getOrigin().y()*resolution + origin.y);
}


float Searcher::calcViewpointGain(const cv::Point& pos, int angle_step, const cv::Mat_<float> &prob_map, const cv::Point& curr_pos, float curr_angle){
  float angle = float(angle_step)/VIEW_ANGLE_STEPS*M_PI*2;
  float prob = 1.0;
  bool interesting_border_seen = false;
  int idx = -1;
  for(const auto& p : seen_kernel_points_[angle_step]){
    idx++;
    if(!(pos+p).inside(cv::Rect(0,0,prob_map.cols,prob_map.rows)))
      continue;
    if(not_fully_viewed_border_(pos+p) > 0 && angleDist(angle, border_dir_map_(pos+p)) < BORDER_SEEN_MAX_ANGLE)
      interesting_border_seen = true;

    prob *= (1.f-prob_map(pos+p)*seen_kernel_points_value_[angle_step][idx]);
  }
  if(!interesting_border_seen)
    return 0.f;

  return (1.f-prob)/calcMoveTime(pos, angle, curr_pos, curr_angle);
}


bool Searcher::calcNextViewpoint(const tf::Transform& curr_pose){
  cv::Point new_origin;
  cv::Mat_<float> prob_map = getProbMap(new_origin);

  cv::Mat_<float> tmp;
  prob_map.copyTo(tmp);
  cv::pow(tmp, 0.25, tmp);
//  for(int x=0; x<prob_map.cols; x++){
//    for(int y=0; y<prob_map.rows; y++){
//      if(border_map_(y,x))
//        tmp(y,x) = 1.f;
//    }
//  }
  showProbImage("probabilities", tmp, 2);

  double max = 0.0;
  int max_i;
  cv::Point max_loc;
  for(int i=0; i<VIEW_ANGLE_STEPS; i++){
    cv::Mat_<float> sdf(prob_map.rows, prob_map.rows, 0.f);
    for(int x=0; x<prob_map.cols; x++){
      for(int y=0; y<prob_map.rows; y++){
        if(accessible_map_(y,x)){
          float prob = calcViewpointGain(cv::Point(x,y), i, prob_map, poseToPoint(curr_pose, obj_map_->getOrigin(), obj_map_->getResolution()), tf::getYaw(curr_pose.getRotation()));
          sdf(y,x) = prob;//*calcMoveTime(cv::Point(x,y), float(i)/VIEW_ANGLE_STEPS*M_PI*2, poseToPoint(curr_pose, obj_map_->getOrigin(), obj_map_->getResolution()), tf::getYaw(curr_pose.getRotation()));
          if(prob > max){
            max = prob;
            max_loc = cv::Point(x,y);
            max_i = i;
          }
        }
        if(border_map_(y,x))
          sdf(y,x) = 1.f;
      }
    }
    showProbImage(std::to_string(i), sdf, 2);
  }

  if(max <= 0.0){
    return true;
  }
//  showProbImage("prob_map", prob_map, 4);
//  showProbImage("accessible_map_", accessible_map_, 2);
//  showProbImage("border_dir", border_dir_map_, 2);

  tf::Transform curr_view(tf::createQuaternionFromYaw(float(max_i)/VIEW_ANGLE_STEPS*M_PI*2.f),
                          tf::Vector3((max_loc.x-obj_map_->getOrigin().x)/RESOLUTION, (max_loc.y-obj_map_->getOrigin().y)/RESOLUTION, 0.0));
  old_view_pose_ = curr_view_pose_;
  tf::poseTFToMsg(curr_view, curr_view_pose_);
  if(old_view_pose_.position.x != curr_view_pose_.position.x || old_view_pose_.position.y != curr_view_pose_.position.y || old_view_pose_.orientation.w != curr_view_pose_.orientation.w){
    curr_view_changed_ = true;
  }

  geometry_msgs::PoseStamped pose;
  pose.header.stamp = ros::Time::now();
  pose.pose = curr_view_pose_;
  pose.header.frame_id = "map";
  next_pose_pub_.publish(pose);
  have_curr_view_ = true;

  return false;
}


bool Searcher::insertIntoSeenMaps(const tf::Transform &curr_pose){
  double angle = tf::getYaw(curr_pose.getRotation());
  while(angle < 0)
    angle += 2*M_PI;
  while(angle >= 2*M_PI)
    angle -= 2*M_PI;
  int idx = angle/(2*M_PI)*SEEN_MAP_STEPS;
  int other_idx = ((angle/(2*M_PI)*SEEN_MAP_STEPS - idx) < 0.5 ? (idx+SEEN_MAP_STEPS-1)%SEEN_MAP_STEPS : (idx+1)%SEEN_MAP_STEPS);

  cv::Point pos = poseToPoint(curr_pose, obj_map_->getOrigin(), RESOLUTION);
  cv::Mat_<uchar> kernel = getViewKernel(angle, SEEN_MAP_MAX_DIST, RESOLUTION);
  int x1=pos.x-kernel.cols/2, y1=pos.y-kernel.rows/2;
  cv::Mat(seen_maps_[idx](cv::Rect(x1,y1,kernel.cols,kernel.rows)) + kernel).copyTo(seen_maps_[idx](cv::Rect(x1,y1,kernel.cols,kernel.rows)));
  cv::Mat(seen_maps_[other_idx](cv::Rect(x1,y1,kernel.cols,kernel.rows)) + kernel).copyTo(seen_maps_[other_idx](cv::Rect(x1,y1,kernel.cols,kernel.rows)));
  cv::threshold(seen_maps_[idx], seen_maps_[idx], 100, 100, cv::THRESH_TRUNC);
  cv::threshold(seen_maps_[other_idx], seen_maps_[other_idx], 100, 100, cv::THRESH_TRUNC);

  border_map_.copyTo(not_fully_viewed_border_);
  bool finished = true;
  for(int x=0; x<border_dir_map_.cols; x++){
    for(int y=0; y<border_dir_map_.rows; y++){
      if(border_map_(y,x) > 0){
        int border_idx = border_dir_map_(y,x)/(2*M_PI)*SEEN_MAP_STEPS;
        if(seen_maps_[border_idx](y,x)+seen_maps_[(border_idx+1)%SEEN_MAP_STEPS](y,x)+seen_maps_[(border_idx+SEEN_MAP_STEPS-1)%SEEN_MAP_STEPS](y,x) >= BORDER_SEEN_THRESH)
          not_fully_viewed_border_(y,x) = 0;
        else
          finished = false;
      }
    }
  }

  cv::Mat tmp;
  cv::threshold(border_map_, tmp, 64, 64, cv::THRESH_TRUNC);
  cv::bitwise_or(tmp, not_fully_viewed_border_, tmp);
  cv::resize(tmp, tmp, cv::Size(not_fully_viewed_border_.cols*2, not_fully_viewed_border_.rows*2), 0, 0, cv::INTER_NEAREST);
  cv::flip(tmp, tmp, 0);
  cv::imshow("not_fully_viewed_border_", tmp);
  cv::waitKey(1);

  return finished;
}
