//
// Created by thomas on 02.01.18.
//

#include <execution_test/Searcher.h>
#include <pcl/common/common.h>
#include <semantic_mapping_v2/ObjectMapSrv.h>
#include <tf/transform_datatypes.h>


inline float angleDist(float angle1, float angle2){
  float angle = angle1-angle2;
  while(angle > M_PI)
    angle-=2*M_PI;
  while(angle <= -M_PI)
    angle+=2*M_PI;
  return std::abs(angle);
}


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
  cv::moveWindow(name, 1000,0);
  cv::waitKey(1);
}

void showProbImage(const std::string& name, const cv::Mat mat, float resize_factor, const cv::Mat& mask){
  cv::Mat tmp;
  cv::resize(mat, tmp, cv::Size(mat.cols*resize_factor, mat.rows*resize_factor), 0, 0, cv::INTER_NEAREST);
  double min, max;
  cv::minMaxIdx(tmp, &min, &max);
  //std::cout << "minmax " << min << " " << max << std::endl;
  cv::flip(tmp, tmp, 0);
  cv::Mat mask_t;
  cv::flip(mask, mask_t, 0);
  cv::resize(mask_t, mask_t, cv::Size(mat.cols*resize_factor, mat.rows*resize_factor), 0, 0, cv::INTER_NEAREST);

  tmp.convertTo(tmp, CV_8U, 150.f);

  cv::Mat o(tmp.rows, tmp.cols, CV_8UC1, cv::Scalar(255));
  cv::merge(std::vector<cv::Mat>({tmp, o, mask_t}), tmp);
  cv::cvtColor(tmp, tmp, CV_HSV2BGR);
  cv::imshow(name, tmp);
  cv::moveWindow(name, 1000,0);
}

std::vector<std::string> obj_name = {"person",  "bicycle",  "car",  "motorbike",  "aeroplane",  "bus",  "train",  "truck",  "boat",  "trafficlight",
                                     "firehydrant",  "stopsign",  "parkmeter",  "bench",  "bird",  "cat",  "dog",  "horse",  "sheep",  "cow",  "elephant",
                                     "bear",  "zebra",  "giraffe",  "backpack",  "umbrella",  "handbag",  "tie",  "suitcase",  "frisbee",  "ski",
                                     "snowboard",  "sportball",  "kite",  "baseballbat",  "glove",  "skateboard",  "surfboard",  "racket",
                                     "bottle",  "wineglass",  "cup",  "fork",  "knife",  "spoon",  "bowl",  "banana",  "apple",  "sandwich",  "orange",
                                     "broccoli",  "carrot",  "hotdog",  "pizza",  "donut",  "cake",  "chair",  "sofa",  "potplant",  "bed",  "table",
                                     "toilet",  "monitor",  "laptop",  "mouse",  "remote",  "keyboard",  "cellphone",  "microwave",  "oven",  "toaster",
                                     "sink",  "refrigerator",  "book",  "clock",  "vase",  "scissor",  "teddybear",  "hairdryer",  "toothbrush"};

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
  private_nh.param("INTERESTING_BORDER_SEEN_REWARD", INTERESTING_BORDER_SEEN_REWARD, INTERESTING_BORDER_SEEN_REWARD);

  private_nh.param("QUICK_SEARCH_VIEWS", QUICK_SEARCH_VIEWS, QUICK_SEARCH_VIEWS);
  private_nh.param("QUICK_SEARCH_MIN_DIST", QUICK_SEARCH_MIN_DIST, QUICK_SEARCH_MIN_DIST);
  private_nh.param("QUICK_SEARCH_MAX_DIST", QUICK_SEARCH_MAX_DIST, QUICK_SEARCH_MAX_DIST);
  private_nh.param("QUICK_SEARCH_ANGLE_AREA", QUICK_SEARCH_ANGLE_AREA, QUICK_SEARCH_ANGLE_AREA);

  while(!obj_map_service_client_.waitForExistence(ros::Duration(0.1))){
    ROS_WARN("HIERARCHY SERVICE NOT EXISTING");
    ros::spinOnce();
  }

  map_sub_ = ros::NodeHandle().subscribe("map_door_blocked", 1, &Searcher::mapCb, this);
  vision_sub_ = ros::NodeHandle().subscribe("vision_result", 1, &Searcher::visionCb, this);
  search_query_sub_ = ros::NodeHandle().subscribe("search_query", 1, &Searcher::searchQueryCb, this);

  octomap_pub_ = ros::NodeHandle().advertise<visualization_msgs::MarkerArray>("searcher_occ", 1, true);
  for(int i=0; i<obj_name.size(); i++){
    obj_pub_.push_back(ros::NodeHandle().advertise<visualization_msgs::MarkerArray>("searcher_obj_"+obj_name[i], 1, true));
    full_pub_.push_back(ros::NodeHandle().advertise<visualization_msgs::MarkerArray>("searcher_full_obj_"+obj_name[i], 1, true));
  }
  obj_map_.resize(80, nullptr);
  prior_prob_map_.resize(80, nullptr);
  next_pose_pub_ = ros::NodeHandle().advertise<geometry_msgs::PoseStamped>("searcher_pose", 1, true);
  obj_found_pub_ = ros::NodeHandle().advertise<geometry_msgs::PoseStamped>("object_found_pose", 1);
  count_pub_ = ros::NodeHandle().advertise<visualization_msgs::MarkerArray>("count_occ", 1, true);
  prior_pub_ = ros::NodeHandle().advertise<visualization_msgs::MarkerArray>("searcher_prior_map", 1, true);

  calcSeenKernels();
}


Searcher::~Searcher(){
//  delete obj_map_;
//  delete prior_prob_map_;
//  delete octo_mapper_;
}


geometry_msgs::Pose Searcher::getNextViewPose(){
  curr_view_changed_ = false;
  did_abort_ = false;
  return curr_view_pose_;
}

void Searcher::start(int searched_obj, bool quick_search, geometry_msgs::Pose& target_pose){
  if(running_)
    return;

  searched_obj_ = searched_obj;
  running_ = true;
  finished_ = false;
  obj_found_ = false;
  have_curr_view_ = false;
  curr_view_changed_= true;
  got_map_ = false;
  got_vision_ = false;
  is_quick_search_ = quick_search;
  quick_search_step_ = 0;
  quick_search_step_viewed_ = true;
  quick_search_target_ = target_pose;
  
  octo_mapper_ = new OctoMapper(RESOLUTION);

  semantic_mapping_v2::ObjectMapSrvRequest req;
  req.id = -1;
  req.room_id = -1;
  semantic_mapping_v2::ObjectMapSrvResponse res;
  if(!obj_map_service_client_.call(req, res)){
    ROS_WARN("SEMANTIC MAP CALL FAILED");
    return;
  }
  for(int i=0; i<prior_prob_map_.size(); i++){
    prior_prob_map_[i] = new ObjectMap(res.maps[i]);
    obj_map_[i] = new ObjectMap(RESOLUTION, 4.0, prior_prob_map_[i]->getMaxHeight(), OBJ_PRIOR_PROB);
    obj_map_[i]->expandUntilFitting(prior_prob_map_[i]->getMinX(), prior_prob_map_[i]->getMaxX(), prior_prob_map_[i]->getMinY(), prior_prob_map_[i]->getMaxY(), OBJ_PRIOR_PROB);
    prior_prob_map_[i]->resample(*obj_map_[i], 0.0);
  }
  seen_maps_.resize(SEEN_MAP_STEPS);
  previous_pose_maps_.resize(VIEW_ANGLE_STEPS);
  for(auto &map : seen_maps_)
    map = cv::Mat_<float>(obj_map_[0]->getHeight(), obj_map_[0]->getWidth(), 0.f);
  for(auto &map : previous_pose_maps_)
    map = cv::Mat_<float>(obj_map_[0]->getHeight(), obj_map_[0]->getWidth(), 0.f);

  tf::StampedTransform transform;
  try{
    tf_listener_->lookupTransform("map", "base_link", ros::Time(0), transform);
  }
  catch (tf::TransformException ex){
    ROS_ERROR("%s",ex.what());
  }
  transform.setRotation(tf::createQuaternionFromYaw(tf::getYaw(transform.getRotation()) + M_PI/6.0));
  tf::poseTFToMsg(transform, curr_view_pose_);

  ROS_INFO("SEARCH STARTED");
}

void Searcher::stop(){
  if(running_)
    ROS_INFO("SEARCH STOPPED");
  running_ = false;
  finished_ = false;
  have_curr_view_ = false;

  seen_maps_.clear();
  previous_pose_maps_.clear();
  accessible_map_ = cv::Mat_<uchar>();
  border_map_ = cv::Mat_<uchar>();
  border_dir_map_ = cv::Mat_<float>();
  not_fully_viewed_border_ = cv::Mat_<uchar>();

//  delete obj_map_;
//  delete prior_prob_map_;
//  delete octo_mapper_;

//  obj_map_ = nullptr;
//  prior_prob_map_ = nullptr;
//  octo_mapper_ = nullptr;
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
          float va = std::min(1.5f-2.f*angleDist(std::atan2(float(diff.y),float(diff.x)), angle), 1.f);
          float vr = std::min(1.5f-std::abs(std::sqrt(float(diff.x)*diff.x+diff.y*diff.y)/(RESOLUTION)-2.f)/1.5f, 1.f);
          seen_kernel_points_value_[i].push_back(va*vr);
        }
      }
    }
  }
}


void Searcher::resize(float x1, float x2, float y1, float y2){
  std::vector<int> border;
  for(int i=0; i<obj_map_.size(); i++){
    border = obj_map_[i]->expandUntilFitting(x1, x2, y1, y2, OBJ_PRIOR_PROB);
    if(!border.empty()){
      prior_prob_map_[i]->resize(border[2], border[3], border[0], border[1], 0.f);
    }
  }
  if(!border.empty()){
    for(auto& m : seen_maps_){
      cv::copyMakeBorder(m, m, border[0], border[1], border[2], border[3], cv::BORDER_CONSTANT, 0.0);
    }
    for(auto& m : previous_pose_maps_){
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

  resize(msg->info.origin.position.x-0.2, msg->info.width*msg->info.resolution+msg->info.origin.position.x+0.4,
         msg->info.origin.position.y-0.2, msg->info.height*msg->info.resolution+msg->info.origin.position.y+0.4);


  cv::Mat dists, nearest, accessible_dil;
  cv::dilate(accessible_mat, accessible_dil, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(robot_kernel_size-4,robot_kernel_size-4)));
  cv::distanceTransform(255-accessible_dil, dists, nearest, CV_DIST_L2, CV_DIST_MASK_PRECISE, cv::DIST_LABEL_PIXEL);
  cv::Mat grad_x, grad_y, mag, dir;

  double factor = obj_map_[0]->getResolution()*msg->info.resolution;
  cv::resize(accessible_mat, accessible_mat, cv::Size(accessible_mat.cols*factor, accessible_mat.rows*factor));
  cv::threshold(accessible_mat, accessible_mat, 192, 255, cv::THRESH_BINARY);
  accessible_map_ = cv::Mat_<uchar>(obj_map_[0]->getHeight(), obj_map_[0]->getWidth(), 0.f);
  accessible_mat.copyTo(accessible_map_(cv::Rect(obj_map_[0]->getXPixel(msg->info.origin.position.x), obj_map_[0]->getYPixel(msg->info.origin.position.y), accessible_mat.cols, accessible_mat.rows)));
  cv::erode(accessible_map_, good_accessible_map_, cv::Mat_<uchar>::ones(3,3), cv::Point(-1,-1), 2);

  cv::Sobel(dists, grad_x, CV_32F, 1, 0, 5);
  cv::Sobel(dists, grad_y, CV_32F, 0, 1, 5);
  cv::resize(grad_x, grad_x, cv::Size(accessible_mat.cols, accessible_mat.rows));
  cv::resize(grad_y, grad_y, cv::Size(accessible_mat.cols, accessible_mat.rows));
  cv::cartToPolar(grad_x, grad_y, mag, dir);

  border_dir_map_ = cv::Mat_<uchar>(obj_map_[0]->getHeight(), obj_map_[0]->getWidth(), 0.f);
  dir.copyTo(border_dir_map_(cv::Rect(obj_map_[0]->getXPixel(msg->info.origin.position.x), obj_map_[0]->getYPixel(msg->info.origin.position.y), dir.cols, dir.rows)));

  cv::Mat_<uchar> tmp, tmp2;
  cv::dilate(accessible_map_, tmp, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(std::ceil(ROBOT_SIZE*factor)*2+1,std::ceil(ROBOT_SIZE*factor)*2+1)));
  cv::dilate(tmp, tmp2, cv::Mat_<uchar>::ones(3,3));
  border_map_ = tmp2-tmp;

  cv::Mat_<float> bla = border_dir_map_/(2.0*M_PI);
  showProbImage("border_dir_map", bla, 4, 255-accessible_map_*0.3);
  cv::flip(dists/20.0, mag, 0);
  cv::resize(mag, mag, cv::Size(mag.cols, mag.rows), 0, 0, cv::INTER_NEAREST);
  cv::imshow("dists", mag);
  cv::flip(accessible_map_, mag, 0);
  cv::resize(mag, mag, cv::Size(mag.cols*4, mag.rows*4), 0, 0, cv::INTER_NEAREST);
  cv::imshow("accessible_map_", mag);
  cv::waitKey(1);

  got_map_ = true;
  //std::cout << "Map processed" << std::endl;
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
  if(!running_)
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

  ObjectMap occ_map(obj_map_[0]->getResolution(), obj_map_[0]->getBaseSize(), obj_map_[0]->getWidth(), obj_map_[0]->getHeight(),
                    obj_map_[0]->getOrigin(), obj_map_[0]->getMaxHeight(), *octo_mapper_);
  for(int i=0; i<obj_map_.size(); i++){
    full_pub_[i].publish((*obj_map_[i]*occ_map).getProbMsg());
  }

  for(int i=0; i<obj_name.size(); i++)
    objFound(i);

  try{
    tf_listener_->lookupTransform("map", "base_link", msg->header.stamp, transform);
  }
  catch (tf::TransformException ex){
    ROS_ERROR("%s",ex.what());
  }
  if(insertIntoSeenMaps(transform) && !is_quick_search_){
    finished_ = true;
    std::cout << "FINISHED, ALL BORDER SEEN" << std::endl;
  }

  tf::Transform target_pose;
  tf::poseMsgToTF(curr_view_pose_, target_pose);
  tf::Transform diff = target_pose.inverseTimes(transform);
  if(diff.getOrigin().length() < 0.15 && std::abs(tf::getYaw(diff.getRotation())) < 0.1)
    quick_search_step_viewed_ = true;

  got_vision_ = true;
  std::cout << "VISION CALLBACK in " << (ros::Time::now()-t).toSec() << std::endl;
}


void Searcher::doCalculations(){
  if(!running_ || !got_map_ || !got_vision_)
    return;

  if(objFound()){
    finished_ = true;
    std::cout << "OBJECT FOUND: " << found_pose_.pose.position.x << " " << found_pose_.pose.position.y << " " << found_pose_.pose.position.z << std::endl;
  }

  tf::StampedTransform transform;
  try{
    tf_listener_->lookupTransform("map", "base_link", ros::Time(0), transform);
  }
  catch (tf::TransformException ex){
    ROS_ERROR("%s",ex.what());
  }
  if(is_quick_search_){
    if(calcNextQuickSearchViewpoint(quick_search_target_)){
      finished_ = true;
      std::cout << "FINISHED, NOT FOUND" << std::endl;
    }
  }
  else{
    if(calcNextViewpoint(transform)){
      finished_ = true;
      std::cout << "FINISHED, NO POSSIBLE POSITIONS LEFT" << std::endl;
    }
  }
}


void Searcher::insertObject(const pcl::PointCloud<pcl::PointXYZ>& cloud, const vision::ObjectDetectionMsg& msg){
  float min_z = std::max(POINTCLOUD_MIN_Z, 0.f);
  float max_z = std::min(POINTCLOUD_MAX_Z, obj_map_[0]->getMaxHeight()-0.001f);

  for(int i=0; i<obj_map_.size(); i++){
    ObjectMap tmp(ObjectMap(obj_map_[i]->getResolution(), obj_map_[i]->getBaseSize(), obj_map_[i]->getWidth(), obj_map_[i]->getHeight(), obj_map_[i]->getOrigin(), obj_map_[i]->getMaxHeight(), -1.f));

    for(int j=0; j<cloud.size(); j++){
      if(cloud[j].z>=min_z && cloud[j].z<=max_z){
        int x = obj_map_[i]->getXPixel(cloud[j].x);
        int y = obj_map_[i]->getYPixel(cloud[j].y);
        int z = obj_map_[i]->getZPixel(cloud[j].z);
        tmp.insertMax(x,y,z,OBJ_MIN_PROB);
        for(const auto& b : msg.samples[j].boxes){
          tmp.insertMax(x,y,z,msg.probs[b*msg.num_objects+i]);
        }
      }
    }

    for(int x=0; x<tmp.getWidth(); x++){
      for(int y=0; y<tmp.getHeight(); y++){
        for(int z=0; z<tmp.getZSteps(); z++){
          if(tmp.getProb(x,y,z) >= 0.f){
            obj_map_[i]->insertProb(x,y,z,tmp.getProb(x,y,z), OBJ_PRIOR_PROB, V_H, V_M, OBJ_MIN_PROB, OBJ_MAX_PROB);
          }
        }
      }
    }
    obj_pub_[i].publish(obj_map_[i]->getProbMsg());
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


bool Searcher::objFound(int obj){
//  ObjectMap multiplied = *obj_map_*ObjectMap(obj_map_->getResolution(), obj_map_->getBaseSize(), obj_map_->getWidth(), obj_map_->getHeight(),
//                                                     obj_map_->getOrigin(), obj_map_->getMaxHeight(), *octo_mapper_);
  if(obj < 0)
    obj = searched_obj_;
  float max_prob = 0.f;
  geometry_msgs::PoseStamped found_pose;
  for(int x=0; x<obj_map_[obj]->getWidth(); x++){
    for(int y=0; y<obj_map_[obj]->getHeight(); y++){
      for(int z=0; z<obj_map_[obj]->getZSteps(); z++){
        if(obj_map_[obj]->getCount(x,y,z) > 0){
          float prob = obj_map_[obj]->getProb(x,y,z)*octo_mapper_->getOccupancy(obj_map_[obj]->getXWorld(x), obj_map_[obj]->getYWorld(y), obj_map_[obj]->getZWorld(z));
//          for(int xk=std::max(0,x-1); xk<std::min(x+2, obj_map_->getWidth()); xk++){
//            for(int yk=std::max(0,y-1); yk<std::min(y+2, obj_map_->getHeight()); yk++){
//              for(int zk=std::max(0,z-1); zk<std::min(z+2, obj_map_->getZSteps()); zk++){
//                prob *= (1.f-multiplied.getProb(xk,yk,zk));
//              }
//            }
//          }
          if(prob > max_prob){
            max_prob = prob;
            found_pose.pose.position.x = x;
            found_pose.pose.position.y = y;
            found_pose.pose.position.z = z;
          }
        }
      }
    }
  }
  octomap_pub_.publish(octo_mapper_->getOccupiedCellMsg(ros::Time::now()));
  count_pub_.publish(octo_mapper_->getCountMsg(ros::Time::now(), 0.1f));
  if(max_prob > OBJECT_FOUND_THRESH){
    found_pose_.header.stamp = ros::Time::now();
    found_pose_.header.frame_id = "map";
    found_pose_.pose.position.x = obj_map_[obj]->getXWorld(found_pose.pose.position.x);
    found_pose_.pose.position.y = obj_map_[obj]->getYWorld(found_pose.pose.position.y);
    found_pose_.pose.position.z = obj_map_[obj]->getZWorld(found_pose.pose.position.z);
    found_pose_.pose.orientation.x = found_pose_.pose.orientation.y = found_pose_.pose.orientation.z = 0.0;
    found_pose_.pose.orientation.w = 1.0;
    obj_found_pub_.publish(found_pose_);
    obj_found_ = true;
  }
  std::cout << "Max Obj Prob " << obj_name[obj] << ":" << max_prob << (max_prob > OBJECT_FOUND_THRESH ? "  ___________________________________________ FOUND" : "") << std::endl;
  return obj_found_;
}


cv::Mat_<float> Searcher::getProbMap(cv::Point& origin){
  ObjectMap count_map(1.f,1.f,1.f,0.f);
  ObjectMap occ_map(obj_map_[0]->getResolution(), obj_map_[0]->getBaseSize(), obj_map_[0]->getWidth(), obj_map_[0]->getHeight(),
                    obj_map_[0]->getOrigin(), obj_map_[0]->getMaxHeight(), *octo_mapper_, &count_map);

  for(int i=0; i<obj_map_.size(); i++){
    full_pub_[i].publish((*obj_map_[i]*occ_map).getProbMsg());
  }

  return obj_map_[searched_obj_]->get2D(occ_map, *prior_prob_map_[searched_obj_], count_map, SAMPLE_COUNT_THRESH, origin);
//  cv::Mat_<float> tmp = obj_map_->get2D(occ_map, *prior_prob_map_, count_map, 1000);
//  float factor = 1.f/(VIEW_POS_RESOLUTION*obj_map_->getResolution();
//  cv::resize(tmp, tmp, cv::Size(factor*obj_map_->getWidth(),factor*obj_map_->getHeight()));
}


cv::Mat_<uchar> Searcher::getViewKernel(float angle, float max_dist, float resolution) const{
  cv::Mat_<uchar> kernel(int(resolution*max_dist)*2+1, int(resolution*max_dist)*2+1, uchar(0));
  cv::Point center(resolution*max_dist,resolution*max_dist);
  std::vector<cv::Point> points;
  points.push_back(cv::Point(std::cos(angle-VIEW_ANGLE/2.f)*VIEW_MIN_DIST*resolution, std::sin(angle-VIEW_ANGLE/2.f)*VIEW_MIN_DIST*resolution)+center);
  points.push_back(cv::Point(std::cos(angle-VIEW_ANGLE/2.f)*max_dist*resolution, std::sin(angle-VIEW_ANGLE/2.f)*max_dist*resolution)+center);
  points.push_back(cv::Point(std::cos(angle+VIEW_ANGLE/2.f)*max_dist*resolution, std::sin(angle+VIEW_ANGLE/2.f)*max_dist*resolution)+center);
  points.push_back(cv::Point(std::cos(angle+VIEW_ANGLE/2.f)*VIEW_MIN_DIST*resolution, std::sin(angle+VIEW_ANGLE/2.f)*VIEW_MIN_DIST*resolution)+center);
  cv::fillConvexPoly(kernel, points, cv::Scalar(255));

  return kernel;
}


float Searcher::calcMoveTime(const cv::Point& pos, float angle, const cv::Point& curr_pos, float curr_angle){
  return 1.f;
  cv::Point diff = pos-curr_pos;
  float move_angle = std::atan2(float(diff.y),float(diff.x));
  return (angleDist(curr_angle,move_angle)+angleDist(move_angle,angle))/TURN_SPEED + std::sqrt(float(diff.x)*diff.x+diff.y*diff.y)/MOVE_SPEED +
    VIEW_TIME + (good_accessible_map_(pos) ? 0.f : 10.f/MOVE_SPEED);
}


inline cv::Point poseToPoint(const tf::Transform& pose, cv::Point origin, float resolution){
  return cv::Point(pose.getOrigin().x()*resolution + origin.x, pose.getOrigin().y()*resolution + origin.y);
}


float Searcher::calcViewpointGain(const cv::Point& pos, int angle_step, const cv::Mat_<float> &prob_map, const cv::Point& curr_pos, float curr_angle){
  float angle = float(angle_step)/VIEW_ANGLE_STEPS*M_PI*2;
  float prob = 1.0;
  float interesting_border_seen = 0.f;
  int idx = -1;
  static int old_angle_step = -1;
  cv::Mat_<float> seen(prob_map.rows, prob_map.cols, 0.f);
  cv::Mat_<float> angle_diff(prob_map.rows, prob_map.cols, 0.f);
  for(const auto& p : seen_kernel_points_[angle_step]){
    idx++;
    if(!(pos+p).inside(cv::Rect(0,0,prob_map.cols,prob_map.rows))){
      std::cout << "out" << std::endl;
      continue;
    }
    //if(not_fully_viewed_border_(pos+p) > 0 && angleDist(angle, border_dir_map_(pos+p)) < BORDER_SEEN_MAX_ANGLE)
      interesting_border_seen += seen_kernel_points_value_[angle_step][idx];

    if(angleDist(border_dir_map_(pos+p), std::atan2(float(p.y),float(p.x))) < BORDER_SEEN_MAX_ANGLE)
      prob *= (1.f-prob_map(pos+p)*seen_kernel_points_value_[angle_step][idx]);

    //angle_diff(pos+p) = 0.5f + angleDist(border_dir_map_(pos+p), std::atan2(float(p.y),float(p.x)))/(2*M_PI);

//    if(old_angle_step != angle_step){
//      if(angleDist(border_dir_map_(pos+p), std::atan2(float((p).y),float((p).x))) < BORDER_SEEN_MAX_ANGLE)
//        seen(pos+p) = 0.5 + 0.5*std::pow(prob_map(pos+p)*seen_kernel_points_value_[angle_step][idx], 1.f);
//      else
//        seen(pos+p) = 0.2;
//    }
  }
  if(interesting_border_seen < 0.0f)
    return 0.f;

//  if(old_angle_step != angle_step){
//    //old_angle_step = angle_step;
//    seen(pos) = 1.f;
//    showProbImage("angle_diff", angle_diff, 4, 255-accessible_map_*0.3);
//    showProbImage("seen" + std::to_string(pos.x) + " " + std::to_string(pos.y) + " " + std::to_string(angle_step) + " " + std::to_string(angle*180.0/M_PI), seen, 4, 255-accessible_map_*0.3);
//    cv::waitKey();
//    cv::destroyWindow("seen" + std::to_string(pos.x) + " " + std::to_string(pos.y) + " " + std::to_string(angle_step) + " " + std::to_string(angle*180.0/M_PI));
//  }

//  std::cout << (1.f-prob + interesting_border_seen*INTERESTING_BORDER_SEEN_REWARD)/calcMoveTime(pos, angle, curr_pos, curr_angle)*100.f << " "
//            << 1.f-prob << " " << pos.x << " " << pos.y << " " << angle_step << " " << angle << " " << interesting_border_seen<< std::endl;
  return (1.f-prob/* + interesting_border_seen*INTERESTING_BORDER_SEEN_REWARD*/)/calcMoveTime(pos, angle, curr_pos, curr_angle);
}


bool Searcher::calcNextViewpoint(const tf::Transform& curr_pose){
  cv::Point new_origin;
  cv::Mat_<float> prob_map = getProbMap(new_origin);

  cv::Mat_<float> tmp;
  prob_map.copyTo(tmp);
  cv::pow(tmp, 0.25, tmp);
  double prob = 1.0;
  for(int x=0; x<prob_map.cols; x++){
    for(int y=0; y<prob_map.rows; y++){
      prob *= (1.0-prob_map(y,x));
    }
  }
  prob = 1-prob;
  std::cout << "Remaining prob: " << prob << std::endl;
  showProbImage("probabilities", tmp, 4, 255-accessible_map_*0.3);

  cv::Point curr_point = poseToPoint(curr_pose, obj_map_[0]->getOrigin(), obj_map_[0]->getResolution());
  float curr_angle = tf::getYaw(curr_pose.getRotation());
  int curr_step = int(curr_angle/(2*M_PI)*VIEW_ANGLE_STEPS+VIEW_ANGLE_STEPS)%VIEW_ANGLE_STEPS;

  double max = -1.0;
  int max_i;
  cv::Point max_loc;
  cv::Mat_<uchar> seen_mat(prob_map.rows, prob_map.cols, uchar(0));
  std::vector<cv::Mat_<float>> prob_mats;
  for(int i=0; i<VIEW_ANGLE_STEPS; i++){
    prob_mats.push_back(cv::Mat_<float>(prob_map.rows, prob_map.cols,0.f));
    //cv::Mat_<float> sdf(prob_map.rows, prob_map.rows, 0.f);
    for(int x=0; x<prob_map.cols; x++){
      for(int y=0; y<prob_map.rows; y++){
        if(accessible_map_(y,x) && !previous_pose_maps_[i](y,x) &&
                ((std::abs(curr_point.x-x)>2 || std::abs(curr_point.y-y)>2 || (std::abs(curr_step-i)>1 && std::abs(curr_step-i) < VIEW_ANGLE_STEPS-2)))){
          float prob = calcViewpointGain(cv::Point(x,y), i, prob_map, curr_point, curr_angle);
          //sdf(y,x) = prob;//*calcMoveTime(cv::Point(x,y), float(i)/VIEW_ANGLE_STEPS*M_PI*2, poseToPoint(curr_pose, obj_map_->getOrigin(), obj_map_->getResolution()), tf::getYaw(curr_pose.getRotation()));
          if(prob > max){
            max = prob;
            max_loc = cv::Point(x,y);
            max_i = i;
            seen_mat = cv::Mat_<uchar>(prob_map.rows, prob_map.cols, uchar(0));
            for(const auto& p : seen_kernel_points_[i]){
              if(!(cv::Point(x,y)+p).inside(cv::Rect(0,0,prob_map.cols,prob_map.rows)))
                continue;
              seen_mat(cv::Point(x,y)+p) = 255;
            }
          }
          prob_mats[i](y,x) = prob;
        }
        //if(border_map_(y,x))
        //  sdf(y,x) = 1.f;
      }
    }
    //showProbImage(std::to_string(i), sdf, 2);
    //showProbImage(std::to_string(i), sdf, 2);
  }

  for(int i=0; i<VIEW_ANGLE_STEPS; i++){
    prob_mats[i] = prob_mats[i]/max;
    showProbImage("prob_mat_"+std::to_string(i*30)+"_max_"+std::to_string(max), prob_mats[i], 4, accessible_map_);
    cv::waitKey(0);
  }

  cv::resize(seen_mat, seen_mat, cv::Size(seen_mat.cols*2, seen_mat.rows*2), 0, 0, cv::INTER_NEAREST);
  cv::flip(seen_mat,seen_mat,0);
  cv::imshow("SEEEEEEEN", seen_mat);
  cv::waitKey(1);

  if(max < 0.0){
    return true;
  }

  tf::Transform curr_view(tf::createQuaternionFromYaw(float(max_i)/VIEW_ANGLE_STEPS*M_PI*2.f),
                          tf::Vector3((max_loc.x-obj_map_[0]->getOrigin().x)/RESOLUTION, (max_loc.y-obj_map_[0]->getOrigin().y)/RESOLUTION, 0.0));
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


bool Searcher::calcNextQuickSearchViewpoint(const geometry_msgs::Pose& target_pose){
  if(quick_search_step_viewed_){
    quick_search_step_viewed_ = false;

    quick_search_step_++;
    if(quick_search_step_ > QUICK_SEARCH_VIEWS)
      return true;

    cv::Point target_pos(obj_map_[0]->getXPixel(target_pose.position.x), obj_map_[0]->getYPixel(target_pose.position.y));
    cv::Point new_pos;
    float new_angle;
    if(accessible_map_(target_pos)){
      int tries = 0;
      do{
        float r = (QUICK_SEARCH_MIN_DIST + (rand()%20)/19.f*(QUICK_SEARCH_MAX_DIST-QUICK_SEARCH_MIN_DIST))*RESOLUTION;
        new_angle = (rand()%180)/180.f*2*M_PI;
        new_pos = target_pos - cv::Point(r*std::cos(new_angle), r*std::sin(new_angle));
      }while(!accessible_map_(new_pos) && tries++ < 1000);
      if(!accessible_map_(new_pos) && tries < 1000)
        return true;
    }
    else{
      int tries = 0;
      float target_angle = border_dir_map_(target_pos);
      do{
        float r = (QUICK_SEARCH_MIN_DIST + (rand()%20)/19.f*(QUICK_SEARCH_MAX_DIST-QUICK_SEARCH_MIN_DIST))*RESOLUTION;
        new_angle = target_angle - QUICK_SEARCH_ANGLE_AREA + (rand()%180)/180.f*(2*QUICK_SEARCH_ANGLE_AREA);
        new_pos = target_pos - cv::Point(r*std::cos(new_angle), r*std::sin(new_angle));
      }while(!accessible_map_(new_pos) && tries++ < 1000);
      if(!accessible_map_(new_pos) && tries < 1000)
        return true;
    }
    tf::Transform curr_view(tf::createQuaternionFromYaw(new_angle), tf::Vector3(obj_map_[0]->getXWorld(new_pos.x), obj_map_[0]->getYWorld(new_pos.y), 0.0));
    tf::poseTFToMsg(curr_view, curr_view_pose_);
    curr_view_changed_ = true;
    geometry_msgs::PoseStamped pose;
    pose.header.stamp = ros::Time::now();
    pose.pose = curr_view_pose_;
    pose.header.frame_id = "map";
    next_pose_pub_.publish(pose);
    have_curr_view_ = true;
  }

  return false;
}

bool Searcher::insertIntoSeenMaps(const tf::Transform &curr_pose){
  if(!got_map_)
    return false;

  double angle = tf::getYaw(curr_pose.getRotation());
  while(angle < 0)
    angle += 2*M_PI;
  while(angle >= 2*M_PI)
    angle -= 2*M_PI;
  int idx = angle/(2*M_PI)*SEEN_MAP_STEPS;
  int other_idx = ((angle/(2*M_PI)*SEEN_MAP_STEPS - idx) < 0.5 ? (idx+SEEN_MAP_STEPS-1)%SEEN_MAP_STEPS : (idx+1)%SEEN_MAP_STEPS);

  cv::Point pos = poseToPoint(curr_pose, obj_map_[0]->getOrigin(), RESOLUTION);
  cv::Mat_<uchar> kernel = getViewKernel(angle, SEEN_MAP_MAX_DIST, RESOLUTION);
  int x1=pos.x-kernel.cols/2, y1=pos.y-kernel.rows/2;
  kernel = kernel(cv::Rect(cv::Point(std::max(-x1,0),std::max(-y1,0)), cv::Point(std::min(kernel.cols,seen_maps_[0].cols-x1),std::min(kernel.rows,seen_maps_[0].rows-y1))));
  x1 = std::max(x1,0);
  y1 = std::max(y1,0);
  cv::Mat(seen_maps_[idx](cv::Rect(x1,y1,kernel.cols,kernel.rows)) + kernel).copyTo(seen_maps_[idx](cv::Rect(x1,y1,kernel.cols,kernel.rows)));
  cv::Mat(seen_maps_[other_idx](cv::Rect(x1,y1,kernel.cols,kernel.rows)) + kernel).copyTo(seen_maps_[other_idx](cv::Rect(x1,y1,kernel.cols,kernel.rows)));

  cv::threshold(seen_maps_[idx], seen_maps_[idx], 100, 100, cv::THRESH_TRUNC);
  cv::threshold(seen_maps_[other_idx], seen_maps_[other_idx], 100, 100, cv::THRESH_TRUNC);

  cv::Mat_<uchar> tmp;
  border_map_.copyTo(tmp);
  for(int x=0; x<border_dir_map_.cols; x++){
    for(int y=0; y<border_dir_map_.rows; y++){
      if(border_map_(y,x) > 0){
        int border_idx = border_dir_map_(y,x)/(2*M_PI)*SEEN_MAP_STEPS;
        if(seen_maps_[border_idx](y,x)+seen_maps_[(border_idx+1)%SEEN_MAP_STEPS](y,x)+seen_maps_[(border_idx+SEEN_MAP_STEPS-1)%SEEN_MAP_STEPS](y,x) >= BORDER_SEEN_THRESH)
          tmp(y,x) = 0;
      }
    }
  }
  not_fully_viewed_border_ = cv::Mat_<uchar>(tmp.rows, tmp.cols, uchar(0));
  for(int x=1; x<tmp.cols-1; x++){
    for(int y=1; y<tmp.rows-1; y++){
      if(tmp(y,x)){
        int sum = 0;
        for(int xk = x - 1; xk <= x + 1; xk++){
          for(int yk = y - 1; yk <= y + 1; yk++){
            if(tmp(yk, xk))
              sum++;
          }
        }
        if(sum > 1)
          not_fully_viewed_border_(y, x) = 255;
      }
    }
  }
  //previous_pose_maps_[angle/(2*M_PI)*VIEW_ANGLE_STEPS](pos) = 255;

  cv::threshold(border_map_, tmp, 64, 64, cv::THRESH_TRUNC);
  cv::bitwise_or(tmp, not_fully_viewed_border_, tmp);
  cv::threshold(kernel, kernel, 32, 32, cv::THRESH_TRUNC);
  cv::bitwise_or(not_fully_viewed_border_(cv::Rect(x1,y1,kernel.cols,kernel.rows)), kernel, tmp(cv::Rect(x1,y1,kernel.cols,kernel.rows)));
  cv::resize(tmp, tmp, cv::Size(not_fully_viewed_border_.cols*2, not_fully_viewed_border_.rows*2), 0, 0, cv::INTER_NEAREST);
  cv::flip(tmp, tmp, 0);
  cv::imshow("not_fully_viewed_border_", tmp);
  cv::moveWindow("not_fully_viewed_border_", 1600,300);
  cv::waitKey(1);
  std::cout << "In seen map inserted" << std::endl;

  return (countNonZero(not_fully_viewed_border_) == 0);
}

void Searcher::searchQueryCb(const geometry_msgs::QuaternionConstPtr& msg){
  int obj = std::round(msg->w);
  tf::Transform trans_tf(tf::createQuaternionFromYaw(msg->z), tf::Vector3(msg->x, msg->y, 0.0));
  semantic_mapping_v2::ObjectMapSrvRequest req;
  req.id = obj;
  req.room_id = -1;
  semantic_mapping_v2::ObjectMapSrvResponse res;
  if(!obj_map_service_client_.call(req, res)){
    ROS_WARN("SEMANTIC MAP CALL FAILED");
    return;
  }

  delete prior_prob_map_[obj];
  prior_prob_map_[obj] = new ObjectMap(res.maps[0]);
  prior_prob_map_[obj]->resample(*obj_map_[obj], 0.0);
  prior_pub_.publish(prior_prob_map_[obj]->getProbMsg(obj, 0.01f));
  searched_obj_ = obj;
  calcNextViewpoint(trans_tf);
}