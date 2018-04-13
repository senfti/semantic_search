//
// Created by thomas on 02.01.18.
//

#include <execution/Searcher.h>
#include <pcl/common/common.h>
#include <pcl/common/geometry.h>
#include <semantic_mapping_v2/ObjectMapSrv.h>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/PoseArray.h>
#include <pcl_ros/point_cloud.h>
#include <tf_conversions/tf_eigen.h>
#include <prob_map_view/ProbMapMsg.h>

const std::vector<std::string> obj_names = { "person","bicycle","bird","cat","dog","backpack","umbrella","handbag","tie","suitcase","frisbee","ski","snowboard",
                                             "sportball","kite","baseballbat","glove","skateboard","surfboard","racket","bottle","wineglass","cup","fork",
                                             "knife","spoon","bowl","banana","apple","sandwich","orange","broccoli","carrot","hotdog","pizza","donut","cake",
                                             "chair","sofa","potplant","bed","table","toilet","monitor","laptop","mouse","remote","keyboard","cellphone",
                                             "microwave","oven","toaster","sink","refrigerator","book","clock","vase","scissor","teddybear","hairdryer","toothbrush","all"};

inline float angleDist(float angle1, float angle2){
  float angle = angle1-angle2;
  while(angle > M_PI)
    angle-=2*M_PI;
  while(angle <= -M_PI)
    angle+=2*M_PI;
  return std::abs(angle);
}


inline float gaussian(float x, float sigma){
  float val = std::exp(-0.5*x*x/(sigma*sigma));
  return (val < 0.01f ? 0.0f : val);
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
  cv::moveWindow(name, 1600,0);
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
  cv::moveWindow(name, 1600,0);
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
  private_nh.param("BAD_ACCESSIBLE_PENALTY", BAD_ACCESSIBLE_PENALTY, BAD_ACCESSIBLE_PENALTY);

  private_nh.param("BORDER_SEEN_THRESH", BORDER_SEEN_THRESH, BORDER_SEEN_THRESH);
  private_nh.param("BORDER_SEEN_SIGMA", BORDER_SEEN_SIGMA, BORDER_SEEN_SIGMA);
  private_nh.param("SEEN_MAP_MAX_DIST", SEEN_MAP_MAX_DIST, SEEN_MAP_MAX_DIST);
  private_nh.param("INTERESTING_BORDER_SEEN_REWARD", INTERESTING_BORDER_SEEN_REWARD, INTERESTING_BORDER_SEEN_REWARD);
  private_nh.param("OBJECT_PROB_UPDATE_INTERVAL", OBJECT_PROB_UPDATE_INTERVAL, OBJECT_PROB_UPDATE_INTERVAL);

  while(!obj_map_service_client_.waitForExistence(ros::Duration(0.1))){
    ROS_WARN("HIERARCHY SERVICE NOT EXISTING");
    ros::spinOnce();
  }

  map_sub_ = ros::NodeHandle().subscribe("map_door_blocked", 1, &Searcher::mapCb, this);
  vision_sub_ = ros::NodeHandle().subscribe("vision_result", 1, &Searcher::visionCb, this);
  obj_found_sub_ = ros::NodeHandle().subscribe("obj_found_in_image", 1, &Searcher::objFoundCb, this);

  octomap_pub_ = ros::NodeHandle().advertise<visualization_msgs::MarkerArray>("searcher_occ", 1, true);
  obj_pub_ = ros::NodeHandle().advertise<visualization_msgs::MarkerArray>("searcher_obj", 1, true);
  full_pub_ = ros::NodeHandle().advertise<visualization_msgs::MarkerArray>("searcher_full_obj", 1, true);
  next_pose_pub_ = ros::NodeHandle().advertise<geometry_msgs::PoseStamped>("searcher_pose", 1, true);
  obj_found_pub_ = ros::NodeHandle().advertise<sensor_msgs::PointCloud2>("object_found_pose", 1, true);
  count_pub_ = ros::NodeHandle().advertise<visualization_msgs::MarkerArray>("count_occ", 1, true);
  count2_pub_ = ros::NodeHandle().advertise<visualization_msgs::MarkerArray>("count2_occ", 1, true);
  prior_pub_ = ros::NodeHandle().advertise<visualization_msgs::MarkerArray>("searcher_prior_map", 1, true);
  prob_map_pub_ = ros::NodeHandle().advertise<prob_map_view::ProbMapMsg>("searcher_prob_map", 1, true);

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

void Searcher::start(int searched_obj, int searched_room){
  if(running_)
    return;

  bool reset_searcher = (searched_obj != searched_obj_ || searched_room != searched_room_);
  searched_obj_ = searched_obj;
  searched_room_ = searched_room;
  running_ = true;
  finished_ = false;
  finished_count_ = 0;
  obj_found_ = false;
  have_curr_view_ = false;
  curr_view_changed_= true;
  got_map_ = false;
  got_vision_ = false;
  search_step_viewed_ = true;

  std::system(("rosrun dynamic_reconfigure dynparam set /move_base/DWAPlannerROS max_rot_vel " + std::to_string(SEARCH_MAX_ROT_VEL)).c_str());
  std::system(("rosrun dynamic_reconfigure dynparam set /move_base/DWAPlannerROS max_trans_vel " + std::to_string(SEARCH_MAX_TRANS_VEL)).c_str());

  if(reset_searcher){
    seen_maps_.clear();
    previous_pose_maps_.clear();
    accessible_map_ = cv::Mat_<uchar>();
    border_map_ = cv::Mat_<uchar>();
    border_dir_map_ = cv::Mat_<float>();
    not_fully_viewed_border_ = cv::Mat_<uchar>();

    delete obj_map_;
    delete octo_mapper_;

    obj_map_ = nullptr;
    octo_mapper_ = nullptr;
  }

  delete prior_prob_map_;
  prior_prob_map_ = nullptr;
  semantic_mapping_v2::ObjectMapSrvRequest req;
  req.id = searched_obj_;
  req.room_id = -1;
  semantic_mapping_v2::ObjectMapSrvResponse res;
  if(!obj_map_service_client_.call(req, res)){
    ROS_WARN("SEMANTIC MAP CALL FAILED");
    return;
  }
  prior_prob_map_ = new ObjectMap(res.maps[0]);

  if(octo_mapper_ == nullptr){
    octo_mapper_ = new OctoMapper(RESOLUTION);
    obj_map_ = new ObjectMap(RESOLUTION, 4.0, prior_prob_map_->getMaxHeight(), OBJ_PRIOR_PROB);
    obj_map_->expandUntilFitting(prior_prob_map_->getMinX(), prior_prob_map_->getMaxX(), prior_prob_map_->getMinY(), prior_prob_map_->getMaxY(), OBJ_PRIOR_PROB);

    seen_maps_.resize(SEEN_MAP_STEPS);
    previous_pose_maps_.resize(VIEW_ANGLE_STEPS);
    for(auto &map : seen_maps_)
      map = cv::Mat_<float>(obj_map_->getHeight(), obj_map_->getWidth(), 0.f);
    for(auto &map : previous_pose_maps_)
      map = cv::Mat_<float>(obj_map_->getHeight(), obj_map_->getWidth(), 0.f);
  }

  prior_pub_.publish(prior_prob_map_->getProbMsg(0.00001f));
  prior_prob_map_->resample(*obj_map_, 0.0);
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
}


void Searcher::calcSeenKernels(){
  std::cout << "a";
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
    //cv::Mat_<float> sdf(kernel.rows, kernel.cols, 0.f);
    for(int x=0;x<kernel.cols; x++){
      for(int y=0; y<kernel.rows; y++){
        if(kernel(y,x)){
          cv::Point diff = cv::Point(x,y)-center;
          seen_kernel_points_[i].push_back(diff);
          float va = std::min(1.5f-2.f*angleDist(std::atan2(float(diff.y),float(diff.x)), angle), 1.f);
          float r = std::sqrt(float(diff.x)*diff.x+diff.y*diff.y)/(RESOLUTION);
          float vr = std::max(std::min(1.f, 1.f-(r-1.f)/2.5f),0.f);
          seen_kernel_points_value_[i].push_back(va*vr);
          //sdf(y,x) = va*vr;
        }
      }
    }
    //showProbImage("kernel", sdf, 4);
    //cv::waitKey(0);
  }
}


void Searcher::resize(float x1, float x2, float y1, float y2){
  std::cout << "b";
  std::vector<int> border = obj_map_->expandUntilFitting(x1, x2, y1, y2, OBJ_PRIOR_PROB);
  if(!border.empty()){
    prior_prob_map_->resize(border[2], border[3], border[0], border[1], 0.f);
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
  if(!running_ || obj_map_==nullptr)
    return;

  std::cout << "c";
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

  std::cout << "d";
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

  std::cout << "e";
  double factor = obj_map_->getResolution()*msg->info.resolution;
  good_accessible_map_.clear();
  for(int i=1; i<=1; i++){
    good_accessible_map_.push_back(cv::Mat_<uchar>(obj_map_->getHeight(), obj_map_->getWidth(), 0.f));
    cv::Mat_<uchar> tmp;
    cv::erode(accessible_mat, tmp, cv::Mat_<uchar>::ones(3,3), cv::Point(-1,-1), i+1);
    cv::resize(tmp, tmp, cv::Size(accessible_mat.cols*factor, accessible_mat.rows*factor));
    cv::threshold(tmp, tmp, 192, 255, cv::THRESH_BINARY);
    tmp.copyTo(good_accessible_map_.back()(cv::Rect(obj_map_->getXPixel(msg->info.origin.position.x), obj_map_->getYPixel(msg->info.origin.position.y), tmp.cols, tmp.rows)));
  }
  cv::resize(accessible_mat, accessible_mat, cv::Size(accessible_mat.cols*factor, accessible_mat.rows*factor));
  cv::threshold(accessible_mat, accessible_mat, 192, 255, cv::THRESH_BINARY);
  accessible_map_ = cv::Mat_<uchar>(obj_map_->getHeight(), obj_map_->getWidth(), uchar(0));
  accessible_mat.copyTo(accessible_map_(cv::Rect(obj_map_->getXPixel(msg->info.origin.position.x), obj_map_->getYPixel(msg->info.origin.position.y), accessible_mat.cols, accessible_mat.rows)));

  cv::Sobel(dists, grad_x, CV_32F, 1, 0, 5);
  cv::Sobel(dists, grad_y, CV_32F, 0, 1, 5);
  cv::resize(grad_x, grad_x, cv::Size(accessible_mat.cols, accessible_mat.rows));
  cv::resize(grad_y, grad_y, cv::Size(accessible_mat.cols, accessible_mat.rows));
  cv::cartToPolar(grad_x, grad_y, mag, dir);

  border_dir_map_ = cv::Mat_<uchar>(obj_map_->getHeight(), obj_map_->getWidth(), 0.f);
  dir.copyTo(border_dir_map_(cv::Rect(obj_map_->getXPixel(msg->info.origin.position.x), obj_map_->getYPixel(msg->info.origin.position.y), dir.cols, dir.rows)));

  cv::Mat_<uchar> tmp, tmp2;
  cv::dilate(accessible_map_, tmp, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(std::ceil(ROBOT_SIZE*factor)*2+1,std::ceil(ROBOT_SIZE*factor)*2+1)));
  cv::dilate(tmp, tmp2, cv::Mat_<uchar>::ones(3,3));
  border_map_ = tmp2-tmp;

//  cv::Mat_<float> bla = border_dir_map_/(2.0*M_PI);
//  showProbImage("border_dir_map", bla, 4, 255-accessible_map_*0.3);
//  cv::flip(dists/20.0, mag, 0);
//  cv::resize(mag, mag, cv::Size(mag.cols, mag.rows), 0, 0, cv::INTER_NEAREST);
//  cv::imshow("dists", mag);
//  cv::flip(accessible_map_, mag, 0);
//  cv::resize(mag, mag, cv::Size(mag.cols*4, mag.rows*4), 0, 0, cv::INTER_NEAREST);
//  cv::imshow("accessible_map_", mag);
//  cv::waitKey(1);

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

  std::cout << "f";
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

  tf::Transform target_pose;
  tf::poseMsgToTF(curr_view_pose_, target_pose);
  tf::Transform diff = target_pose.inverseTimes(transform);
  if(diff.getOrigin().length() < 0.2 && std::abs(tf::getYaw(diff.getRotation())) < 0.05 || search_goal_reached_){
    search_step_viewed_ = true;
    search_goal_reached_ = false;
  }

  got_vision_ = true;
  std::cout << "VISION CALLBACK in " << (ros::Time::now()-t).toSec() << std::endl;
}


bool Searcher::doCalculations(bool force_new){
  if(!running_ || !got_map_ || !got_vision_)
    return false;

  std::cout << "g";
  if(objFound()){
    finished_ = true;
    return false;
  }

  tf::StampedTransform t1, t2;
  tf::StampedTransform transform;
  try{
    tf_listener_->lookupTransform("odom", "base_link", ros::Time(ros::Time::now()-ros::Duration(5.0)), t1);
    tf_listener_->lookupTransform("odom", "base_link", ros::Time(0.0), t2);
    transform = tf::StampedTransform(t1.inverseTimes(t2), ros::Time(), "/base_link", "/odom");
  }
  catch (tf::TransformException ex){
    ROS_ERROR("%s",ex.what());
    return false;
  }

  static ros::Time last_calc_time(0);
  bool need_new_pose = (ros::Time::now() - last_calc_time > ros::Duration(10.0) && transform.getOrigin().length() < 0.2 && angleDist(tf::getYaw(transform.getRotation()),0.0) < 0.2);
  if(!force_new && finished_count_==0 && !search_step_viewed_ && !need_new_pose)
    return false;

  last_calc_time = ros::Time::now();
  search_step_viewed_= false;

  try{
    tf_listener_->lookupTransform("map", "base_link", ros::Time(0), transform);
  }
  catch (tf::TransformException ex){
    ROS_ERROR("%s",ex.what());
    return false;
  }
  if(calcNextViewpoint(transform, need_new_pose)){
    if(finished_count_ > 0){
      finished_ = true;
      std::cout << "FINISHED, NO POSSIBLE POSITIONS LEFT" << std::endl;
    }
    finished_count_++;
    return false;
  }

  finished_count_ = 0;
  return true;
}


void Searcher::insertObject(const pcl::PointCloud<pcl::PointXYZ>& cloud, const vision::ObjectDetectionMsg& msg){
  std::cout << "h";
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
  //obj_pub_.publish(obj_map_->getProbMsg());

  std::cout << "Obj inserted" << std::endl;
}


void Searcher::insertCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, const tf::Point& sensorOriginTf){
  std::cout << "i";
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

  octo_mapper_->insertCloud(pc, pc_ground, sensorOriginTf);
}


void Searcher::objFoundCb(const vision::ObjectFoundMsgConstPtr& msg){
  std::cout << "j";
  tf::StampedTransform transform;
  try{
    tf_listener_->lookupTransform("map", "camera_rgb_optical_frame", msg->header.stamp, transform);
  }
  catch (tf::TransformException ex){
    try{
      tf_listener_->lookupTransform("map", "camera_rgb_optical_frame", ros::Time(0), transform);
    }
    catch (tf::TransformException ex){
      ROS_ERROR("%s",ex.what());
      return;
    }
  }
  for(int i=0; i<msg->object_type.size(); i++){
    if(msg->object_type[i] != searched_obj_)
      continue;

    tf::Vector3 p(msg->positions[i].x,msg->positions[i].y,msg->positions[i].z);
    p = transform*p;
    found_pose_.push_back(pcl::PointXYZ(p.x(), p.y(), p.z()));
    found_pose_.header.frame_id = "map";
    found_pose_.header.stamp = ros::Time::now().toSec();
    obj_found_pub_.publish(found_pose_);
    obj_found_ = true;
    std::cout << "IMAGE OBJ FOUND " << msg->object_type[i] << " " << obj_names[msg->object_type[i]] << " " << p.x() << " " << p.y() << " " << p.z() << std::endl;
  }
}


bool Searcher::objFound(){
//  ObjectMap multiplied = *obj_map_*ObjectMap(obj_map_->getResolution(), obj_map_->getBaseSize(), obj_map_->getWidth(), obj_map_->getHeight(),
//                                                     obj_map_->getOrigin(), obj_map_->getMaxHeight(), *octo_mapper_);
  std::cout << "k";
  float max_prob = 0.f;
  geometry_msgs::PoseStamped found_pose;
  for(int x=0; x<obj_map_->getWidth(); x++){
    for(int y=0; y<obj_map_->getHeight(); y++){
      for(int z=0; z<obj_map_->getZSteps(); z++){
        if(obj_map_->getCount(x,y,z) > 0){
          float prob = obj_map_->getProb(x,y,z)*octo_mapper_->getOccupancy(obj_map_->getXWorld(x), obj_map_->getYWorld(y), obj_map_->getZWorld(z));
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
    //octomap_pub_.publish(octo_mapper_->getOccupiedCellMsg(ros::Time::now()));
    //count_pub_.publish(octo_mapper_->getCountMsg(ros::Time::now(), 0.1f));
  if(max_prob > OBJECT_FOUND_THRESH){
    pcl::PointXYZ found_pos(obj_map_->getXWorld(found_pose.pose.position.x), obj_map_->getYWorld(found_pose.pose.position.y), obj_map_->getZWorld(found_pose.pose.position.z));

    found_pose_.push_back(found_pos);
    found_pose_.header.frame_id = "map";
    found_pose_.header.stamp = ros::Time::now().toSec();
    obj_found_pub_.publish(found_pose_);
    obj_found_ = true;
    std::cout << "OBJ FOUND " << " " << found_pos.x << " " << found_pos.y << " " << found_pos.z << std::endl;
  }
  std::cout << "Max Obj Prob " << obj_names[searched_obj_] << ":" << max_prob << std::endl;
  return obj_found_;
}


cv::Mat_<float> Searcher::getProbMap(cv::Point& origin){
  std::cout << "l";
  ObjectMap occ_map(obj_map_->getResolution(), obj_map_->getBaseSize(), obj_map_->getWidth(), obj_map_->getHeight(),
                    obj_map_->getOrigin(), obj_map_->getMaxHeight(), *octo_mapper_);

  static int i=1;
  if(i%OBJECT_PROB_UPDATE_INTERVAL == 0){
    semantic_mapping_v2::ObjectMapSrvRequest req;
    req.id = searched_obj_;
    req.room_id = -1;
    semantic_mapping_v2::ObjectMapSrvResponse res;
    if(!obj_map_service_client_.call(req, res)){
      ROS_WARN("SEMANTIC MAP CALL FAILED");
    }
    else{
      delete prior_prob_map_;
      prior_prob_map_ = nullptr;
      prior_prob_map_ = new ObjectMap(res.maps[0]);
      prior_pub_.publish(prior_prob_map_->getProbMsg(0.00001f));
      prior_prob_map_->resample(*obj_map_, 0.0);
    }
  }
  i++;

  //full_pub_.publish((*obj_map_*occ_map).getProbMsg());
  //count2_pub_.publish(occ_map.getCountMsg(0.1f));

  cv::Mat_<float> prob_maps = obj_map_->get2D(occ_map, *prior_prob_map_, SAMPLE_COUNT_THRESH, origin);
  cv::Mat_<uchar> occupancy_map_obj(prob_maps.rows, prob_maps.cols, uchar(255));
  prob_map_view::ProbMapMsg msg;
  msg.names.push_back("ProbMap");
  msg.img_are_log = 0;
  msg.occupancy.rows = occupancy_map_obj.rows;
  msg.occupancy.cols = occupancy_map_obj.cols;
  msg.occupancy.type = occupancy_map_obj.type();
  msg.occupancy.data.assign(occupancy_map_obj.datastart, occupancy_map_obj.dataend);
  msg.images.resize(msg.names.size());
  for(int j=0; j<msg.images.size(); j++){
    msg.images[j].rows = prob_maps.rows;
    msg.images[j].cols = prob_maps.cols;
    msg.images[j].type = prob_maps.type();
    msg.images[j].data.assign(prob_maps.datastart, prob_maps.dataend);
  }
  prob_map_pub_.publish(msg);

  return prob_maps;

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


float Searcher::calcMoveTime(const cv::Point& pos, float angle, const cv::Point2f& curr_pos, float curr_angle){
  cv::Point2f diff((pos.x-curr_pos.x)/RESOLUTION, (pos.y-curr_pos.y)/RESOLUTION);
  float bad_accessible_penalty = 0.f;
  for(int i=0; i<good_accessible_map_.size(); i++){
    bad_accessible_penalty += (good_accessible_map_[i](pos) ? 0.f : BAD_ACCESSIBLE_PENALTY/MOVE_SPEED);
  }
  if(std::abs(diff.x) < 0.15 && std::abs(diff.y) < 0.15){
    return angleDist(curr_angle, angle)/TURN_SPEED + VIEW_TIME + bad_accessible_penalty;
  }
  float move_angle = std::atan2(float(diff.y),float(diff.x));
  return (angleDist(curr_angle,move_angle)+angleDist(move_angle,angle))/TURN_SPEED + std::hypot(diff.x,diff.y)/MOVE_SPEED +
    VIEW_TIME + bad_accessible_penalty;
}


inline cv::Point2f poseToPoint(const tf::Transform& pose, cv::Point origin, float resolution){
  return cv::Point2f(pose.getOrigin().x()*resolution + origin.x, pose.getOrigin().y()*resolution + origin.y);
}


float Searcher::calcViewpointGain(const cv::Point& pos, int angle_step, const cv::Mat_<float> &prob_map, const cv::Point2f& curr_pos, float curr_angle){
  float angle = float(angle_step)/VIEW_ANGLE_STEPS*M_PI*2;
  float prob = 1.0;
  float interesting_border_seen = 0.f;
  int idx = -1;
  static int old_angle_step = -1;
  cv::Mat_<float> seen(prob_map.rows, prob_map.cols, 0.f);
  for(const auto& p : seen_kernel_points_[angle_step]){
    idx++;
    if(!(pos+p).inside(cv::Rect(0,0,prob_map.cols,prob_map.rows))){
      std::cout << "out" << std::endl;
      continue;
    }
    if(not_fully_viewed_border_(pos+p) == 255 && std::sqrt(p.x*p.x+p.y*p.y) < SEEN_MAP_MAX_DIST*RESOLUTION){
      float val = gaussian(angleDist(angle, border_dir_map_(pos+p)), BORDER_SEEN_SIGMA/2);
      interesting_border_seen += seen_kernel_points_value_[angle_step][idx]*val;
    }

    float val = gaussian(angleDist(border_dir_map_(pos+p), std::atan2(float(p.y),float(p.x))), BORDER_SEEN_SIGMA);
    prob *= (1.f-prob_map(pos+p)*seen_kernel_points_value_[angle_step][idx]*val);

//    if(old_angle_step != angle_step){
//      if(not_fully_viewed_border_(pos+p) > 0 && angleDist(angle, border_dir_map_(pos+p)) < BORDER_SEEN_SIGMA)
//        seen(pos+p) = 0.5 + 0.5*seen_kernel_points_value_[angle_step][idx];
//      else
//        seen(pos+p) = 0.2;
//    }
  }
  if(interesting_border_seen < 0.8f){
    return 0.f;
  }

//  if(old_angle_step != angle_step){
//    //old_angle_step = angle_step;
//    seen(pos) = 1.f;
//    showProbImage("seen" + std::to_string(pos.x) + " " + std::to_string(pos.y) + " " + std::to_string(angle_step) + " " + std::to_string(angle*180.0/M_PI), seen, 4, 192+not_fully_viewed_border_*0.25);
//    cv::waitKey();
//    cv::destroyWindow("seen" + std::to_string(pos.x) + " " + std::to_string(pos.y) + " " + std::to_string(angle_step) + " " + std::to_string(angle*180.0/M_PI));
//  }

//  std::cout << (1.f-prob + interesting_border_seen*INTERESTING_BORDER_SEEN_REWARD)/calcMoveTime(pos, angle, curr_pos, curr_angle)*100.f << " "
//            << 1.f-prob << " " << pos.x << " " << pos.y << " " << angle_step << " " << angle << " " << interesting_border_seen<< std::endl;
  return (1.f-prob + interesting_border_seen*INTERESTING_BORDER_SEEN_REWARD)/calcMoveTime(pos, angle, curr_pos, curr_angle);
}


bool Searcher::calcNextViewpoint(const tf::Transform& curr_pose, bool need_new_pose){
  cv::Point new_origin;
  cv::Mat_<float> prob_map = getProbMap(new_origin);

  std::cout << "m";
  cv::Mat_<float> tmp;
  prob_map.copyTo(tmp);
  double min_val, max_val;
  cv::minMaxLoc(tmp, &min_val, &max_val);
  double prob = 1.0;
  for(int x=0; x<prob_map.cols; x++){
    for(int y=0; y<prob_map.rows; y++){
      prob *= (1.0-prob_map(y,x));
      tmp(y,x) = tmp(y,x)/max_val;
    }
  }
  cv::putText(tmp, std::to_string(max_val).substr(0,5), cv::Point(0,20), cv::FONT_HERSHEY_PLAIN, 0.5, cv::Scalar(1.0));
  prob = 1-prob;
  std::cout << "Remaining prob: " << prob << std::endl;
  showProbImage("probabilities", tmp, 1, 255-accessible_map_*0.3);
  cv::waitKey(1);

  cv::Point2f curr_pos = poseToPoint(curr_pose, obj_map_->getOrigin(), obj_map_->getResolution());
  tf::Transform old_pose_tf;
  tf::poseMsgToTF(old_view_pose_, old_pose_tf);
  cv::Point2f old_pos = poseToPoint(old_pose_tf, obj_map_->getOrigin(), obj_map_->getResolution());
  cv::Point curr_point(curr_pos);
  float curr_angle = tf::getYaw(curr_pose.getRotation());
  int curr_step = int(curr_angle/(2*M_PI)*VIEW_ANGLE_STEPS+VIEW_ANGLE_STEPS)%VIEW_ANGLE_STEPS;

  double max = -1.0;
  int max_i;
  cv::Point max_loc;
  //cv::Mat_<uchar> seen_mat(prob_map.rows, prob_map.cols, uchar(0));
  std::vector<cv::Mat_<float>> prob_mats;
  cv::Mat_<uchar> accessible = accessible_map_;
  if(need_new_pose)
    cv::erode(accessible, accessible, cv::Mat_<uchar>::ones(3,3));
  for(int i=0; i<VIEW_ANGLE_STEPS; i++){
    std::cout << std::endl;
    prob_mats.push_back(cv::Mat_<float>(prob_map.rows, prob_map.cols,0.f));
    //cv::Mat_<float> sdf(prob_map.rows, prob_map.rows, 0.f);
    for(int x=0; x<prob_map.cols; x++){
      for(int y=0; y<prob_map.rows; y++){
        if(accessible(y,x) && !previous_pose_maps_[i](y,x) && (!need_new_pose || (std::abs(old_pos.x-x) > 1 || std::abs(old_pos.y-y) > 1)) &&
                ((std::abs(curr_point.x-x)>3 || std::abs(curr_point.y-y)>3 || (std::abs(curr_step-i)>1 && std::abs(curr_step-i) < VIEW_ANGLE_STEPS-2)))){
          float prob = calcViewpointGain(cv::Point(x,y), i, prob_map, curr_pos, curr_angle);
          //sdf(y,x) = prob;//*calcMoveTime(cv::Point(x,y), float(i)/VIEW_ANGLE_STEPS*M_PI*2, poseToPoint(curr_pose, obj_map_->getOrigin(), obj_map_->getResolution()), tf::getYaw(curr_pose.getRotation()));
          if(prob > max){
            max = prob;
            max_loc = cv::Point(x,y);
            max_i = i;
//            seen_mat = cv::Mat_<uchar>(prob_map.rows, prob_map.cols, uchar(0));
//            for(const auto& p : seen_kernel_points_[i]){
//              if(!(cv::Point(x,y)+p).inside(cv::Rect(0,0,prob_map.cols,prob_map.rows)))
//                continue;
//              seen_mat(cv::Point(x,y)+p) = 255;
//            }
          }
          prob_mats[i](y,x) = prob;
          if(prob < 0 || prob > 1)
            std::cout << "PROB OUT OF BOUNDS " << prob << std::endl;
        }
        //if(border_map_(y,x))
        //  sdf(y,x) = 1.f;
      }
    }
    //showProbImage(std::to_string(i), sdf, 2);
    //showProbImage(std::to_string(i), sdf, 2);
  }

//  for(int i=0; i<VIEW_ANGLE_STEPS; i++){
//    prob_mats[i] = prob_mats[i]/max;
//    showProbImage("prob_mat_"+std::to_string(i*30)+"_max_"+std::to_string(max), prob_mats[i], 2, accessible_map_);
//    cv::waitKey(0);
//  }

//  cv::resize(seen_mat, seen_mat, cv::Size(seen_mat.cols*2, seen_mat.rows*2), 0, 0, cv::INTER_NEAREST);
//  cv::flip(seen_mat,seen_mat,0);
//  cv::imshow("SEEEEEEEN", seen_mat);
//  cv::waitKey(1);

  if(max <= 0.0){
    return true;
  }

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
  if(!got_map_)
    return false;

  std::cout << "n";
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
        if(seen_maps_[border_idx](y,x) >= BORDER_SEEN_THRESH)
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
  for(int x=std::max(pos.x-3, 0); x<=std::min(pos.x+3,previous_pose_maps_[0].cols-1); x++){
    for(int y=std::max(pos.y-3, 0); y<=std::min(pos.y+3,previous_pose_maps_[0].rows-1); y++){
      previous_pose_maps_[angle/(2*M_PI)*VIEW_ANGLE_STEPS](y,x) = 255;
    }
  }

  cv::threshold(border_map_, tmp, 96, 96, cv::THRESH_TRUNC);
  cv::bitwise_or(tmp, not_fully_viewed_border_, tmp);
  cv::threshold(kernel, kernel, 48, 48, cv::THRESH_TRUNC);
  cv::bitwise_or(not_fully_viewed_border_(cv::Rect(x1,y1,kernel.cols,kernel.rows)), kernel, tmp(cv::Rect(x1,y1,kernel.cols,kernel.rows)));
  //cv::resize(tmp, tmp, cv::Size(not_fully_viewed_border_.cols*2, not_fully_viewed_border_.rows*2), 0, 0, cv::INTER_NEAREST);
  cv::flip(tmp, tmp, 0);
  cv::imshow("not_fully_viewed_border_", tmp);
  cv::moveWindow("not_fully_viewed_border_", 1600,300);
  cv::waitKey(1);
  std::cout << "In seen map inserted" << std::endl;

//  showProbImage("not_fully_viewed_border_dir", border_dir_map_/(2*M_PI), 4, not_fully_viewed_border_);
//  cv::waitKey(1);

  return (countNonZero(not_fully_viewed_border_) == 0);
}