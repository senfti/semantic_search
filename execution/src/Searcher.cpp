//
// Created by thomas on 02.01.18.
//

#include <execution/Searcher.h>
#include <pcl/common/common.h>
#include <semantic_mapping_v2/ObjectMapSrv.h>
#include <tf/transform_datatypes.h>

Searcher::Searcher(int searched_obj, int curr_room, tf::TransformListener *tf_listener)
  : searched_obj_(searched_obj), tf_listener_(tf_listener)
{
  ros::NodeHandle private_nh("~");
  private_nh.param("ObjectMapper/OBJ_PRIOR_PROB", OBJ_PRIOR_PROB, OBJ_PRIOR_PROB);
  private_nh.param("ObjectMapper/OBJ_MIN_PROB", OBJ_MIN_PROB, OBJ_MIN_PROB);
  private_nh.param("ObjectMapper/OBJ_MAX_PROB", OBJ_MAX_PROB, OBJ_MAX_PROB);
  private_nh.param("ObjectMapper/OBJ_DEFAULT_RESOLUTION", OBJ_DEFAULT_RESOLUTION, OBJ_DEFAULT_RESOLUTION);
  private_nh.param("ObjectMapper/OBJ_DEFUALT_MAX_HEIGHT", OBJ_DEFUALT_MAX_HEIGHT, OBJ_DEFUALT_MAX_HEIGHT);
  private_nh.param("ObjectMapper/V_H", V_H, V_H);
  private_nh.param("ObjectMapper/V_M", V_M, V_M);
  private_nh.param("ObjectMapper/ROOM_EXPECTED_SIZE", ROOM_EXPECTED_SIZE, ROOM_EXPECTED_SIZE);
  private_nh.param("Octomap/pointcloud_min_z", POINTCLOUD_MIN_Z,POINTCLOUD_MIN_Z);
  private_nh.param("Octomap/pointcloud_max_z", POINTCLOUD_MAX_Z,POINTCLOUD_MAX_Z);

  map_sub_ = ros::NodeHandle().subscribe("map_door_blocked", 1, &Searcher::mapCb, this);
  vision_sub_ = ros::NodeHandle().subscribe("vision_result", 1, &Searcher::visionCb, this);
  octo_mapper_ = new OctoMapper();


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
  obj_map_ = new ObjectMap(prior_prob_map_->getResolution(), 4.0, prior_prob_map_->getMaxHeight(), OBJ_PRIOR_PROB);

  octomap_pub_ = ros::NodeHandle().advertise<visualization_msgs::MarkerArray>("searcher_occ", 1, true);
}


Searcher::~Searcher(){
  delete obj_map_;
  delete octo_mapper_;
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

  accessible_mat_ = cv::Mat_<uchar>(msg->info.height, msg->info.width, uchar(0));
  std::deque<cv::Point> next[2];
  cv::Mat_<uchar> already_inserted;
  cv::bitwise_not(not_forbidden, already_inserted);
  already_inserted(start) = 255;
  int i=0;
  next[i].push_back(start);
  while(!next[i&1].empty()){
    for(const auto& p : next[i&1]){
      if(not_forbidden(p)){
        accessible_mat_(p) = 255;
      }
      insertNeighbors(p, already_inserted, next[(i+1)&1]);
    }
    next[i&1].clear();
    i++;
  }
  accessible_origin_ = cv::Point(-msg->info.origin.position.x/msg->info.resolution, -msg->info.origin.position.y/msg->info.resolution);
  accessible_resolution_ = msg->info.resolution;
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

  insertObject(*cloud, msg->objects);
  insertCloud(cloud, transform.getOrigin());
}


void Searcher::insertObject(const pcl::PointCloud<pcl::PointXYZ>& cloud, const vision::ObjectDetectionMsg& msg){
  pcl::PointXYZ min, max;
  pcl::getMinMax3D(cloud, min, max);
  int x1 = obj_map_->getXPixel(min.x);
  int x2 = obj_map_->getXPixel(max.x);
  int y1 = obj_map_->getYPixel(min.y);
  int y2 = obj_map_->getYPixel(max.y);

  int top=0, bottom=0, left=0, right=0, base_size = obj_map_->getBaseSize(), old_width=obj_map_->getWidth(), old_height=obj_map_->getHeight();
  while(x1+left < 0)
    left += base_size;
  while(x2 >= old_width+right)
    right += base_size;
  while(y1+top < 0)
    top += base_size;
  while(y2 >= old_height+bottom)
    bottom += base_size;

  if(top!=0 || bottom!=0 || left!=0 || right!=0){
    obj_map_->resize(left, right, top, bottom, OBJ_PRIOR_PROB);
  }

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
  float scale_2 = 1.f/(obj_map_->getResolution()*2);
  for(int x=0; x<obj_map_->getWidth(); x++){
    for(int y=0; y<obj_map_->getHeight(); y++){
      for(int z=0; z<obj_map_->getZSteps(); z++){
        geometry_msgs::Point p;
        p.x = obj_map_->getXWorld(x);
        p.y = obj_map_->getYWorld(y);
        p.z = obj_map_->getZWorld(z);
        max_prob = std::max(max_prob, obj_map_->getProb(x, y, z)*
          octo_mapper_->getOccupancy(p.x - scale_2, p.y - scale_2, p.z - scale_2, p.x + scale_2, p.y + scale_2, p.z + scale_2));
      }
    }
  }
  std::cout << "Max: " << max_prob << std::endl;
  octomap_pub_.publish(octo_mapper_->getOccupiedCellMsg(ros::Time::now()));
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


cv::Mat_<float> Searcher::getViewKernel(int angle_step) const{
  cv::Mat_<float> kernel(int(OBJ_DEFAULT_RESOLUTION*VIEW_MAX_DIST)*2+1, int(OBJ_DEFAULT_RESOLUTION*VIEW_MAX_DIST)*2+1, 0.f);
  float angle = angle_step/VIEW_ANGLE_STEPS*M_PI*2.f;
  std::vector<cv::Point> points;
  points.push_back(cv::Point(std::cos(angle-VIEW_ANGLE/2.f)*VIEW_MIN_DIST, std::sin(angle-VIEW_ANGLE/2.f)*VIEW_MIN_DIST));
  points.push_back(cv::Point(std::cos(angle-VIEW_ANGLE/2.f)*VIEW_MAX_DIST, std::sin(angle-VIEW_ANGLE/2.f)*VIEW_MAX_DIST));
  points.push_back(cv::Point(std::cos(angle+VIEW_ANGLE/2.f)*VIEW_MAX_DIST, std::sin(angle+VIEW_ANGLE/2.f)*VIEW_MAX_DIST));
  points.push_back(cv::Point(std::cos(angle+VIEW_ANGLE/2.f)*VIEW_MIN_DIST, std::sin(angle+VIEW_ANGLE/2.f)*VIEW_MIN_DIST));
  cv::fillConvexPoly(kernel, points, cv::Scalar(1.f));

  return kernel;
}


void makeSameSize(cv::Mat_<float>& resize_map, const cv::Point& resize_orig, const cv::Point& target_orig, int target_width, int target_height){
  if(resize_orig.x < target_orig.x)
    cv::copyMakeBorder(resize_map, resize_map, 0, 0, target_orig.x-resize_orig.x, 0, cv::BORDER_CONSTANT, 0.f);
  else if(resize_orig.x > target_orig.x)
    resize_map = resize_map(cv::Rect(resize_orig.x-target_orig.x, 0, resize_map.cols-(resize_orig.x-target_orig.x), resize_map.rows));

  if(resize_orig.y < target_orig.y)
    cv::copyMakeBorder(resize_map, resize_map, target_orig.y-resize_orig.y, 0, 0, 0, cv::BORDER_CONSTANT, 0.f);
  else if(resize_orig.y > target_orig.y)
    resize_map = resize_map(cv::Rect(resize_map.cols, resize_orig.y-target_orig.y, 0, resize_map.rows-(resize_orig.y-target_orig.y)));

  if(resize_map.cols < target_width)
    cv::copyMakeBorder(resize_map, resize_map, 0, 0, 0, target_width-resize_map.cols, cv::BORDER_CONSTANT, 0.f);
  else if(resize_map.cols > target_width)
    resize_map = resize_map(cv::Rect(0, 0, resize_map.cols, resize_map.rows));

  if(resize_map.rows < target_height)
    cv::copyMakeBorder(resize_map, resize_map, 0, target_height-resize_map.rows, 0, 0, cv::BORDER_CONSTANT, 0.f);
  else if(resize_map.cols > target_width)
    resize_map = resize_map(cv::Rect(0, 0, resize_map.cols, resize_map.rows));
}


inline float angleDist(float angle1, float angle2){
  float angle = angle1-angle2;
  if(angle > M_PI)
    return std::abs(angle-2*M_PI);
  if(angle <= M_PI)
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


void Searcher::calcNextViewpoint(const tf::Transform& curr_pose){
  cv::Point new_origin;
  cv::Mat_<float> prob_map = getProbMap(new_origin);
  float factor = 1.f/(accessible_resolution_*obj_map_->getResolution());
  cv::Point view_prob_origin = cv::Point(new_origin*factor);

  double max = 0.0;
  int max_i;
  cv::Point max_loc;
  for(int i=0; i<VIEW_ANGLE_STEPS; i++){
    cv::Mat_<float> view_probs;
    cv::Mat_<float> kernel = getViewKernel(i);
    cv::filter2D(prob_map, view_probs, -1, kernel, cv::Point(-1,-1), 0.0, cv::BORDER_REPLICATE);
    cv::resize(view_probs, view_probs, cv::Size(view_probs.cols*factor, view_probs.rows*factor));
    makeSameSize(view_probs, accessible_origin_, view_prob_origin, accessible_mat_.cols, accessible_mat_.rows);
    view_probs = view_probs.mul(accessible_mat_).mul(calcMoveTime(view_probs.cols, view_probs.rows, i,
                                                                  cv::Point(curr_pose.getOrigin().x(), curr_pose.getOrigin().y()), tf::getYaw(curr_pose.getRotation())));

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

  tf::Transform curr_view(tf::createQuaternionFromYaw(max_i/VIEW_ANGLE_STEPS*M_PI*2.f),
                          tf::Vector3((max_loc.x-view_prob_origin.x)*accessible_resolution_, (max_loc.y-view_prob_origin.y)*accessible_resolution_, 0.0));
  tf::poseTFToMsg(curr_view, curr_view_pose_);
}
