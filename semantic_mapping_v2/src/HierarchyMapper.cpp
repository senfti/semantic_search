//
// Created by thomas on 19.11.17.
//

#include "semantic_mapping_v2/HierarchyMapper.h"
#include <tf/transform_datatypes.h>
#include <semantic_mapping_v2/RoomSwitchMsg.h>
#include <std_msgs/Int8.h>
#include <semantic_mapping_v2/ObjFoundMsg.h>

HierarchyMapper::HierarchyMapper()
  : service_spinner_(1, &service_queue_)
{
  room_mapper_.reserve(64);
  addMapper(Door());

  laser_sub_ = nh_.subscribe("scan_filtered", 1, &HierarchyMapper::laserCallback, this);
  cloud_sub_ = nh_.subscribe("/camera/depth_registered/points", 1, &HierarchyMapper::cloudCb, this);
  door_pose_sub_ = nh_.subscribe("door_poses", 1, &HierarchyMapper::doorPoseCb, this);
  vision_sub_ = nh_.subscribe("vision_result", 1, &HierarchyMapper::visionCb, this);
  curr_action_sub_ = nh_.subscribe("current_action", 1, &HierarchyMapper::currActionCb, this);
  explored_sub_ = nh_.subscribe("room_explored", 1, &HierarchyMapper::exploredCb, this);

  map_pub_ = nh_.advertise<nav_msgs::OccupancyGrid>("map", 1, true);
  gmap_pub_ = nh_.advertise<nav_msgs::OccupancyGrid>("gmap", 1, true);
  map_switch_pub_ = nh_.advertise<semantic_mapping_v2::RoomSwitchMsg>("map_switch", 1);
  map_door_blocked_pub_ = nh_.advertise<nav_msgs::OccupancyGrid>("map_door_blocked", 1, true);
  map_info_pub_ = nh_.advertise<nav_msgs::MapMetaData>("map_metadata", 1, true);
  marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("occupied_cells_vis_array", 1, true);
  door_pose_pub_ = nh_.advertise<geometry_msgs::PoseArray>("mapper_door_poses", 1, true);
  particle_pose_pub_ = nh_.advertise<geometry_msgs::PoseArray>("particle_poses", 1, true);
  door_found_pub_ = nh_.advertise<std_msgs::Int8>("door_found", 1);
  obj_found_pub_ = nh_.advertise<semantic_mapping_v2::ObjFoundMsg>("obj_found", 1);
  obj_prob_map_view_pub_ = nh_.advertise<prob_map_view::ProbMapMsg>("obj_prob_map_view", 1);
  room_prob_map_view_pub_ = nh_.advertise<prob_map_view::ProbMapMsg>("room_prob_map_view", 1);
  base_obj_prob_map_view_pub_ = nh_.advertise<prob_map_view::ProbMapMsg>("base_obj_prob_map_view", 1);
  base_room_prob_map_view_pub_ = nh_.advertise<prob_map_view::ProbMapMsg>("base_room_prob_map_view", 1);
  sdf_prob_map_view_pub_ = nh_.advertise<prob_map_view::ProbMapMsg>("sdf", 1);
  hierarchy_pub_ = nh_.advertise<semantic_mapping_v2::HierarchySrvResponse>("hierarchy", 1);
  debug_output_map_pub_ = nh_.advertise<nav_msgs::OccupancyGrid>("debug_output_map", 1);
  debug_output_marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("debug_output_occupied_cells", 1);

  ros::NodeHandle service_nh;
  service_nh.setCallbackQueue(&service_queue_);
  gmap_srv_ = service_nh.advertiseService("gmap_srv", &HierarchyMapper::gmapSrvCb, this);
  map_srv_ = service_nh.advertiseService("map_srv", &HierarchyMapper::mapSrvCb, this);
  map_door_blocked_srv_ = service_nh.advertiseService("map_door_blocked_srv", &HierarchyMapper::mapDoorBlockedSrvCb, this);
  octomap_srv_ = service_nh.advertiseService("octomap_srv", &HierarchyMapper::octomapSrvCb, this);
  obj_map_srv_ = service_nh.advertiseService("obj_map_srv", &HierarchyMapper::objMapSrvCb, this);
  room_type_map_srv_ = service_nh.advertiseService("room_type_map_srv", &HierarchyMapper::roomTypeMapSrvCb, this);
  obj_prob_srv_ = service_nh.advertiseService("obj_prob_srv", &HierarchyMapper::objProbSrvCb, this);
  room_type_prob_srv_ = service_nh.advertiseService("room_type_prob_srv", &HierarchyMapper::roomTypeProbSrvCb, this);
  hierarchy_srv_ = service_nh.advertiseService("hierarchy_srv", &HierarchyMapper::hierarchySrvCb, this);
  obj_found_srv_ = service_nh.advertiseService("obj_found_srv", &HierarchyMapper::objFoundSrvCb, this);
  door_pose_srv_ = service_nh.advertiseService("door_pose_srv", &HierarchyMapper::doorPoseSrvCb, this);
  hierarchy_test_sub_ = service_nh.subscribe("hierarchy_test", 1, &HierarchyMapper::hierarchyTestCb, this);

  ros::NodeHandle("~").param("transform_publish_period", transform_publish_period_, 0.05);
  ros::NodeHandle("~").param("publish_period", publish_period_, 0.5);
  ros::NodeHandle("~").param("debug_publish_interval", debug_publish_interval_, debug_publish_interval_);
  ros::NodeHandle("~").param("ROOM_CELL_OBJ_KERNEL_SIZE", ROOM_CELL_OBJ_KERNEL_SIZE, ROOM_CELL_OBJ_KERNEL_SIZE);
  ros::NodeHandle("~").param("OBJ_BASED_ROOM_AREA_TO_CELL_CONFIDENCE", OBJ_BASED_ROOM_AREA_TO_CELL_CONFIDENCE, OBJ_BASED_ROOM_AREA_TO_CELL_CONFIDENCE);
  ros::NodeHandle("~").param("OBJ_FILL_FRACTION", OBJ_FILL_FRACTION, OBJ_FILL_FRACTION);
  ros::NodeHandle("~").param("ROOM_ESTIMATED_OCCUPIED_AREA", ROOM_ESTIMATED_OCCUPIED_AREA, ROOM_ESTIMATED_OCCUPIED_AREA);
  ros::NodeHandle("~").param("ROOM_MIN_UNEXPLORED_AREA", ROOM_MIN_UNEXPLORED_AREA, ROOM_MIN_UNEXPLORED_AREA);
  ros::NodeHandle("~").param("CELL_TO_OBJ_PROB_GAUSSIAN_SIGMA", CELL_TO_OBJ_PROB_GAUSSIAN_SIGMA, CELL_TO_OBJ_PROB_GAUSSIAN_SIGMA);
  ros::NodeHandle("~").param("SINGLE_VIEW_OBJ_KERNEL_SIZE", SINGLE_VIEW_OBJ_KERNEL_SIZE, SINGLE_VIEW_OBJ_KERNEL_SIZE);
  ros::NodeHandle("~").param("TRAVEL_DIST_LIN_FACTOR", TRAVEL_DIST_LIN_FACTOR, TRAVEL_DIST_LIN_FACTOR);
  ros::NodeHandle("~").param("TRAVEL_DIST_QUAD_FACTOR", TRAVEL_DIST_QUAD_FACTOR, TRAVEL_DIST_QUAD_FACTOR);
  ros::NodeHandle("~").param("TRAVEL_TURN_FACTOR", TRAVEL_TURN_FACTOR, TRAVEL_TURN_FACTOR);
  ros::NodeHandle("~").param("SEARCH_TIME_PER_GRID_CELL", SEARCH_TIME_PER_GRID_CELL, SEARCH_TIME_PER_GRID_CELL);
  ros::NodeHandle("~").param("PUBLISH_DEBUG_IMAGES", PUBLISH_DEBUG_IMAGES,PUBLISH_DEBUG_IMAGES);
  ros::NodeHandle("~").param("DEBUG_OUTPUT", DEBUG_OUTPUT, DEBUG_OUTPUT);

  tfB_ = new tf::TransformBroadcaster();
  transform_thread_ = new boost::thread(boost::bind(&HierarchyMapper::transformPublishLoop, this, transform_publish_period_));
  service_spinner_.start();

  std::cout << "HierarchyMapping started" << std::endl;
}

HierarchyMapper::~HierarchyMapper(){
  service_spinner_.stop();
  transform_thread_->join();
  delete transform_thread_;
  for(auto& mapper : room_mapper_)
    delete mapper;
  delete tfB_;

  ROS_ERROR("THIS IS THE END");
  std::cout << "THIS IS THE END" << std::endl;
}


void HierarchyMapper::addMapper(const Door& door){
  //ROS_INFO("Added MAPPER START");
  boost::unique_lock<boost::mutex> lock(mapper_mutex_);
  if(door.isValid()){
    tf::StampedTransform transform;
    try{
      tf_listener_.lookupTransform("map", "base_laser_link", ros::Time(0), transform);
    }
    catch (tf::TransformException ex){
      ROS_ERROR("%s",ex.what());
    }
    room_mapper_.push_back(new RoomMapper(room_mapper_.size(), &tf_listener_,
                                          GMapping::OrientedPoint(transform.getOrigin().x(),transform.getOrigin().y(),tf::getYaw(transform.getRotation())),
                                          room_mapper_[current_mapper_]->getTransform(), door));
    explored_.push_back(false);
  }
  else{
    tf::StampedTransform transform;
    try{
      tf_listener_.waitForTransform("base_laser_link", "odom", ros::Time::now(), ros::Duration(2.0));
      tf_listener_.lookupTransform("base_laser_link", "odom", ros::Time::now(), transform);
    }
    catch (tf::TransformException ex){
      ROS_ERROR("%s",ex.what());
    }
    room_mapper_.push_back(new RoomMapper(room_mapper_.size(), &tf_listener_, GMapping::OrientedPoint(0,0,0), transform, door));
    explored_.push_back(false);
  }
  last_room_msgs_.push_back(semantic_mapping_v2::RoomMsg());
  room_changed_.push_back(true);
  ROS_INFO("Added MAPPER %d", int(room_mapper_.size()) - 1);
  lock.unlock();
  switchMapper(room_mapper_.size() - 1);
}


void HierarchyMapper::switchMapper(int mapper_idx, const Door& door){
  //ROS_INFO("switch MAPPER START");
  int old_mapper = current_mapper_;
  current_mapper_ = -1;
  if(old_mapper >= 0 && old_mapper < room_mapper_.size()){
    room_mapper_[old_mapper]->deactivate();
    room_changed_[old_mapper] = true;
  }

  tf::Transform door_tf(tf::Quaternion(0.0,0.0,0.0,1.0), tf::Vector3(0.0,0.0,0.0));
  if(mapper_idx >= 0 && mapper_idx < room_mapper_.size()){
    if(door.isValid()){
      tf::StampedTransform transform;
      try{
        tf_listener_.lookupTransform("map", "base_laser_link", ros::Time(0), transform);
      }
      catch (tf::TransformException ex){
        ROS_ERROR("%s",ex.what());
      }
      GMapping::OrientedPoint p(transform.getOrigin().x(), transform.getOrigin().y(), tf::getYaw(transform.getRotation()));
      current_mapper_ = mapper_idx;
      room_changed_[current_mapper_] = true;
      door_tf = room_mapper_[current_mapper_]->activate(p, door);
    }
    else{
      current_mapper_ = mapper_idx;
      room_mapper_[current_mapper_]->activate();
    }
  }

  semantic_mapping_v2::RoomSwitchMsg msg;
  msg.new_room = current_mapper_;
  tf::poseTFToMsg(door_tf, msg.transform);
  if(!map_switch_pub_.getTopic().empty())
    map_switch_pub_.publish(msg);

  last_map_switch_time_ = ros::Time::now();
  ROS_INFO("Switched to MAPPER %d", current_mapper_);
}


void HierarchyMapper::cloudCb(const sensor_msgs::PointCloud2::ConstPtr& cloud){
  //ROS_INFO("cloud START");
  if(current_mapper_ >= 0 && current_mapper_ < room_mapper_.size()){
    room_mapper_[current_mapper_]->cloudCb(cloud);
    room_changed_[current_mapper_] = true;
  }
  //ROS_INFO("ce");
}


void HierarchyMapper::laserCallback(const sensor_msgs::LaserScan::ConstPtr& scan){
  //ROS_INFO("ls");
  if(current_mapper_ >= 0 && current_mapper_ < room_mapper_.size()){
    room_mapper_[current_mapper_]->laserCallback(scan);
    room_changed_[current_mapper_] = true;
  }
  //ROS_INFO("le");
}


void HierarchyMapper::doorPoseCb(const geometry_msgs::PoseArray::ConstPtr &msg){
  //ROS_INFO("ds");
  if(current_mapper_ >= 0 && current_mapper_ < room_mapper_.size()){
    if(room_mapper_[current_mapper_]->doorCb(msg))
      door_found_pub_.publish(std_msgs::Int8());
    room_changed_[current_mapper_] = true;
  }
  //ROS_INFO("de");
}


void HierarchyMapper::visionCb(const vision::VisionMsgConstPtr &msg){
  //ROS_INFO("vs");
  if(current_mapper_ >= 0 && current_mapper_ < room_mapper_.size()){
    room_mapper_[current_mapper_]->visionCb(msg);
    room_changed_[current_mapper_] = true;
  }
  //ROS_INFO("ve");
}


void HierarchyMapper::currActionCb(const std_msgs::Int8ConstPtr& msg){
  curr_action_ = msg->data;
}


void HierarchyMapper::exploredCb(const std_msgs::Int8ConstPtr& msg){
  if(msg->data >= 0 && msg->data < explored_.size())
    explored_[msg->data] = true;
}


void HierarchyMapper::transformPublishLoop(double transform_publish_period){
  if(transform_publish_period == 0)
    return;

  ros::Rate r(1.0 / transform_publish_period);
  while(ros::ok()){
    int i = current_mapper_;
    if(i >= 0 && i < room_mapper_.size()){
      tfB_->sendTransform(room_mapper_[i]->getTransform());
    }
    r.sleep();
  }
}


void HierarchyMapper::downprojecAndPublish(){
  //ROS_INFO("pubs");
  ros::Time start = ros::Time::now();
  room_mapper_[current_mapper_]->downprojectMap();
  nav_msgs::OccupancyGrid map = room_mapper_[current_mapper_]->getGMap();
  if(map.data.size() > 0){
    gmap_pub_.publish(map);
  }
  map = room_mapper_[current_mapper_]->getMap();
  nav_msgs::OccupancyGrid map_blocked = room_mapper_[current_mapper_]->getDoorBlockedMap();
  if(curr_action_ == 1 || curr_action_ == 2 || curr_action_ == 3){
    if(map_blocked.data.size() > 0){
      map_pub_.publish(map_blocked);
      map_info_pub_.publish(map_blocked.info);
      map_door_blocked_pub_.publish(map_blocked);
    }
  }
  else{
    if(map.data.size() > 0){
      map_pub_.publish(map);
      map_info_pub_.publish(map.info);
    }
    if(map_blocked.data.size() > 0){
      map_door_blocked_pub_.publish(map_blocked);
    }
  }
  if(DEBUG_OUTPUT)
    marker_pub_.publish(room_mapper_[current_mapper_]->getOccupiedCellMsg());
  particle_pose_pub_.publish(room_mapper_[current_mapper_]->getParticlePoseMsg());
  door_pose_pub_.publish(room_mapper_[current_mapper_]->getDoorPoseMsg());

  semantic_mapping_v2::ObjFoundMsg obj_found_msg;
  std::vector<std::pair<geometry_msgs::Pose, float>> tmp = room_mapper_[current_mapper_]->getObjectMax();
  obj_found_msg.poses.resize(tmp.size());
  obj_found_msg.probs.resize(tmp.size());
  for(int i=0; i<tmp.size(); i++){
    obj_found_msg.poses[i] = tmp[i].first;
    obj_found_msg.probs[i] = tmp[i].second;
  }
  obj_found_pub_.publish(obj_found_msg);

//  static int counter=0;
//  if((counter%debug_publish_interval_) == debug_publish_interval_-1){
//    ;
//    if(base_obj_prob_pub_.empty()){
//      base_obj_prob_pub_.resize(80);
//      for(int i=0; i<80; i++){
//        std::string s = "base_obj_" + ObjectMapper::getObjName(i);
//        std::replace( s.begin(), s.end(), ' ', '_');
//        base_obj_prob_pub_[i] = nh_.advertise<visualization_msgs::MarkerArray>(s, 1, true);
//      }
//    }
//    for(int i=0; i<80; i++)
//      base_obj_prob_pub_[i].publish(room_mapper_[current_mapper_]->getObjectProbMsg(i));
//
//    if(base_room_prob_pub_.empty() && !room_mapper_[current_mapper_]->getRoomName(1).empty()){
//      int num = room_mapper_[current_mapper_]->getRoomNames().size();
//      base_room_prob_pub_.resize(num);
//      for(int i=0; i<num; i++){
//        std::string s = "base_room_" + room_mapper_[current_mapper_]->getRoomName(i);
//        std::replace( s.begin(), s.end(), ' ', '_');
//        base_room_prob_pub_[i] = nh_.advertise<visualization_msgs::MarkerArray>(s, 1, true);
//      }
//    }
//    if(!base_room_prob_pub_.empty())
//      for(int i=0; i<room_mapper_[current_mapper_]->getRoomNames().size(); i++)
//        base_room_prob_pub_[i].publish(room_mapper_[current_mapper_]->getRoomProbMsg(i));
//
//    std::vector<size_t> order;
//    std::vector<float> probs = room_mapper_[current_mapper_]->getRoomTypeProbs(order);
//    if(!probs.empty()){
//      for(int i = 0; i < probs.size(); i++){
//        std::cout << room_mapper_[current_mapper_]->getRoomName(order[i]) << ": " << probs[order[i]] << std::endl;
//      }
//    }
//    std::cout << std::endl;
//    probs = room_mapper_[current_mapper_]->getObjectProbs(order);
//    for(int i=0; i<probs.size(); i++)
//      std::cout << ObjectMapper::getObjName(order[i]) << ": " << probs[order[i]] << std::endl;
//  }
//  counter++;

  std::cout << "Published in " << (ros::Time::now() - start).toSec() << std::endl;
}


void HierarchyMapper::publishRoomTypeProbMap(const RoomTypeMap &map, int idx){
  if(room_prob_pub_.empty() && !room_mapper_[current_mapper_]->getRoomName(1).empty()){
    int num = room_mapper_[current_mapper_]->getRoomNames().size();
    room_prob_pub_.resize(num);
    for(int i=0; i<num; i++){
      std::string s = "room_" + room_mapper_[current_mapper_]->getRoomName(i);
      std::replace( s.begin(), s.end(), ' ', '_');
      room_prob_pub_[i] = nh_.advertise<visualization_msgs::MarkerArray>(s, 1, true);
    }
  }
  if(!room_prob_pub_.empty())
    room_prob_pub_[idx].publish(map.getProbMsg());
}


void HierarchyMapper::publishObjProbMap(const ObjectMap &map, int idx){
  if(obj_prob_pub_.empty()){
    obj_prob_pub_.resize(80);
    for(int i=0; i<80; i++){
      std::string s = "obj_" + ObjectMapper::getObjName(i);
      std::replace( s.begin(), s.end(), ' ', '_');
      obj_prob_pub_[i] = nh_.advertise<visualization_msgs::MarkerArray>(s, 1, true);
    }
  }
  obj_prob_pub_[idx].publish(map.getProbMsg(0, 0.01));
}


void HierarchyMapper::run(){
  ros::Rate rate(50);
  ros::Time last_publish = ros::Time::now();
  while(ros::ok()){
    ros::spinOnce();
    if(current_mapper_ >= 0 && current_mapper_ < room_mapper_.size() && room_mapper_[current_mapper_]->isInitialized()){
        if(ros::Time::now() - last_publish > ros::Duration(publish_period_)){
          downprojecAndPublish();
          last_publish = ros::Time::now();
        }

      Door door = room_mapper_[current_mapper_]->droveThroughDoor();
      if(door.isValid() && (ros::Time::now() - last_map_switch_time_).toSec() > MIN_MAP_SWITCH_TIME){
        ROS_INFO("Door %d, c_id: %d, r: %d, c_r: %d", door.getId(), door.getCounterpartId(), door.getRoom(), door.getOtherRoom());
        int other_room = door.getOtherRoom();
        if(other_room < 0){
          int new_id = Door::getID();
          room_mapper_[current_mapper_]->setDoorRoom(door.getId(), room_mapper_.size(), new_id);

          Door counterpart_door(room_mapper_.size(), door.getPose(), current_mapper_, new_id, door.getId(), 5);
          counterpart_door.flipPose();
          ROS_INFO("New Door %d, c_id: %d, r: %d, c_r: %d", counterpart_door.getId(), counterpart_door.getCounterpartId(), counterpart_door.getRoom(), counterpart_door.getOtherRoom());
          addMapper(counterpart_door);
        }
        else
          switchMapper(other_room, door);
      }
    }
    rate.sleep();
  }
  ROS_WARN("OUT OF MAIN LOOP NOW");
  std::cout << "OUT OF MAIN LOOP NOW" << std::endl;
}


bool HierarchyMapper::gmapSrvCb(semantic_mapping_v2::MapSrv::Request& req, semantic_mapping_v2::MapSrv::Response& res){
  ROS_INFO("SERVICE GMAP");
  boost::lock_guard<boost::mutex> maps_lock(mapper_mutex_);
  int id = (req.room_id >= 0 ? req.room_id : current_mapper_);
  if(id < 0 || id >= room_mapper_.size())
    return false;

  res.map = room_mapper_[id]->getGMap();
  return true;
}


bool HierarchyMapper::mapSrvCb(semantic_mapping_v2::MapSrv::Request& req, semantic_mapping_v2::MapSrv::Response& res){
  ROS_INFO("SERVICE MAP");
  boost::lock_guard<boost::mutex> maps_lock(mapper_mutex_);
  int id = (req.room_id >= 0 ? req.room_id : current_mapper_);
  if(id < 0 || id >= room_mapper_.size())
    return false;

  res.map = room_mapper_[id]->getMap();
  return true;
}


bool HierarchyMapper::mapDoorBlockedSrvCb(semantic_mapping_v2::MapSrv::Request &req, semantic_mapping_v2::MapSrv::Response &res){
  ROS_INFO("SERVICE MAP DOOR BLOCKED");
  boost::lock_guard<boost::mutex> maps_lock(mapper_mutex_);
  int id = (req.room_id >= 0 ? req.room_id : current_mapper_);
  if(id < 0 || id >= room_mapper_.size())
    return false;

  res.map = room_mapper_[id]->getDoorBlockedMap();
  return true;
}


bool HierarchyMapper::octomapSrvCb(semantic_mapping_v2::OctomapSrv::Request& req, semantic_mapping_v2::OctomapSrv::Response& res){
  ROS_INFO("SERVICE OCTOMAP");
  boost::lock_guard<boost::mutex> maps_lock(mapper_mutex_);
  int id = (req.room_id >= 0 ? req.room_id : current_mapper_);
  if(id < 0 || id >= room_mapper_.size())
    return false;

  res.map = room_mapper_[id]->getFullOctoMapMsg();
  return true;
}


bool HierarchyMapper::roomTypeMapSrvCb(semantic_mapping_v2::RoomTypeMapSrv::Request& req, semantic_mapping_v2::RoomTypeMapSrv::Response& res){
  ROS_INFO("SERVICE ROOM_MAP");
  boost::lock_guard<boost::mutex> maps_lock(mapper_mutex_);
  int id = (req.room_id >= 0 ? req.room_id : current_mapper_);
  if(id < 0 || id >= room_mapper_.size())
    return false;

  if(req.id < 0)
    res.maps = room_mapper_[id]->getAllRoomTypeMapMsgs();
  else
    res.maps = std::vector<semantic_mapping_v2::RoomTypeMapMsg>({room_mapper_[id]->getRoomTypeMapMsg(req.id)});

  return true;
}


bool HierarchyMapper::roomTypeProbSrvCb(semantic_mapping_v2::RoomTypeProbSrv::Request& req, semantic_mapping_v2::RoomTypeProbSrv::Response& res){
  ROS_INFO("SERVICE ROOM_PROB");
  boost::lock_guard<boost::mutex> maps_lock(mapper_mutex_);
  int id = (req.room_id >= 0 ? req.room_id : current_mapper_);
  if(id < 0 || id >= room_mapper_.size())
    return false;

  std::vector<size_t> order;
  res.probs = room_mapper_[id]->getRoomTypeProbs(order);
  res.names = room_mapper_[id]->getRoomNames();

  return true;
}


bool HierarchyMapper::objProbSrvCb(semantic_mapping_v2::ObjectProbSrv::Request& req, semantic_mapping_v2::ObjectProbSrv::Response& res){
  ROS_INFO("SERVICE OBJ_PROB");
  boost::lock_guard<boost::mutex> maps_lock(mapper_mutex_);
  int id = (req.room_id >= 0 ? req.room_id : current_mapper_);
  if(id < 0 || id >= room_mapper_.size())
    return false;

  std::vector<size_t> order;
  res.probs = room_mapper_[id]->getObjectProbs(order);
  res.names = ObjectMapper::getObjNames();

  return true;
}


cv::Mat_<float> HierarchyMapper::getBehindDoorMask(const std::vector<Door>& doors, const ObjectMap& obj_map){
  cv::Mat_<float> behind_door_mask(obj_map.getHeight(),obj_map.getWidth(), 0.f);
  for(int x=0; x<behind_door_mask.cols; x++){
    for(int y=0; y<behind_door_mask.rows; y++){
      for(const auto& d : doors){
        if(d.isBehindDoor(obj_map.getXWorld(x),obj_map.getYWorld(y)))
          behind_door_mask(y,x) = 1.f;
      }
    }
  }
  return behind_door_mask;
}


std::vector<cv::Mat_<float>> HierarchyMapper::get2dAreaObjProbMaps(const std::vector<ObjectMap>& obj_maps, const cv::Mat_<float> behind_door_mask, const ObjectMap& occ_map,
                                                  const cv::Point& room_origin, int room_width, int room_height, float kernel_size)
{
  std::vector<cv::Mat_<float>> prob_2d_area;
  for(int o=0; o<obj_maps.size(); o++){
    cv::Mat_<float> prob_2d = 1.f - obj_maps[o].get2D(behind_door_mask, occ_map);
    int k_size = kernel_size/2.f*obj_maps[0].getResolution();
    cv::GaussianBlur(prob_2d, prob_2d, cv::Size(k_size*2+1, k_size*2+1), k_size/3.f, k_size/3.f);
    if(obj_maps[0].getOrigin() != room_origin)
      cv::copyMakeBorder(prob_2d, prob_2d, room_origin.y - obj_maps[0].getOrigin().y, 0, room_origin.x - obj_maps[0].getOrigin().x, 0, cv::BORDER_CONSTANT, cv::Scalar(1.f));
    if(prob_2d.cols != room_width || prob_2d.rows != room_height)
      cv::copyMakeBorder(prob_2d, prob_2d, 0, room_height - prob_2d.rows, 0, room_width - prob_2d.cols, cv::BORDER_CONSTANT, cv::Scalar(1.f));

    prob_2d_area.push_back(cv::Mat_<float>(prob_2d.rows, prob_2d.cols, 1.f));
    cv::copyMakeBorder(prob_2d, prob_2d, k_size, k_size, k_size, k_size, cv::BORDER_CONSTANT, cv::Scalar(1.f));
    for(int x=k_size; x<prob_2d.cols-k_size; x++){
      for(int y=k_size; y<prob_2d.rows-k_size; y++){
        for(int xk=x-k_size; xk<=x+k_size; xk++){
          for(int yk=y-k_size; yk<=y+k_size; yk++){
            prob_2d_area[o](y-k_size,x-k_size) *= prob_2d(yk,xk);
          }
        }
      }
    }
    prob_2d_area[o] = 1.f-prob_2d_area[o];
  }
  return prob_2d_area;
}


std::vector<cv::Mat_<float>> HierarchyMapper::getObjBasedRoomTypeMap(const std::vector<cv::Mat_<float>>& obj_prob_2d_area, int num_room_types){
  std::vector<cv::Mat_<float>> room_type_map;
  cv::Mat_<float> max_mat(obj_prob_2d_area[0].rows, obj_prob_2d_area[0].cols, -99999999999.9f);
  for(int r=0; r<num_room_types; r++){
    room_type_map.push_back(cv::Mat_<float>(obj_prob_2d_area[0].rows, obj_prob_2d_area[0].cols, 0.f));
    for(int o=0; o<obj_prob_2d_area.size(); o++){
      float or_prob = RoomMapper::getObjProbGivenRoom(o,r);
      float o_prob = RoomMapper::getObjProbGivenRoomObjPrior(o);
      float N = num_room_types;
      cv::Mat_<float> tmp;
      cv::log(obj_prob_2d_area[o]*or_prob/N/(o_prob/N) + (1.f-obj_prob_2d_area[o])*(1.f-or_prob)/N/(1.f-o_prob/N), tmp);
      room_type_map[r] += tmp;
    }
    max_mat = cv::max(max_mat, room_type_map[r]);
  }
  cv::Mat_<float> sum_mat(room_type_map[0].rows, room_type_map[0].cols, 0.f);
  for(int r=0; r<room_type_map.size(); r++){
    cv::exp(room_type_map[r]-max_mat, room_type_map[r]);
    room_type_map[r] = room_type_map[r]*OBJ_BASED_ROOM_AREA_TO_CELL_CONFIDENCE+(1.f-room_type_map[r]);
    sum_mat += room_type_map[r];
  }
  for(int r=0; r<room_type_map.size(); r++){
    cv::divide(room_type_map[r], sum_mat, room_type_map[r]);
  }
  return room_type_map;
}


std::vector<cv::Mat_<float>> HierarchyMapper::getCompleteRoomTypeMap(const std::vector<RoomTypeMap>& room_type_map, const std::vector<cv::Mat_<float>>& obj_based_room_type_map){
  std::vector<cv::Mat_<float>> complete_room_type_map;
  cv::Mat_<float> sum(room_type_map[0].getHeight(), room_type_map[0].getWidth(), 0.f);
  for(int i=0; i<room_type_map.size(); i++){
    complete_room_type_map.push_back(room_type_map[i].getMap().mul(obj_based_room_type_map[i]));
    sum += complete_room_type_map[i];
  }
  for(int i=0; i<room_type_map.size(); i++){
    cv::divide(complete_room_type_map[i], sum, complete_room_type_map[i]);
  }
  return complete_room_type_map;
}


std::vector<float> HierarchyMapper::getRoomTypeCellNumberEstimate(const std::vector<cv::Mat_<float>>& complete_room_type_map, const cv::Mat_<uchar>& seen_map, const cv::Point& origin,
                                    const nav_msgs::OccupancyGrid& grid_map, const std::vector<Door>& doors, float resolution, int base_size)
{
  RoomTypeMapper tmp_mapper(complete_room_type_map, seen_map, origin, resolution, base_size);
  return tmp_mapper.getTypeCellNumberEstimate(grid_map,doors);
}


inline double getObjProbGivenRoomPerCell(int o, int r, double object_in_cell_fraction){
  return 1.f-std::pow(1.0-RoomMapper::getObjProbGivenRoom(o, r), object_in_cell_fraction);
}
inline double getObjProbPriorPerCell(int o, int r, double object_in_cell_fraction){
  return 1.f-std::pow(1.0-RoomMapper::getObjProbGivenRoomObjPrior(o), object_in_cell_fraction);
}

void showProbImage(const std::string& name, const cv::Mat mat, float resize_factor){
  cv::Mat tmp;
  cv::resize(mat, tmp, cv::Size(mat.cols*resize_factor, mat.rows*resize_factor));
  double min, max;
  cv::minMaxIdx(tmp, &min, &max);
  std::cout << "minmax " << min << " " << max << std::endl;
  cv::flip(tmp, tmp, 0);

  tmp.convertTo(tmp, CV_8U, 150.f);

  cv::Mat o(tmp.rows, tmp.cols, CV_8UC1, cv::Scalar(255));
  cv::merge(std::vector<cv::Mat>({tmp, o, o}), tmp);
  cv::cvtColor(tmp, tmp, CV_HSV2BGR);
  cv::imshow(name, tmp);
}

std::vector<cv::Mat_<float>> HierarchyMapper::getRoomBasedObjMap(const std::vector<cv::Mat_<float>>& complete_room_type_map,
                                                                 const cv::Point& new_orig, const cv::Size& new_size, int num_obj_types, int obj)
{
  std::vector<cv::Mat_<float> > obj_from_room_maps;
  if(obj >= 0){
    int o = obj;
    obj_from_room_maps.push_back(cv::Mat_<float>(complete_room_type_map[o].rows, complete_room_type_map[o].cols, 0.f));
    cv::Mat_<float> inv = cv::Mat_<float>(complete_room_type_map[o].rows, complete_room_type_map[o].cols, 0.f);
    for(int r = 0; r < complete_room_type_map.size(); r++){
      float or_prob = RoomMapper::getObjProbGivenRoomPerCell(o,r);
      float o_prob = RoomMapper::getObjProbGivenRoomObjPriorPerCell(o);
      obj_from_room_maps.back() += complete_room_type_map[r] * or_prob / o_prob;
      inv += complete_room_type_map[r] * (1.0 - or_prob) / (1.0-o_prob);
    }
    cv::divide(obj_from_room_maps.back(), obj_from_room_maps.back() + inv, obj_from_room_maps.back());
    int k_size = int(CELL_TO_OBJ_PROB_GAUSSIAN_SIGMA*3)*2+1;
    cv::GaussianBlur(obj_from_room_maps.back(), obj_from_room_maps.back(), cv::Size(k_size,k_size), CELL_TO_OBJ_PROB_GAUSSIAN_SIGMA, CELL_TO_OBJ_PROB_GAUSSIAN_SIGMA, cv::BORDER_REPLICATE);
    obj_from_room_maps.back() = obj_from_room_maps.back()(cv::Rect(new_orig.x, new_orig.y, new_size.width, new_size.height));
  }
  else{
    for(int o = 0; o < num_obj_types; o++){
      obj_from_room_maps.push_back(cv::Mat_<float>(complete_room_type_map[o].rows, complete_room_type_map[o].cols, 0.f));
      cv::Mat_<float> inv = cv::Mat_<float>(complete_room_type_map[o].rows, complete_room_type_map[o].cols, 0.f);
      for(int r = 0; r < complete_room_type_map.size(); r++){
        float or_prob = RoomMapper::getObjProbGivenRoomPerCell(o,r);
        float o_prob = RoomMapper::getObjProbGivenRoomObjPriorPerCell(o);
        obj_from_room_maps.back() += complete_room_type_map[r] * or_prob / o_prob;
        inv += complete_room_type_map[r] * (1.0 - or_prob) / (1.0-o_prob);
      }
      cv::divide(obj_from_room_maps.back(), obj_from_room_maps.back() + inv, obj_from_room_maps.back());
      int k_size = int(CELL_TO_OBJ_PROB_GAUSSIAN_SIGMA*3)*2+1;
      cv::GaussianBlur(obj_from_room_maps.back(), obj_from_room_maps.back(), cv::Size(k_size,k_size), CELL_TO_OBJ_PROB_GAUSSIAN_SIGMA, CELL_TO_OBJ_PROB_GAUSSIAN_SIGMA, cv::BORDER_REPLICATE);
      obj_from_room_maps.back() = obj_from_room_maps.back()(cv::Rect(new_orig.x, new_orig.y, new_size.width, new_size.height));
    }
  }
  return obj_from_room_maps;
}

std::vector<cv::Point> HierarchyMapper::getOnlyLaserPoints(const ObjectMap& obj_map, const nav_msgs::OccupancyGrid& map, const cv::Mat_<float> behind_door_mask){
  std::vector<cv::Point> points;
  cv::Mat_<uchar> mask(map.info.height, map.info.width, uchar(0));
  for(int x=0; x<mask.cols; x++){
    for(int y=0; y<mask.rows; y++){
      if(map.data[y * map.info.width + x] > 0)
        mask(y,x) = 255;
    }
  }
  cv::dilate(mask, mask, cv::Mat_<uchar>::ones(3,3), cv::Point(-1,-1), 3);
  cv::Mat_<uchar> map2d = obj_map.getCount2D(behind_door_mask);

  double res = 1.00 / map.info.resolution;
  for(int x=0; x<obj_map.getWidth(); x++){
    for(int y=0; y<obj_map.getHeight(); y++){
      float x_world = obj_map.getXWorld(x);
      float y_world = obj_map.getYWorld(y);
      int x_map = ((x_world - map.info.origin.position.x)) * res;
      int y_map = ((y_world - map.info.origin.position.y)) * res;
      if(map2d(y,x) < 1 && x_map >= 0 && x_map < mask.cols && y_map >= 0 && y_map < mask.rows && mask(y_map, x_map) > 0 && behind_door_mask(y,x) < 0.5f){
        points.push_back(cv::Point(x,y));
      }
    }
  }

  return points;
}

std::vector<ObjectMap> HierarchyMapper::getCompleteObjMap(const std::vector<cv::Mat_<float>>& room_base_obj_maps, const std::vector<ObjectMap>& obj_map, const ObjectMap& occ_map,
                                                          const std::vector<cv::Mat_<float>>& room_type_probs_maps, const cv::Mat_<uchar>& room_seen_map, const cv::Point& new_orig, const cv::Size& new_size,
                                                          const std::vector<cv::Point>& only_laser_points, const std::vector<float>& room_type_probs, const cv::Mat_<uchar>& occ_2d, int obj)
{
  std::vector<cv::Mat_<float>> room_type_maps_filtered;
  for(int i=0; i<room_type_probs_maps.size(); i++){
    room_type_maps_filtered.push_back(cv::Mat_<float>(room_type_probs_maps[i].rows, room_type_probs_maps[i].cols, room_type_probs[i]));
    room_type_probs_maps[i].copyTo(room_type_maps_filtered[i], room_seen_map);
    cv::GaussianBlur(room_type_maps_filtered[i], room_type_maps_filtered[i], cv::Size(13,13), 4.0, 4.0);
    room_type_maps_filtered[i] = room_type_maps_filtered[i](cv::Rect(new_orig.x, new_orig.y, new_size.width, new_size.height));
  }

  std::vector<ObjectMap> complete_obj_map;
  if(obj >= 0){
    complete_obj_map.push_back(ObjectMap(obj_map[obj], occ_map, room_base_obj_maps[0], only_laser_points, room_type_maps_filtered, obj, occ_2d));
  }
  else{
    for(int o=0; o<obj_map.size(); o++){
      complete_obj_map.push_back(ObjectMap(obj_map[o], occ_map, room_base_obj_maps[o], only_laser_points, room_type_maps_filtered, o, occ_2d));
    }
  }
  return complete_obj_map;
}


std::vector<float> HierarchyMapper::getCompleteObjProbs(const std::vector<ObjectMap>& complete_obj_map, std::vector<float> room_type_cell_number, const ObjectMap& occ_map, int unseen_estimate){
  std::vector<float> complete_obj_probs(complete_obj_map.size());
  for(int o=0; o<complete_obj_map.size(); o++){
    double unseen_prob_estimate = 1.f;
    for(int r = 0; r < room_type_cell_number.size(); r++){
      unseen_prob_estimate *= std::pow(1.0-RoomMapper::getObjProbGivenRoomPerCell(o,r),room_type_cell_number[r]*unseen_estimate);
    }
    complete_obj_probs[o] = std::min(1.0-(1.0-complete_obj_map[o].getObjectProb(occ_map, false))*(unseen_prob_estimate), 0.999);
  }
  return complete_obj_probs;
}


inline float angleDist(float angle1, float angle2){
  float angle = angle1-angle2;
  if(angle > M_PI)
    return std::abs(angle-2*M_PI);
  if(angle <= -M_PI)
    return std::abs(angle+2*M_PI);
  return std::abs(angle);
}

float HierarchyMapper::getTravelTime(const geometry_msgs::Pose &door1, const geometry_msgs::Pose &door2){
  tf::Transform d1, d2;
  tf::poseMsgToTF(door1, d1);
  tf::poseMsgToTF(door2, d2);
  tf::Transform diff = d1.inverse()*d2;
  if(diff.getOrigin().length2() == 0)
    return 0;

  float move_angle = tf::getYaw(diff.getRotation());
  return (angleDist(tf::getYaw(d1.getRotation()),move_angle)+angleDist(move_angle,tf::getYaw(d2.getRotation())))*TRAVEL_TURN_FACTOR +
    diff.getOrigin().length()*TRAVEL_DIST_LIN_FACTOR + diff.getOrigin().length2()*TRAVEL_DIST_QUAD_FACTOR;

}


std::vector<float> HierarchyMapper::getToLinkTravelTime(int room, const std::vector<Door>& doors, const nav_msgs::OccupancyGrid& grid_map){
  float x_center = 0.f, y_center = 0.f;
  int num = 0;
  for(int x=0; x<grid_map.info.width; x++){
    for(int y=0; y<grid_map.info.height; y++){
      if(grid_map.data[y * grid_map.info.width + x] == 0){
        x_center += x*grid_map.info.resolution + grid_map.info.origin.position.x;
        y_center += y*grid_map.info.resolution + grid_map.info.origin.position.y;
        num++;
      }
    }
  }
  x_center /= num;
  y_center /= num;

  std::vector<float> times;
  for(const auto& door : doors){
    float move_angle = std::atan2(door.getPose().getOrigin().y()-y_center, door.getPose().getOrigin().x()-x_center);
    float dist2 = (door.getPose().getOrigin().x()-x_center)*(door.getPose().getOrigin().x()-x_center) + (door.getPose().getOrigin().y()-y_center)*(door.getPose().getOrigin().y()-y_center);
    times.push_back((M_PI_2+angleDist(move_angle,tf::getYaw(door.getPose().getRotation())))*TRAVEL_TURN_FACTOR +
           std::sqrt(dist2)*TRAVEL_DIST_LIN_FACTOR + dist2*TRAVEL_DIST_QUAD_FACTOR);
  }

  return times;
}


inline bool isOcc(int x, int y, const nav_msgs::OccupancyGrid& map) {
  return map.data[y * map.info.width + x] > 0;
}
inline bool isUnknown(int x, int y, const nav_msgs::OccupancyGrid& map) {
  return map.data[y * map.info.width + x] < 0;
}
inline bool isFree(int x, int y, const nav_msgs::OccupancyGrid& map) {
  return map.data[y * map.info.width + x] == 0;
}

float HierarchyMapper::getSearchTime(int search_cells){
  return search_cells*SEARCH_TIME_PER_GRID_CELL;
}


std::vector<float> HierarchyMapper::getExpectedSearchTime(float search_time, std::vector<float> obj_probs, const std::vector<ObjectMap>& obj_maps, const cv::Mat_<float>& behind_door_mask){
  std::vector<float> exp_search_times(obj_probs.size(), 0.f);
  int boxes = std::ceil(search_time/5.f);
  float box_time = search_time/boxes;

  for(int i=0; i<obj_probs.size(); i++){
    std::vector<float> prob_distr = obj_maps[i].getProbDistribution(behind_door_mask);
    std::vector<float> box_probs;
    int last_idx = 0;
    for(int j=0; j<boxes; j++){
      box_probs.push_back(std::accumulate(prob_distr.begin()+last_idx, prob_distr.begin()+int(std::round((j+1.f)/boxes*prob_distr.size())), 0.f) * std::exp(-(j+1.f)/boxes));
      last_idx = int(std::round((j+1.f)/boxes*prob_distr.size()));
    }
    float sum = std::accumulate(box_probs.begin(), box_probs.end(), 0.f);
    for(int j=0; j<boxes; j++){
      exp_search_times[i] += box_probs[j]*(obj_probs[i]/sum) * (j+1)*box_time;
    }
    exp_search_times[i] += (1.f-obj_probs[i])*search_time;
  }

  return exp_search_times;
}


bool HierarchyMapper::objMapSrvCb(semantic_mapping_v2::ObjectMapSrv::Request& req, semantic_mapping_v2::ObjectMapSrv::Response& res){
  ROS_INFO("SERVICE OBJ_MAP %d", req.id);
  ros::Time start = ros::Time::now();

  nav_msgs::OccupancyGrid grid_maps;
  std::vector<Door> doors;
  std::vector<ObjectMap> obj_maps;
  std::vector<RoomTypeMap> room_type_maps;

  boost::unique_lock<boost::mutex> maps_lock(mapper_mutex_);
  if(req.room_id < 0)
    req.room_id = current_mapper_;
  grid_maps = room_mapper_[req.room_id]->getMap();
  if(grid_maps.data.size() == 0){
    return false;
  }
  OctoMapper octomaps = (room_mapper_[req.room_id]->getOctomap());
  doors = (room_mapper_[req.room_id]->getDoors());
  obj_maps = (room_mapper_[req.room_id]->getObjMaps());
  room_type_maps = (room_mapper_[req.room_id]->getRoomTypeMaps());
  maps_lock.unlock();

  if(obj_maps.empty() || room_type_maps.empty() || grid_maps.info.width == 0){
    return false;
  }

  cv::Mat_<float> behind_door_mask = getBehindDoorMask(doors, obj_maps[0]);
  ObjectMap occ_map(obj_maps[0].getResolution(), obj_maps[0].getBaseSize(), obj_maps[0].getWidth(), obj_maps[0].getHeight(),
                    obj_maps[0].getOrigin(), obj_maps[0].getMaxHeight(), octomaps, doors);

  std::vector<cv::Mat_<float>> obj_prob_2d_area = get2dAreaObjProbMaps(obj_maps, behind_door_mask, occ_map, room_type_maps[0].getOrigin(),
                                                                       room_type_maps[0].getWidth(), room_type_maps[0].getHeight(), ROOM_CELL_OBJ_KERNEL_SIZE);

  std::vector<cv::Mat_<float>> obj_based_room_type_map = getObjBasedRoomTypeMap(obj_prob_2d_area, room_type_maps.size());
  std::vector<cv::Mat_<float>> complete_room_type_map = getCompleteRoomTypeMap(room_type_maps, obj_based_room_type_map);
  std::vector<float> room_type_cell_number = getRoomTypeCellNumberEstimate(complete_room_type_map, room_type_maps[0].getSeenMap(), room_type_maps[0].getOrigin(),
                                                 grid_maps, doors, room_type_maps[0].getResolution(), room_type_maps[0].getBaseSize());

  std::vector<cv::Mat_<float>> room_based_obj_map = getRoomBasedObjMap(complete_room_type_map, room_type_maps[0].getOrigin() - obj_maps[0].getOrigin(),
                                                                       cv::Size(obj_maps[0].getWidth(), obj_maps[0].getHeight()), obj_maps.size(), req.id);
  std::vector<cv::Point> only_laser_points = getOnlyLaserPoints(obj_maps[0], grid_maps, behind_door_mask);
  std::vector<ObjectMap> complete_obj_map = getCompleteObjMap(room_based_obj_map, obj_maps, occ_map, complete_room_type_map, room_type_maps[0].getSeenMap(),
                                                              room_type_maps[0].getOrigin() - obj_maps[0].getOrigin(), cv::Size(obj_maps[0].getWidth(), obj_maps[0].getHeight()),
                                                              only_laser_points, room_type_cell_number, obj_maps[0].getCount2D(behind_door_mask), req.id);

  if(req.id < 0){
    for(int i=0; i<complete_obj_map.size(); i++){
      res.maps.push_back(complete_obj_map[i].getObjMapMsg());
    }
  }
  else
    res.maps.push_back(complete_obj_map[0].getObjMapMsg());

  ROS_INFO("SERVICE OBJ_MAP IN %.4lf", (ros::Time::now()-start).toSec());
  return true;
}


int HierarchyMapper::estimateUnseen2dCells(const nav_msgs::OccupancyGrid& map, const ObjectMap& obj_map, const cv::Mat_<float> behind_door_mask, bool explored, int& search_cells){

  cv::Mat_<uchar> count2D = obj_map.getCount2D(behind_door_mask);
  cv::Mat_<uchar> occ(count2D.rows, count2D.cols, uchar(0));
  for(int xi = 0; xi < map.info.width; xi++){
    for(int yi = 0; yi < map.info.height; yi++){
      if(map.data[yi * map.info.width + xi] > 0){
        int x = obj_map.getXPixel(map.info.origin.position.x + xi * map.info.resolution);
        int y = obj_map.getYPixel(map.info.origin.position.y + yi * map.info.resolution);
        if(x >= 0 && y >= 0 && x < count2D.cols && y < count2D.rows)
          occ(y, x) = 255;
      }
    }
  }
  int only_laser = 0;
  int seen = 0;
  for(int x = 0; x < count2D.cols; x++){
    for(int y = 0; y < count2D.rows; y++){
      if(occ(y, x) && count2D(y, x) == 0){
        only_laser++;
      }
      else if(occ(y, x)){
        seen++;
      }
    }
  }
  int estimate = ROOM_ESTIMATED_OCCUPIED_AREA * obj_map.getResolution() * obj_map.getResolution();
  int min_unseen = ROOM_MIN_UNEXPLORED_AREA * obj_map.getResolution() * obj_map.getResolution();
  ROS_ERROR("UNSEEN val %d est %d min_unseen %d only_laser %d seen %d", std::max(estimate - seen, min_unseen), estimate, min_unseen, only_laser, seen);
  search_cells = std::max(estimate, min_unseen + seen);
  if(explored)
    return 0;

  return std::max(estimate - seen, min_unseen);
}


void HierarchyMapper::publishDebug(const cv::Mat_<float>& behind_door_mask, const ObjectMap& occ_map, const std::vector<cv::Mat_<float>>& obj_prob_2d_area,
                                   const std::vector<cv::Mat_<float>>& obj_based_room_type_map, const std::vector<cv::Mat_<float>>& complete_room_type_map,
                                   const std::vector<cv::Mat_<float>>& room_based_obj_map, const std::vector<float>& room_type_cell_number,
                                   const std::vector<ObjectMap>& complete_obj_map, const ObjectMap& dummy_occ_map, const std::vector<nav_msgs::OccupancyGrid>& grid_maps,
                                   const std::vector<std::vector<ObjectMap>>& obj_maps, const std::vector<std::vector<RoomTypeMap>>& room_type_maps,
                                   OctoMapper& octomaps, std::vector<cv::Point>& only_laser_points, int i)
{
  for(int j = 0; j < complete_obj_map.size(); j++){
    publishObjProbMap(complete_obj_map[j], j);
  }
  for(int j = 0; j < complete_room_type_map.size(); j++){
    publishRoomTypeProbMap(RoomTypeMap(complete_room_type_map[j], room_type_maps[i][0].getSeenMap(), room_type_maps[i][0].getOrigin(),
                                       room_type_maps[i][0].getResolution(), room_type_maps[i][0].getBaseSize()),j);
  }

  if(base_obj_prob_pub_.empty()){
    base_obj_prob_pub_.resize(80);
    for(int j=0; j<80; j++){
      std::string s = "base_obj_" + ObjectMapper::getObjName(j);
      std::replace( s.begin(), s.end(), ' ', '_');
      base_obj_prob_pub_[j] = nh_.advertise<visualization_msgs::MarkerArray>(s, 1, true);
    }
  }
  for(int j=0; j<80; j++)
    base_obj_prob_pub_[j].publish((obj_maps[i][j]*occ_map).getProbMsg(0, 0.0));

  if(base_room_prob_pub_.empty() && !room_mapper_[0]->getRoomName(1).empty()){
    int num = room_mapper_[0]->getRoomNames().size();
    base_room_prob_pub_.resize(num);
    for(int j=0; j<num; j++){
      std::string s = "base_room_" + room_mapper_[0]->getRoomName(j);
      std::replace( s.begin(), s.end(), ' ', '_');
      base_room_prob_pub_[j] = nh_.advertise<visualization_msgs::MarkerArray>(s, 1, true);
    }
  }
  if(!base_room_prob_pub_.empty())
    for(int j=0; j<room_mapper_[0]->getRoomNames().size(); j++)
      base_room_prob_pub_[j].publish(room_type_maps[i][j].getProbMsg(j));

  std::vector<size_t> order;
  std::vector<float> probs = room_mapper_[0]->getRoomTypeProbs(order);
  if(!probs.empty()){
    for(int j = 0; j < probs.size(); j++){
      std::cout << room_mapper_[0]->getRoomName(order[j]) << ": " << probs[order[j]] << std::endl;
    }
  }
  std::cout << std::endl;
  probs = room_mapper_[0]->getObjectProbs(order);
  for(int j=0; j<probs.size(); j++)
    std::cout << ObjectMapper::getObjName(order[j]) << ": " << probs[order[j]] << std::endl;

  debug_output_map_pub_.publish(grid_maps[i]);
  debug_output_marker_pub_.publish(octomaps.getOccupiedCellMsg(ros::Time::now()));

  std::vector<cv::Mat_<float>> debug_maps = get2dAreaObjProbMaps(complete_obj_map, behind_door_mask, dummy_occ_map, complete_obj_map[0].getOrigin(),
                                                                 complete_obj_map[0].getWidth(), complete_obj_map[0].getHeight(), 0);
//      cv::Mat_<uchar> occupancy_map_obj(debug_maps[0].rows*4, debug_maps[0].cols*4, uchar(0));
//      float factor = 1.f/(complete_obj_map[0].getResolution()*grid_maps[i].info.resolution);
//      for(int x=0; x<occupancy_map_obj.cols; x++){
//        for(int y=0; y<occupancy_map_obj.rows; y++){
//          float xw=complete_obj_map[0].getXWorld(x/4.0), yw=complete_obj_map[0].getYWorld(y/4.0);
//          int xo = (xw-grid_maps[i].info.origin.position.x) / grid_maps[i].info.resolution;
//          int yo = (yw-grid_maps[i].info.origin.position.y) / grid_maps[i].info.resolution;
//          if(xo < 0 || yo < 0 || xo >= grid_maps[i].info.width || yo >= grid_maps[i].info.height)
//            continue;
//          int val = (grid_maps[i].data[yo * grid_maps[i].info.width + xo]);
//          occupancy_map_obj(y,x) = (val > 0 ? 1 : 0);
//          if(factor > 2 && grid_maps[i].info.width > xo+1 && grid_maps[i].info.height > yo+1){
//            occupancy_map_obj(y, x) = ((grid_maps[i].data[(yo + 1) * grid_maps[i].info.width + xo]) > 0 ? 1 : occupancy_map_obj(y, x));
//            occupancy_map_obj(y,x) = ((grid_maps[i].data[(yo) * grid_maps[i].info.width + xo+1]) > 0 ? 1 : occupancy_map_obj(y,x));
//            occupancy_map_obj(y,x) = ((grid_maps[i].data[(yo+1) * grid_maps[i].info.width + xo+1]) > 0 ? 1 : occupancy_map_obj(y,x));
//          }
//        }
//      }
//      cv::Mat_<uchar> occ_dil;
//      int dil_size = 15;
//      cv::copyMakeBorder(occupancy_map_obj, occ_dil, 3*dil_size,3*dil_size,3*dil_size,3*dil_size,cv::BORDER_CONSTANT, 0);
//      cv::dilate(occ_dil, occ_dil, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(dil_size,dil_size)), cv::Point(-1,-1), 3);
//      cv::erode(occ_dil, occ_dil, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(dil_size,dil_size)), cv::Point(-1,-1), 2);
//      cv::bitwise_and(255-occupancy_map_obj*64, occ_dil(cv::Rect(3*dil_size,3*dil_size,occupancy_map_obj.cols,occupancy_map_obj.rows))*255, occupancy_map_obj);
  cv::Mat_<uchar> occupancy_map_obj(debug_maps[0].rows*4, debug_maps[0].cols*4, uchar(128));
  float factor = 1.f/(complete_obj_map[0].getResolution()*grid_maps[i].info.resolution);
  for(int x=0; x<occupancy_map_obj.cols; x++){
    for(int y=0; y<occupancy_map_obj.rows; y++){
      float xw=complete_obj_map[0].getXWorld(x/4.0), yw=complete_obj_map[0].getYWorld(y/4.0);
      int xo = (xw-grid_maps[i].info.origin.position.x) / grid_maps[i].info.resolution;
      int yo = (yw-grid_maps[i].info.origin.position.y) / grid_maps[i].info.resolution;
      if(xo < 0 || yo < 0 || xo >= grid_maps[i].info.width || yo >= grid_maps[i].info.height)
        continue;
      int val = (grid_maps[i].data[yo * grid_maps[i].info.width + xo]);
      occupancy_map_obj(y,x) = (val > 0 ? 196 : (val < 0 ? 128 : 255));
      if(factor > 2 && grid_maps[i].info.width > xo+1 && grid_maps[i].info.height > yo+1){
        occupancy_map_obj(y, x) = ((grid_maps[i].data[(yo + 1) * grid_maps[i].info.width + xo]) > 0 ? 196: occupancy_map_obj(y, x));
        occupancy_map_obj(y,x) = ((grid_maps[i].data[(yo) * grid_maps[i].info.width + xo+1]) > 0 ? 196 : occupancy_map_obj(y,x));
        occupancy_map_obj(y,x) = ((grid_maps[i].data[(yo+1) * grid_maps[i].info.width + xo+1]) > 0 ? 196 : occupancy_map_obj(y,x));
      }
    }
  }

  cv::Mat_<uchar> only_laser_map(debug_maps[0].rows, debug_maps[0].cols, uchar(0));
  for(const auto& p : only_laser_points)
    only_laser_map(p) = 1.0;

  cv::Mat_<uchar> occupancy_map_room;
  occupancy_map_obj.copyTo(occupancy_map_room);
  std::cout << obj_maps[i][0].getOrigin().x << " " << obj_maps[i][0].getOrigin().y << " " << room_type_maps[i][0].getOrigin().x << " " << room_type_maps[i][0].getOrigin().y << std::endl;
  if(obj_maps[i][0].getOrigin() != room_type_maps[i][0].getOrigin())
    cv::copyMakeBorder(occupancy_map_room, occupancy_map_room, room_type_maps[i][0].getOrigin().y*4 - obj_maps[i][0].getOrigin().y*4, 0,
                       room_type_maps[i][0].getOrigin().x*4 - obj_maps[i][0].getOrigin().x*4, 0, cv::BORDER_CONSTANT, cv::Scalar(0.f));
  std::cout << occupancy_map_obj.cols << " " << occupancy_map_obj.rows << " " << room_type_maps[i][0].getWidth() << " " << room_type_maps[i][0].getHeight() << std::endl;
  if(occupancy_map_obj.cols != room_type_maps[i][0].getWidth()*4 || occupancy_map_obj.rows != room_type_maps[i][0].getHeight()*4)
    cv::copyMakeBorder(occupancy_map_room, occupancy_map_room, 0, room_type_maps[i][0].getHeight()*4 - occupancy_map_room.rows, 0, room_type_maps[i][0].getWidth()*4 - occupancy_map_room.cols, cv::BORDER_CONSTANT, cv::Scalar(0.f));

  prob_map_view::ProbMapMsg msg;
  msg.names = ObjectMapper::getObjNames();
  msg.img_are_log = 0;
  msg.occupancy.rows = occupancy_map_obj.rows;
  msg.occupancy.cols = occupancy_map_obj.cols;
  msg.occupancy.type = occupancy_map_obj.type();
  msg.occupancy.data.assign(occupancy_map_obj.datastart, occupancy_map_obj.dataend);
  msg.images.resize(msg.names.size());
  for(int j=0; j<msg.images.size(); j++){
    msg.images[j].rows = debug_maps[j].rows;
    msg.images[j].cols = debug_maps[j].cols;
    msg.images[j].type = debug_maps[j].type();
    msg.images[j].data.assign(debug_maps[j].datastart, debug_maps[j].dataend);
  }
  obj_prob_map_view_pub_.publish(msg);

  debug_maps = get2dAreaObjProbMaps(obj_maps[i], behind_door_mask, occ_map, obj_maps[i][0].getOrigin(),
                                    obj_maps[i][0].getWidth(), obj_maps[i][0].getHeight(), 0);
  msg.img_are_log = 0;
  msg.images.resize(msg.names.size());
  msg.occupancy.rows = occupancy_map_obj.rows;
  msg.occupancy.cols = occupancy_map_obj.cols;
  msg.occupancy.type = occupancy_map_obj.type();
  msg.occupancy.data.assign(occupancy_map_obj.datastart, occupancy_map_obj.dataend);
  for(int j=0; j<msg.images.size(); j++){
    msg.images[j].rows = debug_maps[j].rows;
    msg.images[j].cols = debug_maps[j].cols;
    msg.images[j].type = debug_maps[j].type();
    msg.images[j].data.assign(debug_maps[j].datastart, debug_maps[j].dataend);
  }
  base_obj_prob_map_view_pub_.publish(msg);

  msg.occupancy.rows = occupancy_map_room.rows;
  msg.occupancy.cols = occupancy_map_room.cols;
  msg.occupancy.type = occupancy_map_room.type();
  msg.occupancy.data.assign(occupancy_map_room.datastart, occupancy_map_room.dataend);
  msg.img_are_log = 0;
  msg.images.resize(msg.names.size());
  for(int j=0; j<msg.images.size(); j++){
    msg.images[j].rows = obj_prob_2d_area[j].rows;
    msg.images[j].cols = obj_prob_2d_area[j].cols;
    msg.images[j].type = obj_prob_2d_area[j].type();
    msg.images[j].data.assign(obj_prob_2d_area[j].datastart,obj_prob_2d_area[j].dataend);
  }
  sdf_prob_map_view_pub_.publish(msg);

  msg.occupancy.rows = occupancy_map_obj.rows;
  msg.occupancy.cols = occupancy_map_obj.cols;
  msg.occupancy.type = occupancy_map_obj.type();
  msg.occupancy.data.assign(occupancy_map_obj.datastart, occupancy_map_obj.dataend);
  msg.img_are_log = 4;
  msg.images.resize(msg.names.size());
  for(int j=0; j<msg.images.size(); j++){
    msg.images[j].rows = room_based_obj_map[j].rows;
    msg.images[j].cols = room_based_obj_map[j].cols;
    msg.images[j].type = room_based_obj_map[j].type();
    msg.images[j].data.assign(room_based_obj_map[j].datastart,room_based_obj_map[j].dataend);
  }
  sdf_prob_map_view_pub_.publish(msg);

  msg.occupancy.rows = occupancy_map_obj.rows;
  msg.occupancy.cols = occupancy_map_obj.cols;
  msg.occupancy.type = occupancy_map_obj.type();
  msg.occupancy.data.assign(occupancy_map_obj.datastart, occupancy_map_obj.dataend);
  msg.img_are_log = 5;
  msg.images.resize(msg.names.size());
  std::vector<cv::Mat_<float>> debug_maps2 = get2dAreaObjProbMaps(complete_obj_map, behind_door_mask, dummy_occ_map, complete_obj_map[0].getOrigin(),
                                                                 complete_obj_map[0].getWidth(), complete_obj_map[0].getHeight(), 3);
  for(int j=0; j<msg.images.size(); j++){
    msg.images[j].rows = debug_maps2[j].rows;
    msg.images[j].cols = debug_maps2[j].cols;
    msg.images[j].type = debug_maps2[j].type();
    msg.images[j].data.assign(debug_maps2[j].datastart,debug_maps2[j].dataend);
  }
  sdf_prob_map_view_pub_.publish(msg);

  msg.occupancy.rows = occupancy_map_obj.rows;
  msg.occupancy.cols = occupancy_map_obj.cols;
  msg.occupancy.type = occupancy_map_obj.type();
  msg.occupancy.data.assign(occupancy_map_obj.datastart, occupancy_map_obj.dataend);
  msg.img_are_log = 3;
  msg.images.resize(msg.names.size());
  for(int j=0; j<msg.images.size(); j++){
    msg.images[j].rows = only_laser_map.rows;
    msg.images[j].cols = only_laser_map.cols;
    msg.images[j].type = only_laser_map.type();
    msg.images[j].data.assign(only_laser_map.datastart,only_laser_map.dataend);
  }
  sdf_prob_map_view_pub_.publish(msg);

  msg.occupancy.rows = occupancy_map_room.rows;
  msg.occupancy.cols = occupancy_map_room.cols;
  msg.occupancy.type = occupancy_map_room.type();
  msg.occupancy.data.assign(occupancy_map_room.datastart, occupancy_map_room.dataend);
  msg.names = room_mapper_[0]->getRoomNames();
  msg.img_are_log = 0;
  msg.images.resize(msg.names.size());
  for(int j=0; j<msg.images.size(); j++){
    msg.images[j].rows = complete_room_type_map[j].rows;
    msg.images[j].cols = complete_room_type_map[j].cols;
    msg.images[j].type = complete_room_type_map[j].type();
    msg.images[j].data.assign(complete_room_type_map[j].datastart, complete_room_type_map[j].dataend);
  }
  room_prob_map_view_pub_.publish(msg);

  msg.img_are_log = 0;
  msg.images.resize(msg.names.size());
  for(int j=0; j<msg.images.size(); j++){
    msg.images[j].rows = room_type_maps[i][j].getMap().rows;
    msg.images[j].cols = room_type_maps[i][j].getMap().cols;
    msg.images[j].type = room_type_maps[i][j].getMap().type();
    msg.images[j].data.assign(room_type_maps[i][j].getMap().datastart,room_type_maps[i][j].getMap().dataend);
  }
  base_room_prob_map_view_pub_.publish(msg);

  msg.img_are_log = 1;
  msg.images.resize(msg.names.size());
  for(int j=0; j<msg.images.size(); j++){
    msg.images[j].rows = obj_based_room_type_map[j].rows;
    msg.images[j].cols = obj_based_room_type_map[j].cols;
    msg.images[j].type = obj_based_room_type_map[j].type();
    msg.images[j].data.assign(obj_based_room_type_map[j].datastart,obj_based_room_type_map[j].dataend);
  }
  sdf_prob_map_view_pub_.publish(msg);

  std::vector<cv::Mat_<float>> room_type_maps_filtered;
  cv::Point new_orig = room_type_maps[i][0].getOrigin() - obj_maps[i][0].getOrigin();
  cv::Size new_size(obj_maps[i][0].getWidth(), obj_maps[i][0].getHeight());
  for(int j=0; j<complete_room_type_map.size(); j++){
    room_type_maps_filtered.push_back(cv::Mat_<float>(complete_room_type_map[j].rows, complete_room_type_map[j].cols, room_type_cell_number[j]));
    complete_room_type_map[j].copyTo(room_type_maps_filtered[j], room_type_maps[i][j].getSeenMap());
    cv::GaussianBlur(room_type_maps_filtered[j], room_type_maps_filtered[j], cv::Size(11,11), 3.0, 3.0);
    room_type_maps_filtered[j] = room_type_maps_filtered[j](cv::Rect(new_orig.x, new_orig.y, new_size.width, new_size.height));
  }
  msg.img_are_log = 6;
  msg.images.resize(msg.names.size());
  for(int j=0; j<msg.images.size(); j++){
    msg.images[j].rows = room_type_maps_filtered[j].rows;
    msg.images[j].cols = room_type_maps_filtered[j].cols;
    msg.images[j].type = room_type_maps_filtered[j].type();
    msg.images[j].data.assign(room_type_maps_filtered[j].datastart,room_type_maps_filtered[j].dataend);
  }
  sdf_prob_map_view_pub_.publish(msg);

//      debug_maps = get2dAreaObjProbMaps(std::vector<ObjectMap>({occ_map}), behind_door_mask, dummy_occ_map, obj_maps[i][0].getOrigin(),
//                                        obj_maps[i][0].getWidth(), obj_maps[i][0].getHeight(), 0);
//      msg.occupancy.rows = occupancy_map_obj.rows;
//      msg.occupancy.cols = occupancy_map_obj.cols;
//      msg.occupancy.type = occupancy_map_obj.type();
//      msg.occupancy.data.assign(occupancy_map_obj.datastart, occupancy_map_obj.dataend);
//      msg.names = std::vector<std::string>({"occ"});
//      msg.img_are_log = 2;
//      msg.images.resize(msg.names.size());
//      for(int j=0; j<msg.images.size(); j++){
//        msg.images[j].rows = debug_maps[0].rows;
//        msg.images[j].cols = debug_maps[0].cols;
//        msg.images[j].type = debug_maps[0].type();
//        msg.images[j].data.assign(debug_maps[0].datastart,debug_maps[0].dataend);
//      }
//      sdf_prob_map_view_pub_.publish(msg);
//
//      debug_maps = get2dAreaObjProbMaps(std::vector<ObjectMap>({dummy_occ_map}), behind_door_mask, dummy_occ_map, obj_maps[i][0].getOrigin(),
//                                        obj_maps[i][0].getWidth(), obj_maps[i][0].getHeight(), 0);
//      msg.names = std::vector<std::string>({"occ"});
//      msg.img_are_log = 3;
//      msg.images.resize(msg.names.size());
//      for(int j=0; j<msg.images.size(); j++){
//        msg.images[j].rows = debug_maps[0].rows;
//        msg.images[j].cols = debug_maps[0].cols;
//        msg.images[j].type = debug_maps[0].type();
//        msg.images[j].data.assign(debug_maps[0].datastart,debug_maps[0].dataend);
//      }
//      sdf_prob_map_view_pub_.publish(msg);
}

void HierarchyMapper::insertUnknowRoom(semantic_mapping_v2::RoomMsg& msg, const std::vector<std::vector<Door>>& doors){
  msg.header.stamp = ros::Time::now();
  msg.id = -1;

  msg.obj_probs = std::vector<float>(ObjectMapper::getObjNames().size());
  std::vector<float> room_type_cell_number(82, ROOM_ESTIMATED_OCCUPIED_AREA*ObjectMapper::OBJ_DEFAULT_RESOLUTION*ObjectMapper::OBJ_DEFAULT_RESOLUTION/82);
  for(int o=0; o<msg.obj_probs.size(); o++){
    double unseen_prob_estimate = 1.f;
    for(int r = 0; r < room_type_cell_number.size(); r++){
      unseen_prob_estimate *= std::pow(1.0-RoomMapper::getObjProbGivenRoomPerCell(o,r),room_type_cell_number[r]);
    }
    msg.obj_probs[o] = std::min(1-unseen_prob_estimate, 0.999);
  }

  msg.obj_probs_2 = msg.obj_probs;
  msg.room_type_probs = std::vector<float>(82, 1.f/82);
  msg.room_type_probs_2 = msg.room_type_probs;
  msg.to_link_travel_times.push_back(M_PI*TRAVEL_TURN_FACTOR + 2.f*TRAVEL_DIST_LIN_FACTOR);
  msg.search_time = ROOM_ESTIMATED_OCCUPIED_AREA * ObjectMapper::OBJ_DEFAULT_RESOLUTION * ObjectMapper::OBJ_DEFAULT_RESOLUTION*SEARCH_TIME_PER_GRID_CELL;

  msg.expected_search_time = std::vector<float>(ObjectMapper::getObjNames().size(), 0.f);
  int boxes = std::ceil(msg.search_time/10.f);
  float box_time = msg.search_time/boxes;
  for(int i=0; i<msg.expected_search_time.size(); i++){
    std::vector<float> box_probs(1, 1.f);
    for(int j=1; j<boxes; j++){
      box_probs.push_back(box_probs.back()*0.8);
    }
    float sum = std::accumulate(box_probs.begin(), box_probs.end(), 0.f);
    for(int j=0; j<boxes; j++){
      msg.expected_search_time[i] += box_probs[j]*(msg.obj_probs[i]/sum) * (j+1)*box_time;
    }
    msg.expected_search_time[i] += (1.f-msg.obj_probs[i])*msg.search_time;
  }
}


bool HierarchyMapper::hierarchySrvCb(semantic_mapping_v2::HierarchySrv::Request& req, semantic_mapping_v2::HierarchySrv::Response& res){
  ROS_INFO("SERVICE HIERARCHY");
  ros::Time start = ros::Time::now();

  std::vector<OctoMapper> octomaps;
  std::vector<nav_msgs::OccupancyGrid> gmapping_maps;
  std::vector<nav_msgs::OccupancyGrid> grid_maps;
  std::vector<std::vector<Door>> doors;
  std::vector<std::vector<ObjectMap>> obj_maps;
  std::vector<std::vector<RoomTypeMap>> room_type_maps;
  std::vector<std::vector<float>> base_obj_probs(80);
  std::vector<bool> room_changed;
  std::vector<bool> explored;
  std::vector<semantic_mapping_v2::RoomMsg> last_room_msgs;

  boost::unique_lock<boost::mutex> maps_lock(mapper_mutex_);
  room_changed = room_changed_;
  last_room_msgs = last_room_msgs_;
  explored = explored_;
  for(int i=0; i<room_mapper_.size(); i++){
    grid_maps.push_back(room_mapper_[i]->getMap());
    if(grid_maps.back().data.size() == 0){
      grid_maps.pop_back();
      continue;
    }
    gmapping_maps.push_back(room_mapper_[i]->getGMap());
    octomaps.push_back(room_mapper_[i]->getOctomap());
    doors.push_back(room_mapper_[i]->getDoors());
    obj_maps.push_back(room_mapper_[i]->getObjMaps());
    room_type_maps.push_back(room_mapper_[i]->getRoomTypeMaps());
    std::vector<size_t> order;
    if(DEBUG_OUTPUT)
      base_obj_probs.push_back(room_mapper_[i]->getObjectProbs(order));
    room_changed_[i] = false;
  }
  res.curr_room = current_mapper_;
  maps_lock.unlock();

  for(int i=0; i<grid_maps.size(); i++){
    if(!room_changed[i] && i != req.debug_room){
      res.rooms.push_back(last_room_msgs[i]);
      continue;
    }
    if(obj_maps[i].empty() || room_type_maps[i].empty() || grid_maps[i].info.width == 0){
      res.rooms.push_back(semantic_mapping_v2::RoomMsg());
      continue;
    }

    cv::Mat_<float> behind_door_mask = getBehindDoorMask(doors[i], obj_maps[i][0]);
    ObjectMap occ_map(obj_maps[i][0].getResolution(), obj_maps[i][0].getBaseSize(), obj_maps[i][0].getWidth(), obj_maps[i][0].getHeight(),
                      obj_maps[i][0].getOrigin(), obj_maps[i][0].getMaxHeight(), octomaps[i], doors[i]);

    std::vector<cv::Mat_<float>> obj_prob_2d_area = get2dAreaObjProbMaps(obj_maps[i], behind_door_mask, occ_map, room_type_maps[i][0].getOrigin(),
                                                                     room_type_maps[i][0].getWidth(), room_type_maps[i][0].getHeight(), ROOM_CELL_OBJ_KERNEL_SIZE);
    std::vector<cv::Mat_<float>> obj_based_room_type_map = getObjBasedRoomTypeMap(obj_prob_2d_area, room_type_maps[i].size());
    std::vector<cv::Mat_<float>> complete_room_type_map = getCompleteRoomTypeMap(room_type_maps[i], obj_based_room_type_map);
    std::vector<float> room_type_cell_number = getRoomTypeCellNumberEstimate(complete_room_type_map, room_type_maps[i][0].getSeenMap(), room_type_maps[i][0].getOrigin(),
                                                                   grid_maps[i], doors[i], room_type_maps[i][0].getResolution(), room_type_maps[i][0].getBaseSize());

    std::vector<cv::Mat_<float>> room_based_obj_map = getRoomBasedObjMap(complete_room_type_map, room_type_maps[i][0].getOrigin() - obj_maps[i][0].getOrigin(),
                                                                        cv::Size(obj_maps[i][0].getWidth(), obj_maps[i][0].getHeight()), obj_maps[i].size());
    std::vector<cv::Point> only_laser_points = getOnlyLaserPoints(obj_maps[i][0], grid_maps[i], behind_door_mask);
    int search_cells;
    int num_unseen = estimateUnseen2dCells(gmapping_maps[i], obj_maps[i][0], behind_door_mask, explored[i], search_cells);
    std::vector<ObjectMap> complete_obj_map = getCompleteObjMap(room_based_obj_map, obj_maps[i], occ_map, complete_room_type_map, room_type_maps[i][0].getSeenMap(),
                                                                room_type_maps[i][0].getOrigin() - obj_maps[i][0].getOrigin(), cv::Size(obj_maps[i][0].getWidth(), obj_maps[i][0].getHeight()),
                                                                only_laser_points, room_type_cell_number, obj_maps[i][0].getCount2D(behind_door_mask));
    std::vector<float> complete_obj_probs = getCompleteObjProbs(complete_obj_map, room_type_cell_number, occ_map, num_unseen);
    if(DEBUG_OUTPUT){
      std::vector<float> complete_obj_probs2 = getCompleteObjProbs(complete_obj_map, room_type_cell_number, occ_map, 0);
      for(int i = 0; i < complete_obj_probs.size(); i++){
        std::cout << ObjectMapper::getObjName(i) << " " << complete_obj_probs[i] << " " << complete_obj_probs2[i] << std::endl;
      }

      ObjectMap dummy_occ_map = ObjectMap(obj_maps[i][0].getResolution(), obj_maps[i][0].getBaseSize(), obj_maps[i][0].getWidth(), obj_maps[i][0].getHeight(),
                              obj_maps[i][0].getOrigin(), obj_maps[i][0].getMaxHeight(), 1.f, 1);
      if(PUBLISH_DEBUG_IMAGES && req.debug_room == i){
        publishDebug(behind_door_mask, occ_map, obj_prob_2d_area, obj_based_room_type_map, complete_room_type_map, room_based_obj_map, room_type_cell_number,
                     complete_obj_map, dummy_occ_map, grid_maps, obj_maps, room_type_maps, octomaps[i], only_laser_points, i);
      }
    }

    semantic_mapping_v2::RoomMsg room;
    room.header.stamp = ros::Time::now();
    room.id = i;
    std::vector<size_t> order;
    room.obj_probs = complete_obj_probs;
    room.room_type_probs = room_type_cell_number;
//    room.room_type_probs_2 = getRoomTypeCellNumberEstimate(room_type_maps[i],get, room_type_maps[i][0].getSeenMap(), room_type_maps[i][0].getOrigin(),
//                                                           grid_maps[i], doors[i], room_type_maps[i][0].getResolution(), room_type_maps[i][0].getBaseSize());
    room.obj_probs_2 = base_obj_probs[i];
    room.to_link_travel_times = getToLinkTravelTime(i, doors[i], grid_maps[i]);
    room.search_time = getSearchTime(search_cells)+20.f;
    room.expected_search_time = getExpectedSearchTime(room.search_time, complete_obj_probs, complete_obj_map, behind_door_mask);
    last_room_msgs[i] = room;
    res.rooms.push_back(room);
  }

  int link_count = 0;
  for(int i=0; i<doors.size(); i++){
    for(const auto& door : doors[i]){
      if(door.getOtherRoom() <= i){
        semantic_mapping_v2::HierarchyLinkMsg link;
        link.header.stamp = ros::Time::now();
        if(door.getOtherRoom() < 0){
          link.room1 = i;
          link.room2 = door.getOtherRoom();
          res.rooms[i].links.push_back(link_count);
          tf::poseTFToMsg(door.getPose(), link.door1_pose);
        }
        else{
          link.room1 = door.getOtherRoom();
          link.room2 = i;
          res.rooms[door.getOtherRoom()].links.push_back(link_count);
          res.rooms[i].links.push_back(link_count);
          tf::poseTFToMsg(door.getPose(), link.door2_pose);
          std::vector<Door> other_doors = doors[door.getOtherRoom()];
          for(const auto& door2 : other_doors){
            if(door2.getId() == door.getCounterpartId()){
              tf::poseTFToMsg(door2.getPose(), link.door1_pose);
              break;
            }
          }
        }
        res.links.push_back(link);
        link_count++;
      }
    }
  }

  for(int i=0; i<res.links.size(); i++){
    res.links[i].dists.resize(res.links.size(), std::numeric_limits<float>::max());
    for(int j=0; j<res.links.size(); j++){
      if(res.links[i].room1 == res.links[j].room1)
        res.links[i].dists[j] = getTravelTime(res.links[i].door1_pose, res.links[j].door1_pose);
      else if(res.links[i].room1 == res.links[j].room2)
        res.links[i].dists[j] = getTravelTime(res.links[i].door1_pose, res.links[j].door2_pose);
      else if(res.links[i].room2 == res.links[j].room1)
        res.links[i].dists[j] = getTravelTime(res.links[i].door2_pose, res.links[j].door1_pose);
      else if(res.links[i].room2 != -1 && res.links[i].room2 == res.links[j].room2)
        res.links[i].dists[j] = getTravelTime(res.links[i].door2_pose, res.links[j].door2_pose);
    }
  }

  insertUnknowRoom(res.unknown_room, doors);

  hierarchy_pub_.publish(res);

  maps_lock.lock();
  last_room_msgs_ = last_room_msgs;
  maps_lock.unlock();

  ROS_INFO("SERVICE HIERARCHY IN %.4lf", (ros::Time::now()-start).toSec());
  return true;
}


bool HierarchyMapper::objFoundSrvCb(semantic_mapping_v2::ObjFoundSrv::Request& req, semantic_mapping_v2::ObjFoundSrv::Response& res){
  ROS_INFO("SERVICE OBJ_PROB");
  boost::lock_guard<boost::mutex> maps_lock(mapper_mutex_);
  if(current_mapper_ < 0 || current_mapper_ >= room_mapper_.size())
    return false;

  std::vector<std::pair<geometry_msgs::Pose, float>> tmp = room_mapper_[current_mapper_]->getObjectMax();
  res.max_pose.resize(tmp.size());
  res.max_prob.resize(tmp.size());
  for(int i=0; i<tmp.size(); i++){
    res.max_pose[i] = tmp[i].first;
    res.max_prob[i] = tmp[i].second;
  }

  return true;
}


bool HierarchyMapper::doorPoseSrvCb(semantic_mapping_v2::DoorPoseSrv::Request& req, semantic_mapping_v2::DoorPoseSrv::Response& res){
  tf::Transform old_pose;
  tf::poseMsgToTF(req.old_pose, old_pose);
  boost::unique_lock<boost::mutex> maps_lock(mapper_mutex_);
  if(current_mapper_ < 0 || current_mapper_ >= room_mapper_.size())
    return false;
  std::vector<Door> doors = room_mapper_[current_mapper_]->getDoors();
  maps_lock.unlock();

  double best_confidence = 0.0;
  int best_door = -1;
  for(int i=0; i<doors.size(); i++){
    double confidence = doors[i].getIsDoorConfidence(old_pose);
    if(confidence > best_confidence){
      best_door = i;
      best_confidence = confidence;
    }
  }

  if(best_door < 0)
    return false;

  tf::Transform new_pose;
  tf::Transform diff = new_pose.inverseTimes(old_pose);
  if(std::abs(tf::getYaw(diff.getRotation())) > 0.05 || diff.getOrigin().length() > 0.05){
    res.pose_changed = 1;
    tf::poseTFToMsg(new_pose, res.pose);
  }
  else{
    res.pose_changed = 0;
    tf::poseTFToMsg(old_pose, res.pose);
  }

  return true;
}


void HierarchyMapper::hierarchyTestCb(const semantic_mapping_v2::HierarchyTestMsgConstPtr &msg){
  ROS_INFO("Test HIERARCHY");
  ros::Time start = ros::Time::now();
  semantic_mapping_v2::HierarchySrv::Response res;

  std::vector<OctoMapper> octomaps;
  std::vector<nav_msgs::OccupancyGrid> gmapping_maps;
  std::vector<nav_msgs::OccupancyGrid> grid_maps;
  std::vector<std::vector<Door>> doors;
  std::vector<std::vector<ObjectMap>> obj_maps;
  std::vector<std::vector<RoomTypeMap>> room_type_maps;
  std::vector<std::vector<float>> base_obj_probs;
  std::vector<bool> room_changed;
  std::vector<bool> explored;
  std::vector<semantic_mapping_v2::RoomMsg> last_room_msgs;

  float old_OBJ_BASED_ROOM_AREA_TO_CELL_CONFIDENCE = OBJ_BASED_ROOM_AREA_TO_CELL_CONFIDENCE;
  float old_ROOM_ESTIMATED_OCCUPIED_AREA = ROOM_ESTIMATED_OCCUPIED_AREA;
  float old_ROOM_MIN_UNEXPLORED_AREA = ROOM_MIN_UNEXPLORED_AREA;
  float old_TRAVEL_DIST_LIN_FACTOR = TRAVEL_DIST_LIN_FACTOR;
  float old_TRAVEL_TURN_FACTOR = TRAVEL_TURN_FACTOR;
  float old_SEARCH_TIME_PER_GRID_CELL = SEARCH_TIME_PER_GRID_CELL;

  OBJ_BASED_ROOM_AREA_TO_CELL_CONFIDENCE = msg->OBJ_BASED_ROOM_AREA_TO_CELL_CONFIDENCE;
  ROOM_ESTIMATED_OCCUPIED_AREA = msg->ROOM_ESTIMATED_OCCUPIED_AREA;
  ROOM_MIN_UNEXPLORED_AREA = msg->ROOM_MIN_UNEXPLORED_AREA;
  TRAVEL_DIST_LIN_FACTOR = msg->TRAVEL_DIST_LIN_FACTOR;
  TRAVEL_TURN_FACTOR = msg->TRAVEL_TURN_FACTOR;
  SEARCH_TIME_PER_GRID_CELL = msg->SEARCH_TIME_PER_GRID_CELL;

  boost::unique_lock<boost::mutex> maps_lock(mapper_mutex_);
  RoomMapper::getObjProbGivenRoomPerCell(0,0,true,msg->OBJ_FILL_FRACTION);
  RoomMapper::getObjProbGivenRoomObjPriorPerCell(0,true);
  RoomMapper::getObjProbGivenRoomRoomPriorPerCell(0,true);
  room_changed = room_changed_;
  last_room_msgs = last_room_msgs_;
  explored = explored_;
  for(int i=0; i<room_mapper_.size(); i++){
    grid_maps.push_back(room_mapper_[i]->getMap());
    if(grid_maps.back().data.size() == 0){
      grid_maps.pop_back();
      continue;
    }
    gmapping_maps.push_back(room_mapper_[i]->getGMap());
    octomaps.push_back(room_mapper_[i]->getOctomap());
    doors.push_back(room_mapper_[i]->getDoors());
    obj_maps.push_back(room_mapper_[i]->getObjMaps());
    room_type_maps.push_back(room_mapper_[i]->getRoomTypeMaps());
    std::vector<size_t> order;
    base_obj_probs.push_back(room_mapper_[i]->getObjectProbs(order));
    room_changed_[i] = false;
  }
  res.curr_room = current_mapper_;
  maps_lock.unlock();



  for(int i=0; i<grid_maps.size(); i++){
    if(obj_maps[i].empty() || room_type_maps[i].empty() || grid_maps[i].info.width == 0){
      res.rooms.push_back(semantic_mapping_v2::RoomMsg());
      continue;
    }

    cv::Mat_<float> behind_door_mask = getBehindDoorMask(doors[i], obj_maps[i][0]);
    ObjectMap occ_map(obj_maps[i][0].getResolution(), obj_maps[i][0].getBaseSize(), obj_maps[i][0].getWidth(), obj_maps[i][0].getHeight(),
                      obj_maps[i][0].getOrigin(), obj_maps[i][0].getMaxHeight(), octomaps[i], doors[i]);

    std::vector<cv::Mat_<float>> obj_prob_2d_area = get2dAreaObjProbMaps(obj_maps[i], behind_door_mask, occ_map, room_type_maps[i][0].getOrigin(),
                                                                         room_type_maps[i][0].getWidth(), room_type_maps[i][0].getHeight(), ROOM_CELL_OBJ_KERNEL_SIZE);
    std::vector<cv::Mat_<float>> obj_based_room_type_map = getObjBasedRoomTypeMap(obj_prob_2d_area, room_type_maps[i].size());
    std::vector<cv::Mat_<float>> complete_room_type_map = getCompleteRoomTypeMap(room_type_maps[i], obj_based_room_type_map);
    std::vector<float> room_type_cell_number = getRoomTypeCellNumberEstimate(complete_room_type_map, room_type_maps[i][0].getSeenMap(), room_type_maps[i][0].getOrigin(),
                                                                             grid_maps[i], doors[i], room_type_maps[i][0].getResolution(), room_type_maps[i][0].getBaseSize());

    std::vector<cv::Mat_<float>> room_based_obj_map = getRoomBasedObjMap(complete_room_type_map, room_type_maps[i][0].getOrigin() - obj_maps[i][0].getOrigin(),
                                                                         cv::Size(obj_maps[i][0].getWidth(), obj_maps[i][0].getHeight()), obj_maps[i].size());
    std::vector<cv::Point> only_laser_points = getOnlyLaserPoints(obj_maps[i][0], grid_maps[i], behind_door_mask);
    int search_cells;
    int num_unseen = estimateUnseen2dCells(gmapping_maps[i], obj_maps[i][0], behind_door_mask, explored[i], search_cells);
    std::vector<ObjectMap> complete_obj_map = getCompleteObjMap(room_based_obj_map, obj_maps[i], occ_map, complete_room_type_map, room_type_maps[i][0].getSeenMap(),
                                                                room_type_maps[i][0].getOrigin() - obj_maps[i][0].getOrigin(), cv::Size(obj_maps[i][0].getWidth(), obj_maps[i][0].getHeight()),
                                                                only_laser_points, room_type_cell_number, obj_maps[i][0].getCount2D(behind_door_mask));
    std::vector<float> complete_obj_probs = getCompleteObjProbs(complete_obj_map, room_type_cell_number, occ_map, num_unseen);
    std::vector<float> complete_obj_probs2 = getCompleteObjProbs(complete_obj_map, room_type_cell_number, occ_map, 0);
    for(int i=0; i<complete_obj_probs.size(); i++){
      std::cout << ObjectMapper::getObjName(i) << " " << complete_obj_probs[i] << " " << complete_obj_probs2[i] << std::endl;
    }

    ObjectMap dummy_occ_map(obj_maps[i][0].getResolution(), obj_maps[i][0].getBaseSize(), obj_maps[i][0].getWidth(), obj_maps[i][0].getHeight(),
                            obj_maps[i][0].getOrigin(), obj_maps[i][0].getMaxHeight(), 1.f, 1);

    if(PUBLISH_DEBUG_IMAGES && msg->output_room == i){
      publishDebug(behind_door_mask, occ_map, obj_prob_2d_area, obj_based_room_type_map, complete_room_type_map, room_based_obj_map, room_type_cell_number,
                   complete_obj_map, dummy_occ_map, grid_maps, obj_maps, room_type_maps, octomaps[i], only_laser_points, i);
    }

    semantic_mapping_v2::RoomMsg room;
    room.header.stamp = ros::Time::now();
    room.id = i;
    std::vector<size_t> order;
    room.obj_probs = complete_obj_probs;
    room.room_type_probs = room_type_cell_number;
//    room.room_type_probs_2 = getRoomTypeCellNumberEstimate(room_type_maps[i],get, room_type_maps[i][0].getSeenMap(), room_type_maps[i][0].getOrigin(),
//                                                           grid_maps[i], doors[i], room_type_maps[i][0].getResolution(), room_type_maps[i][0].getBaseSize());
    room.obj_probs_2 = base_obj_probs[i];
    room.to_link_travel_times = getToLinkTravelTime(i, doors[i], grid_maps[i]);
    room.search_time = getSearchTime(search_cells)+20.f;
    room.expected_search_time = getExpectedSearchTime(room.search_time, complete_obj_probs, complete_obj_map, behind_door_mask);
    last_room_msgs[i] = room;
    res.rooms.push_back(room);
  }

  int link_count = 0;
  for(int i=0; i<doors.size(); i++){
    for(const auto& door : doors[i]){
      if(door.getOtherRoom() <= i){
        semantic_mapping_v2::HierarchyLinkMsg link;
        link.header.stamp = ros::Time::now();
        if(door.getOtherRoom() < 0){
          link.room1 = i;
          link.room2 = door.getOtherRoom();
          res.rooms[i].links.push_back(link_count);
          tf::poseTFToMsg(door.getPose(), link.door1_pose);
        }
        else{
          link.room1 = door.getOtherRoom();
          link.room2 = i;
          res.rooms[door.getOtherRoom()].links.push_back(link_count);
          res.rooms[i].links.push_back(link_count);
          tf::poseTFToMsg(door.getPose(), link.door2_pose);
          std::vector<Door> other_doors = doors[door.getOtherRoom()];
          for(const auto& door2 : other_doors){
            if(door2.getId() == door.getCounterpartId()){
              tf::poseTFToMsg(door2.getPose(), link.door1_pose);
              break;
            }
          }
        }
        res.links.push_back(link);
        link_count++;
      }
    }
  }

  for(int i=0; i<res.links.size(); i++){
    res.links[i].dists.resize(res.links.size(), std::numeric_limits<float>::max());
    for(int j=0; j<res.links.size(); j++){
      if(res.links[i].room1 == res.links[j].room1)
        res.links[i].dists[j] = getTravelTime(res.links[i].door1_pose, res.links[j].door1_pose);
      else if(res.links[i].room1 == res.links[j].room2)
        res.links[i].dists[j] = getTravelTime(res.links[i].door1_pose, res.links[j].door2_pose);
      else if(res.links[i].room2 == res.links[j].room1)
        res.links[i].dists[j] = getTravelTime(res.links[i].door2_pose, res.links[j].door1_pose);
      else if(res.links[i].room2 != -1 && res.links[i].room2 == res.links[j].room2)
        res.links[i].dists[j] = getTravelTime(res.links[i].door2_pose, res.links[j].door2_pose);
    }
  }

  insertUnknowRoom(res.unknown_room, doors);

  hierarchy_pub_.publish(res);

  maps_lock.lock();
  last_room_msgs_ = last_room_msgs;
  RoomMapper::getObjProbGivenRoomPerCell(0,0,true,-1.f);
  RoomMapper::getObjProbGivenRoomObjPriorPerCell(0,true);
  RoomMapper::getObjProbGivenRoomRoomPriorPerCell(0,true);
  OBJ_BASED_ROOM_AREA_TO_CELL_CONFIDENCE = old_OBJ_BASED_ROOM_AREA_TO_CELL_CONFIDENCE;
  ROOM_ESTIMATED_OCCUPIED_AREA = old_ROOM_ESTIMATED_OCCUPIED_AREA;
  ROOM_MIN_UNEXPLORED_AREA = old_ROOM_MIN_UNEXPLORED_AREA;
  TRAVEL_DIST_LIN_FACTOR = old_TRAVEL_DIST_LIN_FACTOR;
  TRAVEL_TURN_FACTOR = old_TRAVEL_TURN_FACTOR;
  SEARCH_TIME_PER_GRID_CELL = old_SEARCH_TIME_PER_GRID_CELL;
  maps_lock.unlock();

  std::ofstream file;


  ROS_INFO("Test HIERARCHY IN %.4lf", (ros::Time::now()-start).toSec());
}
