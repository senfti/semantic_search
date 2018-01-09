//
// Created by thomas on 19.11.17.
//

#include "semantic_mapping_v2/HierarchyMapper.h"
#include <tf/transform_datatypes.h>
#include <semantic_mapping_v2/RoomSwitchMsg.h>
#include <std_msgs/Int8.h>

HierarchyMapper::HierarchyMapper()
  : service_spinner_(1, &service_queue_)
{
  addMapper(Door());

  laser_sub_ = nh_.subscribe("scan_filtered", 1, &HierarchyMapper::laserCallback, this);
  cloud_sub_ = nh_.subscribe("/camera/depth_registered/points", 1, &HierarchyMapper::cloudCb, this);
  door_pose_sub_ = nh_.subscribe("door_poses", 1, &HierarchyMapper::doorPoseCb, this);
  vision_sub_ = nh_.subscribe("vision_result", 1, &HierarchyMapper::visionCb, this);

  map_pub_ = nh_.advertise<nav_msgs::OccupancyGrid>("map", 1, true);
  gmap_pub_ = nh_.advertise<nav_msgs::OccupancyGrid>("gmap", 1, true);
  map_switch_pub_ = nh_.advertise<semantic_mapping_v2::RoomSwitchMsg>("map_switch", 1);
  map_door_blocked_pub_ = nh_.advertise<nav_msgs::OccupancyGrid>("map_door_blocked", 1, true);
  map_info_pub_ = nh_.advertise<nav_msgs::MapMetaData>("map_metadata", 1, true);
  marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("occupied_cells_vis_array", 1, true);
  door_pose_pub_ = nh_.advertise<geometry_msgs::PoseArray>("mapper_door_poses", 1, true);
  particle_pose_pub_ = nh_.advertise<geometry_msgs::PoseArray>("particle_poses", 1, true);
  door_found_pub_ = nh_.advertise<std_msgs::Int8>("door_found", 1);

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

  ros::NodeHandle("~").param("transform_publish_period", transform_publish_period_, 0.05);
  ros::NodeHandle("~").param("publish_period", publish_period_, 0.5);
  ros::NodeHandle("~").param("debug_publish_interval", debug_publish_interval_, debug_publish_interval_);
  ros::NodeHandle("~").param("ROOM_CELL_OBJ_KERNEL_SIZE", ROOM_CELL_OBJ_KERNEL_SIZE, ROOM_CELL_OBJ_KERNEL_SIZE);
  ros::NodeHandle("~").param("OBJ_BASED_ROOM_AREA_TO_CELL_CONFIDENCE", OBJ_BASED_ROOM_AREA_TO_CELL_CONFIDENCE, OBJ_BASED_ROOM_AREA_TO_CELL_CONFIDENCE);
  ros::NodeHandle("~").param("OBJ_FILL_FRACTION", OBJ_FILL_FRACTION, OBJ_FILL_FRACTION);
  ros::NodeHandle("~").param("ROOM_ESTIMATED_VOLUME", ROOM_ESTIMATED_VOLUME, ROOM_ESTIMATED_VOLUME);
  ros::NodeHandle("~").param("CELL_TO_OBJ_PROB_GAUSSIAN_SIGMA", CELL_TO_OBJ_PROB_GAUSSIAN_SIGMA, CELL_TO_OBJ_PROB_GAUSSIAN_SIGMA);
  ros::NodeHandle("~").param("SINGLE_VIEW_OBJ_KERNEL_SIZE", SINGLE_VIEW_OBJ_KERNEL_SIZE, SINGLE_VIEW_OBJ_KERNEL_SIZE);
  ros::NodeHandle("~").param("TRAVEL_DIST_LIN_FACTOR", SINGLE_VIEW_OBJ_KERNEL_SIZE, SINGLE_VIEW_OBJ_KERNEL_SIZE);
  ros::NodeHandle("~").param("TRAVEL_DIST_QUAD_FACTOR", SINGLE_VIEW_OBJ_KERNEL_SIZE, SINGLE_VIEW_OBJ_KERNEL_SIZE);
  ros::NodeHandle("~").param("SEARCH_TIME_PER_GRID_CELL", SINGLE_VIEW_OBJ_KERNEL_SIZE, SINGLE_VIEW_OBJ_KERNEL_SIZE);

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
  }
  last_room_msgs_.push_back(semantic_mapping_v2::RoomMsg());
  room_changed_.push_back(true);
  ROS_INFO("Added MAPPER %d", int(room_mapper_.size()) - 1);
  lock.unlock();
  switchMapper(room_mapper_.size() - 1);
}


void HierarchyMapper::switchMapper(int mapper_idx, const Door& door){
  int old_mapper = current_mapper_;
  current_mapper_ = -1;
  if(old_mapper >= 0 && old_mapper < room_mapper_.size()){
    room_mapper_[old_mapper]->deactivate();
    room_changed_[old_mapper] = true;
  }

  tf::Transform door_tf;
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
  if(current_mapper_ >= 0 && current_mapper_ < room_mapper_.size()){
    room_mapper_[current_mapper_]->cloudCb(cloud);
    room_changed_[current_mapper_] = true;
  }
}


void HierarchyMapper::laserCallback(const sensor_msgs::LaserScan::ConstPtr& scan){
  if(current_mapper_ >= 0 && current_mapper_ < room_mapper_.size()){
    room_mapper_[current_mapper_]->laserCallback(scan);
    room_changed_[current_mapper_] = true;
  }
}


void HierarchyMapper::doorPoseCb(const geometry_msgs::PoseArray::ConstPtr &msg){
  if(current_mapper_ >= 0 && current_mapper_ < room_mapper_.size()){
    if(room_mapper_[current_mapper_]->doorCb(msg))
      door_found_pub_.publish(std_msgs::Int8());
    room_changed_[current_mapper_] = true;
  }
}


void HierarchyMapper::visionCb(const vision::VisionMsgConstPtr &msg){
  if(current_mapper_ >= 0 && current_mapper_ < room_mapper_.size()){
    room_mapper_[current_mapper_]->visionCb(msg);
    room_changed_[current_mapper_] = true;
  }
}


void HierarchyMapper::transformPublishLoop(double transform_publish_period){
  if(transform_publish_period == 0)
    return;

  ros::Rate r(1.0 / transform_publish_period);
  while(ros::ok()){
    if(current_mapper_ >= 0 && current_mapper_ < room_mapper_.size()){
      tfB_->sendTransform(room_mapper_[current_mapper_]->getTransform());
    }
    r.sleep();
  }
}


void HierarchyMapper::downprojecAndPublish(){
  ros::Time start = ros::Time::now();
  room_mapper_[current_mapper_]->downprojectMap();
  gmap_pub_.publish(room_mapper_[current_mapper_]->getGMap());
  nav_msgs::OccupancyGrid map = room_mapper_[current_mapper_]->getMap();
  if(map.data.size() > 0){
    map_pub_.publish(map);
    map_info_pub_.publish(map.info);
  }
  map = room_mapper_[current_mapper_]->getDoorBlockedMap();
  if(map.data.size() > 0){
    map_door_blocked_pub_.publish(map);
  }
  marker_pub_.publish(room_mapper_[current_mapper_]->getOccupiedCellMsg());
  particle_pose_pub_.publish(room_mapper_[current_mapper_]->getParticlePoseMsg());
  door_pose_pub_.publish(room_mapper_[current_mapper_]->getDoorPoseMsg());

  static int counter=0;
  if((counter%debug_publish_interval_) == debug_publish_interval_-1){
    if(base_obj_prob_pub_.empty()){
      base_obj_prob_pub_.resize(80);
      for(int i=0; i<80; i++){
        std::string s = "base_obj_" + ObjectMapper::getObjName(i);
        std::replace( s.begin(), s.end(), ' ', '_');
        base_obj_prob_pub_[i] = nh_.advertise<visualization_msgs::MarkerArray>(s, 1, true);
      }
    }
    for(int i=0; i<80; i++)
      base_obj_prob_pub_[i].publish(room_mapper_[current_mapper_]->getObjectProbMsg(i));

    if(base_room_prob_pub_.empty() && !room_mapper_[current_mapper_]->getRoomName(1).empty()){
      int num = room_mapper_[current_mapper_]->getRoomNames().size();
      base_room_prob_pub_.resize(num);
      for(int i=0; i<num; i++){
        std::string s = "base_room_" + room_mapper_[current_mapper_]->getRoomName(i);
        std::replace( s.begin(), s.end(), ' ', '_');
        base_room_prob_pub_[i] = nh_.advertise<visualization_msgs::MarkerArray>(s, 1, true);
      }
    }
    if(!base_room_prob_pub_.empty())
      for(int i=0; i<room_mapper_[current_mapper_]->getRoomNames().size(); i++)
        base_room_prob_pub_[i].publish(room_mapper_[current_mapper_]->getRoomProbMsg(i));

    std::vector<size_t> order;
    std::vector<float> probs = room_mapper_[current_mapper_]->getRoomTypeProbs(order);
    if(!probs.empty()){
      for(int i = 0; i < probs.size(); i++){
        std::cout << room_mapper_[current_mapper_]->getRoomName(order[i]) << ": " << probs[order[i]] << std::endl;
      }
    }
    std::cout << std::endl;
    probs = room_mapper_[current_mapper_]->getObjectProbs(order);
    for(int i=0; i<probs.size(); i++)
      std::cout << ObjectMapper::getObjName(order[i]) << ": " << probs[order[i]] << std::endl;
  }
  counter++;

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


cv::Mat_<float> HierarchyMapper::getBehindDoorMask(const std::vector<Door>& doors, int width, int height){
  cv::Mat_<float> behind_door_mask(height,width, 0.f);
  for(int x=0; x<behind_door_mask.cols; x++){
    for(int y=0; y<behind_door_mask.rows; y++){
      for(const auto& d : doors){
        if(d.isBehindDoor(x,y))
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
    if(obj_maps[0].getOrigin() != room_origin)
      cv::copyMakeBorder(prob_2d, prob_2d, room_origin.y - obj_maps[0].getOrigin().y, 0, room_origin.x - obj_maps[0].getOrigin().x, 0, cv::BORDER_CONSTANT, cv::Scalar(0.f));
    if(prob_2d.cols != room_width || prob_2d.rows != room_height)
      cv::copyMakeBorder(prob_2d, prob_2d, 0, room_height - prob_2d.rows, 0, room_width - prob_2d.cols, cv::BORDER_CONSTANT, cv::Scalar(0.f));

    prob_2d_area.push_back(cv::Mat_<float>(prob_2d.rows, prob_2d.cols, 1.f));
    int k_size = kernel_size/2.f*obj_maps[0].getResolution();
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
      float o_prob = RoomMapper::getObjProbGivenRoomPrior(o);
      float N = room_type_map.size();
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


std::vector<float> HierarchyMapper::getRoomTypeProbs(const std::vector<cv::Mat_<float>>& complete_room_type_map, const cv::Mat_<uchar>& seen_map, const cv::Point& origin,
                                    const nav_msgs::OccupancyGrid& grid_map, const std::vector<Door>& doors, float resolution, int base_size){
  RoomTypeMapper tmp_mapper(complete_room_type_map, seen_map, origin, resolution, base_size);
  std::vector<size_t> order;
  return tmp_mapper.getRoomProb(grid_map,doors,order);
}


inline double getObjProbGivenRoomPerCell(int o, int r, double object_in_cell_fraction){
  return 1.f-std::pow(1.0-RoomMapper::getObjProbGivenRoom(o, r), object_in_cell_fraction);
}


std::vector<ObjectMap> HierarchyMapper::getCompleteObjMap(const std::vector<cv::Mat_<float>>& complete_room_type_map, const std::vector<ObjectMap>& obj_map, const ObjectMap& occ_map, const cv::Point& new_orig){
  std::vector<ObjectMap> complete_obj_map;
  for(int o=0; o<obj_map.size(); o++){
    cv::Mat_<float> obj_from_room_map = cv::Mat_<float>(complete_room_type_map[o].rows, complete_room_type_map[o].cols, 0.f);
    cv::Mat_<float> inv = cv::Mat_<float>(complete_room_type_map[o].rows, complete_room_type_map[o].cols, 0.f);
    for(int r = 0; r < complete_room_type_map.size(); r++){
      float o_r_prob = getObjProbGivenRoomPerCell(o, r, OBJ_FILL_FRACTION);
      obj_from_room_map += complete_room_type_map[r] * o_r_prob;
      inv += complete_room_type_map[r] * (1.0 - o_r_prob);
    }
    cv::divide(obj_from_room_map, obj_from_room_map + inv, obj_from_room_map);
    int k_size = int(CELL_TO_OBJ_PROB_GAUSSIAN_SIGMA*3)*2+1;
    cv::GaussianBlur(obj_from_room_map, obj_from_room_map, cv::Size(k_size,k_size), CELL_TO_OBJ_PROB_GAUSSIAN_SIGMA, CELL_TO_OBJ_PROB_GAUSSIAN_SIGMA, cv::BORDER_REPLICATE);
    obj_from_room_map = obj_from_room_map(cv::Rect(new_orig.x, new_orig.y, obj_map[0].getWidth(), obj_map[0].getHeight()));

    complete_obj_map.push_back(ObjectMap(obj_map[o], occ_map, obj_from_room_map));
  }
  return complete_obj_map;
}


std::vector<float> HierarchyMapper::getCompleteObjProbs(const std::vector<ObjectMap>& complete_obj_map, std::vector<float> room_type_probs, const cv::Mat_<float> behind_door_mask){
  std::vector<float> complete_obj_probs(complete_obj_map.size());
  for(int o=0; o<complete_obj_map.size(); o++){
    float unseen_prob_estimate = 0.f;
    for(int r = 0; r < room_type_probs.size(); r++){
      unseen_prob_estimate += room_type_probs[r] * getObjProbGivenRoomPerCell(o, r, OBJ_FILL_FRACTION);
    }
    complete_obj_probs[o] = complete_obj_map[o].getObjectProb(behind_door_mask, unseen_prob_estimate, ROOM_ESTIMATED_VOLUME);
  }
  return complete_obj_probs;
}


std::vector<float> HierarchyMapper::getTravelTimes(const std::vector<Door> &doors){
  std::vector<float> travel_times;
  for(int i=0; i<doors.size(); i++){
    for(int j=i+1; j<doors.size(); j++){
      double dist_2 = (doors[i].getPose().getOrigin()-doors[j].getPose().getOrigin()).length2();
      travel_times.push_back(std::sqrt(dist_2)*TRAVEL_DIST_LIN_FACTOR + dist_2*TRAVEL_DIST_QUAD_FACTOR);
    }
  }
  return travel_times;
}


float HierarchyMapper::getTravelTime(const geometry_msgs::Pose &door1, const geometry_msgs::Pose &door2){
  double dist_2 = std::pow(door1.position.x-door2.position.x,2) + std::pow(door1.position.y-door2.position.y,2) + std::pow(door1.position.z-door2.position.z,2);
  return std::sqrt(dist_2)*TRAVEL_DIST_LIN_FACTOR + dist_2*TRAVEL_DIST_QUAD_FACTOR;
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

float HierarchyMapper::getSearchTime(const nav_msgs::OccupancyGrid &grid_map){
  bool fully_explored = true;
  int num_cells = 0;
  for(int x=1; x<grid_map.info.width-1; x++){
    for(int y=1; y<grid_map.info.height-1; y++){
      if(grid_map.data[y * grid_map.info.width + x] > 0)
        num_cells++;
      if(isFree(x,y, grid_map) && (isUnknown(x-1,y-1, grid_map) || isUnknown(x-1,y, grid_map) || isUnknown(x-1,y+1, grid_map) || isUnknown(x,y-1, grid_map) ||
                                   isUnknown(x+1,y-1, grid_map) || isUnknown(x+1,y, grid_map) || isUnknown(x+1,y+1, grid_map) || isUnknown(x,y+1, grid_map)))
        fully_explored = false;
    }
  }
  if(!fully_explored)
    num_cells = std::max(num_cells, 320) + 160;

  return num_cells*SEARCH_TIME_PER_GRID_CELL;
}


bool HierarchyMapper::objMapSrvCb(semantic_mapping_v2::ObjectMapSrv::Request& req, semantic_mapping_v2::ObjectMapSrv::Response& res){
  ROS_INFO("SERVICE OBJ_MAP");
  ros::Time start = ros::Time::now();

  std::vector<OctoMapper> octomaps;
  std::vector<nav_msgs::OccupancyGrid> grid_maps;
  std::vector<std::vector<Door>> doors;
  std::vector<std::vector<ObjectMap>> obj_maps;
  std::vector<std::vector<RoomTypeMap>> room_type_maps;
  std::vector<std::vector<float>> base_room_type_probs;
  std::vector<std::vector<float>> base_obj_probs;

  boost::unique_lock<boost::mutex> maps_lock(mapper_mutex_);
  for(int i=0; i<room_mapper_.size(); i++){
    grid_maps.push_back(room_mapper_[i]->getMap());
    octomaps.push_back(room_mapper_[i]->getOctomap());
    doors.push_back(room_mapper_[i]->getDoors());
    obj_maps.push_back(room_mapper_[i]->getObjMaps());
    room_type_maps.push_back(room_mapper_[i]->getRoomTypeMaps());
    std::vector<size_t> order;
    base_room_type_probs.push_back(room_mapper_[i]->getRoomTypeProbs(order));
    base_obj_probs.push_back(room_mapper_[i]->getObjectProbs(order));
  }
  maps_lock.unlock();

  cv::Mat_<float> behind_door_mask = getBehindDoorMask(doors[req.room_id], obj_maps[req.room_id][0].getWidth(), obj_maps[req.room_id][0].getHeight());
  ObjectMap occ_map(obj_maps[req.room_id][0].getResolution(), obj_maps[req.room_id][0].getBaseSize(), obj_maps[req.room_id][0].getWidth(), obj_maps[req.room_id][0].getHeight(),
                    obj_maps[req.room_id][0].getOrigin(), obj_maps[req.room_id][0].getMaxHeight(), octomaps[req.room_id]);

  std::vector<cv::Mat_<float>> obj_prob_2d_area = get2dAreaObjProbMaps(obj_maps[req.room_id], behind_door_mask, occ_map, room_type_maps[req.room_id][0].getOrigin(),
                                                                       room_type_maps[req.room_id][0].getWidth(), room_type_maps[req.room_id][0].getHeight(), ROOM_CELL_OBJ_KERNEL_SIZE);
  std::vector<cv::Mat_<float>> obj_based_room_type_map = getObjBasedRoomTypeMap(obj_prob_2d_area, room_type_maps[req.room_id].size());
  std::vector<cv::Mat_<float>> complete_room_type_map = getCompleteRoomTypeMap(room_type_maps[req.room_id], obj_based_room_type_map);
  std::vector<float> complete_room_type_probs = getRoomTypeProbs(complete_room_type_map, room_type_maps[req.room_id][0].getSeenMap(), room_type_maps[req.room_id][0].getOrigin(),
                                                                 grid_maps[req.room_id], doors[req.room_id], room_type_maps[req.room_id][0].getResolution(), room_type_maps[req.room_id][0].getBaseSize());

  std::vector<ObjectMap> complete_obj_map = getCompleteObjMap(complete_room_type_map, obj_maps[req.room_id], occ_map, room_type_maps[req.room_id][0].getOrigin() - obj_maps[req.room_id][0].getOrigin());

  if(req.id < 0){
    for(int i=0; i<complete_obj_map.size(); i++){
      res.maps.push_back(complete_obj_map[i].getObjMapMsg());
    }
  }
  else
    res.maps.push_back(complete_obj_map[req.id].getObjMapMsg());

  return true;
}


bool HierarchyMapper::hierarchySrvCb(semantic_mapping_v2::HierarchySrv::Request& req, semantic_mapping_v2::HierarchySrv::Response& res){
  ROS_INFO("SERVICE HIERARCHY");
  ros::Time start = ros::Time::now();

  std::vector<OctoMapper> octomaps;
  std::vector<nav_msgs::OccupancyGrid> grid_maps;
  std::vector<std::vector<Door>> doors;
  std::vector<std::vector<ObjectMap>> obj_maps;
  std::vector<std::vector<RoomTypeMap>> room_type_maps;
  std::vector<std::vector<float>> base_room_type_probs;
  std::vector<std::vector<float>> base_obj_probs;
  std::vector<bool> room_changed;
  std::vector<semantic_mapping_v2::RoomMsg> last_room_msgs;

  boost::unique_lock<boost::mutex> maps_lock(mapper_mutex_);
  room_changed = room_changed_;
  last_room_msgs = last_room_msgs_;
  for(int i=0; i<room_mapper_.size(); i++){
    grid_maps.push_back(room_mapper_[i]->getMap());
    octomaps.push_back(room_mapper_[i]->getOctomap());
    doors.push_back(room_mapper_[i]->getDoors());
    obj_maps.push_back(room_mapper_[i]->getObjMaps());
    room_type_maps.push_back(room_mapper_[i]->getRoomTypeMaps());
    std::vector<size_t> order;
    base_room_type_probs.push_back(room_mapper_[i]->getRoomTypeProbs(order));
    base_obj_probs.push_back(room_mapper_[i]->getObjectProbs(order));
    room_changed_[i] = false;
  }
  res.curr_room = current_mapper_;
  maps_lock.unlock();

  for(int i=0; i<grid_maps.size(); i++){
    if(!room_changed[i]){
      res.rooms.push_back(last_room_msgs[i]);
      continue;
    }
    if(obj_maps[i].empty() || room_type_maps[i].empty() || grid_maps[i].info.width == 0){
      res.rooms.push_back(semantic_mapping_v2::RoomMsg());
      continue;
    }

    cv::Mat_<float> behind_door_mask = getBehindDoorMask(doors[i], obj_maps[i][0].getWidth(), obj_maps[i][0].getHeight());
    ObjectMap occ_map(obj_maps[i][0].getResolution(), obj_maps[i][0].getBaseSize(), obj_maps[i][0].getWidth(), obj_maps[i][0].getHeight(),
                      obj_maps[i][0].getOrigin(), obj_maps[i][0].getMaxHeight(), octomaps[i]);

    std::vector<cv::Mat_<float>> obj_prob_2d_area = get2dAreaObjProbMaps(obj_maps[i], behind_door_mask, occ_map, room_type_maps[i][0].getOrigin(),
                                                                     room_type_maps[i][0].getWidth(), room_type_maps[i][0].getHeight(), ROOM_CELL_OBJ_KERNEL_SIZE);
    std::vector<cv::Mat_<float>> obj_based_room_type_map = getObjBasedRoomTypeMap(obj_prob_2d_area, room_type_maps[i].size());
    std::vector<cv::Mat_<float>> complete_room_type_map = getCompleteRoomTypeMap(room_type_maps[i], obj_based_room_type_map);
    std::vector<float> complete_room_type_probs = getRoomTypeProbs(complete_room_type_map, room_type_maps[i][0].getSeenMap(), room_type_maps[i][0].getOrigin(),
                                                                   grid_maps[i], doors[i], room_type_maps[i][0].getResolution(), room_type_maps[i][0].getBaseSize());

    std::vector<ObjectMap> complete_obj_map = getCompleteObjMap(complete_room_type_map, obj_maps[i], occ_map, room_type_maps[i][0].getOrigin() - obj_maps[i][0].getOrigin());
    std::vector<float> complete_obj_probs = getCompleteObjProbs(complete_obj_map, complete_room_type_probs, behind_door_mask);

    if(i==current_mapper_){
      for(int j = 0; j < complete_obj_map.size(); j++){
        publishObjProbMap(complete_obj_map[j], j);
      }
      for(int j = 0; j < complete_room_type_map.size(); j++){
        publishRoomTypeProbMap(RoomTypeMap(complete_room_type_map[j], room_type_maps[i][0].getSeenMap(), room_type_maps[i][0].getOrigin(),
                                           room_type_maps[i][0].getResolution(), room_type_maps[i][0].getBaseSize()),j);
      }
    }

    std::vector<cv::Mat_<float>> single_view_prob_map = get2dAreaObjProbMaps(complete_obj_map, behind_door_mask, occ_map, complete_obj_map[0].getOrigin(),
                                                                             complete_obj_map[0].getWidth(), complete_obj_map[0].getHeight(), SINGLE_VIEW_OBJ_KERNEL_SIZE);
    std::vector<float> single_view_max_probs(single_view_prob_map.size());
    std::vector<geometry_msgs::Point> single_view_points(single_view_prob_map.size());
    for(int o=0;o<single_view_max_probs.size();o++){
      double min, max;
      cv::Point min_loc, max_loc;
      cv::minMaxLoc(single_view_prob_map[o], &min, &max, &min_loc, &max_loc);
      single_view_max_probs[o] = max;
      geometry_msgs::Point point;
      point.x = complete_obj_map[0].getXWorld(min_loc.x);
      point.y = complete_obj_map[0].getXWorld(min_loc.y);
      point.z = 0;
      single_view_points.push_back(point);
    }

    semantic_mapping_v2::RoomMsg room;
    room.header.stamp = ros::Time::now();
    room.id = i;
    std::vector<size_t> order;
    room.obj_probs = complete_obj_probs;
    room.room_type_probs = complete_room_type_probs;
    room.room_type_probs_2 = base_room_type_probs[i];
    room.obj_probs_2 = base_obj_probs[i];
    room.single_view_obj_probs = single_view_max_probs;
    room.single_view_points = single_view_points;
    room.to_link_travel_times.resize(doors[i].size(), 20.f);
    room.single_view_search_time = (room.to_link_travel_times.empty() ? 10.f : std::accumulate(room.to_link_travel_times.begin(), room.to_link_travel_times.end(), 0.f) / room.to_link_travel_times.size());
    room.search_time = getSearchTime(grid_maps[i]);
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
        }
        else{
          link.room1 = door.getOtherRoom();
          link.room2 = i;
          res.rooms[door.getOtherRoom()].links.push_back(link_count);
          res.rooms[i].links.push_back(link_count);
        }
        tf::poseTFToMsg(door.getPose(), link.door1_pose);
        if(door.getOtherRoom() >= 0){
          std::vector<Door> other_doors = doors[door.getOtherRoom()];
          for(const auto& door2 : other_doors){
            if(door2.getId() == door.getCounterpartId()){
              tf::poseTFToMsg(door2.getPose(), link.door2_pose);
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

  maps_lock.lock();
  last_room_msgs_ = last_room_msgs;
  maps_lock.unlock();

  ROS_INFO("SERVICE HIERARCHY IN %.4lf", (ros::Time::now()-start).toSec());
  return true;
}



