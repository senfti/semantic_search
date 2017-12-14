//
// Created by thomas on 19.11.17.
//

#include "semantic_mapping_v2/HierarchyMapper.h"
#include <tf/transform_datatypes.h>

HierarchyMapper::HierarchyMapper(){
  addMapper(Door());

  laser_sub_ = nh_.subscribe("scan_filtered", 1, &HierarchyMapper::laserCallback, this);
  cloud_sub_ = nh_.subscribe("/camera/depth_registered/points", 1, &HierarchyMapper::cloudCb, this);
  door_pose_sub_ = nh_.subscribe("door_poses", 1, &HierarchyMapper::doorPoseCb, this);
  vision_sub_ = nh_.subscribe("vision_result", 1, &HierarchyMapper::visionCb, this);

  map_pub_ = nh_.advertise<nav_msgs::OccupancyGrid>("map", 1, true);
  gmap_pub_ = nh_.advertise<nav_msgs::OccupancyGrid>("gmap", 1, true);
  map_info_pub_ = nh_.advertise<nav_msgs::MapMetaData>("map_metadata", 1, true);
  marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("occupied_cells_vis_array", 1, true);
  door_pose_pub_ = nh_.advertise<geometry_msgs::PoseArray>("mapper_door_poses", 1, true);
  particle_pose_pub_ = nh_.advertise<geometry_msgs::PoseArray>("particle_poses", 1, true);

  gmap_srv_ = nh_.advertiseService("gmap_srv", &HierarchyMapper::gmapSrvCb, this);
  map_srv_ = nh_.advertiseService("map_srv", &HierarchyMapper::mapSrvCb, this);
  octomap_srv_ = nh_.advertiseService("octomap_srv", &HierarchyMapper::octomapSrvCb, this);
  obj_map_srv_ = nh_.advertiseService("obj_map_srv", &HierarchyMapper::objMapSrvCb, this);
  room_type_map_srv_ = nh_.advertiseService("room_type_map_srv", &HierarchyMapper::roomTypeMapSrvCb, this);
  obj_prob_srv_ = nh_.advertiseService("obj_prob_srv", &HierarchyMapper::objProbSrvCb, this);
  room_type_prob_srv_ = nh_.advertiseService("room_type_prob_srv", &HierarchyMapper::roomTypeProbSrvCb, this);
  hierarchy_srv_ = nh_.advertiseService("hierarchy_srv", &HierarchyMapper::hierarchySrvCb, this);

  ros::NodeHandle("~").param("transform_publish_period", transform_publish_period_, 0.05);
  tfB_ = new tf::TransformBroadcaster();
  transform_thread_ = new boost::thread(boost::bind(&HierarchyMapper::transformPublishLoop, this, transform_publish_period_));
  std::cout << "HierarchyMapping started" << std::endl;
}

HierarchyMapper::~HierarchyMapper(){
  transform_thread_->join();
  delete transform_thread_;
  for(auto& mapper : room_mapper_)
    delete mapper;
  delete tfB_;
}


void HierarchyMapper::addMapper(const Door& door){
  if(door.isValid()){
    tf::StampedTransform transform;
    tf_listener_.lookupTransform("map", "base_laser_link", ros::Time(0), transform);

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
  ROS_INFO("Added MAPPER %d", int(room_mapper_.size()) - 1);
  switchMapper(room_mapper_.size() - 1);
}


void HierarchyMapper::switchMapper(int mapper_idx, const Door& door){
  int old_mapper = current_mapper_;
  current_mapper_ = -1;
  if(old_mapper >= 0 && old_mapper < room_mapper_.size())
    room_mapper_[old_mapper]->deactivate();

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
      room_mapper_[current_mapper_]->activate(p, door);
    }
    else{
      current_mapper_ = mapper_idx;
      room_mapper_[current_mapper_]->activate();
    }
  }
  ROS_INFO("Switched to MAPPER %d", current_mapper_);
}


void HierarchyMapper::cloudCb(const sensor_msgs::PointCloud2::ConstPtr& cloud){
  if(current_mapper_ >= 0 && current_mapper_ < room_mapper_.size())
    room_mapper_[current_mapper_]->cloudCb(cloud);
}


void HierarchyMapper::laserCallback(const sensor_msgs::LaserScan::ConstPtr& scan){
  if(current_mapper_ >= 0 && current_mapper_ < room_mapper_.size())
    room_mapper_[current_mapper_]->laserCallback(scan);
}


void HierarchyMapper::doorPoseCb(const geometry_msgs::PoseArray::ConstPtr &msg){
  if(current_mapper_ >= 0 && current_mapper_ < room_mapper_.size())
    room_mapper_[current_mapper_]->doorCb(msg);
}


void HierarchyMapper::visionCb(const vision::VisionMsgConstPtr &msg){
  if(current_mapper_ >= 0 && current_mapper_ < room_mapper_.size())
    room_mapper_[current_mapper_]->visionCb(msg);
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


void HierarchyMapper::publish(){
  gmap_pub_.publish(room_mapper_[current_mapper_]->getGMap());
  nav_msgs::OccupancyGrid map = room_mapper_[current_mapper_]->getMap();
  if(map.data.size() > 0){
    map_pub_.publish(map);
    map_info_pub_.publish(map.info);
  }
  marker_pub_.publish(room_mapper_[current_mapper_]->getOccupiedCellMsg());
  door_pose_pub_.publish(room_mapper_[current_mapper_]->getDoorPoseMsg());

  if(obj_prob_pub_.empty()){
    obj_prob_pub_.resize(80);
    for(int i=0; i<80; i++){
      std::string s = "obj_" + ObjectMapper::getObjName(i);
      std::replace( s.begin(), s.end(), ' ', '_');
      obj_prob_pub_[i] = nh_.advertise<visualization_msgs::MarkerArray>(s, 1, true);
    }
  }
  for(int i=0; i<80; i++)
    obj_prob_pub_[i].publish(room_mapper_[current_mapper_]->getObjectProbMsg(i));

  std::vector<size_t> order;
  std::vector<float> probs = room_mapper_[current_mapper_]->getRoomTypeProbs(order);
  if(room_prob_pub_.empty() && !room_mapper_[current_mapper_]->getRoomName(1).empty()){
    room_prob_pub_.resize(205);
    for(int i=0; i<205; i++){
      std::string s = "room_" + room_mapper_[current_mapper_]->getRoomName(i);
      std::replace( s.begin(), s.end(), ' ', '_');
      room_prob_pub_[i] = nh_.advertise<visualization_msgs::MarkerArray>(s, 1, true);
    }
  }
  if(!room_prob_pub_.empty())
    for(int i=0; i<205; i++)
      room_prob_pub_[i].publish(room_mapper_[current_mapper_]->getRoomProbMsg(i));
  if(!probs.empty())
    for(int i=0; i<10; i++){
      std::cout << room_mapper_[current_mapper_]->getRoomName(order[i]) << ": " << probs[order[i]] << std::endl;
  }
  probs = room_mapper_[current_mapper_]->getObjectProbs(order);
  for(int i=0; i<probs.size(); i++)
    std::cout << ObjectMapper::getObjName(order[i]) << ": " << probs[order[i]] << std::endl;
}


void HierarchyMapper::downprojecAndPublishMap(){
  room_mapper_[current_mapper_]->downprojectMap();
  nav_msgs::OccupancyGrid map = room_mapper_[current_mapper_]->getMap();
  if(map.data.size() > 0){
    map_pub_.publish(map);
    map_info_pub_.publish(map.info);
  }
  door_pose_pub_.publish(room_mapper_[current_mapper_]->getDoorPoseMsg());
  particle_pose_pub_.publish(room_mapper_[current_mapper_]->getParticlePoseMsg());
}


void HierarchyMapper::run(){
  ros::Rate rate(50);
  ros::Time last_publish = ros::TIME_MAX;
  while(ros::ok()){
    ros::spinOnce();
    if(current_mapper_ >= 0 && current_mapper_ < room_mapper_.size() && room_mapper_[current_mapper_]->isInitialized()){
      if(room_mapper_[current_mapper_]->resetWasMapUpdated()){
        publish();
        last_publish = ros::Time::now();
      }
      else if(ros::Time::now() - last_publish > ros::Duration(0.5)){
        downprojecAndPublishMap();
        last_publish = ros::Time::now();
      }

      Door door = room_mapper_[current_mapper_]->droveThroughDoor();
      if(door.isValid()){
        ROS_INFO("Door %d, c_id: %d, r: %d, c_r: %d", door.id_, door.counterpart_id_, door.this_room_, door.other_room_);
        int other_room = door.other_room_;
        if(other_room < 0){
          int new_id = Door::getID();
          room_mapper_[current_mapper_]->setDoorRoom(door.pose_, room_mapper_.size(), new_id);

          Door counterpart_door(room_mapper_.size(), door.pose_, std::min(door.proposal_count_, 5), current_mapper_, new_id, door.id_);
          counterpart_door.pose_.setRotation(tf::Quaternion(tf::Vector3(0,0,1), door.pose_.getRotation().getAngle() + M_PI));
          ROS_INFO("New Door %d, c_id: %d, r: %d, c_r: %d", counterpart_door.id_, counterpart_door.counterpart_id_, counterpart_door.this_room_, counterpart_door.other_room_);
          addMapper(counterpart_door);
        }
        else
          switchMapper(other_room, door);
      }
    }
    rate.sleep();
  }
}


bool HierarchyMapper::gmapSrvCb(semantic_mapping_v2::MapSrv::Request& req, semantic_mapping_v2::MapSrv::Response& res){
  ROS_INFO("SERVICE GMAP");
  if(req.room_id >= 0 && req.room_id < room_mapper_.size()){
    res.map = room_mapper_[req.room_id]->getGMap();
    return true;
  }
  else if(req.room_id < 0 && current_mapper_ >= 0 && current_mapper_ < room_mapper_.size()){
    res.map = room_mapper_[current_mapper_]->getGMap();
    return true;
  }

  res.map = nav_msgs::OccupancyGrid();
  return false;
}


bool HierarchyMapper::mapSrvCb(semantic_mapping_v2::MapSrv::Request& req, semantic_mapping_v2::MapSrv::Response& res){
  ROS_INFO("SERVICE MAP");
  if(req.room_id >= 0 && req.room_id < room_mapper_.size()){
    room_mapper_[req.room_id]->downprojectMap();
    res.map = room_mapper_[req.room_id]->getMap();
    return true;
  }
  else if(req.room_id < 0 && current_mapper_ >= 0 && current_mapper_ < room_mapper_.size()){
    room_mapper_[current_mapper_]->downprojectMap();
    res.map = room_mapper_[current_mapper_]->getMap();
    return true;
  }


  res.map = nav_msgs::OccupancyGrid();
  return false;
}


bool HierarchyMapper::octomapSrvCb(semantic_mapping_v2::OctomapSrv::Request& req, semantic_mapping_v2::OctomapSrv::Response& res){
  ROS_INFO("SERVICE OCTOMAP");
  if(req.room_id >= 0 && req.room_id < room_mapper_.size()){
    res.map = room_mapper_[req.room_id]->getFullOctoMapMsg();
    return true;
  }
  else if(req.room_id < 0 && current_mapper_ >= 0 && current_mapper_ < room_mapper_.size()){
    res.map = room_mapper_[current_mapper_]->getFullOctoMapMsg();
    return true;
  }
  res.map = octomap_msgs::Octomap();
  return false;
}


bool HierarchyMapper::roomTypeMapSrvCb(semantic_mapping_v2::RoomTypeMapSrv::Request& req, semantic_mapping_v2::RoomTypeMapSrv::Response& res){
  ROS_INFO("SERVICE ROOM_MAP");
  int id = 0;
  if(req.room_id >= 0 && req.room_id < room_mapper_.size()){
    id = req.room_id;
  }
  else if(req.room_id < 0 && current_mapper_ >= 0 && current_mapper_ < room_mapper_.size()){
    id = current_mapper_;
  }
  else{
    res = semantic_mapping_v2::RoomTypeMapSrv::Response();
    return false;
  }

  if(req.id < 0)
    res.maps = room_mapper_[id]->getAllRoomTypeMapMsgs();
  else
    res.maps = std::vector<semantic_mapping_v2::RoomTypeMapMsg>({room_mapper_[id]->getRoomTypeMapMsg(req.id)});

  return true;
}


bool HierarchyMapper::roomTypeProbSrvCb(semantic_mapping_v2::RoomTypeProbSrv::Request& req, semantic_mapping_v2::RoomTypeProbSrv::Response& res){
  ROS_INFO("SERVICE ROOM_PROB");
  int id = 0;
  if(req.room_id >= 0 && req.room_id < room_mapper_.size()){
    id = req.room_id;
  }
  else if(req.room_id < 0 && current_mapper_ >= 0 && current_mapper_ < room_mapper_.size()){
    id = current_mapper_;
  }
  else{
    res = semantic_mapping_v2::RoomTypeProbSrv::Response();
    return false;
  }

  std::vector<size_t> order;
  res.probs = room_mapper_[id]->getRoomTypeProbs(order);
  res.names = room_mapper_[current_mapper_]->getRoomNames();

  return true;
}


bool HierarchyMapper::objMapSrvCb(semantic_mapping_v2::ObjectMapSrv::Request& req, semantic_mapping_v2::ObjectMapSrv::Response& res){
  ROS_INFO("SERVICE OBJ_MAP");
  int id = 0;
  if(req.room_id >= 0 && req.room_id < room_mapper_.size()){
    id = req.room_id;
  }
  else if(req.room_id < 0 && current_mapper_ >= 0 && current_mapper_ < room_mapper_.size()){
    id = current_mapper_;
  }
  else{
    res = semantic_mapping_v2::ObjectMapSrv::Response();
    return false;
  }

  if(req.id < 0)
    res.maps = room_mapper_[id]->getAllObjMapMsgs();
  else
    res.maps = std::vector<semantic_mapping_v2::ObjectMapMsg>({room_mapper_[id]->getObjMapMsg(req.id)});

  return true;
}


bool HierarchyMapper::objProbSrvCb(semantic_mapping_v2::ObjectProbSrv::Request& req, semantic_mapping_v2::ObjectProbSrv::Response& res){
  ROS_INFO("SERVICE OBJ_PROB");
  int id = 0;
  if(req.room_id >= 0 && req.room_id < room_mapper_.size()){
    id = req.room_id;
  }
  else if(req.room_id < 0 && current_mapper_ >= 0 && current_mapper_ < room_mapper_.size()){
    id = current_mapper_;
  }
  else{
    res = semantic_mapping_v2::ObjectProbSrv::Response();
    return false;
  }

  std::vector<size_t> order;
  res.probs = room_mapper_[id]->getObjectProbs(order);
  res.names = ObjectMapper::getObjNames();

  return true;
}


bool HierarchyMapper::hierarchySrvCb(semantic_mapping_v2::HierarchySrv::Request& req, semantic_mapping_v2::HierarchySrv::Response& res){
  ROS_INFO("SERVICE HIERARCHY");
  for(int i=0; i<room_mapper_.size(); i++){
    semantic_mapping_v2::RoomMsg room;
    room.header.stamp = ros::Time::now();
    room.id = i;
    std::vector<size_t> order;
    room.obj_probs = room_mapper_[i]->getObjectProbs(order);
    room.room_type_probs = room_mapper_[i]->getRoomTypeProbs(order);
  }

  for(int i=0; i<room_mapper_.size(); i++){
    std::vector<Door> doors = room_mapper_[i]->getDoors();
    for(const auto& door : doors){
      if(door.other_room_ <= i){
        semantic_mapping_v2::HierarchyLinkMsg link;
        link.header.stamp = ros::Time::now();
        link.room1 = i;
        link.room2 = door.other_room_;
        tf::poseTFToMsg(door.pose_, link.door1_pose);
        if(door.other_room_ >= 0){
          std::vector<Door> other_doors = room_mapper_[door.other_room_]->getDoors();
          for(const auto& door2 : other_doors){
            if(door2.id_ == door.counterpart_id_){
              tf::poseTFToMsg(door2.pose_, link.door2_pose);
              break;
            }
          }
        }
        res.links.push_back(link);
      }
    }
  }
  return true;
}



